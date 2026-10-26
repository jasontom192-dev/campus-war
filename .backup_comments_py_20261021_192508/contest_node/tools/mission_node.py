#!/usr/bin/env python3
"""全自动任务节点 —— 底盘跑路线 + 云台按段落对着目标，用血量/符激活判定推进。

路线（10 点 9 段，`nav_data/mission_stages.json` 可改）：
    出生点 → 打符位 → 三号区域 → 二号区域 → 五号区域 → 四号区域
    → 出生点 → 打符位 → 前哨站区域 → 敌方基地区域

每站怎么推进（done）：
    arrive  到点即走
    hp      停住、云台对着该号，把它的血打到 0（订阅 /navigation/enemies，编号明确）
    rune    按住符预设角 + 开打符模式，/game/rune/result 的 activated 从 false 变 true

云台：本节点**不直接发角度**（/target_angles 的发布者只能有一个 = auto_aim_node）。
它往 /mission/cmd 发提示："hold <yaw> <pitch> [rune]" / "auto"，auto_aim_node 里：
    · kSearch 时按住提示角（提示超时 2 s 自动失效，节点死了也不会卡住云台）
    · 见到目标进入跟踪/开火后，提示让位（照样自动锁定+开火）
    · rune 标记 = 任务要求开打符模式（等效进入打符），activated 后自动退回

用法（和 nav_node 同一套参数，另加 --stages）：
    mission_node.py                          # 真跑
    mission_node.py --dry                    # 只打印每站 + 规划每段，不开车
    mission_node.py --vmax 6 --player 1
"""
import argparse
import json
import math
import os
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from tdt_interface.msg import RuneResult

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nav_node import NavNode                                   # noqa: E402
from nav_map import FieldMap                                   # noqa: E402

DATA = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "..", "..", "nav_data"))


class MissionNode(NavNode):
    def __init__(self, args):
        super().__init__(args)                     # NavNode 里已经订好 /autoaim/state
        self.stages = args.stages
        self.rune_yaw = args.rune_yaw
        self.rune_pitch = args.rune_pitch

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.pub_mission = self.create_publisher(String, "/mission/cmd", qos)
        self.create_subscription(String, "/navigation/enemies", self.on_enemies, qos)
        self.create_subscription(RuneResult, "/game/rune/result", self.on_rune, qos)

        self.enemy_hp = {}          # number → health（最近一帧 /navigation/enemies）
        self.enemy_pos = {}         # number → (x, z)
        self.enemy_seen = set()
        self.rune_activated = False
        self.waiting = False
        self.last_aim_pub = 0.0

        self._enter_stage(0)

    # ---- 订阅 ----
    def on_enemies(self, msg):
        try:
            d = json.loads(msg.data)
            hp, pos = {}, {}
            for e in d.get("enemies", []):
                hp[int(e["number"])] = int(e["health"])
                pos[int(e["number"])] = (float(e["x"]), float(e["z"]))
        except Exception:
            return
        self.enemy_hp, self.enemy_pos = hp, pos
        self.enemy_seen.update(hp.keys())

    def on_rune(self, msg):
        if msg.activated and not self.rune_activated:
            self.rune_activated = True
            self.get_logger().info("★ /game/rune/result: activated=true（符打完了）")

    # ---- 任务推进 ----
    def _enter_stage(self, k):
        st = self.stages[k]
        self.stage_i = k
        self.goals = [tuple(st["goal"])]           # NavNode 的"当前目标"
        self.goal_i = 0
        self.path = None
        self.pause_until = 0.0
        self.ever_moved = False
        self.stuck_since = None
        self.waiting = False
        self.rune_activated = False
        self._wait_logged = False
        print(f"\n===== 第 {k + 1}/{len(self.stages)} 站：{st['name']}  "
              f"→ Unity({st['goal'][0]:+.1f},{st['goal'][1]:+.1f})  完成条件 {st['done']} =====",
              flush=True)
        if st.get("aim"):
            self._push_aim(force=True)
        else:
            self._pub_mission("auto")

    def _done_condition_met(self, st):
        if st["done"] == "rune":
            return self.rune_activated
        if st["done"] == "hp":
            n = int(st["number"])
            if self.enemy_hp.get(n, 1) <= 0:
                return True
            # 从列表里消失了也算打完（死了的敌人可能不再广播）
            return n in self.enemy_seen and n not in self.enemy_hp
        return False

    def _push_aim(self, force=False):
        st = self.stages[self.stage_i]
        aim = st.get("aim")
        if not aim or self.pos is None:
            return
        now = time.time()
        if not force and now - self.last_aim_pub < 0.2:
            return
        if aim["type"] == "rune":
            self._pub_mission(f"hold {self.rune_yaw:.2f} {self.rune_pitch:.2f} rune")
        elif aim["type"] == "enemy":
            p = self.enemy_pos.get(int(aim["number"]))
            if p is None:
                return                              # 还没收到它位置，先不夺云台
            yaw = math.degrees(math.atan2(p[0] - self.pos[0], p[1] - self.pos[1]))
            self._pub_mission(f"hold {yaw:.2f} {aim.get('pitch', -4.0):.2f}")
        self.last_aim_pub = now

    def _pub_mission(self, s):
        if self.args.dry_run:
            return
        m = String()
        m.data = s
        self.pub_mission.publish(m)

    # ---- NavNode 钩子 ----
    def on_tick(self):
        st = self.stages[self.stage_i]
        if st.get("aim"):
            self._push_aim()
        if self.waiting and self._done_condition_met(st):
            self.get_logger().info(f"★ {st['name']} 完成（{st['done']}）→ 下一站")
            self._wait_logged = False
            self._advance()

    def on_arrived(self, d_goal):
        st = self.stages[self.stage_i]
        if st["done"] == "arrive":
            self.stop(f"到点（{st['name']}）")
            return self._advance()
        # 要打掉的：停住，等条件（云台还在 aim 里对着）
        self.publish_cmd(0.0, 0.0)
        if not self._wait_logged:
            self._wait_logged = True
            self.get_logger().info(f"已到 {st['name']}，停住开打，等 {st['done']} 条件")
        return True

    def _advance(self):
        if self.stage_i + 1 >= len(self.stages):
            self.stop("★ 全任务走完")
            print("\n★ 全任务完成", flush=True)
            return False
        self.pause_until = time.time() + 0.8
        self._enter_stage(self.stage_i + 1)
        return True


def load_stages(path):
    d = json.load(open(path))
    rp = d.get("rune_preset", {})
    return d["stages"], float(rp.get("yaw", 45.05)), float(rp.get("pitch", -15.04))


def plan_only(stages, vmax):
    """--dry：不开 ROS，只把每段规划一遍打印出来。"""
    m = FieldMap()
    tot = 0.0
    print(f"任务共 {len(stages)} 站：")
    for k, st in enumerate(stages):
        g = st["goal"]
        print(f"  {k + 1:2d}. {st['name']:12s} ({g[0]:+7.2f},{g[1]:+7.2f})  "
              f"done={st['done']:6s}  aim={st.get('aim')}")
    print("\n逐段规划：")
    for k in range(len(stages) - 1):
        a = tuple(stages[k]["goal"]); b = tuple(stages[k + 1]["goal"])
        p = m.plan(a, b, turn_penalty=0.4)
        if p is None:
            print(f"  第{k + 1}段 {a} → {b}: ✗ 无路 !!")
            continue
        L = sum(math.hypot(p[i][0] - p[i - 1][0], p[i][1] - p[i - 1][1])
                for i in range(1, len(p)))
        tot += L
        print(f"  第{k + 1}段 {str(a):20s}→{str(b):20s} {L:5.1f} 单位 = {L / 3:4.1f} m")
    print(f"全程 {tot:.0f} 单位 = {tot / 3:.0f} m   限速 {vmax}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stages", default=os.path.join(DATA, "mission_stages.json"))
    ap.add_argument("--player", type=int, default=1)
    ap.add_argument("--vmax", type=float, default=6.0)
    ap.add_argument("--debug", action="store_true")
    ap.add_argument("--dry", action="store_true", help="只打印站点和每段规划，不开车")
    ap.add_argument("--dry-run", action="store_true", help="发指令抑制（联调用）")
    ap.add_argument("--fresh", action="store_true")
    ap.add_argument("--slow", action="store_true")
    args = ap.parse_args()

    stages, ry, rp = load_stages(args.stages)
    if args.dry:
        plan_only(stages, args.vmax)
        return

    args.stages = stages
    args.rune_yaw, args.rune_pitch = ry, rp
    args.goals = [tuple(stages[0]["goal"])]        # NavNode 初始化用

    rclpy.init()
    node = MissionNode(args)
    print(f"全自动任务启动：{len(stages)} 站，限速 {args.vmax} 单位/s "
          f"({'dry-run' if args.dry_run else '真跑'}，Ctrl+C 停)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        try:
            node._pub_mission("auto")               # 交回云台控制
            node.stop("退出")
            for _ in range(6):
                node.tick()
                time.sleep(0.05)
        except Exception:
            pass
    print("已停。")


if __name__ == "__main__":
    main()
