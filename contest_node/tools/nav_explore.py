#!/usr/bin/env python3
"""自动探路 —— 用已知的速度接口，让机器人自己走一圈，把可通行区域记下来

背景：游戏**不给地图**（没有占据栅格、没有车道图）。规则里只有"导航敌情广播"，
场地几何得自己建。这个脚本是建图的第一步：**用底盘探**

和 speed_calib.py 的关系：那个是标定（量出命令→速度的映射），这个是拿标定好的
接口去探路。接口模型（2026-10-01 实测确认）：

    linear.x/y  = Unity 世界 X/Z 的目标速度，单位【场景单位/s】，1:1
    实际速度以 ~3 场景单位/s²（=1 m/s²）的斜坡爬向目标 —— 不是瞬时
    angular.z   = 目标偏航角速度 rad/s，1:1，符号与 state.twist.angular.z 相反

探索策略：沿一个方向直走，走到"动不了"就换下一个方向（+X → +Z → -X → -Z 轮转）
"动不了"的判据是**先动过、后来不动了** —— 因为起步有斜坡，不能一上来就判被挡

用法：
    source /opt/ros/jazzy/setup.bash
    source ~/tdt-learning4.5/tdt_contest_ws/install/setup.bash
    python3 nav_explore.py                       # 默认 90 秒、速度 6.0（= 2 m/s）
    python3 nav_explore.py --speed 9 --seconds 180
    python3 nav_explore.py --dry-run             # 只打印不发

输出：
    /tmp/nav_track.jsonl   轨迹（t / 命令 / 位置 / 实际速度），留给建图
    屏幕上打印每个方向走了多远、在哪被挡

机器人会真的到处跑。随时 Ctrl+C —— 退出钩子会补发零速度
"""
import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TwistStamped
from tdt_interface.msg import AutoAimState

UNITS_PER_M = 3.0
PREWARM_SEC = 2.0

# "被挡"判据
MOVE_SPEED_RATIO = 0.5    # 实际速度超过命令速度的这么多 → 认为"确实在走"
BLOCKED_SPEED = 0.15      # 实际速度掉到这以下……
BLOCKED_HOLD_SEC = 0.6    # ……并持续这么久 → 判定被挡
SEG_MAX_SEC = 8.0         # 单个方向最多走这么久
SETTLE_SEC = 0.5          # 换方向之间停一下
TRACK_HZ = 20.0           # 轨迹采样率

# 探索方向轮转（Unity 世界系）
DIRS = [(1.0, 0.0, "+X"), (0.0, 1.0, "+Z"), (-1.0, 0.0, "-X"), (0.0, -1.0, "-Z")]


class Explorer(Node):
    def __init__(self, player_n, dry_run, speed):
        super().__init__("nav_explore")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(TwistStamped, f"/target_speed_player_{player_n}", qos)
        self.create_subscription(AutoAimState, "/autoaim/state", self.on_state, qos)

        self.dry_run = dry_run
        self.speed = speed
        self.latest = None
        self.cmd = (0.0, 0.0, 0.0)
        self.track = open("/tmp/nav_track.jsonl", "a")
        self.n_track = 0
        self.last_track = 0.0
        self.create_timer(1.0 / 20.0, self.tick)

    def on_state(self, m):
        self.latest = {
            "t": time.time(),
            "pos": (m.position.x, m.position.y, m.position.z),
            "twist": (m.twist.linear.x, m.twist.linear.y, m.twist.linear.z),
            "yaw": float(m.yaw_degrees),
        }

    def tick(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = self.cmd[0]
        msg.twist.linear.y = self.cmd[1]
        if not self.dry_run:
            self.pub.publish(msg)

    def set_cmd(self, vx=0.0, vy=0.0):
        self.cmd = (float(vx), float(vy), 0.0)

    def spin_for(self, seconds, record=True):
        t_end = time.time() + seconds
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if record and self.latest and time.time() - self.last_track >= 1.0 / TRACK_HZ:
                self.last_track = time.time()
                self.track.write(json.dumps({
                    "t": self.latest["t"], "cmd": self.cmd,
                    "pos": self.latest["pos"], "twist": self.latest["twist"],
                    "yaw": self.latest["yaw"]}) + "\n")
                self.n_track += 1

    def stop(self):
        self.set_cmd(0.0, 0.0)
        self.spin_for(0.3, record=False)


def unity_xy(pos_map):
    """map FLU 的 position → Unity 世界 (x, z)"""
    mx, my, _ = pos_map
    return (-my, mx)


def unity_speed(twist_map):
    """map FLU 的 twist.linear → Unity 世界速度大小"""
    tx, ty, _ = twist_map
    return math.hypot(tx, ty)


def walk_until_blocked(ex, ux, uz, name):
    """沿 (ux,uz) 走，直到被挡 / 到点 / 超时。返回 (起点, 终点, 走了多久, 结局)"""
    ex.set_cmd(ux * ex.speed, uz * ex.speed)
    p0 = unity_xy(ex.latest["pos"])
    t0 = time.time()
    ever_moved = False
    slow_since = None
    while time.time() - t0 < SEG_MAX_SEC:
        rclpy.spin_once(ex, timeout_sec=0.02)
        if not ex.latest:
            continue
        if time.time() - ex.last_track >= 1.0 / TRACK_HZ:
            ex.last_track = time.time()
            ex.track.write(json.dumps({
                "t": ex.latest["t"], "cmd": ex.cmd, "pos": ex.latest["pos"],
                "twist": ex.latest["twist"], "yaw": ex.latest["yaw"]}) + "\n")
            ex.n_track += 1
        v = unity_speed(ex.latest["twist"])
        if v > MOVE_SPEED_RATIO * ex.speed:
            ever_moved = True
            slow_since = None
        elif ever_moved and v < BLOCKED_SPEED:
            if slow_since is None:
                slow_since = time.time()
            elif time.time() - slow_since > BLOCKED_HOLD_SEC:
                p1 = unity_xy(ex.latest["pos"])
                return p0, p1, time.time() - t0, "被挡"
    p1 = unity_xy(ex.latest["pos"])
    return p0, p1, time.time() - t0, "到点/超时"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--player", type=int, default=1)
    ap.add_argument("--speed", type=float, default=6.0, help="探索速度（场景单位/s，3 = 1 m/s）")
    ap.add_argument("--seconds", type=float, default=90.0, help="总探索时长上限")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    ex = Explorer(args.player, args.dry_run, args.speed)
    print("订阅 /autoaim/state …")
    t0 = time.time()
    while ex.latest is None and time.time() - t0 < 5:
        rclpy.spin_once(ex, timeout_sec=0.1)
    if ex.latest is None:
        print("✗ 收不到 /autoaim/state")
        rclpy.shutdown()
        return

    p = unity_xy(ex.latest["pos"])
    print(f"✓ 连上。起点 Unity (x,z) = ({p[0]:+.2f}, {p[1]:+.2f})   速度 {args.speed} 单位/s")
    print(f"\n预热 {PREWARM_SEC:.0f}s（等 DDS 匹配）…")
    ex.stop()
    ex.spin_for(PREWARM_SEC, record=False)

    print(f"\n开始探索（最多 {args.seconds:.0f} 秒，Ctrl+C 随时停）")
    print(f"{'方向':<7}{'起点':<22}{'终点':<22}{'走了':<9}{'时长':<8}结局")
    visited = [p]
    t_start = time.time()
    i = 0
    try:
        while time.time() - t_start < args.seconds:
            ux, uz, name = DIRS[i % len(DIRS)]
            i += 1
            p0, p1, dt, why = walk_until_blocked(ex, ux, uz, name)
            dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            print(f"{name:<7}({p0[0]:+7.2f},{p0[1]:+7.2f})      "
                  f"({p1[0]:+7.2f},{p1[1]:+7.2f})      {dist:6.2f}   {dt:5.1f}s   {why}")
            visited.append(p1)
            ex.stop()
            ex.spin_for(SETTLE_SEC, record=False)
    except KeyboardInterrupt:
        print("\n中断，补发零速度…")
    finally:
        ex.stop()
        ex.track.close()

    xs = [v[0] for v in visited]
    zs = [v[1] for v in visited]
    print(f"\n=== 探索了 {time.time()-t_start:.0f} 秒，{ex.n_track} 个轨迹点 ===")
    print(f"  到过的点 Unity x ∈ [{min(xs):+.2f}, {max(xs):+.2f}]   "
          f"z ∈ [{min(zs):+.2f}, {max(zs):+.2f}]")
    print(f"  跨度 {(max(xs)-min(xs))/UNITS_PER_M:.2f} m × {(max(zs)-min(zs))/UNITS_PER_M:.2f} m")
    print("  轨迹 → /tmp/nav_track.jsonl")


if __name__ == "__main__":
    main()
