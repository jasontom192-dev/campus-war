#!/usr/bin/env python3
"""底盘导航节点 —— 把机器人开到指定点。

独立进程，**不碰自瞄**：只订阅 /autoaim/state（120 Hz，纯状态无图）拿位姿，
往 /target_speed_player_N 发世界系速度。所以可以和 auto_aim_node 同时跑。

控制律（2026-10-01 标定出来的接口模型）：
    linear.x/y = Unity 世界 X/Z 的**目标速度**，1:1，单位场景单位/s（3 = 1 m/s）
    实际速度以 ~3 场景单位/s² 的斜坡爬向目标 —— 所以别指望瞬时响应
    ★ 指令是世界系的，**不需要知道车头朝向**（游戏也不给底盘朝向）

    pure pursuit：取路径上前面 LOOKAHEAD 距离的点，v = clamp(k·Δp, ±VMAX)

安全（都实测过原因）：
    · 看门狗：断流 > 0.5 s 发零速（规则附录 C 检录项）
    · 卡死检测：发了速度但实际 twist≈0 持续 0.6 s → 停车、报告
      （撞墙和掉坑在数据上一样，所以同一套逻辑处理）
    · 到点容差 + 限速 + 非有限值过滤（游戏会整包拒收）

用法：
    source /opt/ros/jazzy/setup.bash   # 或新终端（.bashrc 已带）
    nav_node.py --goal -16.5 9.17              # 开到 6 号前哨站
    nav_node.py --goal -16.5 9.17 --dry-run    # 只规划、不发指令
    nav_node.py --goal -16.5 9.17 --vmax 4     # 限速 4 场景单位/s
    nav_node.py --goal -16.5 9.17 --player 2   # 红方
    # 多途经点连跑（一口气走环线，逐段规划、到点不停车直接下一段）：
    nav_node.py --points "14.5,-34.5 -16.5,9.2 -5.5,-14 14.5,-34.5" --vmax 4
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TwistStamped
from tdt_interface.msg import AutoAimState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nav_map import FieldMap          # noqa: E402

UNITS_PER_M = 3.0
LOOKAHEAD   = 3.0        # 前视距离（场景单位）。★ 6→4.5→3：转弯切内角会撞墙，实测过
TURN_SLOW   = 2.0        # 弯道减速强度：航向变化越大，限速压得越低
KP          = 1.0        # 已废弃：速度增益现在 = vmax/LOOKAHEAD（见 _tick 里的注释）
GOAL_TOL    = 2.0        # 到点容差（场景单位）
VMAX_DEF    = 6.0        # 默认限速（场景单位/s = 2 m/s）
CMD_TIMEOUT = 0.5        # 看门狗阈值（规则）
STUCK_SPEED = 0.6        # 实际速度低于它算"没动"（场景单位/s）
STUCK_HOLD  = 0.6        # 已经动过之后再不动，持续这么久 → 判定卡住
START_GRACE = 3.0        # ★ 一次都没动过时的宽限：要 3 s（速度斜坡 ~1 s + DDS 匹配）
PREWARM_SEC = 2.0        # ★ 连上后先发这么久零速，等 DDS 把发布者/订阅者配上
                         #   （标定时踩过：第一条指令会被匹配延迟吃掉）
MAX_STUCK   = 15         # ★ 2026-10-05 用户改：第 15 次卡住才放弃（原来是第 5 次）
BLOCK_RADIUS = 1.6       # 卡住时标障碍的半径（场景单位）
BLOCK_AHEAD  = 1.4       # ★ 障碍块中心沿"当时的指令方向"前移这么多。
                         #   直接打在车自己的位置上不行 —— 半径 2 的禁区把四邻
                         #   全封住，车被自己围死，重规划"没有通路"（实测踩过）。


class NavNode(Node):
    def __init__(self, args):
        super().__init__("nav_node")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)

        self.args = args
        self.pub = self.create_publisher(TwistStamped, f"/target_speed_player_{args.player}", qos)
        self.create_subscription(AutoAimState, "/autoaim/state", self.on_state, qos)

        self.map = FieldMap()
        # ★ 2026-10-05 起节点不再往这个文件写（幽灵块教训：只在当局内存标障碍），
        # 这里保留读取 —— 文件可当手工障碍清单用。想清空就写成 []。
        self.learned_path = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "..", "nav_data", "learned_blocks.json"))
        self.learned = []
        if os.path.exists(self.learned_path) and not args.fresh:
            try:
                self.learned = json.load(open(self.learned_path))
                for bx, bz in self.learned:
                    self.map.block_around(bx, bz, radius=BLOCK_RADIUS)
                self.get_logger().info(f"载入 {len(self.learned)} 个学到的障碍点")
            except Exception as e:
                self.get_logger().warn(f"learned_blocks.json 读不了（{e}），忽略")
                self.learned = []
        # 途经点序列（--goal 是单点，--points 是多段）
        self.goals = args.goals
        self.goal_i = 0
        self.pause_until = 0.0     # 途经点之间停一小会儿（米级停顿，防顿挫）
        self.pos = None            # Unity (x, z)
        self.vel = (0.0, 0.0)      # Unity (vx, vz)
        self.stamp = None
        self.first_state_t = None  # 收到第一帧的时刻，用来算预热
        self.ever_moved = False    # 有没有动过（区分"还没起步"和"撞上了"）
        self.stuck_count = 0       # 卡住次数，达到 MAX_STUCK 放弃
        self.cmd = (0.0, 0.0)
        self.path = None
        self.path_i = 0            # 路径上已通过的最近点下标
        self.done = False
        self.stuck_since = None
        self.stream_lost = False       # 断流状态，用来只报一次日志
        self.t0 = time.time()
        self.create_timer(1.0 / 20.0, self.tick)

    # ---- 订阅 ----
    def on_state(self, m):
        mx, my, _ = m.position.x, m.position.y, m.position.z
        tx, ty, _ = m.twist.linear.x, m.twist.linear.y, m.twist.linear.z
        self.pos = (-my, mx)                    # map FLU → Unity (x, z)
        self.vel = (-ty, tx)
        self.stamp = time.time()
        if self.first_state_t is None:
            self.first_state_t = self.stamp

    # ---- 工具 ----
    def publish_cmd(self, vx, vy):
        if not all(map(math.isfinite, (vx, vy))):   # 非有限值游戏会整包拒收
            vx = vy = 0.0
        self.cmd = (vx, vy)
        if self.args.dry_run:
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        self.pub.publish(msg)

    # ★ 2026-10-05 起不再被调用（幽灵块教训：只在当局内存标障碍，不落盘）。
    def save_learned(self):
        try:
            json.dump(self.learned, open(self.learned_path, "w"), indent=1)
        except Exception:
            pass

    def stop(self, why=""):
        self.publish_cmd(0.0, 0.0)
        if why:
            self.get_logger().info(f"停车：{why}")

    def tick(self):
        # ★ 包一层：定时器回调里抛异常的话 rclpy 不一定报得出来，
        #   表现是"车走了一小步就不动了、日志啥也没有"（实测踩过）。宁可打出来。
        try:
            self._tick()
        except Exception as e:
            import traceback
            self.get_logger().error(f"tick 抛异常，已停：{e}\n{traceback.format_exc()}")
            self.done = True
            try:
                self.publish_cmd(0.0, 0.0)
            except Exception:
                pass

    def on_tick(self):
        """每拍一进来就调（子类钩子：mission 用）。默认什么都不做。"""

    def on_arrived(self, d_goal):
        """到达当前目标。返回 True=继续运行（已处理），False=结束（调用方置 done）。

        默认行为 = 多途经点推进：还有下一段就换目标 + 停顿 0.8 s；
        没有就停车结束。mission 子类改成按"任务条件"推进。
        """
        if self.goal_i + 1 < len(self.goals):
            self.goal_i += 1
            nxt = self.goals[self.goal_i]
            self.path = None
            self.pause_until = time.time() + 0.8
            self.ever_moved = False
            self.stuck_since = None
            self.stop(f"第 {self.goal_i}/{len(self.goals) - 1} 段到点 —— "
                      f"停顿后开第 {self.goal_i + 1} 段 → "
                      f"Unity({nxt[0]:+.1f},{nxt[1]:+.1f})")
            return True
        self.stop(f"到点（距目标 {d_goal:.1f} 单位）"
                  + ("，全部段走完 ✓" if len(self.goals) > 1 else ""))
        return False

    def _tick(self):
        if self.pos is None:
            return
        self.on_tick()
        # 看门狗：断流超 0.5 s 主动停车（游戏侧也会清，但规则要求参赛程序自己也做）
        if self.stamp is not None and time.time() - self.stamp > CMD_TIMEOUT:
            self.publish_cmd(0.0, 0.0)
            if not self.stream_lost:        # ★ 只报一次，别 20 Hz 刷屏
                self.stream_lost = True
                self.get_logger().warn("断流超 0.5 s —— 持续发零速")
            return
        if self.stream_lost:
            self.stream_lost = False
            self.get_logger().info("数据流恢复")
        if self.done:
            return

        # 途经点之间的小停顿（0.8 s 零速），让位姿稳一下再开下一段
        if time.time() < self.pause_until:
            self.publish_cmd(0.0, 0.0)
            return

        # ★ 预热：连上后先发 PREWARM_SEC 秒零速，等 DDS 把发布者/订阅者配上。
        #   不预热的话第一批指令会被匹配延迟吃掉，机器人一动不动 —— 标定时踩过。
        if time.time() - self.first_state_t < PREWARM_SEC:
            self.publish_cmd(0.0, 0.0)
            return

        if self.path is None:
            self.plan()
            if self.path is None:
                self.done = True
                return

        goal = self.path[-1]
        d_goal = math.hypot(goal[0] - self.pos[0], goal[1] - self.pos[1])
        if d_goal < GOAL_TOL:
            if self.on_arrived(d_goal):
                return
            self.done = True
            return

        # ---- 卡死检测：发了速度但没动 ----
        # 分两种情况（第一版没分，一上来就误判）：
        #   · 已经动过之后再不动 → 撞墙/掉坑，STUCK_HOLD 就够
        #   · 一次都没动过 → 可能只是速度斜坡还没爬满，给 START_GRACE 的宽限
        speed = math.hypot(*self.vel)
        cmd_speed = math.hypot(*self.cmd)
        if speed > STUCK_SPEED:
            self.ever_moved = True
            self.stuck_since = None
        elif cmd_speed > 1.0:
            if self.stuck_since is None:
                self.stuck_since = time.time()
            else:
                hold = STUCK_HOLD if self.ever_moved else START_GRACE
                if time.time() - self.stuck_since > hold:
                    self.stuck_count += 1
                    px, pz = self.pos
                    if self.stuck_count >= MAX_STUCK:
                        self.done = True
                        self.stop(f"卡了 {self.stuck_count} 次，放弃。"
                                  f"最后位置 Unity({px:+.1f},{pz:+.1f})")
                        return
                    # ★ 运行时学障碍：把撞到的这一块标死，重新规划绕开。
                    #   块心沿指令方向前移 —— 打在车身上会把车自己围死。
                    ux, uz = self.cmd
                    un = math.hypot(ux, uz) or 1.0
                    bx, bz = px + ux / un * BLOCK_AHEAD, pz + uz / un * BLOCK_AHEAD
                    self.map.block_around(bx, bz, radius=BLOCK_RADIUS)
                    # ★ 2026-10-05 用户定：只在【当局内存】标障碍（重规划绕开），
                    #   不再写盘 —— "幽灵块"会永久污染后续所有任务。
                    #   save_learned 保留但不再调用。
                    self.stop(f"卡住（命令 {cmd_speed:.1f} 实际 {speed:.1f}）"
                              f"Unity({px:+.1f},{pz:+.1f}) —— 已标为障碍，重新规划"
                              f"（第 {self.stuck_count}/{MAX_STUCK} 次）")
                    self.path = None                 # 下一拍重新 plan()
                    self.ever_moved = False
                    self.stuck_since = None
                    return
        else:
            self.stuck_since = None

        self._dbg()

        # ---- pure pursuit ----
        look = self.lookahead_point()
        dx, dy = look[0] - self.pos[0], look[1] - self.pos[1]
        # ★ 弯道减速：纯追踪在拐角处会切内角，切出去就撞台阶墙（实测栽过）。
        #   量前方一段路径的航向变化，变化越大限速压得越低。
        # ★ 默认全速（用户 2026-10-02 定了：所有地方不减速、全程 max）。
        #   想要老的"转弯前收油 + 到达减速"行为：加 --slow。
        if self.args.slow:
            vmax = self.args.vmax * self.turn_scale() * min(1.0, max(0.3, d_goal / 6.0))
        else:
            vmax = self.args.vmax
        # ★ 速度增益跟着限速走：kp = vmax / LOOKAHEAD，直线速度 = vmax。
        #   旧写法 kp 恒等于 1.0，而前瞻点按定义就在前方 ~LOOKAHEAD 单位 —— 速度
        #   被死死卡在 ~3，--vmax 给多大都无效（实测：vmax 6 的环线只跑 2.2~2.6）。
        kp = self.args.vmax / LOOKAHEAD
        vx, vy = kp * dx, kp * dy
        sp = math.hypot(vx, vy)
        if sp > vmax:
            vx, vy = vx * vmax / sp, vy * vmax / sp
        self.publish_cmd(vx, vy)

    def _dbg(self):
        if not self.args.debug:
            return
        now = time.time()
        if now - self.t0 < 0.5:
            return
        self.t0 = now
        n = len(self.path) if self.path else 0
        px, pz = self.pos
        print(f"  [debug] pos=({px:+6.2f},{pz:+6.2f}) "
              f"实际=({self.vel[0]:+5.2f},{self.vel[1]:+5.2f}) "
              f"指令=({self.cmd[0]:+5.2f},{self.cmd[1]:+5.2f}) "
              f"路径 {self.path_i}/{n}", flush=True)

    def turn_scale(self):
        """看前方 8 单位内的拐角，返回 0.35~1.0 的限速系数。

        ★ 只看"快到了"的拐角、按距离淡出：路径是抽稀过的长直段，把十几单位外
        的拐角也当"马上要转"的话，整条直道都会被压到最低速（实测：环线全程
        2.3 = 0.39×6，只有最后一段能到 5.5）。
        """
        i = self.path_i
        if i + 2 >= len(self.path):
            return 1.0
        p0, p1 = self.path[i], self.path[i + 1]
        a0 = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        # 还差多少到当前段的终点（= 第一个拐角）
        acc = math.hypot(p1[0] - self.pos[0], p1[1] - self.pos[1])
        worst = 0.0
        for k in range(i + 1, len(self.path) - 1):
            if acc > 8.0:
                break
            q0, q1 = self.path[k], self.path[k + 1]
            a1 = math.atan2(q1[1] - q0[1], q1[0] - q0[0])
            d = abs((a1 - a0 + math.pi) % (2 * math.pi) - math.pi)
            worst = max(worst, d * max(0.0, 1.0 - acc / 8.0))   # 距离越远权重越低
            a0 = a1
            acc += math.hypot(q1[0] - q0[0], q1[1] - q0[1])
        return max(0.35, 1.0 / (1.0 + TURN_SLOW * worst))

    def lookahead_point(self):
        """路径上「投影点 + LOOKAHEAD」处的点 —— 纯追踪的目标点。

        ★ 旧写法拿"最近的路径顶点"当锚：在长直段上开出去 ~1.5 单位后，
        算出的前瞻点会正好落在车自己身上 → 指令衰减到 0 → 车停死
        （实测：出生点→(-5,-20) 那条 2 点直线，开出 1 米就不动了）。
        正确做法：先把车**投影到路径线上**（不是找最近顶点），再从投影点沿路径前进。
        """
        if len(self.path) < 2:
            return self.path[-1]
        # 1) 投影：从当前段往后扫（只前进不后退，防止路径自交时来回跳）
        best = (1e18, self.path_i, 0.0)
        for k in range(self.path_i, len(self.path) - 1):
            ax, ay = self.path[k]
            ex, ey = self.path[k + 1][0] - ax, self.path[k + 1][1] - ay
            L2 = ex * ex + ey * ey
            t = 0.0 if L2 < 1e-9 else min(1.0, max(0.0,
                ((self.pos[0] - ax) * ex + (self.pos[1] - ay) * ey) / L2))
            d = math.hypot(ax + t * ex - self.pos[0], ay + t * ey - self.pos[1])
            if d < best[0]:
                best = (d, k, t)
        _, self.path_i, t0 = best
        # 2) 从投影点沿路径前进 LOOKAHEAD
        a, b = self.path[self.path_i], self.path[self.path_i + 1]
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        remain = (1.0 - t0) * seg                    # 投影点到本段末的剩余
        if remain >= LOOKAHEAD:
            r = t0 + LOOKAHEAD / max(seg, 1e-9)
            return (a[0] + r * (b[0] - a[0]), a[1] + r * (b[1] - a[1]))
        acc = remain
        for k in range(self.path_i + 1, len(self.path) - 1):
            seg = math.hypot(self.path[k + 1][0] - self.path[k][0],
                             self.path[k + 1][1] - self.path[k][1])
            if acc + seg >= LOOKAHEAD:
                r = (LOOKAHEAD - acc) / seg if seg > 1e-6 else 0.0
                return (self.path[k][0] + r * (self.path[k + 1][0] - self.path[k][0]),
                        self.path[k][1] + r * (self.path[k + 1][1] - self.path[k][1]))
            acc += seg
        return self.path[-1]

    def plan(self):
        goal = self.goals[self.goal_i]
        if not self.map.is_walkable(*goal):
            self.get_logger().warn(f"目标 Unity({goal[0]:.1f},{goal[1]:.1f}) 不在可通行区，"
                                   f"会就近找个能走的")
        # ★ 机器人当前所在格一定是可走的 —— 它就在那儿站着。
        #   不这么做的话，地图上的针孔会让规划器把起点挪走（实测挪了 3 单位）。
        ci, cj = self.map.cell_of(*self.pos)
        if self.map.in_bounds(ci, cj):
            self.map.walkable[ci, cj] = True
        p = self.map.plan(self.pos, goal, turn_penalty=0.4)
        if p is None:
            # ★ 兜底：万一车被"学到的障碍块"围住（实测：重规划报"没有通路"），
            #   把车周围一圈重新放开，保证至少能退出来再试一次。
            self.get_logger().warn("首次规划无通路 —— 放开车周围一圈重试")
            ci, cj = self.map.cell_of(*self.pos)
            rr = int(math.ceil(1.5 / self.map.res))
            for di in range(-rr, rr + 1):
                for dj in range(-rr, rr + 1):
                    if di * di + dj * dj <= rr * rr and self.map.in_bounds(ci + di, cj + dj):
                        self.map.walkable[ci + di, cj + dj] = True
            p = self.map.plan(self.pos, goal, turn_penalty=0.4)
        if p is None:
            self.get_logger().error("规划失败：没有通路")
            return
        self.path = p
        self.path_i = 0
        L = sum(math.hypot(p[i][0] - p[i - 1][0], p[i][1] - p[i - 1][1])
                for i in range(1, len(p)))
        seg = f"[第 {self.goal_i + 1}/{len(self.goals) - 1} 段] " if len(self.goals) > 1 else ""
        self.get_logger().info(
            f"{seg}已规划：{len(p)} 个点、{L/UNITS_PER_M:.1f} m  "
            f"从 Unity({self.pos[0]:+.1f},{self.pos[1]:+.1f}) "
            f"到 ({goal[0]:+.1f},{goal[1]:+.1f})")
        if self.args.dry_run:
            for i in range(0, len(p), max(1, len(p) // 12)):
                print(f"   ({p[i][0]:+7.2f}, {p[i][1]:+7.2f})")
            self.done = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--goal", type=float, nargs=2, default=None,
                    metavar=("X", "Z"), help="单目标点，Unity 世界坐标（场景单位）")
    ap.add_argument("--points", type=str, default=None,
                    help='多途经点连跑："x,z x,z ..."（逐段规划，到途经点停顿后继续）')
    ap.add_argument("--player", type=int, default=1, help="1 蓝方 / 2 红方")
    ap.add_argument("--vmax", type=float, default=VMAX_DEF,
                    help=f"限速（场景单位/s，默认 {VMAX_DEF} = 2 m/s）")
    ap.add_argument("--dry-run", action="store_true", help="只规划、不发指令")
    ap.add_argument("--debug", action="store_true",
                    help="每 0.5 秒打印一次 位置/实际速度/下发指令/路径进度")
    ap.add_argument("--fresh", action="store_true",
                    help="忽略之前学到的障碍，从干净地图开始")
    ap.add_argument("--slow", action="store_true",
                    help="开启转弯减速 + 到达减速（默认为全速，什么都不减）")
    ap.add_argument("--no-slow", action="store_true", help=argparse.SUPPRESS)
    args = ap.parse_args()
    if args.points:
        args.goals = [tuple(float(v) for v in p.split(","))
                      for p in args.points.replace(";", " ").split()]
    elif args.goal:
        args.goals = [tuple(args.goal)]
    else:
        ap.error("要 --goal 单点，或 --points 多途经点")

    rclpy.init()
    node = NavNode(args)
    if len(args.goals) == 1:
        print(f"目标 Unity({args.goals[0][0]:+.2f}, {args.goals[0][1]:+.2f})  "
              f"限速 {args.vmax} 单位/s = {args.vmax/UNITS_PER_M:.1f} m/s"
              f"{'  [dry-run]' if args.dry_run else ''}")
    else:
        print(f"路线 {len(args.goals) - 1} 段："
              + " → ".join(f"({g[0]:+.1f},{g[1]:+.1f})" for g in args.goals)
              + f"   限速 {args.vmax} 单位/s = {args.vmax/UNITS_PER_M:.1f} m/s"
              f"{'  [dry-run]' if args.dry_run else ''}")
    print("等 /autoaim/state …（Ctrl+C 停）")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:   # SIGTERM（timeout/kill）时
        pass                                            # 不用打一坨 traceback
    finally:
        try:
            node.stop("退出")
            for _ in range(6):      # 多推几拍零速，确保游戏侧收到
                node.tick()
                time.sleep(0.05)
        except Exception:
            pass
    print("已停。")


if __name__ == "__main__":
    main()
