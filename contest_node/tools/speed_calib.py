#!/usr/bin/env python3
"""底盘速度接口标定 —— 量出 /target_speed_player_N 的**方向、增益、上限、符号**

为什么需要它：规则只说了 linear.x/y 是"Unity 世界 X/Z 平移"、angular.z 是"底盘偏航
角速度，符号与 AutoAimState.twist.angular.z 相反"，但**没说单位换算、增益、上限**
这三样只能真的发指令、看 position/twist 怎么动，反解出来

它会真的让机器人动。跑之前确认：
  · 游戏在「半自动：手控云台」或「全自动」模式（规则表 13：只有这两种收速度话题）
  · 场地里机器人周围没有别的东西挡着，你自己看得到它
  · 随时 Ctrl+C：脚本的退出钩子会补发零速度

用法：
    source /opt/ros/jazzy/setup.bash
    source ~/tdt-learning4.5/tdt_contest_ws/install/setup.bash
    python3 speed_calib.py                 # 默认 player_n=1（蓝方）
    python3 speed_calib.py --player 2      # 红方
    python3 speed_calib.py --dry-run       # 只打印要发的指令，不发（先看一眼）

输出：屏幕上分段的表格 + /tmp/speed_calib.json（原始数据，留着后面复核）

── 第一版（2026-10-01）踩到的三件事，这一版都治了 ──────────────────────
1. 第一条指令会被 DDS 匹配延迟吃掉 → 先发 PREWARM_SEC 秒零速度预热
2. 0.8 s 窗口太短：机器人的速度是**斜坡爬上去**的（约 3 场景单位/s²），
   后半段平均只能量到"爬到一半"的值 → 窗口拉到 MOVE_SEC，并只取末尾
   STEADY_SEC 秒的稳态
3. 增益那一列除错了分量（除的是 map 的 y，动的是 map 的 x）→ 现在统一
   把 twist 换成 Unity 世界速度，再往命令方向上投影
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

# 场景单位 ↔ 米（规则：3 场景单位 = 1 m）
UNITS_PER_M = 3.0

PREWARM_SEC = 2.0        # 开跑前先发这么久零速度，等 DDS 匹配
MOVE_SEC = 2.0           # 每档持续多久
STEADY_SEC = 0.5         # 稳态取窗口末尾这么多秒
SETTLE_SEC = 0.6         # 两档之间停一下
PROBE_MAGNITUDES = [1.0, 2.0, 3.0, 4.0, 6.0, 9.0]
ABORT_DIST = 15.0        # 单步位移超过它就中止后续（怕有一档特别快）


class Calib(Node):
    def __init__(self, player_n, dry_run):
        super().__init__("speed_calib")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)

        self.pub = self.create_publisher(TwistStamped, f"/target_speed_player_{player_n}", qos)
        self.create_subscription(AutoAimState, "/autoaim/state", self.on_state, qos)

        self.dry_run = dry_run
        self.latest = None            # 最近一帧 state
        self.cmd = (0.0, 0.0, 0.0)    # (vx, vy, wz)，Unity 世界系
        self.create_timer(1.0 / 20.0, self.tick)   # 20 Hz 发，远快于 0.5 s 超时

    # ---- 订阅 ----
    def on_state(self, m):
        self.latest = {
            "t": time.time(),
            "pos": (m.position.x, m.position.y, m.position.z),      # map FLU
            "twist": (m.twist.linear.x, m.twist.linear.y, m.twist.linear.z),
            "ang": (m.twist.angular.x, m.twist.angular.y, m.twist.angular.z),
            "yaw": float(m.yaw_degrees), "pitch": float(m.pitch_degrees),
            "hp": int(m.health), "maxhp": int(m.max_health),
            "ammo": int(m.ammo_remaining), "cap": int(m.ammo_capacity),
            "seq": int(m.frame_sequence),
        }

    # ---- 发布 ----
    def tick(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = self.cmd[0]
        msg.twist.linear.y = self.cmd[1]
        msg.twist.angular.z = self.cmd[2]
        if not self.dry_run:
            self.pub.publish(msg)

    def set_cmd(self, vx=0.0, vy=0.0, wz=0.0):
        self.cmd = (float(vx), float(vy), float(wz))

    def spin_for(self, seconds):
        """边转边按 20 Hz 发指令，返回这段时间内采到的 (twist, ang) 序列"""
        seg = []
        t_end = time.time() + seconds
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.latest is not None:
                seg.append({"t": self.latest["t"], "twist": self.latest["twist"],
                            "ang": self.latest["ang"]})
        return seg

    def stop(self):
        self.set_cmd(0.0, 0.0, 0.0)
        self.spin_for(0.3)                      # 多推几拍，确保游戏侧收到零速

    def wait_state(self, timeout=5.0):
        t0 = time.time()
        while self.latest is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest is not None


# ============================================================================
# 换算与统计
# ============================================================================
def unity_xy(pos_map):
    """map FLU 的 position → Unity 世界 (x, z)。规则：map = (Unity Z, -Unity X, Unity Y)"""
    mx, my, _ = pos_map
    return (-my, mx)


def unity_vel(twist_map):
    """map FLU 的 twist.linear → Unity 世界速度 (vx, vz)。同一个换法"""
    tx, ty, _ = twist_map
    return (-ty, tx)


def steady(seg, sec=STEADY_SEC):
    """取窗口末尾 sec 秒的平均 (Unity 速度, 偏航角速度)"""
    if not seg:
        return (0.0, 0.0), 0.0
    t_end = seg[-1]["t"]
    tail = [s for s in seg if s["t"] >= t_end - sec] or [seg[-1]]
    n = len(tail)
    vx = sum(unity_vel(s["twist"])[0] for s in tail) / n
    vz = sum(unity_vel(s["twist"])[1] for s in tail) / n
    wz = sum(s["ang"][2] for s in tail) / n
    return (vx, vz), wz


def peak_along(seg, ux, uz):
    """窗口内「沿指定方向的速度分量」的最大值

    ★ 为什么需要它：末段平均（steady）在**撞墙**的那一档上是废的 —— 机器人撞上
    障碍就停了，末段当然是 0，但它在撞上之前确实达到过目标速度。峰值才能反映
    "这条命令被接受成了多快"。第一版用末段平均，把"撞墙"误判成了"命令被拒"
    """
    if not seg:
        return 0.0
    best = 0.0
    for s in seg:
        vx, vz = unity_vel(s["twist"])
        v = vx * ux + vz * uz
        if abs(v) > abs(best):
            best = v
    return best


def run_move(cal, vx=0.0, vy=0.0, wz=0.0, seconds=MOVE_SEC):
    """发一段指令，返回 (Unity 位移Δ, 稳态速度, 稳态角速度, 原始采样)"""
    cal.set_cmd(vx, vy, wz)
    p0 = cal.latest["pos"]
    seg = cal.spin_for(seconds)
    p1 = cal.latest["pos"]
    a, b = unity_xy(p0), unity_xy(p1)
    vel, rate = steady(seg)
    return (b[0] - a[0], b[1] - a[1]), vel, rate, seg


def infer_mode(st):
    """从血量/弹量容量反推控制模式（规则表 7）"""
    cap = st.get("cap", 0)
    if cap >= 300:
        return "全自动（400 HP / 300 发）"
    if cap >= 200:
        return "半自动（300 HP / 200 发）"
    if cap > 0:
        return "全手动（200 HP / 100 发）"
    return "未知"


# ============================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--player", type=int, default=1, help="1 蓝方 / 2 红方")
    ap.add_argument("--dry-run", action="store_true", help="只打印，不真的发")
    ap.add_argument("--max", action="store_true",
                    help="只做【最大速度扫描】：沿 Unity X 交替 ±，密集档位找'可行最大速度'")
    ap.add_argument("--ladder", default="2,3,4,5,6,7",
                    help="扫描档位（场景单位/s，默认 2~7；想探更高就往后加，"
                         "但起点西边要给足空档：窗口 ≈ 命令/3+2 秒 × 命令速度）")
    args = ap.parse_args()
    ladder = [float(v) for v in args.ladder.split(",")]

    rclpy.init()
    cal = Calib(args.player, args.dry_run)
    print(f"订阅 /autoaim/state …（player_n={args.player}）")
    if not cal.wait_state():
        print("✗ 收不到 /autoaim/state —— 游戏开了吗？ROS_DOMAIN_ID 对吗？")
        rclpy.shutdown()
        return

    st = cal.latest
    print(f"✓ 已连上。位置 map FLU = {tuple(round(v, 2) for v in st['pos'])}   "
          f"Unity (x,z) = {tuple(round(v, 2) for v in unity_xy(st['pos']))}")
    print(f"  血量 {st['hp']}/{st['maxhp']}  弹量 {st['ammo']}/{st['cap']}  "
          f"→ 推断模式：{infer_mode(st)}")
    print(f"  yaw={st['yaw']:.1f}°  pitch={st['pitch']:.1f}°  seq={st['seq']}")

    print(f"\n预热：发 {PREWARM_SEC:.0f}s 零速度，等 DDS 把发布者/订阅者配上…")
    cal.stop()
    cal.spin_for(PREWARM_SEC)
    print("  预热完毕。")

    result = {"player": args.player, "start_pos": cal.latest["pos"],
              "mode_guess": infer_mode(st), "tests": []}
    aborted = False

    if args.max:
        # ★ 最大速度扫描（第二版，2026-10-02）：
        #   第一版每档固定 2 s 窗口 → 加速斜坡 ~3 单位/s² 下，2 秒最多爬到 6，
        #   7/8/10 档全被"窗口太短"误判成"跟不上"（手动数据实测能开 6.5~8.9）
        #   现在：窗口 = 1.5 + 命令/3 + 0.5 s（爬坡段 + 稳态段），只量尾巴；
        #   方向改成沿 -X 跑、按 +4 慢速自动回退 —— 起点只需西边一条空档
        print(f"\n=== 最大速度扫描（沿 -X 跑 + 慢速回退，档位 {ladder}）===")
        print("★ 窗口按爬坡时间加长了；看【稳态增益】：≈1=有效；<0.9=真到顶；峰值≈0=被拒")
        print(f"{'命令':<9}{'峰值':<10}{'稳态':<10}{'稳态增益':<10}{'位移':<9}判定")
        x_home = unity_xy(cal.latest["pos"])[0]
        best_ok = 0.0
        try:
            for mag in ladder:
                window = 1.5 + mag / 3.0 + 0.5          # 爬坡(≈cmd/3 秒) + 稳态
                d, vel, _, seg = run_move(cal, vx=-mag, seconds=window)
                v_peak = peak_along(seg, -1.0, 0.0)     # 沿 -X 的峰值（u=(-1,0) 时已取正）
                v_tail = -vel[0]                        # 稳态：窗口尾巴的 -X 速度
                gain = v_tail / mag if mag else 0.0
                step = math.hypot(*d)
                if v_peak < 0.05:
                    verdict = "被拒（没动）"
                elif gain < 0.90:
                    verdict = "← 到顶 / 跟不上"
                else:
                    verdict = "OK"
                    best_ok = max(best_ok, mag)
                print(f"{-mag:+6.1f}   {v_peak:+8.3f}  {v_tail:+8.3f}  "
                      f"{gain:+8.3f}   {step:7.2f}  {verdict}")
                result["tests"].append({"kind": "max", "cmd": -mag,
                                        "vel_peak": v_peak, "vel_tail": v_tail,
                                        "gain": gain, "delta": d})
                cal.stop()
                cal.spin_for(0.3)
                # 慢速回退到起点附近（只为下一档留空档，不测）
                t_ret = time.time() + 20.0
                while time.time() < t_ret:
                    if unity_xy(cal.latest["pos"])[0] >= x_home - 2.0:
                        break
                    cal.set_cmd(vx=+4.0)
                    cal.spin_for(0.3)
                cal.stop()
                cal.spin_for(SETTLE_SEC)
                if step > 45.0:
                    print(f"  ⚠ 单步位移 {step:.1f} 超过 45，中止后续")
                    aborted = True
                    break
            result["max_ok"] = best_ok
            print(f"\n可行最大速度（稳态增益 ≥0.9 的最大档）≈ {best_ok:.1f} 场景单位/s "
                  f"= {best_ok/UNITS_PER_M:.2f} m/s")
        except KeyboardInterrupt:
            print("\n中断，补发零速度…")
        finally:
            cal.stop()
            result["end_pos"] = cal.latest["pos"] if cal.latest else None
            with open("/tmp/speed_calib.json", "w") as f:
                json.dump(result, f, indent=1, default=str)
        print("原始数据 → /tmp/speed_calib.json")
        rclpy.shutdown()
        return

    try:
        # ---- 1/4 方向 ----
        print("\n=== 1/4 方向：每个方向给 1.0，持续 2 s ===")
        print(f"{'命令':<14}{'位移ΔUnity(x,z)':<26}{'稳态速度Unity(vx,vz)':<28}{'判定'}")
        axes = [(1.0, 0.0, "+X"), (-1.0, 0.0, "-X"), (0.0, 1.0, "+Z"), (0.0, -1.0, "-Z")]
        for vx, vy, name in axes:
            d, vel, _, _ = run_move(cal, vx=vx, vy=vy)
            step = math.hypot(*d)
            ok = "动了" if step > 0.3 else "没动/被挡"
            print(f"({vx:+.1f},{vy:+.1f}) {name:<7}"
                  f"{str(tuple(round(v, 2) for v in d)):<26}"
                  f"{str(tuple(round(v, 2) for v in vel)):<28}{ok}  |Δ|={step:.2f}")
            result["tests"].append({"kind": "dir", "name": name, "vx": vx, "vy": vy,
                                    "delta": d, "vel": vel})
            cal.stop()
            cal.spin_for(SETTLE_SEC)

        # ---- 2/4 增益/饱和（沿 Z 轴来回，避免越走越远）----
        print(f"\n=== 2/4 增益与上限：沿 Z 轴交替正负，给 {PROBE_MAGNITUDES} ===")
        print("★ 看【峰值】：撞墙的那几档末段必然是 0，只有峰值反映命令被接受成了多快")
        print(f"{'命令':<9}{'峰值':<12}{'增益':<11}{'末段':<11}{'位移':<10}{'备注'}")
        for i, mag in enumerate(PROBE_MAGNITUDES):
            vy = mag if i % 2 == 0 else -mag
            d, vel, _, seg = run_move(cal, vy=vy)
            v_peak = peak_along(seg, 0.0, math.copysign(1.0, vy))   # 沿命令方向的峰值
            v_tail = vel[1]                       # 末段（撞墙后≈0）
            gain = v_peak / abs(vy)               # 峰值 / 命令大小
            step = math.hypot(*d)
            note = ""
            if v_peak < 0.05:
                note = "← 真的没动（被拒？）"
            elif gain < 0.85:
                note = "← 没爬到目标（窗口不够 / 上限）"
            elif step < 0.5 * abs(v_peak) * MOVE_SEC * 0.5:
                note = "← 动了但半路停住（撞墙）"
            print(f"{vy:+6.2f}   {v_peak:+10.3f}  {gain:+8.3f}    {v_tail:+10.3f}  {step:8.2f}   {note}")
            result["tests"].append({"kind": "gain", "cmd": vy, "vel_peak": v_peak,
                                    "vel_tail": v_tail, "gain": gain, "delta": d})
            cal.stop()
            cal.spin_for(SETTLE_SEC)
            if step > ABORT_DIST:
                print(f"  ⚠ 单步位移 {step:.1f} 超过 {ABORT_DIST}，中止后续档位")
                aborted = True
                break

        # ---- 3/4 偏航 ----
        print("\n=== 3/4 偏航：wz=+1.0 / -1.0，各 2 s ===")
        for wz in (1.0, -1.0):
            d, vel, rate, _ = run_move(cal, wz=wz)
            print(f"wz={wz:+.1f}  → 稳态 angular.z = {rate:+.4f} rad/s   "
                  f"符号{'相同' if rate * wz > 0 else '相反'}   增益 {abs(rate / wz):.3f}")
            result["tests"].append({"kind": "yaw", "wz": wz, "rate": rate})
            cal.stop()
            cal.spin_for(SETTLE_SEC)

        # ---- 4/4 回零 ----
        print("\n=== 4/4 收尾：发零速度停车 ===")
        cal.stop()
        cal.spin_for(0.5)
        print("  已停。")

    except KeyboardInterrupt:
        print("\n中断，补发零速度…")
    finally:
        cal.stop()
        result["end_pos"] = cal.latest["pos"] if cal.latest else None
        with open("/tmp/speed_calib.json", "w") as f:
            json.dump(result, f, indent=1, default=str)

    print("\n=== 汇总 ===")
    print(f"  模式（推断）：{result['mode_guess']}" + ("   ⚠ 后续档位被中止" if aborted else ""))
    print("  方向：命令 +X/-X/+Z/-Z 分别对应上面第 1 段的位移方向")
    print("  增益：稳态速度 ÷ 命令值 —— 接近 1.0 = 命令就是场景单位/s；"
          "突然掉到 0 = 那档被拒")
    print("原始数据 → /tmp/speed_calib.json")


if __name__ == "__main__":
    main()
