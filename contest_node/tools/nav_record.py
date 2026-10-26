#!/usr/bin/env python3
"""手动探路时的被动录制 —— 只订阅 /autoaim/state，**不发任何指令**

用途：你自己键鼠开着机器人在场地里跑一圈，这个脚本把轨迹记下来用于建图
因为不发任何东西，全手动模式下也能安全地录（不会和你的键鼠抢控制权）

为什么要录：地图是从轨迹反推出来的 —— 走过的地方 = 可通行。没录就等于没探

用法：
    source /opt/ros/jazzy/setup.bash
    source ~/tdt-learning4.5/tdt_contest_ws/install/setup.bash   # 或者新终端（.bashrc 已带）
    nav_record.py                    # 录到 Ctrl+C
    nav_record.py --hz 10            # 降采样（默认 20 Hz）
    nav_record.py --note "贴墙一圈"   # 给这次录制起个名字

输出：/home/king/tdt-learning4.5/nav_data/manual_<时间戳>[_note].jsonl
      （放持久目录，不放 /tmp —— /tmp 会清）

控制模式：必须是你自己开底盘的那种（全手动 / 半自动：手控底盘）
          ROS 管底盘的模式（半自动：手控云台）你没法用键鼠开
"""
import argparse
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from tdt_interface.msg import AutoAimState

OUT_DIR = "/home/king/tdt-learning4.5/nav_data"
UNITS_PER_M = 3.0


class Recorder(Node):
    def __init__(self, path, hz):
        super().__init__("nav_record")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(AutoAimState, "/autoaim/state", self.on_state, qos)
        self.f = open(path, "w")
        self.hz = hz
        self.last = 0.0
        self.n = 0
        self.n_msg = 0
        self.t0 = time.time()
        self.last_report = 0.0
        self.prev_unity = None
        self.dist = 0.0
        self.pts = []
        self.create_timer(2.0, self.report)

    def on_state(self, m):
        self.n_msg += 1
        now = time.time()
        if now - self.last < 1.0 / self.hz:
            return
        self.last = now
        mx, my, mz = m.position.x, m.position.y, m.position.z
        ux, uz = -my, mx                      # map FLU → Unity 世界 (x, z)
        if self.prev_unity is not None:
            self.dist += math.hypot(ux - self.prev_unity[0], uz - self.prev_unity[1])
        self.prev_unity = (ux, uz)
        self.pts.append((ux, uz))
        self.n += 1
        self.f.write(json.dumps({
            "t": now, "ux": ux, "uz": uz, "uy": mz,           # ★ uy = Unity 高度
            "vx": -m.twist.linear.y, "vz": m.twist.linear.x,   # Unity 世界速度
            "yaw": float(m.yaw_degrees), "pitch": float(m.pitch_degrees),
            "hp": int(m.health), "ammo": int(m.ammo_remaining),
            "remain": float(m.remaining_seconds)}) + "\n")
        self.f.flush()

    def report(self):
        if self.prev_unity is None:
            print(f"  …还没收到 /autoaim/state（{time.time()-self.t0:.0f}s）"
                  f" —— 游戏开了吗？这个模式发不发 state？", flush=True)
            return
        ux, uz = self.prev_unity
        print(f"[{time.strftime('%H:%M:%S')}] Unity({ux:+7.2f},{uz:+7.2f})  "
              f"路程 {self.dist/UNITS_PER_M:6.1f} m  {self.n} 点  "
              f"({self.n_msg} 帧)", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hz", type=float, default=20.0, help="录制采样率")
    ap.add_argument("--note", default="", help="给这次录制起个名字，写进文件名")
    args = ap.parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    name = f"manual_{stamp}" + (f"_{args.note}" if args.note else "") + ".jsonl"
    path = os.path.join(OUT_DIR, name)

    rclpy.init()
    rec = Recorder(path, args.hz)
    print(f"被动录制中（只订阅，不发任何指令）→ {path}")
    print("现在用键鼠开你的机器人跑一圈。建议【贴着墙走】，把边界走出来。")
    print("Ctrl+C 结束。\n")
    try:
        rclpy.spin(rec)
    except KeyboardInterrupt:
        pass
    finally:
        rec.f.close()

    if rec.pts:
        xs = [p[0] for p in rec.pts]
        zs = [p[1] for p in rec.pts]
        print(f"\n=== 录了 {(time.time()-rec.t0)/60:.1f} 分钟、{rec.n} 个点 ===")
        print(f"  路程 {rec.dist/UNITS_PER_M:.1f} m")
        print(f"  到过的范围 Unity x ∈ [{min(xs):+.2f}, {max(xs):+.2f}]（{(max(xs)-min(xs))/UNITS_PER_M:.1f} m）")
        print(f"                   z ∈ [{min(zs):+.2f}, {max(zs):+.2f}]（{(max(zs)-min(zs))/UNITS_PER_M:.1f} m）")
        print(f"  文件 → {path}")
    else:
        print("\n✗ 一个点都没录到 —— 检查游戏是否开着、控制模式是否发 state")


if __name__ == "__main__":
    main()
