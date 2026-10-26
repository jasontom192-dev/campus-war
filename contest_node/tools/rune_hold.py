#!/usr/bin/env python3
"""手控底盘调符用：把云台"暂时固定"在打符预设角上（可选强制大符）

全自动里，打符站的"提示角"是 mission_node.py 持续发 "hold <yaw> <pitch> rune"
顶住的（提示 2 秒过期，要靠重发）；手控底盘调符时没有任务节点，这个脚本就干
这一件事：以 5 Hz 向 /mission/cmd 重发同款提示

用法（先起节点、且 publish_angles:=true，云台才归节点管）：
    python3 contest_node/tools/rune_hold.py                     # 默认打符位预设角（小符/按计数）
    python3 contest_node/tools/rune_hold.py --big               # 强制按大符打（调大符用）
    python3 contest_node/tools/rune_hold.py --yaw 30 --pitch -12   # 自己给角
  Ctrl+C 退出时自动补发 "auto"，交回自动搜索

开符由节点自动叫符 / 任务触发；本脚本只负责按住云台提示角
节点看到符后会自己瞄（大符链路）。大符旋钮在
third_party/campus-rune/config/power_rune.json（热生效，改完不用重启）

环境（和其它工具一样）：
    source /opt/ros/jazzy/setup.bash
    source ~/tdt-learning4.5/tdt_contest_ws/install/local_setup.bash
    export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # 和 run.sh 一致
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaw", type=float, default=38.10, help="云台 yaw 提示角（度，游戏系）")
    ap.add_argument("--pitch", type=float, default=-24.20, help="云台 pitch 提示角（度，向下为正）")
    ap.add_argument("--big", action="store_true", help="强制按大符打（隐含打符模式）")
    ap.add_argument("--rate", type=float, default=5.0, help="重发频率 Hz（提示 2s 过期，别低于 1）")
    a = ap.parse_args()

    rclpy.init()
    n = Node("rune_hold")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
    pub = n.create_publisher(String, "/mission/cmd", qos)

    cmd = f"hold {a.yaw:.2f} {a.pitch:.2f} rune" + (" big" if a.big else "")
    print(f'按住云台："{cmd}"（{a.rate:g} Hz 重发；Ctrl+C 退出并发 auto）')
    print("开符由节点自动叫符/任务触发；本脚本只按住云台。")

    last_beat = 0.0
    try:
        while True:
            m = String()
            m.data = cmd
            pub.publish(m)
            now = time.time()
            if now - last_beat > 10.0:      # 心跳：确认脚本还活着
                last_beat = now
                print(f"…按住中（{cmd}）", flush=True)
            time.sleep(1.0 / max(a.rate, 1.0))
    except KeyboardInterrupt:
        pass
    finally:
        stop = String()
        stop.data = "auto"
        for _ in range(6):                  # 多推几拍，确保节点收到
            pub.publish(stop)
            time.sleep(0.05)
        print("已发 auto，交回自动搜索。")
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
