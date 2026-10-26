#!/usr/bin/env python3
"""喂帧工具：把一段测试视频按 /autoaim/frame 的格式发出去，用来在没有游戏的时候
验证 ROS 节点（解码→检测→跟踪→解算→发角度）整条链路

    # 终端 1：起节点（接管云台 + 允许射击，方便看输出）
    cd ~/tdt-learning4.5/tdt_contest_ws
    ./run.sh --ros-args -p publish_angles:=true -p auto_fire:=true

    # 终端 2：喂帧
    python3 contest_node/tools/publish_test_video.py 近距离陀螺.avi [--loop]

内参用仓库 camera.json 的真值（8mm 镜头、1440×1080），与 config.yaml 的 video 块一致
游戏真跑时这些字段由游戏提供，这个脚本只是替代品
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Header

from tdt_interface.msg import AutoAimFrame, AutoAimState, SendData

VIDEO_DIR = ("/home/king/下载/26_NNDeployment_Lib_and_Detection_Models-main(1)"
             "/26_NNDeployment_Lib_and_Detection_Models-main/测试视频")

# 仓库 camera.json —— 8mm 镜头那台真相机
K = [2319.9, 0.0, 721.4435, 0.0, 2322.7, 606.3215, 0.0, 0.0, 1.0]
D = [-0.0859, 1.065, -0.0011, -0.0016, 0.0]


class Feeder(Node):
    def __init__(self, video, fps, loop, yaw0, pitch0, yaw_rate, raw=False,
                 follow=False, slew_rate=300.0, blank=False,
                 ammo=180, ammo_capacity=180, ammo_run_dry_at=0.0,
                 ammo_refill_at=0.0):
        super().__init__("frame_feeder")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(AutoAimFrame, "/autoaim/frame", qos)
        self.create_subscription(SendData, "/target_angles_player_1",
                                 self.on_cmd, qos)
        # --blank: 发纯黑图（=空场景），用来测"没目标"时的搜索状态
        self.blank = blank
        if blank:
            self.cap = None
            self.w, self.h = 1440, 1080
        else:
            self.cap = cv2.VideoCapture(video)
            if not self.cap.isOpened():
                raise SystemExit(f"打不开视频 {video}")
            self.w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps = fps
        self.loop = loop
        self.yaw0, self.pitch0, self.yaw_rate = yaw0, pitch0, yaw_rate
        self.raw = raw
        # --follow: 模拟游戏侧云台伺服 —— 报出去的 state.yaw_degrees 会以
        # slew_rate(度/秒) 追节点发来的 /target_angles_player_1 指令
        # 开火决策的"云台已到位"那一条就是靠它才能被触发
        self.follow = follow
        self.slew_rate = slew_rate
        self.gimbal_yaw = yaw0
        self.cmd_yaw = None
        # 弹药：默认填一个正常值（节点默认 min_ammo_to_fire=0，
        # 字段恒为 0 会让它一直停在"无弹"态）
        #   --ammo-run-dry-at T  第 T 秒打空（测 跟踪→无弹）
        #   --ammo-refill-at T   第 T 秒补满（测 无弹→跟踪/搜索）
        self.ammo = ammo
        self.ammo_capacity = ammo_capacity
        self.ammo_run_dry_at = ammo_run_dry_at
        self.ammo_refill_at = ammo_refill_at
        self.seq = 0
        self.t0 = time.time()
        self.create_timer(1.0 / fps, self.tick)
        self.get_logger().info(
            f"喂帧中: {os.path.basename(video)} {self.w}x{self.h} @{fps}fps "
            f"(topic=/autoaim/frame, 只有压缩图像载荷)")

    def on_cmd(self, m: SendData):
        self.cmd_yaw = m.yaw
        self.get_logger().info(
            f"节点回令: yaw={m.yaw:+7.2f}° pitch={m.pitch:+6.2f}° "
            f"if_shoot={'是' if m.if_shoot else '否'} "
            f"(云台在 {self.gimbal_yaw:+7.2f}°)")

    def tick(self):
        if self.blank:
            frame = np.zeros((self.h, self.w, 3), np.uint8)
        else:
            ok, frame = self.cap.read()
        if not self.blank and not ok:
            if not self.loop:
                self.get_logger().info("视频放完，退出")
                raise SystemExit(0)
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
            if not ok:
                raise SystemExit(0)

        self.seq += 1
        # ★ 同一个 hdr 对象被复用到【所有】嵌套 header 上（外层 + state +
        #   camera_info + image）。节点会逐个核对它们的时间戳是否与外层一致
        #   （规则 7.2），这里共用一个就天然满足
        #   想让节点丢帧做故障演练，就单独改其中一个 header 的 stamp
        now = self.get_clock().now().to_msg()
        hdr = Header(stamp=now, frame_id="camera")

        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            return
        ci = CameraInfo()
        ci.header = hdr
        ci.width, ci.height = self.w, self.h
        ci.k = K
        ci.d = D
        ci.distortion_model = "plumb_bob"

        st = AutoAimState()
        st.header = hdr
        st.frame_sequence = self.seq
        # 云台姿态：视频本身是固定相机拍的，这里给的值只是"告诉节点云台转到哪了"，
        # 用来验证世界系旋转（--yaw 30 表示云台已经顺时针转了 30°）
        if self.follow:
            if self.cmd_yaw is not None:      # 按 slew_rate 追最新指令
                step = self.slew_rate / self.fps
                d = self.cmd_yaw - self.gimbal_yaw
                self.gimbal_yaw += max(-step, min(step, d))
            st.yaw_degrees = self.gimbal_yaw
        else:
            st.yaw_degrees = self.yaw0 + self.yaw_rate * (time.time() - self.t0)
        st.pitch_degrees = self.pitch0
        st.projectile_speed = 25.0
        st.health = st.max_health = 100

        # 弹药随时间变化，用来演练"打空 → 无弹 → 补给 → 恢复"整条链
        elapsed = time.time() - self.t0
        ammo = self.ammo
        if self.ammo_run_dry_at > 0 and elapsed >= self.ammo_run_dry_at:
            ammo = 0
        if self.ammo_refill_at > 0 and elapsed >= self.ammo_refill_at:
            ammo = self.ammo_capacity
        st.ammo_remaining = ammo
        st.ammo_capacity = self.ammo_capacity

        msg = AutoAimFrame()
        msg.header = hdr
        msg.state = st
        msg.camera_info = ci
        if self.raw:
            # 未压缩载荷：rgb8（游戏两种都可能发，这里用来对比 JPEG 的影响）
            # ★ OpenCV 读出来是 BGR，而 encoding="rgb8" 声明的是 RGB，
            #   所以必须 cvtColor 换序后再 tobytes() —— 节点那边会再转回 BGR
            #   漏了这一步就是"红蓝互换"，且因为两边对称，光看代码很难发现
            from sensor_msgs.msg import Image
            im = Image()
            im.header = hdr
            im.height, im.width = frame.shape[:2]
            im.encoding = "rgb8"
            im.step = frame.shape[1] * 3
            im.data = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB).tobytes()
            msg.image = im
        else:
            msg.compressed_image.header = hdr
            msg.compressed_image.format = "jpeg"
            msg.compressed_image.data = bytes(buf.tobytes())
        msg.capture_to_publish_ms = 1.0   # 游戏给的"采集到发布"耗时，这里填个示意值
        self.pub.publish(msg)

        if self.seq % 30 == 0:
            self.get_logger().info(f"已发 {self.seq} 帧，实际 {self.seq/(time.time()-self.t0):.1f} fps")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video", help="测试视频文件名（在仓库 测试视频/ 目录下）")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--yaw", type=float, default=0.0, help="云台当前 yaw（度）")
    ap.add_argument("--pitch", type=float, default=0.0, help="云台当前 pitch（度，向下为正）")
    ap.add_argument("--yaw-rate", type=float, default=0.0, help="云台 yaw 角速度（度/秒）")
    ap.add_argument("--raw", action="store_true", help="发未压缩 rgb8 而不是 JPEG")
    ap.add_argument("--blank", action="store_true", help="发纯黑图（空场景）")
    ap.add_argument("--follow", action="store_true",
                    help="模拟云台伺服：报出的 yaw 追节点指令")
    ap.add_argument("--slew-rate", type=float, default=300.0, help="云台角速度（度/秒）")
    ap.add_argument("--ammo", type=int, default=180, help="初始弹量（默认 180）")
    ap.add_argument("--ammo-capacity", type=int, default=180, help="弹量容量")
    ap.add_argument("--ammo-run-dry-at", type=float, default=0.0,
                    help="第几秒打空弹量（0=不演练）")
    ap.add_argument("--ammo-refill-at", type=float, default=0.0,
                    help="第几秒补满弹量（0=不演练）")
    a = ap.parse_args()

    path = a.video if os.path.isabs(a.video) else os.path.join(VIDEO_DIR, a.video)
    if not a.blank and not os.path.isfile(path):
        raise SystemExit(f"找不到视频 {path}")

    rclpy.init()
    try:
        rclpy.spin(Feeder(path, a.fps, a.loop, a.yaw, a.pitch, a.yaw_rate, a.raw,
                       a.follow, a.slew_rate, a.blank))
    except (KeyboardInterrupt, SystemExit) as e:
        if isinstance(e, SystemExit) and e.code not in (0, None):
            sys.exit(e.code)
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
