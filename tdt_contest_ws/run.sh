#!/usr/bin/env bash
# ============================================================================
# 运行自瞄节点
#
#   ./run.sh                              默认【只观察】：检测/跟踪/解算全跑，但一个指令都不发
#   ./run.sh --ros-args -p publish_angles:=true    真的把角度发给游戏（接管云台）
#   ./run.sh --ros-args -p publish_angles:=true -p auto_fire:=true   附带射击许可
#   ./run.sh --ros-args -p show_window:=true       开个窗口看检测效果
#
# 前置条件
#   1. 已执行 ./build.sh
#   2. tdt-campus-game 已启动，并在游戏内按 F8 启用 ROS
#      （游戏自带 ROS 2 桥，监听 127.0.0.1:10000，不要再跑 campusgame_ros 的 start.sh）
# ============================================================================
set -euo pipefail
# 同 build.sh：从 contest_node/ 下的软链调用时也要能定位到工作区
ws="$(cd -- "$(dirname -- "$(realpath "${BASH_SOURCE[0]}")")" && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
source "$ws/install/local_setup.bash"
set -u

export ROS_HOME="$ws/.ros"
export ROS_LOG_DIR="$ws/.ros/log"
mkdir -p "$ROS_LOG_DIR"

# 与游戏侧一致：DOMAIN 0 + 本机发现
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

exec ros2 run auto_aim_contest auto_aim_node \
  --ros-args \
  -p config_file:="$ws/install/auto_aim_contest/share/auto_aim_contest/config_contest.yaml" \
  "$@"
