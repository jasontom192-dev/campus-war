#!/usr/bin/env bash
# 编译自瞄比赛节点（tdt_interface 消息 + auto_aim_contest 节点）
set -euo pipefail
# 用 realpath 解析自身真实路径：本脚本在 contest_node/ 下有同名软链，
# 从那儿调用时 $BASH_SOURCE 指向软链，直接取 dirname 会算错工作区
ws="$(cd -- "$(dirname -- "$(realpath "${BASH_SOURCE[0]}")")" && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
set -u

cd "$ws"
colcon build --symlink-install \
  --base-paths src \
  --packages-select tdt_interface auto_aim_contest \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
