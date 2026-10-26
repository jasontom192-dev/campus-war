#!/usr/bin/env bash
# 目标机上的部署/自检脚本
#
#   bash deploy/deploy.sh check      检查依赖、GPU、TensorRT、engine 能否加载
#   sudo bash deploy/deploy.sh deps  安装 Ubuntu/ROS 侧的依赖（需要联网 + sudo）
#   bash deploy/deploy.sh relocate   把源码/配置里的 /home/king/tdt-learning4.5 改成当前目录
#   bash deploy/deploy.sh build      编 ROS 节点 + 离线工具（不要拷旧 build/install）
#   bash deploy/deploy.sh engines    用包内 ONNX 和本机 trtexec 重编两个 engine
#   bash deploy/deploy.sh run        启动全自动任务（真正发角度/开车）
#   bash deploy/deploy.sh all        relocate → build → engines
#
# 关键：TensorRT 的 .engine 绑定 GPU 架构 + TensorRT 版本。目标机 GPU/TRT 和
# 打包机（RTX 5060 Laptop / sm120 / TRT 10.16.1 / CUDA 13.2）不一致时，
# 必须先 engines，直接跑旧 engine 会反序列化失败
set -euo pipefail

ROOT="$(cd "$(dirname "$(realpath "$0")")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
TRT_DIR="${TRT_DIR:-$HOME/TensorRT}"
ONNX_INF="$ROOT/models/onnx/Infantry-v8n-fp16-20260726-D1.8w-B16.onnx"
ONNX_RUNE="$ROOT/models/onnx/Rune-v8n-fp16-20260624.onnx"

say()  { printf '\033[1;36m[deploy]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[警告]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[错误]\033[0m %s\n' "$*" >&2; exit 1; }

find_trtexec() {
  if [ -x "$TRT_DIR/bin/trtexec" ]; then
    printf '%s\n' "$TRT_DIR/bin/trtexec"
  elif command -v trtexec >/dev/null 2>&1; then
    command -v trtexec
  fi
}

source_ros() {
  if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    # shellcheck disable=SC1090
    set +u; source "/opt/ros/$ROS_DISTRO/setup.bash"; set -u
  else
    warn "没有 /opt/ros/$ROS_DISTRO/setup.bash（ROS 2 $ROS_DISTRO 未安装？）"
  fi
}

cmd_check() {
  say "目录: $ROOT"
  . /etc/os-release 2>/dev/null || true
  echo "OS:      ${PRETTY_NAME:-unknown}  $(uname -m)"
  echo "ROS:     ${ROS_DISTRO:-未设置}  ($( [ -f /opt/ros/$ROS_DISTRO/setup.bash ] && echo 已安装 || echo 未安装 ))"
  echo "nvidia:  $(nvidia-smi --query-gpu=name,driver_version,compute_cap --format=csv,noheader 2>/dev/null | head -1 || echo '无 nvidia-smi')"
  echo "TRT_DIR: $TRT_DIR"
  local trtexec; trtexec="$(find_trtexec || true)"
  if [ -n "$trtexec" ]; then
    echo "trtexec: $trtexec"
    "$trtexec" --version 2>&1 | head -1 || true
  else
    warn "找不到 trtexec（$TRT_DIR/bin/trtexec 或 PATH）"
  fi
  echo "libnvinfer: $(ldconfig -p 2>/dev/null | awk '/libnvinfer\.so/{print $NF; exit}' || true)"
  echo "libcudart:  $(ldconfig -p 2>/dev/null | awk '/libcudart\.so/{print $NF; exit}' || true)"
  echo "OpenCV:     $(pkg-config --modversion opencv4 2>/dev/null || echo 'pkg-config 找不到')"
  [ -f "$ROOT/third_party/sophus_install/share/sophus/cmake/SophusConfig.cmake" ] \
    && echo "Sophus:     包内 $(du -sh "$ROOT/third_party/sophus_install" | cut -f1)" \
    || warn "包内没有 Sophus"
  source_ros
  if command -v ros2 >/dev/null 2>&1; then
    ros2 pkg prefix rclcpp >/dev/null 2>&1 && echo "rclcpp:     $(ros2 pkg prefix rclcpp)" || warn "rclcpp 找不到"
    ros2 pkg prefix tdt_interface >/dev/null 2>&1 && echo "tdt_interface: $(ros2 pkg prefix tdt_interface)" || true
  fi

  echo
  if [ -n "$trtexec" ]; then
    for e in "$ROOT/models/infantry_v8n.engine" "$ROOT/models/rune_v8n.engine"; do
      [ -f "$e" ] || { warn "缺 engine: $e"; continue; }
      say "试加载 $(basename "$e")（失败=必须重编）"
      if "$trtexec" --loadEngine="$e" --skipInference 2>&1 | grep -q "Engine loaded"; then
        echo "  OK: $(basename "$e") 能在本机加载"
      else
        "$trtexec" --loadEngine="$e" --skipInference 2>&1 | tail -5 || true
        warn "$(basename "$e") 加载失败 → 跑 bash deploy/deploy.sh engines"
      fi
    done
  fi
  echo
  say "磁盘: $(df -h "$ROOT" | tail -1 | awk '{print $4" 可用"}')"
}

cmd_deps() {
  [ "$(id -u)" -eq 0 ] || die "请用 sudo 运行: sudo bash deploy/deploy.sh deps"
  . /etc/os-release
  [ "${VERSION_ID:-}" = "24.04" ] || warn "本脚本按 Ubuntu 24.04 / ROS $ROS_DISTRO 写；当前 ${PRETTY_NAME:-unknown}"
  [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ] || die "未安装 ROS 2 $ROS_DISTRO。先按 ROS 官方文档装 ros-$ROS_DISTRO-ros-base（或 desktop），再回来跑这一步。"

  local pkgs=(
    build-essential cmake git pkg-config rsync
    libopencv-dev libeigen3-dev libyaml-cpp-dev
    libceres-dev libgoogle-glog-dev libgflags-dev libfmt-dev libsuitesparse-dev
    python3-opencv python3-numpy python3-pil python3-yaml
    "ros-$ROS_DISTRO-ros-base" "ros-$ROS_DISTRO-rclcpp"
    "ros-$ROS_DISTRO-sensor-msgs" "ros-$ROS_DISTRO-std-msgs" "ros-$ROS_DISTRO-geometry-msgs"
    "ros-$ROS_DISTRO-rosidl-default-generators" "ros-$ROS_DISTRO-rosidl-default-runtime"
    ros-dev-tools python3-colcon-common-extensions
  )
  apt-get update
  apt-get install -y "${pkgs[@]}"
  say "apt 依赖装完。TensorRT/CUDA 不在这里装，见 deploy/README.md 第 2 节。"
}

cmd_relocate() {
  say "把绝对路径改到 $ROOT（只改文本源码/配置，不碰二进制）"
  python3 - "$ROOT" <<'PY'
import os, sys
root = sys.argv[1]
repls = [
    ("/home/king/tdt-learning4.5", root),
    ("/home/king/下载/26_NNDeployment_Lib_and_Detection_Models-main(1)/26_NNDeployment_Lib_and_Detection_Models-main", root + "/models/onnx"),
    ("/home/king/下载/ai.memory/nav-mission-stages.md", root + "/docs/nav-mission-stages.md"),
]
exts = {".cpp", ".hpp", ".h", ".py", ".yaml", ".yml", ".json", ".sh", ".md", ".txt", ".cmake"}
names = {"CMakeLists.txt"}
skip = {"build", "install", "log", ".ros", "dist", "__pycache__", ".git", "deploy"}
changed = 0
for dp, dns, fns in os.walk(root):
    dns[:] = [d for d in dns if d not in skip and not d.startswith(".backup")]
    for fn in fns:
        if os.path.splitext(fn)[1].lower() not in exts and fn not in names:
            continue
        p = os.path.join(dp, fn)
        try:
            s = open(p, encoding="utf-8").read()
        except (UnicodeDecodeError, OSError):
            continue
        t = s
        for a, b in repls:
            t = t.replace(a, b)
        if t != s:
            open(p, "w", encoding="utf-8").write(t)
            changed += 1
print(f"  改了 {changed} 个文本文件")
PY
  # 验证还有没有漏网的旧绝对路径（config.yaml 的离线测试视频除外）
  # deploy/ 里的旧路径是脚本文档自身的说明，relocate 有意不改；
  # build/install/log/dist 是产物，也不参与检查（relocate 本身也会跳过）
  if grep -rIl "/home/king/tdt-learning4.5" "$ROOT" 2>/dev/null \
       | grep -vE '/(deploy|build|install|log|dist|\.ros)/' | head -5 | grep -q .; then
    warn "还有文件含旧工程路径："
    grep -rIl "/home/king/tdt-learning4.5" "$ROOT" 2>/dev/null \
      | grep -vE '/(deploy|build|install|log|dist|\.ros)/' | head -10
  else
    say "旧工程路径已清零"
  fi
}

cmd_build() {
  [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ] || die "ROS $ROS_DISTRO 未安装"
  export CMAKE_PREFIX_PATH="$ROOT/third_party/sophus_install:${CMAKE_PREFIX_PATH:-}"
  # 顶层 CMake 会 glob $HOME/TensorRT*；TRT_DIR 不在默认位置时补一个链接
  if [ -d "$TRT_DIR" ] && [ ! -e "$HOME/TensorRT" ]; then
    ln -s "$TRT_DIR" "$HOME/TensorRT"
    say "已建软链 $HOME/TensorRT -> $TRT_DIR"
  fi
  source_ros

  say "编 ROS 节点（tdt_interface + auto_aim_contest）"
  ( cd "$ROOT/tdt_contest_ws" && ./build.sh )

  say "编离线工具（build/auto_aim、rune_aim、rune_campus）"
  mkdir -p "$ROOT/build"
  ( cd "$ROOT/build" && cmake .. -DCMAKE_BUILD_TYPE=Release && cmake --build . -j"$(nproc)" )
  say "编译完成"
}

cmd_engines() {
  local trtexec; trtexec="$(find_trtexec || true)"
  [ -n "$trtexec" ] || die "找不到 trtexec。装完整 TensorRT tarball 到 $TRT_DIR，或把 trtexec 放进 PATH。"
  [ -f "$ONNX_INF" ] || die "缺 $ONNX_INF"
  [ -f "$ONNX_RUNE" ] || die "缺 $ONNX_RUNE"
  [ -f "$ROOT/models/infantry_v8n.engine" ] && cp -a "$ROOT/models/infantry_v8n.engine" "$ROOT/models/infantry_v8n.engine.bak"
  [ -f "$ROOT/models/rune_v8n.engine" ] && cp -a "$ROOT/models/rune_v8n.engine" "$ROOT/models/rune_v8n.engine.bak"

  # trtexec 依赖 libnvinfer / libcudart；TRT 包里的 cudart 位置各版本不同，能加的目录都加上
  export LD_LIBRARY_PATH="$TRT_DIR/lib:${LD_LIBRARY_PATH:-}"
  for d in "$TRT_DIR"/cudart/usr/local/cuda-*/targets/x86_64-linux/lib \
           /usr/local/cuda/lib64 /usr/local/cuda/lib /usr/local/cuda*/lib64 /usr/local/cuda*/lib; do
    [ -d "$d" ] && export LD_LIBRARY_PATH="$d:$LD_LIBRARY_PATH"
  done

  say "编译 infantry_v8n.engine（FP16）"
  "$trtexec" --onnx="$ONNX_INF" --saveEngine="$ROOT/models/infantry_v8n.engine" \
             --fp16 --memPoolSize=workspace:1024
  say "编译 rune_v8n.engine（FP16）"
  "$trtexec" --onnx="$ONNX_RUNE" --saveEngine="$ROOT/models/rune_v8n.engine" \
             --fp16 --memPoolSize=workspace:1024
  say "engine 完成："
  ls -lh "$ROOT/models"/*.engine
}

cmd_run() {
  source_ros
  export CAMPUS_RUNE_CONFIG="$ROOT/third_party/campus-rune/config/power_rune.json"
  cd "$ROOT/tdt_contest_ws"
  exec ./run.sh --ros-args \
    -p publish_angles:=true -p auto_fire:=true -p mission:=true "$@"
}

cmd_all() {
  cmd_relocate
  cmd_build
  cmd_engines
  cmd_check
}

case "${1:-help}" in
  check)    shift; cmd_check "$@" ;;
  deps)     shift; cmd_deps "$@" ;;
  relocate) shift; cmd_relocate "$@" ;;
  build)    shift; cmd_build "$@" ;;
  engines)  shift; cmd_engines "$@" ;;
  run)      shift; cmd_run "$@" ;;
  all)      shift; cmd_all "$@" ;;
  *) sed -n '2,20p' "$0" ;;
esac
