#!/usr/bin/env bash
# 打一个"换机器可重新编译"的部署包
# 用法: bash deploy/pack.sh [输出目录]      默认 dist/
#
# 原则：
#   · 不带 build/ install/ log/ .ros/ .backup* —— 这些里面全是本机的绝对路径，
#     拷过去只会把 CMakeCache / colcon 软链指回旧机器
#   · 带源码、配置、地图、模型、ONNX、deploy 脚本
#   · TensorRT engine 也带（同 GPU + 同 TRT 可直接用），但换机器默认应重编：
#     目标机跑 deploy.sh engines
set -euo pipefail

ROOT="$(cd "$(dirname "$(realpath "$0")")/.." && pwd)"
OUT_DIR="${1:-$ROOT/dist}"
STAMP="$(date +%Y%m%d-%H%M)"
STAGE="$(mktemp -d)"
DEST="$STAGE/tdt-learning4.5"
ARCHIVE="$OUT_DIR/tdt-learning4.5-deploy-$STAMP.tar.gz"
trap 'rm -rf "$STAGE"' EXIT

mkdir -p "$DEST" "$OUT_DIR"

say() { printf '\033[1;36m[pack]\033[0m %s\n' "$*"; }

# rsync 排除项：任何构建产物/日志/备份都不进包
EX=(--exclude 'build/' --exclude 'install/' --exclude 'log/' --exclude '.ros/'
    --exclude '__pycache__/' --exclude '*.pyc' --exclude '.backup*'
    --exclude 'dist/' --exclude 'CMakeCache.txt' --exclude 'CMakeFiles/')

say "源目录: $ROOT"

# ---- 顶层源码与文档 ----
for p in CMakeLists.txt config.yaml 工程结构说明.md 换模型-TensorRT-说明.md \
         src contest_node tools models nav_data deploy; do
  [ -e "$ROOT/$p" ] || { echo "缺少 $p"; exit 1; }
  rsync -a "${EX[@]}" "$ROOT/$p" "$DEST/"
done

# ---- 第三方库（只带源码；不带它们在根 build/ 下的产物）----
mkdir -p "$DEST/third_party"
for p in NNdeployment campus-rune; do
  rsync -a "${EX[@]}" "$ROOT/third_party/$p" "$DEST/third_party/"
done

# ---- ROS 工作区：脚本 + src 软链；不带 build/install/log ----
mkdir -p "$DEST/tdt_contest_ws"
rsync -a "${EX[@]}" "$ROOT/tdt_contest_ws/build.sh" "$ROOT/tdt_contest_ws/run.sh" \
      "$DEST/tdt_contest_ws/"
rsync -a -l "${EX[@]}" "$ROOT/tdt_contest_ws/src" "$DEST/tdt_contest_ws/"

# ---- tdt_interface 消息包（tdt_contest_ws/src 的软链指向这里）----
mkdir -p "$DEST/campusgame_ros/ros2_auto_framework/src"
rsync -a "${EX[@]}" "$ROOT/campusgame_ros/ros2_auto_framework/src/tdt_interface" \
      "$DEST/campusgame_ros/ros2_auto_framework/src/"

# ---- 地图不带动辄几十 MB 的手工录制轨迹；运行时只要 grid/站点/修正 ----
rsync -a --exclude 'manual_*.jsonl' --exclude 'nav_probe_*.jsonl' \
      "$ROOT/nav_data/" "$DEST/nav_data/"

# ---- 模型 ONNX：换 GPU/TRT 后重编 engine 的原料 ----
ONNX_SRC="/home/king/下载/26_NNDeployment_Lib_and_Detection_Models-main(1)/26_NNDeployment_Lib_and_Detection_Models-main/所有模型/onnx"
mkdir -p "$DEST/models/onnx"
for f in Infantry-v8n-fp16-20260726-D1.8w-B16.onnx Rune-v8n-fp16-20260624.onnx; do
  if [ -f "$ONNX_SRC/$f" ]; then
    cp -a "$ONNX_SRC/$f" "$DEST/models/onnx/"
  else
    echo "警告: 找不到 ONNX $ONNX_SRC/$f（目标机将无法重编 engine）" >&2
  fi
done

# ---- 站点编辑稿（stages_from_md.py 的默认输入；不在运行路径上，但方便改站）----
mkdir -p "$DEST/docs"
if [ -f "/home/king/下载/ai.memory/nav-mission-stages.md" ]; then
  cp -a "/home/king/下载/ai.memory/nav-mission-stages.md" "$DEST/docs/"
fi

# ---- Sophus：不要指望目标机有；头文件 + cmake 配置一共 ~400KB，直接带 ----
if [ -d /usr/local/include/sophus ] && [ -d /usr/local/share/sophus ]; then
  mkdir -p "$DEST/third_party/sophus_install/include" \
           "$DEST/third_party/sophus_install/share"
  rsync -a /usr/local/include/sophus "$DEST/third_party/sophus_install/include/"
  rsync -a /usr/local/share/sophus "$DEST/third_party/sophus_install/share/"
else
  echo "警告: 本机没有 /usr/local/include/sophus，包内不含 Sophus" >&2
fi

# ---- 记录本机的关键版本，目标机 check 时对照 ----
{
  echo "packed_at: $(date -Is)"
  echo "host: $(uname -a)"
  echo "os: $(. /etc/os-release; echo "$PRETTY_NAME")"
  echo "gpu: $(nvidia-smi --query-gpu=name,driver_version,compute_cap --format=csv,noheader 2>/dev/null || echo 'no nvidia-smi')"
  echo "tensorrt: $(/home/king/TensorRT/bin/trtexec --version 2>/dev/null | head -1 || echo unknown)"
  echo "opencv: $(grep -h 'set(OpenCV_VERSION ' /usr/local/lib/cmake/opencv4/OpenCVConfig-version.cmake 2>/dev/null || pkg-config --modversion opencv4 2>/dev/null || echo unknown)"
  echo "ros: ${ROS_DISTRO:-jazzy}"
} > "$DEST/DEPLOY_SOURCE_INFO.txt"

say "打包中..."
tar -C "$STAGE" -czf "$ARCHIVE" tdt-learning4.5
# 用相对文件名，目标机上 `sha256sum -c` 才能直接校验
( cd "$OUT_DIR" && sha256sum "$(basename "$ARCHIVE")" > "$(basename "$ARCHIVE").sha256" )
ls -lh "$ARCHIVE" "$ARCHIVE.sha256"
say "完成。目标机上解压后先读 deploy/README.md"
