#!/bin/bash
# 用 TensorRT 自带的 trtexec 把 ONNX 编成 engine（不需要 python 的 tensorrt 包）
#
# 为什么不用 tools/build_engine.py：那个要求 pip 装得上 tensorrt-cu13，
# 而 10.13 之后 NVIDIA 把 wheel 挪去了 pypi.nvidia.com（本机下不动），
# 所以改走 ~/TensorRT 里的 trtexec —— 功能等价，且不用装任何 python 包
set -e

TRT_DIR="$HOME/TensorRT"
ONNX="/home/king/下载/26_NNDeployment_Lib_and_Detection_Models-main(1)/26_NNDeployment_Lib_and_Detection_Models-main/所有模型/onnx/Infantry-v8n-fp16-20260726-D1.8w-B16.onnx"
ENGINE="$(dirname "$(realpath "$0")")/../models/infantry_v8n.engine"

export LD_LIBRARY_PATH="$TRT_DIR/lib:$TRT_DIR/cudart/usr/local/cuda-13.2/targets/x86_64-linux/lib:$LD_LIBRARY_PATH"

[ -f "$ONNX" ] || { echo "找不到 ONNX: $ONNX"; exit 1; }

echo "== trtexec 版本 =="
"$TRT_DIR/bin/trtexec" --version 2>&1 | head -5 || true

echo
echo "== 开始编译（FP16，480x640 小模型，通常一两分钟）=="
"$TRT_DIR/bin/trtexec" \
  --onnx="$ONNX" \
  --saveEngine="$ENGINE" \
  --fp16 \
  --memPoolSize=workspace:1024 \
  --verbose 2>&1 | grep -viE "^\s*\[[0-9/]+/.*\] (Layer|Node|Tensor|Plugin|Engine generation|Building|Running|Loading|Detecting)" | tail -60

echo
echo "== 产物 =="
ls -la "$ENGINE"
