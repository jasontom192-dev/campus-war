#!/usr/bin/env python3
"""把 Infantry-v8n 的 ONNX 编译成 TensorRT engine

引擎文件和 GPU 架构、TensorRT 版本是绑死的 —— 换机器、升级 TRT、换模型，
都要重跑这个脚本，不能把 .engine 拷来拷去

用法:
    python3 tools/build_engine.py                    # 用下面的默认路径
    python3 tools/build_engine.py --onnx X --out Y

⚠ 本机【跑不通】这个脚本：它要 `import tensorrt`（pip 的 wheel），而 10.13
  之后 NVIDIA 把真正的 wheel 挪去了 pypi.nvidia.com，这台机器下不动
  功能等价的替代品是 tools/build_engine.sh —— 用 ~/TensorRT/bin/trtexec，
  不需要任何 python 包。本脚本留档：以后在装了 pip tensorrt 的机器上仍可用

模型: Infantry-v8n-fp16-20260726-D1.8w-B16.onnx
    in  images  [1, 3, 480, 640] f32   (RGB, 0~1, NCHW)
    out output0 [1, 21, 6300] f32
        [0:4]   颜色 4 类  蓝 红 白 紫
        [4:13]  兵种 9 类  哨兵 英雄 工程 3/4/5号步兵 前哨站 基地(底) 基地(顶)
        [13:21] 4 个关键点 (x, y)
"""

import argparse
import os
import sys

import tensorrt as trt

DEFAULT_ONNX = (
    "/home/king/下载/26_NNDeployment_Lib_and_Detection_Models-main(1)"
    "/26_NNDeployment_Lib_and_Detection_Models-main/所有模型/onnx"
    "/Infantry-v8n-fp16-20260726-D1.8w-B16.onnx"
)
DEFAULT_OUT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "models", "infantry_v8n.engine",
)

log = trt.Logger(trt.Logger.WARNING)


def build(onnx_path, out_path, fp16):
    logger = trt.Logger(trt.Logger.INFO)
    logger.min_severity = trt.Logger.Severity.WARNING

    builder = trt.Builder(logger)

    # 显式 batch 在 TRT 10 里是唯一模式；旧版这个 flag 还是要显式给一下
    try:
        network = builder.create_network(
            1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    except Exception:
        network = builder.create_network()

    parser = trt.OnnxParser(network, logger)
    with open(onnx_path, "rb") as f:
        if not parser.parse(f.read()):
            for i in range(parser.num_errors):
                print(f"  解析错误: {parser.get_error(i)}", file=sys.stderr)
            raise SystemExit("ONNX 解析失败")

    print("ONNX 输入输出:")
    for i in range(network.num_inputs):
        t = network.get_input(i)
        print(f"  in  {t.name} {t.shape} {t.dtype}")
    for i in range(network.num_outputs):
        t = network.get_output(i)
        print(f"  out {t.name} {t.shape} {t.dtype}")

    config = builder.create_builder_config()
    # 工作空间给 1 GiB：480×640 的小模型绰绰有余
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 30)
    if fp16:
        if builder.platform_has_fast_fp16:
            config.set_flag(trt.BuilderFlag.FP16)
            print("已启用 FP16")
        else:
            print("警告: 本 GPU 不支持快速 FP16，退回 FP32")

    print("编译中（首次会比较慢）...")
    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise SystemExit("编译失败: build_serialized_network 返回 None")

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "wb") as f:
        f.write(serialized)
    print(f"已写出: {out_path}  ({os.path.getsize(out_path) / 1e6:.1f} MB)")

    # 反序列化一遍，确认文件可用并打印最终张量信息
    runtime = trt.Runtime(logger)
    engine = runtime.deserialize_cuda_engine(serialized)
    if engine is None:
        raise SystemExit("反序列化失败，engine 文件不可用")
    print("engine 校验通过:")
    for i in range(engine.num_io_tensors):
        name = engine.get_tensor_name(i)
        print(f"  {engine.get_tensor_mode(name)} {name} "
              f"{engine.get_tensor_shape(name)} {engine.get_tensor_dtype(name)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--onnx", default=DEFAULT_ONNX)
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--fp32", action="store_true", help="关掉 FP16")
    args = ap.parse_args()

    if not os.path.isfile(args.onnx):
        raise SystemExit(f"找不到 ONNX: {args.onnx}")

    print(f"TensorRT {trt.__version__}")
    build(args.onnx, args.out, fp16=not args.fp32)


if __name__ == "__main__":
    main()
