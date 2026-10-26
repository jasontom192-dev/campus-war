#!/usr/bin/env python3
"""用 OpenVINO 跑同一个模型的 IR 版本，核对 detector.cpp 里对模型的两条假设：

  ① 输入是 [1,3,480,640]、输出是 [1,21,6300]
     （480=高、640=宽；21 通道 = 4 颜色 + 9 兵种 + 4 关键点×2）

  ② 4 个关键点的顺序到底是不是"左上 左下 右下 右上"
     detector.cpp 里按 [0,3,2,1] 重排过。判据很直接：
       - 若顺序是【左上 左下 右下 右上】，则 p0/p3 在上边、p1/p2 在下边
       - 若是【左上 右上 右下 左下】，则 p0/p1 在上边、p2/p3 在下边
     打印每个检出点的上下/左右关系，一眼就能看出来

IR 和 ONNX 是同一份权重的两种导出，结论对 TensorRT 那边同样成立
"""

import sys

import cv2
import numpy as np
import openvino as ov

IR_XML = ("/home/king/下载/26_NNDeployment_Lib_and_Detection_Models-main(1)"
          "/26_NNDeployment_Lib_and_Detection_Models-main/所有模型/openvino"
          "/Infantry-v8n-fp16-20260726-D1.8w-B16"
          "/Infantry-v8n-fp16-20260726-D1.8w-B16.xml")
VIDEO = ("/home/king/下载/26_NNDeployment_Lib_and_Detection_Models-main(1)"
         "/26_NNDeployment_Lib_and_Detection_Models-main/测试视频/近距离陀螺.avi")

COLORS = ["蓝", "红", "白", "紫"]
ARMORS = ["哨兵", "英雄", "工程", "3号步兵", "4号步兵", "5号步兵",
          "前哨站", "基地(底)", "基地(顶)"]


def main():
    core = ov.Core()
    model = core.read_model(IR_XML)
    for i in range(len(model.inputs)):
        print(f"输入 {model.input(i).any_name}: {model.input(i).shape} "
              f"{model.input(i).element_type}")
    for i in range(len(model.outputs)):
        # IR 里输出张量没带名字，any_name 会抛异常，这里只用形状
        print(f"输出[{i}]: {model.outputs[i].shape} {model.outputs[i].element_type}")

    inp = model.input(0)
    _, _, H, W = inp.shape
    print(f"→ 模型输入 高={H} 宽={W}")

    compiled = core.compile_model(model, "CPU")
    out_layer = compiled.output(0)

    cap = cv2.VideoCapture(VIDEO)
    if not cap.isOpened():
        raise SystemExit(f"打不开视频 {VIDEO}")

    for fid in range(61):
        ok, frame = cap.read()
        if not ok:
            raise SystemExit("视频帧读完了")
    cap.release()
    oh, ow = frame.shape[:2]
    print(f"视频帧 {ow}x{oh}")

    # 纯 resize（部署库的 preprocess 用 pad/scale，4:3 输入到这里 padding 为 0）
    blob = cv2.resize(frame, (W, H)).astype(np.float32) / 255.0
    blob = cv2.cvtColor(blob, cv2.COLOR_BGR2RGB)
    blob = blob.transpose(2, 0, 1)[None]  # NCHW

    out = compiled([blob])[out_layer]
    out = np.asarray(out)
    print(f"推理输出 shape = {out.shape}")

    r = out[0]  # [21, 6300]
    if r.shape[0] != 21:
        raise SystemExit(f"第一维不是 21，而是 {r.shape[0]}")

    scores = r[4:13].max(axis=0)
    order = np.argsort(-scores)[:5]
    sx, sy = ow / W, oh / H

    for k, ci in enumerate(order):
        if scores[ci] < 0.5:
            break
        class_id = int(r[4:13, ci].argmax())
        color_id = int(r[0:4, ci].argmax())
        pts = [(float(r[13 + 2 * j, ci]) * sx, float(r[14 + 2 * j, ci]) * sy)
               for j in range(4)]
        ys = [p[1] for p in pts]
        xs = [p[0] for p in pts]
        top = [j for j in range(4) if ys[j] < np.mean(ys)]
        bottom = [j for j in range(4) if ys[j] >= np.mean(ys)]
        print(f"\n#{k} 兵种={ARMORS[class_id]}({class_id}) "
              f"颜色={COLORS[color_id]}({color_id}) 置信度={scores[ci]:.3f}")
        for j, p in enumerate(pts):
            print(f"    p{j} = ({p[0]:7.1f}, {p[1]:7.1f})")
        print(f"    上边两点={top}  下边两点={bottom}")
        if top == [0, 3]:
            print("    → 顺序 = 左上 左下 右下 右上  ✔ detector.cpp 的 [0,3,2,1] 正确")
        elif top == [0, 1]:
            print("    → 顺序 = 左上 右上 右下 左下  ✘ 应该改成 [0,1,2,3]")
        else:
            print(f"    → 上下关系不符合两种假设（上边={top}），需要人工看盒子")


if __name__ == "__main__":
    sys.exit(main())
