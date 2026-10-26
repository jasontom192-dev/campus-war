#!/usr/bin/env python3
"""把 tdt-campus-test 录制目录里的 meta_epN.jsonl 转成 rune_aim 要的 csv

    python3 tools/meta_to_csv.py <录制目录>/meta_ep1.jsonl [输出.csv]

输出每帧一行：t,yaw,pitch,v
  t     = 录制时的单调秒（只用来排序/对齐，算法不用）
  yaw   = 云台世界 yaw（游戏角度：Unity +Z 为 0、顺时针为正）
  pitch = 云台世界 pitch（游戏角度：向下为正）
  v     = 弹速（场景单位/s，节点里按量级自动判 ÷3）
行序 = 视频帧序（第 0 行对应视频第 0 帧），所以 rune_aim 直接按行号取云台角
"""
import json
import sys


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else src.replace(".jsonl", ".csv")

    n = 0
    with open(src) as f, open(dst, "w") as g:
        g.write("# t,yaw,pitch,v  （由 meta_to_csv.py 从 %s 转出）\n" % src)
        for line in f:
            line = line.strip()
            if not line:
                continue
            m = json.loads(line)
            g.write("%.4f,%.4f,%.4f,%.4f\n" % (
                float(m.get("n") or 0.0),
                float(m.get("yaw") or 0.0),
                float(m.get("pitch") or 0.0),
                float(m.get("v") or 23.5)))
            n += 1
    print(f"{n} 行 -> {dst}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
