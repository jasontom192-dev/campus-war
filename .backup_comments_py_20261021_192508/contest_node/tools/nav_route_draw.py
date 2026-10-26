#!/usr/bin/env python3
"""把一条任务路线（多个途经点）规划并画到地图上 —— 出图给人和实跑对照。

默认路线：出生点 → 6号前哨站 → （下南台阶）→ 回出生点。

用法：
    python3 nav_route_draw.py                       # 默认环线
    python3 nav_route_draw.py --points "14.5,-34.5 -16.5,9.2 14.5,-34.5"
    python3 nav_route_draw.py --out /tmp/xx.png

规划用和 nav_node 完全一样的口径（threat=台阶墙代价、turn_penalty=0.4、单向闸门）。
"""
import argparse
import math
import os
import subprocess
import sys

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageEnhance

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from nav_map import FieldMap            # noqa: E402

DATA = os.path.abspath(os.path.join(HERE, "..", "..", "nav_data"))
BG   = os.path.join(DATA, "tacmap_bg.png")

K = 11.939
OX, OZ = 240.0, 34.76
OY, OXX = 395.0, -4.86
FX0, FX1, FZ0, FZ1 = -27.4, 17.7, -41.94, 41.94


def to_img(x, z):
    return (OX + (OZ - z) * K, OY - (x - OXX) * K)


def fontp(sz, bold=False):
    return ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-%s.ttc"
                              % ("Bold" if bold else "Regular"), sz)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--points", default="14.5,-34.5 -16.5,9.2 -5.5,-14 14.5,-34.5",
                    help='途经点 "x,z x,z ..."（默认：出生点→前哨站→台阶中段→出生点）')
    ap.add_argument("--out", default=os.path.join(DATA, "nav_roundtrip.png"))
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()
    wps = [tuple(float(v) for v in p.split(",")) for p in args.points.split()]

    m = FieldMap()
    legs = []
    for k in range(len(wps) - 1):
        p = m.plan(wps[k], wps[k + 1], turn_penalty=0.4)
        if p is None:
            print(f"✗ 第{k+1}段 {wps[k]}→{wps[k+1]} 规划失败")
            return
        L = sum(math.hypot(p[i][0] - p[i - 1][0], p[i][1] - p[i - 1][1])
                for i in range(1, len(p)))
        print(f"第{k+1}段 {wps[k]}→{wps[k+1]}: {len(p)} 点 {L:.1f} 单位 = {L/3:.1f} m")
        legs.append(p)
    route = legs[0]
    for p in legs[1:]:
        route = route + p[1:]

    # ---------- 底图 ----------
    im = Image.open(BG).convert("RGB")
    a0 = np.asarray(im).astype(int)
    mk = ((a0[:, :, 0] >= a0[:, :, 1] - 2) | (a0[:, :, 1] > a0[:, :, 2] + 20) |
          (a0[:, :, 2] > a0[:, :, 1] + 40)).astype(np.uint8)
    mk = cv2.dilate(mk, np.ones((5, 5), np.uint8), iterations=2)
    im = Image.fromarray(cv2.inpaint(np.asarray(im), mk, 5, cv2.INPAINT_TELEA))
    im = ImageEnhance.Brightness(im).enhance(0.88)

    x0, y0 = to_img(FX1, FZ1)
    x1, y1 = to_img(FX0, FZ0)
    M = 30
    crop = (int(x0) - M, int(y0) - M, int(x1) + M, int(y1) + M)
    img = im.crop(crop)
    BW, BH = img.size
    cx0, cy0 = crop[0], crop[1]

    # 栅格淡色叠加（能走=绿、墙=橙）
    py = np.arange(cy0, cy0 + BH)
    px = np.arange(cx0, cx0 + BW)
    Xw = OXX + (OY - py) / K
    Zw = OZ - (px - OX) / K
    I = np.clip(np.round((Xw - m.fx0) / m.res).astype(int), 0, m.ni - 1)
    J = np.clip(np.round((Zw - m.fz0) / m.res).astype(int), 0, m.nj - 1)
    walk_px = m.walkable[I[:, None], J[None, :]]
    base = np.asarray(img).astype(np.float32)
    for mask, color, alpha in [(walk_px, (52, 170, 108), 0.18)]:
        a = (mask.astype(np.float32) * alpha)[..., None]
        base = base * (1 - a) + np.array(color, np.float32) * a
    img = Image.fromarray(base.astype(np.uint8))
    d = ImageDraw.Draw(img, "RGBA")

    def L(x, z):
        p = to_img(x, z)
        return (p[0] - cx0, p[1] - cy0)

    # ---------- 网格 ----------
    for z in range(-40, 43, 5):
        pxx = L(0.0, z)[0]
        if 0 <= pxx < BW:
            d.line([(pxx, 0), (pxx, BH)], fill=(255, 255, 0, 50), width=1)
            d.text((pxx + 3, 2), f"z{z:+d}", font=fontp(12, True),
                   fill=(255, 255, 130, 200), stroke_width=2, stroke_fill=(0, 0, 0))
    for x in range(-25, 18, 5):
        pyy = L(x, 0.0)[1]
        if 0 <= pyy < BH:
            d.line([(0, pyy), (BW, pyy)], fill=(255, 255, 0, 50), width=1)
            d.text((4, pyy + 2), f"x{x:+d}", font=fontp(12, True),
                   fill=(255, 255, 130, 200), stroke_width=2, stroke_fill=(0, 0, 0))

    # ---------- 墙 = 几何折线 ----------
    for s in m.wall_segs:
        d.line([L(float(s[0]), float(s[1])), L(float(s[2]), float(s[3]))],
               fill=(255, 90, 30, 245), width=2)

    # ---------- 路线（黑描边 + 亮色芯线）----------
    xy = [L(*p) for p in route]
    d.line(xy, fill=(0, 0, 0, 220), width=11, joint="curve")
    d.line(xy, fill=(120, 230, 255, 255), width=6, joint="curve")
    # 箭头：沿线每 ~10 单位放一个
    acc = 0.0
    for k in range(1, len(route)):
        seg = math.hypot(route[k][0] - route[k-1][0], route[k][1] - route[k-1][1])
        acc += seg
        if acc > 10.0 and k < len(route) - 1:
            acc = 0.0
            a_ = L(*route[k - 1]); b_ = L(*route[k + 1])
            dx, dy = b_[0] - a_[0], b_[1] - a_[1]
            n = math.hypot(dx, dy) or 1.0
            dx, dy = dx / n, dy / n
            p_ = L(*route[k])
            tip = (p_[0] + dx * 14, p_[1] + dy * 14)
            l_ = (p_[0] - dx * 8 - dy * 7, p_[1] - dy * 8 + dx * 7)
            r_ = (p_[0] - dx * 8 + dy * 7, p_[1] - dy * 8 - dx * 7)
            d.polygon([tip, l_, r_], fill=(255, 255, 255, 245), outline=(0, 0, 0, 255))

    # ---------- 途经点标记 ----------
    names = ["起点 出生点", "前哨站", "台阶中段（下行）", "回到出生点"]
    for k, (x, z) in enumerate(wps):
        p_ = L(x, z)
        col = (70, 255, 120) if k == 0 else ((255, 190, 70) if k < len(wps) - 1 else (255, 80, 80))
        d.ellipse([p_[0]-10, p_[1]-10, p_[0]+10, p_[1]+10], fill=col + (255,), outline=(0, 0, 0), width=3)
        tag = names[k] if k < len(names) else f"途经{k}"
        if k == len(wps) - 1:
            d.ellipse([p_[0]-16, p_[1]-16, p_[0]+16, p_[1]+16], outline=col + (255,), width=3)
        d.text((p_[0] + 15, p_[1] - 10), tag, font=fontp(16, True), fill=col,
               stroke_width=3, stroke_fill=(0, 0, 0))
    # 台阶段：回程段里第一次下到 z=-12 的位置（南台阶的下行点，线性插值取准确）
    if len(legs) > 1:
        for k in range(1, len(legs[1])):
            xa, za = legs[1][k - 1]; xb, zb = legs[1][k]
            if za > -12 >= zb:
                t = (za + 12) / max(za - zb, 1e-9)
                xm = xa + t * (xb - xa)
                if -16.5 <= xm <= 4.5:
                    p_ = L(xm, -12)
                    d.ellipse([p_[0]-9, p_[1]-9, p_[0]+9, p_[1]+9], fill=(255, 220, 80, 255),
                              outline=(0, 0, 0), width=3)
                    d.text((p_[0] + 14, p_[1] + 6), "↓ 下台阶（单向）", font=fontp(15, True),
                           fill=(255, 220, 80), stroke_width=3, stroke_fill=(0, 0, 0))
                    break

    # ---------- 右上角方向标 ----------
    cw, ch = 196, 60
    cbx, cby = BW - cw - 8, 6
    d.rectangle([cbx, cby, cbx + cw, cby + ch], fill=(0, 0, 0, 225),
                outline=(255, 255, 255, 100), width=2)
    for i, t in enumerate(["↑  +x 东（东陷坑侧）",
                           "↓  -x 西（西陷坑侧）",
                           "←  +z 红方基地方向",
                           "→  -z 蓝方 / 出生点方向"]):
        d.text((cbx + 8, cby + 2 + i * 14), t, font=fontp(12, True),
               fill=(255, 230, 120, 255), stroke_width=2, stroke_fill=(0, 0, 0))

    d.text((16, 12), "任务路线：出生点 → 前哨站 → 下南台阶 → 回出生点",
           font=fontp(17, True), fill=(240, 240, 240), stroke_width=3, stroke_fill=(0, 0, 0))
    d.text((16, 38), "规划口径与 nav_node 一致（台阶墙代价 + 单向闸门 + 拐弯惩罚）· 浅绿=可走 · 橙=台阶墙",
           font=fontp(12), fill=(200, 200, 200), stroke_width=3, stroke_fill=(0, 0, 0))

    img.save(args.out)
    print(f"图已存 {args.out}  {img.size[0]}x{img.size[1]}")
    if args.show:
        subprocess.Popen(["eog", args.out], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


if __name__ == "__main__":
    main()
