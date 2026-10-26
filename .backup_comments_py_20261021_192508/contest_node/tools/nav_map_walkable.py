#!/usr/bin/env python3
"""能走区域轮廓图 —— 把 nav_grid.npz 里规划器**实际能走**的区域画出来。

跟 nav_map_report / nav_map_clean 的区别：那两张是"地形/覆盖"，这张是**规划视角**——
`FieldMap.walkable`（free 扣除陷坑、再按机器人半径外扩）到底是什么形状。

图层：
    绿  能走（plan/A* 能落点、车能过的区域）
    橙  地形能走、但贴着障碍被外扩吃掉的窄缝（车过不去）
    红  陷坑（硬禁行，实测标定）
    细橙边  等级硬边 = 台阶墙（高代价层，规划器会主动离它远）
    青轮廓  能走区域的边界 —— 就是"能走的轮廓"

用法：
    python3 nav_map_walkable.py [--show]
"""
import argparse
import json
import os
import subprocess
import sys

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageEnhance

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from nav_map import FieldMap, ROBOT_RADIUS          # noqa: E402

DATA = os.path.abspath(os.path.join(HERE, "..", "..", "nav_data"))
BG   = os.path.join(DATA, "tacmap_bg.png")

K = 11.939                 # px / 场景单位
OX, OZ = 240.0, 34.76      # img_x = OX + (OZ - z) * K
OY, OXX = 395.0, -4.86     # img_y = OY - (x - OXX) * K
FX0, FX1, FZ0, FZ1 = -27.4, 17.7, -41.94, 41.94
MCX, MCZ = -4.86, 0.05          # 镜像中心（点反射，实测场地对称）
HAZARD_MARGIN = 1.5


def to_img(x, z):
    return (OX + (OZ - z) * K, OY - (x - OXX) * K)


def fontp(sz, bold=False):
    return ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-%s.ttc"
                              % ("Bold" if bold else "Regular"), sz)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    m = FieldMap()                                   # 直接吃 nav_grid.npz，不重算
    free = m.free & ~m.forbidden
    walk = m.walkable & ~m.forbidden                 # ★ 规划器看到的就是这层
    shaved = free & ~walk                            # 地形能走、被外扩吃掉的

    # ---------- 底图：inpaint UI 标记 + 压暗，让轮廓跳出来 ----------
    im = Image.open(BG).convert("RGB")
    a0 = np.asarray(im).astype(int)
    mk = ((a0[:, :, 0] >= a0[:, :, 1] - 2) |         # 红/粉/橙
          (a0[:, :, 1] > a0[:, :, 2] + 20) |         # 绿
          (a0[:, :, 2] > a0[:, :, 1] + 40)).astype(np.uint8)   # 蓝
    mk = cv2.dilate(mk, np.ones((5, 5), np.uint8), iterations=2)
    im = Image.fromarray(cv2.inpaint(np.asarray(im), mk, 5, cv2.INPAINT_TELEA))
    im = ImageEnhance.Brightness(im).enhance(0.80)   # 留亮：路网要看得见
    im = ImageEnhance.Contrast(im).enhance(1.15)

    x0, y0 = to_img(FX1, FZ1)
    x1, y1 = to_img(FX0, FZ0)
    M = 30
    crop = (int(x0) - M, int(y0) - M, int(x1) + M, int(y1) + M)
    img = im.crop(crop)
    BW, BH = img.size
    cx0, cy0 = crop[0], crop[1]

    # ---------- 把栅格放大到像素（每像素查它落在哪个格子）----------
    py = np.arange(cy0, cy0 + BH)
    px = np.arange(cx0, cx0 + BW)
    Xw = OXX + (OY - py) / K                          # 世界 x 随图像 y 变
    Zw = OZ - (px - OX) / K                           # 世界 z 随图像 x 变
    I = np.clip(np.round((Xw - m.fx0) / m.res).astype(int), 0, m.ni - 1)
    J = np.clip(np.round((Zw - m.fz0) / m.res).astype(int), 0, m.nj - 1)
    ij = lambda layer: layer[I[:, None], J[None, :]]
    walk_px, shaved_px, free_px = ij(walk), ij(shaved), ij(free)

    # ---------- 叠色（半透明）----------
    rgb = np.asarray(img).astype(np.float32)

    def blend(color, mask, alpha):
        a = (mask.astype(np.float32) * alpha)[..., None]
        return rgb * (1 - a) + np.array(color, np.float32) * a

    rgb = blend((58, 108, 158), free_px, 0.22)        # 地形可通行（含被吃掉的）
    rgb = blend((185, 112, 48), shaved_px, 0.55)      # 被外扩吃掉的窄缝
    rgb = blend((52, 170, 108), walk_px, 0.30)        # ★ 能走
    img = np.asarray(rgb).astype(np.uint8)

    # ---------- 轮廓：能走区域的边界 ----------
    u8 = (walk_px * 255).astype(np.uint8)
    u8 = cv2.morphologyEx(u8, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(u8, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cnts = [c for c in cnts if cv2.contourArea(c) > 12]      # 去掉噪点小圈
    cv2.drawContours(img, cnts, -1, (60, 225, 255), 3)        # 青轮廓（RGB）

    img = Image.fromarray(img)
    d = ImageDraw.Draw(img, "RGBA")

    def L(x, z):
        p = to_img(x, z)
        return (p[0] - cx0, p[1] - cy0)

    # ---------- 坐标网格（每 5 单位，细线+边缘标值 —— 方便报坐标指位置）----------
    for z in range(-40, 43, 5):
        px = L(0.0, z)[0]
        if 0 <= px < BW:
            d.line([(px, 0), (px, BH)], fill=(255, 255, 0, 55), width=1)
            d.text((px + 3, 2), f"z{z:+d}", font=fontp(12, True),
                   fill=(255, 255, 130, 210), stroke_width=2, stroke_fill=(0, 0, 0))
    for x in range(-25, 18, 5):
        py = L(x, 0.0)[1]
        if 0 <= py < BH:
            d.line([(0, py), (BW, py)], fill=(255, 255, 0, 55), width=1)
            d.text((4, py + 2), f"x{x:+d}", font=fontp(12, True),
                   fill=(255, 255, 130, 210), stroke_width=2, stroke_fill=(0, 0, 0))

    # ---------- 墙 = 几何折线（和规划器同一份数据；干净直线，不是格子块）----------
    for s in m.wall_segs:
        d.line([L(float(s[0]), float(s[1])), L(float(s[2]), float(s[3]))],
               fill=(255, 90, 30, 245), width=2)

    # ---------- 陷坑 ----------
    haz = []
    pth = os.path.join(DATA, "hazards.json")
    if os.path.exists(pth):
        try:
            haz = json.load(open(pth)).get("pits", [])
        except Exception:
            pass
    for h in haz:
        bx, bz = h["box_x"], h["box_z"]
        a_ = L(bx[0] - HAZARD_MARGIN, bz[1] + HAZARD_MARGIN)
        b_ = L(bx[1] + HAZARD_MARGIN, bz[0] - HAZARD_MARGIN)
        box = [min(a_[0], b_[0]), min(a_[1], b_[1]), max(a_[0], b_[0]), max(a_[1], b_[1])]
        d.rectangle(box, fill=(220, 40, 40, 95), outline=(255, 60, 60, 255), width=3)
        for s in (0, 1):
            d.line([(box[0], box[1] + s * (box[3] - box[1])),
                    (box[2], box[3] - s * (box[3] - box[1]))], fill=(255, 60, 60, 255), width=3)
        d.text((box[0] - 4, box[1] - 20), "陷坑·禁行", font=fontp(13, True),
               fill=(255, 90, 90), stroke_width=3, stroke_fill=(0, 0, 0))

    # ---------- 实测修正框（nav_overrides.json）：坡 + 单向台阶 ----------
    ovp = os.path.join(DATA, "nav_overrides.json")
    if os.path.exists(ovp):
        try:
            ov = json.load(open(ovp))
        except Exception:
            ov = {}

        def _mk_box(b, mir):
            bx, bz = b["box_x"], b["box_z"]
            if mir:
                bx = [2*MCX - bx[1], 2*MCX - bx[0]]
                bz = [2*MCZ - bz[1], 2*MCZ - bz[0]]
            pa = L(bx[0], bz[1]); pb = L(bx[1], bz[0])
            return [min(pa[0], pb[0]), min(pa[1], pb[1]),
                    max(pa[0], pb[0]), max(pa[1], pb[1])]

        for b in ov.get("passable_boxes", []):
            for mir in ([False, True] if b.get("mirror") else [False]):
                r = _mk_box(b, mir)
                d.rectangle(r, outline=(60, 255, 120, 255), width=3)
                # 标签可用 JSON 里的 label 覆盖；默认还是坡那套（老条目没写 label）
                d.text((r[0]+6, r[1]+4), b.get("label", "实测坡·双向可走"),
                       font=fontp(13, True),
                       fill=(60, 255, 120), stroke_width=3, stroke_fill=(0, 0, 0))
        for gg in ov.get("oneway_gates", []):
            for mir in ([False, True] if gg.get("mirror") else [False]):
                r = _mk_box(gg, mir)
                dxg, dzg = gg["dir"]
                if mir:
                    dxg, dzg = -dxg, -dzg
                d.rectangle(r, outline=(255, 170, 50, 255), width=3)
                # 图像方向：px 随 -z、py 随 -x → 箭头向量 = (-dz, -dx)
                cx, cy = (r[0]+r[2])/2, (r[1]+r[3])/2
                ax = -dzg * (r[2]-r[0]) * 0.30
                ay = -dxg * (r[3]-r[1]) * 0.30
                d.line([(cx-ax, cy-ay), (cx+ax, cy+ay)], fill=(255, 220, 80, 255), width=6)
                d.ellipse([cx+ax-10, cy+ay-10, cx+ax+10, cy+ay+10], fill=(255, 220, 80, 255))
                d.text((r[0]+6, r[3]-24), "实测台阶·只能顺箭头", font=fontp(13, True),
                       fill=(255, 190, 60), stroke_width=3, stroke_fill=(0, 0, 0))

    # ---------- 关键点 ----------
    for x, z, text, col in [(-16.50, 9.17, "6号 前哨站", (255, 190, 80)),
                            (-4.93, 34.76, "敌方基地", (255, 110, 110)),
                            (14.52, -34.46, "出生点", (120, 200, 255))]:
        p = L(x, z)
        d.ellipse([p[0] - 5, p[1] - 5, p[0] + 5, p[1] + 5], fill=col,
                  outline=(0, 0, 0, 200), width=2)
        d.text((p[0] + 9, p[1] - 8), text, font=fontp(14, True), fill=col,
               stroke_width=3, stroke_fill=(0, 0, 0))

    # ---------- 标题 / 图例 / 比例尺 ----------
    d.text((16, 12), "能走区域轮廓（规划器视角）",
           font=fontp(17, True), fill=(240, 240, 240), stroke_width=3, stroke_fill=(0, 0, 0))
    d.text((16, 38), f"nav_grid.npz · 机器人半径 {ROBOT_RADIUS} 场景单位外扩后",
           font=fontp(12), fill=(190, 190, 190), stroke_width=3, stroke_fill=(0, 0, 0))

    LX, LY = BW - 306, BH - 208
    d.rectangle([LX, LY, BW - 10, BH - 10], fill=(0, 0, 0, 210),
                outline=(255, 255, 255, 70))
    for i, (c, t) in enumerate([((52, 170, 108), "能走（可规划/可落点）"),
                                ((185, 112, 48), "贴障碍被外扩吃掉（过不去）"),
                                ((255, 60, 60), "陷坑（硬禁行）"),
                                ((255, 90, 30), "台阶墙（硬禁·折线判横穿）"),
                                ((60, 225, 255), "能走区域轮廓"),
                                ((58, 108, 158), "地形可通行（未外扩）")]):
        y = LY + 14 + i * 27
        d.rectangle([LX + 14, y, LX + 36, y + 16], fill=c, outline=(255, 255, 255, 90))
        d.text((LX + 46, y - 1), t, font=fontp(13), fill=(238, 238, 238))

    sx, sy = 20, BH - 26
    L3 = 3.0 * K
    d.rectangle([sx - 9, sy - 22, sx + L3 + 70, sy + 18], fill=(0, 0, 0, 205))
    d.line([(sx, sy), (sx + L3, sy)], fill=(255, 255, 255), width=3)
    for t in (0, L3):
        d.line([(sx + t, sy - 7), (sx + t, sy + 7)], fill=(255, 255, 255), width=3)
    d.text((sx + L3 + 9, sy - 10), "3 m", font=fontp(14, True), fill=(255, 255, 255))

    # ---------- 右上角方向标（图的上=+x 东；左=+z 红方；右=-z 蓝方）----------
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

    out = os.path.join(DATA, "nav_walkable.png")
    img.save(out)

    # ---------- 数字：连通性报告 ----------
    n_lab, lab = cv2.connectedComponents(walk.astype(np.uint8), connectivity=8)
    sizes = sorted((np.bincount(lab.ravel())[1:]).tolist(), reverse=True)
    area = lambda n: n * m.res ** 2 / 9.0
    print(f"能走 {int(walk.sum())} 格 = {area(walk.sum()):.0f} m²"
          f"（地形可通行 {int(free.sum())} 格，被外扩吃掉 {int(shaved.sum())} 格）")
    print(f"连通块 {n_lab - 1} 个，最大 {sizes[0]} 格（占 {sizes[0]/max(int(walk.sum()),1)*100:.0f}%）"
          if sizes else "能走区域为空")
    if len(sizes) > 1:
        print(f"  其余小块：{sizes[1:][:12]}{' …' if len(sizes) > 13 else ''}")
    print(f"图已存 {out}  {img.size[0]}x{img.size[1]}")
    if args.show:
        subprocess.Popen(["eog", out], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


if __name__ == "__main__":
    main()
