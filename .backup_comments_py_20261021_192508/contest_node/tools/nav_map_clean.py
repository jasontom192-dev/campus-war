#!/usr/bin/env python3
"""干净的场地图：底图 = 游戏战术地图截图（方方正正的路网），只叠标注，不画轨迹。

跟 nav_map_report.py 的分工：
    nav_map_report.py   合并驾驶轨迹 → 算覆盖率、出栅格、看还缺哪块（有轨迹线）
    nav_map_clean.py    出图给人和规划器看地形（无轨迹线，几何清楚）

叠上去的东西：补给区、危险区、陷坑（禁行）、敌方 2-7 号活动范围 + 镜像、
基地/前哨站/出生点。全部来自实测（hazards.json / nav_probe / state）。

用法：
    python3 nav_map_clean.py [--show]
"""
import argparse
import collections
import glob
import json
import os
import subprocess

import numpy as np
import cv2
from PIL import Image, ImageDraw, ImageFont, ImageEnhance

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.abspath(os.path.join(HERE, "..", "..", "nav_data"))
BG   = os.path.join(DATA, "tacmap_bg.png")

K = 11.939
OX, OZ = 240.0, 34.76
OY, OXX = 395.0, -4.86
FX0, FX1, FZ0, FZ1 = -27.4, 17.7, -41.94, 41.94
MCX, MCZ = -4.86, 0.05
HAZARD_MARGIN = 1.5


def to_img(x, z):
    return (OX + (OZ - z) * K, OY - (x - OXX) * K)


def load_json(path, key, default):
    if not os.path.exists(path):
        return default
    try:
        return json.load(open(path)).get(key, default)
    except Exception:
        return default


def fontp(sz, bold=False):
    return ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-%s.ttc"
                              % ("Bold" if bold else "Regular"), sz)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    # ---------- 底图：inpaint 掉 UI 标记 + 提亮 ----------
    im = Image.open(BG).convert("RGB")
    a0 = np.asarray(im).astype(int)
    # 战术地图底图是蓝灰系（r < g < b，且 b-g 只有 10~20）；UI 标记是饱和色。
    # ★ 第一版只抓了暖色（红），把绿/蓝标记全漏了 —— 出生点那个绿色 1 号标记
    #   残留下来，被当成"高台"，还造出一圈假硬边。三种色系都要抓：
    mk = ((a0[:, :, 0] >= a0[:, :, 1] - 2) |      # 红/粉/橙（基地、前哨、红方车）
          (a0[:, :, 1] > a0[:, :, 2] + 20) |      # 绿（1 号标记那种 (34,242,128)）
          (a0[:, :, 2] > a0[:, :, 1] + 40)).astype(np.uint8)
    mk = cv2.dilate(mk, np.ones((5, 5), np.uint8), iterations=2)
    im = Image.fromarray(cv2.inpaint(np.asarray(im), mk, 5, cv2.INPAINT_TELEA))
    im = ImageEnhance.Brightness(im).enhance(1.55)
    im = ImageEnhance.Contrast(im).enhance(1.18)

    # 斜坡不再单独着色：真能走的坡，轨迹早就走过了，覆盖率数据里已经有；
    # 检测出来的那圈只是"高地的斜边"，不代表都能上。（判据仍记在 hazards.json 里备查）

    x0, y0 = to_img(FX1, FZ1)
    x1, y1 = to_img(FX0, FZ0)
    M = 30
    crop = (int(x0) - M, int(y0) - M, int(x1) + M, int(y1) + M)
    img = im.crop(crop)
    BW, BH = img.size
    d = ImageDraw.Draw(img, "RGBA")

    def L(x, z):
        px, py = to_img(x, z)
        return (px - crop[0], py - crop[1])

    def rbox(p, q):
        return [min(p[0], q[0]), min(p[1], q[1]), max(p[0], q[0]), max(p[1], q[1])]

    def label(x, z, text, col, sub=None):
        p = L(x, z)
        d.ellipse([p[0]-6, p[1]-6, p[0]+6, p[1]+6], fill=col, outline=(0, 0, 0, 200), width=2)
        d.text((p[0]+10, p[1]-19), text, font=fontp(15, True), fill=col,
               stroke_width=3, stroke_fill=(0, 0, 0))
        if sub:
            d.text((p[0]+10, p[1]+1), sub, font=fontp(11), fill=col,
                   stroke_width=3, stroke_fill=(0, 0, 0))

    # ---------- 区域 ----------
    def zone(xa, za, xb, zb, fill, border, t1, t2):
        box = rbox(L(xa, za), L(xb, zb))
        d.rectangle(box, fill=fill + (85,), outline=border + (255,), width=3)
        _tx = box[0] + 7
        if _tx + 150 > BW:                       # 贴右边就把文字收进框内
            _tx = max(4, box[2] - 150)
        d.text((_tx, box[1]+5), t1, font=fontp(14, True), fill=border,
               stroke_width=3, stroke_fill=(0, 0, 0))
        d.text((_tx, box[1]+24), t2, font=fontp(11), fill=border,
               stroke_width=3, stroke_fill=(0, 0, 0))

    zone(10.5, -36.5, 18.0, -30.5, (60, 210, 120), (60, 210, 120),
         "补给区", "+50HP/s · 停3s补弹")
    zone(-27.5, 39.0, -21.0, 30.5, (235, 70, 70), (235, 70, 70),
         "危险区", "实测 3 次挨打")

    # ---------- 陷坑（禁行）----------
    for h in load_json(os.path.join(DATA, "hazards.json"), "pits", []):
        bx, bz = h["box_x"], h["box_z"]
        box = rbox(L(bx[0]-HAZARD_MARGIN, bz[1]+HAZARD_MARGIN),
                   L(bx[1]+HAZARD_MARGIN, bz[0]-HAZARD_MARGIN))
        d.rectangle(box, fill=(220, 40, 40, 95), outline=(255, 60, 60, 255), width=3)
        for s in (0, 1):
            d.line([(box[0], box[1] + s*(box[3]-box[1])),
                    (box[2], box[3] - s*(box[3]-box[1]))], fill=(255, 60, 60, 255), width=3)
        _ty = box[1] - 20 if box[1] > 26 else box[3] + 4   # 靠顶就往下写，别被切
        d.text((box[0]-4, _ty), "陷坑·禁行", font=fontp(13, True), fill=(255, 90, 90),
               stroke_width=3, stroke_fill=(0, 0, 0))

    # ---------- 关键点 ----------
    label(-4.93, 34.76, "7号 基地", (255, 110, 110), "敌方 5000HP")
    label(-16.50, 9.17, "6号 前哨站", (255, 175, 70), "1500HP")
    label(14.52, -34.46, "出生点 / 补给区", (110, 200, 255), "蓝方")

    # ---------- 敌方单位活动范围 + 镜像 ----------
    box = collections.defaultdict(list)
    for path in glob.glob(os.path.join(DATA, "nav_probe_*.jsonl")):
        try:
            for line in open(path):
                r = json.loads(line)
                if r.get("kind") == "enemies":
                    for e in r["payload"]["enemies"]:
                        box[e["number"]].append((e["x"], e["z"], e["attack_range"]))
        except Exception:
            pass
    for n in sorted(box):
        xs = [v[0] for v in box[n]]; zs = [v[1] for v in box[n]]; ar = box[n][0][2]
        a_ = L(max(xs), max(zs)); b_ = L(min(xs), min(zs))
        lb = rbox(a_, b_)
        d.rectangle(lb, outline=(255, 90, 90, 235), width=2)
        d.text((lb[0]+5, lb[1]+4), f"{n}号", font=fontp(13, True), fill=(255, 130, 130),
               stroke_width=3, stroke_fill=(0, 0, 0))
        if ar > 0:
            d.text((lb[0]+5, lb[1]+21), f"射程{ar/3:.0f}m", font=fontp(10),
                   fill=(255, 160, 160), stroke_width=3, stroke_fill=(0, 0, 0))
        a2 = L(2*MCX - max(xs), 2*MCZ - max(zs)); b2 = L(2*MCX - min(xs), 2*MCZ - min(zs))
        lb2 = rbox(a2, b2)
        d.rectangle(lb2, outline=(110, 200, 255, 170), width=2)
        d.text((lb2[0]+5, lb2[3]-18), f"{n}号镜像", font=fontp(12, True), fill=(130, 210, 255),
               stroke_width=3, stroke_fill=(0, 0, 0))

    # ---------- 比例尺 / 标题 / 图例 ----------
    sx, sy = 20, BH - 26
    L3 = 3.0 * K
    d.rectangle([sx-9, sy-22, sx+L3+70, sy+18], fill=(0, 0, 0, 205))
    d.line([(sx, sy), (sx+L3, sy)], fill=(255, 255, 255), width=3)
    for t in (0, L3):
        d.line([(sx+t, sy-7), (sx+t, sy+7)], fill=(255, 255, 255), width=3)
    d.text((sx+L3+9, sy-10), "3 m", font=fontp(14, True), fill=(255, 255, 255))

    LX, LY = BW - 300, BH - 180
    d.rectangle([LX, LY, BW-10, BH-10], fill=(0, 0, 0, 210), outline=(255, 255, 255, 70))
    for i, (c, t) in enumerate([((60, 210, 120), "补给区"), ((235, 70, 70), "危险区"),
                                ((255, 60, 60), "陷坑（禁行）"),
                                ((255, 90, 90), "敌方 2-7 号活动范围"),
                                ((110, 200, 255), "镜像 → 我方等效位置"),
                                ((110, 200, 255), "出生点")]):
        y = LY + 14 + i*27
        d.rectangle([LX+14, y, LX+36, y+16], fill=c, outline=(255, 255, 255, 90))
        d.text((LX+46, y-1), t, font=fontp(13), fill=(238, 238, 238))

    d.text((16, 14), "TDT Campus Game 场地结构图",
           font=fontp(17, True), fill=(240, 240, 240), stroke_width=3, stroke_fill=(0, 0, 0))
    d.text((16, 40), "底图：游戏战术地图（已去掉 UI 标记）  ·  坐标 Unity 世界 (x,z)，3 场景单位 = 1 m",
           font=fontp(12), fill=(190, 190, 190), stroke_width=3, stroke_fill=(0, 0, 0))
    d.text((16, BH-46), "★ 未走过的区域没有画 —— 这张图是地形，不是覆盖情况（那张看 nav_map_report.py）",
           font=fontp(12), fill=(200, 200, 200), stroke_width=3, stroke_fill=(0, 0, 0))

    out = os.path.join(DATA, "nav_map_clean.png")
    img.save(out)
    print(f"图已存 {out}  {img.size[0]}x{img.size[1]}")
    if args.show:
        subprocess.Popen(["eog", out], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


if __name__ == "__main__":
    main()
