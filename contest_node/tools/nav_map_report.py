#!/usr/bin/env python3
"""把录到的驾驶轨迹合成一张场地通行图，并报告覆盖率

把 nav_data/ 目录下**所有** manual_*.jsonl 当成一次探索（多轮自动合并），
叠到游戏自带的战术地图截图上，输出：

    nav_data/nav_map_real.png    通行图（可通行 / 高地 / 没走到 / 轨迹 / 敌方活动范围）
    nav_data/nav_grid.npz        栅格地图，给导航用（含 free 掩膜与坐标变换）

因为不发任何指令、不连 ROS，跑完一轮随时可以打开看进度

用法：
    python3 nav_map_report.py                # 合并全部录制出图
    python3 nav_map_report.py --show         # 出图后自动打开

坐标变换（用两个基地徽记标定，2026-10-01 对话里推的）：
    img_x = 240 + (34.76 - z) * 11.939      # 红基地 z=+34.76 落在 px240
    img_y = 395 - (x + 4.86)  * 11.939      # 两个基地徽记都在中线
颜色分层（实测）：
    sum  66  低层地面（基地/边界层）  可通行
    sum 216  公路面                   可通行
    sum 240  高地表面（抬升一档）      可通行
    sum 296+ 结构                     障碍
    暖色     战术地图 UI 标记（车/基地/前哨）→ 先 inpaint 掉，别留黑洞
"""
import argparse
import collections
import glob
import json
import math
import os
import subprocess

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.abspath(os.path.join(HERE, "..", "..", "nav_data"))
BG   = os.path.join(DATA, "tacmap_bg.png")

K = 11.939                 # px / 场景单位
OX, OZ = 240.0, 34.76      # img_x = OX + (OZ - z) * K
OY, OXX = 395.0, -4.86     # img_y = OY - (x - OXX) * K

# 场地范围（空气墙，实测反推）
FX0, FX1, FZ0, FZ1 = -27.4, 17.7, -41.94, 41.94
MCX, MCZ = -4.86, 0.05     # 场地中心，镜像用

SWEEP = 3.0                # 轨迹按多宽的走廊计入“走过”（场景单位）
GRID_RES = 1.0             # 栅格分辨率（场景单位/格）


def to_img(x, z):
    return (OX + (OZ - z) * K, OY - (x - OXX) * K)


def unity_to_cell(x, z):
    return (int(round((x - FX0) / GRID_RES)), int(round((z - FZ0) / GRID_RES)))


def cell_to_unity(i, j):
    return (FX0 + i * GRID_RES, FZ0 + j * GRID_RES)


def load_tracks(min_pts=500):
    """读 nav_data 下所有 manual_*.jsonl，返回 [(名字, [(x,z), ...]), ...]"""
    out = []
    for path in sorted(glob.glob(os.path.join(DATA, "manual_*.jsonl"))):
        pts = []
        for line in open(path):
            try:
                r = json.loads(line)
                pts.append((r["ux"], r["uz"]))
            except Exception:
                pass
        if len(pts) >= min_pts:
            out.append((os.path.basename(path), pts))
    return out


def load_enemies():
    """读 nav_probe 系列里的敌方广播，返回 {号: [(x,z,attack_range), ...]}"""
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
    return box


HAZARD_MARGIN = 1.5        # 洞外扩这么多（场景单位）当禁行：机器人半宽 + 余量
EDGE_GRAD = 200.0          # 等级硬边的梯度阈值（斜坡实测 40~90，硬边 240~340，取中间偏上）


def load_hazards():
    """读 nav_data/hazards.json 里实测的陷坑"""
    p = os.path.join(DATA, "hazards.json")
    if not os.path.exists(p):
        return []
    try:
        return json.load(open(p)).get("pits", [])
    except Exception as e:
        print(f"⚠ hazards.json 读不了：{e}")
        return []


def load_overrides():
    """读 nav_data/nav_overrides.json 里实测的路况修正（坡/单向台阶）

    这些是"车实际能走、但像素颜色和梯度判不出来"的地方 —— 实测标定，别名改
    """
    p = os.path.join(DATA, "nav_overrides.json")
    if not os.path.exists(p):
        return {"passable_boxes": [], "wall_boxes": [], "oneway_gates": []}
    try:
        d = json.load(open(p))
        return {"passable_boxes": d.get("passable_boxes", []),
                "wall_boxes": d.get("wall_boxes", []),
                "oneway_gates": d.get("oneway_gates", [])}
    except Exception as e:
        print(f"⚠ nav_overrides.json 读不了：{e}")
        return {"passable_boxes": [], "wall_boxes": [], "oneway_gates": []}


def build_level_barriers():
    """等级硬边 = 墙

    场地是分层的（实测亮度：66 低层 / 216 公路 / 240 高地 / 296 结构 / 433 中央顶）
    两层之间怎么连：
      · **渐变过渡**（跨 ~1.3 单位、|∇|≈40~90）= 斜坡，能走
      · **硬边跳变**（1~2 像素、|∇|≈240~340）= 台阶墙，走不上去

    2026-10-01 实测：机器人在 Unity(-11.8,-12.2) 撞的就是硬边 ——
    x=-11.5 还是 66，x=-11.0 就跳到 255，0.5 单位内差 189

    斜坡的梯度远低于阈值，所以不会误伤；规划器拿到这层就会自动离边界远一点，
    不会被 pure pursuit 的切角带上去
    """
    im = Image.open(BG).convert("RGB")
    a0 = np.asarray(im).astype(int)
    # 战术地图底图是蓝灰系（r < g < b，且 b-g 只有 10~20）；UI 标记是饱和色
    # ★ 第一版只抓了暖色（红），把绿/蓝标记全漏了 —— 出生点那个绿色 1 号标记
    #   残留下来，被当成"高台"，还造出一圈假硬边。三种色系都要抓：
    mk = ((a0[:, :, 0] >= a0[:, :, 1] - 2) |      # 红/粉/橙（基地、前哨、红方车）
          (a0[:, :, 1] > a0[:, :, 2] + 20) |      # 绿（1 号标记那种 (34,242,128)）
          (a0[:, :, 2] > a0[:, :, 1] + 40)).astype(np.uint8)
    mk = cv2.dilate(mk, np.ones((5, 5), np.uint8), iterations=2)
    im = Image.fromarray(cv2.inpaint(np.asarray(im), mk, 5, cv2.INPAINT_TELEA))

    lum = np.asarray(im).astype(np.float32).sum(2)
    lum = cv2.GaussianBlur(lum, (0, 0), 0.8)          # 轻微平滑，去掉单像素噪声
    gx = cv2.Sobel(lum, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(lum, cv2.CV_32F, 0, 1, ksize=3)
    g = np.hypot(gx, gy)
    edge = (g > EDGE_GRAD).astype(np.uint8)           # 200：远高于斜坡的 40~90
    edge = cv2.dilate(edge, np.ones((5, 5), np.uint8))  # 外扩，别贴着墙规划
    return edge, g


def build_wall_polylines():
    """把硬边（墙）描成**几何折线** —— 规划器用"线段横穿"判穿墙

    ★ 为什么不用"墙格 + 法线"：1 单位的格子里分不开"墙脸"和"墙根的路"
      （用户沿墙的合法走廊和墙标记共用格子），方向法线又在拐角处总指错
      （实测 (14.6,-32) 指反 90°，反复穿墙）。折线是几何对象，横穿与否
      一次相交测试就定了，不需要任何方向估计
    输出：每段墙 → (x1,z1,x2,z2)，世界系场景单位
    """
    im = Image.open(BG).convert("RGB")
    a0 = np.asarray(im).astype(int)
    mk = ((a0[:, :, 0] >= a0[:, :, 1] - 2) | (a0[:, :, 1] > a0[:, :, 2] + 20) |
          (a0[:, :, 2] > a0[:, :, 1] + 40)).astype(np.uint8)
    mk = cv2.dilate(mk, np.ones((5, 5), np.uint8), iterations=2)
    im = Image.fromarray(cv2.inpaint(np.asarray(im), mk, 5, cv2.INPAINT_TELEA))
    lum = np.asarray(im).astype(np.float32).sum(2)
    lum = cv2.GaussianBlur(lum, (0, 0), 0.8)
    gx = cv2.Sobel(lum, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(lum, cv2.CV_32F, 0, 1, ksize=3)
    m = np.hypot(gx, gy) > EDGE_GRAD          # 原始细线，不膨胀
    ys, xs = np.nonzero(m)
    pts = set(zip(ys.tolist(), xs.tolist()))
    NEB = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

    def deg(p):
        return sum(((p[0] + dy, p[1] + dx) in pts) for dy, dx in NEB)

    visited = set()
    chains = []

    def walk(s):
        chain, cur = [s], s
        visited.add(s)
        while True:
            nxt = None
            for dy, dx in NEB:
                q = (cur[0] + dy, cur[1] + dx)
                if q in pts and q not in visited:
                    nxt = q
                    break
            if nxt is None:
                return chain
            visited.add(nxt)
            chain.append(nxt)
            cur = nxt

    for s in [p for p in pts if deg(p) == 1]:       # 先描端点开始的线
        if s not in visited:
            c = walk(s)
            if len(c) >= 6:
                chains.append(c)
    for p in pts:                                    # 剩余的环
        if p not in visited:
            c = walk(p)
            if len(c) >= 6:
                chains.append(c)

    segs = []
    for c in chains:
        d = c[::4] + [c[-1]]                         # 降采样，折线顶点
        for k in range(1, len(d)):
            y1, x1 = d[k - 1]
            y2, x2 = d[k]
            wx1 = OXX + (OY - y1) / K
            wz1 = OZ - (x1 - OX) / K
            wx2 = OXX + (OY - y2) / K
            wz2 = OZ - (x2 - OX) / K
            segs.append((wx1, wz1, wx2, wz2))
    return np.asarray(segs, np.float32)


def build_masks():
    """从战术地图截图里分出 可通行 / 高地 / 障碍，并把 UI 标记 inpaint 掉

    返回 (passable, highland, sm)：sm 是逐像素的亮度等级和（66 低层 / 216 公路 /
    240 高地 / 296 台座 / 433 结构顶）。它单独带出来是给 overrides 里的
    `passable_boxes.levels` 用的 —— 让修正框能"只开放某个等级带"的格子
    （比如中央高地顶：开放 296 台座、保住 433 椭圆体本身）
    """
    im0 = Image.open(BG).convert("RGB")
    a0 = np.asarray(im0).astype(int)
    # 战术地图底图是蓝灰系（r < g < b，且 b-g 只有 10~20）；UI 标记是饱和色
    # ★ 第一版只抓了暖色（红），把绿/蓝标记全漏了 —— 出生点那个绿色 1 号标记
    #   残留下来，被当成"高台"，还造出一圈假硬边。三种色系都要抓：
    mk = ((a0[:, :, 0] >= a0[:, :, 1] - 2) |      # 红/粉/橙（基地、前哨、红方车）
          (a0[:, :, 1] > a0[:, :, 2] + 20) |      # 绿（1 号标记那种 (34,242,128)）
          (a0[:, :, 2] > a0[:, :, 1] + 40)).astype(np.uint8)
    mk = cv2.dilate(mk, np.ones((5, 5), np.uint8), iterations=2)
    im0 = Image.fromarray(cv2.inpaint(np.asarray(im0), mk, 5, cv2.INPAINT_TELEA))

    a = np.asarray(im0).astype(int)
    sm = a.sum(2)
    r, g, b = a[:, :, 0], a[:, :, 1], a[:, :, 2]
    fieldish = (r < g - 2) & (b >= g) & (g >= r)
    passable = fieldish & (sm > 40) & (sm < 262)
    highland = fieldish & (sm >= 228) & (sm < 262)
    return passable, highland, sm


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--show", action="store_true", help="出图后自动打开")
    ap.add_argument("--sweep", type=float, default=SWEEP, help="轨迹走廊宽度（场景单位）")
    args = ap.parse_args()

    tracks = load_tracks()
    if not tracks:
        print(f"✗ {DATA} 下没有 manual_*.jsonl —— 先跑 nav_record.py 录一轮")
        return
    print("载入轨迹：")
    for name, pts in tracks:
        d = sum(math.hypot(pts[i][0] - pts[i-1][0], pts[i][1] - pts[i-1][1])
                for i in range(1, len(pts)))
        print(f"  {name:<44} {len(pts):5d} 点  {d/3:6.0f} m")

    passable, highland, lvl_sum = build_masks()
    edge_px, _grad = build_level_barriers()
    wall_segs = build_wall_polylines()          # 墙的几何折线（规划器判"横穿"用）

    # ---------- 裁到场地 ----------
    x0, y0 = to_img(FX1, FZ1)
    x1, y1 = to_img(FX0, FZ0)
    M = 26
    cx0, cy0, cx1, cy1 = int(x0) - M, int(y0) - M, int(x1) + M, int(y1) + M
    BW, BH = cx1 - cx0, cy1 - cy0
    PAS = passable[cy0:cy1, cx0:cx1]
    HIGH = highland[cy0:cy1, cx0:cx1]
    LVL = lvl_sum[cy0:cy1, cx0:cx1]               # 等级和（裁到和 PAS 同一个局部坐标系）
    EDGE = edge_px[cy0:cy1, cx0:cx1] > 0          # 等级硬边 = 墙

    def to_local(x, z):
        px, py = to_img(x, z)
        return (px - cx0, py - cy0)

    # ---------- 覆盖：轨迹扫过的走廊 ----------
    hit = Image.new("L", (BW, BH), 0)
    hd = ImageDraw.Draw(hit)
    for _, pts in tracks:                   # ★ 逐条画，不能把所有点拼成一条线 ——
        xy = [to_local(*p) for p in pts]    #   那样会在两轮之间连出根本不存在的直线
        if len(xy) > 1:
            hd.line(xy, fill=255, width=int(args.sweep * K), joint="curve")
    hit = np.asarray(hit.filter(ImageFilter.GaussianBlur(2))) > 90
    cov = (hit & PAS).sum() / max(PAS.sum(), 1)
    print(f"\n可通行 {PAS.sum()/(K*K)/9:.0f} m²（其中高地 {HIGH.sum()/(K*K)/9:.0f} m²）")
    print(f"轨迹够到 {cov*100:.1f}% 的可通行区域  ← 多跑几轮把这个数字顶上去")

    # ---------- 保存栅格 ----------
    gi, gj = np.meshgrid(
        np.arange(int(round((FX1-FX0)/GRID_RES)) + 1),
        np.arange(int(round((FZ1-FZ0)/GRID_RES)) + 1), indexing="ij")
    gx, gz = cell_to_unity(gi, gj)
    gpx = np.clip(((OX + (OZ - gz) * K) - cx0).astype(int), 0, BW-1)
    gpy = np.clip(((OY - (gx - OXX) * K) - cy0).astype(int), 0, BH-1)
    free = PAS[gpy, gpx]
    high = HIGH[gpy, gpx]
    # 每个栅格点所在的像素等级（供 overrides 的 levels 过滤用，见 build_masks 注释）
    # ★ 必须用裁剪过的 LVL —— 和 PAS/HIGH 同一个局部坐标系；拿全图数组配
    #   局部坐标会整片错位（试过一次：框全落空、台座一格没开）
    lvl = LVL[gpy, gpx]

    # ★ 填针孔：栅格采样的 free 会有零散的单格空洞（轨迹没精确扫到），
    #   后果是"机器人自己站的那格不可走"，规划器只能把起点挪到很远的地方，
    #   于是指令指向一个奇怪的方向（实测：出生点被挪到 3 单位外、直奔墙角）
    #   闭运算把 ≤2 格的小洞填掉，大洞（真正没走过的地方）保留
    _k = np.ones((3, 3), np.uint8)
    free = cv2.morphologyEx(free.astype(np.uint8), cv2.MORPH_CLOSE, _k).astype(bool)
    free = cv2.morphologyEx(free.astype(np.uint8), cv2.MORPH_CLOSE, _k).astype(bool)

    # ★ 陷坑 → 禁行层。地图颜色分不出洞，只能靠 hazards.json 里实测的框，再外扩一点
    haz = load_hazards()
    forbidden = np.zeros_like(free)
    for h in haz:
        bx, bz = h["box_x"], h["box_z"]
        m = ((gx >= bx[0] - HAZARD_MARGIN) & (gx <= bx[1] + HAZARD_MARGIN) &
             (gz >= bz[0] - HAZARD_MARGIN) & (gz <= bz[1] + HAZARD_MARGIN))
        forbidden |= m
        free &= ~m                       # 顺手从可通行里抠掉，双保险

    # ★ 等级硬边 → 单独存一层 barrier，**不**从 free 里抠掉
    #   抠掉会把本来就不宽的走廊切碎（实测：free 只掉 12%，但外扩后掉 37%，
    #   四条测试路径全废）。改成让规划器加高代价 —— 会主动离墙远，实在没路还能过
    barrier = np.zeros_like(free)
    for _i in range(free.shape[0]):
        for _j in range(free.shape[1]):
            _px, _py = gpx[_i, _j], gpy[_i, _j]
            # ★ 采样半径是"像素"：0.6 场景单位 = 0.6*K ≈ 7 px
            #   旧写法 r = int(0.6/GRID_RES) = int(0.6) = 0 → 只看格子中心 1 个像素，
            #   硬边线不从正中心过就漏标（车从"缝"里钻过去撞墙，实测多次）
            r = max(1, int(round(0.6 * K)))
            if EDGE[max(0, _py-r):_py+r+1, max(0, _px-r):_px+r+1].any():
                barrier[_i, _j] = True

    # ★ 实测修正（nav_overrides.json）：坡 = 强制双向可通行；台阶 = 单向闸门
    #   这些框叠在像素判据之上 —— 像素看不出的真实路况以实测为准
    forced = np.zeros_like(free)                     # 强制可通行格（免疫障碍外扩）
    gate_dx = np.zeros(free.shape, np.int8)          # 单向闸门：允许方向（0,0 = 非闸门）
    gate_dz = np.zeros(free.shape, np.int8)
    ov = load_overrides()

    def _box(bx, bz, mirror=False):
        if mirror:                                   # 点反射镜像（整张图对称）
            bx = [2*MCX - bx[1], 2*MCX - bx[0]]
            bz = [2*MCZ - bz[1], 2*MCZ - bz[0]]
        return (gx >= bx[0]) & (gx <= bx[1]) & (gz >= bz[0]) & (gz <= bz[1])

    # 陷坑的"精确洞体"（不含外扩余量）—— 修正框绝不能把洞本身打开
    hole = np.zeros_like(free)
    for h in haz:
        bx, bz = h["box_x"], h["box_z"]
        hole |= (gx >= bx[0]) & (gx <= bx[1]) & (gz >= bz[0]) & (gz <= bz[1])

    # ★ 实测补墙（wall_boxes）：这些地方的墙标记太细，斜向格间留"缝"，A* 会从缝里
    #   钻过去撞墙（实测）。框内把 barrier 膨胀一格：填缝 + 加宽到 2 格，墙本身不动
    #   注意顺序：在 passable/闸门之前做，让它们该清的还是清
    n_wall = 0
    for wb in ov.get("wall_boxes", []):
        # ★ 2026-10-05：支持 "grow": N —— 框内 barrier 膨胀 N 次（默认 1 次，
        #   即老行为"填缝 + 加宽到 2 格"）。前哨站墙加厚用的就是 grow=2
        for _ in range(max(1, int(wb.get("grow", 1)))):
            for mir in ([False, True] if wb.get("mirror") else [False]):
                m = _box(wb["box_x"], wb["box_z"], mir)
                grown = cv2.dilate(barrier.astype(np.uint8),
                                   np.ones((3, 3), np.uint8)).astype(bool)
                add = grown & m & ~barrier
                barrier |= add
                n_wall += int(add.sum())

    n_box = 0
    for b in ov["passable_boxes"]:
        for mir in ([False, True] if b.get("mirror") else [False]):
            m = _box(b["box_x"], b["box_z"], mir) & ~hole
            # ★ 可选 levels=[lo,hi)：框内只开放等级落在这个区间的格子
            #   中央高地顶的实测修正靠它 —— 296 台座要开通、433 椭圆体必须
            #   保持禁行，一个矩形框做不到"只开外圈"，加等级过滤就精确了
            if b.get("levels"):
                lo, hi = b["levels"]
                m &= (lvl >= lo) & (lvl < hi)
            free |= m; forbidden &= ~m; barrier &= ~m; forced |= m
            n_box += 1
    for gg in ov["oneway_gates"]:
        dx, dz = gg["dir"]
        for mir in ([False, True] if gg.get("mirror") else [False]):
            m = _box(gg["box_x"], gg["box_z"], mir) & ~hole
            free |= m; forbidden &= ~m; barrier &= ~m; forced |= m
            ddx, ddz = (-dx, -dz) if mir else (dx, dz)       # 镜像下方向也取反
            gate_dx[m] = ddx; gate_dz[m] = ddz
            n_box += 1

    np.savez_compressed(os.path.join(DATA, "nav_grid.npz"),
                        free=free, highland=high, forbidden=forbidden, barrier=barrier,
                        wall_segs=wall_segs,
                        forced=forced, gate_dx=gate_dx, gate_dz=gate_dz,
                        fx0=FX0, fz0=FZ0, res=GRID_RES)

    # ★ 顺带导一份**简单二进制** nav_grid.bin 给 C++ 节点（npz 要在 C++ 里解 zip，不值得）
    #   格式：magic "NAVG" + int32 版本(1) + int32 ni,nj + float64 fx0,fz0,res
    #        + 5 个 uint8 掩膜（行优先 ni*nj：free/highland/forbidden/barrier/forced）
    #        + 2 个 int8 闸门方向（gate_dx/gate_dz，**带符号**，-1 表示允许往负方向走）
    #        + int32 nsegs + nsegs*4 float32（墙折线）
    with open(os.path.join(DATA, "nav_grid.bin"), "wb") as f:
        f.write(b"NAVG")
        f.write(np.int32(1).tobytes())
        f.write(np.int32([free.shape[0], free.shape[1]]).tobytes())
        f.write(np.float64([FX0, FZ0, GRID_RES]).tobytes())
        for arr in (free, high, forbidden, barrier, forced):
            f.write(np.ascontiguousarray(arr).astype(np.uint8).tobytes())
        f.write(np.ascontiguousarray(gate_dx).astype(np.int8).tobytes())
        f.write(np.ascontiguousarray(gate_dz).astype(np.int8).tobytes())
        f.write(np.int32(len(wall_segs)).tobytes())
        f.write(np.ascontiguousarray(wall_segs, np.float32).tobytes())
    print(f"  nav_grid.bin 已存（C++ 节点用） 墙折线 {len(wall_segs)} 段")
    ng = int(((gate_dx != 0) | (gate_dz != 0)).sum())
    print(f"栅格已存 nav_data/nav_grid.npz  {free.shape[0]}x{free.shape[1]}  "
          f"可通行格 {int(free.sum())} = {int(free.sum())*GRID_RES**2/9:.0f} m²"
          f"  禁行格 {int(forbidden.sum())}（{len(haz)} 个陷坑）"
          f"  等级硬边墙 {int(barrier.sum())} 格"
          f"  实测修正 {n_box} 框（强制可通行 {int(forced.sum())} 格、单向闸门 {ng} 格）"
          f"  补墙 +{n_wall} 格  墙折线 {len(wall_segs)} 段")

    # ---------- 出图 ----------
    img = Image.fromarray(np.where(PAS[..., None], (46, 84, 128), (26, 30, 38)).astype(np.uint8))
    img = Image.fromarray(np.where(HIGH[..., None], (58, 108, 158),
                                   np.asarray(img)).astype(np.uint8))
    img = Image.fromarray(np.where((PAS & ~hit)[..., None], (150, 92, 40),
                                   np.asarray(img)).astype(np.uint8))
    d = ImageDraw.Draw(img, "RGBA")
    for _, pts in tracks:
        xy = [to_local(*p) for p in pts]
        if len(xy) > 1:
            d.line(xy, fill=(255, 232, 120, 205), width=2, joint="curve")

    def fp(sz, bold=False):
        return ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-%s.ttc"
                                  % ("Bold" if bold else "Regular"), sz)

    # 敌方活动范围 + 镜像
    box = load_enemies()
    def rbox(p, q):
        return [min(p[0], q[0]), min(p[1], q[1]), max(p[0], q[0]), max(p[1], q[1])]
    for n in sorted(box):
        xs = [v[0] for v in box[n]]; zs = [v[1] for v in box[n]]; ar = box[n][0][2]
        for sx, sz, col, tag in [(1, 1, (255, 90, 90, 230), f"{n}号"),
                                 (-1, -1, (110, 200, 255, 175), f"{n}号镜像")]:
            if sx < 0:
                a_ = to_local(2*MCX - max(xs), 2*MCZ - max(zs))
                b_ = to_local(2*MCX - min(xs), 2*MCZ - min(zs))
            else:
                a_ = to_local(max(xs), max(zs)); b_ = to_local(min(xs), min(zs))
            lb = rbox(a_, b_)
            d.rectangle(lb, outline=col, width=2)
            d.text((lb[0]+5, lb[1]+4), tag, font=fp(12, True),
                   fill=col[:3], stroke_width=3, stroke_fill=(0, 0, 0))
            if ar > 0 and sx > 0:
                d.text((lb[0]+5, lb[1]+21), f"射程{ar/3:.0f}m", font=fp(10),
                       fill=col[:3], stroke_width=3, stroke_fill=(0, 0, 0))

    def mark(x, z, text, col):
        p = to_local(x, z)
        d.ellipse([p[0]-6, p[1]-6, p[0]+6, p[1]+6], fill=col, outline=(0, 0, 0, 180), width=2)
        d.text((p[0]+10, p[1]-8), text, font=fp(14, True), fill=col,
               stroke_width=3, stroke_fill=(0, 0, 0))
    mark(-4.93, 34.76, "7号 基地（敌方 5000HP）", (255, 110, 110))
    mark(-16.50, 9.17, "6号 前哨站（1500HP）", (255, 175, 70))
    mark(14.52, -34.46, "出生点 / 补给区", (110, 200, 255))

    # 陷坑：画成红叉 + 外扩后的禁行框
    for h in haz:
        bx, bz = h["box_x"], h["box_z"]
        a_ = to_local(bx[0] - HAZARD_MARGIN, bz[1] + HAZARD_MARGIN)
        b_ = to_local(bx[1] + HAZARD_MARGIN, bz[0] - HAZARD_MARGIN)
        lb = rbox(a_, b_)
        d.rectangle(lb, fill=(220, 40, 40, 90), outline=(255, 60, 60, 255), width=3)
        for s in (0, 1):
            d.line([(lb[0], lb[1] + s * (lb[3] - lb[1])),
                    (lb[2], lb[3] - s * (lb[3] - lb[1]))], fill=(255, 60, 60, 255), width=3)
        d.text((lb[0] - 2, lb[1] - 22), "陷坑·禁行", font=fp(14, True), fill=(255, 90, 90),
               stroke_width=3, stroke_fill=(0, 0, 0))

    sx0, sy0 = 18, BH - 24
    L3 = 3.0 * K
    d.rectangle([sx0-8, sy0-20, sx0+L3+64, sy0+16], fill=(0, 0, 0, 200))
    d.line([(sx0, sy0), (sx0+L3, sy0)], fill=(255, 255, 255), width=3)
    for t in (0, L3):
        d.line([(sx0+t, sy0-6), (sx0+t, sy0+6)], fill=(255, 255, 255), width=3)
    d.text((sx0+L3+8, sy0-9), "3 m", font=fp(13, True), fill=(255, 255, 255))

    LX, LY = BW - 350, BH - 184
    d.rectangle([LX, LY, BW-10, BH-10], fill=(0, 0, 0, 205), outline=(255, 255, 255, 60))
    for i, (c, t) in enumerate([((46, 84, 128), "可通行（轨迹够到）"),
                                ((58, 108, 158), "高地（可上）"),
                                ((150, 92, 40), "没走到（待补）"),
                                ((255, 232, 120), "驾驶轨迹"),
                                ((255, 90, 90), "敌方 2-7 号活动范围"),
                                ((110, 200, 255), "镜像 → 我方等效位置")]):
        y = LY + 12 + i * 24
        d.rectangle([LX+12, y, LX+34, y+15], fill=c, outline=(255, 255, 255, 80))
        d.text((LX+44, y-1), t, font=fp(13), fill=(235, 235, 235))
    d.text((LX+12, LY+12+6*24+4), f"轨迹够到 {cov*100:.0f}% 可通行区域",
           font=fp(12), fill=(200, 200, 200))

    d.text((14, 12), f"TDT Campus Game 场地通行图  ·  {len(tracks)} 轮录制  ·  覆盖率 {cov*100:.0f}%",
           font=fp(15, True), fill=(240, 240, 240), stroke_width=3, stroke_fill=(0, 0, 0))

    out = os.path.join(DATA, "nav_map_real.png")
    img.save(out)
    print(f"图已存 {out}  {img.size[0]}x{img.size[1]}")
    if args.show:
        subprocess.Popen(["eog", out], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


if __name__ == "__main__":
    main()
