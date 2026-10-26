#!/usr/bin/env python3
"""导航用的栅格地图 + A* 规划（纯算法，不依赖 ROS，可以离线自测）。

地图从 nav_data/nav_grid.npz 读（由 nav_map_report.py 生成），里面有这些层：
    free      地形可通行（颜色判据）
    highland  高地表面（可上，标记用）
    forbidden 陷坑等硬禁行（实测的，颜色在战术地图上分不出来）
    barrier   等级硬边/台阶墙（高代价，不硬禁）
    forced    实测坡道等强制可通行（免疫障碍外扩，见 nav_overrides.json）
    gate_*    实测单向闸门（如中央台阶只能下不能上），A* 里按方向过滤

坐标系：**Unity 世界 (x, z)**，场景单位（3 单位 = 1 m）。
    x 是场地的短轴（±22.5），z 是长轴（±42），红方在 +z、蓝方在 -z。
    这就是游戏 /target_speed_player_N 用的那个系，所以规划出来的点可以直接用。

自测（不需要游戏）：
    python3 nav_map.py            # 画一条 出生点 → 前哨站 的路径看看
"""
import heapq
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
NPZ = os.path.abspath(os.path.join(HERE, "..", "..", "nav_data", "nav_grid.npz"))

# 机器人半宽（场景单位）。规划时把障碍外扩这么多，机器人当质点处理。
ROBOT_RADIUS = 1.2
# 台阶墙（等级硬边）：**硬禁**（walkable 里直接抠掉，见 __init__）。
# 走过弯路：先是"每格收费"（把用户沿墙的合法走廊罚死）→ 改"每穿一次收费"
# （A* 学会付 50 穿墙抄近路，把 z=-32 的真墙也穿了）→ 用户拍板：直接不准穿。
# 墙格现在进不了 walkable，line_clear 也拒（含对角角格）。
# 贴墙惩罚（代价地图"膨胀"思想，任务2 那种连续代价）：
# 离墙/障碍 2.0 单位以内，越近代价越高（线性衰减）。
# ★ 要"温和"：它是等长路线间的偏好微调，不能重到把"贴着墙但明显更近的
#   合法走廊"顶掉（实测：4.0 时西侧近路被顶成 106 等效、和东边绕远打平）。
PROX_R = 2.0
PROX_COST = 1.5

SQRT2 = math.sqrt(2.0)
NEIGHBORS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)]


class FieldMap:
    def __init__(self, npz_path=NPZ):
        d = np.load(npz_path)
        self.free = d["free"].astype(bool)
        self.highland = d["highland"].astype(bool)
        self.forbidden = d["forbidden"].astype(bool) if "forbidden" in d else np.zeros_like(self.free)
        # 等级硬边（台阶墙）：不硬禁，只加高代价，让规划器主动离它远
        self.barrier = d["barrier"].astype(bool) if "barrier" in d else np.zeros_like(self.free)
        # ★ 实测修正：强制可通行（坡等，免疫障碍外扩）+ 单向闸门（允许方向）
        self.forced = d["forced"].astype(bool) if "forced" in d else np.zeros_like(self.free)
        # ★ 墙 = 几何折线（谁横穿谁撞墙）。格子模型分不开"墙脸"和"墙根的路"，
        #   方向法线在拐角又总指错（实测反复穿墙），折线一次相交测试就定了。
        self.wall_segs = d["wall_segs"].astype(np.float32) if "wall_segs" in d \
            else np.zeros((0, 4), np.float32)
        self.gate_dx = d["gate_dx"].astype(np.int8) if "gate_dx" in d else np.zeros(self.free.shape, np.int8)
        self.gate_dz = d["gate_dz"].astype(np.int8) if "gate_dz" in d else np.zeros(self.free.shape, np.int8)
        self.fx0 = float(d["fx0"])
        self.fz0 = float(d["fz0"])
        self.res = float(d["res"])
        self.ni, self.nj = self.free.shape
        self._wb = {}                                 # 墙折线的 2 单位空间索引
        for _idx in range(len(self.wall_segs)):
            _s = self.wall_segs[_idx]
            _i0 = int((min(_s[0], _s[2]) - self.fx0) // 2)
            _i1 = int((max(_s[0], _s[2]) - self.fx0) // 2)
            _j0 = int((min(_s[1], _s[3]) - self.fz0) // 2)
            _j1 = int((max(_s[1], _s[3]) - self.fz0) // 2)
            for _bi in range(_i0, _i1 + 1):
                for _bj in range(_j0, _j1 + 1):
                    self._wb.setdefault((_bi, _bj), []).append(_idx)
        # 障碍外扩：可通行的格子，周围 ROBOT_RADIUS 内有禁行/不可通行 → 也不能走
        self.walkable = self._inflate(self.free & ~self.forbidden, ROBOT_RADIUS) \
            | (self.forced & ~self.forbidden)
        # ★ 贴墙惩罚（代价地图"膨胀"思想）：每格到最近障碍/墙的距离 → 线性惩罚。
        #   A*、拉直、自检三处共用（step_extra），等长路线自动挑走廊中间。
        obst = (self.barrier | ~self.walkable).astype(np.uint8)
        dist = cv2.distanceTransform(1 - obst, cv2.DIST_L2, 3)   # 单位 = 格
        self._prox = (np.clip((PROX_R - dist) / PROX_R, 0.0, 1.0) * PROX_COST).astype(float)

    def _inflate(self, free, radius):
        r = int(math.ceil(radius / self.res))
        if r <= 0:
            return free.copy()
        bad = ~free
        grown = bad.copy()
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if di * di + dj * dj > r * r:
                    continue
                grown |= np.roll(np.roll(bad, di, 0), dj, 1)
        return free & ~grown

    # ---- 坐标换算 ----
    def cell_of(self, x, z):
        return (int(round((x - self.fx0) / self.res)),
                int(round((z - self.fz0) / self.res)))

    def world_of(self, i, j):
        return (self.fx0 + i * self.res, self.fz0 + j * self.res)

    def in_bounds(self, i, j):
        return 0 <= i < self.ni and 0 <= j < self.nj

    def is_walkable(self, x, z):
        i, j = self.cell_of(x, z)
        return self.in_bounds(i, j) and self.walkable[i, j]

    def step_extra(self, cur, nb, length):
        """一步 (cur→nb) 的附加代价：现在只剩贴墙惩罚（墙本身已硬禁，走不进去）。

        ★ A* 扩展、直线拉直（_string_pull/_seg_cost）、自检 三处共用，
          口径不一致拉直就会"改道"（实测教训）。
        """
        return float(self._prox[nb]) * length

    def wall_step_blocked(self, cur, nb):
        """这一步的线段是否【横穿】墙折线。返回 True=禁（硬禁，穿墙不允许）。

        ★ 墙是几何折线，判据就一句"横穿吗"：
          · 沿墙走（贴墙、走墙根的路）→ 不横穿 → 放行；
          · 穿越墙 → 横穿 → 禁死。
          严格的"内部横穿"（两侧符号都严格相反）才算 —— 端点接触、共线重叠
          （比如路径正好压在折线上走）都不算穿越，留给沿墙的路。
        """
        if not len(self.wall_segs):
            return False
        # ★ 闸门框内交给闸门规则管：台阶自带的"台阶线"也会被描进折线，
        #   不豁免的话下台阶会被自己的台阶线挡住（实测：台阶中段→南 无路）。
        if (self.gate_dx[cur] or self.gate_dz[cur]) or \
           (self.gate_dx[nb] or self.gate_dz[nb]):
            return False
        x1, z1 = self.world_of(*cur)
        x2, z2 = self.world_of(*nb)
        i0 = int((min(x1, x2) - self.fx0) // 2)
        i1 = int((max(x1, x2) - self.fx0) // 2)
        j0 = int((min(z1, z2) - self.fz0) // 2)
        j1 = int((max(z1, z2) - self.fz0) // 2)
        seen = set()
        for bi in range(i0, i1 + 1):
            for bj in range(j0, j1 + 1):
                for idx in self._wb.get((bi, bj), ()):
                    if idx in seen:
                        continue
                    seen.add(idx)
                    s = self.wall_segs[idx]
                    if self._proper_cross(x1, z1, x2, z2,
                                          float(s[0]), float(s[1]),
                                          float(s[2]), float(s[3])):
                        return True
        return False

    @staticmethod
    def _proper_cross(ax, az, bx, bz, cx, cz, dx, dz):
        """线段 ab 与 cd 是否严格横穿（端点接触/共线不算）。"""
        def cr(ox, oz, px, pz, qx, qz):
            return (px - ox) * (qz - oz) - (pz - oz) * (qx - ox)
        d1 = cr(cx, cz, dx, dz, ax, az)
        d2 = cr(cx, cz, dx, dz, bx, bz)
        d3 = cr(ax, az, bx, bz, cx, cz)
        d4 = cr(ax, az, bx, bz, dx, dz)
        return (d1 * d2 < 0) and (d3 * d4 < 0)

    def gate_step_blocked(self, pi, pj, i, j):
        """从格 (pi,pj) 走一步到 (i,j)，是否违反单向闸门。返回 True=非法。

        ★ A* 扩展、直线拉直（line_clear）、自检 三处必须用同一套判据 ——
          各写一份会互相漏（实测：拉直抄出过反向穿闸门的直线段）。
        规则：
          · 这一步"涉及"闸门格就算 —— 进闸门格、或从闸门格出来（回退查上一格）；
          · 对角步会跳过角上的两格，也要按同一步方向查（否则直线从格子角上钻过去）。
        """
        di, dj = i - pi, j - pj
        chk = [(i, j)]
        if di and dj:
            chk += [(i, pj), (pi, j)]
        for ci, cj in chk:
            gdx, gdz = self.gate_dx[ci, cj], self.gate_dz[ci, cj]
            if not (gdx or gdz):
                gdx, gdz = self.gate_dx[pi, pj], self.gate_dz[pi, pj]
            if (gdx or gdz) and di * int(gdx) + dj * int(gdz) < 0:
                return True
        return False

    def block_around(self, x, z, radius=2.0):
        """把某个位置周围标成走不通 —— 运行时学到的障碍（撞墙/掉坑都走这条）。

        地图只记录了"轨迹走过 = 能走"，没有"走过但旁边有台阶"这层信息，
        所以第一次撞上台阶墙是必然的。撞过之后标死，重新规划就绕开了。
        """
        i, j = self.cell_of(x, z)
        r = int(math.ceil(radius / self.res))
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if di * di + dj * dj > r * r:
                    continue
                a, b = i + di, j + dj
                if self.in_bounds(a, b):
                    self.walkable[a, b] = False

    def nearest_walkable(self, i, j, max_r=40):
        """起点/终点万一落在不可走的格子上，就近找一个能走的。"""
        if self.in_bounds(i, j) and self.walkable[i, j]:
            return (i, j)
        for r in range(1, max_r):
            for di in range(-r, r + 1):
                for dj in (-r, r):
                    for a, b in ((i + di, j + dj), (i + dj, j + di)):
                        if self.in_bounds(a, b) and self.walkable[a, b]:
                            return (a, b)
        return None

    # ---- A* ----
    def plan(self, start, goal, turn_penalty=0.0):
        """start/goal 是 Unity (x, z)。返回 [(x,z), ...] 或 None。

        代价 = 几何长度 + 穿墙（进入时一次）+ 贴墙惩罚（step_extra）；
        通行性 = walkable（含外扩/实测坡）+ 单向闸门。
        """
        s = self.nearest_walkable(*self.cell_of(*start))
        g = self.nearest_walkable(*self.cell_of(*goal))
        if s is None or g is None:
            return None
        if s == g:
            return [self.world_of(*g)]

        openq = [(0.0, s, None)]
        came, gcost = {}, {s: 0.0}
        closed = set()
        while openq:
            f, cur, _ = heapq.heappop(openq)
            if cur in closed:
                continue
            closed.add(cur)
            if cur == g:
                break
            ci, cj = cur
            for di, dj, w in NEIGHBORS:
                nb = (ci + di, cj + dj)
                if not self.in_bounds(*nb) or not self.walkable[nb] or nb in closed:
                    continue
                # ★ 穿墙硬禁（带方向）+ 单向闸门：判据和直线拉直、自检共用
                if self.wall_step_blocked(cur, nb):
                    continue
                if self.gate_step_blocked(ci, cj, nb[0], nb[1]):
                    continue
                step = w * self.res
                step += self.step_extra(cur, nb, w * self.res)   # 贴墙惩罚（墙本身硬禁）
                # 拐弯惩罚：让路径少打折线，pure pursuit 跟起来稳
                if turn_penalty > 0 and cur in came:
                    pi, pj = came[cur]
                    if (ci - pi, cj - pj) != (di, dj):
                        step += turn_penalty
                ng = gcost[cur] + step
                if ng < gcost.get(nb, 1e18):
                    gcost[nb] = ng
                    came[nb] = cur
                    hh = math.hypot(nb[0] - g[0], nb[1] - g[1]) * self.res
                    heapq.heappush(openq, (ng + hh, nb, cur))
        if g not in came and g != s:
            return None
        # 回溯
        path = [g]
        while path[-1] != s:
            path.append(came[path[-1]])
        path.reverse()
        path = self._string_pull(path)
        return [self.world_of(*c) for c in path]

    def _true_wall_blocked(self, x1, z1, x2, z2):
        """真实线段 (x1,z1)-(x2,z2) 是否横穿墙折线（精确相交；闸门框内的交点豁免）。

        ★ 不能用 wall_step_blocked 代替：那个用的是**格中心之间**的线段，和真实
          采样路径错位（最多差半格），采样稍稀就会漏掉细墙 —— 实测第 6 段被
          string_pull 抄出一段穿墙直线，(12,-1) 处压墙 1 步。
        """
        if not len(self.wall_segs):
            return False
        seen = set()
        for px_, pz_ in ((x1, z1), (x2, z2)):     # 子线段短：交点必在端点 ±1 桶内
            bi = int((px_ - self.fx0) // 2)
            bj = int((pz_ - self.fz0) // 2)
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    key = (bi + di, bj + dj)
                    if key in seen:
                        continue
                    seen.add(key)
                    for idx in self._wb.get(key, ()):
                        s = self.wall_segs[idx]
                        if self._proper_cross(x1, z1, x2, z2, float(s[0]), float(s[1]),
                                              float(s[2]), float(s[3])):
                            ix, iz = self._intersect(x1, z1, x2, z2, float(s[0]), float(s[1]),
                                                     float(s[2]), float(s[3]))
                            ci, cj = self.cell_of(ix, iz)
                            if self.in_bounds(ci, cj) and (self.gate_dx[ci, cj]
                                                           or self.gate_dz[ci, cj]):
                                continue              # 出口线/台阶内的穿越 = 放行
                            return True
        return False

    @staticmethod
    def _intersect(ax, az, bx, bz, cx, cz, dx, dz):
        """两线段（已知严格横穿）的交点。"""
        den = (bx - ax) * (dz - cz) - (bz - az) * (dx - cx)
        if abs(den) < 1e-12:
            return ((ax + bx + cx + dx) / 4.0, (az + bz + cz + dz) / 4.0)
        t = ((cx - ax) * (dz - cz) - (cz - az) * (dx - cx)) / den
        return (ax + t * (bx - ax), az + t * (bz - az))

    def line_clear(self, a, b):
        """a→b 的直线是否全程可走（按 0.5 格步长采样，机器人当质点）。

        ★ 单向闸门也要遵守：采样线穿过闸门格时，前进方向不能与闸门方向相反
        （否则拉直会"抄"出一条反向穿台阶的直线段）。
        ★ 墙用【真实子线段精确相交】判，不用格中心段（会错位漏检，实测踩过）。
        """
        ax, az = self.world_of(*a)
        bx, bz = self.world_of(*b)
        n = int(math.hypot(bx - ax, bz - az) / 0.5) + 1
        pi = pj = None
        pw = None
        for k in range(n + 1):
            r = k / n
            wx, wz = ax + r * (bx - ax), az + r * (bz - az)
            i, j = self.cell_of(wx, wz)
            if not self.in_bounds(i, j) or not self.walkable[i, j]:
                return False
            if pi is not None:
                # ★ 对角步会跳过角上的两格：角格是禁行（陷坑）时，直线等于
                #   从禁行框的角上切过去（实测：擦西陷坑的角）—— 拒掉。
                if i != pi and j != pj:
                    for ci, cj in ((i, pj), (pi, j)):
                        if self.in_bounds(ci, cj) and self.forbidden[ci, cj]:
                            return False
                if self._true_wall_blocked(pw[0], pw[1], wx, wz):
                    return False
                # ★ 单向闸门：和 A*、自检共用同一个判据
                if self.gate_step_blocked(pi, pj, i, j):
                    return False
            pi, pj = i, j
            pw = (wx, wz)
        return True

    def _seg_cost(self, a, b):
        """直线段 a→b（格坐标）的等效代价 —— 尺子必须和 A* 一模一样：
        几何长度 + 贴墙惩罚（按长度；墙已硬禁，线穿墙的情况由 line_clear 直接拒）。
        """
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.5))
        cost = 0.0
        pi = pj = None
        for k in range(n + 1):
            r = k / n
            i = int(round(a[0] + r * (b[0] - a[0])))
            j = int(round(a[1] + r * (b[1] - a[1])))
            if pi is not None and (i, j) != (pi, pj):
                ln = math.hypot(i - pi, j - pj) * self.res
                cost += ln + self.step_extra((pi, pj), (i, j), ln)
            pi, pj = i, j
        return cost

    def _string_pull(self, cells):
        """拉直：能直线到达就跳过中间点。

        ★ 为什么必须做：8 连通栅格出来的路径全是 45° 锯齿，纯追踪跟着它走会
        不停画龙（等效于一直在转弯），弯道减速也会被误触发到全程生效。
        拉直之后路径是「直段 + 真拐角」，跟踪和减速才都合理。

        ★ 两个约束必须和 A* 用同一把尺子，否则拉直会"改道"而不是"简化"：
          · line_clear —— 可走 + 单向闸门；
          · 代价不许变高 —— 直线的等效代价 ≤ 被替换子路径的代价。
            （实测：不带这个约束，会把 A* 花代价绕开的墙抄成穿墙直线，车撞墙。）
        """
        if len(cells) <= 2:
            return cells
        # 原子路径的累计代价（与 A* 同记账：step_extra = 长度 + 穿墙 + 贴墙）
        C = [0.0]
        for k in range(1, len(cells)):
            ln = math.hypot(cells[k][0] - cells[k - 1][0],
                            cells[k][1] - cells[k - 1][1]) * self.res
            C.append(C[-1] + ln + self.step_extra(cells[k - 1], cells[k], ln))
        out = [cells[0]]
        i = 0
        while i < len(cells) - 1:
            j = len(cells) - 1
            while j > i + 1 and not (
                    self.line_clear(cells[i], cells[j])
                    and self._seg_cost(cells[i], cells[j]) <= C[j] - C[i] + 1e-6):
                j -= 1
            out.append(cells[j])
            i = j
        return out


def _self_test():
    m = FieldMap()
    print(f"地图 {m.ni}x{m.nj} 格，分辨率 {m.res} 单位/格")
    print(f"  可通行 {int(m.free.sum())} 格 = {int(m.free.sum())*m.res**2/9:.0f} m²")
    print(f"  禁行   {int(m.forbidden.sum())} 格")
    print(f"  外扩后能走的 {int(m.walkable.sum())} 格（机器人半径 {ROBOT_RADIUS} 单位）")

    cases = [((14.52, -34.46), (-16.50, 9.17), "出生点 → 6号前哨站"),
             ((14.52, -34.46), (-4.93, 30.0), "出生点 → 敌方基地外围"),
             ((-16.50, 9.17), (5.0, -30.0), "前哨站 → 回家"),
             ((-25.0, 5.0), (16.0, 5.0), "西边缘 → 东边缘（要绕开两个陷坑）"),
             ((12.8, 3.4), (-5.0, 0.0), "低层东侧 → 高地（该走①坡上）"),
             ((-5.0, 0.0), (-5.0, -20.0), "高地 → 南侧低层（该走南台阶下）"),
             ((-5.0, -20.0), (-5.0, 0.0), "南侧低层 → 高地（台阶不能上，该绕①坡）"),
             ((14.60, -34.94), (-5.0, -20.0), "出生点 → 南侧低层（该绕 z=-32 的墙）")]
    ngate = m.gate_dx | m.gate_dz

    def wall_hits(path):
        """路径里"穿墙"（硬禁方向）的步数 —— 规划器正确时应恒为 0。"""
        hits = 0
        pi = pj = None
        for k in range(len(path) - 1):
            (ax, az), (bx, bz) = path[k], path[k + 1]
            n = max(1, int(math.hypot(bx - ax, bz - az) / 0.25))
            for s in range(n + 1):
                r = s / n
                i, j = m.cell_of(ax + r * (bx - ax), az + r * (bz - az))
                if pi is not None and (i, j) != (pi, pj) \
                        and m.wall_step_blocked((pi, pj), (i, j)):
                    hits += 1
                pi, pj = i, j
        return hits

    def gate_hits(path):
        """返回 (进入闸门框次数, 逆向穿越步数)。

        按线段 0.25 步长采样（只看拐点会漏掉中段穿越）；逆向判据和 A*/拉直共用
        gate_step_blocked —— 规划器正常情况下应该一次都不产生。
        """
        entries = 0
        illegal = 0
        prev_in = False
        pi = pj = None
        for k in range(len(path) - 1):
            (ax, az), (bx, bz) = path[k], path[k + 1]
            n = max(1, int(math.hypot(bx - ax, bz - az) / 0.25))
            for s in range(n + 1):
                r = s / n
                i, j = m.cell_of(ax + r * (bx - ax), az + r * (bz - az))
                cur_in = m.in_bounds(i, j) and bool(ngate[i, j])
                if cur_in and not prev_in:
                    entries += 1
                if pi is not None and m.gate_step_blocked(pi, pj, i, j):
                    illegal += 1
                prev_in = cur_in
                pi, pj = i, j
        return entries, illegal

    for a, b, name in cases:
        # 和 nav_node 同口径（穿墙 + 贴墙 + 拐弯惩罚，都内置了）
        p = m.plan(a, b, turn_penalty=0.4)
        if p is None:
            print(f"  ✗ {name}: 规划失败")
            continue
        L = sum(math.hypot(p[i][0]-p[i-1][0], p[i][1]-p[i-1][1]) for i in range(1, len(p)))
        entries, illegal = gate_hits(p)
        wh = wall_hits(p)
        tag = f"（穿墙格 {wh} 次"
        if ngate.any():
            tag += f"、进闸门框 {entries} 次、逆向 {illegal} 步"
            if illegal:
                tag += "  ★ 单向规则漏了！"
        tag += "）"
        print(f"  ✓ {name}: {len(p)} 个点，路程 {L:.1f} 单位 = {L/3:.1f} m" + tag)


if __name__ == "__main__":
    _self_test()
