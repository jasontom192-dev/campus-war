// nav_grid.cpp —— Python 版 nav_map.py 的逐项移植。改动请两边一起改（tools/nav_map.py）。
#include "nav_grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <queue>
#include <set>

#include <opencv2/opencv.hpp>

namespace {
constexpr double kSqrt2 = 1.4142135623730951;
struct Move { int di, dj; double w; };
const Move kMoves[8] = {{1, 0, 1.0},  {-1, 0, 1.0}, {0, 1, 1.0},   {0, -1, 1.0},
                        {1, 1, kSqrt2}, {1, -1, kSqrt2}, {-1, 1, kSqrt2}, {-1, -1, kSqrt2}};
}  // namespace

// ============================================================================
// 导航规划器实现说明（C++ 实跑版）
//
// 一次 plan() 的完整流程：
//   1. cell_of() 把世界坐标 (x,z) 换成格子下标 (i,j)
//   2. nearest_walkable() 保证起终点落在可走格
//   3. A*：8 邻接、优先队列、gcost/came/closed
//        代价 = 几何长度 + 贴墙(step_extra) + 拐弯(turn_penalty)
//        禁走 = 不可走格 / 横穿墙折线 / 逆单向闸门
//   4. 从终点沿 came 回溯到起点
//   5. string_pull() 拉直：能直线走且等效代价不高于原段
//   6. world_of() 把格中心换回世界坐标
//
// ★ 三个判据（A*、拉直、上层自检）必须共用：
//    step_extra / gate_step_blocked / wall_step_blocked(true_wall_blocked)
//   改任何一处都要同步另一处，否则会出现"拉直把绕墙路抄成穿墙直线"。
// ============================================================================

bool NavGrid::load(const std::string & path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  char magic[4];
  f.read(magic, 4);
  if (!f || std::string(magic, 4) != "NAVG") return false;
  int32_t ver = 0, a = 0, b = 0;
  f.read(reinterpret_cast<char *>(&ver), 4);
  f.read(reinterpret_cast<char *>(&a), 4);
  f.read(reinterpret_cast<char *>(&b), 4);
  double xyz[3];
  f.read(reinterpret_cast<char *>(xyz), 24);
  if (!f || ver != 1) return false;
  ni = a; nj = b; fx0 = xyz[0]; fz0 = xyz[1]; res = xyz[2];
  const size_t n = static_cast<size_t>(ni) * nj;

  auto read8 = [&](std::vector<uint8_t> & v, size_t count) {
    v.resize(count);
    f.read(reinterpret_cast<char *>(v.data()), count);
  };
  read8(free_, n);
  read8(highland_, n);
  read8(forbidden_, n);
  read8(barrier_, n);
  read8(forced_, n);
  gate_dx_.resize(n);
  f.read(reinterpret_cast<char *>(gate_dx_.data()), n);
  gate_dz_.resize(n);
  f.read(reinterpret_cast<char *>(gate_dz_.data()), n);
  int32_t nseg = 0;
  f.read(reinterpret_cast<char *>(&nseg), 4);
  wall_segs_.resize(static_cast<size_t>(nseg) * 4);
  f.read(reinterpret_cast<char *>(wall_segs_.data()), wall_segs_.size() * 4);
  if (!f) return false;

  // 文件布局与 nav_map_report.py 的写入顺序一一对应：
  //   "NAVG" + int32 ver + int32 ni + int32 nj + double fx0,fz0,res
  //   + free/highland/forbidden/barrier/forced（各 ni*nj 个 uint8）
  //   + gate_dx/gate_dz（各 ni*nj 个 int8）
  //   + int32 nseg + nseg*4 个 float32（墙折线）
  // 当前地图是 46x85、res=1.0、8237 段墙。

  // ---- 派生层 1：walkable_ ----
  // walkable = inflate(free & ~forbidden, R) | (forced & ~forbidden)
  // ★ 和 Python 的 numpy.roll 一样带环绕（贴边行为要一致）。
  std::vector<uint8_t> src(n), bad(n);
  for (size_t k = 0; k < n; ++k) {
    src[k] = (free_[k] && !forbidden_[k]) ? 1 : 0;
    bad[k] = src[k] ? 0 : 1;
  }
  const int r = static_cast<int>(std::ceil(kRobotRadius / res));
  std::vector<uint8_t> grown = bad;
  for (int di = -r; di <= r; ++di) {
    for (int dj = -r; dj <= r; ++dj) {
      if (di * di + dj * dj > r * r) continue;
      for (int i = 0; i < ni; ++i) {
        const int si = ((i - di) % ni + ni) % ni;
        for (int j = 0; j < nj; ++j) {
          const int sj = ((j - dj) % nj + nj) % nj;
          if (bad[static_cast<size_t>(si) * nj + sj]) grown[static_cast<size_t>(i) * nj + j] = 1;
        }
      }
    }
  }
  walkable_.assign(n, 0);
  for (size_t k = 0; k < n; ++k)
    // 和 Python 同语义：walkable = (src & ~grown) | (forced & ~forbidden)
    walkable_[k] = ((src[k] && !grown[k]) || (forced_[k] && !forbidden_[k])) ? 1 : 0;

  // ---- 派生层 2：prox_ ----
  // obst = barrier | !walkable；离障碍/墙越近代价越高。
  // 贴墙惩罚：cv2.distanceTransform(1-obst, L2, 3) 同款 ----
  cv::Mat obst(ni, nj, CV_8UC1);
  for (int i = 0; i < ni; ++i)
    for (int j = 0; j < nj; ++j)
      obst.at<uint8_t>(i, j) = (barrier_[idx(i, j)] || !walkable_[idx(i, j)]) ? 1 : 0;
  cv::Mat srcm, dist;
  // ★ 2026-10-05 修（车蹭前哨站的根因）：obst 是 0/1，原来 bitwise_not 出来是
  //   255/254 —— 【全是非零】→ distanceTransform 认为整幅图没有障碍 → 整个
  //   prox_ 层全 0，C++ 规划器从移植以来就没付过贴墙代价（会穿前哨站、贴墙
  //   抄近路；Python 侧 1-obst 是对的，所以两边路径一直不一致）。
  //   改成"障碍=0、自由=255"，与 Python 的 1-obst 语义逐位一致。
  cv::compare(obst, 0, srcm, cv::CMP_EQ);
  cv::distanceTransform(srcm, dist, cv::DIST_L2, 3);
  prox_.resize(n);
  for (int i = 0; i < ni; ++i)
    for (int j = 0; j < nj; ++j) {
      const double d = dist.at<float>(i, j);
      const double c = std::clamp((kProxR - d) / kProxR, 0.0, 1.0) * kProxCost;
      prox_[idx(i, j)] = static_cast<float>(c);
    }

  // ---- 墙折线空间桶（2 单位一格）----
  // 墙有 8237 段，A* 每一步都遍历全部会太慢。这里按 2x2 单位分桶，
  // 查询时只检查线段包围盒覆盖到的桶。键 = (bi+512)<<32 | (bj+512)。
  const size_t nseg2 = wall_segs_.size() / 4;
  for (size_t k = 0; k < nseg2; ++k) {
    const float * s = &wall_segs_[k * 4];
    const int i0 = static_cast<int>(std::floor((std::min(s[0], s[2]) - fx0) / 2.0));
    const int i1 = static_cast<int>(std::floor((std::max(s[0], s[2]) - fx0) / 2.0));
    const int j0 = static_cast<int>(std::floor((std::min(s[1], s[3]) - fz0) / 2.0));
    const int j1 = static_cast<int>(std::floor((std::max(s[1], s[3]) - fz0) / 2.0));
    for (int bi = i0; bi <= i1; ++bi)
      for (int bj = j0; bj <= j1; ++bj)
        wb_[(static_cast<uint64_t>(bi + 512) << 32) | static_cast<uint32_t>(bj + 512)]
            .push_back(static_cast<int>(k));
  }
  return true;
}

// 一步 (ci,cj)→(i,j) 的附加代价：贴墙惩罚。
// 只看目标格的 prox_，再乘这一步的长度；前两个参数为兼容 Python 签名保留。
double NavGrid::step_extra(int, int, int i, int j, double length) const {
  return static_cast<double>(prox_[idx(i, j)]) * length;
}

// 单向闸门判据：这一步是否逆着允许方向。
//   直走：查目标格 (i,j)
//   对角：还要查两个角格 (i,pj)、(pi,j)
//   当前格无闸门标记：回退查上一格 (pi,pj)
// 方向判据：di*gx + dj*gz < 0 → 逆着走 → 禁。
bool NavGrid::gate_step_blocked(int pi, int pj, int i, int j) const {
  const int di = i - pi, dj = j - pj;
  int chk[3][2] = {{i, j}, {0, 0}, {0, 0}};
  int nchk = 1;
  if (di && dj) {
    chk[1][0] = i; chk[1][1] = pj;
    chk[2][0] = pi; chk[2][1] = j;
    nchk = 3;
  }
  for (int k = 0; k < nchk; ++k) {
    const int ci = chk[k][0], cj = chk[k][1];
    int gx = (in_bounds(ci, cj)) ? gate_dx_[idx(ci, cj)] : 0;
    int gz = (in_bounds(ci, cj)) ? gate_dz_[idx(ci, cj)] : 0;
    if (!(gx || gz)) {
      gx = in_bounds(pi, pj) ? gate_dx_[idx(pi, pj)] : 0;
      gz = in_bounds(pi, pj) ? gate_dz_[idx(pi, pj)] : 0;
    }
    if ((gx || gz) && di * gx + dj * gz < 0) return true;
  }
  return false;
}

// 严格横穿：ab 的端点分别在 cd 两侧，且 cd 的端点分别在 ab 两侧。
// 端点接触、共线重叠都不算横穿 —— 这样沿墙走/压在墙上的合法路径才不会被误禁。
bool NavGrid::proper_cross(double ax, double az, double bx, double bz,
                           double cx, double cz, double dx, double dz) {
  auto cr = [](double ox, double oz, double px, double pz, double qx, double qz) {
    return (px - ox) * (qz - oz) - (pz - oz) * (qx - ox);
  };
  const double d1 = cr(cx, cz, dx, dz, ax, az);
  const double d2 = cr(cx, cz, dx, dz, bx, bz);
  const double d3 = cr(ax, az, bx, bz, cx, cz);
  const double d4 = cr(ax, az, bx, bz, dx, dz);
  return (d1 * d2 < 0) && (d3 * d4 < 0);
}

// A* 用：这一步（格中心 → 格中心）是否横穿墙折线。
//   · 闸门格内不判墙：台阶自己的"台阶线"也被描进了折线，必须豁免。
//   · 只查线段包围盒覆盖到的 2 单位桶，不遍历全部 8237 段墙。
bool NavGrid::wall_step_blocked(int ci, int cj, int i, int j) const {
  if (wall_segs_.empty()) return false;
  // 闸门框内交给闸门规则（台阶自己的"台阶线"不算墙）
  if ((gate_dx_[idx(ci, cj)] || gate_dz_[idx(ci, cj)]) ||
      (gate_dx_[idx(i, j)] || gate_dz_[idx(i, j)]))
    return false;
  const auto [x1, z1] = world_of(ci, cj);
  const auto [x2, z2] = world_of(i, j);
  const int i0 = static_cast<int>(std::floor((std::min(x1, x2) - fx0) / 2.0));
  const int i1 = static_cast<int>(std::floor((std::max(x1, x2) - fx0) / 2.0));
  const int j0 = static_cast<int>(std::floor((std::min(z1, z2) - fz0) / 2.0));
  const int j1 = static_cast<int>(std::floor((std::max(z1, z2) - fz0) / 2.0));
  for (int bi = i0; bi <= i1; ++bi)
    for (int bj = j0; bj <= j1; ++bj) {
      auto it = wb_.find((static_cast<uint64_t>(bi + 512) << 32) |
                         static_cast<uint32_t>(bj + 512));
      if (it == wb_.end()) continue;
      for (int si : it->second) {
        const float * s = &wall_segs_[static_cast<size_t>(si) * 4];
        if (proper_cross(x1, z1, x2, z2, s[0], s[1], s[2], s[3])) return true;
      }
    }
  return false;
}

// 拉直用：真实采样点 (x1,z1)→(x2,z2) 是否横穿墙折线。
// 不能用 wall_step_blocked 代替：格中心段和真实路径最多差半格，细墙会漏。
//   · 精确求交点；
//   · 交点落在闸门格内 → 豁免（出口线/台阶线）；
//   · 其余 → 横穿，硬禁。
// seen 防止相邻桶重复检查同一段墙。
bool NavGrid::true_wall_blocked(double x1, double z1, double x2, double z2) const {
  if (wall_segs_.empty()) return false;
  // 真实子线段（0.5 单位）与墙折线精确相交；交点落在闸门格内 → 豁免（出口线）
  std::set<uint64_t> seen;
  const double pts[2][2] = {{x1, z1}, {x2, z2}};
  for (const auto & p : pts) {
    const int bi = static_cast<int>(std::floor((p[0] - fx0) / 2.0));
    const int bj = static_cast<int>(std::floor((p[1] - fz0) / 2.0));
    for (int di = -1; di <= 1; ++di)
      for (int dj = -1; dj <= 1; ++dj) {
        const uint64_t key = (static_cast<uint64_t>(bi + di + 512) << 32) |
                             static_cast<uint32_t>(bj + dj + 512);
        if (!seen.insert(key).second) continue;
        auto it = wb_.find(key);
        if (it == wb_.end()) continue;
        for (int si : it->second) {
          const float * s = &wall_segs_[static_cast<size_t>(si) * 4];
          if (proper_cross(x1, z1, x2, z2, s[0], s[1], s[2], s[3])) {
            const auto ip = intersect(x1, z1, x2, z2, s[0], s[1], s[2], s[3]);
            const auto [ci, cj] = cell_of(ip.first, ip.second);
            if (in_bounds(ci, cj) && (gate_dx_[idx(ci, cj)] || gate_dz_[idx(ci, cj)]))
              continue;                       // 出口线内的穿越 = 放行
            return true;
          }
        }
      }
  }
  return false;
}

// 两线段（已知严格横穿）的交点；平行/共线时退化为四点中心。
std::pair<double, double> NavGrid::intersect(double ax, double az, double bx, double bz,
                                             double cx, double cz, double dx, double dz) {
  const double den = (bx - ax) * (dz - cz) - (bz - az) * (dx - cx);
  if (std::abs(den) < 1e-12)
    return {(ax + bx + cx + dx) / 4.0, (az + bz + cz + dz) / 4.0};
  const double t = ((cx - ax) * (dz - cz) - (cz - az) * (dx - cx)) / den;
  return {ax + t * (bx - ax), az + t * (bz - az)};
}

// 拉直采样：a→b 这条直线能不能全程可走。
// 以 0.5 单位为步长：
//   1. 每个采样点必须在 walkable_ 内
//   2. 对角跨格时，两个角格不能是 forbidden_（防止从陷坑角切过去）
//   3. 真实子线段不能横穿墙（true_wall_blocked）
//   4. 步进方向不能逆单向闸门
bool NavGrid::line_clear(std::pair<int, int> a, std::pair<int, int> b) const {
  const auto aw = world_of(a.first, a.second);
  const auto bw = world_of(b.first, b.second);
  const int n = static_cast<int>(std::hypot(bw.first - aw.first, bw.second - aw.second) / 0.5) + 1;
  int pi = 0, pj = 0;
  double pwx = 0, pwz = 0;
  bool has_prev = false;
  for (int k = 0; k <= n; ++k) {
    const double rr = static_cast<double>(k) / n;
    const double wx = aw.first + rr * (bw.first - aw.first);
    const double wz = aw.second + rr * (bw.second - aw.second);
    const auto [i, j] = cell_of(wx, wz);
    if (!in_bounds(i, j) || !walkable_[idx(i, j)]) return false;
    if (has_prev) {
      if (i != pi && j != pj) {
        const int c1[2] = {i, pj}, c2[2] = {pi, j};
        for (const int * c : {c1, c2})
          if (in_bounds(c[0], c[1]) && forbidden_[idx(c[0], c[1])]) return false;
      }
      // ★ 墙：真实子线段精确相交（格中心段会错位漏检，Python 版同款修）
      if (true_wall_blocked(pwx, pwz, wx, wz)) return false;
      if (gate_step_blocked(pi, pj, i, j)) return false;
    }
    pi = i; pj = j; pwx = wx; pwz = wz; has_prev = true;
  }
  return true;
}

// 一条直线段的等效代价，记账口径与 A* 完全一致：
//   几何长度 + 目标格 step_extra。
// 不含 turn_penalty（拐弯是路径偏好，不是几何/安全代价）。
// string_pull 用它保证"拉直不改道"。
double NavGrid::seg_cost(std::pair<int, int> a, std::pair<int, int> b) const {
  const int n = std::max(1, static_cast<int>(
      std::hypot(b.first - a.first, b.second - a.second) / 0.5));
  double cost = 0.0;
  int pi = 0, pj = 0;
  bool has_prev = false;
  for (int k = 0; k <= n; ++k) {
    const double rr = static_cast<double>(k) / n;
    const int i = static_cast<int>(std::llround(a.first + rr * (b.first - a.first)));
    const int j = static_cast<int>(std::llround(a.second + rr * (b.second - a.second)));
    if (has_prev && (i != pi || j != pj)) {
      const double len = std::hypot(i - pi, j - pj) * res;
      cost += len + step_extra(pi, pj, i, j, len);
    }
    pi = i; pj = j; has_prev = true;
  }
  return cost;
}

// 拉直：把 A* 的 8 邻接锯齿压成"直段 + 真拐角"。
// 从当前点 i 尽量找最远的 j，要求同时满足：
//   ① line_clear(i,j) 通过；
//   ② seg_cost(i,j) <= C[j]-C[i]（拉直后的等效代价不高于原 A* 子路径）。
// 只有①会把 A* 花代价绕开的墙抄成穿墙直线 —— 实测撞墙过。
std::vector<std::pair<int, int>> NavGrid::string_pull(
    const std::vector<std::pair<int, int>> & cells) const {
  if (cells.size() <= 2) return cells;
  std::vector<double> C(cells.size(), 0.0);
  for (size_t k = 1; k < cells.size(); ++k) {
    const double len = std::hypot(cells[k].first - cells[k - 1].first,
                                  cells[k].second - cells[k - 1].second) * res;
    C[k] = C[k - 1] + len + step_extra(cells[k - 1].first, cells[k - 1].second,
                                       cells[k].first, cells[k].second, len);
  }
  std::vector<std::pair<int, int>> out{cells[0]};
  size_t i = 0;
  while (i + 1 < cells.size()) {
    size_t j = cells.size() - 1;
    while (j > i + 1 &&
           !(line_clear(cells[i], cells[j]) &&
             seg_cost(cells[i], cells[j]) <= C[j] - C[i] + 1e-6))
      --j;
    out.push_back(cells[j]);
    i = j;
  }
  return out;
}

// 起点/终点可能落在墙里、陷坑里或建筑内部。
// 从 r=1 开始一圈圈找最近的可走格；max_r=40 是安全上限。
// 返回 {-1,-1} = 40 格内没有可走格，plan() 会返回 nullopt。
std::pair<int, int> NavGrid::nearest_walkable(int i, int j, int max_r) const {
  if (walkable(i, j)) return {i, j};
  for (int r = 1; r < max_r; ++r) {
    for (int di = -r; di <= r; ++di) {
      for (int dj : {-r, r}) {
        const int cand[2][2] = {{i + di, j + dj}, {i + dj, j + di}};
        for (const auto & c : cand)
          if (walkable(c[0], c[1])) return {c[0], c[1]};
      }
    }
  }
  return {-1, -1};
}

// ============================================================================
// A* 主入口
//
// 输入：起点/终点世界坐标 (sx,sz)/(gx,gz)，turn_penalty 拐弯附加代价。
// 输出：拉直后的世界坐标路径点；无路返回 nullopt。
//
// 搜索空间：ni*nj 个格子，8 邻接。
// 代价模型（每一步）：
//     step = 几何长度（直走 1，斜走 √2）
//          + step_extra（目标格贴墙代价）
//          + turn_penalty（方向变了才加）
// 禁走条件：
//     · 格子不在 walkable_
//     · wall_step_blocked：格中心段横穿墙
//     · gate_step_blocked：逆着单向闸门
//
// 注意：这里的状态只有"格子"，没有把"进入方向"放进状态；
// 所以 turn_penalty 是一个偏好项，不保证方向增广意义下的严格最优。
// 对这种 46x85 的小图和"少打折线"的目标，这个近似足够好。
// ============================================================================
std::optional<std::vector<std::pair<double, double>>> NavGrid::plan(
    double sx, double sz, double gx, double gz, double turn_penalty) const {
  // 1) 世界坐标 → 格子；起终点若不可走，就近挪到可走格。
  auto sc = cell_of(sx, sz);
  auto gc = cell_of(gx, gz);
  auto s = nearest_walkable(sc.first, sc.second);
  auto g = nearest_walkable(gc.first, gc.second);
  if (s.first < 0 || g.first < 0) return std::nullopt;
  if (s == g) return std::vector<std::pair<double, double>>{world_of(g.first, g.second)};

  // 2) A* 数据结构。
  //    gcost[v]  从起点到 v 的已知最小代价（初始 ∞）
  //    came[v]   v 的前驱（线性下标，-1=未访问）
  //    closed[v] v 是否已经弹出并扩展过
  //    openq     小根堆，元素 (f, 线性下标)，f = gcost + 启发式
  // 没有 decrease-key：同一个点可能入堆多次，弹出时用 closed 去重。
  const size_t n = static_cast<size_t>(ni) * nj;
  const double kInf = std::numeric_limits<double>::infinity();
  std::vector<double> gcost(n, kInf);
  std::vector<int> came(n, -1);
  std::vector<uint8_t> closed(n, 0);
  using Item = std::pair<double, int>;      // (f, 线性下标)
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> openq;
  const int si = idx(s.first, s.second);
  gcost[si] = 0.0;
  openq.push({0.0, si});
  const int gi = idx(g.first, g.second);

  // 3) A* 主循环。
  while (!openq.empty()) {
    // 弹出当前 f 最小的格子；closed 的重复项直接丢掉（lazy deletion）。
    const auto [f, u] = openq.top();
    openq.pop();
    if (closed[u]) continue;
    closed[u] = 1;
    if (u == gi) break;
    const int ci = u / nj, cj = u % nj;
    // 4) 8 邻接扩展。kMoves 在文件顶部：4 直走 + 4 对角，对角 w=√2。
    for (const Move & m : kMoves) {
      const int ni2 = ci + m.di, nj2 = cj + m.dj;
      if (!in_bounds(ni2, nj2) || !walkable_[idx(ni2, nj2)] || closed[idx(ni2, nj2)])
        continue;
      // 硬约束：横穿墙 / 逆单向闸门，直接不走。
      if (wall_step_blocked(ci, cj, ni2, nj2)) continue;
      if (gate_step_blocked(ci, cj, ni2, nj2)) continue;
      // 代价 = 几何长度 + 目标格贴墙代价。
      double step = m.w * res;
      step += step_extra(ci, cj, ni2, nj2, m.w * res);
      // 拐弯惩罚：和"上一步的方向"不同才加。
      // came[u] 是 u 的前驱；用 (当前-前驱) 和这一步 (di,dj) 比。
      if (turn_penalty > 0 && came[u] >= 0) {
        const int pi = came[u] / nj, pj = came[u] % nj;
        if ((ci - pi) != m.di || (cj - pj) != m.dj) step += turn_penalty;
      }
      // 松弛：找到更优的 g 就更新前驱并入堆。
      const double ng = gcost[u] + step;
      const int v = idx(ni2, nj2);
      if (ng < gcost[v]) {
        gcost[v] = ng;
        came[v] = u;
        // 启发式 h = 到终点的欧氏距离（世界单位）。
        // 每条边的实际代价 >= 欧氏长度（还多了贴墙/拐弯），
        // 所以 h 可采纳，A* 不会因为 h 漏掉更优路径。
        const double hh = std::hypot(ni2 - g.first, nj2 - g.second) * res;
        openq.push({ng + hh, v});
      }
    }
  }
  // 5) 终点从未被松弛到 → 无路。
  if (came[gi] < 0) return std::nullopt;

  // 6) 回溯：从终点沿 came 一路走到起点。
  //    注意这里先得到"终点→起点"的顺序，最后 reverse。
  std::vector<std::pair<int, int>> cells;
  for (int cur = gi; cur != si; cur = came[cur]) cells.push_back({cur / nj, cur % nj});
  cells.push_back(s);
  std::reverse(cells.begin(), cells.end());
  // 7) 拉直：A* 的 8 邻接路径全是 45° 锯齿，pure pursuit 跟着会画龙。
  //    string_pull 会把它压成"直段 + 真拐角"，同时保证不改变代价口径。
  cells = string_pull(cells);
  // 8) 格子中心 → 世界坐标。调用方拿到的是路径点，不是格子下标。
  std::vector<std::pair<double, double>> out;
  out.reserve(cells.size());
  for (const auto & c : cells) out.push_back(world_of(c.first, c.second));
  return out;
}

// 运行时学障碍：把 (x,z) 周围 radius 内标成不可走。
// 只改 walkable_，不重算 prox_：新障碍立即硬不可走，周围暂时没有贴墙代价。
// 卡死检测里块心会沿指令方向前移 1.4，避免把车自己围死。
void NavGrid::block_around(double x, double z, double radius) {
  const auto [ci, cj] = cell_of(x, z);
  const int r = static_cast<int>(std::ceil(radius / res));
  for (int di = -r; di <= r; ++di)
    for (int dj = -r; dj <= r; ++dj) {
      if (di * di + dj * dj > r * r) continue;
      if (in_bounds(ci + di, cj + dj)) walkable_[idx(ci + di, cj + dj)] = 0;
    }
}

// 自救解封：把 (x,z) 周围 radius 内重新标成可走。
// 只在"首次规划无通路"时调用，保证被学到的障碍围死时至少能退出来。
// 同样只改 walkable_，不重算 prox_。
void NavGrid::unblock_around(double x, double z, double radius) {
  const auto [ci, cj] = cell_of(x, z);
  const int r = static_cast<int>(std::ceil(radius / res));
  for (int di = -r; di <= r; ++di)
    for (int dj = -r; dj <= r; ++dj) {
      if (di * di + dj * dj > r * r) continue;
      if (in_bounds(ci + di, cj + dj)) walkable_[idx(ci + di, cj + dj)] = 1;
    }
}
