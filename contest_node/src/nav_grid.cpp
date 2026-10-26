//导航栅格加载与 A* 规划实现
//A*：8 邻接，代价=长度+prox_+拐弯；墙/闸门/贴墙判据在 A* 与拉直中必须一致
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
} //namespace

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

  //文件布局："NAVG" + ver/ni/nj + fx0/fz0/res + 5 层 uint8 + gate_dx/gate_dz + nseg + 墙段 float32
  //walkable = 膨胀(free & ~forbidden, R) | (forced & ~forbidden)；膨胀带环绕（与生成端一致）

  //障碍外扩
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
    walkable_[k] = ((src[k] && !grown[k]) || (forced_[k] && !forbidden_[k])) ? 1 : 0;

  //prox_：障碍 = barrier | !walkable；prox = clamp((kProxR-d)/kProxR,0,1)*kProxCost
  cv::Mat obst(ni, nj, CV_8UC1);
  for (int i = 0; i < ni; ++i)
    for (int j = 0; j < nj; ++j)
      obst.at<uint8_t>(i, j) = (barrier_[idx(i, j)] || !walkable_[idx(i, j)]) ? 1 : 0;
  cv::Mat srcm, dist;
  //distanceTransform 需要障碍=0、自由=255，写反会让整层 prox_ 全 0
  cv::compare(obst, 0, srcm, cv::CMP_EQ);
  cv::distanceTransform(srcm, dist, cv::DIST_L2, 3);
  prox_.resize(n);
  for (int i = 0; i < ni; ++i)
    for (int j = 0; j < nj; ++j) {
      const double d = dist.at<float>(i, j);
      const double c = std::clamp((kProxR - d) / kProxR, 0.0, 1.0) * kProxCost;
      prox_[idx(i, j)] = static_cast<float>(c);
    }

  //墙折线按 2x2 单位分桶加速；键 = (bi+512)<<32 | (bj+512)
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

//一步的贴墙代价 = 目标格 prox_ × 步长；前两个参数为兼容签名保留
double NavGrid::step_extra(int, int, int i, int j, double length) const {
  return static_cast<double>(prox_[idx(i, j)]) * length;
}

//逆单向闸门则禁：查目标格（对角再查两角格，无标记回退查上一格），di*gx+dj*gz<0 为逆向
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

//严格横穿：两线段端点各在对方异侧；端点接触、共线重叠不算（沿墙走不误禁）
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

//A* 用：格中心连线段是否横穿墙折线；闸门格内豁免，只查包围盒覆盖的桶
bool NavGrid::wall_step_blocked(int ci, int cj, int i, int j) const {
  if (wall_segs_.empty()) return false;
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

//拉直用：真实采样段与墙折线精确求交，闸门格内豁免；格中心段会漏细墙，不能复用 wall_step_blocked
bool NavGrid::true_wall_blocked(double x1, double z1, double x2, double z2) const {
  if (wall_segs_.empty()) return false;
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
              continue; //出口线内穿越放行
            return true;
          }
        }
      }
  }
  return false;
}

//两线段（已知严格横穿）的交点；平行/共线时退化为四点中心
std::pair<double, double> NavGrid::intersect(double ax, double az, double bx, double bz,
                                             double cx, double cz, double dx, double dz) {
  const double den = (bx - ax) * (dz - cz) - (bz - az) * (dx - cx);
  if (std::abs(den) < 1e-12)
    return {(ax + bx + cx + dx) / 4.0, (az + bz + cz + dz) / 4.0};
  const double t = ((cx - ax) * (dz - cz) - (cz - az) * (dx - cx)) / den;
  return {ax + t * (bx - ax), az + t * (bz - az)};
}

//拉直采样：0.5 单位步长，采样点可走、对角角格非 forbidden、不穿墙、不逆闸门
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
      if (true_wall_blocked(pwx, pwz, wx, wz)) return false;
      if (gate_step_blocked(pi, pj, i, j)) return false;
    }
    pi = i; pj = j; pwx = wx; pwz = wz; has_prev = true;
  }
  return true;
}

//直线等效代价 = 长度 + 目标格 step_extra，与 A* 同口径（不含拐弯）
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

//拉直：从 i 贪心找最远 j，要求 line_clear 通过且 seg_cost 不高于原 A* 子路径
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

//从 r=1 起环形向外找最近可走格；max_r 内找不到返回 {-1,-1}，plan() 随后返回 nullopt
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

//A*
std::optional<std::vector<std::pair<double, double>>> NavGrid::plan(
    double sx, double sz, double gx, double gz, double turn_penalty) const {
  //世界坐标->格子；起终点不可走时就近挪到可走格
  auto sc = cell_of(sx, sz);
  auto gc = cell_of(gx, gz);
  auto s = nearest_walkable(sc.first, sc.second);
  auto g = nearest_walkable(gc.first, gc.second);
  if (s.first < 0 || g.first < 0) return std::nullopt;
  if (s == g) return std::vector<std::pair<double, double>>{world_of(g.first, g.second)};

  //gcost 最小已知代价、came 前驱（-1=未访问）、closed 已扩展；无 decrease-key，堆内可能重复
  const size_t n = static_cast<size_t>(ni) * nj;
  const double kInf = std::numeric_limits<double>::infinity();
  std::vector<double> gcost(n, kInf);//最小代价
  std::vector<int> came(n, -1); //前驱用来回溯路径
  std::vector<uint8_t> closed(n, 0);
  using Item = std::pair<double, int>; //(f, 线性下标)
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> openq;
  const int si = idx(s.first, s.second);//入队
  gcost[si] = 0.0;
  openq.push({0.0, si});
  const int gi = idx(g.first, g.second);

  while (!openq.empty()) {
    const auto [f, u] = openq.top();
    openq.pop();
    if (closed[u]) continue;
    closed[u] = 1;
    if (u == gi) break;
    const int ci = u / nj, cj = u % nj;
    //扩展
    for (const Move & m : kMoves) {
      const int ni2 = ci + m.di, nj2 = cj + m.dj;
      if (!in_bounds(ni2, nj2) || !walkable_[idx(ni2, nj2)] || closed[idx(ni2, nj2)])//越界
        continue;
      if (wall_step_blocked(ci, cj, ni2, nj2)) continue;//穿墙
      if (gate_step_blocked(ci, cj, ni2, nj2)) continue;//台阶
      double step = m.w * res;
      step += step_extra(ci, cj, ni2, nj2, m.w * res);
      //方向与上一步不同才加 turn_penalty；入口方向未纳入状态，不保证严格最优
      if (turn_penalty > 0 && came[u] >= 0) {
        const int pi = came[u] / nj, pj = came[u] % nj;
        if ((ci - pi) != m.di || (cj - pj) != m.dj) step += turn_penalty;
      }
      const double ng = gcost[u] + step;
      const int v = idx(ni2, nj2);
      if (ng < gcost[v]) {
        gcost[v] = ng;
        came[v] = u;
        //h = 到终点欧氏距离（世界单位）；边代价 >= 欧氏长度，可采纳
        const double hh = std::hypot(ni2 - g.first, nj2 - g.second) * res;
        openq.push({ng + hh, v});//f
      }
    }
  }
  if (came[gi] < 0) return std::nullopt;

  //沿 came 回溯，先得终点→起点，最后 reverse
  std::vector<std::pair<int, int>> cells;
  for (int cur = gi; cur != si; cur = came[cur]) cells.push_back({cur / nj, cur % nj});
  cells.push_back(s);
  std::reverse(cells.begin(), cells.end());
  //拉直成直段+拐角，避免 pure pursuit 跟随 45° 锯齿画龙；代价口径不变
  cells = string_pull(cells);
  std::vector<std::pair<double, double>> out;
  out.reserve(cells.size());
  for (const auto & c : cells) out.push_back(world_of(c.first, c.second));
  return out;
}

//把 (x,z) 周围 radius 内标为不可走；只改 walkable_，不重算 prox_
void NavGrid::block_around(double x, double z, double radius) {
  const auto [ci, cj] = cell_of(x, z);
  const int r = static_cast<int>(std::ceil(radius / res));
  for (int di = -r; di <= r; ++di)
    for (int dj = -r; dj <= r; ++dj) {
      if (di * di + dj * dj > r * r) continue;
      if (in_bounds(ci + di, cj + dj)) walkable_[idx(ci + di, cj + dj)] = 0;
    }
}

//规划无通路时把 (x,z) 周围恢复可走；只改 walkable_，不重算 prox_
void NavGrid::unblock_around(double x, double z, double radius) {
  const auto [ci, cj] = cell_of(x, z);
  const int r = static_cast<int>(std::ceil(radius / res));
  for (int di = -r; di <= r; ++di)
    for (int dj = -r; dj <= r; ++dj) {
      if (di * di + dj * dj > r * r) continue;
      if (in_bounds(ci + di, cj + dj)) walkable_[idx(ci + di, cj + dj)] = 1;
    }
}
