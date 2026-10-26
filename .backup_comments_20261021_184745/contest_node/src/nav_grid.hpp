// 导航栅格 + A*（C++ 版）—— 从 Python 的 nav_map.py 逐项移植（那里已验证过）。
// 数据从 nav_data/nav_grid.bin 读（由 nav_map_report.py 生成；简单二进制，不折腾 npz）。
//
// ★ 口径必须和 Python 一模一样，改任何一处判据时两边一起改（对拍脚本会抓）：
//   · 墙 = 几何折线，横穿才禁（wall_step_blocked，严格相交）
//   · 单向闸门（gate_step_blocked，进/出闸门格 + 对角角格都要查）
//   · 贴墙惩罚 step_extra（prox 按长度）；A*、拉直（line_clear/_seg_cost）共用
//   · 拉直许可 = line_clear 通过【且】等效代价不高于被替换段
//
// ── C++ 导航阅读顺序 ───────────────────────────────────────────────────────
//   1. NavGrid 的成员：几层地图 + 一个 A* 规划器。
//   2. load()：读 nav_data/nav_grid.bin，算 walkable_ / prox_，建墙折线空间桶。
//   3. plan()：A* 主循环（8 邻接）→ 从 came 回溯 → string_pull 拉直。
//   4. 调用方：auto_aim_node.cpp 的 mission_tick()（50 ms 一次）。
//
// 坐标：Unity 世界 (x,z)，3 场景单位 = 1 m。
//   i = x 方向格子下标，j = z 方向格子下标，idx(i,j) = i*nj + j。
#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

struct NavGrid {
  static constexpr double kRobotRadius = 1.2;
  static constexpr double kProxR = 2.0;      // 贴墙惩罚半径（场景单位）
  static constexpr double kProxCost = 1.5;   // 墙边每单位长度的最大附加代价

  int ni = 0, nj = 0;                 // 栅格尺寸：ni 个 x 格，nj 个 z 格
  double fx0 = 0, fz0 = 0, res = 1.0; // 格子 (0,0) 的世界坐标 / 每格多少场景单位

  // ---- 地图原始层（load() 从 nav_grid.bin 读入，顺序与文件一致）----
  // free_      地形可通行（战术地图颜色分层）
  // highland_  高地表面：只用于画图/人工核对，不参与规划
  // forbidden_ 陷坑等硬禁行：从可走层里扣掉
  // barrier_   等级硬边/台阶墙：不硬禁，只作为 prox_ 贴墙代价的来源
  // forced_    实测坡道/闸门：免疫障碍外扩，强制可走
  std::vector<uint8_t> free_, highland_, forbidden_, barrier_, forced_;
  std::vector<int8_t> gate_dx_, gate_dz_; // 单向闸门允许方向；(0,0)=不是闸门
  std::vector<float> wall_segs_;             // 墙折线，每段 (x1,z1,x2,z2)
  std::vector<uint8_t> walkable_;            // load() 算出的最终可走层（外扩 + forced）
  std::vector<float> prox_;                  // 贴墙代价层：离障碍/墙越近代价越高
  std::unordered_map<uint64_t, std::vector<int>> wb_;   // 墙折线空间桶（2 单位）

  // 读 nav_grid.bin：恢复原始层 → 算 walkable_/prox_ → 建墙折线空间桶。
  // 返回 false = 文件不存在 / magic 不对 / 版本不对 / 读不全。
  bool load(const std::string & path);

  // ---- 坐标换算 ----
  inline int idx(int i, int j) const { return i * nj + j; }
  inline bool in_bounds(int i, int j) const { return i >= 0 && i < ni && j >= 0 && j < nj; }
  inline std::pair<int, int> cell_of(double x, double z) const {
    return {static_cast<int>(std::llround((x - fx0) / res)),
            static_cast<int>(std::llround((z - fz0) / res))};
  }
  inline std::pair<double, double> world_of(int i, int j) const {
    return {fx0 + i * res, fz0 + j * res};
  }
  inline bool walkable(int i, int j) const { return in_bounds(i, j) && walkable_[idx(i, j)]; }
  inline bool barrier(int i, int j) const { return in_bounds(i, j) && barrier_[idx(i, j)]; }
  inline bool forbidden(int i, int j) const { return in_bounds(i, j) && forbidden_[idx(i, j)]; }

  // ---- 判据（和 Python 同名同逻辑）----
  // step_extra：一步 (ci,cj)→(i,j) 的贴墙附加代价，只看目标格 (i,j)。
  double step_extra(int ci, int cj, int i, int j, double length) const;
  // gate_step_blocked：这一步是否逆着单向闸门（对角步连角格一起查）。
  bool gate_step_blocked(int pi, int pj, int i, int j) const;
  // wall_step_blocked：A* 用。格中心 → 格中心的线段是否横穿墙折线。
  bool wall_step_blocked(int ci, int cj, int i, int j) const;
  // true_wall_blocked：拉直用。真实采样线段是否横穿墙折线（交点落在闸门格内豁免）。
  bool true_wall_blocked(double x1, double z1, double x2, double z2) const;
  static bool proper_cross(double ax, double az, double bx, double bz,
                           double cx, double cz, double dx, double dz);
  static std::pair<double, double> intersect(double ax, double az, double bx, double bz,
                                             double cx, double cz, double dx, double dz);

  // line_clear：按 0.5 单位采样，直线全程可走 + 不穿墙 + 不逆闸门。
  bool line_clear(std::pair<int, int> a, std::pair<int, int> b) const;
  // seg_cost：一条直线段的等效代价（长度 + 目标格 step_extra），与 A* 同尺子。
  double seg_cost(std::pair<int, int> a, std::pair<int, int> b) const;
  // string_pull：把 8 邻接锯齿拉成"直段 + 真拐角"；
  // 允许拉直的条件 = line_clear 通过【且】seg_cost 不高于被替换段。
  std::vector<std::pair<int, int>> string_pull(
      const std::vector<std::pair<int, int>> & cells) const;

  // 起点/终点不在可走格时，环形搜索最近的可走格；找不到返回 {-1,-1}。
  std::pair<int, int> nearest_walkable(int i, int j, int max_r = 40) const;

  // A* 主入口：start/goal 是 Unity 世界 (x, z)，返回路径点（世界系）或 nullopt。
  // turn_penalty 是拐弯附加代价（调用方通常传 0.4，让路径少打折线）。
  std::optional<std::vector<std::pair<double, double>>> plan(
      double sx, double sz, double gx, double gz, double turn_penalty = 0.4) const;

  // 运行时学障碍：把 (x,z) 半径 r 内标成不可走（撞墙/掉坑都走这条）。
  // ★ 只改 walkable_，不重算 prox_：新障碍立即硬不可走，周围暂时没有贴墙代价。
  void block_around(double x, double z, double radius = 1.6);

  // 自救"解封"：把 (x,z) 半径 r 内重新标成可走。和 Python 版 nav_node.py 的
  // "首次规划无通路 → 放开车周围一圈重试"同口径 —— 被"学到的障碍"围死时
  // 至少能退出来再试，避免原地卡死报"没有通路"。
  // ★ 同样只改 walkable_，不重算 prox_。
  void unblock_around(double x, double z, double radius = 1.5);
};
