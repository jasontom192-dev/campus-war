// 导航栅格 + A*（C++ 版）—— 从 Python 的 nav_map.py 逐项移植（那里已验证过）。
// 数据从 nav_data/nav_grid.bin 读（由 nav_map_report.py 生成；简单二进制，不折腾 npz）。
//
// ★ 口径必须和 Python 一模一样，改任何一处判据时两边一起改（对拍脚本会抓）：
//   · 墙 = 几何折线，横穿才禁（wall_step_blocked，严格相交）
//   · 单向闸门（gate_step_blocked，进/出闸门格 + 对角角格都要查）
//   · 贴墙惩罚 step_extra（prox 按长度）；A*、拉直（line_clear/_seg_cost）共用
//   · 拉直许可 = line_clear 通过【且】等效代价不高于被替换段
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

  int ni = 0, nj = 0;
  double fx0 = 0, fz0 = 0, res = 1.0;

  std::vector<uint8_t> free_, highland_, forbidden_, barrier_, forced_;
  std::vector<int8_t> gate_dx_, gate_dz_;
  std::vector<float> wall_segs_;             // 每段 (x1,z1,x2,z2)
  std::vector<uint8_t> walkable_;
  std::vector<float> prox_;
  std::unordered_map<uint64_t, std::vector<int>> wb_;   // 墙折线空间桶（2 单位）

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
  double step_extra(int ci, int cj, int i, int j, double length) const;
  bool gate_step_blocked(int pi, int pj, int i, int j) const;
  bool wall_step_blocked(int ci, int cj, int i, int j) const;
  bool true_wall_blocked(double x1, double z1, double x2, double z2) const;
  static bool proper_cross(double ax, double az, double bx, double bz,
                           double cx, double cz, double dx, double dz);
  static std::pair<double, double> intersect(double ax, double az, double bx, double bz,
                                             double cx, double cz, double dx, double dz);

  bool line_clear(std::pair<int, int> a, std::pair<int, int> b) const;
  double seg_cost(std::pair<int, int> a, std::pair<int, int> b) const;
  std::vector<std::pair<int, int>> string_pull(
      const std::vector<std::pair<int, int>> & cells) const;

  std::pair<int, int> nearest_walkable(int i, int j, int max_r = 40) const;

  // start/goal 是 Unity 世界 (x, z)。返回路径点（世界系）或 nullopt。
  std::optional<std::vector<std::pair<double, double>>> plan(
      double sx, double sz, double gx, double gz, double turn_penalty = 0.4) const;

  // 运行时学障碍：把 (x,z) 半径 r 内标成不可走（撞墙/掉坑都走这条）。
  void block_around(double x, double z, double radius = 1.6);

  // 自救"解封"：把 (x,z) 半径 r 内重新标成可走。和 Python 版 nav_node.py 的
  // "首次规划无通路 → 放开车周围一圈重试"同口径 —— 被"学到的障碍"围死时
  // 至少能退出来再试，避免原地卡死报"没有通路"。
  void unblock_around(double x, double z, double radius = 1.5);
};
