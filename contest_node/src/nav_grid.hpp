//导航栅格与 A*：读 nav_data/nav_grid.bin（由 nav_map_report.py 生成）
//坐标：Unity 世界 (x,z)，3 场景单位 = 1 m；i=x 格下标、j=z 格下标，idx(i,j)=i*nj+j
//墙为折线（严格横穿才禁），另有单向闸门与 prox_ 贴墙代价层
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
  static constexpr double kProxR = 2.0; //贴墙惩罚半径（场景单位）
  static constexpr double kProxCost = 1.5; //墙边每单位长度的最大附加代价

  int ni = 0, nj = 0; //栅格尺寸：ni 个 x 格，nj 个 z 格
  double fx0 = 0, fz0 = 0, res = 1.0; //格子 (0,0) 的世界坐标 / 每格多少场景单位

  //原始层（与文件顺序一致）：free 地形 / highland 仅核对 / forbidden 硬禁 / barrier 作 prox 来源 / forced 强制可走
  std::vector<uint8_t> free_, highland_, forbidden_, barrier_, forced_;
  std::vector<int8_t> gate_dx_, gate_dz_; //单向闸门允许方向；(0,0)=不是闸门
  std::vector<float> wall_segs_; //墙折线，每段 (x1,z1,x2,z2)
  std::vector<uint8_t> walkable_; //load() 算出的最终可走层（外扩 + forced）
  std::vector<float> prox_; //贴墙代价层：离障碍/墙越近代价越高
  std::unordered_map<uint64_t, std::vector<int>> wb_; //墙折线空间桶（2 单位）

  //读 nav_grid.bin 并生成 walkable_/prox_；失败（缺文件/magic/版本/截断）返回 false
  bool load(const std::string & path);

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

  //一步 (ci,cj)→(i,j) 的贴墙附加代价，只看目标格 (i,j)
  double step_extra(int ci, int cj, int i, int j, double length) const;
  //这一步是否逆着单向闸门（对角步连两个角格一起查）
  bool gate_step_blocked(int pi, int pj, int i, int j) const;
  //A* 用：格中心到格中心的线段是否横穿墙折线
  bool wall_step_blocked(int ci, int cj, int i, int j) const;
  //拉直用：真实采样线段是否横穿墙折线（交点落在闸门格内豁免）
  bool true_wall_blocked(double x1, double z1, double x2, double z2) const;
  static bool proper_cross(double ax, double az, double bx, double bz,
                           double cx, double cz, double dx, double dz);
  static std::pair<double, double> intersect(double ax, double az, double bx, double bz,
                                             double cx, double cz, double dx, double dz);

  //按 0.5 单位采样：全程可走、不穿墙、不逆闸门
  bool line_clear(std::pair<int, int> a, std::pair<int, int> b) const;
  //直线等效代价（长度 + 目标格 step_extra），与 A* 同口径
  double seg_cost(std::pair<int, int> a, std::pair<int, int> b) const;
  //8 邻接锯齿拉成直段+拐角：line_clear 通过且等效代价不高于原段才拉
  std::vector<std::pair<int, int>> string_pull(
      const std::vector<std::pair<int, int>> & cells) const;

  //环形向外找最近可走格；max_r 内找不到返回 {-1,-1}
  std::pair<int, int> nearest_walkable(int i, int j, int max_r = 40) const;

  //A* 入口：start/goal 为世界 (x,z)，返回世界系路径点或 nullopt；turn_penalty 为拐弯附加代价（常用 0.4）
  std::optional<std::vector<std::pair<double, double>>> plan(
      double sx, double sz, double gx, double gz, double turn_penalty = 0.4) const;

  //把 (x,z) 半径 r 内标为不可走；只改 walkable_，不重算 prox_
  void block_around(double x, double z, double radius = 1.6);

  //把 (x,z) 半径 r 内恢复可走，规划失败时自救；只改 walkable_，不重算 prox_
  void unblock_around(double x, double z, double radius = 1.5);
};
