#pragma once

#include <map>
#include <list>
#include <chrono>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "types.hpp"
#include "target.hpp"
#include "solver.hpp"

// ============================================================================
// Tracker —— 多目标跟踪器
//
// 职责：把 detector 每帧吐出来的一堆 Armor（已经过 solver.solvePnP 解算），
//       归到不同的 track 上，每个 track 跑一个独立的 EKF，并决定"锁谁"。
//
// 数据流：
//   main.cpp:  armors ──> [本类 update()] ──> has_lock()/locked_id() ──> Aimer
//
// 一个重要的设计前提：本类【假设传进来的 Armor 已经解算过了】，
//   a.xyz_in_camera / a.yaw 是 solvePnP 的输出。main.cpp:77 在调用本类之前
//   已经 for(auto& a : armors) solver.solve(a)，这里直接用现成的值。
//   （所以下面那个 Solver& solver_ 成员其实从未被使用，见 tracker.hpp:54）
// ============================================================================

namespace YAML { class Node; }

class Tracker {
public:
  Tracker(const YAML::Node & cfg, Solver & solver);

  // 每帧调用一次：用本帧所有检测到的装甲板更新所有 track
  void update(const std::list<Armor> & armors,
              std::chrono::steady_clock::time_point now);

  // 所有 track，key 是装甲板编号（见 .cpp 里对 vid 的说明）
  const std::map<int, Target> & tracks() const { return tracks_; }

  // 不改动 t 的内部状态地【试算】dt 秒后某块装甲板的位置（返回到相机系）
  // 用于可视化时画"预测箭头"，以及将来做提前量
  Eigen::Vector3d predict_armor(const Target & t, int armor_id, double dt = 0.04) const;

  // 判断一个观测（相机系下的装甲板）对应 Target 模型里的第几块板（0..3）
  // 用于把检测到的 2D 观测，和内部 4 块装甲板的模型对应起来
  int get_armor_plate_id(const Target & t, const Eigen::Vector4d & obs) const;

  // EKF 发散被重建的次数（发散自愈，见 .cpp 更新循环）
  int diverged_resets() const { return diverged_resets_; }

  int locked_id() const { return locked_id_; }
  bool has_lock() const { return locked_id_ >= 0; }

private:
  // 注意：solver_ 目前没有任何地方使用（只是构造时存了下来），是残留代码。
  // 本类要用的位姿解算结果，由调用方在传入前算好。
  Solver & solver_;

  // 所有 track：key = 装甲板编号(int)a.name，value = 该编号对应的目标状态
  std::map<int, Target> tracks_;

  // 每个装甲板编号上一次被更新的时刻，用于算 EKF 的 dt（帧间隔可能不均匀）
  std::map<int, std::chrono::steady_clock::time_point> last_t_;

  // 当前锁定的装甲板编号；-1 表示没有锁
  int locked_id_ = -1;

  // 锁定目标连续丢帧计数。连续丢满 max_temp_lost_count_ 帧就解锁（见 .cpp update 开头）
  int lost_frames_ = 0;

  // 丢帧容忍帧数，来自 config 的 max_temp_lost_count（默认 15，和 sp_vision 一致）
  int max_temp_lost_count_ = 15;

  // 发散重建计数（只增，给上层打日志用）
  int diverged_resets_ = 0;

  // 敌方颜色。注意：本类自己并不用它做过滤，main.cpp:75 已经过滤过了；
  // 这个成员目前是只写不读的。
  Color enemy_color_ = red;

  // 用一块装甲板的初次观测，初始化一个 Target（半径、板数、P0 都按兵种区分）
  void init_target(Target & t, const Armor & a,
                   std::chrono::steady_clock::time_point now);

  // 没有锁定时，从本帧的装甲板里按优先级挑一个锁上
  void lock_target(const std::list<Armor> & armors);
};
