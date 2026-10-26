#pragma once

#include <map>
#include <list>
#include <chrono>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "types.hpp"
#include "target.hpp"
#include "solver.hpp"

//Tracker：多目标跟踪与锁定
//输入 Armor 必须已完成 Solver::solve()，track key=(int)a.name
//每个 track 各跑一个 EKF，锁定优先级见 get_armor_priority()

namespace YAML { class Node; }

class Tracker {
public:
  Tracker(const YAML::Node & cfg, Solver & solver);

  //每帧调用一次，更新所有 track
  void update(const std::list<Armor> & armors,
              std::chrono::steady_clock::time_point now);

  //所有 track，key 为装甲板编号
  const std::map<int, Target> & tracks() const { return tracks_; }

  //试算 dt 秒后某块板在相机系的位置，不修改 t 的状态
  Eigen::Vector3d predict_armor(const Target & t, int armor_id, double dt = 0.04) const;

  //返回观测对应的板编号 0..armor_num_-1
  int get_armor_plate_id(const Target & t, const Eigen::Vector4d & obs) const;

  //发生发散并重建的次数
  int diverged_resets() const { return diverged_resets_; }

  int locked_id() const { return locked_id_; }
  bool has_lock() const { return locked_id_ >= 0; }

private:
  //当前未使用，位姿已在传入前解算完成
  Solver & solver_;

  //key 为装甲板编号，value 为该编号对应的目标状态
  std::map<int, Target> tracks_;

  //每个编号上次推进预测的时刻，用于计算非均匀 dt
  std::map<int, std::chrono::steady_clock::time_point> last_t_;

  //当前锁定的装甲板编号，-1 表示未锁定
  int locked_id_ = -1;

  //锁定目标连续未检测到的帧数
  int lost_frames_ = 0;

  //连续丢帧超过此值才解锁，来自 config 的 max_temp_lost_count
  int max_temp_lost_count_ = 15;

  //发散重建次数
  int diverged_resets_ = 0;

  //当前只写不读，敌我过滤在外部完成
  Color enemy_color_ = red;

  //按兵种初始化半径、板数和初始协方差
  void init_target(Target & t, const Armor & a,
                   std::chrono::steady_clock::time_point now);

  //未锁定时按优先级和距离选一个目标
  void lock_target(const std::list<Armor> & armors);
};
