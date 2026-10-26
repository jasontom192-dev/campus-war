#pragma once

//打符瞄准与开火决策：预测目标、解弹道、判稳、confirm 开火
//输入目标需传副本(aim 会连续 predict 改写状态)；输出 yaw/pitch 为 spv 世界系 rad
//大符一片一发；等待命中确认，超时按脱靶补枪

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "rune_geometry.hpp"
#include "rune_trajectory.hpp"

//一套开火节奏参数（小符 / 大符各一份，从 config 读）
struct RuneFireProfile {
  double fire_gap_time = 0.52; //两轮之间的最小间隔(s)
  double burst_gap = 0.45; //一轮内发间隔(s)
  int    fire_burst = 3; //一轮最多几发，大符=1
  double miss_timeout = 1.0; //打完这么久没确认判脱靶(s)，允许补枪
  double predict_time = 0.10; //额外提前量(s)
  double gimbal_ff_s = 0.0; //云台滞后前馈(s)
  double pitch_extra_deg = 0.0; //额外抬枪角(度)，6m 处 0.1°≈1cm
  double yaw_extra_deg = 0.0; //额外水平偏置(度)，正值向右，6m 处 0.1°≈1cm

  //大符第二击门控：命中确认后窗口内，预测角连续平稳才放行；0=关闭(小符)
  double second_gate_deg = 0.0; //平稳判据：预测角变化二阶差分(度/帧^2)
  int    second_gate_frames = 6; //连续平稳多少帧才允许放
  double second_window_s = 0.85; //第二击窗口(s)
};

class RuneAimer {
public:
  struct Result {
    bool control = false; //判稳通过，云台可以跟随
    bool shoot = false; //射击许可
    double yaw = 0.0; //spv 世界系方位角(rad)
    double pitch = 0.0; //spv 世界系发射仰角(rad)，抬头为正
    double fly_time = 0.0; //飞行时间(s)
    const char * block = ""; //未开火原因
  };

  RuneAimer() = default;

  //换开火节奏，不重置统计
  void set_profile(const RuneFireProfile & p) { prof_ = p; }
  const RuneFireProfile & profile() const { return prof_; }

  void reset() {
    mistake_count_ = 0;
    switch_fanblade_ = false;
    last_fire_t_ = -1.0;
    pending_shot_ = false;
    burst_left_ = 0;
    last_control_ = false;
    last_hit_t_ = -1e9; //第二击窗口计时
    stable_cnt_ = 0;
    last_dy_ = last_dp_ = 0.0;
  }

  //进入打符模式时对齐上一帧云台角(spv 系，rad)，避免首帧被误判跳变
  void set_last_angles(double yaw, double pitch) {
    last_yaw_ = yaw;
    last_pitch_ = pitch;
  }

  //detect_gap=图像时间戳到现在的秒数；now=单调秒；hit_confirmed=外部命中证据
  template <class TargetT>
  Result aim(TargetT target_copy, double detect_gap, double bullet_speed,
             double now, bool hit_confirmed) {
    Result r;
    if (bullet_speed < 10.0) bullet_speed = 24.0; //初速无效时的兜底
    if (last_fire_t_ < 0.0) last_fire_t_ = now;

    r.yaw = last_yaw_;
    r.pitch = last_pitch_;

    //预测到云台该到位的时刻：检测延时+提前量+云台前馈
    const double future = detect_gap + prof_.predict_time + prof_.gimbal_ff_s;
    double yaw = 0.0, pitch = 0.0, fly = 0.0;
    const bool solved = send_angle(target_copy, future, bullet_speed, yaw, pitch, fly);

    if (solved) {
      r.fly_time = fly;
      const double thr = jump_thres_deg * M_PI / 180.0;
      const double dy = std::abs(last_yaw_ - yaw);
      const double dp = std::abs(last_pitch_ - pitch);
      //平稳度=预测角变化的二阶差分；换片跳变或观测修正时会明显变大
      if (prof_.second_gate_deg > 0.0) {
        const double syaw = yaw - last_yaw_, spitch = pitch - last_pitch_;
        const double jy = std::abs(syaw - last_dy_);
        const double jp = std::abs(spitch - last_dp_);
        last_dy_ = syaw;
        last_dp_ = spitch;
        if (jy < prof_.second_gate_deg * M_PI / 180.0 &&
            jp < prof_.second_gate_deg * M_PI / 180.0) {
          ++stable_cnt_;
        } else {
          stable_cnt_ = 0;
        }
      } else {
        stable_cnt_ = prof_.second_gate_frames; //门关时恒稳
      }
      if (dy > thr || dp > thr) {
        switch_fanblade_ = true;
        mistake_count_++;
        r.control = false;
        if (mistake_count_ > mistake_to_switch) { //连跳够了，按真换叶处理
          mistake_count_ = 0;
          r.control = true;
        }
      } else {
        switch_fanblade_ = false;
        mistake_count_ = 0;
        r.control = true;
      }
      last_yaw_ = yaw;
      last_pitch_ = pitch;
      r.yaw = yaw;
      r.pitch = pitch;
    }

    if (switch_fanblade_) {
      r.shoot = false;
      r.block = "跳变冻结";
      last_fire_t_ = now; //冻结期间持续压住节拍
    } else if (!r.control) {
      r.shoot = false; //瞄点不稳，不开火
      r.block = "锁定中";
    } else {
      confirm_fire(now, hit_confirmed, r);
    }

    last_control_ = r.control;
    return r;
  }

  //统计与状态，日志/窗口用
  int    shots() const { return shots_; }
  int    hits() const { return hits_; }
  int    misses() const { return misses_; }
  int    mistake_count() const { return mistake_count_; }
  bool   pending() const { return pending_shot_; }
  double last_yaw() const { return last_yaw_; }
  double last_pitch() const { return last_pitch_; }

  double jump_thres_deg = 5.0; //判稳阈值(度)
  int    mistake_to_switch = 3; //连跳几帧算真换叶

private:
  //两点迭代求世界系 yaw/pitch；target 是副本，可安全 predict；无解返回 false
  template <class TargetT>
  bool send_angle(TargetT & target, double predict_time, double bullet_speed,
                  double & yaw, double & pitch, double & fly) const {
    if (target.is_unsolved()) return false;

    target.predict(predict_time);
    Eigen::Vector3d aim_w = target.aim_point_world();
    double d = std::hypot(aim_w(0), aim_w(1));
    RuneTrajectory traj0(bullet_speed, d, aim_w(2));
    if (!traj0.ok) return false;

    //按第一段飞行时间再预测一次
    target.predict(traj0.fly_time);
    aim_w = target.aim_point_world();
    d = std::hypot(aim_w(0), aim_w(1));
    RuneTrajectory traj1(bullet_speed, d, aim_w(2));
    if (!traj1.ok) return false;
    if (std::abs(traj1.fly_time - traj0.fly_time) > 0.01) return false; //两次飞时差>0.01s 视为不收敛

    yaw = std::atan2(aim_w(1), aim_w(0)); //spv 球坐标：atan2(前, 右)
    //弹道仰角 + 抬枪补偿 + 水平偏置
    pitch = traj1.pitch + prof_.pitch_extra_deg * M_PI / 180.0;
    yaw += prof_.yaw_extra_deg * M_PI / 180.0;
    fly = traj1.fly_time;
    return true;
  }

  //confirm 节奏：打一发->等命中确认->再打；超时判脱靶
  void confirm_fire(double now, bool hit_confirmed, Result & r) {
    if (hit_confirmed) {
      if (pending_shot_) hits_++;
      pending_shot_ = false; //命中：这一轮结束
      burst_left_ = 0;
      last_hit_t_ = now; //第二击窗口从命中确认起算
      stable_cnt_ = 0; //换片后重新攒平稳计数
    }
    if (pending_shot_ && burst_left_ == 0 &&
        (now - last_fire_t_) > prof_.miss_timeout) {
      pending_shot_ = false; //超时未确认，判脱靶
      misses_++;
    }
    //起一轮连发
    if (burst_left_ == 0 && !pending_shot_ &&
        (now - last_fire_t_) > prof_.fire_gap_time) {
      pending_shot_ = true;
      burst_left_ = std::max(1, prof_.fire_burst);
    }
    //第 1 发等 fire_gap_time，之后用 burst_gap
    const int fired_in_burst = std::max(1, prof_.fire_burst) - burst_left_;
    const double gap = (fired_in_burst > 0) ? prof_.burst_gap : prof_.fire_gap_time;

    if (burst_left_ > 0 && (now - last_fire_t_) > gap) {
      //稳定门(大符)：预测角不稳就不开枪，避免重捕获后相位跳变的坏帧
      const bool in_second = (now - last_hit_t_) < prof_.second_window_s;
      const bool stable_enough = prof_.second_gate_deg <= 0.0 ||
                                 stable_cnt_ >= prof_.second_gate_frames;
      if (!stable_enough) {
        static thread_local char buf2[48];
        std::snprintf(buf2, sizeof(buf2), in_second ? "第二击等稳(%d/%d帧)"
                                                    : "等稳(%d/%d帧)",
                      stable_cnt_, prof_.second_gate_frames);
        r.block = buf2;
      } else {
        r.shoot = true;
        burst_left_--;
        last_fire_t_ = now;
        shots_++;
        r.block = "";
      }
    } else if (pending_shot_) {
      static thread_local char buf[32];
      std::snprintf(buf, sizeof(buf), "连发中(剩%d)", burst_left_);
      r.block = burst_left_ ? buf : "等命中确认";
    } else {
      r.block = "节拍等待";
    }
  }

  RuneFireProfile prof_{};

  double last_yaw_ = 0.0, last_pitch_ = 0.0;
  double last_fire_t_ = -1.0;
  double last_hit_t_ = -1e9; //最近一次命中确认，第二击窗口起点
  int    stable_cnt_ = 0; //连续平稳帧数
  double last_dy_ = 0.0, last_dp_ = 0.0; //上一帧的预测角变化量
  int    mistake_count_ = 0;
  bool   switch_fanblade_ = false;
  bool   last_control_ = false;
  bool   pending_shot_ = false;
  int    burst_left_ = 0;
  int    shots_ = 0, hits_ = 0, misses_ = 0;
};
