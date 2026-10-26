#pragma once

// ============================================================================
// rune_aimer.hpp —— 打符的瞄准 + 开火决策（照 tdt-campus-test 的 aimer.py 移植）
//
// 流程（和 sp_vision buff_aimer.cpp 同源）：
//   ① target.predict(检测延时 + predict_time + 云台前馈)  → 瞄点挪到"云台该到位"的时刻
//   ② 解一次弹道得 fly_time
//   ③ target.predict(fly_time) 再解一次 → 两次飞时差 < 0.01 s 才算收敛
//   ④ 输出 spv 世界系 yaw/pitch（调用方再 to_game_angles 换成游戏角度）
//
// 判稳（sp_vision 原样）：
//   相邻帧预测角变化 < jump_thres(5°) → 跟随、允许开火
//   变化 > 5° → 立即停火 + 冻结云台，跳变计数 +1；连跳 4 帧 → 认了这是真换叶，
//   接受新相位、恢复控制。
//
// 开火节奏（confirm 模式，和小符/大符两套参数）：
//   打一发 → 等"命中确认"（小符=新叶亮起；大符=目标从检出里消失）→ 再打下一片；
//   超 miss_timeout 还没确认 = 判脱靶，允许补枪。
//   ★ 大符用 fire_burst=1：**一片一发、绝不连发** —— 连发会砸到刚打中的板上，
//     规则里那是"误击"（从第 1 回合重新累计）。
//
// ★ 为什么是模板：SmallTarget / BigTarget 的接口一样（predict / aim_point_world /
//   is_unsolved / ekf_x），共用这一份实现。**传进来的必须是副本** —— 本类会连续
//   predict() 原地改写滤波器状态（Python 版是显式 clone()，C++ 里拷贝构造即可）。
// ============================================================================

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "rune_geometry.hpp"
#include "rune_trajectory.hpp"

// 一套开火节奏参数（小符 / 大符各一份，从 config 读）
struct RuneFireProfile {
  double fire_gap_time = 0.52;   // 两轮之间的最小间隔
  double burst_gap = 0.45;       // 一轮里的发间隔
  int    fire_burst = 3;         // 一轮最多几发（大符=1）
  double miss_timeout = 1.0;     // 打完这么久没确认 = 脱靶，允许补枪
  double predict_time = 0.10;    // 额外提前量
  double gimbal_ff_s = 0.0;      // 云台滞后前馈
  double pitch_extra_deg = 0.0;  // 额外的抬枪角（度）。弹道解算按真空抛物线给
                                 // 的重力补偿，游戏实际弹道若略有出入（打低了）
                                 // 就用这个补，实机校：6 m 处 0.1° ≈ 1 cm 高。
  double yaw_extra_deg = 0.0;    // 额外的水平偏置（度）：偏航零点/枪口左侧视差
                                 // 的实机校准。实机"整团稳定偏一侧"（不随叶片
                                 // 转位变方向）就是它。6 m 处 0.1° ≈ 1 cm 横移；
                                 // 正值 = 往（云台顺时针的）右偏。

  // ---- 第二击纪律（大符专用；小符保持 0=关闭）----
  // 规则：大符每回合"首块有效命中后，第二块允许在 1 秒内命中，超时按该回合
  // 通过并记录已命中环数"。所以第二击是可选的加分项——第二块刚出现时相位
  // 刚换片跳变、滤波器还在恢复，抢着打既拉低平均环数、又可能蹭到刚打中的板
  // （误击 = 从第 1 回合重新累计）。改成：命中确认后的窗口内，只有"预测角
  // 连续平稳"才放第二击；不稳就等，窗口过了不补枪（超时由游戏自己算通过）。
  double second_gate_deg = 0.0;      // 平稳判据：相邻帧预测角变化的二阶差分
                                     // （度/帧²）。0 = 关闭这道门（小符）
  int    second_gate_frames = 6;     // 连续平稳这么多帧才允许放
  double second_window_s = 0.85;     // 第二击窗口（规则 1s，留检测延迟余量）
};

class RuneAimer {
public:
  struct Result {
    bool control = false;        // 判稳通过、云台可以跟
    bool shoot = false;          // 射击许可
    double yaw = 0.0;            // spv 世界系方位角（rad）
    double pitch = 0.0;          // spv 世界系发射仰角（rad，抬头为正）
    double fly_time = 0.0;       // 弹道解出的飞行时间（s）—— 日志拆提前量用
    const char * block = "";     // 没开火的原因（日志/窗口直接用）
  };

  RuneAimer() = default;

  // 换开火节奏（不重置统计）
  void set_profile(const RuneFireProfile & p) { prof_ = p; }
  const RuneFireProfile & profile() const { return prof_; }

  void reset() {
    mistake_count_ = 0;
    switch_fanblade_ = false;
    last_fire_t_ = -1.0;
    pending_shot_ = false;
    burst_left_ = 0;
    last_control_ = false;
    last_hit_t_ = -1e9;      // 第二击窗口计时
    stable_cnt_ = 0;
    last_dy_ = last_dp_ = 0.0;
  }

  // 进打符模式时把"上一条指令"对齐到云台当前姿态（spv 系，rad）。
  // 不这么做的话，第一帧的预测角相对 0 会是个大跳变，判稳直接冻结几帧。
  void set_last_angles(double yaw, double pitch) {
    last_yaw_ = yaw;
    last_pitch_ = pitch;
  }

  // 主入口。detect_gap = 从图像时间戳到现在的秒数（帧龄）；now = 单调秒。
  // hit_confirmed = 这一帧的外部命中证据（小符：新叶亮起；大符：目标消失）。
  template <class TargetT>
  Result aim(TargetT target_copy, double detect_gap, double bullet_speed,
             double now, bool hit_confirmed) {
    Result r;
    if (bullet_speed < 10.0) bullet_speed = 24.0;   // 初速没填对时的兜底
    if (last_fire_t_ < 0.0) last_fire_t_ = now;

    r.yaw = last_yaw_;
    r.pitch = last_pitch_;

    // 检测延时 + 提前量 + 云台前馈：预测到"弹丸该出发/云台该到位"的时刻
    const double future = detect_gap + prof_.predict_time + prof_.gimbal_ff_s;
    double yaw = 0.0, pitch = 0.0, fly = 0.0;
    const bool solved = send_angle(target_copy, future, bullet_speed, yaw, pitch, fly);

    if (solved) {
      r.fly_time = fly;
      const double thr = jump_thres_deg * M_PI / 180.0;
      const double dy = std::abs(last_yaw_ - yaw);
      const double dp = std::abs(last_pitch_ - pitch);
      // 第二击纪律：预测角"平稳度" = 相邻帧变化量的二阶差分（jerk）。
      // 正弦转动本身的二阶差分只有 ~0.006°/帧（可忽略不计）；换片跳变后的
      // 收敛期、观测修正落进来的那几帧会跳到零点几~几度 —— 正好当判据。
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
        stable_cnt_ = prof_.second_gate_frames;   // 这道门关着 → 恒"稳"
      }
      if (dy > thr || dp > thr) {
        switch_fanblade_ = true;
        mistake_count_++;
        r.control = false;
        if (mistake_count_ > mistake_to_switch) {    // 连跳够了：认了，真换叶
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
      last_fire_t_ = now;                    // 冻结期间持续压住节拍
    } else if (!r.control) {
      r.shoot = false;                       // 瞄点不稳，绝不开火
      r.block = "锁定中";
    } else {
      confirm_fire(now, hit_confirmed, r);
    }

    last_control_ = r.control;
    return r;
  }

  // ---- 统计 / 状态（日志、窗口用）----
  int    shots() const { return shots_; }
  int    hits() const { return hits_; }
  int    misses() const { return misses_; }
  int    mistake_count() const { return mistake_count_; }
  bool   pending() const { return pending_shot_; }
  double last_yaw() const { return last_yaw_; }
  double last_pitch() const { return last_pitch_; }

  double jump_thres_deg = 5.0;       // 判稳阈值（度）
  int    mistake_to_switch = 3;      // 连跳几帧算真换叶

private:
  // 两点迭代解出世界系 yaw/pitch；无解返回 false。
  // target 是副本，可以放心 predict（会原地改状态）
  template <class TargetT>
  bool send_angle(TargetT & target, double predict_time, double bullet_speed,
                  double & yaw, double & pitch, double & fly) const {
    if (target.is_unsolved()) return false;

    target.predict(predict_time);
    Eigen::Vector3d aim_w = target.aim_point_world();
    double d = std::hypot(aim_w(0), aim_w(1));
    RuneTrajectory traj0(bullet_speed, d, aim_w(2));
    if (!traj0.ok) return false;

    // 按第一段飞行时间再推一次（两点迭代）
    target.predict(traj0.fly_time);
    aim_w = target.aim_point_world();
    d = std::hypot(aim_w(0), aim_w(1));
    RuneTrajectory traj1(bullet_speed, d, aim_w(2));
    if (!traj1.ok) return false;
    if (std::abs(traj1.fly_time - traj0.fly_time) > 0.01) return false;   // 不收敛

    yaw = std::atan2(aim_w(1), aim_w(0));    // spv 球坐标：atan2(前, 右)
    // 真空抛物线解 + 实机抬高补偿（pitch_extra_deg）+ 实机水平偏置（yaw_extra_deg）
    pitch = traj1.pitch + prof_.pitch_extra_deg * M_PI / 180.0;
    yaw += prof_.yaw_extra_deg * M_PI / 180.0;
    fly = traj1.fly_time;
    return true;
  }

  // confirm 节奏：打一发 → 等命中确认 → 再打；超时判脱靶允许补枪
  void confirm_fire(double now, bool hit_confirmed, Result & r) {
    if (hit_confirmed) {
      if (pending_shot_) hits_++;
      pending_shot_ = false;      // 打中了：这一轮结束，下一片重新起一轮
      burst_left_ = 0;
      last_hit_t_ = now;          // 第二击窗口从"命中确认"起算
      stable_cnt_ = 0;            // 换片后重新攒"平稳"计数
    }
    if (pending_shot_ && burst_left_ == 0 &&
        (now - last_fire_t_) > prof_.miss_timeout) {
      pending_shot_ = false;      // 一轮打完还没等到确认 = 脱靶
      misses_++;
    }
    // 起一轮连发
    if (burst_left_ == 0 && !pending_shot_ &&
        (now - last_fire_t_) > prof_.fire_gap_time) {
      pending_shot_ = true;
      burst_left_ = std::max(1, prof_.fire_burst);
    }
    // 一轮里的第 1 发用 fire_gap_time（和上一轮的间隔），之后几发用 burst_gap
    const int fired_in_burst = std::max(1, prof_.fire_burst) - burst_left_;
    const double gap = (fired_in_burst > 0) ? prof_.burst_gap : prof_.fire_gap_time;

    if (burst_left_ > 0 && (now - last_fire_t_) > gap) {
      // ★ 稳定放行门（大符）：预测角不稳就不开枪 —— 不止第二击窗口内。
      //   实测：丢检重捕获时拾取到别的叶，相位跳 ±72/144°，滤波器会被抽到
      //   300+°/s，之后二三十帧瞄点是垃圾；而"连跳 4 帧就认了"的机制恰好
      //   在这段垃圾期恢复开枪 → 那些发必散（"本来就不稳定"的主要来源）。
      //   小符 second_gate_deg=0，恒放行（小符行为不受影响）。
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
  double last_hit_t_ = -1e9;                 // 最近一次命中确认（第二击窗口起点）
  int    stable_cnt_ = 0;                    // 连续"平稳"帧数（二阶差分判据）
  double last_dy_ = 0.0, last_dp_ = 0.0;     // 上一帧的预测角变化量
  int    mistake_count_ = 0;
  bool   switch_fanblade_ = false;
  bool   last_control_ = false;
  bool   pending_shot_ = false;
  int    burst_left_ = 0;
  int    shots_ = 0, hits_ = 0, misses_ = 0;
};
