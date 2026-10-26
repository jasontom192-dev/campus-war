#pragma once

// ============================================================================
// rune_trajectory.hpp —— 弹道解算（照 tdt-campus-test 的 trajectory.py 移植，
// 源头是 sp_vision_25 tools/trajectory.cpp）
//
// 解"以初速 v0 打中水平距离 d、竖直高度 h 的点"所需的【发射仰角】和【飞行时间】。
// 真空抛物线、无空气阻力。二次方程有两个根（低伸 / 高抛），取【飞行时间短的】
// —— 打符要的是快、直，高抛弹道没有意义。
//
// ★ 车那套（aimer.hpp 的 solve_traj）是同一个物理，但那份的 pitch 是相对水平面的
//   发射仰角、g=9.7833；这里连常数都对齐（9.7833），照着 sp_vision 的公式写，
//   两个模型（小符/大符）和离线工具共用这一份。
// ============================================================================

#include <cmath>

inline constexpr double kRuneG = 9.7833;   // 和车的 Aimer 用同一个重力常数

struct RuneTrajectory {
  bool   ok = false;        // false = 这个初速够不着（判别式 < 0）
  double pitch = 0.0;       // 发射仰角（rad），抬头为正
  double fly_time = 0.0;    // 飞行时间（s）

  RuneTrajectory() = default;

  RuneTrajectory(double v0, double d, double h) { solve(v0, d, h); }

  void solve(double v0, double d, double h) {
    ok = false;
    pitch = 0.0;
    fly_time = 0.0;

    if (v0 <= 1e-6) return;
    const double a = kRuneG * d * d / (2.0 * v0 * v0);
    const double b = -d;
    const double c = a + h;
    const double delta = b * b - 4.0 * a * c;
    if (delta < 0.0) return;

    const double tan1 = (-b + std::sqrt(delta)) / (2.0 * a);
    const double tan2 = (-b - std::sqrt(delta)) / (2.0 * a);
    const double p1 = std::atan(tan1);
    const double p2 = std::atan(tan2);
    const double t1 = d / (v0 * std::cos(p1));
    const double t2 = d / (v0 * std::cos(p2));

    if (t1 < t2) { pitch = p1; fly_time = t1; }
    else         { pitch = p2; fly_time = t2; }
    ok = true;
  }
};
