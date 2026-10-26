#pragma once

//弹道解算：由初速、水平距离和高度差求发射仰角与飞行时间
//真空抛物线、无阻力；双解取飞行时间短的(低伸弹道)
//pitch 为抬头正(rad)，重力常数 9.7833

#include <cmath>

inline constexpr double kRuneG = 9.7833; //重力常数

struct RuneTrajectory {
  bool   ok = false; //false=该初速够不着(判别式<0)
  double pitch = 0.0; //发射仰角(rad)，抬头为正
  double fly_time = 0.0; //飞行时间(s)

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
