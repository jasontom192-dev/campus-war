#pragma once

// ============================================================================
// rune_geometry.hpp —— 打符用的数学与坐标换算（照 tdt-campus-test 的 geometry.py 移植）
//
// 为什么单独一个头文件：这些是纯函数（无状态、不碰 ROS），Solver / Target /
// 可视化三边都要用；坐标系约定集中在一处，改错只有这一个地方。
//
// ★★ 坐标系约定和车那套【不同】，千万别混用 ★★
//
//   spv 世界系（打符用）= (x 右, y 前, z 上)，右手系；
//       它的 yaw = atan2(y, x) = atan2(前, 右)
//   游戏角度（规则 7.3）：yaw 以 Unity +Z 为 0、顺时针为正；pitch 向下为正
//   两者换算：game_yaw = π/2 − spv_yaw ；game_pitch = −spv_pitch
//
//   车的世界系见 target.hpp 顶部那张表（Unity map 系）。两套换算长得像，
//   但混用会让云台角差 90°（19 年 RM 就踩过这个坑）。
//
//   角度折回用 types.hpp 里的 limit_rad（和本文件同一套 (-π, π] 约定）。
// ============================================================================

#include <Eigen/Dense>
#include <cmath>

#include "types.hpp"    // limit_rad

namespace rune {

// 直角坐标 → 球坐标 (yaw, pitch, distance)
inline Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz) {
  const double x = xyz(0), y = xyz(1), z = xyz(2);
  return {std::atan2(y, x), std::atan2(z, std::hypot(x, y)),
          std::sqrt(x * x + y * y + z * z)};
}

// 球坐标 → 直角坐标
inline Eigen::Vector3d ypd2xyz(const Eigen::Vector3d & ypd) {
  const double yaw = ypd(0), pitch = ypd(1), d = ypd(2);
  return {d * std::cos(pitch) * std::cos(yaw),
          d * std::cos(pitch) * std::sin(yaw),
          d * std::sin(pitch)};
}

// xyz2ypd 对 xyz 的雅可比（3×3）—— 打符 EKF 的非线性观测用
inline Eigen::Matrix3d xyz2ypd_jacobian(const Eigen::Vector3d & xyz) {
  const double x = xyz(0), y = xyz(1), z = xyz(2);
  const double rxy2 = x * x + y * y;
  const double rxy  = std::sqrt(rxy2);
  const double r3   = std::pow(x * x + y * y + z * z, 0.5);
  const double k    = z * z / rxy2 + 1.0;

  Eigen::Matrix3d J;
  J << -y / rxy2,  x / rxy2,  0.0,
       -(x * z) / (k * std::pow(rxy2, 1.5)), -(y * z) / (k * std::pow(rxy2, 1.5)),
           1.0 / (k * rxy),
        x / r3,  y / r3,  z / r3;
  return J;
}

// ypd2xyz 对 (yaw, pitch, distance) 的雅可比（3×3）
inline Eigen::Matrix3d ypd2xyz_jacobian(const Eigen::Vector3d & ypd) {
  const double yaw = ypd(0), pitch = ypd(1), d = ypd(2);
  const double cy = std::cos(yaw),  sy = std::sin(yaw);
  const double cp = std::cos(pitch), sp = std::sin(pitch);

  Eigen::Matrix3d J;
  J << -d * cp * sy, -d * sp * cy, cp * cy,
        d * cp * cy, -d * sp * sy, cp * sy,
        0.0,          d * cp,      sp;
  return J;
}

// 欧拉角 (yaw, pitch, roll) → 旋转矩阵，ZYX 顺序（与 math_tools.cpp 逐项一致）
inline Eigen::Matrix3d rotation_matrix_zyx(const Eigen::Vector3d & ypr) {
  const double cy = std::cos(ypr(0)), sy = std::sin(ypr(0));
  const double cp = std::cos(ypr(1)), sp = std::sin(ypr(1));
  const double cr = std::cos(ypr(2)), sr = std::sin(ypr(2));

  Eigen::Matrix3d R;
  R << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
       sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
       -sp,     cp * sr,                cp * cr;
  return R;
}

// 旋转矩阵 → (yaw, pitch, roll)，ZYX 顺序（解析式，和上面互为逆运算）
inline Eigen::Vector3d eulers_zyx(const Eigen::Matrix3d & R) {
  const double pitch = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
  double yaw, roll;
  if (std::abs(std::cos(pitch)) < 1e-9) {          // 万向锁：yaw/roll 只能定出一个和
    yaw  = std::atan2(-R(0, 1), R(1, 1));
    roll = 0.0;
  } else {
    yaw  = std::atan2(R(1, 0), R(0, 0));
    roll = std::atan2(R(2, 1), R(2, 2));
  }
  return {limit_rad(yaw), limit_rad(pitch), limit_rad(roll)};
}

// 由云台世界光轴角构造 相机系(右/下/前) → 世界系(右/前/上) 的旋转矩阵。
// 游戏角度约定：yaw 从 Unity +Z 起算、顺时针为正；pitch 向下为正。
inline Eigen::Matrix3d R_gimbal2world(double yaw_deg, double pitch_deg) {
  const double psi = yaw_deg * M_PI / 180.0;
  const double th  = pitch_deg * M_PI / 180.0;

  Eigen::Vector3d z(std::sin(psi) * std::cos(th),
                    std::cos(psi) * std::cos(th),
                    -std::sin(th));
  Eigen::Vector3d x(std::cos(psi), -std::sin(psi), 0.0);
  x.normalize();
  Eigen::Vector3d y = z.cross(x);

  Eigen::Matrix3d R;
  R.col(0) = x;
  R.col(1) = y;
  R.col(2) = z;
  return R;
}

// spv 世界系角度 → 游戏角度（返回"度"）。与 R_gimbal2world 互逆的那一套换算。
inline Eigen::Vector2d to_game_angles(double spv_yaw, double spv_pitch) {
  const double game_yaw   = limit_rad(M_PI / 2.0 - spv_yaw);
  const double game_pitch = -spv_pitch;
  return {game_yaw * 180.0 / M_PI, game_pitch * 180.0 / M_PI};
}

}  // namespace rune
