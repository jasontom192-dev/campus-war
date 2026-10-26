#pragma once

//打符用的数学与坐标换算，集中放 spv 世界系约定
//spv 世界系(x 右, y 前, z 上)，yaw=atan2(y,x)
//游戏角 yaw 从 Unity +Z 起顺时针为正、pitch 向下为正；game_yaw=π/2-spv_yaw，game_pitch=-spv_pitch

#include <Eigen/Dense>
#include <cmath>

#include "types.hpp" //limit_rad

namespace rune {

//直角坐标 -> 球坐标(yaw,pitch,distance)
inline Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz) {
  const double x = xyz(0), y = xyz(1), z = xyz(2);
  return {std::atan2(y, x), std::atan2(z, std::hypot(x, y)),
          std::sqrt(x * x + y * y + z * z)};
}

inline Eigen::Vector3d ypd2xyz(const Eigen::Vector3d & ypd) {
  const double yaw = ypd(0), pitch = ypd(1), d = ypd(2);
  return {d * std::cos(pitch) * std::cos(yaw),
          d * std::cos(pitch) * std::sin(yaw),
          d * std::sin(pitch)};
}

//xyz2ypd 对 xyz 的 3x3 雅可比，EKF 非线性观测用
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

//ypd2xyz 对 (yaw,pitch,distance) 的 3x3 雅可比
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

//ZYX 欧拉角(yaw,pitch,roll) -> 旋转矩阵
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

//旋转矩阵 -> ZYX 欧拉角，和上面互为逆运算
inline Eigen::Vector3d eulers_zyx(const Eigen::Matrix3d & R) {
  const double pitch = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
  double yaw, roll;
  if (std::abs(std::cos(pitch)) < 1e-9) { //万向锁：yaw/roll 只能定出一个和
    yaw  = std::atan2(-R(0, 1), R(1, 1));
    roll = 0.0;
  } else {
    yaw  = std::atan2(R(1, 0), R(0, 0));
    roll = std::atan2(R(2, 1), R(2, 2));
  }
  return {limit_rad(yaw), limit_rad(pitch), limit_rad(roll)};
}

//由云台世界角(游戏约定，度)构造 相机系(右/下/前) -> 世界系(右/前/上) 的旋转
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

//spv 角度(rad) -> 游戏角度(度)，yaw 折回(-π,π]
inline Eigen::Vector2d to_game_angles(double spv_yaw, double spv_pitch) {
  const double game_yaw   = limit_rad(M_PI / 2.0 - spv_yaw);
  const double game_pitch = -spv_pitch;
  return {game_yaw * 180.0 / M_PI, game_pitch * 180.0 / M_PI};
}

}
