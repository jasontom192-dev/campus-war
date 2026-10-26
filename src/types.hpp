#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>

//共用的枚举和装甲板结构体
//ArmorName 的整数值同时是 Tracker 的 track key

//模型 color_id 0=蓝、1=红（白/紫在 detector 中丢弃）
enum Color { red = 0, blue = 1};
const std::vector<std::string> COLORS = {"red", "blue"};

//顺序对应模型 armor_id 0..8，映射见 detector.cpp
enum ArmorName {
  one = 0, two, three, four, five, sentry, outpost, base, not_armor
};
const std::vector<std::string> ARMOR_NAMES = {
  "one", "two", "three", "four", "five",
  "sentry", "outpost", "base", "not_armor"};

//只决定 PnP 的 3D 尺寸：大板宽 230mm，小板宽 135mm
enum ArmorType { big = 0, small = 1 };
const std::vector<std::string> ARMOR_TYPES = {"big", "small"};

//Detector 只填 2D 字段，Solver::solve() 填 3D 字段；tracker.update() 前必须先解算
struct Armor {

  float confidence = 0; //网络置信度，0~1
  cv::Rect box; //四角点外接矩形，仅用于显示
  std::vector<cv::Point2f> points; //左上、右上、右下、左下，顺序与 pnp_points 一致
  ArmorName name = not_armor;
  ArmorType type = small;
  Color color = blue;

  cv::Vec3d rvec, tvec; //PnP 结果，相机系（右/下/前），tvec 为板中心
  Eigen::Vector3d xyz_in_camera = {0, 0, 0}; //板中心在相机系，ROS 节点会原地改成世界系
  double yaw = 0; //板法线方位角 atan2(nz,nx)，方向为板→车中心
};

//数值越小越优先，同优先级按距离由近到远
inline int get_armor_priority(ArmorName name) {
  switch (name) {
    case outpost: return 1;
    case base:    return 2;
    case three:
    case four:
    case five:    return 3;
    case sentry:  return 4;
    case one:     return 5;
    case two:     return 6;
    default:      return 99;
  }
}

//折回 (-π,π]（-π→+π）；先 fmod 防巨值，±inf 返回 NaN，由调用方判无效
inline double limit_rad(double a) {
  a = std::fmod(a, 2.0 * CV_PI);
  while (a > CV_PI) a -= 2 * CV_PI;
  while (a <= -CV_PI) a += 2 * CV_PI;
  return a;
}
