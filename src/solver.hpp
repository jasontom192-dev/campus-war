#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <vector>
#include "types.hpp"

struct Box3D {
  std::vector<cv::Point3f> corners;
};

class Solver {
public:
  Solver(const cv::Mat & K, const cv::Mat & D);

  //更新相机内参，ROS 节点每帧从 camera_info 获取
  void set_intrinsics(const cv::Mat & K, const cv::Mat & D) { K_ = K; D_ = D; }

  //设置相机系←世界系旋转，用于 project_world()；默认单位阵
  void set_frame_rotation(const Eigen::Matrix3d & cam_from_world) {
    cam_from_world_ = cam_from_world;
  }

  void solve(Armor & a);

  Box3D left_lightbar(ArmorType t) const;
  Box3D right_lightbar(ArmorType t) const;
  Box3D digit_box(ArmorType t) const;

  std::vector<cv::Point2f> project(
    const std::vector<cv::Point3f> & pts3d,
    const cv::Vec3d & rvec, const cv::Vec3d & tvec) const;

  cv::Point2f project_world(const Eigen::Vector3d & p) const;

  static std::vector<cv::Point3f> pnp_points(ArmorType t);

private:
  //扫描 yaw 并重投影，消除平面板 PnP 的二义性
  void optimize_yaw(Armor & a) const;

  cv::Mat K_;
  cv::Mat D_;

  //相机系←世界系旋转，默认单位阵
  Eigen::Matrix3d cam_from_world_ = Eigen::Matrix3d::Identity();

  //装甲板尺寸，单位米；尺寸误差会直接放大 PnP 距离误差
  static constexpr float HW_BIG   = 0.115f; //大板半宽
  static constexpr float HW_SMALL = 0.0675f; //小板半宽
  static constexpr float HH_LB    = 0.028f; //灯条半高
  static constexpr float HW_LB    = 0.0055f; //灯条半宽
  static constexpr float HH_ARMOR = 0.063f; //数字区半高
};
