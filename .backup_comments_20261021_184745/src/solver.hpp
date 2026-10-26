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

  // ROS 节点用：内参来自每帧的 camera_info，会变，所以要能换。
  // 离线跑视频用不上（构造时定死）。
  void set_intrinsics(const cv::Mat & K, const cv::Mat & D) { K_ = K; D_ = D; }

  // ROS 节点用：Tracker/状态搬到世界系后，状态里的点要转回相机系才能投影成
  // 像素。这里存【相机系 ← 世界系】的旋转，只作用在 project_world() 上。
  // 不设就是单位阵 —— 离线跑视频那条路完全不受影响。
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
  // 用重投影误差暴力扫 yaw，消掉平面板 PnP 的二义性（见 solver.cpp）
  void optimize_yaw(Armor & a) const;

  cv::Mat K_;
  cv::Mat D_;

  // 见 set_frame_rotation()：相机系 ← 世界系，默认单位阵
  Eigen::Matrix3d cam_from_world_ = Eigen::Matrix3d::Identity();

  // ---- 装甲板的物理尺寸（米），板坐标系，见 solver.cpp 文件头 ----
  // ★ 这几个常数直接决定 PnP 解出的【距离】准不准：尺寸给大一倍，距离就
  //   会解成两倍。仓库测试视频验收时专门核对过大小板 —— 按小板算 2.26 m，
  //   按大板算会变成 4.1 m，与画面里机器人的成像高度明显不符。
  static constexpr float HW_BIG   = 0.115f;   // 大装甲板半宽（整宽 230mm）
  static constexpr float HW_SMALL = 0.0675f;  // 小装甲板半宽（整宽 135mm）
  static constexpr float HH_LB    = 0.028f;   // 灯条半高（整高 56mm）
  static constexpr float HW_LB    = 0.0055f;  // 灯条半宽（整宽 11mm）
  static constexpr float HH_ARMOR = 0.063f;   // 数字区半高（比灯条略矮）
};
