#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <array>
#include <string>
#include <vector>

#include "rune_detector.hpp" //RuneBlade

//符叶位姿解算：四角 PnP、角点排列自标定、相机/云台/世界换系
//输出为 spv 世界系(x 右, y 前, z 上)，转游戏角由调用方做
//buff 系：x=符面法线，z=叶片径向，叶心 (0,0,0.7) m，原点在机关中心

//解算结果（世界系）
struct RuneSolution {
  bool valid = false;

  Eigen::Vector3d xyz; //机关中心位置(m)
  Eigen::Vector3d ypd; //机关中心球坐标(yaw,pitch,distance)
  Eigen::Vector3d ypr; //符面姿态的 ZYX 欧拉角
  Eigen::Vector3d blade_xyz; //叶心位置，buff 系 (0,0,0.7)
  Eigen::Vector3d blade_ypd; //叶心世界球坐标，EKF 的 z2 量测
  Eigen::Matrix3d R_buff2world;

  double reproj = 0.0; //重投影误差(px)
  std::vector<int> corner_order; //本次四角排列(26 模型下标)
};

class RuneSolver {
public:
  RuneSolver(const cv::Mat & K, const cv::Mat & D);

  //ROS 节点逐帧更新内参(K/D)
  void set_intrinsics(const cv::Mat & K, const cv::Mat & D) { K_ = K; D_ = D; }

  //R_gimbal2world 需由同帧云台角构造；未标定且重投影>60px 时 valid=false
  RuneSolution solve(const RuneBlade & blade, const Eigen::Matrix3d & R_gimbal2world);

  //世界系点投回像素；相机与云台同轴，点在相机后方返回(-1,-1)
  cv::Point2f project_world(const Eigen::Vector3d & p_world,
                            const Eigen::Matrix3d & R_gimbal2world) const;

  double last_reproj() const { return last_reproj_; }
  bool   calibrated()  const { return !locked_.empty(); }

private:
  //按排列重排成 PnP 的 4 点，perm 索引 kCornerIdx
  std::vector<cv::Point2f> order_pts(const RuneBlade & blade,
                                     const std::array<int, 4> & perm) const;

  bool solve_pnp(const std::vector<cv::Point2f> & pts4,
                 cv::Mat & rvec, cv::Mat & tvec) const;

  //打分：叶心对四角均值 + 0.5×机关中心对第 2 点，单位 px
  double reproj_err(const cv::Mat & rvec, const cv::Mat & tvec,
                    const RuneBlade & blade) const;

  //8 个排列中取法线朝相机且重投影最小者，误差<60px 后锁定
  std::array<int, 4> corner_order_for(const RuneBlade & blade) const;

  cv::Mat K_, D_;

  //自标定锁定的四角排列
  mutable std::array<int, 4> locked_{};
  mutable bool locked_valid_ = false;
  mutable double last_reproj_ = -1.0;
};
