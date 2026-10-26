#pragma once

#include <Eigen/Dense>
#include <vector>
#include <opencv2/opencv.hpp>
#include "ekf.hpp"
#include "types.hpp"

//Target：单个目标的 EKF 模型，状态在 pseudo 系（右/前/上），pseudo=(cam.x,cam.z,-cam.y)
//车 11 维 [x,vx,y,vy,z,vz,yaw,omega,r,dr,dh]；r 为板半径，dr/dh 为侧板半径/高度增量
//前哨站 10 维 [x,vx,y,vy,z,vz,yaw,v_yaw,dz1,dz2]；观测为 [yaw,pitch,dist,face_yaw]，相机系

inline constexpr double kOutpostR    = 0.275; //旋转半径，直径 550mm
inline constexpr double kOutpostDz1  = 0.102; //1 号板相对 0 号板的高度差
inline constexpr double kOutpostDz2  = 0.204; //2 号板相对 0 号板的高度差
inline constexpr double kOutpostVyaw = 0.8 * M_PI; //转速大小 0.8π rad/s，方向由投票决定
//dz1/dz2 的过程噪声尺度，保留缓慢修正通道
inline constexpr double kOutpostQdz   = 10.0;
inline constexpr int    kOutpostDirVotes = 30; //判定方向需要的样本数

//板法线与自身方位角的最大偏差，超过此值的观测不进入 EKF
inline constexpr double kOutpostFacingGate = 35.0 * M_PI / 180.0;

class Target {
public:
  ArmorName name = not_armor; //兵种
  ArmorType armor_type = small; //大小装甲板
  int armor_num_ = 4; //板数：步兵 4，前哨站/基地 3
  bool is_outpost_ = false; //true 时使用 10 维前哨站模型

  //0=未定，+1=逆时针，-1=顺时针；定向后 v_yaw 固定为 ±0.8π
  int outpost_dir_ = 0;
  int dir_ccw_ = 0, dir_cw_ = 0; //方向投票的票数

  //当前已攒的方向投票样本数
  int dir_votes() const { return static_cast<int>(dir_data_.size()); }
  bool jumped = false; //最近一次关联是否跳板，当前只写不读
  int last_id = 0; //上一帧关联的板编号，当前只写不读
  ExtendedKalmanFilter ekf_;

  Target() = default;

  //用首次观测初始化模型，反推车中心；radius 为车中心到板的水平距离
  void init(const Armor & armor, double radius, int armor_num,
            const Eigen::VectorXd & P0_dig);

  //按匀速模型预测 dt 秒，并更新协方差
  void predict(double dt);

  //带入数据关联的 EKF 更新
  void update(const Eigen::Vector4d & z_obs, const Eigen::Vector4d & R_dig);

  //按当前状态返回所有板的 [x,y,z,yaw]，pseudo 系
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  //前哨站方向投票，样本攒够后锁定转向和 v_yaw
  void vote_direction(double orient_yaw);

  //半径或前哨站高度/转速超出合理范围时返回 true
  bool diverged() const;

  //相机系坐标和板法线角转为 [yaw,pitch,dist,face_yaw]
  static Eigen::Vector4d camera_to_obs(const Eigen::Vector3d & cam_xyz, double face_yaw);

  //pseudo 系 → 相机系
  static Eigen::Vector3d pseudo_to_camera(const Eigen::Vector3d & p);

  //观测模型：给定状态，返回第 id 块板在 pseudo 系的位置
  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;

private:

  //前哨站方向投票的观测缓存
  std::vector<double> dir_data_;

  //h(x) 的雅可比，车模型 4×11，前哨站 4×10
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;

  //直角坐标转球坐标 [yaw,pitch,dist]
  static Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz);

  //xyz2ypd 的 3×3 雅可比
  static Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d & xyz);
};
