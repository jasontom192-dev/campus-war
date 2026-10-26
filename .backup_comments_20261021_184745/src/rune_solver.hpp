#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <array>
#include <string>
#include <vector>

#include "rune_detector.hpp"   // RuneBlade

// ============================================================================
// RuneSolver —— 符叶位姿解算（照 tdt-campus-test 的 solver.py 移植）
//
// 干三件事：
//   ① solvePnP(IPPE)：符叶四角（5 点里的 0/1/3/4）→ buff 系位姿
//   ② 角点排列自标定：26 模型四角的环序和 sp_vision 模型点环序【相反】，
//      直接对映会解出镜像位姿（重投影误差一样是 0，判断不了）。
//      做法：8 个候选排列各解一次，取重投影最小且法线朝相机的那个，锁定复用。
//   ③ 相机系 → 云台系 → 世界系：R_gimbal2world 由同帧云台角构造（rune_geometry.hpp）
//
// ★ 输出全部在【spv 世界系】(x 右, y 前, z 上)。发给游戏前还要 to_game_angles()
//   换成游戏角度（yaw 以 Unity +Z 为 0、顺时针为正；pitch 向下为正）——
//   这一步在节点/瞄准器里做，本类不管。
// ★ buff 系约定：x = 符面法线，z = 叶片径向（叶心在 (0,0,0.7)），原点在机关中心。
// ============================================================================

// 解算结果（世界系）
struct RuneSolution {
  bool valid = false;

  Eigen::Vector3d xyz;          // 机关中心位置（米）
  Eigen::Vector3d ypd;          // 机关中心球坐标 (yaw, pitch, distance)
  Eigen::Vector3d ypr;          // 符面姿态欧拉角（zyx 分解）
  Eigen::Vector3d blade_xyz;    // 叶心（= R 标中心，buff 系 (0,0,0.7)）位置
  Eigen::Vector3d blade_ypd;    // 叶心球坐标 ← EKF 的非线性观测要的就是它
  Eigen::Matrix3d R_buff2world;

  double reproj = 0.0;          // 重投影误差（px）：解得好不好一眼看这个
  std::vector<int> corner_order;  // 本次用的四角排列（26 模型的下标）
};

class RuneSolver {
public:
  RuneSolver(const cv::Mat & K, const cv::Mat & D);

  // ROS 节点用：内参逐帧来自 camera_info（规则 7.1），会变
  void set_intrinsics(const cv::Mat & K, const cv::Mat & D) { K_ = K; D_ = D; }

  // 解一片符叶。R_gimbal2world 由同帧云台世界角构造（rune::R_gimbal2world）。
  // 自标定还没锁定、且重投影太差时返回 valid=false（调用方当丢帧处理）。
  RuneSolution solve(const RuneBlade & blade, const Eigen::Matrix3d & R_gimbal2world);

  // 把【世界系】里一点投回像素（画瞄点/预测箭头用）。
  // 相机与云台同轴（外参单位阵、零平移），所以只要 R_gimbal2world。
  // 点在相机后面时返回 {-1,-1}。
  cv::Point2f project_world(const Eigen::Vector3d & p_world,
                            const Eigen::Matrix3d & R_gimbal2world) const;

  double last_reproj() const { return last_reproj_; }
  bool   calibrated()  const { return !locked_.empty(); }

private:
  // 按排列把 5 点检测重排成 PnP 要的 4 点（顺序对应 OBJECT_POINTS 的 0..3）
  std::vector<cv::Point2f> order_pts(const RuneBlade & blade,
                                     const std::array<int, 4> & perm) const;

  bool solve_pnp(const std::vector<cv::Point2f> & pts4,
                 cv::Mat & rvec, cv::Mat & tvec) const;

  // 重投影打分：把 buff 系叶心 (0,0,0.7) 和机关中心 (0,0,0) 投回图像，
  // 与"检测到的四角均值"和"第 2 点"比距离（和 Python 版同一个打分口径）
  double reproj_err(const cv::Mat & rvec, const cv::Mat & tvec,
                    const RuneBlade & blade) const;

  // 8 个候选排列里挑一个（正面优先 + 重投影最小），误差够小就锁定
  std::array<int, 4> corner_order_for(const RuneBlade & blade) const;

  cv::Mat K_, D_;

  // 自标定结果：8 个候选里锁定的那个；空 = 还没锁定
  mutable std::array<int, 4> locked_{};
  mutable bool locked_valid_ = false;
  mutable double last_reproj_ = -1.0;
};
