#pragma once

#include <Eigen/Dense>
#include <vector>
#include <opencv2/opencv.hpp>
#include "ekf.hpp"
#include "types.hpp"

// ============================================================================
// Target —— 单个目标的 EKF 模型（整个项目最核心的类）
//
// 它不知道"装甲板检测"这回事，只维护一个抽象问题：
//
//     有一辆车，车身上绕着中心均匀装了 armor_num_ 块装甲板，
//     车在匀速平动、可能还在匀速自转（小陀螺），
//     我每帧只能看到其中 1~2 块板的位姿，
//     怎么估计出这辆车的完整状态？
//
// 答案是一个 11 维的 EKF。状态向量 x 的定义（target.cpp:63-67）：
//
//   下标:  [0]  [1]  [2]  [3]  [4]  [5]  [6]   [7]    [8] [9]  [10]
//   含义:   x   vx    y   vy    z   vz   yaw  omega   r   dr   dh
//          └──┬──┘  └──┬──┘  └──┬──┘  └─┬─┘  └─┬──┘ └────┬────┘
//          车中心位置  车中心速度  朝向 角速度   半径相关参数
//   ★ 注意：这三个位置分量（x/y/z）不是"前后/左右/上下"，而是 pseudo 系
//     下的 右(x) / 前(y) / 上(z)，见下面 pseudo 系轴的定义。
//
//   ★ 注意：这里的坐标系不是相机系，是 pseudo 系（"假想世界系"）：
//       pseudo x =  相机 x  （向右）
//       pseudo y =  相机 z  （向前）
//       pseudo z = -相机 y  （向上）   ← 竖直方向
//     换算是 target.cpp:13-18 和 pseudo_to_camera()
//     这么绕是为了把竖直轴单独拎成一个分量（p(2)）、让水平面落在前两个
//     下标上，yaw 才好解释成绕竖直轴的转角（相机系里竖直是 y、深度是 z，
//     直接拿状态来排就会交错）。
//
//   ★ r 是"车中心到装甲板在水平面内的距离"。为什么要估它？因为不同兵种
//     半径不同（步兵 0.20、前哨站 0.2765……），而且实际值总有偏差，
//     交给滤波器自己收敛比写死常量鲁棒。
//     dr 是【侧面装甲板】相对正面装甲板的半径增量，dh 是它们的高度差。
//     4 块板正交布局时，左右两块（id 1、3）在切向上会比前后两块"更远/更高"，
//     这两个量就是描述这个不对称性的。
//
// 观测向量 z 是 4 维（在【相机系】下）：
//       [yaw, pitch, dist, face_yaw]
//   前三个是指向装甲板的球坐标，第四个是装甲板法线的朝向角。
//   见 target.cpp:456 camera_to_obs()
// ============================================================================

// ============================================================================
// 前哨站专用模型（参考 auto_aim_rps26 的 outpost_model）
//
// 前哨站和普通车的几何完全不同，所以单独一套状态（10 维）：
//
//   [0]  [1]  [2]  [3]  [4]  [5]  [6]   [7]    [8]  [9]
//    x   vx    y   vy    z   vz   yaw   v_yaw  dz1  dz2
//
//   - x/y 是水平面位置、z 是高度（和车模型一样，pseudo 系）
//   - yaw 是前哨站的转角：第 id 块板的方位角 = yaw + id * 2π/3
//   - v_yaw 是转速，**只有 ±0.8π 两种**（顺/逆时针），靠投票定下来
//   - dz1/dz2 是 1、2 号板相对 0 号板的高度增量（三块板一样高就不需要了）
//   - 半径是固定的（不估），三块板匀布、绕中心转
//
// 和车模型的关键差别：
//   · 没有 r/dr（半径固定 0.275），高度差用 dz1/dz2 表示
//   · 转速不是自由状态：投票定了方向之后直接写成 ±0.8π 的硬约束
//   · 没有"静止"这种可能 —— 前哨站只有顺、逆时针两种（所以投票里
//     不投"stable"这一档，见 target.cpp 的 vote_direction）
// ============================================================================
inline constexpr double kOutpostR    = 0.275;        // 旋转直径 550mm → 半径
inline constexpr double kOutpostDz1  = 0.102;        // 1216 - 1114 = 102mm
inline constexpr double kOutpostDz2  = 0.204;        // 1318 - 1114 = 204mm
inline constexpr double kOutpostVyaw = 0.8 * M_PI;   // 转速 |v_yaw| = 0.8π rad/s
// dz1/dz2 的过程噪声（和 v1/v2 同一量纲的"加速度方差"）：
// 高差本身是常数，但给一点点随机游走能留一条极细的修正通道 —— 否则 Q=0 时
// 协方差只减不增，收敛后滤波器会"自负"，先验抄得不准也再也纠不回来。
// ★ 取 rps26 的原值 10（原来是"保守一半"的 5）：dz 初值现在从 0 开始自学
//   （见 target.cpp init），多留一点修正通道更利于高度基准的恢复。
//   （注：这个量对前 ~3 秒的收敛几乎没影响，收敛靠的是卡尔曼增益本身。）
inline constexpr double kOutpostQdz   = 10.0;
inline constexpr int    kOutpostDirVotes = 30;       // 定方向需要的观测样本数

// ---- 前哨站观测门（照 auto_aim_rps26 outpost_model 的 matched 前置门）----
// 实机在前哨站跟前（2~4m，为了进 4m 开火门必须贴上去）出现"发散重建→几帧
// 又炸→再重建"（dz 冲 1e9/NaN、v_yaw 飞到 ±11）。斜视板的 PnP 位姿不可信，
// 原来无条件喂给滤波器。sim 实测加这道门误差还略降（6/6，0.045~0.059m）。
inline constexpr double kOutpostFacingGate = 35.0 * M_PI / 180.0;  // 板法线偏离自身方位角上限

class Target {
public:
  ArmorName name = not_armor;    // 兵种（one/two/three/.../outpost/base）
  ArmorType armor_type = small;  // 大小装甲板
  int armor_num_ = 4;            // 这块目标有几块装甲板（步兵4，前哨站/基地3）
  bool is_outpost_ = false;      // true = 走上面那套 10 维前哨站模型

  // 前哨站旋转方向：0 = 还没定，+1 = 逆时针，-1 = 顺时针（投票定的）
  // 定了之后 v_yaw 被写成硬约束 ±0.8π，不再让 EKF 自由估
  int outpost_dir_ = 0;
  int dir_ccw_ = 0, dir_cw_ = 0;   // 定方向那一次投票的票数（给日志用）

  // 投票进度（还没定方向时是已攒样本数，定了就归零）
  int dir_votes() const { return static_cast<int>(dir_data_.size()); }
  bool jumped = false;           // 上一帧数据关联有没有"跳板"（⚠ 只写不读，见 .cpp）
  int last_id = 0;               // 上一帧关联到的板编号（⚠ 只写不读）
  ExtendedKalmanFilter ekf_;     // 状态机本体

  Target() = default;

  // 用一块装甲板的【首次观测】把整个模型建起来（反推车中心位置，见 .cpp:5）
  //   radius    : 车中心→板的半径，按兵种给
  //   armor_num : 板数
  //   P0_dig    : 11 维初始协方差的对角元
  void init(const Armor & armor, double radius, int armor_num,
            const Eigen::VectorXd & P0_dig);

  // EKF 预测步：按匀速模型往前走 dt 秒，同时协方差随 Q 膨胀
  void predict(double dt);

  // EKF 更新步：拿一个装甲板观测修正状态。内部含【数据关联】——
  // 自动判断这个观测对应 4 块里的哪一块（见 .cpp:204）
  void update(const Eigen::Vector4d & z_obs, const Eigen::Vector4d & R_dig);

  // 按当前状态，算出所有 armor_num_ 块装甲板的位置和朝向
  // 返回 armor_num_ 个 [x, y, z, yaw]（pseudo 系）
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  // 【前哨站专用】方向投票：每帧喂一个观测到的板法线角（orient_yaw），
  // 攒够 kOutpostDirVotes 个样本后投出顺/逆时针，并锁定 v_yaw = ±0.8π。
  void vote_direction(double orient_yaw);

  // 检测滤波器是否发散。判据是半径估计跑出合理区间（见 .cpp:283）
  // main.cpp:89 和 visualizer.cpp:18,55 用它做保护，发散了就不开火/不画预测
  bool diverged() const;

  // 相机系下的量 → 观测向量 [yaw,pitch,dist,face_yaw]
  static Eigen::Vector4d camera_to_obs(const Eigen::Vector3d & cam_xyz, double face_yaw);

  // pseudo 系 → 相机系 的坐标变换
  static Eigen::Vector3d pseudo_to_camera(const Eigen::Vector3d & p);

  // 【观测模型 h(x)】给定状态和第 id 块板，算出那块板在 pseudo 系下的位置
  // 这是 EKF 里 h(x) 的核心，也是雅可比的求导对象
  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;

private:

  // 前哨站投票用的历史观测（板法线角），投完就清空
  std::vector<double> dir_data_;

  // h(x) 对状态 x 的雅可比（车模型 4×11 / 前哨站 4×10）。EKF 每帧都要重算
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;

  // 直角坐标 → 球坐标 [yaw, pitch, dist]，观测模型的前三个分量
  static Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz);

  // 上面那个变换的雅可比（3×3），链式法则的一环
  static Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d & xyz);
};
