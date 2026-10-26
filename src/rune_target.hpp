#pragma once

//符目标的 EKF 模型：小符 7 维，大符 10 维
//共用状态：[0]R_yaw [1]v_R_yaw [2]R_pitch [3]R_dis [4]yaw [5]相位 [6]spd
//大符 [7]a [8]w [9]fi，spd=a*sin(w*t+fi)+2.09-a

#include <Eigen/Dense>
#include <cmath>
#include <deque>
#include <string>
#include <utility>

#include "ekf.hpp"
#include "rune_solver.hpp" //RuneSolution

//小符标称转速 π/3 rad/s
inline constexpr double kRuneSmallW = M_PI / 3.0;

//叶心 / R 标中心到机关中心的距离(m)，z2 量测使用
inline constexpr double kRuneRMarkRadius = 0.7;

//转速拟合：0.8s 窗、≥10 样本、首尾≥0.4s、残差 RMS<12°；可信范围 40~100°/s
inline constexpr double kRuneOmegaMin     = 40.0 * M_PI / 180.0;
inline constexpr double kRuneOmegaMax     = 100.0 * M_PI / 180.0;
inline constexpr double kRuneOmegaWindowS = 0.8;
inline constexpr int    kRuneOmegaMinSamples = 10;
inline constexpr double kRuneOmegaMinSpanS   = 0.4;
inline constexpr double kRuneOmegaMaxRms     = 12.0 * M_PI / 180.0;
//相邻帧相位差>45° 视为换叶/误检(换叶实际为 72°，45° 高于量测噪声)
inline constexpr double kRuneOmegaJumpReset  = 45.0 * M_PI / 180.0;

//大符正弦模型中心值(rad/s)，spd=a*sin(wt+fi)+(kBigSpdMean-a)
inline constexpr double kBigSpdMean = 2.09;

//正弦参数约 250 次更新后冻结；a/w 初值与标称范围如下
inline constexpr int    kBigFreezeAt = 250;
inline constexpr double kBigA0 = 0.9125, kBigW0 = 1.942;
inline constexpr double kBigAMin = 0.78,  kBigAMax = 1.045;
inline constexpr double kBigWMin = 1.884, kBigWMax = 2.0;
inline constexpr double kBigDiverge = 1.5; //a/w 超出标称 1.5 倍判发散

//方向投票：累加相位增减，|cnt|>50 后停止；正值=顺时针
class RuneVoter {
public:
  void vote(double angle_last, double angle_now) {
    if (std::abs(cnt_) > 50) return;
    cnt_ += (angle_last > angle_now) ? -1 : 1;
  }
  int clockwise() const { return cnt_ > 0 ? 1 : -1; }
  int count() const { return cnt_; }
  void set_count(int c) { cnt_ = c; }
private:
  int cnt_ = 0;
};

//小符：7 维匀速模型，z1 中心球坐标+相位，z2 叶心球坐标
class SmallTarget {
public:
  explicit SmallTarget(double aim_radius = kRuneRMarkRadius,
                       double phase_noise = 0.1,
                       const std::string & omega_mode = "auto");

  void reset();

  //每帧入口；p=nullptr 表示丢帧，now 为单调秒
  void get_target(const RuneSolution * p, double now);

  //原地推进 dt 秒；瞄准时先拷贝一份再调
  void predict(double dt);

  bool is_unsolved() const { return unsolvable_; }
  bool has_ekf() const { return has_ekf_; }
  const Eigen::VectorXd & ekf_x() const { return ekf_.x; }

  //buff 系 -> spv 世界系(右/前/上)
  Eigen::Vector3d point_buff2world(const Eigen::Vector3d & pt) const;
  //瞄点=叶片几何中心，buff 系 (0,0,aim_radius)
  Eigen::Vector3d aim_point_world() const;

  //估计转速(rad/s)；NaN=未采信，回退标称 ±60°/s
  double omega_est() const { return omega_est_; }
  double spd() const { return spd_; }

  //诊断用
  int  lost_cn() const { return lost_cn_; }
  bool first_in() const { return first_in_; }
  int  phase_samples() const { return static_cast<int>(phases_.size()); }

private:
  void init(double nowtime, const RuneSolution & p);
  void update(double nowtime, const RuneSolution & p);
  void init_ekf(const RuneSolution & p);
  //z2 的 3x7 雅可比
  Eigen::MatrixXd h_jacobian() const;

  //相位解缠入队；跳变过大时清空
  void push_phase(double t, double phase_raw);
  //最小二乘拟合相位斜率(rad/s)；不可信返回 NaN
  double estimate_omega();

  double aim_radius_;
  double phase_noise_;
  std::string omega_mode_;

  ExtendedKalmanFilter ekf_;
  RuneVoter voter_;
  bool has_ekf_ = false;
  bool first_in_ = true;
  bool unsolvable_ = true;
  int lost_cn_ = 0;

  bool has_start_ = false;
  double start_time_ = 0.0;
  double lasttime_ = 0.0;
  double spd_ = 0.0;

  std::deque<std::pair<double, double>> phases_; //(t, 解缠后的相位量测)
  double omega_est_ = std::nan(""); //估计转速(rad/s)，NaN=未采信
  double omega_rms_ = std::nan("");
};

//大符：10 维，转速按正弦模型变化，z1/z2 同小符
class BigTarget {
public:
  explicit BigTarget(double aim_radius = kRuneRMarkRadius,
                     double phase_noise = 0.5);

  void reset();
  void get_target(const RuneSolution * p, double now);
  //原地推进 dt 秒；估计转速可信时优先用估计值，否则走正弦模型
  void predict(double dt);

  bool is_unsolved() const { return unsolvable_; }
  bool has_ekf() const { return has_ekf_; }
  const Eigen::VectorXd & ekf_x() const { return ekf_.x; }

  Eigen::Vector3d point_buff2world(const Eigen::Vector3d & pt) const;
  //瞄点=环心，buff 系 (0,0,aim_radius)，默认 0.7
  Eigen::Vector3d aim_point_world() const;

  //正弦模型的瞬时转速(rad/s)
  double omega_est() const { return omega_est_; }
  double a_est() const { return a_est_; }
  double w_est() const { return w_est_; }

  //诊断用；大符没有相位队列，phase_samples() 返回 -1
  int  lost_cn() const { return lost_cn_; }
  bool first_in() const { return first_in_; }
  int  phase_samples() const { return -1; }

private:
  void init(double nowtime, const RuneSolution & p);
  void update(double nowtime, const RuneSolution & p);
  void init_ekf(const RuneSolution & p);
  Eigen::MatrixXd h_jacobian() const;

  //相位解缠入队；跳变>45° 时清空
  void push_phase(double t, double phase_raw);
  double estimate_omega();

  double aim_radius_;
  double phase_noise_;

  ExtendedKalmanFilter ekf_;
  RuneVoter voter_;
  bool has_ekf_ = false;
  bool first_in_ = true;
  bool unsolvable_ = true;
  int lost_cn_ = 0;

  bool has_start_ = false;
  double start_time_ = 0.0;
  double lasttime_ = 0.0;

  int updates_ = 0; //本段更新次数，冻结计时用
  //上次收敛的正弦参数：a/w 跨开符沿用，fi 仅同一段沿用
  double a_fit_ = kBigA0, w_fit_ = kBigW0, fi_fit_ = 0.0;
  bool fresh_entry_ = true; //本段是否新一次开符(fi 不沿用)
  bool seeded_ = false; //本段是否沿用收敛参数(提前冻结)
  double omega_est_ = std::nan("");
  double a_est_ = kBigA0;
  double w_est_ = kBigW0;
  std::deque<std::pair<double, double>> phases_; //(t, 解缠后的相位量测)
  double omega_meas_ = std::nan(""); //估计转速(rad/s，带符号)
};
