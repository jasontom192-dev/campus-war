#pragma once

// ============================================================================
// rune_target.hpp —— 符目标的 EKF 模型（照 tdt-campus-test 的 target.py 移植）
//
// 两个模型，和车/前哨站的关系一样：都是 ExtendedKalmanFilter（ekf.hpp）上的
// 一层"物理"，区别只在状态维数和转速怎么来。
//
//   SmallTarget（小符）7 维：
//     [0] R_yaw  机关中心球坐标方位角      [1] v_R_yaw 遗留（打符场景≈0）
//     [2] R_pitch 仰角                    [3] R_dis   距离
//     [4] yaw    符面朝向                 [5] row/相位 哪片叶转到哪了
//     [6] spd    角速度  ← 常量 ±(π/3) rad/s，只用方向投票定正负号
//
//   BigTarget（大符）10 维：前 6 维同上，后 4 维是"正弦转速"的参数 ——
//     [6] spd = a·sin(w·t + fi) + (2.09 − a)   ← 大符不是匀速转！
//     [7] a   摆幅（标称 0.78–1.045 rad/s）  [8] w 角频率（1.884–2.0）
//     [9] fi  初相
//     predict() 里相位按这个模型【解析积分】推进（不是匀速乘 dt）。
//
// ★ 量测分两路（两个模型一样）：
//     z1 线性  = [机关中心方位角, 仰角, 距离, 相位]      （4 维，选择矩阵 H）
//     z2 非线性 = 叶心（R 标中心）的世界球坐标            （3 维，h(x) + 手工雅可比）
//   打符策略的核心量是相位（状态 5）：它由模型外推、由两路量测反复纠正。
//
// ★ 和车的 Target 最大的不同：目标【不换身份】—— 一片叶打掉就换下一片，
//   但模型（机关中心 + 相位）是连续的。所以这里没有 track/数据关联那一套。
// ============================================================================

#include <Eigen/Dense>
#include <cmath>
#include <deque>
#include <string>
#include <utility>

#include "ekf.hpp"
#include "rune_solver.hpp"   // RuneSolution

// 小符转速：RM 2026 "小能量机关的转速固定为 (1/3)π rad/s"。
// 本比赛的规则手册没有写转速条款，沿用这个值；大符用正弦模型自己估。
inline constexpr double kRuneSmallW = M_PI / 3.0;

// 模型量到的那个点（= R 标中心，z2 量测/雅可比都用它）
inline constexpr double kRuneRMarkRadius = 0.7;

// 转速自估计的可信区间（小符）：标称 60°/s，留 ±40°/s 余量；
// 超出这个范围的拟合基本都是换叶/误检造成的假斜率，直接丢掉
inline constexpr double kRuneOmegaMin     = 40.0 * M_PI / 180.0;
inline constexpr double kRuneOmegaMax     = 100.0 * M_PI / 180.0;
inline constexpr double kRuneOmegaWindowS = 0.8;    // 拟合窗口时长
inline constexpr int    kRuneOmegaMinSamples = 10;  // 至少这么多帧才拟合
inline constexpr double kRuneOmegaMinSpanS   = 0.4; // 窗口首尾至少隔这么久
inline constexpr double kRuneOmegaMaxRms     = 12.0 * M_PI / 180.0;  // 残差 RMS 上限
// 相邻帧相位差超过它就是换叶/误检（72° 才是换叶）。
// 45° 是量出来的：相位量测自身抖动 σ≈9°、相邻帧差值 σ≈12.7° —— 卡到 20° 会
// 被噪声每十几帧清空一次，永远攒不够拟合样本（这是第一版的实际事故）。
inline constexpr double kRuneOmegaJumpReset  = 45.0 * M_PI / 180.0;

// 大符正弦模型标称（sp_vision buff_target.cpp）—— 2026-10-01 用游戏真机录像
// 实测 a≈0.82、w≈1.95 正落在区间里，说明本游戏的大符就是同一套模型
inline constexpr double kBigSpdMean = 2.09;          // spd = a·sin(wt+fi) + (均值 − a)

// 收敛后冻结正弦参数的更新计数门槛（次）。头 ~2.5s（250 次更新）参数就稳了，
// 之后单帧观测继续拽 a/w/fi 只会让转速/相位外推抖（见 predict 里的说明）。
inline constexpr int    kBigFreezeAt = 250;
inline constexpr double kBigA0 = 0.9125, kBigW0 = 1.942;
inline constexpr double kBigAMin = 0.78,  kBigAMax = 1.045;
inline constexpr double kBigWMin = 1.884, kBigWMax = 2.0;
inline constexpr double kBigDiverge = 1.5;           // 发散判据：a/w 冲出标称 ±1.5 倍就重来

// 转速方向投票：累加相位的增减，饱和在 ±50，取符号（顺/逆时针）
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

// ─────────────────────────────────────────────────────────────────────────
// 小符：7 维，匀速转速（±60°/s + 方向投票）
// ─────────────────────────────────────────────────────────────────────────
class SmallTarget {
public:
  explicit SmallTarget(double aim_radius = kRuneRMarkRadius,
                       double phase_noise = 0.1,
                       const std::string & omega_mode = "auto");

  void reset();

  // 每帧入口。p == nullptr 表示这一帧没解出（丢帧）。
  // now 是调用方给的单调秒（节点里用 time.monotonic 等价物）。
  void get_target(const RuneSolution * p, double now);

  // 按当前状态往前走 dt 秒（注意：会原地改写滤波器状态 ——
  // 瞄准时要先拷贝一份再调，Python 版是 clone()，C++ 里默认拷贝构造即可）
  void predict(double dt);

  bool is_unsolved() const { return unsolvable_; }
  bool has_ekf() const { return has_ekf_; }
  const Eigen::VectorXd & ekf_x() const { return ekf_.x; }

  // buff 系一点 → 世界系（SPV 系：右/前/上）
  Eigen::Vector3d point_buff2world(const Eigen::Vector3d & pt) const;
  // 瞄点 = 叶片几何中心（buff 系 (0,0,aim_radius)）
  Eigen::Vector3d aim_point_world() const;

  // 实测转速（rad/s）；NaN = 还没采信（此时用标称 ±60°/s）
  double omega_est() const { return omega_est_; }
  double spd() const { return spd_; }

  // 诊断用（rune_main --diag / 排查"明明有检出却未解出"）
  int  lost_cn() const { return lost_cn_; }
  bool first_in() const { return first_in_; }
  int  phase_samples() const { return static_cast<int>(phases_.size()); }

private:
  void init(double nowtime, const RuneSolution & p);
  void update(double nowtime, const RuneSolution & p);
  void init_ekf(const RuneSolution & p);
  // z2 = H3·H2·H1·H0·x（3×7）—— 非线性观测的雅可比，照 buff_target.cpp 逐项搬
  Eigen::MatrixXd h_jacobian() const;

  // 相位解缠入队（换叶/误检时清空重来）
  void push_phase(double t, double phase_raw);
  // 最小二乘拟合相位斜率；不可信返回 NaN
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

  std::deque<std::pair<double, double>> phases_;   // (t, 解缠后的实测相位)
  double omega_est_ = std::nan("");                // 实测转速（rad/s），NaN = 未采信
  double omega_rms_ = std::nan("");
};

// ─────────────────────────────────────────────────────────────────────────
// 大符：10 维，正弦转速（spd = a·sin(wt+fi) + 2.09 − a）
// ─────────────────────────────────────────────────────────────────────────
class BigTarget {
public:
  explicit BigTarget(double aim_radius = kRuneRMarkRadius,
                     double phase_noise = 0.5);

  void reset();
  void get_target(const RuneSolution * p, double now);
  void predict(double dt);

  bool is_unsolved() const { return unsolvable_; }
  bool has_ekf() const { return has_ekf_; }
  const Eigen::VectorXd & ekf_x() const { return ekf_.x; }

  Eigen::Vector3d point_buff2world(const Eigen::Vector3d & pt) const;
  // 瞄点 = 环心（buff 系 (0,0,aim_radius)，aim_radius 默认 0.7）
  Eigen::Vector3d aim_point_world() const;

  // 给可视化/日志用：正弦模型的瞬时速度（rad/s）。大符没有"标称转速"，
  // 所以这里的值直接当"实测"显示
  double omega_est() const { return omega_est_; }
  double a_est() const { return a_est_; }
  double w_est() const { return w_est_; }

  // 诊断用（同 SmallTarget；大符没有相位队列，-1 占位）
  int  lost_cn() const { return lost_cn_; }
  bool first_in() const { return first_in_; }
  int  phase_samples() const { return -1; }

private:
  void init(double nowtime, const RuneSolution & p);
  void update(double nowtime, const RuneSolution & p);
  void init_ekf(const RuneSolution & p);
  Eigen::MatrixXd h_jacobian() const;

  // 实测转速（照 SmallTarget 的同一套：0.8s 窗最小二乘 + 可信门槛）。
  // 这台机器的转速接近"匀速 60~90°/s 带小波动"，不跟 RM 标准正弦的深谷；
  // 模型谷底（25°/s）处真机有 ~66°/s —— 提前量在慢速段差十几厘米。
  // 实测可信时直接用它推相位/提前量（见 predict），正弦模型退居备用。
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

  int updates_ = 0;              // 本段跟踪内的更新次数（冻结正弦参数的计时用）
  // 上一次收敛的正弦参数（"机器常数"）：重初始化时拿来当种子，别从头学。
  // a/w 跨开符也沿用；fi 只在本段内沿用（换一次开符机器可能停/重起，相位
  // 关系续不上）。见 init_ekf。
  double a_fit_ = kBigA0, w_fit_ = kBigW0, fi_fit_ = 0.0;
  bool fresh_entry_ = true;      // 本段是不是"新一次开符"（reset() 置位）
  bool seeded_ = false;          // 本段初始化是否沿用了收敛参数（→ 提前冻结）
  double omega_est_ = std::nan("");
  double a_est_ = kBigA0;
  double w_est_ = kBigW0;
  std::deque<std::pair<double, double>> phases_;   // (t, 解缠后的实测相位)
  double omega_meas_ = std::nan("");               // 实测转速（rad/s，带符号）
};
