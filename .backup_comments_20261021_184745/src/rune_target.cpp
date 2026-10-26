// ============================================================================
// rune_target.cpp —— 符目标 EKF 的实现（照 tdt-campus-test 的 target.py 移植）
//
// 两个类共用三段工具函数（本文件顶部）：
//   x_add_small / x_add_big —— EKF 的"状态相加"钩子（角度分量回卷）
//   buff2world              —— buff 系一点 → 世界系（前 6 维状态是两模型共用的）
//
// ★ 什么时候用哪一路量测（update 里顺序不能换）：
//   ① 先处理"扇叶跳变"：相位忽然差出一大截（换叶/换片）时，把量测掰到
//      离当前估计最近的等价角（±72° 的整数倍）—— 不做这一步，EKF 会以为
//      符瞬间转了 72°，把角速度状态带飞。
//   ② 方向投票 + 纠正 spd 符号。
//   ③ predict(dt) —— 注意这发生在【量测更新之前】（先预测后更新是 EKF 的标准顺序）。
//   ④ z1 线性量测（机关中心球坐标 + 相位）。
//   ⑤ z2 非线性量测（叶心世界球坐标）—— 它把"叶心在哪"这个几何约束喂给滤波器。
// ============================================================================

#include "rune_target.hpp"
#include "rune_geometry.hpp"

#include <algorithm>

namespace {

// EKF 的"状态相加"钩子：只有角度分量需要回卷（照 sp_vision 的 lambda）
Eigen::VectorXd x_add_small(const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
  Eigen::VectorXd c = a + b;
  for (int i : {0, 2, 4, 5}) c(i) = limit_rad(c(i));
  return c;
}

Eigen::VectorXd x_add_big(const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
  Eigen::VectorXd c = a + b;
  for (int i : {0, 2, 4, 5, 9}) c(i) = limit_rad(c(i));
  return c;
}

// buff 系一点 → 世界系（7 维/10 维状态的前 6 维排布相同，所以这一个函数两边通用）。
// 与 sp_vision buff_target.cpp:24-39 一致：符面 pitch 记 0，roll 就是相位。
Eigen::Vector3d buff2world(const Eigen::VectorXd & x, const Eigen::Vector3d & pt) {
  const Eigen::Matrix3d R = rune::rotation_matrix_zyx({x(4), 0.0, x(5)});
  const Eigen::Vector3d t(x(3) * std::cos(x(2)) * std::cos(x(0)),
                          x(3) * std::cos(x(2)) * std::sin(x(0)),
                          x(3) * std::sin(x(2)));
  return R * pt + t;
}

// H0：从状态里挑出 z2/h 用得到的分量（小符 [0,2,3,4,5]，大符多一个 fi 不参与）
Eigen::MatrixXd pick_rows(int rows, int cols, std::initializer_list<int> idx) {
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(rows, cols);
  int r = 0;
  for (int s : idx) H(r++, s) = 1.0;
  return H;
}

// 非线性量测的中间两层（H1/H2）—— 两个模型完全一样，只有 H0 的列数不同
Eigen::MatrixXd h_mid_layers(const Eigen::VectorXd & x, double mark_radius,
                             Eigen::MatrixXd & H3_out) {
  // H1：球坐标 → 直角坐标 的雅可比（只作用在前三个分量上）
  Eigen::MatrixXd H1 = Eigen::MatrixXd::Identity(5, 5);
  H1.block<3, 3>(0, 0) =
      rune::ypd2xyz_jacobian({x(0), x(2), x(3)});

  // H2：叶心在 buff 系是 (0,0,mark_radius)，对 yaw(第4) 和 roll/相位(第5) 求导
  const double yaw = x(4), roll = x(5);
  const double cy = std::cos(yaw),  sy = std::sin(yaw);
  const double cr = std::cos(roll), sr = std::sin(roll);
  Eigen::MatrixXd H2 = Eigen::MatrixXd::Zero(3, 5);
  H2 << 1.0, 0.0, 0.0, mark_radius * cy * sr,  mark_radius * sy * cr,
        0.0, 1.0, 0.0, mark_radius * sy * sr, -mark_radius * cy * cr,
        0.0, 0.0, 1.0, 0.0,                   -mark_radius * sr;

  // H3：直角坐标 → 球坐标 的雅可比，作用在叶心的世界坐标上
  const Eigen::Vector3d B_xyz = buff2world(x, {0.0, 0.0, mark_radius});
  H3_out = rune::xyz2ypd_jacobian(B_xyz);

  return H2 * H1;   // 3×5
}

}  // namespace

// ============================================================================
// SmallTarget —— 小符 7 维
// ============================================================================

SmallTarget::SmallTarget(double aim_radius, double phase_noise,
                         const std::string & omega_mode)
  : aim_radius_(aim_radius), phase_noise_(phase_noise), omega_mode_(omega_mode) {}

void SmallTarget::reset() {
  first_in_ = true;
  unsolvable_ = true;
  lost_cn_ = 0;
  phases_.clear();
  omega_est_ = std::nan("");
  omega_rms_ = std::nan("");
}

Eigen::Vector3d SmallTarget::point_buff2world(const Eigen::Vector3d & pt) const {
  if (unsolvable_ || !has_ekf_) return Eigen::Vector3d::Zero();
  return buff2world(ekf_.x, pt);
}

Eigen::Vector3d SmallTarget::aim_point_world() const {
  return point_buff2world({0.0, 0.0, aim_radius_});
}

void SmallTarget::init_ekf(const RuneSolution & p) {
  // 转速初值：有可信实测就用实测（方向仍由 Voter 管符号），否则标称 ±60°/s。
  // ★ 必须和 get_target 的发散判据用同一个参考值：复位时如果写死标称 60，
  //   而机器实测转 75°/s，下一帧就会被判"偏离参考 15°"再次复位 —— init 写
  //   标称、判据锚实测，两边打架就是又一个复位死循环（离线实测残余 895 帧
  //   全是这个：发散帧的 x(6) 中位 = -60.0 = 刚到手的标称初值）。
  const double spd0 = std::isnan(omega_est_) ? kRuneSmallW : std::abs(omega_est_);
  Eigen::VectorXd x0(7);
  x0 << p.ypd(0), 0.0, p.ypd(1), p.ypd(2), p.ypr(0), p.ypr(2),
        spd0 * voter_.clockwise();

  Eigen::VectorXd p0 = Eigen::VectorXd::Constant(7, 10.0);
  p0(6) = 1e-2;                       // 转速初值很确定（标称 ±60°/s），方差给小
  ekf_ = ExtendedKalmanFilter(x0, p0.asDiagonal(), x_add_small);
  has_ekf_ = true;
}

void SmallTarget::init(double nowtime, const RuneSolution & p) {
  lasttime_ = nowtime;
  init_ekf(p);
}

void SmallTarget::predict(double dt) {
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(7, 7);
  A(0, 1) = dt;                        // R_yaw += dt · v_R_yaw
  A(5, 6) = dt;                        // 相位 += dt · spd

  const double v1 = 0.001;             // 角加速度方差（sp_vision 原值）
  const double a = dt * dt * dt * dt / 4.0;
  const double b = dt * dt * dt / 2.0;
  const double c = dt * dt;
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(7, 7);
  Q(0, 0) = a * v1;  Q(0, 1) = b * v1;
  Q(1, 0) = b * v1;  Q(1, 1) = c * v1;

  ekf_.predict(A, Q, [&](const Eigen::VectorXd & xv) {
    Eigen::VectorXd xp = A * xv;
    for (int i : {0, 2, 4, 5}) xp(i) = limit_rad(xp(i));
    return xp;
  });
}

Eigen::MatrixXd SmallTarget::h_jacobian() const {
  Eigen::MatrixXd H3;
  const Eigen::MatrixXd mid = h_mid_layers(ekf_.x, kRuneRMarkRadius, H3);
  return H3 * mid * pick_rows(5, 7, {0, 2, 3, 4, 5});   // 3×7
}

void SmallTarget::push_phase(double t, double phase_raw) {
  if (!phases_.empty()) {
    const double last = phases_.back().second;
    const double d = limit_rad(phase_raw - last);
    if (std::abs(d) > kRuneOmegaJumpReset) phases_.clear();   // 换叶/误检：重来
    else phase_raw = last + d;                                // 解缠
  }
  phases_.emplace_back(t, phase_raw);
  while (phases_.size() > 400) phases_.pop_front();           // 和 Python 的 maxlen 对齐
}

double SmallTarget::estimate_omega() {
  if (phases_.size() < 2) return std::nan("");

  const double t_end = phases_.back().first;
  size_t i0 = 0;
  while (i0 < phases_.size() && phases_[i0].first < t_end - kRuneOmegaWindowS) ++i0;
  const size_t n = phases_.size() - i0;
  if (static_cast<int>(n) < kRuneOmegaMinSamples) return std::nan("");
  const double span = phases_.back().first - phases_[i0].first;
  if (span < kRuneOmegaMinSpanS) return std::nan("");

  // 最小二乘拟合斜率（相位 = 斜率·t + 截距）
  double tm = 0.0, pm = 0.0;
  for (size_t i = i0; i < phases_.size(); ++i) {
    tm += phases_[i].first;
    pm += phases_[i].second;
  }
  tm /= n;  pm /= n;

  double sxx = 0.0, sxy = 0.0;
  for (size_t i = i0; i < phases_.size(); ++i) {
    const double dt = phases_[i].first - tm;
    sxx += dt * dt;
    sxy += dt * (phases_[i].second - pm);
  }
  if (sxx <= 1e-12) return std::nan("");
  const double slope = sxy / sxx;

  double sr = 0.0;
  for (size_t i = i0; i < phases_.size(); ++i) {
    const double r = (phases_[i].second - pm) - slope * (phases_[i].first - tm);
    sr += r * r;
  }
  const double rms = std::sqrt(sr / n);

  if (rms > kRuneOmegaMaxRms) return std::nan("");
  if (std::abs(slope) < kRuneOmegaMin || std::abs(slope) > kRuneOmegaMax) return std::nan("");

  omega_rms_ = rms;
  return slope;
}

void SmallTarget::update(double nowtime, const RuneSolution & p) {
  Eigen::VectorXd & x = ekf_.x;

  // ① 扇叶跳变：见文件头
  if (std::abs(p.ypr(2) - x(5)) > M_PI / 12) {
    for (int i = -5; i <= 5; ++i) {
      const double angle_c = x(5) + i * 2 * M_PI / 5;
      if (std::abs(angle_c - p.ypr(2)) < M_PI / 5) {
        x(5) += i * 2 * M_PI / 5;
        break;
      }
    }
  }

  // ② 方向投票 + 纠正 spd 符号
  voter_.vote(x(5), p.ypr(2));
  if (voter_.clockwise() * x(6) < 0) x(6) *= -1;

  // ③ 预测
  predict(nowtime - lasttime_);

  // ④ 线性量测：机关中心球坐标 + 相位
  const Eigen::MatrixXd H1 = pick_rows(4, 7, {0, 2, 3, 5});
  Eigen::Vector4d R1_diag(0.01, 0.01, 0.5, phase_noise_);
  const Eigen::Vector4d z1(p.ypd(0), p.ypd(1), p.ypd(2), p.ypr(2));
  auto z_sub1 = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
    Eigen::VectorXd c = a - b;
    c(0) = limit_rad(c(0));
    c(1) = limit_rad(c(1));
    c(3) = limit_rad(c(3));
    return c;
  };
  ekf_.update(z1, H1, R1_diag.asDiagonal(),
              [&H1](const Eigen::VectorXd & xv) { return H1 * xv; }, z_sub1);

  // ⑤ 非线性量测：叶心世界球坐标
  const Eigen::MatrixXd H2 = h_jacobian();
  Eigen::Vector3d R2_diag(0.01, 0.01, 0.5);
  const Eigen::Vector3d z2 = p.blade_ypd;
  auto h2 = [](const Eigen::VectorXd & xv) {
    return rune::xyz2ypd(buff2world(xv, {0.0, 0.0, kRuneRMarkRadius}));
  };
  auto z_sub2 = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
    Eigen::VectorXd c = a - b;
    c(0) = limit_rad(c(0));
    c(1) = limit_rad(c(1));
    return c;
  };
  ekf_.update(z2, H2, R2_diag.asDiagonal(), h2, z_sub2);

  lasttime_ = nowtime;
  spd_ = voter_.clockwise() * kRuneSmallW;

  // ⑥ 转速自估计：够可信就改用实测值外推（方向仍由投票管）
  push_phase(nowtime, p.ypr(2));
  if (omega_mode_ != "fixed") {
    const double est = estimate_omega();
    if (!std::isnan(est)) {
      omega_est_ = est;
      ekf_.x(6) = est;
    }
  }
}

void SmallTarget::get_target(const RuneSolution * p, double now) {
  if (!has_start_) {
    start_time_ = now;
    has_start_ = true;
  }
  const double t = now - start_time_;

  if (p == nullptr) {
    unsolvable_ = true;
    lost_cn_++;
    return;
  }

  if (first_in_) {
    unsolvable_ = true;
    init(t, *p);
    first_in_ = false;
  }

  if (lost_cn_ > 6) {                 // 连续丢太久：重来
    unsolvable_ = true;
    lost_cn_ = 0;
    first_in_ = true;
    return;
  }
  // ★ 成功帧清零：lost_cn_ 改成【连续丢帧】计数（原来照 Python 是累计计数、
  //   成功帧不清零 —— 散布的丢检会攒到 7 就强制复位一次，每 ~0.5 s 一次）。
  //   大符的正弦模型每次复位都把 a/w/fi 打回出厂值、要好几秒才重新收敛，
  //   而单次开符窗口很短 —— 实机表现就是"大符老是未解出、一次未解等于白打
  //   重来"（小符不受影响：匀速模型一帧就收敛）。改成连续计数后，零星丢检
  //   不再触发复位，只有真"连续丢 >6 帧"（≈70 ms 全无检出）才重建。
  lost_cn_ = 0;

  unsolvable_ = false;
  update(t, *p);

  // 发散判据：转速偏离"当前最可信的参考值"超过 10° 就重来。
  // 参考值 = 实测转速（auto 模式量到可信值时）；还没量到时才用标称 ±60°/s。
  //
  // ★ 不能死卡标称 60°/s（原样照抄 sp_vision 的写法）：这个游戏的符转速
  //   本来就有 45~85°/s 的起伏（Python 版当初加"转速自估计"就是为了它）。
  //   死卡标称会把"机器本来就转得快/慢"判成发散 → 每帧"复位→重估→再发散"
  //   死循环。离线实测（records/20260930_191353）：8995 个有解的帧里 2223 帧
  //   被这么判掉（98.8% 是发散复位，最长死循环串 117 帧），游戏里表现为
  //   "明明识别到了却一直未解出"。锚到实测转速后，x(6) 每帧都被 update 写成
  //   实测值、与参考值重合，只有真·状态被带飞（偏离实测 >10°）才会触发复位。
  const double spd_ref = std::isnan(omega_est_) ? kRuneSmallW
                                                : std::abs(omega_est_);
  if (std::abs(std::abs(ekf_.x(6)) - spd_ref) > M_PI / 18.0) {
    unsolvable_ = true;
    first_in_ = true;
  }
}

// ============================================================================
// BigTarget —— 大符 10 维（正弦转速）
// ============================================================================

BigTarget::BigTarget(double aim_radius, double phase_noise)
  : aim_radius_(aim_radius), phase_noise_(phase_noise) {}

void BigTarget::reset() {
  first_in_ = true;
  unsolvable_ = true;
  lost_cn_ = 0;
  updates_ = 0;                  // 新一次开符：冻结计时清零（a/w 沿用，见 init_ekf）
  fresh_entry_ = true;           // 新一次开符：fi 不沿用（机器可能停过/重起）
  omega_est_ = std::nan("");
  a_est_ = kBigA0;
  w_est_ = kBigW0;
  phases_.clear();
  omega_meas_ = std::nan("");
}

// 相位解缠入队 + 最小二乘测速（与 SmallTarget 逐行同款：>45° 当换叶/误检清空，
// 0.8s 窗、≥10 样本、span≥0.4s、残差 RMS<12° 才采信 —— 相位量测自身抖动
// ~9~12° 量级，门槛再严就永远攒不够样本）
void BigTarget::push_phase(double t, double phase_raw) {
  if (!phases_.empty()) {
    const double last = phases_.back().second;
    const double d = limit_rad(phase_raw - last);
    if (std::abs(d) > kRuneOmegaJumpReset) phases_.clear();
    else phase_raw = last + d;
  }
  phases_.emplace_back(t, phase_raw);
  while (phases_.size() > 400) phases_.pop_front();
}

double BigTarget::estimate_omega() {
  if (phases_.size() < 2) return std::nan("");

  const double t_end = phases_.back().first;
  size_t i0 = 0;
  while (i0 < phases_.size() && phases_[i0].first < t_end - kRuneOmegaWindowS) ++i0;
  const size_t n = phases_.size() - i0;
  if (static_cast<int>(n) < kRuneOmegaMinSamples) return std::nan("");
  const double span = phases_.back().first - phases_[i0].first;
  if (span < kRuneOmegaMinSpanS) return std::nan("");

  double tm = 0.0, pm = 0.0;
  for (size_t i = i0; i < phases_.size(); ++i) {
    tm += phases_[i].first;
    pm += phases_[i].second;
  }
  tm /= n;  pm /= n;

  double sxx = 0.0, sxy = 0.0;
  for (size_t i = i0; i < phases_.size(); ++i) {
    const double dt = phases_[i].first - tm;
    sxx += dt * dt;
    sxy += dt * (phases_[i].second - pm);
  }
  if (sxx <= 1e-12) return std::nan("");
  const double slope = sxy / sxx;

  double sr = 0.0;
  for (size_t i = i0; i < phases_.size(); ++i) {
    const double r = (phases_[i].second - pm) - slope * (phases_[i].first - tm);
    sr += r * r;
  }
  const double rms = std::sqrt(sr / n);
  if (rms > kRuneOmegaMaxRms) return std::nan("");
  if (std::abs(slope) < kRuneOmegaMin || std::abs(slope) > kRuneOmegaMax) return std::nan("");
  return slope;
}

Eigen::Vector3d BigTarget::point_buff2world(const Eigen::Vector3d & pt) const {
  if (unsolvable_ || !has_ekf_) return Eigen::Vector3d::Zero();
  return buff2world(ekf_.x, pt);
}

Eigen::Vector3d BigTarget::aim_point_world() const {
  return point_buff2world({0.0, 0.0, aim_radius_});
}

void BigTarget::init_ekf(const RuneSolution & p) {
  // ★ 种子用"上次收敛的正弦参数"而不是标称出厂值：
  //   机器就那一台，a/w 在整局里不变；每次丢检重捕获都从头学一遍的话，
  //   头 ~2.5 秒参数没收敛，观测会把转速/相位拽得乱抖 —— 离线实测所有
  //   转速暴走尖峰（>300°/s，最高 2015）**全部**落在重初始化后 2~11 帧。
  //   fi 只在本段内沿用（同一次开符内机器相位是连续的）；新一次开符
  //   （reset() → fresh_entry_）fi 清零重估，a/w 照旧沿用。
  const bool fit_ok = std::abs(a_fit_) > 0.4 && std::abs(a_fit_) < 1.5 &&
                      std::abs(w_fit_) > 1.4 && std::abs(w_fit_) < 2.8;
  const double a0 = fit_ok ? a_fit_ : kBigA0;
  const double w0 = fit_ok ? w_fit_ : kBigW0;
  const bool seed_fi = fit_ok && !fresh_entry_;
  const double fi0 = seed_fi ? fi_fit_ : 0.0;
  // 种子好（fi 也沿用了）→ 参数已经是收敛值，predict 里几帧后就冻结；
  // 否则（新开符/没有历史）照常从头学。
  seeded_ = seed_fi;
  fresh_entry_ = false;
  Eigen::VectorXd x0(10);
  x0 << p.ypd(0), 0.0, p.ypd(1), p.ypd(2), p.ypr(0), p.ypr(2),
        kBigSpdMean - a0, a0, w0, fi0;               // spd 初值按种子的 a 给

  Eigen::VectorXd p0 = Eigen::VectorXd::Constant(10, 10.0);
  p0(6) = 100.0;    // spd
  p0(9) = 400.0;    // fi —— 初相最不确定
  ekf_ = ExtendedKalmanFilter(x0, p0.asDiagonal(), x_add_big);
  has_ekf_ = true;
  updates_ = 0;     // 重新开始学正弦参数（predict 里的"收敛后冻结"计时）
}

void BigTarget::init(double nowtime, const RuneSolution & p) {
  // 隔了好几秒才回来（丢检一段后重捕获 / 离线录像里跨开符）= 当"新一次开符"，
  // fi 不沿用（机器期间可能停过，相位关系续不上）；短丢检（<5s）沿用 fi。
  if (nowtime - lasttime_ > 5.0) fresh_entry_ = true;
  lasttime_ = nowtime;
  init_ekf(p);
}

void BigTarget::predict(double dt) {
  Eigen::VectorXd & x = ekf_.x;
  double a  = x(7);
  double w  = x(8);
  const double fi = x(9);
  if (std::abs(w) < 1e-6) w = 1e-6;          // a/w 要除，防呆
  const int cw = voter_.clockwise();
  const double t = lasttime_ + dt;

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(10, 10);
  A(0, 1) = dt;
  A(5, 6) = cw * dt;
  A(6, 7) = std::sin(w * t + fi) - 1.0;       // ∂spd/∂a
  A(6, 8) = t * a * std::cos(w * t + fi);     // ∂spd/∂w
  A(6, 9) = a * std::cos(w * t + fi);         // ∂spd/∂fi

  const double v1 = 0.9;                      // 角加速度方差（sp_vision 大符原值）
  const double a1 = dt * dt * dt * dt / 4.0;
  const double b1 = dt * dt * dt / 2.0;
  const double c1 = dt * dt;
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(10, 10);
  Q(0, 0) = a1 * v1;  Q(0, 1) = b1 * v1;
  Q(1, 0) = b1 * v1;  Q(1, 1) = c1 * v1;
  Q(5, 5) = 0.09;                             // 相位
  Q(6, 6) = 0.5;                              // spd
  Q(9, 9) = 1.0;                              // fi

  // ★ 收敛后冻结正弦参数（a/w/fi）和由它们决定的转速：
  //   在线估计头 ~2.5s（kBigFreezeAt 次更新）参数就稳了（离线实测 a/w→0.83/1.97）；
  //   之后单帧观测再拽它们，只会让转速/相位外推抖 —— 日志里"转速"在 9~154°/s
  //   乱跳、甚至冲出模型数学上限（2000+ 也出现过），瞄点跟着散（实机"落点散、
  //   后面几轮更散"的主嫌疑）。冻结后：spd = 参数化的确定性正弦，相位照常由
  //   观测修正 —— 行为和稳的小符一致（小符能打就是因为它只有"相位+常量转速"）。
  //   参数还没收敛到合理区间时继续学，不冻。mid-round 重初始化（init_ekf）会
  //   把 updates_ 清零，自然解冻重估。
  if (updates_ > (seeded_ ? 5 : kBigFreezeAt) &&
      std::abs(x(7)) > 0.5 && std::abs(x(7)) < 1.4 &&
      std::abs(x(8)) > 1.5 && std::abs(x(8)) < 2.6) {
    Q(6, 6) = 0.0;
    Q(9, 9) = 0.0;
    for (int i : {6, 7, 8, 9}) {          // 连交叉协方差一起隔离：
      ekf_.P.row(i).setZero();            // 否则 K 还能从旁路把冻结态拽动
      ekf_.P.col(i).setZero();
      ekf_.P(i, i) = 1e-6;
    }
  }

  // 相位推进：实测转速可信就用实测（照 SmallTarget —— 实测是带符号的，
  // 连方向一起用，不再乘 cw），否则回落正弦模型的解析积分。
  //   ★ 这台机器的转速接近"匀速 60~90°/s 带小波动"，和 RM 标准正弦的深谷
  //     （25°/s）对不上：相位一直被观测锚着所以残差好看，但提前量用的是
  //     模型导数、没人直接测它 —— 慢速段每发的提前量能差十几厘米（实机
  //     "叶片刚出来/转速低的时候打不准"）。实测只有 0.8s 窗的滞后，而真机
  //     转速变化本来就慢（±11°/s 量级），滞后无所谓。
  const bool have_meas = !std::isnan(omega_meas_);
  ekf_.predict(A, Q, [&](const Eigen::VectorXd & xv) {
    Eigen::VectorXd xp = xv;
    xp(0) = limit_rad(xp(0) + dt * xp(1));
    xp(2) = limit_rad(xp(2));
    xp(4) = limit_rad(xp(4));
    if (have_meas) {
      xp(5) = limit_rad(xp(5) + omega_meas_ * dt);
      xp(6) = omega_meas_;
    } else {
      // 相位按正弦模型【解析积分】推进（不是匀速乘 dt）：
      //   相位增量 = ∫spd dt = −a/w·cos(w·t+fi) + a/w·cos(w·lasttime+fi) + (2.09−a)·dt
      xp(5) = limit_rad(xp(5) + cw * (-a / w * std::cos(w * t + fi)
                                      + a / w * std::cos(w * lasttime_ + fi)
                                      + (kBigSpdMean - a) * dt));
      xp(6) = a * std::sin(w * t + fi) + kBigSpdMean - a;
    }
    return xp;
  });
}

Eigen::MatrixXd BigTarget::h_jacobian() const {
  Eigen::MatrixXd H3;
  const Eigen::MatrixXd mid = h_mid_layers(ekf_.x, kRuneRMarkRadius, H3);
  return H3 * mid * pick_rows(5, 10, {0, 2, 3, 4, 5});   // 3×10
}

void BigTarget::update(double nowtime, const RuneSolution & p) {
  Eigen::VectorXd & x = ekf_.x;

  if (std::abs(p.ypr(2) - x(5)) > M_PI / 12) {
    for (int i = -5; i <= 5; ++i) {
      const double angle_c = x(5) + i * 2 * M_PI / 5;
      if (std::abs(angle_c - p.ypr(2)) < M_PI / 5) {
        x(5) += i * 2 * M_PI / 5;
        break;
      }
    }
  }

  voter_.vote(x(5), p.ypr(2));

  predict(nowtime - lasttime_);

  // ① 线性量测（相位噪声用 phase_noise_ —— 大符默认 0.5，比小符大得多：
  //    正弦模型已经用真机录像验证过，多信模型、少信单帧量测，击打点就不抖）
  const Eigen::MatrixXd H1 = pick_rows(4, 10, {0, 2, 3, 5});
  Eigen::Vector4d R1_diag(0.01, 0.01, 0.5, phase_noise_);
  const Eigen::Vector4d z1(p.ypd(0), p.ypd(1), p.ypd(2), p.ypr(2));
  auto z_sub1 = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
    Eigen::VectorXd c = a - b;
    c(0) = limit_rad(c(0));
    c(1) = limit_rad(c(1));
    c(3) = limit_rad(c(3));
    return c;
  };
  ekf_.update(z1, H1, R1_diag.asDiagonal(),
              [&H1](const Eigen::VectorXd & xv) { return H1 * xv; }, z_sub1);

  // ② 非线性量测
  const Eigen::MatrixXd H2 = h_jacobian();
  Eigen::Vector3d R2_diag(0.01, 0.01, 0.5);
  const Eigen::Vector3d z2 = p.blade_ypd;
  auto h2 = [](const Eigen::VectorXd & xv) {
    return rune::xyz2ypd(buff2world(xv, {0.0, 0.0, kRuneRMarkRadius}));
  };
  auto z_sub2 = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
    Eigen::VectorXd c = a - b;
    c(0) = limit_rad(c(0));
    c(1) = limit_rad(c(1));
    return c;
  };
  ekf_.update(z2, H2, R2_diag.asDiagonal(), h2, z_sub2);

  lasttime_ = nowtime;
  omega_est_ = ekf_.x(6);        // 瞬时速度（正弦模型算出来的）
  a_est_ = ekf_.x(7);
  w_est_ = ekf_.x(8);
  ++updates_;                    // "收敛后冻结"的计时（见 predict）
  // 存"上次收敛的参数"当种子（够多帧才信，见 init_ekf）
  if (updates_ >= 60) {
    a_fit_ = ekf_.x(7);
    w_fit_ = ekf_.x(8);
    fi_fit_ = ekf_.x(9);
  }
  // 实测转速（可信才刷新；不可信时沿用上一次可信值）
  push_phase(nowtime, p.ypr(2));
  const double est = estimate_omega();
  if (!std::isnan(est)) omega_meas_ = est;
}

void BigTarget::get_target(const RuneSolution * p, double now) {
  if (!has_start_) {
    start_time_ = now;
    has_start_ = true;
  }
  const double t = now - start_time_;

  if (p == nullptr) {
    unsolvable_ = true;
    lost_cn_++;
    return;
  }

  if (first_in_) {
    unsolvable_ = true;
    init(t, *p);
    first_in_ = false;
  }

  if (lost_cn_ > 6) {
    unsolvable_ = true;
    lost_cn_ = 0;
    first_in_ = true;
    return;
  }
  // ★ 成功帧清零：改"连续丢帧"计数（同 SmallTarget 的说明 —— 累计计数会
  //   让散布丢检每 ~0.5 s 强制复位一次正弦模型，大符窗口短根本来不及收敛）
  lost_cn_ = 0;

  unsolvable_ = false;
  update(t, *p);

  // 发散判据：a、w 冲出标称 ±1.5 倍就重来（sp_vision buff_target.cpp:394）
  const double a = ekf_.x(7), w = ekf_.x(8);
  if (!(kBigAMin / kBigDiverge <= a && a <= kBigAMax * kBigDiverge) ||
      !(kBigWMin / kBigDiverge <= w && w <= kBigWMax * kBigDiverge)) {
    first_in_ = true;
  }
}
