//符目标 EKF 实现(小符 7 维 / 大符 10 维)
//每帧顺序：相位解缠 -> 方向投票 -> predict -> z1(中心+相位) -> z2(叶心)
//大符正弦参数收敛后冻结；估计转速可信时优先按估计值推进相位

#include "rune_target.hpp"
#include "rune_geometry.hpp"

#include <algorithm>

namespace {

//EKF 状态相加钩子，角度分量回卷
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

//buff 系 -> spv 世界系；符面 pitch 记 0，roll 为相位
Eigen::Vector3d buff2world(const Eigen::VectorXd & x, const Eigen::Vector3d & pt) {
  const Eigen::Matrix3d R = rune::rotation_matrix_zyx({x(4), 0.0, x(5)});
  const Eigen::Vector3d t(x(3) * std::cos(x(2)) * std::cos(x(0)),
                          x(3) * std::cos(x(2)) * std::sin(x(0)),
                          x(3) * std::sin(x(2)));
  return R * pt + t;
}

//H0：从状态中挑出 z2 用到的分量 {0,2,3,4,5}
Eigen::MatrixXd pick_rows(int rows, int cols, std::initializer_list<int> idx) {
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(rows, cols);
  int r = 0;
  for (int s : idx) H(r++, s) = 1.0;
  return H;
}

//非线性量测的中间两层 H1/H2，H3 经引用返回
Eigen::MatrixXd h_mid_layers(const Eigen::VectorXd & x, double mark_radius,
                             Eigen::MatrixXd & H3_out) {
  //H1：球坐标 -> 直角坐标雅可比，作用在前三维
  Eigen::MatrixXd H1 = Eigen::MatrixXd::Identity(5, 5);
  H1.block<3, 3>(0, 0) =
      rune::ypd2xyz_jacobian({x(0), x(2), x(3)});

  //H2：叶心 buff 系 (0,0,mark_radius)，对 yaw(4) 和相位(5) 求导
  const double yaw = x(4), roll = x(5);
  const double cy = std::cos(yaw),  sy = std::sin(yaw);
  const double cr = std::cos(roll), sr = std::sin(roll);
  Eigen::MatrixXd H2 = Eigen::MatrixXd::Zero(3, 5);
  H2 << 1.0, 0.0, 0.0, mark_radius * cy * sr,  mark_radius * sy * cr,
        0.0, 1.0, 0.0, mark_radius * sy * sr, -mark_radius * cy * cr,
        0.0, 0.0, 1.0, 0.0,                   -mark_radius * sr;

  //H3：叶心世界坐标的直角 -> 球坐标雅可比
  const Eigen::Vector3d B_xyz = buff2world(x, {0.0, 0.0, mark_radius});
  H3_out = rune::xyz2ypd_jacobian(B_xyz);

  return H2 * H1; //3×5
}

}

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
  //转速初值优先用估计值；须与发散判据用同一参考值
  const double spd0 = std::isnan(omega_est_) ? kRuneSmallW : std::abs(omega_est_);
  Eigen::VectorXd x0(7);
  x0 << p.ypd(0), 0.0, p.ypd(1), p.ypd(2), p.ypr(0), p.ypr(2),
        spd0 * voter_.clockwise();

  Eigen::VectorXd p0 = Eigen::VectorXd::Constant(7, 10.0);
  p0(6) = 1e-2; //转速初值方差
  ekf_ = ExtendedKalmanFilter(x0, p0.asDiagonal(), x_add_small);
  has_ekf_ = true;
}

void SmallTarget::init(double nowtime, const RuneSolution & p) {
  lasttime_ = nowtime;
  init_ekf(p);
}

void SmallTarget::predict(double dt) {
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(7, 7);
  A(0, 1) = dt; //R_yaw += dt*v_R_yaw
  A(5, 6) = dt; //相位 += dt*spd

  const double v1 = 0.001; //角加速度方差
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
  return H3 * mid * pick_rows(5, 7, {0, 2, 3, 4, 5}); //3x7
}

void SmallTarget::push_phase(double t, double phase_raw) {
  if (!phases_.empty()) {
    const double last = phases_.back().second;
    const double d = limit_rad(phase_raw - last);
    if (std::abs(d) > kRuneOmegaJumpReset) phases_.clear(); //跳变过大：清空重来
    else phase_raw = last + d; //解缠
  }
  phases_.emplace_back(t, phase_raw);
  while (phases_.size() > 400) phases_.pop_front(); //最多保留 400 点
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

  //最小二乘拟合相位斜率
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

  //相位差>15° 时按 72° 整数倍搬到最近的等价角
  if (std::abs(p.ypr(2) - x(5)) > M_PI / 12) {
    for (int i = -5; i <= 5; ++i) {
      const double angle_c = x(5) + i * 2 * M_PI / 5;
      if (std::abs(angle_c - p.ypr(2)) < M_PI / 5) {
        x(5) += i * 2 * M_PI / 5;
        break;
      }
    }
  }

  //方向投票并纠正 spd 符号
  voter_.vote(x(5), p.ypr(2));
  if (voter_.clockwise() * x(6) < 0) x(6) *= -1;

  predict(nowtime - lasttime_);

  //z1：机关中心球坐标 + 相位
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

  //z2：叶心世界球坐标
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

  //估计转速可信时改用估计值外推，方向仍由投票管
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

  if (lost_cn_ > 6) { //连续丢检>6 帧：重建
    unsolvable_ = true;
    lost_cn_ = 0;
    first_in_ = true;
    return;
  }
  //成功帧清零，使 lost_cn_ 表示连续丢帧数
  lost_cn_ = 0;

  unsolvable_ = false;
  update(t, *p);

  //spd 偏离可信参考值>10° 判发散；参考值优先估计值，未采信时用 π/3
  const double spd_ref = std::isnan(omega_est_) ? kRuneSmallW
                                                : std::abs(omega_est_);
  if (std::abs(std::abs(ekf_.x(6)) - spd_ref) > M_PI / 18.0) {
    unsolvable_ = true;
    first_in_ = true;
  }
}

BigTarget::BigTarget(double aim_radius, double phase_noise)
  : aim_radius_(aim_radius), phase_noise_(phase_noise) {}

void BigTarget::reset() {
  first_in_ = true;
  unsolvable_ = true;
  lost_cn_ = 0;
  updates_ = 0; //新一次开符：冻结计时清零，a/w 沿用
  fresh_entry_ = true; //新一次开符：fi 不沿用
  omega_est_ = std::nan("");
  a_est_ = kBigA0;
  w_est_ = kBigW0;
  phases_.clear();
  omega_meas_ = std::nan("");
}

//相位解缠入队 + 最小二乘测速，门槛同 SmallTarget
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
  //种子用上次收敛的 a/w；fi 仅同一次开符内沿用，避免重捕获时从头学
  const bool fit_ok = std::abs(a_fit_) > 0.4 && std::abs(a_fit_) < 1.5 &&
                      std::abs(w_fit_) > 1.4 && std::abs(w_fit_) < 2.8;
  const double a0 = fit_ok ? a_fit_ : kBigA0;
  const double w0 = fit_ok ? w_fit_ : kBigW0;
  const bool seed_fi = fit_ok && !fresh_entry_;
  const double fi0 = seed_fi ? fi_fit_ : 0.0;
  //fi 也沿用则视为已收敛，predict 中几帧后冻结
  seeded_ = seed_fi;
  fresh_entry_ = false;
  Eigen::VectorXd x0(10);
  x0 << p.ypd(0), 0.0, p.ypd(1), p.ypd(2), p.ypr(0), p.ypr(2),
        kBigSpdMean - a0, a0, w0, fi0; //spd 初值按种子的 a 给

  Eigen::VectorXd p0 = Eigen::VectorXd::Constant(10, 10.0);
  p0(6) = 100.0; //spd
  p0(9) = 400.0; //fi，初相最不确定
  ekf_ = ExtendedKalmanFilter(x0, p0.asDiagonal(), x_add_big);
  has_ekf_ = true;
  updates_ = 0; //重新开始冻结计时
}

void BigTarget::init(double nowtime, const RuneSolution & p) {
  //距上次超过 5s 当作新一次开符，fi 不沿用
  if (nowtime - lasttime_ > 5.0) fresh_entry_ = true;
  lasttime_ = nowtime;
  init_ekf(p);
}

void BigTarget::predict(double dt) {
  Eigen::VectorXd & x = ekf_.x;
  double a  = x(7);
  double w  = x(8);
  const double fi = x(9);
  if (std::abs(w) < 1e-6) w = 1e-6; //防除零
  const int cw = voter_.clockwise();
  const double t = lasttime_ + dt;

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(10, 10);
  A(0, 1) = dt;
  A(5, 6) = cw * dt;
  A(6, 7) = std::sin(w * t + fi) - 1.0; //∂spd/∂a
  A(6, 8) = t * a * std::cos(w * t + fi); //∂spd/∂w
  A(6, 9) = a * std::cos(w * t + fi); //∂spd/∂fi

  const double v1 = 0.9; //角加速度方差
  const double a1 = dt * dt * dt * dt / 4.0;
  const double b1 = dt * dt * dt / 2.0;
  const double c1 = dt * dt;
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(10, 10);
  Q(0, 0) = a1 * v1;  Q(0, 1) = b1 * v1;
  Q(1, 0) = b1 * v1;  Q(1, 1) = c1 * v1;
  Q(5, 5) = 0.09; //相位
  Q(6, 6) = 0.5; //spd
  Q(9, 9) = 1.0; //fi

  //参数收敛后冻结 spd/fi，并隔离对应协方差，避免单帧观测拖动外推
  if (updates_ > (seeded_ ? 5 : kBigFreezeAt) &&
      std::abs(x(7)) > 0.5 && std::abs(x(7)) < 1.4 &&
      std::abs(x(8)) > 1.5 && std::abs(x(8)) < 2.6) {
    Q(6, 6) = 0.0;
    Q(9, 9) = 0.0;
    for (int i : {6, 7, 8, 9}) { //交叉协方差一起清零，防止 K 从旁路改变冻结态
      ekf_.P.row(i).setZero();
      ekf_.P.col(i).setZero();
      ekf_.P(i, i) = 1e-6;
    }
  }

  //估计转速可信时按估计值推进相位(带符号)，否则按正弦模型解析积分
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
      //相位增量 = ∫spd dt = -a/w*cos(w*t+fi) + a/w*cos(w*lasttime+fi) + (2.09-a)*dt
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
  return H3 * mid * pick_rows(5, 10, {0, 2, 3, 4, 5}); //3x10
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

  //z1；大符相位噪声默认 0.5，比小符更信模型
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

  //z2
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
  omega_est_ = ekf_.x(6); //正弦模型的瞬时转速
  a_est_ = ekf_.x(7);
  w_est_ = ekf_.x(8);
  ++updates_; //冻结计时
  //更新够多才保存正弦参数，作为下次初始化的种子
  if (updates_ >= 60) {
    a_fit_ = ekf_.x(7);
    w_fit_ = ekf_.x(8);
    fi_fit_ = ekf_.x(9);
  }
  //估计转速可信才刷新，否则沿用上次可信值
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
  //成功帧清零，使 lost_cn_ 表示连续丢帧数
  lost_cn_ = 0;

  unsolvable_ = false;
  update(t, *p);

  //a、w 超出标称 ±1.5 倍判发散，下一帧重建
  const double a = ekf_.x(7), w = ekf_.x(8);
  if (!(kBigAMin / kBigDiverge <= a && a <= kBigAMax * kBigDiverge) ||
      !(kBigWMin / kBigDiverge <= w && w <= kBigWMax * kBigDiverge)) {
    first_in_ = true;
  }
}
