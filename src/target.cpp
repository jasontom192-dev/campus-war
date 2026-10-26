#include "target.hpp"
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <cstdlib>

//用单块板的首次观测反推车中心，建立车或前哨站模型
void Target::init(const Armor & armor, double radius, int armor_num,
                  const Eigen::VectorXd & P0_dig) {
  name       = armor.name;
  armor_type = armor.type;
  armor_num_ = armor_num;
  jumped     = false;
  last_id    = 0;

  double r = radius;

  //相机系(x右 y下 z前)转 pseudo 系(x右 y前 z上)
  double px =  armor.xyz_in_camera(0); //右
  double py =  armor.xyz_in_camera(2); //前
  double pz = -armor.xyz_in_camera(1); //上

  //armor.yaw 为板→车中心方位角，车中心 = 板位置 + r(cos yaw, sin yaw)
  double face_yaw = armor.yaw;

  double center_x = px + r * std::cos(face_yaw);
  double center_y = py + r * std::sin(face_yaw);
  double center_z = pz;

  //前哨站使用 10 维模型，普通车 11 维；速度初值均为 0
  is_outpost_ = (name == outpost);
  outpost_dir_ = 0;
  dir_data_.clear();

  Eigen::VectorXd x0(is_outpost_ ? 10 : 11);
  if (is_outpost_) {
    //前哨站：dz1/dz2 从 0 开始由观测学习，避免先看到的板决定整条高度基准
    x0 << center_x, 0.0,
          center_y, 0.0,
          center_z, 0.0,
          face_yaw, 0.0, //v_yaw 待方向投票后确定
          0.0, 0.0;
  } else {
    x0 << center_x, 0.0,
          center_y, 0.0,
          center_z, 0.0,
          face_yaw, 0.0,
          r, 0.0, 0.0;
  }

  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  //状态相加时回卷 yaw 分量
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c(6) = limit_rad(c(6));
    return c;
  };

  ekf_ = ExtendedKalmanFilter(x0, P0, x_add);
}

//匀速模型预测：位置加 dt·速度，yaw 加 dt·角速度
void Target::predict(double dt) {
  if (dt <= 0) return;

  //前哨站 10 维，普通车 11 维
  const int N = is_outpost_ ? 10 : 11;

  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(N, N);
  F(0, 1) = dt;
  F(2, 3) = dt;
  F(4, 5) = dt;
  F(6, 7) = dt;

  //过程噪声 Q 由连续白噪声加速度模型离散化：a=dt⁴/4，b=dt³/2，c=dt²
  double v1 = (name == outpost) ? 10.0 : 100.0;
  double v2 = (name == outpost) ? 0.1 : 400.0;

  //v1 作用于位置/速度，v2 作用于 yaw/角速度；前哨站静止，取值更小
  double a = dt * dt * dt * dt / 4.0;
  double b = dt * dt * dt / 2.0;
  double c = dt * dt;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(N, N);

  Q(0,0) = a * v1;  Q(0,1) = b * v1; //x 方向
  Q(1,0) = b * v1;  Q(1,1) = c * v1;

  Q(2,2) = a * v1;  Q(2,3) = b * v1; //y 方向
  Q(3,2) = b * v1;  Q(3,3) = c * v1;

  Q(4,4) = a * v1;  Q(4,5) = b * v1; //z 方向
  Q(5,4) = b * v1;  Q(5,5) = c * v1;

  Q(6,6) = a * v2;  Q(6,7) = b * v2; //yaw 方向
  Q(7,6) = b * v2;  Q(7,7) = c * v2;

  //r/dr/dh 视为常数，Q 为 0
  //dz1/dz2 用 dt²·kOutpostQdz 的随机游走，保留高度差的修正通道
  if (is_outpost_) {
    Q(8, 8) = c * kOutpostQdz;
    Q(9, 9) = c * kOutpostQdz;
  }

  //方向确定后，前哨站转速固定为 ±0.8π
  if (is_outpost_ && outpost_dir_ != 0) {
    ekf_.x(7) = outpost_dir_ * kOutpostVyaw;
    ekf_.P(7, 7) = 1e-4;
  }

  //三层板高差最大 0.204，dz 夹在 ±0.25
  if (is_outpost_) {
    ekf_.x(8) = std::clamp(ekf_.x(8), -0.25, 0.25);
    ekf_.x(9) = std::clamp(ekf_.x(9), -0.25, 0.25);
  }

  //线性状态转移，额外回卷 yaw
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd xp = F * x;
    xp(6) = limit_rad(xp(6));
    return xp;
  };

  ekf_.predict(F, Q, f);
}

//数据关联后执行 EKF 更新；前哨站额外过滤斜视板并累计方向投票
void Target::update(const Eigen::Vector4d & z_obs, const Eigen::Vector4d & R_dig) {

  const auto & xyza_list = armor_xyza_list();

  std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
  for (int i = 0; i < armor_num_; i++)
    xyza_i_list.push_back({xyza_list[i], i});

  //按预测板到相机的距离排序
  std::sort(xyza_i_list.begin(), xyza_i_list.end(),
    [](const auto & a, const auto & b) {
      Eigen::Vector3d v_a(a.first(0), a.first(1), a.first(2));
      Eigen::Vector3d v_b(b.first(0), b.first(1), b.first(2));
      Eigen::Vector3d ypd_a = xyz2ypd(v_a);
      Eigen::Vector3d ypd_b = xyz2ypd(v_b);
      return ypd_a(2) < ypd_b(2);
    });

  //只在最近的 3 块板中做最近邻关联，减少高速旋转时误关联到背面板
  int id = 0;
  double min_angle_error = 1e10;
  for (int i = 0; i < std::min(3, armor_num_); i++) {
    const auto & xyza = xyza_i_list[i].first;
    Eigen::Vector3d ypd_pred = xyz2ypd(xyza.head<3>());
    //比较观测 yaw 与预测方位角、观测 face_yaw 与预测法线角，角度差先回卷
    double angle_error = std::abs(limit_rad(z_obs(0) - ypd_pred(0))) +
                         std::abs(limit_rad(z_obs(3) - xyza(3)));
    if (angle_error < min_angle_error) {
      id = xyza_i_list[i].second;
      min_angle_error = angle_error;
    }
  }

  //前哨站斜视板的 PnP 位姿不可信，|face_yaw-yaw| 超过 35° 时丢弃本帧观测
  const bool op_gate_fail =
      is_outpost_ && std::abs(limit_rad(z_obs(3) - z_obs(0))) > kOutpostFacingGate;

  //OUTPOST_DEBUG=1 时打印关联 id、状态 z/dz 和四维创新
  static const bool op_debug = (std::getenv("OUTPOST_DEBUG") != nullptr);
  if (is_outpost_ && op_debug) {
    static long op_seq = 0;
    const Eigen::Vector3d xyz_p = h_armor_xyz(ekf_.x, id);
    const Eigen::Vector3d ypd_p = xyz2ypd(xyz_p);
    const double face_p = limit_rad(ekf_.x(6) + id * 2.0 * CV_PI / armor_num_);
    std::printf("#OP %ld id=%d gate=%d yaw=%8.4f z=%7.4f dz=%+.4f/%+.4f | "
                "obs %.4f %.4f %.4f %.4f | innov %+.4f %+.4f %+.4f %+.4f\n",
                op_seq++, id, op_gate_fail ? 0 : 1, limit_rad(ekf_.x(6)), ekf_.x(4),
                ekf_.x(8), ekf_.x(9), z_obs(0), z_obs(1), z_obs(2), z_obs(3),
                limit_rad(z_obs(0) - ypd_p(0)), limit_rad(z_obs(1) - ypd_p(1)),
                z_obs(2) - ypd_p(2), limit_rad(z_obs(3) - face_p));
    std::fflush(stdout);
  }

  if (op_gate_fail) return;

  //雅可比 H=∂h/∂x，在当前状态处求值
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);

  //观测噪声对角阵，由调用方按距离和角度计算
  Eigen::Matrix4d R = R_dig.asDiagonal();

  //观测模型：状态 → [yaw,pitch,dist,face_yaw]
  auto h_func = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::Vector3d xyz = h_armor_xyz(x, id);
    Eigen::Vector3d ypd = xyz2ypd(xyz);
    //第 id 块板的法线角为 yaw + id·2π/板数
    double angle = limit_rad(x(6) + id * 2.0 * CV_PI / armor_num_);
    return {ypd(0), ypd(1), ypd(2), angle};
  };

  //创新中的 yaw、pitch、face_yaw 需要回卷，dist 不回卷
  auto z_sub = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c(0) = limit_rad(c(0)); //yaw
    c(1) = limit_rad(c(1)); //pitch
    c(3) = limit_rad(c(3)); //face_yaw
    return c;
  };

  ekf_.update(z_obs, H, R, h_func, z_sub);

  //每帧给方向投票提供一个样本，已定向时函数直接返回
  if (is_outpost_) vote_direction(z_obs(3));
}

//前哨站方向投票：对间隔 3 个样本的角度差计票，超过阈值的不计
void Target::vote_direction(double orient_yaw) {
  if (outpost_dir_ != 0) return;

  dir_data_.push_back(orient_yaw);
  if (static_cast<int>(dir_data_.size()) < kOutpostDirVotes) return;

  constexpr std::size_t kInterval   = 3; //每隔 3 个样本比较一次
  constexpr double      kJumpThresh = 0.4; //角度差超过 0.4 rad 视为换板跳变

  int ccw = 0, cw = 0;
  for (std::size_t i = kInterval; i < dir_data_.size(); ++i) {
    double d = dir_data_[i] - dir_data_[i - kInterval];
    d = limit_rad(d);
    if (std::abs(d) > kJumpThresh) continue;
    if (d > 0) ++ccw; else ++cw; //角度增大为俯视逆时针
  }
  dir_data_.clear();
  dir_ccw_ = ccw;
  dir_cw_ = cw;

  if (ccw > cw)       outpost_dir_ = +1;
  else if (cw > ccw)  outpost_dir_ = -1;
  //平票时清空后重新累计
}

//返回全部板的 [x,y,z,yaw]，pseudo 系
std::vector<Eigen::Vector4d> Target::armor_xyza_list() const {
  std::vector<Eigen::Vector4d> list;
  for (int i = 0; i < armor_num_; i++) {
    //第 i 块板相对车正面的角度偏移为 i·2π/板数
    double angle = limit_rad(ekf_.x(6) + i * 2.0 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
    list.push_back({xyz(0), xyz(1), xyz(2), angle});
  }
  return list;
}

//半径估计超出 (0.05,0.5)m 或前哨站高度/转速异常时返回 true
bool Target::diverged() const {
  //前哨站：|dz| 应小于 0.5m，|v_yaw| 不超过 0.8π+1
  if (is_outpost_) {
    const bool dz_ok = std::abs(ekf_.x(8)) < 0.5 && std::abs(ekf_.x(9)) < 0.5;
    const bool w_ok  = std::abs(ekf_.x(7)) <= kOutpostVyaw + 1.0;
    return !(dz_ok && w_ok);
  }

  double r_ok = ekf_.x(8) > 0.05 && ekf_.x(8) < 0.5;
  double l_ok = ekf_.x(8) + ekf_.x(9) > 0.05 && ekf_.x(8) + ekf_.x(9) < 0.5;
  return !(r_ok && l_ok);
}

//观测模型：板位置 = 车中心 - r(cos angle,sin angle)
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const {
  double angle = limit_rad(x(6) + id * 2.0 * CV_PI / armor_num_);

  //前哨站半径固定，1、2 号板分别加 dz1、dz2
  if (is_outpost_) {
    const double h = (id == 1) ? x(8) : (id == 2 ? x(9) : 0.0);
    return {x(0) - kOutpostR * std::cos(angle),
            x(2) - kOutpostR * std::sin(angle),
            x(4) + h};
  }

  bool is_side = (armor_num_ == 4) && (id == 1 || id == 3);

  //4 块板时 id 1/3 的半径加 dr，高度加 dh
  double r   = is_side ? x(8) + x(9) : x(8);
  double a_x = x(0) - r * std::cos(angle);
  double a_y = x(2) - r * std::sin(angle);
  double a_z = is_side ? x(4) + x(10) : x(4);

  //返回 pseudo 系的 右、前、上
  return {a_x, a_y, a_z};
}

//h(x) 对状态的雅可比：H=(∂ypd/∂xyz)·(∂xyz/∂state)
Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const {
  double angle = limit_rad(x(6) + id * 2.0 * CV_PI / armor_num_);

  //前哨站为 4×10；半径常数，高度只与 z 和对应 dz 有关
  if (is_outpost_) {
    Eigen::MatrixXd H_armor(4, 10);
    H_armor.setZero();
    H_armor(0, 0) = 1.0; //∂ax/∂x
    H_armor(0, 6) = kOutpostR * std::sin(angle); //∂ax/∂yaw
    H_armor(1, 2) = 1.0; //∂ay/∂y
    H_armor(1, 6) = -kOutpostR * std::cos(angle); //∂ay/∂yaw
    H_armor(2, 4) = 1.0; //∂az/∂z
    if (id == 1) H_armor(2, 8) = 1.0; //∂az/∂dz1
    if (id == 2) H_armor(2, 9) = 1.0; //∂az/∂dz2
    H_armor(3, 6) = 1.0; //法线角对 yaw

    Eigen::MatrixXd H_ypda(4, 4);
    H_ypda.setZero();
    H_ypda.block<3, 3>(0, 0) = xyz2ypd_jacobian(h_armor_xyz(x, id));
    H_ypda(3, 3) = 1.0;
    return H_ypda * H_armor;
  }

  //普通车为 4×11；仅 id 1/3 带 dr/dh
  bool is_side = (armor_num_ == 4) && (id == 1 || id == 3);
  double r = is_side ? x(8) + x(9) : x(8);

  //∂(板位置)/∂(状态)
  double dr_dr1 = 1.0; //对 x(8)
  double dr_ddr = is_side ? 1.0 : 0.0; //对 x(9)

  double dx_dx  = 1.0;
  double dx_da  = r * std::sin(angle);
  double dx_dr1 = -std::cos(angle) * dr_dr1;
  double dx_ddr = -std::cos(angle) * dr_ddr;

  double dy_dy  = 1.0;
  double dy_da  = -r * std::cos(angle);
  double dy_dr1 = -std::sin(angle) * dr_dr1;
  double dy_ddr = -std::sin(angle) * dr_ddr;

  double dz_dz  = 1.0;
  double dz_dh  = is_side ? 1.0 : 0.0;

  Eigen::MatrixXd H_armor_xyza(4, 11);
  H_armor_xyza.setZero();

  H_armor_xyza(0, 0) = dx_dx;
  H_armor_xyza(0, 6) = dx_da;
  H_armor_xyza(0, 8) = dx_dr1;
  H_armor_xyza(0, 9) = dx_ddr;

  H_armor_xyza(1, 2) = dy_dy;
  H_armor_xyza(1, 6) = dy_da;
  H_armor_xyza(1, 8) = dy_dr1;
  H_armor_xyza(1, 9) = dy_ddr;

  H_armor_xyza(2, 4) = dz_dz;
  H_armor_xyza(2, 10)= dz_dh;

  H_armor_xyza(3, 6) = 1.0;

  //∂(球坐标)/∂(直角坐标)
  Eigen::Vector3d a_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_ypd = xyz2ypd_jacobian(a_xyz);

  //前 3 行球坐标，第 4 行 face_yaw 的系数为 1
  Eigen::MatrixXd H_ypda(4, 4);
  H_ypda.setZero();
  H_ypda.block<3,3>(0,0) = H_ypd;
  H_ypda(3, 3) = 1.0;

  return H_ypda * H_armor_xyza;
}

//直角坐标转球坐标 [yaw,pitch,dist]；pseudo 系下 pitch 为正表示在上方
Eigen::Vector3d Target::xyz2ypd(const Eigen::Vector3d & xyz) {
  double x = xyz(0), y = xyz(1), z = xyz(2);
  double yaw   = std::atan2(y, x);
  double pitch = std::atan2(z, std::sqrt(x * x + y * y));
  double dist  = std::sqrt(x * x + y * y + z * z);
  return {yaw, pitch, dist};
}

//xyz2ypd 的 3×3 雅可比；r=水平距离，d=斜距，r≈0 时分母为零
Eigen::MatrixXd Target::xyz2ypd_jacobian(const Eigen::Vector3d & xyz) {
  double x = xyz(0), y = xyz(1), z = xyz(2);
  double r2 = x * x + y * y; //水平距离平方
  double r  = std::sqrt(r2);
  double d2 = r2 + z * z; //斜距平方
  double d  = std::sqrt(d2);

  double dyaw_dx = -y / r2;
  double dyaw_dy =  x / r2;
  double dyaw_dz = 0.0;

  double dpitch_dx = -(x * z) / (d2 * r);
  double dpitch_dy = -(y * z) / (d2 * r);
  double dpitch_dz = r / d2;

  double ddist_dx = x / d;
  double ddist_dy = y / d;
  double ddist_dz = z / d;

  Eigen::MatrixXd J(3, 3);
  J << dyaw_dx,  dyaw_dy,  dyaw_dz,
       dpitch_dx, dpitch_dy, dpitch_dz,
       ddist_dx,  ddist_dy,  ddist_dz;
  return J;
}

//相机系坐标和板法线角转为 [yaw,pitch,dist,face_yaw]
Eigen::Vector4d Target::camera_to_obs(const Eigen::Vector3d & cam_xyz, double face_yaw) {

  //相机系(x右 y下 z前)转 pseudo 系(x右 y前 z上)
  double px =  cam_xyz(0);
  double py =  cam_xyz(2);
  double pz = -cam_xyz(1);

  Eigen::Vector3d p(px, py, pz);
  Eigen::Vector3d ypd = xyz2ypd(p);
  return {ypd(0), ypd(1), ypd(2), face_yaw};
}

//pseudo 系 → 相机系
Eigen::Vector3d Target::pseudo_to_camera(const Eigen::Vector3d & p) {
  return {p(0), -p(2), p(1)};
}
