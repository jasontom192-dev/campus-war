#include "target.hpp"
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <cstdlib>

// ============================================================================
// 初始化：用一块装甲板的首次观测，反推出【车中心】的位置
//
// 输入只有一块板的观测（板中心在相机系的位置 + 板法线角），
// 但要初始化的是【整个车】的状态。怎么做？
//
// 这里是坐标系最容易看晕的地方，先理清楚：
//
//   相机系 (cam):  x 右,  y 下,  z 前         <- solvePnP 输出的系
//   pseudo 系 (p): x 右,  y 前,  z 上         <- EKF 状态用的系
//
//   p(0) =  cam(0)   = 右    (相机系 x 轴)
//   p(1) =  cam(2)   = 前    (相机系 z 轴)
//   p(2) = -cam(1)   = 上    (相机系 y 轴的反向)
//
//   所以下面三行是：相机系 → pseudo 系的映射 —— 只是重排分量，没有平移，
//   原点仍然是相机光心。
//   这么换是为了把竖直轴单独拎成一个分量（p(2)）、让水平面落在前两个
//   下标上，yaw 才好解释成绕竖直轴的转角（相机系里竖直是 y、深度是 z，
//   直接拿状态来排就会交错）。
//
// 几何关系（水平面内）：armor.yaw 是【从板指向车中心】的方位角，所以
//   车中心 = 板位置 + r * (cos(armor.yaw), sin(armor.yaw))
//   （板在车的外圈，从板往车中心走 r 就到达车中心）
// ============================================================================
void Target::init(const Armor & armor, double radius, int armor_num,
                  const Eigen::VectorXd & P0_dig) {
  name       = armor.name;
  armor_type = armor.type;
  armor_num_ = armor_num;
  jumped     = false;
  last_id    = 0;

  double r = radius;

  // 相机系 → pseudo 系（见上面的大注释）
  double px =  armor.xyz_in_camera(0);     // 右
  double py =  armor.xyz_in_camera(2);     // 前
  double pz = -armor.xyz_in_camera(1);     // 上

  // armor.yaw 是【从板指向车中心】的方向在水平面内的方位角
  // （solver.cpp:31 取旋转矩阵第 0 列的 atan2，得到的就是这个方向）。
  // ★ 它和模型里那块板的 angle = x(6) + id*2π/armor_num_ 是同一个量：
  //   h_armor_xyz 里 plate = 车中心 - r*(cos(angle), sin(angle))，符号配套。
  double face_yaw = armor.yaw;

  // 从板出发，沿 (cos, sin) 方向走 r，到达车中心
  double center_x = px + r * std::cos(face_yaw);
  double center_y = py + r * std::sin(face_yaw);
  double center_z = pz;                    // 竖直方向直接用板的高度

  // ---------------------------------------------------------------
  // 11 维状态向量的初值
  //   [0]x [1]vx [2]y [3]vy [4]z [5]vz [6]yaw [7]omega [8]r [9]dr [10]dh
  // 位置三项用刚算出来的值；速度、角速度全部置 0（先假设静止，
  // 靠后面的观测把速度收敛出来）；r 用传入的先验值
  // ---------------------------------------------------------------
  // 前哨站走 10 维的专用模型（target.hpp 顶部有状态定义）
  is_outpost_ = (name == outpost);
  outpost_dir_ = 0;
  dir_data_.clear();

  Eigen::VectorXd x0(is_outpost_ ? 10 : 11);
  if (is_outpost_) {
    //   [x, vx, y, vy, z, vz, yaw, v_yaw, dz1, dz2]
    // ★ dz1/dz2 初值给 0（照 auto_aim_rps26 的 outpost_model）：不再把
    //   "先看到的那块是底下那块（另外两块 +0.102/+0.204）"写死当先验。
    //   原因：第一帧看到的是哪块高度的板并不知道；先看到的若是中间/上面
    //   那块，写死"其余更高"会把高度基准钉偏 0.1~0.2 m，而 dz 的过程噪声
    //   极小、几乎纠不回来 —— 结果就是"初始看到哪块板 → 哪一套高度基准"，
    //   瞄准高度出现几种不同的固定偏差（实机反馈的"三种预测状态"）。
    //   给 0 让滤波自己学：三种初始情形对应 (+0.102,+0.204)、(−0.102,+0.102)、
    //   (−0.204,−0.102)，全在 predict 的 ±0.25 夹取范围内，都能自洽收敛。
    x0 << center_x, 0.0,
          center_y, 0.0,
          center_z, 0.0,
          face_yaw, 0.0,          // v_yaw 先给 0，等方向投票定（见 vote_direction）
          0.0, 0.0;
  } else {
    x0 << center_x, 0.0,
          center_y, 0.0,
          center_z, 0.0,
          face_yaw, 0.0,
          r, 0.0, 0.0;
  }

  // 初始协方差：只给对角元（各态相互独立），非对角元为 0
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 状态相加时必须处理角度回卷 —— 传给 EKF 做 x = x + K*创新
  // 否则会出现 "179° + 2° = 181°" 而不折回 -179° 的经典 bug
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c(6) = limit_rad(c(6));       // 只对 yaw 分量回卷
    return c;
  };

  ekf_ = ExtendedKalmanFilter(x0, P0, x_add);
}

// ============================================================================
// EKF 预测步
//
// 状态按【匀速模型】(CV, Constant Velocity) 往前走：
//   位置 += 速度 * dt
//   角度 += 角速度 * dt
//   速度、角速度、半径参数保持不变
//
// 对应的状态转移矩阵 F 就是"对角线上全是 1，再把速度和位置耦合上"
// ============================================================================
void Target::predict(double dt) {
  if (dt <= 0) return;              // 防御：dt 非法则不动

  // 前哨站：状态 10 维（见 target.hpp 顶部），但状态转移完全一样 ——
  // 都是匀速模型，只是没有 r/dr，多两个高度增量 dz1/dz2（它们是常数）。
  const int N = is_outpost_ ? 10 : 11;

  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(N, N);
  F(0, 1) = dt;    // x   += vx * dt
  F(2, 3) = dt;    // y   += vy * dt
  F(4, 5) = dt;    // z   += vz * dt
  F(6, 7) = dt;    // yaw += omega * dt

  // ---------------------------------------------------------------
  // 过程噪声 Q：标准的"连续时间白噪声加速度模型"离散化
  //
  // 假设加速度是一个方差为 sigma^2 的白噪声，积分到位置/速度上， 白噪声：加速度噪声（也就是加速度的不确定值）的方差
  // 就得到 2x2 的块：
  //
  //     [ dt^4/4   dt^3/2 ]          <- 位置行、位置列
  //     [ dt^3/2   dt^2   ] * sigma^2
  //
  // 所以下面的 a/b/c 就是这么来的：
  //     a = dt^4/4  (位置-位置) //位置自身的方差
  //     b = dt^3/2  (位置-速度) //位置和速度的协方差
  //     c = dt^2    (速度-速度) //速度自身的方差
  //
  // Q 的含义："我的匀速假设有多不可信"。Q 调大 -> 更信观测（反应快但抖），
  // Q 调小 -> 更信任模型（平滑但跟不上机动）。
  // ---------------------------------------------------------------
  double v1 = (name == outpost) ? 10.0 : 100.0;
  double v2 = (name == outpost) ? 0.1 : 400.0;

  // ★ 前哨站的过程噪声故意比普通机器人小两个数量级 —— 因为前哨站是
  //   【静止】的，而机器人会机动。这是个按兵种手工调的参数。
  //   v1 管位置/速度三项，v2 管 yaw/omega 那一项。

  double a = dt * dt * dt * dt / 4.0;
  double b = dt * dt * dt / 2.0;
  double c = dt * dt;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(N, N);

  Q(0,0) = a * v1;  Q(0,1) = b * v1;      // x 方向
  Q(1,0) = b * v1;  Q(1,1) = c * v1;

  Q(2,2) = a * v1;  Q(2,3) = b * v1;      // y 方向
  Q(3,2) = b * v1;  Q(3,3) = c * v1;

  Q(4,4) = a * v1;  Q(4,5) = b * v1;      // z 方向
  Q(5,4) = b * v1;  Q(5,5) = c * v1;

  Q(6,6) = a * v2;  Q(6,7) = b * v2;      // yaw 方向
  Q(7,6) = b * v2;  Q(7,7) = c * v2;

  // ★ 半径 r、dr、dh 三项的 Q 为 0：假设它们是【常数】。
  //   合理，因为车的几何尺寸不会变。
  //
  // 前哨站的 dz1/dz2（x(8)、x(9)）不一样：准常数，但必须留一条【真实可用】
  // 的修正通道 —— 三块板的高度差要靠它从观测里学出来。
  //
  // ★ 2026-10-04 修（实机日志定案）：原来用的是 dt⁴/4（"a"，位置类噪声的
  //   尺度）：120Hz 下每帧只给 dz 增加 1.2e-8 的方差。开局靠 P0=0.1 学到
  //   一个粗糙值之后，协方差被观测压塌、Q 又补不回来 → 增益≈0，dz 彻底
  //   冻死（实测：460 帧只动 0.0002，而三块板的观测俯仰差着 ~0.1 rad）。
  //   后果：模型的三块板"几乎同高"，只能让 z 逐块去贴当前被观测的板 ——
  //   z 在 0.44~0.85 之间来回甩，"观测量哪块板就是哪种预测"（三种状态）。
  //   这也解释了时好时坏：开局几帧观测好，dz 学到像样的值就是"好的那种"。
  //   改法：用随机游走尺度 dt²·kOutpostQdz（稳衡增益 ~0.07，半秒级收敛、
  //   收敛后 ~2cm 抖动）。dz 没有速度分量，不乘 dt³/dt⁴ 的耦合项。
  if (is_outpost_) {
    Q(8, 8) = c * kOutpostQdz;
    Q(9, 9) = c * kOutpostQdz;
  }

  // -----------------------------------------------------------------
  // 前哨站：投票定了方向之后，转速就是已知常数 ±0.8π —— 直接写死，
  // 不让 EKF 再自由估（前哨站只有顺/逆时针两种，没有"转速是别的值"这种
  // 可能）。同时把它的方差压小，免得观测噪声把它拽走。
  // 方向还没定时（outpost_dir_ == 0）不碰，让 EKF 自己先收敛着。
  // -----------------------------------------------------------------
  if (is_outpost_ && outpost_dir_ != 0) {
    ekf_.x(7) = outpost_dir_ * kOutpostVyaw;
    ekf_.P(7, 7) = 1e-4;
  }

  // ★ dz1/dz2 每帧夹在 ±0.25（照 rps26 outpost_model.cpp:202-205）：
  //   三层板相对高度最多 0.204，±0.25 给足余量；初始基准未知时 dz 可以
  //   是负的（先看到的是中间/上面那块），夹取防止观测噪声把高度差带飞。
  //   （顺带：diverged() 里 |dz|<0.5 的判据因此不会误报 —— 夹取后到不了 0.5。）
  if (is_outpost_) {
    ekf_.x(8) = std::clamp(ekf_.x(8), -0.25, 0.25);
    ekf_.x(9) = std::clamp(ekf_.x(9), -0.25, 0.25);
  }

  // 非线性的状态转移函数（本项目里其实就是线性的 F*x，只是要多做一次角度回卷）
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd xp = F * x;
    xp(6) = limit_rad(xp(6));        // yaw 回卷
    return xp;
  };

  ekf_.predict(F, Q, f);
}

// ============================================================================
// EKF 更新步 —— 整个文件最绕的一段
//
// 要解决的核心问题：【数据关联】
//   模型内部有 4 块板，但这一帧只看到 1 块。看到的到底是第几块？
//   因为每块板的角度位置不同（yaw + id*π/2），关联错了更新就会把状态带偏。
//
// 这里的做法是"最近邻"：
//   对模型预测的每块板，算出"如果看到它，观测应该长什么样"，
//   和实际观测比较，谁最接近就认为是它。
// ============================================================================
void Target::update(const Eigen::Vector4d & z_obs, const Eigen::Vector4d & R_dig) {

  const auto & xyza_list = armor_xyza_list();

  std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
  for (int i = 0; i < armor_num_; i++)
    xyza_i_list.push_back({xyza_list[i], i});

  // 按"到相机的距离"从近到远排序
  // ★ 这里比较的是 xyz2ypd(...)(2)，也就是球坐标的第三个分量 = 距离。
  //   写成 (2) 而不是直接 norm()，是因为 xyz2ypd 已经顺带算出来了。
  std::sort(xyza_i_list.begin(), xyza_i_list.end(),
    [](const auto & a, const auto & b) {
      Eigen::Vector3d v_a(a.first(0), a.first(1), a.first(2));
      Eigen::Vector3d v_b(b.first(0), b.first(1), b.first(2));
      Eigen::Vector3d ypd_a = xyz2ypd(v_a);
      Eigen::Vector3d ypd_b = xyz2ypd(v_b);
      return ypd_a(2) < ypd_b(2);
    });

  // 在前 3 近的板里，找"如果看到它，观测该长什么样"最接近实际观测的那块
  //
  // ★ 只在前 3 块里找，是个【刻意的启发式限制】：
  //   如果 4 块全找，会出现"车背面那块板的预测观测恰好也对得上"的情况，
  //   （尤其当车转速很快、或者 yaw 还没收敛时），一旦关联到背面的板，
  //   状态就会被狠狠地拽偏。限制成 3 块是降低这种风险的经验做法。
  //   ⚠ 代价：如果真实能看到的板恰好落在第 4 近，关联就会错。
  int id = 0;
  double min_angle_error = 1e10;
  for (int i = 0; i < std::min(3, armor_num_); i++) {
    const auto & xyza = xyza_i_list[i].first;
    Eigen::Vector3d ypd_pred = xyz2ypd(xyza.head<3>());
    // 两个角度误差相加：
    //   ① 观测的 yaw  vs 预测板的方位角
    //   ② 观测的 face_yaw vs 预测板的法线角
    //   注意两个比较都要 limit_rad 归一化，否则靠近 ±π 分界时会误判
    double angle_error = std::abs(limit_rad(z_obs(0) - ypd_pred(0))) +
                         std::abs(limit_rad(z_obs(3) - xyza(3)));
    if (angle_error < min_angle_error) {
      id = xyza_i_list[i].second;
      min_angle_error = angle_error;
    }
  }

  // ---- 前哨站观测门：只收"基本正面朝我们"的板（照 auto_aim_rps26 的
  // outpost_model 的 matched 前置门）----
  // 原来关联到的观测无条件喂滤波器。实机在前哨站跟前（2~4m——为了进 4m
  // 开火门必须贴上去打）观测里混着大量斜视板的垃圾位姿，直接把状态拽炸
  // （dz 冲 1e9/NaN、v_yaw 飞到 ±11、发散重建死循环）。
  // 判据：|板法线角 z3 − 板自身方位角 z0| < 35°。
  //   z3 是"板→中心"方向（正面板上它≈从相机看板的方位角）；背面/侧面板
  //   一个差 ~180° 一个差 ~90°，斜视板的 PnP yaw/位置本身就不可信。
  // ★ 只做这一道门：rps26 的另外两道（位置匹配门 + 连续拒→软重置）连位置
  //   关联一起搬过来会把 sim 从 6/6 打到 3/6（关联在相位未定时滑板 + 门把
  //   纠偏观测也拒掉的局部自洽解），先不上——留档在 rps26 outpost_model.cpp。
  const bool op_gate_fail =
      is_outpost_ && std::abs(limit_rad(z_obs(3) - z_obs(0))) > kOutpostFacingGate;

  // ---- 前哨站逐帧调试（环境变量 OUTPOST_DEBUG=1 时开；排查用，可拆）----
  // 打：关联到哪块 id / 状态 (z, dz1, dz2) / 本帧观测 / 创新（观测 − 预测）。
  // 排查"观测不同板 → 预测质量不同"（见 memory: tdt-outpost-three-state），
  // 三种机制的区分：滑板=id 相对物理相位跳；未观测 dz 漂移=某块转出来时创新骤大；
  // 板间观测质量差=按 id 分组的创新系统性偏大。
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

  // 雅可比 H = ∂h/∂x，在【当前状态】处求值，4×11
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);

  // 观测噪声的对角阵（由调用方按距离/角度算好传进来，见 tracker.cpp:167）
  Eigen::Matrix4d R = R_dig.asDiagonal();

  // 观测函数 h(x)：状态 → 预期观测 [yaw, pitch, dist, face_yaw]
  auto h_func = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::Vector3d xyz = h_armor_xyz(x, id);
    Eigen::Vector3d ypd = xyz2ypd(xyz);
    // 第 id 块板的法线角 = 车朝向 + id 分到的角度间隔（绕一圈是 2π）
    double angle = limit_rad(x(6) + id * 2.0 * CV_PI / armor_num_);
    return {ypd(0), ypd(1), ypd(2), angle};
  };

  // 创新 = 观测 - 预测观测。三个角度分量都要回卷，
  // 否则 179° 和 -179° 相减会算成 358° 的误差（其实是 2°）
  auto z_sub = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c(0) = limit_rad(c(0));    // yaw
    c(1) = limit_rad(c(1));    // pitch
    c(3) = limit_rad(c(3));    // face_yaw
    // ★ c(2) = dist 不做回卷，它是长度不是角度
    return c;
  };

  ekf_.update(z_obs, H, R, h_func, z_sub);

  // 前哨站：每帧喂一个方向投票样本（定了方向之后它自己会直接返回）
  if (is_outpost_) vote_direction(z_obs(3));
}

// ============================================================================
// 前哨站旋转方向投票（参考 auto_aim_rps26 的 OutpostModel::calDirection）
//
// 前哨站只有【顺时针/逆时针】两种状态，所以这里不投"静止"那一档。
// 做法：攒 kOutpostDirVotes 个"观测到的板法线角"，逐对比较间隔 3 个样本的
// 角度差，按符号计票；差得太大（换板了、关联跳了）的不计票。
// 投出方向后写死 v_yaw = ±0.8π（predict 里每帧都会重新压住）。
// ============================================================================
void Target::vote_direction(double orient_yaw) {
  if (outpost_dir_ != 0) return;            // 方向已定，不再投

  dir_data_.push_back(orient_yaw);
  if (static_cast<int>(dir_data_.size()) < kOutpostDirVotes) return;

  constexpr std::size_t kInterval   = 3;    // 隔几个样本比较一次
  constexpr double      kJumpThresh = 0.4;  // 角度差超过它 = 跳变（换板），不计票

  int ccw = 0, cw = 0;
  for (std::size_t i = kInterval; i < dir_data_.size(); ++i) {
    double d = dir_data_[i] - dir_data_[i - kInterval];
    d = limit_rad(d);                       // 归一化到 (-π, π]
    if (std::abs(d) > kJumpThresh) continue;
    if (d > 0) ++ccw; else ++cw;            // 角度增大 = 俯视逆时针
  }
  dir_data_.clear();
  dir_ccw_ = ccw;
  dir_cw_ = cw;

  if (ccw > cw)       outpost_dir_ = +1;    // 逆时针
  else if (cw > ccw)  outpost_dir_ = -1;    // 顺时针
  // 平票（几乎不会出现）：什么都不做，下一轮重新攒样本再投
}

// ============================================================================
// 按当前状态，算出所有板的位置和朝向（pseudo 系）
// 返回 armor_num_ 个 [x, y, z, yaw]
// 用途：数据关联时预测"应该看到什么"、可视化画预测框、Aimer 选板
// ============================================================================
std::vector<Eigen::Vector4d> Target::armor_xyza_list() const {
  std::vector<Eigen::Vector4d> list;
  for (int i = 0; i < armor_num_; i++) {
    // 第 i 块板相对车正面的角度偏移 = i * (2π / 板数)
    double angle = limit_rad(ekf_.x(6) + i * 2.0 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
    list.push_back({xyz(0), xyz(1), xyz(2), angle});
  }
  return list;
}

// ============================================================================
// 发散检测
//
// 判据：半径估计是否还在合理区间。真车的半径在 0.15~0.25 左右，
// 这里放宽到 (0.05, 0.5) 都算正常，跑出这个范围说明滤波器已经跟丢了。
//   - r_ok   : 正面板的半径
//   - l_ok   : 正面板半径 + 侧面增量（即侧面板的半径），也要在区间内
//
// 上层用法：main.cpp:89 检查后才开火，visualizer.cpp:18,55 检查后才画预测
// ============================================================================
bool Target::diverged() const {
  // 前哨站没有 r/dr（半径固定），改判两个高度增量和转速是否离谱：
  // 板间高差只有 0.1m 量级，跑出 ±0.5m 说明已经跟丢了。
  if (is_outpost_) {
    const bool dz_ok = std::abs(ekf_.x(8)) < 0.5 && std::abs(ekf_.x(9)) < 0.5;
    const bool w_ok  = std::abs(ekf_.x(7)) <= kOutpostVyaw + 1.0;
    return !(dz_ok && w_ok);
  }

  double r_ok = ekf_.x(8) > 0.05 && ekf_.x(8) < 0.5;
  double l_ok = ekf_.x(8) + ekf_.x(9) > 0.05 && ekf_.x(8) + ekf_.x(9) < 0.5;
  return !(r_ok && l_ok);
}

// ============================================================================
// 【观测模型的核心】h_armor_xyz —— 给定状态，算第 id 块板在哪
//
// 几何：板在车的"外圈"上，从车中心沿 angle 方向走 r 就到达板。
//       所以  板位置 = 车中心 - r * (cos angle, sin angle)
//       （公式里写的是 x(0) - r*cos，等价）
//
// 4 块板布局的特殊性（只有 armor_num_ == 4 时才有）：
//   奇数编号的两块（id 1、3，车的左右两侧）的半径和高度与 0/2（前/后）
//   不同，模型里用 x(9) = dr（半径增量）和 x(10) = dh（高度增量）来描述。
//
//   ★ 这里只有一个几何约定值得记牢：4 块板时【id 1 和 3 才是加 dr/dh 的那两块】。
//     写成 id == 1 || id == 3、写成 id % 2 == 1、或者用文字说"左右两侧"，
//     说的都是同一件事（该行代码从 4.2 起就没变过）。
//   ★ 板位置的符号是【中心 − r·(cos, sin)】：和 init() 里"中心 = 板 + r·(cos, sin)"
//     互为反解，也就和观测的 face_yaw（方向是"板 → 车中心"）自洽。
//     改动这个小减号会让整条链路的方向反掉。
// ============================================================================
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const {
  double angle = limit_rad(x(6) + id * 2.0 * CV_PI / armor_num_);

  // 前哨站：半径固定（不估），三块板高度不同 —— 0 号板在 x(4)，
  // 1、2 号板分别抬高 dz1、dz2（x(8)、x(9)）。
  // 参考 auto_aim_rps26 的 OutpostMeasure。
  if (is_outpost_) {
    const double h = (id == 1) ? x(8) : (id == 2 ? x(9) : 0.0);
    return {x(0) - kOutpostR * std::cos(angle),
            x(2) - kOutpostR * std::sin(angle),
            x(4) + h};
  }

  bool is_side = (armor_num_ == 4) && (id == 1 || id == 3);

  // 左右两块的半径要加上增量 dr；高度同理加 dh
  double r   = is_side ? x(8) + x(9) : x(8);
  double a_x = x(0) - r * std::cos(angle);
  double a_y = x(2) - r * std::sin(angle);
  double a_z = is_side ? x(4) + x(10) : x(4);

  // 注意返回顺序是 (x, y, z)，在 pseudo 系下分别是 右、前、上
  return {a_x, a_y, a_z};
}

// ============================================================================
// h(x) 对状态 x 的雅可比（4×11）—— EKF 的"线性化"就在这一步
//
// 这是个链式法则的结果，拆成两段乘：
//
//   H = ∂[yaw,pitch,dist,face_yaw]/∂[x,y,z,yaw,...]
//     = (∂ypd/∂xyz) · (∂xyz/∂状态)
//        └── H_ypd ──┘   └─ H_armor_xyza ─┘
//        (3×3 球坐标)     (4×11 几何关系)
//
// 然后再把第 4 行（face_yaw 对 yaw 的导数，恒为 1）接上去。
// ============================================================================
Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const {
  double angle = limit_rad(x(6) + id * 2.0 * CV_PI / armor_num_);

  // 前哨站：4×10。板位置 = 中心 - R*(cos,sin)（R 是常数，导数为 0），
  // 高度只跟 z 和该块板自己的 dz 有关。
  if (is_outpost_) {
    Eigen::MatrixXd H_armor(4, 10);
    H_armor.setZero();
    H_armor(0, 0) = 1.0;                              // ∂ax/∂x
    H_armor(0, 6) = kOutpostR * std::sin(angle);      // ∂ax/∂yaw
    H_armor(1, 2) = 1.0;                              // ∂ay/∂y
    H_armor(1, 6) = -kOutpostR * std::cos(angle);     // ∂ay/∂yaw
    H_armor(2, 4) = 1.0;                              // ∂az/∂z
    if (id == 1) H_armor(2, 8) = 1.0;                 // ∂az/∂dz1
    if (id == 2) H_armor(2, 9) = 1.0;                 // ∂az/∂dz2
    H_armor(3, 6) = 1.0;                              // 板法线角 = yaw + id*2π/3

    Eigen::MatrixXd H_ypda(4, 4);
    H_ypda.setZero();
    H_ypda.block<3, 3>(0, 0) = xyz2ypd_jacobian(h_armor_xyz(x, id));
    H_ypda(3, 3) = 1.0;
    return H_ypda * H_armor;
  }

  // 同上：4 块板时只有 id 1/3 带 dr/dh（3 块板的前哨站已在上面单独返回）
  bool is_side = (armor_num_ == 4) && (id == 1 || id == 3);
  double r = is_side ? x(8) + x(9) : x(8);

  // ---- 第一段：∂(板位置)/∂(状态) ----
  // 板位置 = 车中心 - r*(cos angle, sin angle)，对各项分别求偏导
  // （x(8)=r 与 x(9)=dr 是"基本半径 + 增量"，is_side 时 r 里同时含两者）

  // 板位置对半径 r 的偏导：只有 is_side 时才要额外对 dr 求导
  double dr_dr1 = 1.0;                       // 对 x(8)
  double dr_ddr = is_side ? 1.0 : 0.0;       // 对 x(9)

  double dx_dx  = 1.0;                              // ∂ax/∂x  = 1
  double dx_da  = r * std::sin(angle);              // ∂ax/∂yaw = +r*sin
  double dx_dr1 = -std::cos(angle) * dr_dr1;        // ∂ax/∂r
  double dx_ddr = -std::cos(angle) * dr_ddr;        // ∂ax/∂dr

  double dy_dy  = 1.0;                              // ∂ay/∂y  = 1
  double dy_da  = -r * std::cos(angle);             // ∂ay/∂yaw = -r*cos
  double dy_dr1 = -std::sin(angle) * dr_dr1;
  double dy_ddr = -std::sin(angle) * dr_ddr;

  double dz_dz  = 1.0;                              // ∂az/∂z  = 1
  double dz_dh  = is_side ? 1.0 : 0.0;              // ∂az/∂dh

  // 4 行分别对应 [ax, ay, az, angle]，11 列对应状态
  Eigen::MatrixXd H_armor_xyza(4, 11);
  H_armor_xyza.setZero();

  H_armor_xyza(0, 0) = dx_dx;     // ax 对 x
  H_armor_xyza(0, 6) = dx_da;     // ax 对 yaw
  H_armor_xyza(0, 8) = dx_dr1;    // ax 对 r
  H_armor_xyza(0, 9) = dx_ddr;    // ax 对 dr

  H_armor_xyza(1, 2) = dy_dy;     // ay 对 y
  H_armor_xyza(1, 6) = dy_da;     // ay 对 yaw
  H_armor_xyza(1, 8) = dy_dr1;    // ay 对 r
  H_armor_xyza(1, 9) = dy_ddr;    // ay 对 dr

  H_armor_xyza(2, 4) = dz_dz;     // az 对 z
  H_armor_xyza(2, 10)= dz_dh;     // az 对 dh

  H_armor_xyza(3, 6) = 1.0;       // 板朝向角对车 yaw 的导数恒为 1

  // ---- 第二段：∂(球坐标)/∂(直角坐标)，3×3 ----
  Eigen::Vector3d a_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_ypd = xyz2ypd_jacobian(a_xyz);

  // 拼成 4×4：前 3 行是球坐标，第 4 行是 face_yaw（它只跟 yaw 有关，
  // 而那部分已经在 H_armor_xyza 的第 4 行里了，所以这里只放一个 1）
  Eigen::MatrixXd H_ypda(4, 4);
  H_ypda.setZero();
  H_ypda.block<3,3>(0,0) = H_ypd;
  H_ypda(3, 3) = 1.0;

  // ---- 链式法则：两段相乘，得到最终的 4×11 ----
  return H_ypda * H_armor_xyza;
}

// ============================================================================
// 直角坐标 → 球坐标
//   yaw   = atan2(y, x)              水平方位角（绕竖直轴）
//   pitch = atan2(z, sqrt(x²+y²))    俯仰角
//   dist  = sqrt(x²+y²+z²)           距离
//
// ★ 在 pseudo 系里 x-y 是水平面、第三个分量是"上"（见 target.cpp:13-18），
//   所以 pitch 为正表示目标在【上方】。
//   注意这里没有 limit_rad(yaw)，因为 atan2 的输出本来就在 (-π, π]
// ============================================================================
Eigen::Vector3d Target::xyz2ypd(const Eigen::Vector3d & xyz) {
  double x = xyz(0), y = xyz(1), z = xyz(2);
  double yaw   = std::atan2(y, x);
  double pitch = std::atan2(z, std::sqrt(x * x + y * y));
  double dist  = std::sqrt(x * x + y * y + z * z);
  return {yaw, pitch, dist};
}

// ============================================================================
// 上面那个变换的雅可比（3×3）
//
// 就是对三个分量分别求偏导，纯微积分，没有算法：
//
//   ∂yaw/∂x = -y/(x²+y²)        ∂yaw/∂y = x/(x²+y²)
//   ∂pitch/∂x = -(x·z)/(d²·r)   ∂pitch/∂y = -(y·z)/(d²·r)   ∂pitch/∂z = r/d²
//   ∂dist/∂x = x/d              ∂dist/∂y = y/d              ∂dist/∂z = z/d
//
//   其中 r = sqrt(x²+y²)（水平距离），d = sqrt(x²+y²+z²)（斜距）
//
// ★ 注意 r 出现在分母上，所以目标接近正上下方（x≈y≈0）时会除零。
//   实际场景里装甲板基本都在水平面附近，不会触发。
// ============================================================================
Eigen::MatrixXd Target::xyz2ypd_jacobian(const Eigen::Vector3d & xyz) {
  double x = xyz(0), y = xyz(1), z = xyz(2);
  double r2 = x * x + y * y;      // 水平距离的平方
  double r  = std::sqrt(r2);
  double d2 = r2 + z * z;         // 斜距的平方
  double d  = std::sqrt(d2);

  double dyaw_dx = -y / r2;
  double dyaw_dy =  x / r2;
  double dyaw_dz = 0.0;           // yaw 与高度无关

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

// ============================================================================
// 相机系 → 观测向量
//
// 输入是 solvePnP 给出的板中心（相机系）和板法线角，
// 输出 EKF 用的 4 维观测 [yaw, pitch, dist, face_yaw]。
//
// ★ 注意这里做了【相机系 → pseudo 系】的转换，和 init() 里那三行是
//   完全一样的映射（p = (cam.x, cam.z, -cam.y)）。
//   EKF 的状态在 pseudo 系，所以观测也必须换到同一个系才能比较。
// ============================================================================
Eigen::Vector4d Target::camera_to_obs(const Eigen::Vector3d & cam_xyz, double face_yaw) {

  double px =  cam_xyz(0);      // 右
  double py =  cam_xyz(2);      // 前
  double pz = -cam_xyz(1);      // 上

  Eigen::Vector3d p(px, py, pz);
  Eigen::Vector3d ypd = xyz2ypd(p);
  return {ypd(0), ypd(1), ypd(2), face_yaw};
}

// ============================================================================
// pseudo 系 → 相机系（上面那个映射的逆变换）
//   cam.x =  p.x
//   cam.y = -p.z
//   cam.z =  p.y
// ============================================================================
Eigen::Vector3d Target::pseudo_to_camera(const Eigen::Vector3d & p) {
  return {p(0), -p(2), p(1)};
}
