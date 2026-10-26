// ============================================================================
// 前哨站模型的离线自检（不依赖相机/游戏）
//
// 造一个"真值前哨站"：半径 0.275、三块板高度 0/0.102/0.204（相对最低那块）、
// 以 ±0.8π rad/s 匀速旋转。每帧按"当前正对相机的那块板"生成一个观测，
// 喂给 Target::update，看四件事：
//
//   ① 位置/高度能不能收敛到真值
//   ② 方向投票（30 个样本）能不能投出正确的顺/逆时针
//   ③ 定了方向之后 v_yaw 是不是被压成 ±0.8π
//   ④ 不发散（diverged() 一直为 false）
//
// 编译（在工程根目录）：
//   g++ -std=c++20 -O2 -I src tests/outpost_sim.cpp src/target.cpp \
//       $(pkg-config --cflags --libs opencv4) -I /usr/include/eigen3 -o /tmp/outpost_sim
// ============================================================================

#include <cmath>
#include <cstdio>
#include <random>

#include "target.hpp"

namespace {

constexpr double kDt = 0.01;          // 100 Hz
constexpr int    kFrames = 300;

// 真值状态（10 维，和 Target 的前哨站模型一致）
Eigen::VectorXd truth_state(double yaw, double v_yaw) {
  Eigen::VectorXd x(10);
  x << 2.0, 0.0,      // 右 2m
       6.0, 0.0,      // 前 6m
      -0.5, 0.0,      // 相机下方 0.5m
       yaw, v_yaw,
       kOutpostDz1, kOutpostDz2;
  return x;
}

// 独立的几何公式（不复用 h_armor_xyz，避免"自己证自己"）：
//   板位置 = 中心 + R*(cos(angle+π), sin(angle+π))，高度 = 中心高度 + dz
Eigen::Vector3d plate_pos(const Eigen::VectorXd & x, int id) {
  const double angle = x(6) + id * 2.0 * CV_PI / 3.0;
  const double dz = (id == 1) ? kOutpostDz1 : (id == 2 ? kOutpostDz2 : 0.0);
  return {x(0) + kOutpostR * std::cos(angle + CV_PI),
          x(2) + kOutpostR * std::sin(angle + CV_PI),
          x(4) + dz};
}

}  // namespace

int main(int argc, char ** argv) {
  const double v_yaw = (argc > 1 && argv[1][0] == '-') ? -kOutpostVyaw : kOutpostVyaw;
  // 第二个参数：首帧看到的是第几块板（0=底下 / 1=中间 / 2=上面）。
  // 三种初始情形应该都收敛到同一个真值 —— 验证"初始板高度不再被写死"。
  const int init_id = (argc > 2) ? std::atoi(argv[2]) : 0;

  std::mt19937 rng(12345);
  std::normal_distribution<double> noise_yaw(0.0, 0.02);    // 2mrad ≈ 0.1°
  std::normal_distribution<double> noise_dist(0.0, 0.02);   // 2cm

  Target t;
  Eigen::VectorXd x_true = truth_state(0.3, v_yaw);

  // 先建一条"首帧观测"来初始化（和 tracker 里一样：拿一块板建模型）
  {
    const Eigen::Vector3d p = plate_pos(x_true, init_id);
    Armor a;
    a.name = outpost;
    a.type = small;
    a.yaw = x_true(6) + init_id * 2.0 * CV_PI / 3.0;      // 板→中心方向（该板自己的方位角）
    a.xyz_in_camera = {p(0), -p(2), p(1)};                // pseudo → 相机排版
    Eigen::VectorXd P0_dig(10);
    P0_dig << 0.04, 0.01, 0.04, 0.01, 0.04, 0.01, 0.25, 6.25, 0.1, 0.1;
    t.init(a, kOutpostR, 3, P0_dig);
  }

  int bad = 0;
  for (int f = 0; f < kFrames; f++) {
    const double time = f * kDt;
    x_true = truth_state(0.3 + v_yaw * time, v_yaw);

    t.predict(kDt);

    // 这一帧"看到"的是最正对相机的那块板（和实车一样只看到一块）
    int best_id = 0;
    double best = 1e9;
    for (int id = 0; id < 3; id++) {
      const Eigen::Vector3d p = plate_pos(x_true, id);
      const double facing = std::abs(limit_rad(std::atan2(p(1), p(0)) -
                                               (x_true(6) + id * 2.0 * CV_PI / 3.0)));
      if (facing < best) { best = facing; best_id = id; }
    }

    const Eigen::Vector3d p = plate_pos(x_true, best_id);
    Eigen::Vector3d cam = {p(0), -p(2), p(1)};
    cam(0) += noise_yaw(rng) * cam.norm();                    // 加一点点观测噪声
    cam(2) += noise_dist(rng);
    const double face_yaw = x_true(6) + best_id * 2.0 * CV_PI / 3.0;

    Eigen::Vector4d z = Target::camera_to_obs(cam, face_yaw);
    Eigen::Vector4d R_dig;
    R_dig << 4e-3, 4e-3, 1.0, 9e-2;
    t.update(z, R_dig);

    if (t.diverged()) bad++;
  }

  const Eigen::VectorXd & x = t.ekf_.x;
  const double pos_err = std::sqrt(std::pow(x(0) - x_true(0), 2) +
                                   std::pow(x(2) - x_true(2), 2) +
                                   std::pow(x(4) - x_true(4), 2));

  // ★ 验收判据用【三块板的预测位置】而不是中心坐标：
  //   (center_z, dz1, dz2) 三个量有一个规范自由度 —— 三者一起平移
  //   【不改变任何一块板的实际位置】。首帧看到的若是中间/上面那块，滤波
  //   会收敛到平移过的那支等价解（中心数值偏、板位置照样准），拿中心坐标
  //   当判据会误杀。打出去用的就是板位置，所以直接验它。
  //   板上标号相对真值是轮转过的：est 的 id k ↔ 真值第 (init_id + k) 块板。
  double plate_err = 0.0;
  int worst_k = 0;
  for (int k = 0; k < 3; ++k) {
    const Eigen::Vector3d pe = t.h_armor_xyz(t.ekf_.x, k);
    const Eigen::Vector3d pt = plate_pos(x_true, (init_id + k) % 3);
    const double e = (pe - pt).norm();
    if (e > plate_err) { plate_err = e; worst_k = k; }
  }

  std::printf("真值 v_yaw = %+.3f rad/s (%.0f°/s)\n", v_yaw, v_yaw * 180 / M_PI);
  std::printf("首帧看到第 %d 块板；估计 dz1 / dz2 = %.3f / %.3f\n",
              init_id, x(8), x(9));
  std::printf("板预测误差(最差)    = %.3f m (id %d)\n", plate_err, worst_k);
  std::printf("中心坐标差(参考)    = %.3f m  ← 有规范自由度，数值偏不代表打偏\n", pos_err);
  std::printf("投票方向            = %s\n",
              t.outpost_dir_ == 0 ? "未定" : (t.outpost_dir_ > 0 ? "逆时针(+1)" : "顺时针(-1)"));
  std::printf("估计 yaw / 未定票数 = %.1f° / %d\n",
              x(6) * 180 / M_PI, t.dir_votes());
  std::printf("── 板位置对照（每块估计板找最近的真值板）──\n");
  for (int k = 0; k < 3; ++k) {
    const Eigen::Vector3d pe = t.h_armor_xyz(t.ekf_.x, k);
    double best = 1e18; int bj = -1;
    for (int j = 0; j < 3; ++j) {
      const double d = (pe - plate_pos(x_true, j)).norm();
      if (d < best) { best = d; bj = j; }
    }
    std::printf("  估计id%d (%+6.2f,%+6.2f,%+6.2f) → 真值板%d 距离 %.3f\n",
                k, pe(0), pe(1), pe(2), bj, best);
  }
  std::printf("估计 v_yaw          = %+.6f rad/s (真值 %+.6f，差 %.2e)\n",
              x(7), v_yaw, std::abs(x(7) - v_yaw));
  std::printf("发散帧数            = %d / %d\n", bad, kFrames);

  const bool dir_ok = (v_yaw > 0 && t.outpost_dir_ == +1) ||
                      (v_yaw < 0 && t.outpost_dir_ == -1);
  // v_yaw 允许 2% 的偏差：投票定完方向后被压成 ±0.8π，但 EKF 的更新步
  // 还会在观测的作用下微调它（不是完全冻结）。
  // 板误差阈值 0.08：稳态跟踪滞后 ≈ 5.5cm（144°/s × 0.2765m 半径 × 滤波
  // 滞后 ~0.08s）是三种初始情形共有的底噪；本测试真正的验收点是
  // 【三种初始情形结果一致且都在阈值内】—— 都靠住才算过。
  const bool ok = dir_ok && plate_err < 0.08 && bad == 0 &&
                  std::abs(x(7) - v_yaw) < 0.05;
  std::printf("%s\n", ok ? "== 通过 ==" : "== 不通过 ==");
  return ok ? 0 : 1;
}
