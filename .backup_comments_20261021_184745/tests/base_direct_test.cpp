// ============================================================================
// 基地"直瞄"自检：Aimer::best_aim_direct 的弹道解和重力补偿对不对
//
// 直瞄的数学是解析的，可以直接手算对：
//   v0=25 m/s、水平距离 d=5m、目标比枪口低 1m（h=-1）
//     a      = g·d²/(2v0²)          = 0.19567
//     tanθ   = (d - √(d²-4a(a+h)))/(2a)   ← 取低伸解
//     t      = d/(v0·cosθ)
//     下坠   = ½·g·t²
//   aim_3d 应该是板中心抬高"下坠"那么多
//
// 编译（工程根目录）：
//   g++ -std=c++20 -O2 -Wno-deprecated-enum-enum-conversion -I src \
//       tests/base_direct_test.cpp src/aimer.cpp src/solver.cpp src/target.cpp \
//       $(pkg-config --cflags --libs opencv4) -I /usr/include/eigen3 -o /tmp/base_direct_test
// ============================================================================

#include <cmath>
#include <cstdio>

#include "aimer.hpp"

int main() {
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = 1159.420;
  K.at<double>(1, 1) = 1159.420;
  K.at<double>(0, 2) = 480.0;
  K.at<double>(1, 2) = 360.0;
  cv::Mat D = cv::Mat::zeros(1, 5, CV_64F);
  Solver solver(K, D);

  const double v0 = 25.0, d = 5.0, h = -1.0, g = 9.7833;
  Aimer aimer(v0);

  // 状态系（右/前/上）：正前方 5m、低 1m
  const Eigen::Vector3d xyz(0.0, d, h);

  // ★ 换系往返：Armor::xyz_in_camera 是【相机排版】(右/下/前)，
  //   节点里要先转成状态系才能喂给 best_aim_direct。这里把这一步也测了：
  //     相机排版 = (右, -上, 前)      状态系 = (右, 前, 上) = (x, z, -y)
  const Eigen::Vector3d cam_layout(xyz(0), -xyz(2), xyz(1));
  const Eigen::Vector3d back(cam_layout(0), cam_layout(2), -cam_layout(1));
  if ((back - xyz).norm() > 1e-12) {
    std::printf("换系往返失败: (%.3f,%.3f,%.3f) → (%.3f,%.3f,%.3f)\n",
                xyz(0), xyz(1), xyz(2), back(0), back(1), back(2));
    return 1;
  }

  const Aimer::AimResult r = aimer.best_aim_direct(back, solver);

  // 手算对照
  const double a = g * d * d / (2 * v0 * v0);
  const double tan_low = (d - std::sqrt(d * d - 4 * a * (a + h))) / (2 * a);
  const double pitch_exp = std::atan(tan_low);
  const double t_exp = d / (v0 * std::cos(pitch_exp));
  const double drop_exp = 0.5 * g * t_exp * t_exp;

  std::printf("valid=%d\n", (int)r.valid);
  std::printf("pitch      = %+.4f°  (手算 %+.4f°)\n",
              r.pitch * 180 / M_PI, pitch_exp * 180 / M_PI);
  std::printf("flight     = %.4f s  (手算 %.4f s)\n", r.flight_time, t_exp);
  std::printf("aim_3d     = (%.3f, %.3f, %.3f)  (期望 z = %.3f + 下坠 %.3f = %.3f)\n",
              r.aim_3d(0), r.aim_3d(1), r.aim_3d(2), h, drop_exp, h + drop_exp);

  const bool ok = r.valid &&
                  std::abs(r.pitch - pitch_exp) < 1e-9 &&
                  std::abs(r.flight_time - t_exp) < 1e-9 &&
                  std::abs(r.aim_3d(2) - (h + drop_exp)) < 1e-9;
  std::printf("%s\n", ok ? "== 通过 ==" : "== 不通过 ==");
  return ok ? 0 : 1;
}
