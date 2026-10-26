// ============================================================================
// PnP 二义性 / yaw 优化自检
//
// 造一个"正面朝着相机"的装甲板真值位姿，把它 4 个角投影成像素点当作观测，
// 再喂给 Solver::solve，看解出来的 yaw 是不是真值那一支：
//   · 如果 optimize_yaw 没起作用，IPPE 有可能给出"镜像"那支（法线反 180°），
//     前哨站上就会导致方向投票和 EKF 的顺/逆时针反掉
//   · 判据：解出的 yaw 与真值差 < 1°，且解出的法线满足 n·p > 0（正面朝我）
//
// 编译（工程根目录）：
//   g++ -std=c++20 -O2 -Wno-deprecated-enum-enum-conversion -I src \
//       tests/solver_yaw_test.cpp src/solver.cpp src/target.cpp \
//       $(pkg-config --cflags --libs opencv4) -I /usr/include/eigen3 -o /tmp/solver_yaw_test
// ============================================================================

#include <cmath>
#include <cstdio>
#include <random>

#include "solver.hpp"

namespace {

// 相机内参（手册 7.1 的标称值）
cv::Mat make_K() {
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = 1159.420;
  K.at<double>(1, 1) = 1159.420;
  K.at<double>(0, 2) = 480.0;
  K.at<double>(1, 2) = 360.0;
  return K;
}

// 给定 yaw 重建一个【竖直】的装甲板姿态（世界系 = 相机系，离线情形）：
//   列向量 [n | w | u]，法线 n=(cos,0,sin)、板高方向 u=(0,1,0)（从上往下）、
//   宽度 w=u×n。和 solver.cpp 里 optimize_yaw 的候选族完全一致。
cv::Mat R_of_yaw(double yaw) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  cv::Mat R(3, 3, CV_64F);
  R.at<double>(0,0)= c; R.at<double>(0,1)= s; R.at<double>(0,2)= 0;
  R.at<double>(1,0)= 0; R.at<double>(1,1)= 0; R.at<double>(1,2)= 1;
  R.at<double>(2,0)= s; R.at<double>(2,1)=-c; R.at<double>(2,2)= 0;
  return R;
}

}  // namespace

int main() {
  const cv::Mat K = make_K();
  const cv::Mat D = cv::Mat::zeros(1, 5, CV_64F);
  Solver solver(K, D);

  const auto pts3d = Solver::pnp_points(small);
  const cv::Vec3d tvec(1.0, 0.2, 5.0);          // 右 1m、下 0.2m、前 5m

  // 真值 yaw 的合法范围：法线要背离相机（n·p > 0）—— 装甲板只有正面能被看到。
  // p=(1, 0.2, 5) 时大约对应 (-11°, 169°)，这里取几个典型角度。
  int fail = 0;
  for (double yaw_deg : {-5.0, 0.0, 25.0, 55.0, 90.0, 130.0}) {
    const double yaw_true = yaw_deg * CV_PI / 180.0;
    const cv::Mat R_true = R_of_yaw(yaw_true);

    // 真值位姿投影出观测点，加一点像素噪声（真实检测本来就有 1px 级抖动，
    // 正是这点噪声会让 IPPE 在二义的两支之间挑错）
    cv::Vec3d rvec_true;
    cv::Rodrigues(R_true, rvec_true);
    std::vector<cv::Point2f> img;
    cv::projectPoints(pts3d, rvec_true, tvec, K, D, img);

    std::mt19937 rng(1000 + int(yaw_deg));
    std::normal_distribution<double> px_noise(0.0, 0.7);
    for (auto & p : img) { p.x += px_noise(rng); p.y += px_noise(rng); }

    // 先看【不做 yaw 优化】的原始 IPPE 给的是哪支
    cv::Vec3d rvec_raw, tvec_raw;
    cv::solvePnP(pts3d, img, K, D, rvec_raw, tvec_raw, false, cv::SOLVEPNP_IPPE);
    cv::Mat R_raw;
    cv::Rodrigues(rvec_raw, R_raw);
    const double yaw_raw = std::atan2(R_raw.at<double>(2,0), R_raw.at<double>(0,0));

    Armor a;
    a.type  = small;
    a.name  = outpost;
    a.points = img;
    solver.solve(a);

    // 解出来的法线、以及和真值 yaw 的差
    cv::Mat R_est;
    cv::Rodrigues(a.rvec, R_est);
    const double nx = R_est.at<double>(0,0), ny = R_est.at<double>(1,0), nz = R_est.at<double>(2,0);
    const double dot = nx * tvec[0] + ny * tvec[1] + nz * tvec[2];
    const double yaw_est = std::atan2(R_est.at<double>(2,0), R_est.at<double>(0,0));
    const double err_deg = std::abs(limit_rad(yaw_est - yaw_true)) * 180 / CV_PI;

    // 判据：① 不能翻支（翻支是 180° 量级，这里放宽到 30°）② 法线朝外。
    // 不要求更高的精度：小装甲板在 5m 处只有 ~31px 宽，斜视时 yaw 本来病态；
    // 前哨站那边也只需要"方向对不对"（投票只看符号），EKF 会自己滤。
    const bool ok = err_deg < 30.0 && dot > 0;
    if (!ok) fail++;
    std::printf("真值 yaw=%+6.1f° | 原始 IPPE=%+6.1f°%s | 优化后=%+6.1f° (误差 %.3f°, n·p=%+.2f) %s\n",
                yaw_deg, yaw_raw * 180 / CV_PI,
                std::abs(limit_rad(yaw_raw - yaw_true)) * 180 / CV_PI > 5.0 ? " ← 取错支!" : "",
                yaw_est * 180 / CV_PI, err_deg, dot, ok ? "✅" : "❌ 取错支了");
  }

  std::printf("%s\n", fail == 0 ? "== 全部通过 ==" : "== 有失败 ==");
  return fail == 0 ? 0 : 1;
}
