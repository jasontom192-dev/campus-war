//PNP yaw 自检：IPPE 二义性；正面装甲板真值位姿投影加噪，看 solve 是否取对支
//判据 yaw 误差<30°(不翻支)且法线 n·p>0；6个真值角 -5/0/25/55/90/130°
//编译：g++ -std=c++20 -O2 -Wno-deprecated-enum-enum-conversion -I src tests/solver_yaw_test.cpp src/solver.cpp src/target.cpp $(pkg-config --cflags --libs opencv4) -I /usr/include/eigen3 -o /tmp/solver_yaw_test

#include <cmath>
#include <cstdio>
#include <random>

#include "solver.hpp"

namespace {

//相机内参，手册7.1标称值
cv::Mat make_K() {
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = 1159.420;
  K.at<double>(1, 1) = 1159.420;
  K.at<double>(0, 2) = 480.0;
  K.at<double>(1, 2) = 360.0;
  return K;
}

//构造竖直板姿态 [n|w|u]：n=(cos,0,sin)，u=(0,1,0)，w=u×n
cv::Mat R_of_yaw(double yaw) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  cv::Mat R(3, 3, CV_64F);
  R.at<double>(0,0)= c; R.at<double>(0,1)= s; R.at<double>(0,2)= 0;
  R.at<double>(1,0)= 0; R.at<double>(1,1)= 0; R.at<double>(1,2)= 1;
  R.at<double>(2,0)= s; R.at<double>(2,1)=-c; R.at<double>(2,2)= 0;
  return R;
}

}  //namespace

int main() {
  const cv::Mat K = make_K();
  const cv::Mat D = cv::Mat::zeros(1, 5, CV_64F);
  Solver solver(K, D);

  const auto pts3d = Solver::pnp_points(small);
  const cv::Vec3d tvec(1.0, 0.2, 5.0);          //右1m、下0.2m、前5m

  //法线须背离相机(n·p>0)；p=(1,0.2,5) 时合法约(-11°,169°)
  int fail = 0;
  for (double yaw_deg : {-5.0, 0.0, 25.0, 55.0, 90.0, 130.0}) {
    const double yaw_true = yaw_deg * CV_PI / 180.0;
    const cv::Mat R_true = R_of_yaw(yaw_true);

    //真值投影加0.7px噪声，IPPE 二义性在这点噪声下可能选错支
    cv::Vec3d rvec_true;
    cv::Rodrigues(R_true, rvec_true);
    std::vector<cv::Point2f> img;
    cv::projectPoints(pts3d, rvec_true, tvec, K, D, img);

    std::mt19937 rng(1000 + int(yaw_deg));
    std::normal_distribution<double> px_noise(0.0, 0.7);
    for (auto & p : img) { p.x += px_noise(rng); p.y += px_noise(rng); }

    //原始 IPPE 选支结果
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

    //估计法线及与真值的差
    cv::Mat R_est;
    cv::Rodrigues(a.rvec, R_est);
    const double nx = R_est.at<double>(0,0), ny = R_est.at<double>(1,0), nz = R_est.at<double>(2,0);
    const double dot = nx * tvec[0] + ny * tvec[1] + nz * tvec[2];
    const double yaw_est = std::atan2(R_est.at<double>(2,0), R_est.at<double>(0,0));
    const double err_deg = std::abs(limit_rad(yaw_est - yaw_true)) * 180 / CV_PI;

    //判据：yaw 误差<30°(翻支约180°)，法线 n·p>0
    //小装甲板5m处约31px宽，斜视 yaw 病态；前哨站投票只看方向
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
