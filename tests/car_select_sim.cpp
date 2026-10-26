//车选板离线自检：真值小陀螺(4板,R=0.2m,匀速ω,车不动)
//δ=板法线-车心方位角(命中时刻)，换板帧旧≈41.25°、新≈48.75°，thr=7.5°；另验与 rps26 原文、前哨站、低速档的一致性
//编译：g++ -std=c++20 -O2 -I src tests/car_select_sim.cpp src/aimer.cpp src/target.cpp src/solver.cpp $(pkg-config --cflags --libs opencv4) -I /usr/include/eigen3 -o /tmp/car_select_sim

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "aimer.hpp"
#include "solver.hpp"
#include "target.hpp"
#include "types.hpp"

namespace {

constexpr int    kNum       = 4;            //板数
constexpr double kRCar      = 0.2;          //步兵半径，与 Target::init 一致
constexpr double kProcDelay = 0.005;        //Aimer 默认值
constexpr double kSwitchDeg = 7.5;          //CAR_SWITCH_DEG
constexpr double kBullet    = 25.0;         //弹速
constexpr double kFwd       = 4.0;          //正前方4m
constexpr double kVisible   = 75.0 * M_PI / 180.0;   //VISIBLE_LIMIT
constexpr double kFx        = 1159.420;     //手册标称内参
constexpr double kCx        = 480.0, kCy = 360.0;

//真值车状态 [x vx y vy z vz yaw omega r dr dh]，pseudo 系(右/前/上)
Eigen::VectorXd car_truth(double yaw, double omega) {
  Eigen::VectorXd x(11);
  x << 0.0, 0.0,      //右0
       kFwd, 0.0,     //前4m
       0.0, 0.0,      //上0，与相机同高
       yaw, omega,
       kRCar, 0.0, 0.0;
  return x;
}

//前哨站真值 [x vx y vy z vz yaw v_yaw dz1 dz2]，半径0.275
Eigen::VectorXd outpost_truth(double yaw, double v_yaw) {
  Eigen::VectorXd x(10);
  x << 0.0, 0.0,
       kFwd, 0.0,
       0.0, 0.0,
       yaw, v_yaw,
       kOutpostDz1, kOutpostDz2;
  return x;
}

//独立法线角：θ=yaw+id*2π/n，δ=θ-atan2(前,右)；车在正前方时方位角=90°
double bot_delta(const Eigen::VectorXd & x, int id, int num) {
  const double theta = x(6) + id * 2.0 * CV_PI / num;
  return limit_rad(theta - std::atan2(x(2), x(0)));
}

//复刻 rps26 getClosestArmor：adv=min(0.2,thr/|ω|)，index=0 起扫，领先 thr 才换；predict_time=命中时刻
int rps26_pick(const Eigen::VectorXd & x, int num, double predict_time, double thr_deg) {
  const double omega = x(7);
  const double thr = thr_deg * M_PI / 180.0;
  const double adv = std::min(0.2, thr / std::abs(omega));   //高速档无0保护
  const double yaw_eval = x(6) + omega * (predict_time + adv);
  const double center_yaw = std::atan2(x(2), x(0));          //方位角不变
  double d[8];
  for (int i = 0; i < num; i++)
    d[i] = std::abs(limit_rad(limit_rad(yaw_eval + i * 2.0 * CV_PI / num) - center_yaw));
  int index = 0;
  for (int i = 0; i < num; i++)
    if (d[i] + thr < d[index]) index = i;
  return index;
}

//前哨站规则：可见板中 ω>0 取 δ 最小，ω<0 取 δ 最大
int outpost_reference(const Eigen::VectorXd & x) {
  int pick = -1;
  double best = 0;
  for (int i = 0; i < 3; i++) {
    const double d = bot_delta(x, i, 3);
    if (std::abs(d) >= kVisible) continue;
    if (pick < 0 || (x(7) > 0 ? d < best : d > best)) { pick = i; best = d; }
  }
  return pick;
}

//针孔投影：pseudo(右/前/上)→相机(右/-上/前)，u=fx*X/Z+cx，v=fy*Y/Z+cy
cv::Point2f project(const Eigen::VectorXd & x, int id, int num, double r) {
  const double theta = x(6) + id * 2.0 * CV_PI / num;
  const double X = x(0) - r * std::cos(theta);      //相机x=右
  const double Z = x(2) - r * std::sin(theta);      //相机z=前
  const double Y = -x(4);                           //相机y=-上
  return {(float)(kFx * X / Z + kCx), (float)(kFx * Y / Z + kCy)};
}

int argmin_abs_delta(const Eigen::VectorXd & x, int num) {
  int best = 0;
  for (int i = 1; i < num; i++)
    if (std::abs(bot_delta(x, i, num)) < std::abs(bot_delta(x, best, num))) best = i;
  return best;
}

}  //namespace

int main() {
  //光心恰为画面中心，低速档"离画面中心最近"无二义
  cv::Mat K = (cv::Mat_<double>(3, 3) << kFx, 0, kCx, 0, kFx, kCy, 0, 0, 1);
  cv::Mat D = cv::Mat::zeros(1, 5, CV_64F);
  Solver solver(K, D);
  const cv::Point2f img_center(kCx, kCy);

  int fail = 0;
  std::printf("Aimer: 弹速 %.0f m/s, proc_delay %.0f ms, CAR_SWITCH_DEG %.1f°\n\n",
              kBullet, kProcDelay * 1e3, kSwitchDeg);

  //实验 1：小陀螺车 |ω|≥LOW_W=1.5 rad/s，正反方向各扫 180°
  for (double omega_deg : {+400.0, -400.0}) {
    const double omega = omega_deg * M_PI / 180.0;
    Target t;
    t.name        = three;
    t.armor_type  = small;
    t.armor_num_  = kNum;
    t.is_outpost_ = false;
    t.ekf_.x = car_truth(0.0, omega);
    Aimer aimer(kBullet, kProcDelay);

    //帧序必须与 ω 同向；反向扫会把换板签名 41.25→48.75 镜像成 48.75→41.25
    const int    steps = 18000;                 //步长0.01°
    const double dth = (omega > 0 ? 1.0 : -1.0) * 180.0 * M_PI / 180.0 / steps;
    const double yaw0 = (omega > 0 ? -90.0 : +90.0) * M_PI / 180.0;

    int prev_pick = -1;
    int n_switch = 0, n_bad = 0, n_unconv = 0, n_mismatch = 0, n_not_fair = 0;
    double not_fair_deg = 0;                    //打了非最正对板的转角量
    double max_abs_delta = 0;                   //选中板命中时刻|δ|上界
    double sw_old[4] = {0}, sw_new[4] = {0};    //换板帧旧/新板|δ|(°)
    double sw_gap[4] = {0};                     //距上次换板的转角(°)
    double last_switch_yaw = 0;
    double mm_lo[8] = {0}, mm_hi[8] = {0};      //与原文不一致的yaw区间
    int    mm_n = 0;
    bool   in_mm = false;

    for (int f = 0; f <= steps; f++) {
      t.ekf_.x(6) = limit_rad(yaw0 + f * dth);
      const Aimer::AimResult r = aimer.best_aim(t, img_center, solver);
      if (!r.valid) { n_bad++; continue; }
      if (!r.converged) n_unconv++;

      //命中时刻几何：t_hit=proc_delay+飞时
      const double t_hit = kProcDelay + r.flight_time;
      const Eigen::VectorXd x_hit = car_truth(limit_rad(t.ekf_.x(6) + omega * t_hit), omega);
      const double d_sel = std::abs(bot_delta(x_hit, r.plate_id, kNum));
      max_abs_delta = std::max(max_abs_delta, d_sel);
      if (r.plate_id != argmin_abs_delta(x_hit, kNum)) {
        n_not_fair++;
        not_fair_deg += std::abs(dth) * 180.0 / M_PI;
      }

      //与原文扫描比对，记录不一致窗口
      const int ref = rps26_pick(t.ekf_.x, kNum, t_hit, kSwitchDeg);
      if (ref != r.plate_id) {
        n_mismatch++;
        if (!in_mm) { in_mm = true; if (mm_n < 8) mm_lo[mm_n] = t.ekf_.x(6); }
        if (mm_n < 8) mm_hi[mm_n] = t.ekf_.x(6);
      } else if (in_mm) {
        in_mm = false;
        mm_n++;
      }

      if (prev_pick >= 0 && r.plate_id != prev_pick) {
        if (n_switch < 4) {
          sw_gap[n_switch] = std::abs(t.ekf_.x(6) - last_switch_yaw) * 180.0 / M_PI;
          sw_old[n_switch] = std::abs(bot_delta(x_hit, prev_pick, kNum)) * 180.0 / M_PI;
          sw_new[n_switch] = d_sel * 180.0 / M_PI;
        }
        last_switch_yaw = t.ekf_.x(6);
        n_switch++;
      }
      prev_pick = r.plate_id;
    }
    if (in_mm) mm_n++;

    std::printf("── 小陀螺车 ω = %+.0f°/s（扫 180°，步长 0.01°）──\n", omega_deg);
    std::printf("  换板次数 %d；间隔(°) :", n_switch);
    for (int i = 1; i < std::min(n_switch, 4); i++)
      std::printf(" %.3f", sw_gap[i]);
    std::printf("\n  换板帧 |δ@命中| 旧→新(°):");
    for (int i = 0; i < std::min(n_switch, 4); i++)
      std::printf("  %.2f→%.2f", sw_old[i], sw_new[i]);
    std::printf("\n  选中板 |δ@命中| 上界 : %.2f°；打非最正对的转速累计 %.2f°\n",
                max_abs_delta * 180.0 / M_PI, not_fair_deg);
    std::printf("  与 rps26 原文不一致 : %d 帧，窗口 %d 个", n_mismatch, mm_n);
    for (int i = 0; i < std::min(mm_n, 8); i++)
      std::printf(" [%+.2f°,%+.2f°]", mm_lo[i] * 180 / M_PI, mm_hi[i] * 180 / M_PI);
    std::printf("\n  valid/未收敛/总帧  : %d / %d / %d\n", n_bad, n_unconv, steps + 1);

    const bool ok_switch_num = (n_switch == 2);                 //180°内两次换板
    bool ok_landmark = true;
    for (int i = 0; i < std::min(n_switch, 4); i++) {
      if (std::abs(sw_old[i] - 41.25) > 0.6) ok_landmark = false; //旧板|δ|≈41.25°(±0.6°)
      if (std::abs(sw_new[i] - 48.75) > 0.6) ok_landmark = false; //新板|δ|≈48.75°(±0.6°)
      if (sw_new[i] <= sw_old[i]) ok_landmark = false;          //新板|δ|更大=提前换板
    }
    const bool ok_limit = (max_abs_delta * 180.0 / M_PI < 49.6);  //不越45+thr/2
    const bool ok_bad   = (n_bad == 0 && n_unconv == 0);
    const bool ok_notfair = (not_fair_deg < 10.0);              //≈3.75°×2次
    bool ok_mm = (mm_n == n_switch);                            //每个换板点一个窗口
    for (int i = 0; i < std::min(mm_n, 8); i++) {
      const double w = std::abs(mm_hi[i] - mm_lo[i]) * 180 / M_PI;
      if (w > kSwitchDeg + 1.0) ok_mm = false;                  //窗口宽≤thr+1°
    }
    const bool ok = ok_switch_num && ok_landmark && ok_limit && ok_bad && ok_notfair && ok_mm;
    if (!ok) fail = 1;
    std::printf("  %s\n\n", ok ? "通过" : "不通过");
  }

  //实验 2：前哨站仍用 argmin/argmax，与独立实现逐帧对比
  for (double dir : {+1.0, -1.0}) {
    const double v_yaw = dir * kOutpostVyaw;            //±0.8π，恒走高速档
    Target t;
    t.name        = outpost;
    t.armor_type  = small;
    t.armor_num_  = 3;
    t.is_outpost_ = true;
    t.ekf_.x = outpost_truth(0.4, v_yaw);
    Aimer aimer(kBullet, kProcDelay);

    int mismatch = 0, n_bad = 0;
    const int steps = 120 * 100;                        //0.01°，扫120°（一次交接）
    const double dth = (v_yaw > 0 ? 1.0 : -1.0) * 120.0 * M_PI / 180.0 / steps;
    for (int f = 0; f <= steps; f++) {
      t.ekf_.x(6) = limit_rad(0.4 + f * dth);
      const Aimer::AimResult r = aimer.best_aim(t, img_center, solver);
      if (!r.valid) { n_bad++; continue; }
      if (outpost_reference(t.ekf_.x) != r.plate_id) mismatch++;
    }
    const bool ok = (mismatch == 0 && n_bad == 0);
    if (!ok) fail = 1;
    std::printf("── 前哨站 v_yaw = %+.3f rad/s（扫 120°）──\n", v_yaw);
    std::printf("  与老 argmin/argmax 不一致: %d 帧 | invalid: %d | %s\n\n",
                mismatch, n_bad, ok ? "通过" : "不通过");
  }

  //实验 3：慢转档 |ω|<LOW_W（±30°/s），选离光心最近的可见板，允许锁定迟滞
  for (double omega_deg : {+30.0, -30.0}) {
    const double omega = omega_deg * M_PI / 180.0;
    Target t;
    t.name        = three;
    t.armor_type  = small;
    t.armor_num_  = kNum;
    t.is_outpost_ = false;
    t.ekf_.x = car_truth(0.0, omega);
    Aimer aimer(kBullet, kProcDelay);

    int mismatch = 0, n_switch = 0, n_bad = 0, prev = -1;
    const int steps = 180 * 100;
    const double dth = (omega > 0 ? 1.0 : -1.0) * 180.0 * M_PI / 180.0 / steps;
    for (int f = 0; f <= steps; f++) {
      t.ekf_.x(6) = limit_rad((omega > 0 ? -90.0 : +90.0) * M_PI / 180.0 + f * dth);
      const Aimer::AimResult r = aimer.best_aim(t, img_center, solver);
      if (!r.valid) { n_bad++; continue; }

      //在选板状态(外推 proc_delay)上做针孔投影，取离画面中心最近的可见板
      const Eigen::VectorXd x_sel =
          car_truth(limit_rad(t.ekf_.x(6) + omega * kProcDelay), omega);
      int nearest = -1;
      double best = 1e18;
      for (int id = 0; id < kNum; id++) {
        if (std::abs(bot_delta(x_sel, id, kNum)) >= kVisible) continue;
        const cv::Point2f px = project(x_sel, id, kNum, kRCar);
        const double d2 = std::pow(px.x - img_center.x, 2) + std::pow(px.y - img_center.y, 2);
        if (d2 < best) { best = d2; nearest = id; }
      }
      //锁定板仍可见时保持不换
      const bool locked_visible =
          prev >= 0 && std::abs(bot_delta(x_sel, prev, kNum)) < kVisible;
      if (!((r.plate_id == nearest) || (r.plate_id == prev && locked_visible))) mismatch++;
      if (prev >= 0 && r.plate_id != prev) n_switch++;
      prev = r.plate_id;
    }
    const bool ok = (mismatch == 0 && n_bad == 0);
    if (!ok) fail = 1;
    std::printf("── 慢转车 ω = %+.0f°/s（扫 180°）──\n", omega_deg);
    std::printf("  与'光心最近板(含锁定迟滞)'不符: %d 帧 | 换板 %d 次 | invalid %d | %s\n\n",
                mismatch, n_switch, n_bad, ok ? "通过" : "不通过");
  }

  std::printf("%s\n", fail ? "== 不通过 ==" : "== 通过 ==");
  return fail;
}
