// ============================================================================
// 车的选板策略离线自检（小陀螺档；2026-10-16 晚，照 auto_aim_rps26"提前选板"）
//
// 造一个"真值小陀螺"：四块板、半径 0.2m、绕竖直轴以 ω 匀速转、车不动
// （只看选板，不掺平动）。真值直接塞进 Target::ekf_.x —— 选板逻辑不关心状态
// 是怎么来的，这样就完全排除滤波暂态，看的纯粹是策略本身。
//
// 扫一遍车朝向 yaw，逐帧调 Aimer::best_aim（走完整链路：命中时刻选板 +
// 不动点迭代 + 换板门槛），核对四件事：
//
//   ① 换板点 —— 应发生在"新旧两块板与视线等距"【之前】= 提前选板。门限
//      thr=7.5°（CAR_SWITCH_DEG）的账：换板那一帧，被换掉的板 |δ|≈41.25°、
//      接班的板 |δ|≈48.75°（δ = 板法线 − 车心方位角，都在【命中时刻】评估），
//      即比"等距交叉"提前 3.75° = thr/2 的转速。
//   ② 与 rps26 原文逐帧扫描（angle_solver.cpp:127-147）的差异 —— 机制一致，
//      但原文每帧从 index=0 起扫、"序号小的板天然占优一个门限宽"，差异应只
//      落在每个换板点前 ≈7.5° 的转速窗口里（本工程的状态版按住旧板晚半拍）。
//   ③ 前哨站 —— 必须还是老的 argmin/argmax（"只改车"），逐帧对独立实现。
//   ④ 慢转档（|ω| < LOW_W）—— 选板 = 离光心最近的可见板（投影拿手册内参
//      独立复算），且锁定迟滞不许出现"锁定的板还可见却换人"。
//
// 编译（在工程根目录）：
//   g++ -std=c++20 -O2 -I src tests/car_select_sim.cpp src/aimer.cpp \
//       src/target.cpp src/solver.cpp $(pkg-config --cflags --libs opencv4) \
//       -I /usr/include/eigen3 -o /tmp/car_select_sim
// ============================================================================

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "aimer.hpp"
#include "solver.hpp"
#include "target.hpp"
#include "types.hpp"

namespace {

constexpr int    kNum       = 4;            // 车的板数
constexpr double kRCar      = 0.2;          // 步兵半径（和节点给 Target::init 的一致）
constexpr double kProcDelay = 0.005;        // Aimer 构造默认值（节点没传）
constexpr double kSwitchDeg = 7.5;          // 对齐 aimer.hpp 的 CAR_SWITCH_DEG
constexpr double kBullet    = 25.0;         // 题目的弹速
constexpr double kFwd       = 4.0;          // 车停在正前方 4m（进 4m 开火门）
constexpr double kVisible   = 75.0 * M_PI / 180.0;   // 同 aimer.hpp VISIBLE_LIMIT
constexpr double kFx        = 1159.420;     // 手册表 12 标称内参（见配置头注释）
constexpr double kCx        = 480.0, kCy = 360.0;

// 真值：11 维车状态 [x vx y vy z vz yaw omega r dr dh]（pseudo 系：右/前/上）
Eigen::VectorXd car_truth(double yaw, double omega) {
  Eigen::VectorXd x(11);
  x << 0.0, 0.0,      // 右 0
       kFwd, 0.0,     // 前 4m
       0.0, 0.0,      // 上 0（和相机同高，最省事）
       yaw, omega,
       kRCar, 0.0, 0.0;
  return x;
}

// 前哨站真值：10 维 [x vx y vy z vz yaw v_yaw dz1 dz2]，半径固定 0.275
Eigen::VectorXd outpost_truth(double yaw, double v_yaw) {
  Eigen::VectorXd x(10);
  x << 0.0, 0.0,
       kFwd, 0.0,
       0.0, 0.0,
       yaw, v_yaw,
       kOutpostDz1, kOutpostDz2;
  return x;
}

// 独立的板法线角（不复用 h_armor_xyz/plates_of，避免"自己证自己"）：
//   板法线 θ = yaw + id*2π/n；δ = θ − 车心方位角 atan2(前, 右)。
//   车停在正前方时车心方位角恒 = 90°。
double bot_delta(const Eigen::VectorXd & x, int id, int num) {
  const double theta = x(6) + id * 2.0 * CV_PI / num;
  return limit_rad(theta - std::atan2(x(2), x(0)));
}

// 独立复刻 rps26 的 AngleSolver::getClosestArmor（angle_solver.cpp:127-147），
// 逐帧无状态：
//   ① adv = min(0.2, D2R(thr)/|ω|)（原文 0.2 是硬写的上限）；
//   ② 在 predict_time + adv 时刻评估所有板（原文 predictArmors）；
//   ③ 扫描从 index=0 起，挑战者要好出 thr 才顶掉当前 index（序号偏置就在这里）。
// predict_time 传"命中时刻"（原文的 hit_time；本工程 = proc_delay + 飞时）。
int rps26_pick(const Eigen::VectorXd & x, int num, double predict_time, double thr_deg) {
  const double omega = x(7);
  const double thr = thr_deg * M_PI / 180.0;
  const double adv = std::min(0.2, thr / std::abs(omega));   // 高速档 ω≠0，无 0 保护
  const double yaw_eval = x(6) + omega * (predict_time + adv);
  const double center_yaw = std::atan2(x(2), x(0));          // 车不动，方位角不随时变
  double d[8];
  for (int i = 0; i < num; i++)
    d[i] = std::abs(limit_rad(limit_rad(yaw_eval + i * 2.0 * CV_PI / num) - center_yaw));
  int index = 0;
  for (int i = 0; i < num; i++)
    if (d[i] + thr < d[index]) index = i;
  return index;
}

// 前哨站老规则（本工程 choose_high_speed 的 outpost 分支）独立复算：
// 可见板里 ω>0 取 δ 最小、ω<0 取 δ 最大
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

// 手工针孔投影（和 Solver::project_world 同一模型，独立写一遍）：
//   pseudo(右,前,上) → 相机(右,−上,前) → u = fx*X/Z+cx, v = fy*Y/Z+cy
cv::Point2f project(const Eigen::VectorXd & x, int id, int num, double r) {
  const double theta = x(6) + id * 2.0 * CV_PI / num;
  const double X = x(0) - r * std::cos(theta);      // 相机 x = pseudo 右
  const double Z = x(2) - r * std::sin(theta);      // 相机 z = pseudo 前
  const double Y = -x(4);                           // 相机 y = −pseudo 上
  return {(float)(kFx * X / Z + kCx), (float)(kFx * Y / Z + kCy)};
}

int argmin_abs_delta(const Eigen::VectorXd & x, int num) {
  int best = 0;
  for (int i = 1; i < num; i++)
    if (std::abs(bot_delta(x, i, num)) < std::abs(bot_delta(x, best, num))) best = i;
  return best;
}

}  // namespace

int main() {
  // 光心 (cx,cy) 恰好 = 画面中心，低速档的"离画面中心最近"就没有二义。
  cv::Mat K = (cv::Mat_<double>(3, 3) << kFx, 0, kCx, 0, kFx, kCy, 0, 0, 1);
  cv::Mat D = cv::Mat::zeros(1, 5, CV_64F);
  Solver solver(K, D);
  const cv::Point2f img_center(kCx, kCy);

  int fail = 0;
  std::printf("Aimer: 弹速 %.0f m/s, proc_delay %.0f ms, CAR_SWITCH_DEG %.1f°\n\n",
              kBullet, kProcDelay * 1e3, kSwitchDeg);

  // ==========================================================================
  // 实验 1：小陀螺车（|ω| ≥ LOW_W=1.5 rad/s），正反两个方向各扫 180°
  // ==========================================================================
  for (double omega_deg : {+400.0, -400.0}) {
    const double omega = omega_deg * M_PI / 180.0;
    Target t;
    t.name        = three;
    t.armor_type  = small;
    t.armor_num_  = kNum;
    t.is_outpost_ = false;
    t.ekf_.x = car_truth(0.0, omega);
    Aimer aimer(kBullet, kProcDelay);

    // ★ 帧序必须跟 ω 同向（= 物理时间序）：ω<0 时 yaw 递减着扫。
    //   反着扫只是把时间倒放，换板签名(旧 41.25→新 48.75)会镜像成 48.75→41.25，
    //   看起来像"提前变滞后"——第一版测试就栽在这（假发现）。
    const int    steps = 18000;                 // 步长 0.01°
    const double dth = (omega > 0 ? 1.0 : -1.0) * 180.0 * M_PI / 180.0 / steps;
    const double yaw0 = (omega > 0 ? -90.0 : +90.0) * M_PI / 180.0;

    int prev_pick = -1;
    int n_switch = 0, n_bad = 0, n_unconv = 0, n_mismatch = 0, n_not_fair = 0;
    double not_fair_deg = 0;                    // "打了非最正对板"的转速总量
    double max_abs_delta = 0;                   // 选中板在命中时刻的 |δ| 上界
    double sw_old[4] = {0}, sw_new[4] = {0};    // 换板帧：被换掉/接班的 |δ|(°)
    double sw_gap[4] = {0};                     // 距上一次换板的转速间隔(°)
    double last_switch_yaw = 0;
    double mm_lo[8] = {0}, mm_hi[8] = {0};      // 与原文不一致的窗口（yaw 区间）
    int    mm_n = 0;
    bool   in_mm = false;

    for (int f = 0; f <= steps; f++) {
      t.ekf_.x(6) = limit_rad(yaw0 + f * dth);
      const Aimer::AimResult r = aimer.best_aim(t, img_center, solver);
      if (!r.valid) { n_bad++; continue; }
      if (!r.converged) n_unconv++;

      // 命中时刻的独立几何：t_hit = proc_delay + 飞时（本帧结果的直接函数）
      const double t_hit = kProcDelay + r.flight_time;
      const Eigen::VectorXd x_hit = car_truth(limit_rad(t.ekf_.x(6) + omega * t_hit), omega);
      const double d_sel = std::abs(bot_delta(x_hit, r.plate_id, kNum));
      max_abs_delta = std::max(max_abs_delta, d_sel);
      if (r.plate_id != argmin_abs_delta(x_hit, kNum)) {
        n_not_fair++;
        not_fair_deg += std::abs(dth) * 180.0 / M_PI;
      }

      // 与原文扫描比对，记账不一致窗口
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

    // ---- 判据 ----
    const bool ok_switch_num = (n_switch == 2);                 // 180° 里两次交接
    bool ok_landmark = true;
    for (int i = 0; i < std::min(n_switch, 4); i++) {
      if (std::abs(sw_old[i] - 41.25) > 0.6) ok_landmark = false;
      if (std::abs(sw_new[i] - 48.75) > 0.6) ok_landmark = false;
      if (sw_new[i] <= sw_old[i]) ok_landmark = false;          // "提前"的签名
    }
    const bool ok_limit = (max_abs_delta * 180.0 / M_PI < 49.6);  // 不越 45+thr/2
    const bool ok_bad   = (n_bad == 0 && n_unconv == 0);
    const bool ok_notfair = (not_fair_deg < 10.0);              // ≈ 3.75° × 2 次
    bool ok_mm = (mm_n == n_switch);                            // 每个换板点一个窗口
    for (int i = 0; i < std::min(mm_n, 8); i++) {
      const double w = std::abs(mm_hi[i] - mm_lo[i]) * 180 / M_PI;
      if (w > kSwitchDeg + 1.0) ok_mm = false;
    }
    const bool ok = ok_switch_num && ok_landmark && ok_limit && ok_bad && ok_notfair && ok_mm;
    if (!ok) fail = 1;
    std::printf("  %s\n\n", ok ? "通过" : "不通过");
  }

  // ==========================================================================
  // 实验 2：前哨站必须还是老的 argmin/argmax（"只改车"）
  // ==========================================================================
  for (double dir : {+1.0, -1.0}) {
    const double v_yaw = dir * kOutpostVyaw;            // ±0.8π，恒走高速档
    Target t;
    t.name        = outpost;
    t.armor_type  = small;
    t.armor_num_  = 3;
    t.is_outpost_ = true;
    t.ekf_.x = outpost_truth(0.4, v_yaw);
    Aimer aimer(kBullet, kProcDelay);

    int mismatch = 0, n_bad = 0;
    const int steps = 120 * 100;                        // 0.01°，扫 120°（一次交接）
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

  // ==========================================================================
  // 实验 3：慢转档（|ω| < LOW_W）—— 选板 = 离光心最近的可见板 + 锁定迟滞
  // ==========================================================================
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

      // 独立复算：在【选板用的状态】（外推 proc_delay）上针孔投影，
      // 取离画面中心最近的可见板
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
      // 迟滞允许：锁定板仍可见 → 保持不换
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
