#include "aimer.hpp"
#include <algorithm>
#include <cmath>

//构造后参数不再变化
Aimer::Aimer(double bullet_speed, double proc_delay)
  : bullet_speed_(bullet_speed), proc_delay_(proc_delay) {}

//清除选板迟滞状态，下一帧重新选板
void Aimer::reset_lock() {
  locked_plate_id_   = -1;
  locked_target_name_ = -1;
}

//选板后用飞行时间不动点迭代计算提前量，返回瞄准点和发射角
Aimer::AimResult Aimer::best_aim(const Target  & target,
                                  const cv::Point2f & img_center,
                                  const Solver & solver) {
  AimResult result;

  const auto & x = target.ekf_.x;

  int target_name = (int)target.name;

  //目标编号变化时清空板锁
  if (locked_target_name_ != target_name) {
    locked_plate_id_   = -1;
    locked_target_name_ = target_name;
  }

  //普通车用命中时刻的预测状态选板，前哨站用当前状态
  const bool future_sel = !target.is_outpost_;

  const Eigen::VectorXd x_sel = future_sel ? predict_state(x, proc_delay_) : x;
  int chosen_id = select_plate(target, x_sel, img_center, solver, locked_plate_id_);
  if (chosen_id < 0) {
    locked_plate_id_ = -1;
    return result;
  }
  locked_plate_id_ = chosen_id;

  //先用当前位置解一次弹道，得到迭代初值并排除不可达目标
  double prev_ft = 0.0;
  {
    Eigen::Vector3d a0 = target.h_armor_xyz(x, chosen_id);
    double d0 = std::sqrt(a0(0) * a0(0) + a0(1) * a0(1)); //水平距离
    double h0 = a0(2); //高度差
    auto   tr = solve_traj(bullet_speed_, d0, h0);
    if (tr.unsolvable) return result;
    prev_ft = tr.fly_time;
  }

  //不动点迭代：飞行时间决定外推位置，外推位置又决定飞行时间
  //总提前量为处理延迟加弹丸飞行时间
  for (int iter = 0; iter < MAX_ITER; iter++) {
    double total_dt = proc_delay_ + prev_ft;

    Eigen::VectorXd x_future = predict_state(x, total_dt);

    //命中时刻重新选板；板变化后继续迭代，避免按旧板收敛
    bool plate_changed = false;
    if (future_sel) {
      const int id_f = select_plate(target, x_future, img_center, solver, chosen_id);
      if (id_f >= 0 && id_f != chosen_id) {
        chosen_id = id_f;
        plate_changed = true;
      }
    }

    Eigen::Vector3d ap = target.h_armor_xyz(x_future, chosen_id);
    double dp = std::sqrt(ap(0) * ap(0) + ap(1) * ap(1));
    double hp = ap(2);
    auto   tr = solve_traj(bullet_speed_, dp, hp);
    if (tr.unsolvable) return result;

    //两轮飞行时间差小于 1ms 视为收敛
    if (!plate_changed && std::abs(tr.fly_time - prev_ft) < CONVERGE_EPS) {
      result.flight_time = tr.fly_time;
      result.pitch       = tr.pitch;
      result.aim_3d      = ap;
      //瞄准点抬高 ½gt² 补偿下坠，pitch 已含重力补偿
      result.aim_3d(2)  += 0.5 * g_ * tr.fly_time * tr.fly_time;
      result.aim_2d      = pseudo_to_pixel(result.aim_3d, solver);
      result.plate_id    = chosen_id;
      result.valid       = true;
      result.converged   = true;
      return result;
    }

    prev_ft = tr.fly_time;
  }

  //达到迭代上限时返回最后一次结果，converged 保持 false
  {
    double total_dt = proc_delay_ + prev_ft;
    Eigen::VectorXd x_future = predict_state(x, total_dt);
    Eigen::Vector3d ap = target.h_armor_xyz(x_future, chosen_id);
    auto tr = solve_traj(bullet_speed_,
                         std::sqrt(ap(0)*ap(0) + ap(1)*ap(1)),
                         ap(2));
    result.flight_time = tr.fly_time;
    result.pitch       = tr.pitch;
    result.aim_3d      = ap;
    result.aim_3d(2)  += 0.5 * g_ * tr.fly_time * tr.fly_time;
    result.aim_2d      = pseudo_to_pixel(result.aim_3d, solver);
    result.plate_id    = chosen_id;
    result.valid       = !tr.unsolvable;
  }

  return result;
}

//返回法线方向与车心方位角相差小于 75° 的板下标
std::vector<int> Aimer::visible_plates(
    const std::vector<Eigen::Vector4d> & plates,
    double center_yaw) const {
  std::vector<int> ids;
  for (int i = 0; i < (int)plates.size(); i++) {
    double delta = limit_rad(plates[i](3) - center_yaw);
    if (std::abs(delta) < VISIBLE_LIMIT)
      ids.push_back(i);
  }
  return ids;
}

//按给定状态计算所有板的 [x,y,z,yaw]，可用未来状态做选板
std::vector<Eigen::Vector4d> Aimer::plates_of(const Target & t, const Eigen::VectorXd & x) {
  std::vector<Eigen::Vector4d> list;
  for (int i = 0; i < t.armor_num_; i++) {
    const double angle = limit_rad(x(6) + i * 2.0 * CV_PI / t.armor_num_);
    const Eigen::Vector3d p = t.h_armor_xyz(x, i);
    list.push_back({p(0), p(1), p(2), angle});
  }
  return list;
}

//选板
int Aimer::select_plate(const Target & target, const Eigen::VectorXd & x,
                        const cv::Point2f & img_center, const Solver & solver,
                        int locked_id) const {
  const auto plates = plates_of(target, x);
  const double center_yaw = std::atan2(x(2), x(0));
  const double omega = x(7);
  const auto vis = visible_plates(plates, center_yaw);
  if (vis.empty()) return -1;

  //高速走提前选板策略，低速走画面中心最近策略
  if (std::abs(omega) >= LOW_W)
    return choose_high_speed(plates, vis, omega, center_yaw, target.is_outpost_,
                             locked_id);

  //低速时锁定板仍可见就不换
  if (locked_id >= 0)
    for (int id : vis)
      if (id == locked_id) return locked_id;

  return choose_low_speed(target, x, vis, img_center, solver);
}

//低速选板：投影到像素后选离画面中心最近的板
int Aimer::choose_low_speed(const Target  & target,
                             const Eigen::VectorXd & x,
                             const std::vector<int> & visible_ids,
                             const cv::Point2f & img_center,
                             const Solver & solver) const {
  if (visible_ids.size() == 1)
    return visible_ids[0];

  int    best_id = visible_ids[0];
  double best_d2 = 1e10;

  for (int id : visible_ids) {
    Eigen::Vector3d pw = target.h_armor_xyz(x, id);
    cv::Point2f px = pseudo_to_pixel(pw, solver);
    double dx = px.x - img_center.x;
    double dy = px.y - img_center.y;
    double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_id = id;
    }
  }
  return best_id;
}

//高速选板：车在提前 CAR_SWITCH_DEG 的评估时刻选 |δ| 最小的板，并保留换板迟滞
int Aimer::choose_high_speed(const std::vector<Eigen::Vector4d> & plates,
                              const std::vector<int> & visible_ids,
                              double omega, double center_yaw, bool outpost,
                              int locked_id) const {
  //前哨站保持 argmin/argmax 规则
  if (outpost) {
    if (visible_ids.size() == 1)
      return visible_ids[0];

    std::vector<std::pair<int, double>> candidates;
    for (int id : visible_ids) {
      double delta = limit_rad(plates[id](3) - center_yaw);
      candidates.push_back({id, delta});
    }

    if (omega > 0) {
      //正转时 delta 最小的板最先转到正对相机
      auto it = std::min_element(candidates.begin(), candidates.end(),
        [](const auto & a, const auto & b) { return a.second < b.second; });
      return it->first;
    } else {
      //反转时 delta 最大的板最先转到正对相机
      auto it = std::max_element(candidates.begin(), candidates.end(),
        [](const auto & a, const auto & b) { return a.second < b.second; });
      return it->first;
    }
  }

  //评估时刻按命中时刻再提前 CAR_SWITCH_DEG/|ω|
  const double switch_tol = CAR_SWITCH_DEG * M_PI / 180.0;
  const double adv = std::min(CAR_SWITCH_MAX_ADV,
                              switch_tol / std::max(std::abs(omega), 1e-6));
  auto delta_eval = [&](int id) {
    return limit_rad(limit_rad(plates[id](3) - center_yaw) + omega * adv);
  };

  int pick = -1;
  double best = 1e9;
  for (int id : visible_ids) {
    const double d = std::abs(delta_eval(id));
    if (d < best) { pick = id; best = d; }//最优板
  }
  if (pick < 0) return -1;

  //锁定板仍可见且未落后超过 CAR_SWITCH_DEG 时继续锁定
  if (locked_id >= 0 && pick != locked_id &&
      std::find(visible_ids.begin(), visible_ids.end(), locked_id) != visible_ids.end() &&
      std::abs(delta_eval(locked_id)) <= best + switch_tol)
    return locked_id;
  return pick;
}

//斜抛解算，判别式小于 0 时无解，有双解取飞行时间较短者
Aimer::TrajResult Aimer::solve_traj(double v0, double d, double h) const {
  //d=v0cos(p)t，h=v0sin(p)t-½gt²；消 t 得 a·tan²p-d·tan(p)+(a+h)=0，a=g·d²/(2v0²)
  //d=0 时 a=0，后续 2a 会除零
  TrajResult r;
  double a = g_ * d * d / (2.0 * v0 * v0);
  double b = -d;
  double c = a + h;
  double delta = b * b - 4.0 * a * c;

  if (delta < 0) { r.unsolvable = true; return r; }

  double tan1 = (-b + std::sqrt(delta)) / (2.0 * a); //高抛解
  double tan2 = (-b - std::sqrt(delta)) / (2.0 * a); //低伸解
  double p1 = std::atan(tan1), p2 = std::atan(tan2);
  double t1 = d / (v0 * std::cos(p1));
  double t2 = d / (v0 * std::cos(p2));

  //取飞行时间短的解
  if (t1 < t2) { r.pitch = p1; r.fly_time = t1; }
  else         { r.pitch = p2; r.fly_time = t2; }

  r.unsolvable = false;
  return r;
}

//基地直瞄：目标静止，直接对板中心解算弹道，不做提前量
Aimer::AimResult Aimer::best_aim_direct(const Eigen::Vector3d & xyz_state,
                                        const Solver & solver) const {
  AimResult r;

  const double d = std::sqrt(xyz_state(0) * xyz_state(0) + xyz_state(1) * xyz_state(1));
  const double h = xyz_state(2);

  const TrajResult tr = solve_traj(bullet_speed_, d, h);
  if (tr.unsolvable) return r;

  r.valid       = true;
  r.converged   = true; //解析解，无迭代
  r.flight_time = tr.fly_time;
  r.pitch       = tr.pitch;
  r.plate_id    = 0;

  //瞄准点抬高 ½gt² 补偿下坠，仅用于显示
  r.aim_3d = xyz_state;
  r.aim_3d(2) += 0.5 * g_ * tr.fly_time * tr.fly_time;
  r.aim_2d = pseudo_to_pixel(r.aim_3d, solver);
  return r;
}

//按匀速模型外推状态，不修改 EKF
Eigen::VectorXd Aimer::predict_state(const Eigen::VectorXd & x, double dt) {
  Eigen::VectorXd xp = x;
  xp(0) += xp(1) * dt;
  xp(2) += xp(3) * dt;
  xp(4) += xp(5) * dt;
  xp(6)  = limit_rad(xp(6) + xp(7) * dt);
  return xp;
}

//pseudo 系转像素，相机后方由 project_world 返回 (-1,-1)
cv::Point2f Aimer::pseudo_to_pixel(const Eigen::Vector3d & p,
                                    const Solver & solver) {
  Eigen::Vector3d cam(p(0), -p(2), p(1));
  return solver.project_world(cam);
}
