#include "aimer.hpp"
#include <algorithm>
#include <cmath>

// 两个参数构造后不再变；main.cpp 只传初速，proc_delay_ 走默认值
Aimer::Aimer(double bullet_speed, double proc_delay)
  : bullet_speed_(bullet_speed), proc_delay_(proc_delay) {}

// 迟滞状态清零 —— 下一帧 best_aim 会因为 locked_plate_id_ < 0 重新走选板分支
void Aimer::reset_lock() {
  locked_plate_id_   = -1;
  locked_target_name_ = -1;
}

// ============================================================================
// best_aim —— 一次完整的自瞄解算（选板 → 提前量迭代 → 投影）
//
// 整体是五步，下面按这五步分段：
//   1. 取 EKF 状态、算出所有板的位姿、判断要不要重置锁定
//   2. 选板：锁定优先，没锁定就按 |omega| 走低速/高速两套策略
//   3. 用【当前位置】先解一次弹道，拿到迭代初值（顺便筛掉够不着的情况）
//   4. 不动点迭代：外推 → 解弹道 → 看飞行时间收没收敛
//   5. 收尾：迭代没收敛也要给结果（取最后一次的），并做重力补偿和投影
// ============================================================================
Aimer::AimResult Aimer::best_aim(const Target  & target,
                                  const cv::Point2f & img_center,
                                  const Solver & solver) {
  AimResult result;

  // ---- 第 1 步：取 EKF 状态 ----
  const auto & x = target.ekf_.x;

  int target_name = (int)target.name;

  // 换车了（兵种编号变了）→ 之前记的板编号没有意义，作废重来。
  // Tracker 换锁时已经调过 reset_lock()，这里是第二道保险。
  if (locked_target_name_ != target_name) {
    locked_plate_id_   = -1;
    locked_target_name_ = target_name;
  }

  // 车用“命中时刻”的预测状态选板；前哨站保持当前状态（只改车）。
  const bool future_sel = !target.is_outpost_;

  // ---- 第 2 步：选板（迟滞 → 低速/高速两套策略）----
  const Eigen::VectorXd x_sel = future_sel ? predict_state(x, proc_delay_) : x;
  int chosen_id = select_plate(target, x_sel, img_center, solver, locked_plate_id_);
  if (chosen_id < 0) {
    locked_plate_id_ = -1;
    return result;                // result.valid 仍是 false —— 这一帧没有可打的板
  }
  locked_plate_id_ = chosen_id;   // 记下来，后面帧只要它还可见就不换

  // ---- 第 3 步：先按当前位置解一次弹道，给迭代一个初值 ----
  // 这一步也顺手筛掉"以这个距离根本打不到"的目标（unsolvable）
  double prev_ft = 0.0;
  {
    Eigen::Vector3d a0 = target.h_armor_xyz(x, chosen_id);
    double d0 = std::sqrt(a0(0) * a0(0) + a0(1) * a0(1));   // 水平距离
    double h0 = a0(2);                                      // 高度差（pseudo z 就是"上"）
    auto   tr = solve_traj(bullet_speed_, d0, h0);
    if (tr.unsolvable) return result;
    prev_ft = tr.fly_time;
  }

  // ---- 第 4 步：不动点迭代求飞行时间 ----
  // 飞行时间 t 取决于打多远，而距离又取决于 t 时刻板在哪——循环依赖。
  // 迭代：猜 t → 外推板位置 → 解弹道得到更准的 t → 再外推，直到收敛。
  // 总提前量 = proc_delay_（处理+出膛固定耗时）+ prev_ft（弹丸飞行时间）。
  for (int iter = 0; iter < MAX_ITER; iter++) {
    double total_dt = proc_delay_ + prev_ft;

    Eigen::VectorXd x_future = predict_state(x, total_dt);         // 目标在那一时刻的“假想状态”

    // 迭代里按“命中时刻”重选板。板换了就再迭代一轮，避免假收敛
    // （板已经换人、飞时还是按旧板收敛的）。
    bool plate_changed = false;
    if (future_sel) {
      const int id_f = select_plate(target, x_future, img_center, solver, chosen_id);
      if (id_f >= 0 && id_f != chosen_id) {
        chosen_id = id_f;
        plate_changed = true;
      }
    }

    Eigen::Vector3d ap = target.h_armor_xyz(x_future, chosen_id);  // 用假想状态算板在哪
    double dp = std::sqrt(ap(0) * ap(0) + ap(1) * ap(1));
    double hp = ap(2);
    auto   tr = solve_traj(bullet_speed_, dp, hp);
    if (tr.unsolvable) return result;

    // 新的飞行时间和上一轮差不到 1ms，就认为不动点已经收敛
    if (!plate_changed && std::abs(tr.fly_time - prev_ft) < CONVERGE_EPS) {
      result.flight_time = tr.fly_time;
      result.pitch       = tr.pitch;
      result.aim_3d      = ap;
      // 重力补偿：ap 是板的位置，子弹飞行中会下坠 ½·g·t²。
      // 把瞄准点抬高同样的量，给出正确的 3D/2D 瞄准点。
      // （pitch 里已经含重力；这个抬高只用于瞄准点，不要再拿去解一次弹道。）
      result.aim_3d(2)  += 0.5 * g_ * tr.fly_time * tr.fly_time;
      result.aim_2d      = pseudo_to_pixel(result.aim_3d, solver);
      result.plate_id    = chosen_id;
      result.valid       = true;
      result.converged   = true;   // ★ 迭代收敛（≤1ms），节点才允许开火
      return result;
    }

    prev_ft = tr.fly_time;
  }

  // ---- 第 5 步：迭代满 MAX_ITER 次仍未收敛（目标机动/板切换/状态抖动），
  //      用最后一次迭代值作为结果。
  //      converged 保持 false：瞄准照发，但节点 decide_fire 用它挡开火。
  //      unsolvable 时不提前返回，而是把 valid 置 false 兜底。
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

// ============================================================================
// 可见性筛选
//
// 判据：板的法线朝向 plates[i](3) 与"相机→车中心"方位角 center_yaw 的夹角
//       小于 VISIBLE_LIMIT(75°)。返回的是下标数组。
//
// 几何直觉：车中心在前方 center_yaw 方向，一块 yaw ≈ center_yaw 的板，
//   位置在车中心【靠相机这一侧】（h_armor_xyz 里：板 = 中心 − r·(cos,sin)），
//   法线也正好朝着相机 —— 这就是正对着我的那块。
//   反过来 yaw 差 180° 的板在车的另一侧、背对着我，打上去也是擦边。
//
// ⚠ 这是个纯角度判据，没考虑车在画面里的张角：车离得越近，"75°"相对车本身
//   的角尺寸就越松，可能出现"几何上已经被车身挡住"的板仍被判为可见。
//   要更严谨得把 r 和距离也算进来（atan(r/dist)）。
// ============================================================================
std::vector<int> Aimer::visible_plates(
    const std::vector<Eigen::Vector4d> & plates,
    double center_yaw) const {
  std::vector<int> ids;
  for (int i = 0; i < (int)plates.size(); i++) {
    double delta = limit_rad(plates[i](3) - center_yaw);   // 回卷到 (-π, π]
    if (std::abs(delta) < VISIBLE_LIMIT)
      ids.push_back(i);
  }
  return ids;
}

// ============================================================================
// 给定状态算所有板位姿 [x,y,z,yaw]。
// 选板要用未来状态（子弹飞行期间车还在转），所以接一个状态参数。
// ============================================================================
std::vector<Eigen::Vector4d> Aimer::plates_of(const Target & t, const Eigen::VectorXd & x) {
  std::vector<Eigen::Vector4d> list;
  for (int i = 0; i < t.armor_num_; i++) {
    const double angle = limit_rad(x(6) + i * 2.0 * CV_PI / t.armor_num_);
    const Eigen::Vector3d p = t.h_armor_xyz(x, i);
    list.push_back({p(0), p(1), p(2), angle});
  }
  return list;
}

int Aimer::select_plate(const Target & target, const Eigen::VectorXd & x,
                        const cv::Point2f & img_center, const Solver & solver,
                        int locked_id) const {
  const auto plates = plates_of(target, x);
  const double center_yaw = std::atan2(x(2), x(0));
  const double omega = x(7);
  const auto vis = visible_plates(plates, center_yaw);
  if (vis.empty()) return -1;                     // 这个状态里没有"正面朝我"的板

  // 高速档：车走“命中时刻选板 + 换板门槛”；前哨站走原 argmin/argmax。
  // 低速档：锁定板可见就保持，否则选离画面中心最近的。
  if (std::abs(omega) >= LOW_W)
    return choose_high_speed(plates, vis, omega, center_yaw, target.is_outpost_,
                             locked_id);

  // 慢转档的迟滞：锁定的板在这个状态里还可见 → 继续打它，不重新打分（防准星来回跳）
  if (locked_id >= 0)
    for (int id : vis)
      if (id == locked_id) return locked_id;

  return choose_low_speed(target, x, vis, img_center, solver);
}

// ============================================================================
// 低速/静止目标的选板：投影到像素，选离画面中心最近的那块。
//
// 为什么选离中心近：画面中心≈枪口轴线，云台转角小，畸变和弹道误差也小。
// img_center 是画面几何中心，不是标定光心；对选板影响很小。
// x 是“算板位置用的状态”：车传未来状态，前哨站传当前状态。
// ============================================================================
int Aimer::choose_low_speed(const Target  & target,
                             const Eigen::VectorXd & x,
                             const std::vector<int> & visible_ids,
                             const cv::Point2f & img_center,
                             const Solver & solver) const {
  if (visible_ids.size() == 1)
    return visible_ids[0];

  int    best_id = visible_ids[0];
  double best_d2 = 1e10;          // 初始值取一个足够大的数，第一轮必被替换

  for (int id : visible_ids) {
    Eigen::Vector3d pw = target.h_armor_xyz(x, id);               // 板在 pseudo 系的位置
    cv::Point2f px = pseudo_to_pixel(pw, solver);                 // 投到像素
    double dx = px.x - img_center.x;
    double dy = px.y - img_center.y;
    double d2 = dx * dx + dy * dy;                                // 到画面中心的距离平方
    if (d2 < best_d2) {
      best_d2 = d2;
      best_id = id;
    }
  }
  return best_id;
}

// ============================================================================
// 小陀螺（|omega| >= LOW_W）的选板。
//
// 车：
//   1. 评估时刻 = 命中时刻 + CAR_SWITCH_DEG/|ω|（上限 0.2s）；
//   2. 在该时刻挑 |δ| 最小的板（δ = 板法线角 − 车心方位角）；
//   3. 锁定板仍可见、且没被别的板好出 CAR_SWITCH_DEG → 继续锁定。
//   换板因此发生在两块板等距之前，命中姿态更接近正对；代价是换板瞬间
//   指令会有一次跳变（4m 约 4.3°），可能触发节点的到位门。
//
// 前哨站：保持原 argmin/argmax 规则（只改车）。
// ============================================================================
int Aimer::choose_high_speed(const std::vector<Eigen::Vector4d> & plates,
                              const std::vector<int> & visible_ids,
                              double omega, double center_yaw, bool outpost,
                              int locked_id) const {
  // 前哨站：保持原 argmin/argmax 规则（只改车）
  if (outpost) {
    if (visible_ids.size() == 1)
      return visible_ids[0];

    std::vector<std::pair<int, double>> candidates;
    for (int id : visible_ids) {
      double delta = limit_rad(plates[id](3) - center_yaw);
      candidates.push_back({id, delta});
    }

    if (omega > 0) {
      // 选 delta 最小（最负）的 —— 顺着转向，它最先转到正对相机
      auto it = std::min_element(candidates.begin(), candidates.end(),
        [](const auto & a, const auto & b) { return a.second < b.second; });
      return it->first;
    } else {
      // 选 delta 最大（最正）的 —— 同理，逆着转向它最先转到正对相机
      auto it = std::max_element(candidates.begin(), candidates.end(),
        [](const auto & a, const auto & b) { return a.second < b.second; });
      return it->first;
    }
  }

  // ---- 车：命中时刻选 |δ| 最小 + 换板门槛 + 提前评估 ----
  // 评估状态来自 best_aim 迭代（已推到命中时刻）；这里把板法线再按 ω
  // 往前转 adv，等效于在“转过门槛角”的时刻比较。
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
    if (d < best) { pick = id; best = d; }
  }
  if (pick < 0) return -1;                 // 一块可见的都没有：本帧不打

  // 换板门槛（迟滞）：锁定板还可见，且没被别的板好出 CAR_SWITCH_DEG → 继续锁定。
  // 与 rps26 原文的差异：原文每帧从 index=0 扫、序号小的板占一个门限宽的优势；
  // 本工程作用在 locked_id 上，两个转向对称。
  if (locked_id >= 0 && pick != locked_id &&
      std::find(visible_ids.begin(), visible_ids.end(), locked_id) != visible_ids.end() &&
      std::abs(delta_eval(locked_id)) <= best + switch_tol)
    return locked_id;
  return pick;
}

// ============================================================================
// 斜抛弹道解算（真空、无空气阻力）
//
// 已知初速 v0，目标在水平距离 d、高度差 h 处，求发射仰角 pitch 和飞行时间。
//
// 推导（把 t 消掉后的标准斜抛方程）：
//   水平： d = v0·cos(p)·t
//   竖直： h = v0·sin(p)·t − ½·g·t²
//   消 t 后两边同除以 cos²p，整理成关于 tan(p) 的一元二次方程：
//       a·tan²p + b·tan(p) + c = 0
//   其中 a = g·d²/(2·v0²)，b = −d，c = a + h
//
// 判别式 Δ < 0 → 这个初速够不着那么远/那么高，置 unsolvable 返回。
// Δ ≥ 0 有两个根，对应两条弹道（低伸弹道 / 高抛弹道）；取【飞行时间短】
// 的那条 —— 飞得越久，目标机动得越多、误差越大。
//
// ⚠ 退化输入：d = 0 时 a = 0，2a 除零会算出 NaN（Δ 恰好为 0，-b±0 除以 0）。
//   实车上板不可能正好在相机正上方，所以没做保护；要复用此函数需给 d 加下限。
// ⚠ 没有空气阻力、也没有考虑枪口与相机的安装角差（题目前提 2 说二者重合）。
// ============================================================================
Aimer::TrajResult Aimer::solve_traj(double v0, double d, double h) const {
  TrajResult r;
  double a = g_ * d * d / (2.0 * v0 * v0);
  double b = -d;
  double c = a + h;
  double delta = b * b - 4.0 * a * c;

  if (delta < 0) { r.unsolvable = true; return r; }

  double tan1 = (-b + std::sqrt(delta)) / (2.0 * a);   // 高抛解
  double tan2 = (-b - std::sqrt(delta)) / (2.0 * a);   // 低伸解
  double p1 = std::atan(tan1), p2 = std::atan(tan2);
  double t1 = d / (v0 * std::cos(p1));
  double t2 = d / (v0 * std::cos(p2));

  // 两条弹道都能命中，取飞得快的（低伸）那条
  if (t1 < t2) { r.pitch = p1; r.fly_time = t1; }
  else         { r.pitch = p2; r.fly_time = t2; }

  r.unsolvable = false;
  return r;
}

// ============================================================================
// 基地：直接瞄准（不预测）
//
// 和 best_aim 的区别只有一个字：**不提前**。目标是静止结构，不需要外推，
// 也不需要 EKF —— 直接拿这一帧的板中心解弹道、把瞄准点抬高子弹的下坠量。
// ============================================================================
Aimer::AimResult Aimer::best_aim_direct(const Eigen::Vector3d & xyz_state,
                                        const Solver & solver) const {
  AimResult r;

  const double d = std::sqrt(xyz_state(0) * xyz_state(0) + xyz_state(1) * xyz_state(1));
  const double h = xyz_state(2);

  const TrajResult tr = solve_traj(bullet_speed_, d, h);
  if (tr.unsolvable) return r;            // 这个初速够不着，valid 保持 false

  r.valid       = true;
  r.converged   = true;                   // 基地是解析解（无迭代），恒收敛
  r.flight_time = tr.fly_time;
  r.pitch       = tr.pitch;               // 发射仰角（pitch 里已经含重力）
  r.plate_id    = 0;

  // 给可视化用的瞄准点：板中心 + 抬高 ½g t²（子弹飞行中会掉这么多）
  r.aim_3d = xyz_state;
  r.aim_3d(2) += 0.5 * g_ * tr.fly_time * tr.fly_time;
  r.aim_2d = pseudo_to_pixel(r.aim_3d, solver);
  return r;
}

// ============================================================================
// 匀速模型外推（纯线性，不涉及协方差）
//
// 跟 Target::predict() 的区别：那个是 EKF 的预测步，会同时滚协方差 P、要处理
// 过程噪声 Q，而且【会改动 target 自己的状态】；这里只是"试算一下未来"，
// 所以做成 static + 按值传参，绝不碰 tracker 里那个正在跑的滤波器。
// （和 Tracker::predict_armor() 是同一类"影子预测"。）
//
//   位置 += 速度·dt，yaw += omega·dt，其余分量（速度/角速度/r/dr/dh）不变
//   —— 匀速假设下高阶项不用管，这正是 Target::predict() 里 F 矩阵的非零项。
// ============================================================================
Eigen::VectorXd Aimer::predict_state(const Eigen::VectorXd & x, double dt) {
  Eigen::VectorXd xp = x;
  xp(0) += xp(1) * dt;
  xp(2) += xp(3) * dt;
  xp(4) += xp(5) * dt;
  // yaw 是唯一需要回卷的分量，不回卷就会出 179°+2°=181° 的经典 bug
  xp(6)  = limit_rad(xp(6) + xp(7) * dt);
  return xp;
}

// ============================================================================
// pseudo 系 → 像素
//
// 两步并作一步写：
//   1. pseudo → 相机：cam = (p.x, −p.z, p.y)，就是 Target::pseudo_to_camera()
//      （pseudo 的"上" p.z 变成相机的 −y，"前" p.y 变成相机的 z）
//   2. 相机 → 像素：Solver::project_world()，针孔模型；
//      注意它在 z ≤ 0（点在相机背后）时会返回 (−1, −1) 作为哨兵值
// ============================================================================
cv::Point2f Aimer::pseudo_to_pixel(const Eigen::Vector3d & p,
                                    const Solver & solver) {
  Eigen::Vector3d cam(p(0), -p(2), p(1));
  return solver.project_world(cam);
}
