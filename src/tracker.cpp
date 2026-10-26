#include "tracker.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>

Tracker::Tracker(const YAML::Node & cfg, Solver & solver) : solver_(solver) {
  std::string ec = cfg["enemy_color"].as<std::string>();
  enemy_color_ = (ec == "blue") ? blue : red;

  //锁定目标连续丢帧超过该值才解锁
  if (cfg["max_temp_lost_count"])
    max_temp_lost_count_ = cfg["max_temp_lost_count"].as<int>();
}

//按兵种设置半径、板数和初始方差
void Tracker::init_target(Target & t, const Armor & a,
                          std::chrono::steady_clock::time_point now) {
  Eigen::VectorXd P0_dig(11);

  switch (a.name) {
    case outpost:
      //前哨站 10 维，半径固定，v_yaw 初值方差较大
      P0_dig << 0.04, 0.01, 0.04, 0.01, 0.04, 0.01, 0.25, 6.25, 0.1, 0.1;
      t.init(a, kOutpostR, 3, P0_dig); //3 块板，半径 0.275
      break;

    case base:
      //基地位置更远，位置方差更大
      P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0;
      t.init(a, 0.3205, 3, P0_dig); //3 块板，半径 0.3205
      break;

    case three:
    case four:
    case five:
      //3/4/5 号板按 0.20m 半径建模
      if (a.type == big) {
        P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 0.01, 1, 1;
        t.init(a, 0.20, 2, P0_dig); //大板按 2 块板建模
      } else {
        P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 0.01, 1, 1;
        t.init(a, 0.20, 4, P0_dig);
      }
      break;

    default:
      P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 0.01, 1, 1;
      t.init(a, 0.20, 4, P0_dig);
      break;
  }

  //init 在 now 时刻建立状态，本帧不再预测
  last_t_[a.name] = now;
}

//维护锁定、推进所有 track，再用本帧观测更新
void Tracker::update(const std::list<Armor> & armors,
                     std::chrono::steady_clock::time_point now) {

  //锁定目标本帧仍被检测到则清零丢帧计数，否则累计到上限后解锁
  bool locked_seen = false;
  if (has_lock()) {
    for (auto & a : armors) {
      if ((int)a.name == locked_id_) { locked_seen = true; break; }
    }
    if (!locked_seen) {
      lost_frames_++;
      if (lost_frames_ >= max_temp_lost_count_) {
        locked_id_ = -1;
        lost_frames_ = 0;
      }
    } else {
      lost_frames_ = 0;
    }
  }

  //无论本帧是否有观测，所有track都按各自上次时刻推进预测
  {
    //单次预测的时间上限，避免协方差被大dt吹飞
    constexpr double MAX_COAST = 0.5;

    for (auto & kv : tracks_) {
      int vid = kv.first;
      Target & t = kv.second;

      //dt 取该 track 上次推进到当前的间隔，可跨过漏检帧
      auto lt = last_t_.find(vid);
      if (lt == last_t_.end()) continue;

      double dt = std::chrono::duration<double>(now - lt->second).count();
      lt->second = now;
      if (dt <= 0) continue;
      if (dt > MAX_COAST) dt = MAX_COAST;

      t.predict(dt);
    }
  }

  //逐块板更新；同一车本帧看到两块板就连续更新两次
  for (auto & a : armors) {
    //track 身份为 (int)a.name，同一编号始终对应同一 track
    int vid = (int)a.name;

    if (!has_lock()) lock_target(armors);

    auto it = tracks_.find(vid);
    if (it == tracks_.end()) {
      //首次见到该编号，用当前观测建立 track
      Target t;
      init_target(t, a, now);
      tracks_[vid] = t;
      it = tracks_.find(vid);
    }

    auto & t = it->second;

    //发散时用当前观测重建该 track
    if (t.diverged()) {
      init_target(t, a, now);
      ++diverged_resets_;
    }

    //观测为 [yaw,pitch,dist,face_yaw]
    Eigen::Vector4d z_obs = Target::camera_to_obs(a.xyz_in_camera, a.yaw);

    //法线角与车心方位角之差，越大说明板越侧
    double center_yaw = std::atan2(t.ekf_.x(2), t.ekf_.x(0));
    double delta_angle = std::abs(limit_rad(a.yaw - center_yaw));
    double dist = a.xyz_in_camera.norm();

    //观测噪声 R：距离随侧视角度增大，face_yaw 随距离增大
    Eigen::Vector4d R_dig;
    R_dig << 4e-3,
             4e-3,
             std::log(delta_angle + 1.0) + 1.0,
             std::log(dist + 1.0) / 200.0 + 9e-2;

    //内部完成数据关联和 EKF 更新
    t.update(z_obs, R_dig);
  }
  //无观测的 track 只预测不更新，tracks_ 中的条目不会删除
}

//按优先级选锁，优先级相同取距离最近
void Tracker::lock_target(const std::list<Armor> & armors) {
  if (armors.empty()) return;

  struct Candidate {
    int vid;
    int priority;
    double distance;
  };
  std::vector<Candidate> candidates;
  for (auto & a : armors) {
    candidates.push_back({
      (int)a.name,
      get_armor_priority(a.name),
      a.xyz_in_camera.norm()
    });
  }

  std::sort(candidates.begin(), candidates.end(),
    [](const Candidate & a, const Candidate & b) {
      if (a.priority != b.priority) return a.priority < b.priority;
      return a.distance < b.distance;
    });

  locked_id_ = candidates.front().vid;
  lost_frames_ = 0;
}

//试算 dt 秒后某块板的位置，返回相机系，不修改 t 的状态
Eigen::Vector3d Tracker::predict_armor(const Target & t, int armor_id, double dt) const {

  //predict 会原地修改状态，先备份均值和协方差
  Eigen::VectorXd x_saved = t.ekf_.x;
  Eigen::MatrixXd P_saved = t.ekf_.P;

  Target & mutable_t = const_cast<Target &>(t);
  mutable_t.predict(dt);

  Eigen::Vector3d pseudo_pos = t.h_armor_xyz(mutable_t.ekf_.x, armor_id);

  mutable_t.ekf_.x = x_saved;
  mutable_t.ekf_.P = P_saved;

  return Target::pseudo_to_camera(pseudo_pos);
}

//数据关联：返回观测对应的板编号 0..armor_num_-1
int Tracker::get_armor_plate_id(const Target & t, const Eigen::Vector4d & obs) const {
  const auto & xyza_list = t.armor_xyza_list();

  std::vector<std::pair<Eigen::Vector4d, int>> candidates;
  for (int i = 0; i < t.armor_num_; i++)
    candidates.push_back({xyza_list[i], i});

  //按到相机距离由近到远排序
  std::sort(candidates.begin(), candidates.end(),
    [](const auto & a, const auto & b) {
      double da = Eigen::Vector3d(a.first(0), a.first(1), a.first(2)).norm();
      double db = Eigen::Vector3d(b.first(0), b.first(1), b.first(2)).norm();
      return da < db;
    });

  int best_id = 0;
  double min_err = 1e10;
  //只在最近 3 块中匹配，降低旋转时误关联到背面板的概率
  for (int i = 0; i < std::min(3, t.armor_num_); i++) {
    const auto & xyza = candidates[i].first;
    //比较方位角与法线角，角度差先回卷
    double yaw_err   = std::abs(limit_rad(obs(0) - std::atan2(xyza(1), xyza(0))));
    double angle_err = std::abs(limit_rad(obs(3) - xyza(3)));
    double err = yaw_err + angle_err;
    if (err < min_err) { min_err = err; best_id = candidates[i].second; }
  }

  return best_id;
}
