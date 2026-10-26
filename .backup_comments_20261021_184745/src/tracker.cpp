#include "tracker.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>

Tracker::Tracker(const YAML::Node & cfg, Solver & solver) : solver_(solver) {
  std::string ec = cfg["enemy_color"].as<std::string>();
  enemy_color_ = (ec == "blue") ? blue : red;

  // 丢帧容忍：连续多少帧看不到锁定的目标才真解锁（会进 temp_lost 再回来）。
  // 默认 15 帧 —— 和 sp_vision_25 的 max_temp_lost_count 一样。
  if (cfg["max_temp_lost_count"])
    max_temp_lost_count_ = cfg["max_temp_lost_count"].as<int>();
}

// ----------------------------------------------------------------------------
// 初始化一个 Target（= 给它建一套 EKF）
//
// 这里做的事：根据【兵种】决定三个东西
//   1. 车中心到装甲板的半径 r（.cpp 里那个 0.2765 / 0.3205 / 0.20）
//   2. 装甲板数量 armor_num（3 或 4）
//   3. 初始协方差 P0_dig（对角阵的对角元，表示"开局我有多不确定"）
//
// 为什么半径和板数要按兵种区分？
//   步兵/英雄是 4 块板的正交布局，前哨站和基地是 3 块板的三角形布局。
//   半径是几何参数，直接决定 EKF 里 h(x) 这个观测量算出来对不对。
//
// P0_dig 是 11 维状态向量 x 的初始方差，状态定义见 target.cpp:63-67：
//   [0]x [1]vx [2]y [3]vy [4]z [5]vz [6]yaw [7]omega [8]r [9]dr [10]dh
//   （第三个分量 z 是竖直方向，"半径" r 指的是车中心到装甲板在水平面内的距离）
//
// 大装甲板(P0 里 [9][10] 为 1) 和 小装甲板(为 0.01) 的区别：
//   大板只有 1、基地才有，尺寸大、特征明显，PnP 解出来的位姿更可信，
//   所以初始给它更小的方差。
// ----------------------------------------------------------------------------
void Tracker::init_target(Target & t, const Armor & a,
                          std::chrono::steady_clock::time_point now) {
  Eigen::VectorXd P0_dig(11);

  switch (a.name) {
    case outpost:
      // 前哨站走 10 维专用模型（见 target.hpp 顶部）：
      //   [x, vx, y, vy, z, vz, yaw, v_yaw, dz1, dz2]
      // 半径固定 kOutpostR（0.275，旋转直径 550mm），不再当状态估；
      // 转速靠投票定成 ±0.8π，所以 v_yaw 的初值方差给大一点（6.25）。
      P0_dig << 0.04, 0.01, 0.04, 0.01, 0.04, 0.01, 0.25, 6.25, 0.1, 0.1;
      t.init(a, kOutpostR, 3, P0_dig);   // 3 块板、半径 0.275
      break;

    case base:
      // 基地同理，但 z 方向更不确定（81 -> 64），因为基地位置更远
      P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0;
      t.init(a, 0.3205, 3, P0_dig);      // 基地：半径 0.3205，3 块板
      break;

    case three:
    case four:
    case five:
      // 这三块板是同一个机器人的（编号来自 YOLO 分类头），所以模型一样：
      // 半径 0.20、4 块板
      if (a.type == big) {
        // 大装甲板：yaw 和半径的初始方差给得小（0.4 / 100 / 0.01,1,1）
        P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 0.01, 1, 1;
        t.init(a, 0.20, 2, P0_dig);      // !! 注意这里是 2，不是 4
      } else {
        P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 0.01, 1, 1;
        t.init(a, 0.20, 4, P0_dig);
      }
      break;

    default:
      // one / two / sentry / not_armor 等都走这里
      P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 0.01, 1, 1;
      t.init(a, 0.20, 4, P0_dig);
      break;
  }

  // 本 track 的"上次更新时刻"。dt 是按这个算的，所以一定要在 init 时设上
  last_t_[a.name] = now;
}

// ----------------------------------------------------------------------------
// 每帧的主循环
//
// 三个阶段：
//   1. 维护锁定状态（丢帧计数、超时解锁）
//   2. 【所有 track】推进时间（predict）—— 与这一帧有没有观测无关
//   3. 用本帧的观测更新（update）：遍历检测到的板，找到/新建它的 track
//      （没有锁定时顺便 lock_target）
// ----------------------------------------------------------------------------
void Tracker::update(const std::list<Armor> & armors,
                     std::chrono::steady_clock::time_point now) {

  // === 阶段1：锁定维护 ===
  // 当前锁定的那个编号，本帧还在不在检测结果里？
  bool locked_seen = false;
  if (has_lock()) {
    for (auto & a : armors) {
      if ((int)a.name == locked_id_) { locked_seen = true; break; }
    }
    if (!locked_seen) {
      // 没看到（被遮挡/转走了），计数；连续 max_temp_lost_count_ 帧都没看到才真解锁。
      // 短暂丢失很常见（车转走了、板被自己挡住），立刻解锁会导致目标乱跳。
      // 这个值由 config 的 max_temp_lost_count 给（默认 15，和 sp_vision 一致）。
      lost_frames_++;
      if (lost_frames_ >= max_temp_lost_count_) {
        locked_id_ = -1;
        lost_frames_ = 0;
      }
    } else {
      lost_frames_ = 0;   // 看到了就清零
    }
  }

  // === 阶段2：所有 track 先推进时间 ===
  //
  // ★ 时间推进和"这一帧有没有观测"无关：车不管你看不看它，都在走、都在转。
  //   所以 predict 必须对所有 track 每帧都做，只有 update（阶段3）才依赖观测。
  //
  // 旧写法把"算 dt + predict"塞在下面那个 armors 循环里，后果是：
  //   "这一帧没被检测到的车"状态和时间戳都冻在原地 —— 而世界已经过了好几帧。
  //   等它重现时（比如被遮挡 5 帧后重新识别），状态还停在 5 帧前，
  //   预测点就会偏到板点的侧下方，然后连续几帧往回追（看起来就是抖）。
  {
    // 单次推进的时间上限：长时间没有观测时，Q 会按 dt⁴ 膨胀，
    // 给个上限避免一次把协方差吹到离谱（更严谨的做法是超限就重建 track）。
    constexpr double MAX_COAST = 0.5;

    for (auto & kv : tracks_) {
      int vid = kv.first;
      Target & t = kv.second;

      // 用【这个 track 上次被推进的时刻】算 dt，而不是上一帧的时刻：
      // 中间丢了几帧（被挡住/漏检）时，间隔本来就不是 1/wait_ms。
      auto lt = last_t_.find(vid);
      if (lt == last_t_.end()) continue;

      double dt = std::chrono::duration<double>(now - lt->second).count();
      lt->second = now;                  // 不管 dt 合不合法，时刻都要推进
      if (dt <= 0) continue;             // 时间戳倒流 / 同一帧被推进第二次
      if (dt > MAX_COAST) dt = MAX_COAST;

      t.predict(dt);
    }
  }

  // === 阶段3：用本帧的观测更新 ===
  // 逐个遍历检测到的板：找到它所属的 track（没有就新建），然后喂给 EKF。
  // 注意这是【逐块板】的循环，一辆车这帧看到 2 块板就会进来 2 次，
  // 同一个 Target 会被连续更新两次 —— 这是刻意的（两块板是两条独立观测）。
  for (auto & a : armors) {
    // !!! 易看漏：track 的身份 = 装甲板编号 (int)a.name，不是 IOU 匹配，
    //     也不是位置最近匹配。也就是说"编号为 three 的那块板"永远对应同一个 track。
    //     这在装甲板编号会互换的场景下会出问题（见下方 t.update 后 jumped 的说明）。
    int vid = (int)a.name;

    // !!! 易看漏：这行在循环体里，每轮都调；
    //     但 lock_target 内部会重设 locked_id_，所以第二块板进来时 has_lock() 已经是 true，
    //     实际只会在"本帧第一次遇到没锁定"时生效一次。功能上没错，写法上冗余。
    if (!has_lock()) lock_target(armors);

    auto it = tracks_.find(vid);
    if (it == tracks_.end()) {
      // 第一次见到这个编号 -> 新建 track
      // （init_target 会把状态建在 now 时刻，所以这一帧不需要再 predict）
      Target t;
      init_target(t, a, now);            // 内部会设置 last_t_[vid] = now
      tracks_[vid] = t;
      it = tracks_.find(vid);
    }

    auto & t = it->second;

    // --- 发散自愈 ---
    // diverged() 判的是"半径估计跑出 (0.05, 0.5)"，也就是滤波器已经跟丢了。
    // 以前这里什么都不做：发散只被 main/visualizer 读来"跳过动作"，而 tracks_
    // 只增不删、发散后也没有任何复位 —— 结果是【某台车一旦发散就整局废掉】：
    // 锁着它但不瞄不打，而且再也不会恢复。
    // 现在：发散时拿本帧这块板的观测把它重建一次，从头收敛。
    if (t.diverged()) {
      init_target(t, a, now);
      ++diverged_resets_;
    }

    // --- 组装观测 z 和观测噪声 R ---
    // 观测是 4 维：[yaw, pitch, dist, face_yaw]
    // camera_to_obs 把"相机系下的三维坐标 + 装甲板法线角"转成前三个球坐标分量
    Eigen::Vector4d z_obs = Target::camera_to_obs(a.xyz_in_camera, a.yaw);

    // delta_angle = 观测到的装甲板法线方向 与 车中心朝向 的夹角
    //   正面朝向车时接近 0，侧面装甲板会接近 pi/2
    double center_yaw = std::atan2(t.ekf_.x(2), t.ekf_.x(0));
    double delta_angle = std::abs(limit_rad(a.yaw - center_yaw));
    double dist = a.xyz_in_camera.norm();

    // --- 观测噪声 R（对角阵）---
    // 这四个数不是拍脑袋，是按"这个观测量有多不可信"来的：
    //   [0] yaw      : 固定 4e-3
    //   [1] pitch    : 固定 4e-3
    //   [2] dist     : log(delta_angle + 1) + 1
    //                  板越"侧"（delta_angle 越大），PnP 解出的距离越不准
    //   [3] face_yaw : log(dist + 1) / 200 + 9e-2
    //                  距离越远，角度分辨率越差，噪声越大
    // 注意 R 是【逐块板、逐帧】算出来的，不是固定常数 —— 这是实车上的关键，
    // 如果所有观测共用一个固定 R，侧面那块倾斜的板会把状态拽偏。
    Eigen::Vector4d R_dig;
    R_dig << 4e-3,
             4e-3,
             std::log(delta_angle + 1.0) + 1.0,
             std::log(dist + 1.0) / 200.0 + 9e-2;

    // --- EKF 更新步 ---
    // 内部会做：① 用当前状态预测每块装甲板的位置
    //           ② 挑出和这个观测最匹配的那块（数据关联，见 target.cpp:204）
    //           ③ 按正常的 EKF 公式修正状态
    // 如果关联结果不是第 0 块，会把 t.jumped 置 true（表示"编号跳板了"）
    t.update(z_obs, R_dig);
  }
  // 注意：本帧没出现的编号，阶段2 里照样会被 predict（时间照常推进），
  //      只是不会 update。tracks_ 里的 Target 从不删除，所以见过的车会一直留着。
}

// ----------------------------------------------------------------------------
// 从本帧检测到的装甲板里选一个锁定
//
// 排序规则：优先级相同时比距离（近的先锁）
// 优先级表见 types.hpp:38 get_armor_priority()，经验值：
//   前哨站(1) < 基地(2) < 三/四/五号板(3) < 哨兵(4) < 一号板(5) < 二号板(6)
// 也就是优先打前哨站，其次基地，然后是普通机器人。
// ----------------------------------------------------------------------------
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
      a.xyz_in_camera.norm()             // 用板中心到相机的距离做近远排序
    });
  }

  std::sort(candidates.begin(), candidates.end(),
    [](const Candidate & a, const Candidate & b) {
      if (a.priority != b.priority) return a.priority < b.priority;  // 优先级为主
      return a.distance < b.distance;                                // 距离为辅
    });

  locked_id_ = candidates.front().vid;
  lost_frames_ = 0;
}

// ----------------------------------------------------------------------------
// 【试算】dt 秒后某块装甲板的位置，但不改变 t 的状态
//
// 为什么要这么绕？因为 Target::predict() 是【原地修改】state 和 P 的。
// 想"试算"就必须先备份、算完再还原。而 t 是 const 引用，只能 const_cast。
//
// 典型用途：visualizer.cpp:60 画那个红色预测箭头 —— 主循环里的 EKF 已经
// 用当前帧的观测更新过了，如果直接再 predict 一次会污染状态。
//
// 返回的是【相机系】坐标（pseudo_to_camera 做了一次坐标变换，
// 因为 EKF 内部用的是 pseudo 系：x 右、y 前、z 上，见 target.cpp:13-18）
// ----------------------------------------------------------------------------
Eigen::Vector3d Tracker::predict_armor(const Target & t, int armor_id, double dt) const {

  // 备份 EKF 的完整状态（均值和协方差都要）
  Eigen::VectorXd x_saved = t.ekf_.x;
  Eigen::MatrixXd P_saved = t.ekf_.P;

  Target & mutable_t = const_cast<Target &>(t);
  mutable_t.predict(dt);

  Eigen::Vector3d pseudo_pos = t.h_armor_xyz(mutable_t.ekf_.x, armor_id);

  // 还原，保证本函数对 t 没有任何副作用
  mutable_t.ekf_.x = x_saved;
  mutable_t.ekf_.P = P_saved;

  return Target::pseudo_to_camera(pseudo_pos);
}

// ----------------------------------------------------------------------------
// 数据关联：判断这个观测对应模型里的第几块装甲板（返回 0..armor_num_-1）
//
// 背景：Target 是"整个车"的模型，内部按 yaw 均匀排布着 4 块板，
//       但一帧里通常只能看到 1~2 块。需要知道"我看到的是第几块"，
//       才能用对应那块板的几何关系去做 EKF 更新。
//
// 做法：对模型的每块板，算它"如果被观测到，yaw 和 face_yaw 应该是多少"，
//       和实际观测比，取误差最小的。
//
// !!! 易看漏：先按【距离】排序，然后只在最近的 3 块里找最优。
//     为什么要限制？
//       因为装甲板绕车中心均匀分布时，有几块板在观测角度上可能非常接近
//       （尤其 yaw、omega 还没收敛的时候），全找会匹配到车背面的板上去。
//       限制在最近的 3 块是个启发式，利用"能看到的板肯定是近的那几块"。
//     这也是个隐患：如果可见性判断和实际不符，会关联错板。
// ----------------------------------------------------------------------------
int Tracker::get_armor_plate_id(const Target & t, const Eigen::Vector4d & obs) const {
  const auto & xyza_list = t.armor_xyza_list();   // 4 块板的 [x,y,z,yaw]

  std::vector<std::pair<Eigen::Vector4d, int>> candidates;
  for (int i = 0; i < t.armor_num_; i++)
    candidates.push_back({xyza_list[i], i});

  // 按到相机的距离从近到远排
  std::sort(candidates.begin(), candidates.end(),
    [](const auto & a, const auto & b) {
      double da = Eigen::Vector3d(a.first(0), a.first(1), a.first(2)).norm();
      double db = Eigen::Vector3d(b.first(0), b.first(1), b.first(2)).norm();
      return da < db;
    });

  int best_id = 0;
  double min_err = 1e10;
  for (int i = 0; i < std::min(3, t.armor_num_); i++) {   // 只看最近的 3 块
    const auto & xyza = candidates[i].first;
    // 两个误差：位置向量的朝向角 和 装甲板法线角（都要归一化到 (-pi, pi]）
    double yaw_err   = std::abs(limit_rad(obs(0) - std::atan2(xyza(1), xyza(0))));
    double angle_err = std::abs(limit_rad(obs(3) - xyza(3)));
    double err = yaw_err + angle_err;
    if (err < min_err) { min_err = err; best_id = candidates[i].second; }
  }

  return best_id;
}
