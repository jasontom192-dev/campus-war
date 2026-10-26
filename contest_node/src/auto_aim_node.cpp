//自瞄节点：/autoaim/frame → 检测 → PnP → 世界系 EKF/瞄准 → /target_angles_player_N
//规则 7.1/7.2/7.5：同帧 camera_info、嵌套时间戳一致、帧龄 250ms、序号回退清 tracker
//观测经 world_from_cam 转世界系；输出 world_yaw=atan2(右,前)、world_pitch=-仰角（不再加云台角）

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>

#include "tdt_interface/msg/auto_aim_frame.hpp"
#include "tdt_interface/msg/rune_request.hpp"
#include "tdt_interface/msg/rune_result.hpp"
#include "tdt_interface/msg/send_data.hpp"

#include "aimer.hpp"
#include "detector.hpp"
#include "nav_grid.hpp"
#include "solver.hpp"
#include "tracker.hpp"
#include "visualizer.hpp"

//打符（kRune）：RuneDetector 检测 + third_party/campus-rune 精修/位姿/相位/弹道
#include "rune_activation.hpp"
#include "rune_detector.hpp"
#include "rune_campus.hpp"
#include "power_rune_interface.hpp" //debug_get_tracked()（RUNE_DEBUG=1 时用）

using namespace std::chrono_literals;

namespace {

//规则 7.2：帧龄窗口与允许的时钟抖动
constexpr int64_t kMaxFrameAgeNs = 250'000'000; //250 ms
constexpr int64_t kMaxFutureNs   = 20'000'000; //20 ms

//规则 7.2：与 Unity 侧一致的 0.5 s 超时
constexpr double kCommandTimeoutSec = 0.5;

//表 12 标称内参（960×720，无畸变），仅作回落
constexpr double kFallbackFx = 1159.420, kFallbackFy = 1159.420;
constexpr double kFallbackCx = 480.0,    kFallbackCy = 360.0;

int64_t stamp_ns(const std_msgs::msg::Header & h) {
  return static_cast<int64_t>(h.stamp.sec) * 1'000'000'000LL +
         static_cast<int64_t>(h.stamp.nanosec);
}
double rad2deg(double r) { return r * 180.0 / M_PI; }
double deg2rad(double d) { return d * M_PI / 180.0; }

//收进 (-180,180]；先 fmod，巨值直接 while 会卡死
double wrap180(double d) {
  d = std::fmod(d, 360.0);
  while (d > 180.0) d -= 360.0;
  while (d <= -180.0) d += 360.0;
  return d;
}

//云台姿态→世界系旋转（相机排版）：yaw 自 +Z 起顺时针为正，pitch 向下为正，R=Rz(-yaw)Rx(-pitch)
Eigen::Matrix3d world_from_cam(double yaw_deg, double pitch_deg) {
  Eigen::Matrix3d M; //pseudo = M · cam
  M << 1, 0, 0,
       0, 0, 1,
       0,-1, 0;
  const Eigen::Matrix3d R_pseudo =
      (Eigen::AngleAxisd(-deg2rad(yaw_deg), Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(-deg2rad(pitch_deg), Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  return M.transpose() * R_pseudo * M;
}

//基地上下两块板同现时，选离画面中心最近的一块（枪口轴线附近，云台转得少）
const Armor * pick_base_plate(const std::list<Armor> & plates, const cv::Size & sz) {
  const Armor * best = nullptr;
  double best_d = 1e18;
  for (const auto & a : plates) {
    const cv::Point2f c =
        (a.points[0] + a.points[1] + a.points[2] + a.points[3]) * 0.25f;
    const double d = std::hypot(c.x - sz.width / 2.0, c.y - sz.height / 2.0);
    if (d < best_d) { best_d = d; best = &a; }
  }
  return best;
}

//观测位置和板法线（a.yaw=atan2(nz,nx)）转到世界系；rvec/tvec/points 留在相机系供画框
void armors_to_world(std::list<Armor> & armors, const Eigen::Matrix3d & R) {
  for (auto & a : armors) {
    a.xyz_in_camera = R * a.xyz_in_camera;
    Eigen::Vector3d n(std::cos(a.yaw), 0.0, std::sin(a.yaw));
    n = R * n;
    a.yaw = std::atan2(n(2), n(0));
  }
}

} //namespace

class AutoAimNode : public rclcpp::Node {
public:
  //状态机：kSearch 搜索 / kDetecting 确认 / kTracking 跟踪 / kTempLost 暂丢 / kNoAmmo 无弹（不动作时重发上一条指令）
  enum class AimState { kSearch, kDetecting, kTracking, kTempLost, kNoAmmo };

  //工作模式：kArmor 车自瞄；kRune 打符（rune_active_s 窗口后回车，rune.mode 可强制）
  enum class Mode { kArmor, kRune };

  AutoAimNode() : Node("auto_aim_node") {
    const int player_n = declare_parameter("player_n", 1); //1 蓝方 / 2 红方
    const std::string enemy_color =
        declare_parameter("enemy_color", std::string("red"));
    publish_angles_ = declare_parameter("publish_angles", false); //false = 只观察
    auto_fire_      = declare_parameter("auto_fire", false);
    show_window_    = declare_parameter("show_window", false);
    pitch_limit_    = declare_parameter("pitch_limit_deg", 60.0);
    const std::string cfg_path =
        declare_parameter("config_file", std::string("config_contest.yaml"));

    cfg_ = YAML::LoadFile(cfg_path);
    cfg_["enemy_color"] = enemy_color;
    enemy_ = (enemy_color == "blue") ? blue : red;

    //开火决策容差与距离分档
    if (cfg_["first_tolerance"])  first_tol_  = cfg_["first_tolerance"].as<double>();
    if (cfg_["second_tolerance"]) second_tol_ = cfg_["second_tolerance"].as<double>();
    if (cfg_["judge_distance"])   judge_dist_ = cfg_["judge_distance"].as<double>();

    if (cfg_["search_yaw_rate"])     search_rate_   = cfg_["search_yaw_rate"].as<double>();
    if (cfg_["max_temp_lost_count"]) max_temp_lost_ = cfg_["max_temp_lost_count"].as<int>();
    if (cfg_["min_detect_count"])    min_detect_count_ = cfg_["min_detect_count"].as<int>();

    //最大开火水平距离；只挡开火不挡瞄准，<=0 不限。锁定/跟踪不限距离
    if (cfg_["max_fire_distance"]) max_fire_distance_ = cfg_["max_fire_distance"].as<double>();
    //弹量 <= min_ammo_to_fire_ 视为打空（默认 0）
    if (cfg_["min_ammo_to_fire"]) min_ammo_to_fire_ = cfg_["min_ammo_to_fire"].as<int>();

    //底盘速度门：Unity 单位/s（3 单位=1m），<=0 关闭
    if (cfg_["fire_max_chassis_speed"])
      fire_max_chassis_speed_ = cfg_["fire_max_chassis_speed"].as<double>();

    //前哨站锁定后 outpost_settle_s 秒内不开火（等 dz1/dz2 收敛）
    if (cfg_["outpost_settle_s"]) outpost_settle_s_ = cfg_["outpost_settle_s"].as<double>();

    //车换板后 plate_switch_settle_s 秒内不开火（前哨站不适用）
    if (cfg_["plate_switch_settle_s"])
      plate_switch_settle_s_ = cfg_["plate_switch_settle_s"].as<double>();

    cv::Mat K0 = cv::Mat::eye(3, 3, CV_64F), D0 = cv::Mat::zeros(1, 5, CV_64F);
    K0.at<double>(0, 0) = kFallbackFx; K0.at<double>(1, 1) = kFallbackFy;
    K0.at<double>(0, 2) = kFallbackCx; K0.at<double>(1, 2) = kFallbackCy;

    detector_  = std::make_unique<Detector>(cfg_);
    solver_    = std::make_unique<Solver>(K0, D0);
    tracker_   = std::make_unique<Tracker>(cfg_, *solver_);
    aimer_     = std::make_unique<Aimer>(25.0); //初速每帧按 state 更新
    visualizer_ = std::make_unique<Visualizer>(*solver_);

    //打符配置：缺 rune 段或模型加载失败只禁用 kRune，车自瞄照常
    {
      const YAML::Node rc = cfg_["rune"];
      if (rc) {
        if (rc["mode"]) {
          const std::string m = rc["mode"].as<std::string>();
          if (m == "rune")  { forced_mode_ = Mode::kRune;  mode_forced_ = true; }
          if (m == "armor") { forced_mode_ = Mode::kArmor; mode_forced_ = true; }
        }
        if (rc["rune_active_s"])  rune_active_s_ = rc["rune_active_s"].as<double>();
        if (rc["max_small_runes"]) max_small_runes_ = rc["max_small_runes"].as<int>();
        if (rc["lost_tolerance_frames"]) rune_lost_tolerance_ = rc["lost_tolerance_frames"].as<int>();
        //到打符位后自动叫符的重试间隔（秒→ms）
        if (rc["request_retry_s"])
          rune_req_retry_ms_ = static_cast<int>(rc["request_retry_s"].as<double>() * 1000.0);

        //弹道/开火冷却/二值化阈值在 third_party/campus-rune/config/power_rune.json

        //符灯=本方颜色：由敌方颜色反推（敌红→蓝=1）；rune.target_color 可覆盖
        rune_my_color_ = (enemy_color == "red") ? 1 : 0;
        if (rc["target_color"]) {
          const std::string tc = rc["target_color"].as<std::string>();
          if (tc == "red")  rune_my_color_ = 0;
          if (tc == "blue") rune_my_color_ = 1;
        }

        //搜索：俯仰按累计转角的余弦在 [min,max] 之间平滑来回
        if (rc["search_pitch_min_deg"]) search_pitch_min_ = rc["search_pitch_min_deg"].as<double>();
        if (rc["search_pitch_max_deg"]) search_pitch_max_ = rc["search_pitch_max_deg"].as<double>();
        if (rc["search_pitch_cycles"])  search_pitch_cycles_ = rc["search_pitch_cycles"].as<double>();

        try {
          rune_detector_ = std::make_unique<RuneDetector>(cfg_);
          rune_campus_ = std::make_unique<RuneCampus>();
          rune_enabled_  = true;
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(),
                      "打符模型加载失败（%s）—— kRune 模式禁用，车自瞄不受影响。"
                      "检查 config 的 rune.model_path", e.what());
          rune_enabled_ = false;
        }
      } else {
        RCLCPP_WARN(get_logger(), "config 里没有 rune: 段 —— kRune 模式禁用");
      }
    }

    rune_counter_ = RuneActivationCounter(rune_active_s_);
    //叫符 request_id 按毫秒时间戳播种，之后逐条 +1（非零且单调）
    rune_req_id_ = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());

    if (rune_enabled_) {
      RCLCPP_INFO(get_logger(),
                  "打符就绪: 算法=CampusRune 模式=%s 单次窗口=%.0fs "
                  "小符最多=%d 次 目标颜色=%s",
                  forced_mode_ == Mode::kRune ? "强制rune" :
                  (forced_mode_ == Mode::kArmor ? "强制armor" : "自动(任务/叫符)"),
                  rune_active_s_, max_small_runes_,
                  rune_my_color_ == 0 ? "红" : "蓝");
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    sub_ = create_subscription<tdt_interface::msg::AutoAimFrame>(
        "/autoaim/frame", qos,
        [this](tdt_interface::msg::AutoAimFrame::SharedPtr m) { on_frame(std::move(m)); });

    pub_ = create_publisher<tdt_interface::msg::SendData>(
        "/target_angles_player_" + std::to_string(player_n), qos);

    //叫符请求出口：到打符站每 request_retry_s 发一条，直到 accepted
    pub_rune_req_ = create_publisher<tdt_interface::msg::RuneRequest>(
        "/game/rune/request", qos);

    //BEST_EFFORT 可匹配任意发布端；丢消息不影响安全
    sub_rune_ = create_subscription<tdt_interface::msg::RuneResult>(
        "/game/rune/result", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        [this](tdt_interface::msg::RuneResult::SharedPtr m) {
          on_rune_result(std::move(m));
        });

    //控制回调组（底盘/任务）；当前单线程执行器，成员仍按并发加锁/atomic
    ctrl_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions ctrl_opts;
    ctrl_opts.callback_group = ctrl_cb_group_;

    //外部云台提示（任务节点也可发；内建任务直接写成员）
    sub_mission_ = create_subscription<std_msgs::msg::String>(
        "/mission/cmd", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        [this](std_msgs::msg::String::SharedPtr m) { on_mission(std::move(m)); }, ctrl_opts);

    //全自动任务（底盘+站点推进，默认关）
    mission_on_   = declare_parameter("mission", false);
    mission_vmax_ = declare_parameter("mission_vmax", 6.0);
    if (mission_on_) {
      const std::string grid_path = declare_parameter(
          "mission_grid", std::string("/home/king/tdt-learning4.5/nav_data/nav_grid.bin"));
      const std::string stages_path = declare_parameter(
          "mission_stages", std::string("/home/king/tdt-learning4.5/nav_data/mission_stages.json"));
      learned_path_ = declare_parameter(
          "mission_learned", std::string("/home/king/tdt-learning4.5/nav_data/learned_blocks.json"));
      if (!grid_.load(grid_path)) {
        RCLCPP_ERROR(get_logger(), "栅格读不了（%s）—— 全自动任务关闭", grid_path.c_str());
        mission_on_ = false;
      } else if (!load_stages(stages_path)) {
        RCLCPP_ERROR(get_logger(), "站点表读不了（%s）—— 全自动任务关闭", stages_path.c_str());
        mission_on_ = false;
      } else {
        load_learned();
        speed_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
            "/target_speed_player_" + std::to_string(player_n), rclcpp::QoS(10).reliable());
        sub_enemies_ = create_subscription<std_msgs::msg::String>(
            "/navigation/enemies", rclcpp::QoS(10).best_effort(),
            [this](std_msgs::msg::String::SharedPtr m) { on_enemies(std::move(m)); }, ctrl_opts);
        enter_stage(0);
      }
    }

    last_rx_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(50ms, [this] {
      watchdog();
      if (mission_on_) mission_tick(); //全自动任务：底盘+站点推进
    }, ctrl_cb_group_);

    RCLCPP_INFO(get_logger(), "auto_aim_node 启动: player_n=%d 敌方=%s 发指令=%s",
                player_n, enemy_color.c_str(),
                publish_angles_ ? "是" : "否(只观察)");
    RCLCPP_INFO(get_logger(),
                "开火决策(照 sp_vision_25): 自动开火=%s 近距容差=%.1f° 远距容差=%.1f° 分档=%.1fm",
                auto_fire_ ? "是" : "否", first_tol_, second_tol_, judge_dist_);
    RCLCPP_INFO(get_logger(), "状态机: 搜索转速=%.0f°/s 暂丢容忍=%d 帧",
                search_rate_, max_temp_lost_);
    RCLCPP_INFO(get_logger(), "开火限制: 最大距离=%.1fm 弹量门槛=%d 发（锁定/跟踪不限距离）",
                max_fire_distance_, min_ammo_to_fire_);
  }

private:
  //收帧：先校验，再解码处理（单线程直接算）
  void on_frame(tdt_interface::msg::AutoAimFrame::SharedPtr msg) {
    const int64_t stamp = stamp_ns(msg->header);
    const int64_t now = static_cast<int64_t>(get_clock()->now().nanoseconds());

    //raw 与 compressed 必须二选一；同有或同空都撤许可丢弃
    const bool raw = !msg->image.data.empty();
    const bool jpg = !msg->compressed_image.data.empty();
    if (raw == jpg) { withdraw_fire(); return; }

    //规则 7.2：state/camera_info/image 的 header 必须与外层同戳，否则整帧不用
    const int64_t h_img = raw ? stamp_ns(msg->image.header)
                              : stamp_ns(msg->compressed_image.header);
    if (stamp_ns(msg->state.header) != stamp ||
        stamp_ns(msg->camera_info.header) != stamp || h_img != stamp) {
      withdraw_fire();
      return;
    }

    //帧龄 >250ms（积压）或超前 >20ms（时钟不同步）都丢
    if (now - stamp > kMaxFrameAgeNs || now - stamp < -kMaxFutureNs) {
      withdraw_fire();
      return;
    }

    //采集帧率只统计通过校验的帧，避免垃圾帧掩盖丢帧
    rx_times_.push_back(std::chrono::steady_clock::now());
    if (rx_times_.size() > kFpsWindow) rx_times_.pop_front();

    const auto cb_t0 = std::chrono::steady_clock::now();
    decode_and_aim(*msg, raw);

    //处理成功后才更新 last_rx_（看门狗判的是"多久没成功处理"）
    last_rx_ = std::chrono::steady_clock::now();

    //单帧 >200ms 告警（每秒最多一条），用于发现解算被巨值带飞
    const double cb_ms =
        std::chrono::duration<double, std::milli>(last_rx_ - cb_t0).count();
    if (cb_ms > 200.0 &&
        std::chrono::duration<double>(last_rx_ - last_slow_warn_).count() > 1.0) {
      last_slow_warn_ = last_rx_;
      RCLCPP_WARN(get_logger(),
                  "单帧处理 %.0f ms（正常 5~10 ms）| 状态=%s 锁定=%s —— 疑似解算被带飞",
                  cb_ms, state_name(state_),
                  tracker_->has_lock() ? ARMOR_NAMES[tracker_->locked_id()].c_str() : "无");
    }
  }

  void decode_and_aim(const tdt_interface::msg::AutoAimFrame & msg, bool raw) {
    const auto t0 = std::chrono::steady_clock::now();

    cv::Mat frame;
    if (raw) {
      //原始图只接受 rgb8；零拷贝包装 ROS buffer
      const auto & im = msg.image;
      if (im.encoding != "rgb8" || im.width == 0 || im.height == 0) return;
      cv::Mat rgb(static_cast<int>(im.height), static_cast<int>(im.width), CV_8UC3,
                  const_cast<uint8_t *>(im.data.data()), static_cast<size_t>(im.step));
      //转 BGR：检测和画图整条链路都按 BGR，漏转会红蓝互换
      cv::cvtColor(rgb, frame, cv::COLOR_RGB2BGR);
    } else {
      //压缩图整块交给 imdecode
      cv::Mat buf(1, static_cast<int>(msg.compressed_image.data.size()), CV_8UC1,
                  const_cast<uint8_t *>(msg.compressed_image.data.data()));
      frame = cv::imdecode(buf, cv::IMREAD_COLOR);
      if (frame.empty()) return; //坏 JPEG 丢弃
    }

    //frame_sequence 回退=换局，重建 Tracker 并清锁定/Aimer/打符状态
    const uint64_t seq = msg.state.frame_sequence;
    if (seq == 0) return;
    if (seq <= last_seq_) {
      const uint64_t prev = last_seq_;
      tracker_ = std::make_unique<Tracker>(cfg_, *solver_);
      aimer_->reset_lock();
      last_locked_ = -1;
      last_seq_ = seq;
      //换局也清打符计数；内部按时间窗自建，只清节点标记
      rune_counter_.reset();
      rune_count_at_stage_entry_ = 0; //计数与进站快照一起清零
      rune_started_ = false;
      rune_finished_ = false;
      RCLCPP_WARN(get_logger(), "frame_sequence 回退(%lu → %lu)，按换局处理，状态已清",
                  static_cast<unsigned long>(prev),
                  static_cast<unsigned long>(seq));
    } else {
      last_seq_ = seq;
    }

    //内参必须在检测/解算前更新，PnP 用的就是刚设的这份
    apply_intrinsics(msg.camera_info, frame.size());

    const double yaw_now = msg.state.yaw_degrees;
    const double pitch_now = msg.state.pitch_degrees;

    //底盘速度（Unity 单位/s，3 单位=1m），打车开火门用
    chassis_speed_ = std::hypot(msg.state.twist.linear.x, msg.state.twist.linear.y);

    //底盘最新位姿/速度缓存（map FLU → Unity (x,z)）
    if (mission_on_) {
      st_ux_ = -msg.state.position.y;
      st_uz_ = msg.state.position.x;
      st_uvx_ = -msg.state.twist.linear.y;
      st_uvz_ = msg.state.twist.linear.x;
      st_stamp_ = std::chrono::steady_clock::now();
      st_seen_ = true;
    }

    if (should_run_rune()) {
      rune_aim_step(msg, frame, yaw_now, pitch_now);
      return;
    }
    if (rune_started_) { //窗口结束，下次开符重新初始化
      rune_started_ = false;
      RCLCPP_INFO(get_logger(), "打符窗口结束 -> 回到车自瞄");
    }

    //检测→PnP（都在相机系）
    auto armors = detector_->detect(frame);
    armors.remove_if([&](const Armor & a) { return a.color != enemy_; });
    //任务期非当前站目标的板不进跟踪器；mask=0 的站全部丢
    if (mission_on_) {
      armors.remove_if([&](const Armor & a) {
        return mission_filter_mask_ == 0 ||
               ((1u << static_cast<int>(a.name)) & mission_filter_mask_) == 0;
      });
    }
    for (auto & a : armors) solver_->solve(a);

    //观测转世界系后再进 EKF，rvec/tvec/points 仍留相机系
    const Eigen::Matrix3d R_world_from_cam = world_from_cam(yaw_now, pitch_now);
    armors_to_world(armors, R_world_from_cam);
    solver_->set_frame_rotation(R_world_from_cam.transpose()); //投影像素时转回相机系

    tracker_->update(armors, std::chrono::steady_clock::now());

    //处理耗时到 EKF 更新为止（模型+PnP+EKF），不含可视化
    const auto t_ekf = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(t_ekf - t0).count();
    ema_ms_ = ema_ms_ * 0.9 + ms * 0.1;
    proc_times_.push_back(t_ekf);
    if (proc_times_.size() > kFpsWindow) proc_times_.pop_front();

    //状态机迁移
    const bool seen = !armors.empty(); //本帧有检出敌方板
    const bool locked = tracker_->has_lock();
    const int locked_now = locked ? tracker_->locked_id() : -1;

    //锁定目标变化：清 Aimer 迟滞和换板门，前哨站收敛计时重起
    if (locked_now != last_locked_) {
      aimer_->reset_lock();
      outpost_lock_t0_ = (locked_now == (int)outpost) ? steady_seconds() : -1e9;
      last_aim_plate_id_ = -1;
      plate_switch_t_ = -1e9;
      RCLCPP_INFO(get_logger(), "锁定切换: %s → %s",
                  last_locked_ < 0 ? "-" : ARMOR_NAMES[last_locked_].c_str(),
                  locked_now < 0 ? "-" : ARMOR_NAMES[locked_now].c_str());
    }
    last_locked_ = locked_now;

    //前哨站旋向确定时打印一次票数
    if (locked && tracker_->tracks().count(locked_now)) {
      const Target & tgt = tracker_->tracks().at(locked_now);
      if (tgt.is_outpost_ && tgt.outpost_dir_ != last_outpost_dir_) {
        if (tgt.outpost_dir_ != 0)
          RCLCPP_INFO(get_logger(), "前哨站旋转方向: %s (投票 ccw=%d cw=%d, %d 个样本)",
                      tgt.outpost_dir_ > 0 ? "逆时针" : "顺时针",
                      tgt.dir_ccw_, tgt.dir_cw_, kOutpostDirVotes);
        last_outpost_dir_ = tgt.outpost_dir_;
      }
    }

    //进 kTracking 需连续 min_detect_count_ 帧看到且已锁定；弹空进 kNoAmmo，开局空弹只警告一次
    const bool ammo_empty = msg.state.ammo_remaining <= min_ammo_to_fire_;
    if (ammo_empty && !warned_ammo_ && processed_ < 10) {
      RCLCPP_WARN(get_logger(),
                  "ammo_remaining=%d 判为空弹 → 进入无弹状态。若游戏侧本就填 0，"
                  "请把 config 的 min_ammo_to_fire 设为负数关掉这个门",
                  msg.state.ammo_remaining);
      warned_ammo_ = true;
    }

    const AimState prev_state = state_;

    //状态机类型转换条件:弹空优先
    if (ammo_empty && state_ != AimState::kNoAmmo) {
      state_ = AimState::kNoAmmo;
      lost_ = 0;
      detect_count_ = 0;
    } else switch (state_) {
      case AimState::kSearch: //搜索
        if (seen)       { state_ = AimState::kDetecting; detect_count_ = 1; lost_ = 0; }
        break;

      case AimState::kDetecting: //确认
        if (seen) {
          lost_ = 0;
          if (++detect_count_ >= min_detect_count_ && locked) {
            state_ = AimState::kTracking;
            detect_count_ = 0;
          }
        } else if (++lost_ >= kDetectGiveUp) {
          state_ = AimState::kSearch;
          lost_ = 0;
        }
        break;

      case AimState::kTracking: //跟踪
        if (!locked)    { state_ = AimState::kTempLost;  lost_ = 0; }
        break;

      case AimState::kTempLost: //暂丢
        if (locked)     { state_ = AimState::kTracking;  lost_ = 0; }
        else if (++lost_ >= max_temp_lost_) { state_ = AimState::kSearch; lost_ = 0; }
        break;

      case AimState::kNoAmmo: //无弹
        if (!ammo_empty) {
          //弹药恢复：还锁着回跟踪，否则回搜索
          state_ = locked ? AimState::kTracking : AimState::kSearch;
          lost_ = 0;
        }
        break;
    }
    //状态变化打一条日志
    if (state_ != prev_state)
      RCLCPP_INFO(get_logger(), "状态: %s → %s", state_name(prev_state), state_name(state_));

    //初速为最近成功一发的实际值（可能为 0），20~30 m/s 之外不更新
    const double v0 = msg.state.projectile_speed;
    if (v0 > 20.0 && v0 < 30.0) aimer_->set_bullet_speed(v0);

    //首帧把上一条指令对齐到云台实际角，避免一上来甩云台
    if (!started_) {
      last_yaw_ = yaw_now;
      last_pitch_ = pitch_now;
      started_ = true;
    }

    //默认保持上一条指令（kDetecting/kTempLost/kNoAmmo 不改 world_*）；游戏 0.5s 无指令会清状态
    double world_yaw = last_yaw_, world_pitch = last_pitch_;
    bool shoot = false;
    Aimer::AimResult aim;

    //距上一帧时间，搜索积分用；限幅 0.5 s 防卡顿后一次转一大截
    const auto now_tp = std::chrono::steady_clock::now();
    const double dt = std::clamp(
        std::chrono::duration<double>(now_tp - last_frame_t_).count(), 0.0, 0.5);
    last_frame_t_ = now_tp;

    aiming_base_ = false; //本帧是否基地直瞄（日志）
    last_fire_block_[0] = '\0'; //开火否决原因（日志）

    //状态机具体实现
    if (state_ == AimState::kSearch) {
      //任务提示优先按住提示角，超时再搜索
      double my = 0.0, mp = 0.0;
      if (mission_hold(my, mp)) {
        world_yaw = my;
        world_pitch = mp;
      } else {
        //从上次发出的角继续转（不用实际角，避免未到位时跳变）
        search_step(dt, world_yaw, world_pitch);
      }
    } else if (state_ == AimState::kTracking &&
               tracker_->locked_id() == static_cast<int>(base)) {
      //基地是静止目标，直接拿本帧板中心直瞄解弹道，不走 EKF 预测
      const Armor * plate = pick_base_plate(armors, frame.size());
      aiming_base_ = (plate != nullptr);
      if (plate) {
        //换系：xyz_in_camera 是相机排版(右/下/前)，best_aim_direct 要状态系(右/前/上)：pseudo=(x,z,-y)
        const Eigen::Vector3d & pc = plate->xyz_in_camera;
        const Eigen::Vector3d p_state(pc(0), pc(2), -pc(1));

        aim = aimer_->best_aim_direct(p_state, *solver_);
        if (aim.valid) {
          world_yaw = rad2deg(std::atan2(aim.aim_3d(0), aim.aim_3d(1)));
          world_pitch = -rad2deg(aim.pitch);
          shoot = decide_fire(world_yaw, yaw_now,
                              std::hypot(p_state(0), p_state(1)), aim.converged);
        }
      }
    } else if (state_ == AimState::kTracking) {
      auto it = tracker_->tracks().find(tracker_->locked_id());
      if (it != tracker_->tracks().end() && !it->second.diverged()) {
        const cv::Point2f center(frame.cols / 2.0f, frame.rows / 2.0f);
        aim = aimer_->best_aim(it->second, center, *solver_);
        if (aim.valid) {
          //车换板时记时刻，decide_fire 用它挡换板瞬态；前哨站选板无迟滞，不适用
          if (!it->second.is_outpost_ &&
              last_aim_plate_id_ >= 0 && aim.plate_id != last_aim_plate_id_)
            plate_switch_t_ = steady_seconds();
          last_aim_plate_id_ = aim.plate_id;

          //aim_3d 是世界系 (右,前,上)，yaw=atan2(右,前)；pitch 向上为正，游戏向下为正故取负
          world_yaw = rad2deg(std::atan2(aim.aim_3d(0), aim.aim_3d(1)));
          world_pitch = -rad2deg(aim.pitch);

          //前哨站锁后 outpost_settle_s 内不开火（dz1/dz2 还在收敛），只挡开火不挡瞄准
          const bool outpost_settling =
              it->second.is_outpost_ &&
              (steady_seconds() - outpost_lock_t0_) < outpost_settle_s_;
          if (outpost_settling) {
            shoot = false;
            std::snprintf(last_fire_block_, sizeof(last_fire_block_),
                          "前哨站收敛中(%.1f/%.0fs)",
                          steady_seconds() - outpost_lock_t0_, outpost_settle_s_);
          } else {
            //末参为弹道迭代是否收敛（不收敛不开火）
            const Eigen::VectorXd & x = it->second.ekf_.x; //状态：右/前/上
            shoot = decide_fire(world_yaw, yaw_now,
                                std::sqrt(x(0) * x(0) + x(2) * x(2)), aim.converged);
          }
        }
      }
    }

    //任务期只打当前站目标，否则不开火并把云台转回提示角
    if (mission_on_) {
      const bool is_target = tracker_->has_lock() && mission_filter_mask_ != 0 &&
                             ((1u << tracker_->locked_id()) & mission_filter_mask_) != 0;
      if (!is_target) {
        shoot = false;
        double my = 0.0, mp = 0.0;
        if (mission_hold(my, mp)) { //提示新鲜才夺云台
          world_yaw = my;
          world_pitch = mp;
        }
      }
    }

    send_angles(world_yaw, world_pitch, shoot);

    if (show_window_) {
      visualizer_->draw(frame, armors, *tracker_, frame_id_,
                        aim.valid ? &aim : nullptr,
                        !aiming_base_); //基地直瞄时不画 EKF 预测
      cv::imshow("auto_aim", frame);
      if ((char)cv::waitKey(1) == 27) rclcpp::shutdown();
    }
    ++frame_id_;

    //每 30 帧一条：处理/采集帧率、锁定目标、观测列表；采集>处理说明在丢帧、提前量会偏
    if (++processed_ % 30 == 0) {
      std::string obs;
      for (const auto & a : armors) {
        char b[80];
        std::snprintf(b, sizeof(b), "%s/%s/%.2f/%.1fm/%s ", ARMOR_NAMES[a.name].c_str(),
                      COLORS[a.color].c_str(), a.confidence,
                      a.xyz_in_camera.norm(), ARMOR_TYPES[a.type].c_str());
        obs += b;
      }

      std::string lock = "-";
      if (aiming_base_) {
        lock = "base(直瞄)"; //基地不走跟踪器，锁的是本帧看到的板
      } else if (tracker_->has_lock()) {
        auto it = tracker_->tracks().find(tracker_->locked_id());
        if (it != tracker_->tracks().end()) {
          const Eigen::VectorXd & x = it->second.ekf_.x;
          const double d = std::sqrt(x(0) * x(0) + x(2) * x(2) + x(4) * x(4));
          char b[128];
          //车看 r/dr；前哨站看 dz1/dz2 和旋向（0 未定/+1 逆/-1 顺）
          if (it->second.is_outpost_) {
            char dir[32];
            if (it->second.outpost_dir_ == 0)
              std::snprintf(dir, sizeof(dir), "未定(%d/%d)",
                            it->second.dir_votes(), kOutpostDirVotes);
            else
              std::snprintf(dir, sizeof(dir), "%s",
                            it->second.outpost_dir_ > 0 ? "逆时针" : "顺时针");
            std::snprintf(b, sizeof(b), "%s %.2fm v_yaw%+.2frad/s 转向=%s dz=%.3f/%.3f%s",
                          ARMOR_NAMES[tracker_->locked_id()].c_str(), d, x(7),
                          dir, x(8), x(9),
                          it->second.diverged() ? " 发散!" : "");
          }
          else
            std::snprintf(b, sizeof(b), "%s %.2fm ω%+.0f°/s r=%.3f dr=%+.3f%s",
                          ARMOR_NAMES[tracker_->locked_id()].c_str(), d,
                          rad2deg(x(7)), x(8), x(9),
                          it->second.diverged() ? " 发散!" : "");
          lock = b;
        }
      }

      //开火栏带否决原因（超距/收敛中/指令突变/云台未到位）
      char fire_txt[96];
      if (fire_sent_)               std::snprintf(fire_txt, sizeof(fire_txt), "是");
      else if (last_fire_block_[0]) std::snprintf(fire_txt, sizeof(fire_txt), "否(%s)", last_fire_block_);
      else                          std::snprintf(fire_txt, sizeof(fire_txt), "否");

      RCLCPP_INFO(get_logger(),
                  "处理帧率 %.1f fps（模型+PnP+EKF %.1f ms/帧）| 采集 %.1f fps | 状态=%s "
                  "| 锁定=%s | 开火=%s | 发散重建=%d | 观测=[%s]",
                  rate_fps(proc_times_), ema_ms_, rate_fps(rx_times_),
                  state_name(state_), lock.c_str(), fire_txt,
                  tracker_->diverged_resets(), obs.empty() ? "无" : obs.c_str());
    }
  }

  //开火判据：指令不突变、云台对上一指令到位、距离/收敛/换板门；只判 yaw。容差近 3°、远 2°
  bool decide_fire(double cmd_yaw, double gimbal_yaw, double horizon_dist, bool aim_converged) {
    last_fire_block_[0] = '\0';
    if (!auto_fire_) { std::snprintf(last_fire_block_, sizeof(last_fire_block_), "未开自动开火"); return false; }

    //底盘未停稳不开火（Unity 单位/s，<=0 关闭）；只挡开火，云台照常跟
    if (fire_max_chassis_speed_ > 0.0 && chassis_speed_ > fire_max_chassis_speed_) {
      std::snprintf(last_fire_block_, sizeof(last_fire_block_), "车在动(%.1f等停稳)",
                    chassis_speed_);
      return false;
    }

    const double tol = (horizon_dist > judge_dist_) ? second_tol_ : first_tol_;

    const bool no_jump   = have_prev_cmd_ && std::abs(prev_cmd_yaw_ - cmd_yaw) < tol * 2.0;
    const bool on_target = have_prev_cmd_ && std::abs(gimbal_yaw - prev_cmd_yaw_) < tol;

    //prev_cmd_yaw_ 在距离门前更新：超距期间指令仍在跟目标，也要记录
    prev_cmd_yaw_ = cmd_yaw; //无论开不开火都更新
    have_prev_cmd_ = true;

    //距离门：max_fire_distance_ <=0 不限
    if (max_fire_distance_ > 0.0 && horizon_dist > max_fire_distance_) {
      std::snprintf(last_fire_block_, sizeof(last_fire_block_), "超距");
      return false;
    }

    //弹道迭代未收敛不给许可（aim 返回最后迭代值，瞄准照发）
    if (!aim_converged) {
      std::snprintf(last_fire_block_, sizeof(last_fire_block_), "收敛中");
      return false;
    }

    //换板后等待，只挡开火不挡瞄准
    if (plate_switch_settle_s_ > 0.0 &&
        steady_seconds() - plate_switch_t_ < plate_switch_settle_s_) {
      std::snprintf(last_fire_block_, sizeof(last_fire_block_),
                    "换板等待(%.0fms)",
                    (steady_seconds() - plate_switch_t_) * 1000.0);
      return false;
    }

    if (!no_jump) {
      std::snprintf(last_fire_block_, sizeof(last_fire_block_), "指令突变");
      return false;
    }
    if (!on_target) {
      std::snprintf(last_fire_block_, sizeof(last_fire_block_), "云台未到位");
      return false;
    }
    return true;
  }

  //规则 7.1：同帧 camera_info 有效就用，否则回落表 12 标称值；fx>0 判有效，0 会让 PnP 除零
  void apply_intrinsics(const sensor_msgs::msg::CameraInfo & ci, const cv::Size & sz) {
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat D = cv::Mat::zeros(1, 5, CV_64F); //接口是无畸变针孔模型

    if (ci.width > 0 && ci.height > 0 && ci.k[0] > 0.0) {
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) K.at<double>(r, c) = ci.k[r * 3 + c];
      if (!intr_ready_)
        RCLCPP_INFO(get_logger(), "内参来自 camera_info: fx=%.3f fy=%.3f cx=%.3f cy=%.3f (%ux%u)",
                    ci.k[0], ci.k[4], ci.k[2], ci.k[5], ci.width, ci.height);
      intr_ready_ = true;
    } else {
      //回落值按 960×720 给，尺寸不同按比例缩放
      const double sx = sz.width > 0 ? sz.width / 960.0 : 1.0;
      const double sy = sz.height > 0 ? sz.height / 720.0 : 1.0;
      K.at<double>(0, 0) = kFallbackFx * sx;
      K.at<double>(1, 1) = kFallbackFy * sy; 
      K.at<double>(0, 2) = kFallbackCx * sx;
      K.at<double>(1, 2) = kFallbackCy * sy;
      if (!warned_fb_) {
        RCLCPP_WARN(get_logger(),
                    "camera_info 无效 → 回落表 12 标称值(按 %dx%d 缩放)", sz.width, sz.height);
        warned_fb_ = true;
      }
    }
    solver_->set_intrinsics(K, D);
    //打符用同一份内参（规则 7.1 要求同帧），每帧传进 process()
    intr_K_ = K;
    intr_D_ = D;
  }

  //打符：检测 → 精修/位姿/相位/弹道 → 瞄准/开火；复用 kSearch/kTracking/kTempLost 状态
  static double steady_seconds() {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  //该不该打符：任务要求优先，其次 config 强制，最后叫符窗口；activated 立即退出
  bool should_run_rune() const {
    if (!rune_enabled_) return false;
    if (mission_rune_) return true; //任务要求打符
    if (mode_forced_) return forced_mode_ == Mode::kRune;
    if (rune_finished_) return false;
    return rune_counter_.active(steady_seconds());
  }

  //accepted=受理（开符），activated=本轮打完，收到立即回车自瞄
  void on_rune_result(tdt_interface::msg::RuneResult::SharedPtr msg) {
    if (msg->activated) {
      //本站必须叫过符（accepted 在前）才认 activated，防上一轮迟到记到下一站
      const bool engaged = rune_req_ok_.load() ||
                           rune_counter_.count() > rune_count_at_stage_entry_;
      if (!engaged) {
        RCLCPP_WARN(get_logger(),
                    "收到 activated=true 但本站还没叫过符（request_id=%lu）→ 忽略"
                    "（上一轮的迟到/重发？）",
                    static_cast<unsigned long>(msg->request_id));
        return;
      }
      mission_rune_done_ = true; //任务判据：符站可过
      RCLCPP_INFO(get_logger(),
                  "★ 增益已激活（request_id=%lu）—— 这轮符打完了，立即切回车自瞄",
                  static_cast<unsigned long>(msg->request_id));
      rune_finished_ = true;
      //内部按时间窗重建，不用收尾
    } else if (msg->accepted) {
      //accepted 必须是当前请求；开符计数 +1（第 3 次起大符，也作"本站叫过符"判据）
      if (msg->request_id == rune_req_last_accepted_id_) return; //同一 id 重发防重复计数
      if (msg->request_id != rune_req_last_id_.load()) {
        RCLCPP_WARN(get_logger(),
                    "收到 accepted=true 但不是当前请求（id=%lu，当前 %lu）→ 忽略",
                    static_cast<unsigned long>(msg->request_id),
                    static_cast<unsigned long>(rune_req_last_id_.load()));
        return;
      }
      rune_req_last_accepted_id_ = msg->request_id;
      rune_req_ok_.store(true); //叫符循环停止
      rune_finished_ = false; //清上一轮 activated 标记
      const int c = rune_counter_.mark_opened(steady_seconds());
      RCLCPP_INFO(get_logger(),
                  "★ 叫符成功（request_id=%lu）—— 进入打符模式（本局第 %d 次开符）",
                  static_cast<unsigned long>(msg->request_id), c);
    } else {
      //被拒：按重试间隔再发；日志限流 5s 一条
      const double now_s = steady_seconds();
      if (now_s - rune_req_reject_log_t_ > 5.0) {
        rune_req_reject_log_t_ = now_s;
        RCLCPP_WARN(get_logger(),
                    "叫符被拒（request_id=%lu）—— 每 %.1fs 自动重试",
                    static_cast<unsigned long>(msg->request_id),
                    rune_req_retry_ms_ / 1000.0);
      }
    }
  }

  //解析 /mission/cmd："hold yaw pitch [rune] [big]" 按住提示角 2s（超时失效），"auto" 交回搜索；big 隐含 rune
  void on_mission(std_msgs::msg::String::SharedPtr msg) {
    std::istringstream ss(msg->data);
    std::string tok;
    ss >> tok;
    if (tok == "hold") {
      double y = 0.0, p = 0.0;
      if (!(ss >> y >> p)) return;
      std::lock_guard<std::mutex> lk(mission_mu_);
      mission_yaw_ = y;
      mission_pitch_ = p;
      mission_stamp_ = steady_seconds();
      mission_rune_ = false;
      //尾标记 rune/big 顺序随意，big 隐含 rune；每次 hold 重置
      bool big = false;
      std::string extra;
      while (ss >> extra) {
        if (extra == "rune") mission_rune_ = true;
        else if (extra == "big") { big = true; mission_rune_ = true; }
      }
      mission_rune_big_ = big;
    } else {
      std::lock_guard<std::mutex> lk(mission_mu_);
      mission_stamp_ = -1e9;
      mission_rune_ = false;
      mission_rune_big_ = false;
    }
  }

  bool mission_hold(double & yaw, double & pitch) const {
    std::lock_guard<std::mutex> lk(mission_mu_);
    if (steady_seconds() - mission_stamp_ > 2.0) return false;
    yaw = mission_yaw_;
    pitch = mission_pitch_;
    return true;
  }

  //全自动任务：mission:=true 启用；底盘速度发 /target_speed_player_N（Unity 单位/s，3 单位=1m）
  bool load_stages(const std::string & path) {
    try {
      YAML::Node d = YAML::LoadFile(path);
      if (d["rune_preset"]) {
        rune_yaw_ = d["rune_preset"]["yaw"].as<double>();
        rune_pitch_ = d["rune_preset"]["pitch"].as<double>();
      }
      stages_.clear();
      for (const auto & s : d["stages"]) {
        Stage st;
        st.name = s["name"].as<std::string>();
        st.gx = s["goal"][0].as<double>();
        st.gz = s["goal"][1].as<double>();
        st.done = s["done"].as<std::string>();
        if (s["number"]) st.number = s["number"].as<int>();
        if (s["wait"]) st.wait = s["wait"].as<double>(); //到点后停留 N 秒（补弹等）
        if (s["stop"]) st.stop = s["stop"].as<double>(); //距目标多远停车打（0=默认 4.5）
        if (s["fixed"]) st.fixed = s["fixed"].as<bool>(); //定点打：不跟实时位置
        if (s["aim"] && !s["aim"].IsNull()) {
          st.aim_type = s["aim"]["type"].as<std::string>();
          if (s["aim"]["number"]) st.aim_number = s["aim"]["number"].as<int>();
          if (s["aim"]["pitch"]) st.aim_pitch = s["aim"]["pitch"].as<double>();
        }
        stages_.push_back(st);
      }
      return !stages_.empty();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "站点表解析失败：%s", e.what());
      return false;
    }
  }

  void load_learned() {
    try {
      YAML::Node d = YAML::LoadFile(learned_path_);
      for (const auto & p : d)
        if (p.IsSequence() && p.size() == 2) {
          learned_.push_back({p[0].as<double>(), p[1].as<double>()});
          grid_.block_around(learned_.back().first, learned_.back().second, 1.6);
        }
      if (!learned_.empty())
        RCLCPP_INFO(get_logger(), "载入 %zu 个学到的障碍点", learned_.size());
    } catch (const std::exception &) {
      //文件不存在=还没学过
    }
  }

  //保留落盘函数但当前不调用（学障碍只进内存）
  void save_learned() const {
    FILE * f = std::fopen(learned_path_.c_str(), "w");
    if (!f) return;
    std::fprintf(f, "[\n");
    for (size_t i = 0; i < learned_.size(); ++i)
      std::fprintf(f, " [%.6f, %.6f]%s\n", learned_[i].first, learned_[i].second,
                   i + 1 < learned_.size() ? "," : "");
    std::fprintf(f, "]\n");
    std::fclose(f);
  }

  void on_enemies(std_msgs::msg::String::SharedPtr msg) {
    try {
      YAML::Node d = YAML::Load(msg->data);
      std::unordered_map<int, int> hp;
      std::unordered_map<int, std::pair<double, double>> pos;
      for (const auto & e : d["enemies"]) {
        const int n = e["number"].as<int>();
        hp[n] = e["health"].as<int>();
        pos[n] = {e["x"].as<double>(), e["z"].as<double>()};
        enemy_seen_.insert(n);
      }
      enemy_hp_ = std::move(hp);
      enemy_pos_ = std::move(pos);
    } catch (const std::exception &) {
      //解析失败忽略这一帧
    }
  }

  void enter_stage(int k) {
    stage_i_ = k;
    const Stage & st = stages_[k];
    goal_ = {st.gx, st.gz};
    path_valid_ = false;
    path_i_ = 0;
    pause_until_ = {};
    ever_moved_ = false;
    stuck_since_ = {};
    waiting_ = false;
    wait_logged_ = false;
    mission_rune_done_ = false;
    dwell_t0_ = {}; //停留计时
    rune_arrive_t0_ = {}; //打符站到点计时
    rune_req_active_ = false; //自动叫符循环
    rune_req_ok_ = false;
    rune_req_last_id_ = 0;
    live_probe_done_ = false; //实时目标可达性缓存（每站重探）
    standoff_reachable_ = false;
    plan_fail_t0_ = {}; //规划失败重试计时
    last_plan_warn_s_ = 0.0;
    rune_count_at_stage_entry_ = rune_counter_.count(); //进站时开符计数快照
    //任务期只打当前站目标，mask=0 谁都不打；2/3/4 工程、5 哨兵（含 five）、6 前哨、7 基地
    mission_filter_mask_ = 0;
    if (st.aim_type == "enemy") {
      switch (st.aim_number) {
        case 2: mission_filter_mask_ = 1u << two; break;
        case 3: mission_filter_mask_ = 1u << three; break;
        case 4: mission_filter_mask_ = 1u << four; break;
        case 5: mission_filter_mask_ = (1u << sentry) | (1u << five); break;
        case 6: mission_filter_mask_ = 1u << outpost; break;
        case 7: mission_filter_mask_ = 1u << base; break;
        default: break;
      }
    }
    RCLCPP_INFO(get_logger(), "===== 第 %d/%zu 站：%s → Unity(%+.1f,%+.1f)  完成条件 %s =====",
                k + 1, stages_.size(), st.name.c_str(), st.gx, st.gz, st.done.c_str());
    if (st.aim_type.empty()) {
      std::lock_guard<std::mutex> lk(mission_mu_);
      mission_stamp_ = -1e9; //交回自动搜索
      mission_rune_ = false;
    } else {
      push_aim(true);
    }
  }

  //任务判据：rune 看激活，hp 看血量<=0 或已从敌人列表消失，arrive 到点即完成
  bool check_condition() const {
    const Stage & st = stages_[stage_i_];
    if (st.done == "rune") return mission_rune_done_;
    if (st.done == "hp") {
      auto it = enemy_hp_.find(st.number);
      if (it != enemy_hp_.end() && it->second <= 0) return true;
      return enemy_seen_.count(st.number) > 0 && enemy_hp_.count(st.number) == 0;
    }
    return false; //arrive 到点即推进
  }

  //生成云台提示角：rune 用预设角，enemy 用 atan2(dx,dz)；默认限频 0.2s
  void push_aim(bool force = false) {
    const Stage & st = stages_[stage_i_];
    if (st.aim_type.empty()) return;
    const auto now = std::chrono::steady_clock::now();
    if (!force && std::chrono::duration<double>(now - last_aim_pub_).count() < 0.2) return;
    if (st.aim_type == "rune") {
      std::lock_guard<std::mutex> lk(mission_mu_);
      mission_yaw_ = rune_yaw_;
      mission_pitch_ = rune_pitch_;
      mission_rune_ = true;
      mission_stamp_ = steady_seconds();
    } else if (st.aim_type == "enemy") {
      auto it = enemy_pos_.find(st.aim_number);
      if (it == enemy_pos_.end()) return; //不知道位置，先不夺云台
      std::lock_guard<std::mutex> lk(mission_mu_);
      mission_yaw_ = rad2deg(std::atan2(it->second.first - st_ux_, it->second.second - st_uz_));
      mission_pitch_ = st.aim_pitch;
      mission_rune_ = false;
      mission_stamp_ = steady_seconds();
    }
    last_aim_pub_ = now;
  }

  //推进
  void advance_stage() {
    if (stage_i_ + 1 >= static_cast<int>(stages_.size())) {
      mission_finished_ = true;
      publish_speed(0, 0);
      RCLCPP_INFO(get_logger(), "★ 全任务走完");
      return;
    }
    //先 enter_stage 再设停顿，否则 pause_until_ 被进站初始化清掉
    enter_stage(stage_i_ + 1);
    pause_until_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(800);
  }

  void publish_speed(double vx, double vz) {
    cmd_vx_ = vx;
    cmd_vz_ = vz;
    if (!speed_pub_ || !publish_angles_) return; //观察模式连底盘也不动
    geometry_msgs::msg::TwistStamped m;
    m.header.stamp = now();
    m.twist.linear.x = vx;
    m.twist.linear.y = vz;
    speed_pub_->publish(m);
  }

  //pure pursuit 前瞻点：车投影到路径后沿路径前进 kLookahead（场景单位）
  std::pair<double, double> lookahead_point() {
    if (path_.size() < 2) return path_.back();
    double best_d = 1e18, best_t = 0.0;
    size_t best_seg = path_i_;
    for (size_t k = path_i_; k + 1 < path_.size(); ++k) {
      const double ax = path_[k].first, az = path_[k].second;
      const double ex = path_[k + 1].first - ax, ez = path_[k + 1].second - az;
      const double L2 = ex * ex + ez * ez;
      double t = 0.0;
      if (L2 > 1e-9)
        t = std::clamp(((st_ux_ - ax) * ex + (st_uz_ - az) * ez) / L2, 0.0, 1.0);
      const double d = std::hypot(ax + t * ex - st_ux_, az + t * ez - st_uz_);
      if (d < best_d) { best_d = d; best_seg = k; best_t = t; }
    }
    path_i_ = best_seg;
    const double seg0 = std::hypot(path_[best_seg + 1].first - path_[best_seg].first,
                                   path_[best_seg + 1].second - path_[best_seg].second);
    double acc = (1.0 - best_t) * seg0;
    if (acc >= kLookahead) {
      const double rr = best_t + kLookahead / std::max(seg0, 1e-9);
      return {path_[best_seg].first + rr * (path_[best_seg + 1].first - path_[best_seg].first),
              path_[best_seg].second + rr * (path_[best_seg + 1].second - path_[best_seg].second)};
    }
    for (size_t k = best_seg + 1; k + 1 < path_.size(); ++k) {
      const double seg = std::hypot(path_[k + 1].first - path_[k].first,
                                    path_[k + 1].second - path_[k].second);
      if (acc + seg >= kLookahead) {
        const double rr = (kLookahead - acc) / std::max(seg, 1e-9);
        return {path_[k].first + rr * (path_[k + 1].first - path_[k].first),
                path_[k].second + rr * (path_[k + 1].second - path_[k].second)};
      }
      acc += seg;
    }
    return path_.back();
  }

  void mission_tick() {
    using Clock = std::chrono::steady_clock;
    const auto now_tp = Clock::now();
    if (!st_seen_ ||
        std::chrono::duration<double>(now_tp - st_stamp_).count() > kCommandTimeoutSec) {
      publish_speed(0, 0); //断流：停车等恢复
      return;
    }
    if (mission_finished_) { publish_speed(0, 0); return; }
    if (prewarm_t0_.time_since_epoch().count() == 0) prewarm_t0_ = now_tp;
    if (std::chrono::duration<double>(now_tp - prewarm_t0_).count() < 2.0) {
      publish_speed(0, 0); //DDS 预热 2s，首发指令会被匹配延迟吃掉
      return;
    }
    push_aim();
    service_rune_requests(); //到打符位自动叫符（内部有 active/间隔门）
    if (now_tp < pause_until_) { publish_speed(0, 0); return; }

    if (!path_valid_) {
      auto p = grid_.plan(st_ux_, st_uz_, goal_.first, goal_.second, 0.4);
      if (!p) {
        //规划失败先停车自救（放开周围 1.5）再重试，持续 kPlanGiveUp 秒才放弃
        if (plan_fail_t0_.time_since_epoch().count() == 0) {
          plan_fail_t0_ = now_tp;
          grid_.unblock_around(st_ux_, st_uz_, 1.5);
        }
        publish_speed(0, 0);
        const double fail_s =
            std::chrono::duration<double>(now_tp - plan_fail_t0_).count();
        if (fail_s > kPlanGiveUp) {
          mission_finished_ = true;
          RCLCPP_ERROR(get_logger(), "规划持续失败 %.0f s，放弃任务。车在 (%+.1f,%+.1f)",
                       fail_s, st_ux_, st_uz_);
        } else if (fail_s - last_plan_warn_s_ > 3.0) {
          last_plan_warn_s_ = fail_s;
          RCLCPP_WARN(get_logger(),
                      "规划失败：没有通路（%+.1f,%+.1f → %+.1f,%+.1f），停车重试中…",
                      st_ux_, st_uz_, goal_.first, goal_.second);
        }
        return;
      }
      plan_fail_t0_ = {};
      last_plan_warn_s_ = 0.0;
      path_ = std::move(*p);
      path_i_ = 0;
      path_valid_ = true;
      double L = 0;
      for (size_t k = 1; k < path_.size(); ++k)
        L += std::hypot(path_[k].first - path_[k - 1].first,
                        path_[k].second - path_[k - 1].second);
      RCLCPP_INFO(get_logger(), "已规划：%zu 个点、%.1f m  从 (%+.1f,%+.1f) 到 (%+.1f,%+.1f)",
                  path_.size(), L / 3.0, st_ux_, st_uz_, goal_.first, goal_.second);
    }

    const Stage & st = stages_[stage_i_];

    //hp 站实时规划：目标移动 >kReplanDist 重规划；进 stop_d 停车打，跑出 stop_d+1.5 再追
    if (st.done == "hp" && st.fixed) {
      //定点打：只开到配置点停住打，不跟实时位置；路上目标被打掉则直接推进
      if (check_condition()) {
        RCLCPP_INFO(get_logger(), "★ %s 完成（%s，目标已在路上被解决）→ 下一站",
                    st.name.c_str(), st.done.c_str());
        advance_stage();
        return;
      }
    } else if (st.done == "hp") {
      auto it = enemy_pos_.find(st.number);
      if (it != enemy_pos_.end()) {
        const auto & live = it->second;
        //每站可覆盖停车距离（st.stop）；迟滞 1.5 单位
        const double stop_d = st.stop > 0.0 ? st.stop : kStopDist;
        if (std::hypot(live.first - goal_.first, live.second - goal_.second) > kReplanDist) {
          //目标本体可能不可驶入（基地/前哨在围墙内），改用朝车一侧退 stop_d 的射击位；同位置只探一次
          if (!live_probe_done_ ||
              std::hypot(live.first - live_probe_.first,
                         live.second - live_probe_.second) > 1.0) {
            live_probe_ = live;
            live_probe_done_ = true;
            live_reachable_ =
                grid_.plan(st_ux_, st_uz_, live.first, live.second, 0.4).has_value();
            standoff_reachable_ = false;
            if (!live_reachable_) {
              double dx = st_ux_ - live.first, dz = st_uz_ - live.second;
              const double n = std::hypot(dx, dz);
              if (n > 1e-6) { dx /= n; dz /= n; } else { dx = 0.0; dz = 1.0; } //重合兜底
              standoff_ = {live.first + dx * stop_d, live.second + dz * stop_d};
              standoff_reachable_ = grid_
                  .plan(st_ux_, st_uz_, standoff_.first, standoff_.second, 0.4)
                  .has_value();
              RCLCPP_WARN(get_logger(),
                          "%d 号本体 Unity(%+.1f,%+.1f) 无路可达（不可驶入区）→ 行驶目标"
                          "用其前方 %.1f 单位射击位 (%+.1f,%+.1f)%s",
                          st.number, live.first, live.second, stop_d,
                          standoff_.first, standoff_.second,
                          standoff_reachable_ ? "" : "（该点也无路，回落配置活动区点）");
            }
          }
          const std::pair<double, double> want = live_reachable_ ? live
              : (standoff_reachable_ ? standoff_ : std::make_pair(st.gx, st.gz));
          if (std::hypot(want.first - goal_.first, want.second - goal_.second) > 1e-6) {
            goal_ = want;
            path_valid_ = false;
            RCLCPP_INFO(get_logger(), "%d 号移动 → 行驶目标 Unity(%+.1f,%+.1f)，重规划",
                        st.number, goal_.first, goal_.second);
          }
        }
        const double d_live = std::hypot(live.first - st_ux_, live.second - st_uz_);
        const double resume_d = stop_d + 1.5;
        if (waiting_) {
          if (d_live > resume_d) {
            waiting_ = false; //跑出射程，继续追
            RCLCPP_INFO(get_logger(), "%d 号跑出射程（%.1f）→ 追", st.number, d_live);
          } else {
            publish_speed(0, 0); //射程内停下打
            if (check_condition()) {
              RCLCPP_INFO(get_logger(), "★ %s 完成（%s）→ 下一站", st.name.c_str(),
                          st.done.c_str());
              advance_stage();
            }
            return;
          }
        } else if (d_live < stop_d) {
          waiting_ = true;
          publish_speed(0, 0);
          RCLCPP_INFO(get_logger(), "进入射程（%.1f < %.1f）→ 停车打 %d 号",
                      d_live, stop_d, st.number);
          if (check_condition()) {
            RCLCPP_INFO(get_logger(), "★ %s 完成（%s）→ 下一站", st.name.c_str(),
                        st.done.c_str());
            advance_stage();
          }
          return;
        }
      }
    }

    const double d_goal = std::hypot(goal_.first - st_ux_, goal_.second - st_uz_);
    if (d_goal < kGoalTol) {
      publish_speed(0, 0);
      if (st.done == "arrive") {
        if (st.wait > 0.0 && dwell_t0_.time_since_epoch().count() == 0) {
          dwell_t0_ = now_tp;
          RCLCPP_INFO(get_logger(), "到点（%s）—— 停留 %.0f 秒（补弹/补给）",
                      st.name.c_str(), st.wait);
        }
        if (st.wait > 0.0 &&
            std::chrono::duration<double>(now_tp - dwell_t0_).count() < st.wait) {
          return; //停留中保持零速
        }
        RCLCPP_INFO(get_logger(), "到点（%s）", st.name.c_str());
        advance_stage();
        return;
      }
      if (!wait_logged_) {
        wait_logged_ = true;
        RCLCPP_INFO(get_logger(), "已到 %s，停住开打，等 %s 条件", st.name.c_str(),
                    st.done.c_str());
      }
      //打符站到点最多等 kRuneStageTimeout 秒，未激活也推进；激活则走 check_condition
      if (st.done == "rune") {
        if (rune_arrive_t0_.time_since_epoch().count() == 0) {
          rune_arrive_t0_ = now_tp;
          //到打符位即开始叫符，下一拍发第一条
          rune_req_active_ = true;
          rune_req_next_t_ = now_tp;
          RCLCPP_INFO(get_logger(), "到打符位：开始自动叫符（每 %.1fs 重试直到受理）",
                      rune_req_retry_ms_ / 1000.0);
        }
        const double at = std::chrono::duration<double>(now_tp - rune_arrive_t0_).count();
        if (at > kRuneStageTimeout) {
          RCLCPP_WARN(get_logger(), "打符站 %s 到点已 %.0f s（%s）→ 超时去下一站",
                      st.name.c_str(), at, mission_rune_done_ ? "已激活" : "未激活");
          advance_stage();
          return;
        }
      }
      if (check_condition()) {
        RCLCPP_INFO(get_logger(), "★ %s 完成（%s）→ 下一站", st.name.c_str(), st.done.c_str());
        advance_stage();
      }
      return;
    }

    //卡死检测：有指令但实际速度≈0；块心沿指令方向前移 1.4，防把车自己围死
    const double speed = std::hypot(st_uvx_, st_uvz_);
    const double cmd_speed = std::hypot(cmd_vx_, cmd_vz_);
    if (speed > 0.6) {
      ever_moved_ = true;
      stuck_since_ = {};
    } else if (cmd_speed > 1.0) {
      if (stuck_since_.time_since_epoch().count() == 0) {
        stuck_since_ = now_tp;
      } else {
        const double hold = ever_moved_ ? 0.6 : 3.0;
        if (std::chrono::duration<double>(now_tp - stuck_since_).count() > hold) {
          ++stuck_count_;
          if (stuck_count_ >= kStuckGiveUp) {
            mission_finished_ = true;
            publish_speed(0, 0);
            RCLCPP_ERROR(get_logger(), "卡了 %d 次，放弃。最后位置 (%.1f,%.1f)",
                         stuck_count_, st_ux_, st_uz_);
            return;
          }
          const double un = cmd_speed > 1e-6 ? cmd_speed : 1.0;
          const double bx = st_ux_ + cmd_vx_ / un * 1.4;
          const double bz = st_uz_ + cmd_vz_ / un * 1.4;
          grid_.block_around(bx, bz, 1.6);
          //只在当局内存标障碍、下一拍重规划绕开；不落盘，避免假障碍污染后续任务
          publish_speed(0, 0);
          RCLCPP_WARN(get_logger(),
                      "卡住（命令 %.1f 实际 %.1f）Unity(%+.1f,%+.1f) —— 已标障碍重规划（第 %d/%d 次）",
                      cmd_speed, speed, st_ux_, st_uz_, stuck_count_, kStuckGiveUp);
          path_valid_ = false;
          ever_moved_ = false;
          stuck_since_ = {};
          return;
        }
      }
    } else {
      stuck_since_ = {};
    }

    //pure pursuit：速度朝前瞻点，mission_vmax_ 封顶（Unity 单位/s）
    const auto look = lookahead_point();
    const double kp = mission_vmax_ / kLookahead;
    double vx = kp * (look.first - st_ux_);
    double vz = kp * (look.second - st_uz_);
    const double sp = std::hypot(vx, vz);
    if (sp > mission_vmax_) {
      vx *= mission_vmax_ / sp;
      vz *= mission_vmax_ / sp;
    }
    publish_speed(vx, vz);
  }

  //到打符位后每 rune_req_retry_ms 发一条 RuneRequest，直到 accepted；离开站点由 enter_stage 复位
  void service_rune_requests() {
    if (!rune_req_active_ || rune_req_ok_.load() || !pub_rune_req_) return;

    const auto now_tp = std::chrono::steady_clock::now();
    if (now_tp < rune_req_next_t_) return;
    rune_req_next_t_ = now_tp + std::chrono::milliseconds(rune_req_retry_ms_);

    tdt_interface::msg::RuneRequest req;
    req.header.stamp = now(); //请求创建时间
    req.request_id = ++rune_req_id_; //非零且单调
    rune_req_last_id_.store(req.request_id);
    pub_rune_req_->publish(req);
    RCLCPP_INFO(get_logger(), "叫符: 已发 RuneRequest #%lu（%.1fs 没受理就重发）",
                static_cast<unsigned long>(req.request_id), rune_req_retry_ms_ / 1000.0);
  }

  //搜索：yaw 匀速转，俯仰按累计转角的余弦在 [min,max] 抬上放下；负角=往上看
  void search_step(double dt, double & yaw, double & pitch) {
    const double step = search_rate_ * dt;
    search_travel_ += step;
    yaw = wrap180(last_yaw_ + step); //从上次发出的角继续转
    const double lo = search_pitch_min_, hi = search_pitch_max_;
    pitch = lo + (hi - lo) *
            (1.0 - std::cos(search_pitch_cycles_ * deg2rad(search_travel_))) / 2.0;
  }

  //projectile_speed 可能是场景单位/s：15~35 当 m/s，45~105 除以 3，否则用 23.5 并告警一次
  double rune_bullet_speed(double raw) {
    if (raw >= 15.0 && raw <= 35.0) return raw;
    if (raw >= 45.0 && raw <= 105.0) return raw / 3.0;
    if (!rune_warned_v_) {
      rune_warned_v_ = true;
      RCLCPP_WARN(get_logger(), "projectile_speed=%.1f 不在合理区间，用标称 23.5 m/s", raw);
    }
    return 23.5;
  }

  void rune_aim_step(const tdt_interface::msg::AutoAimFrame & msg, const cv::Mat & frame,
                     double yaw_now, double pitch_now) {
    //第 3 次起大符（先用完 2 次小符）；任务提示 big 可强制
    const bool big = mission_rune_big_.load() ||
                     rune_counter_.count() > max_small_runes_;

    if (!rune_started_) {
      search_travel_ = 0.0;
      rune_lost_ = 0;
      rune_seen_ = false; //新开符，从没跟到过开始
      rune_started_ = true;
      RCLCPP_INFO(get_logger(), "-> 打符模式（CampusRune 链路：%s，第 %d 次开符）",
                  big ? "大符" : "小符", rune_counter_.count());
    }

    //RuneDetector 检测；精修/位姿/相位/弹道一次调用完成
    auto blades = rune_detector_->detect(frame);
    int n_cls0 = 0, n_cls1 = 0;
    for (const auto & b : blades) {
      if (b.class_id == 0) n_cls0++;
      else if (b.class_id == 1) n_cls1++;
    }

    //car 系=世界对齐的 (右/下/前)，与 world_from_cam 语义一致
    const Eigen::Matrix3d R_world_from_cam = world_from_cam(yaw_now, pitch_now);
    const uint64_t stamp = static_cast<uint64_t>(stamp_ns(msg.header));
    const double v0 = rune_bullet_speed(msg.state.projectile_speed);
    //精修
    RuneCampus::Result res;
    try {
      res = rune_campus_->process(frame, blades, big, intr_K_, intr_D_,
                                R_world_from_cam, stamp, rune_my_color_, v0);
    } catch (const std::exception & e) {
      //内部异常兜住，本帧按未找到处理，节点不崩
      if (!rune_warned_exc_) {
        rune_warned_exc_ = true;
        RCLCPP_ERROR(get_logger(),
                     "CampusRune 抛异常（已兜住，本帧按未找到处理）：%s", e.what());
      }
    }

    double game_yaw = last_yaw_, game_pitch = last_pitch_;
    bool shoot = false;
    const char * block = "搜索";
    const char * state_txt = "搜索";
   //火控
    if (res.found) {
      rune_lost_ = 0; //跟住就清暂丢计数
      rune_seen_ = true; //这次开符跟到过符，之后丢检只保持
      //yaw 已是游戏约定（弧度）；pitch 抬头为正，游戏向下为正故取负
      game_yaw = rad2deg(res.yaw);
      game_pitch = -rad2deg(res.pitch);
      shoot = res.fire && auto_fire_;
      block = res.fire ? "" : "CampusRune未放行";
      state_txt = "跟随";
    } else {
      //暂丢：打掉一片/换片时检出会闪断，先冻结云台等回来，连续丢够才搜索
      const auto now_tp = std::chrono::steady_clock::now();
      const double dt = std::clamp(
          std::chrono::duration<double>(now_tp - last_frame_t_).count(), 0.0, 0.5);
      if (!rune_seen_ && ++rune_lost_ > rune_lost_tolerance_) {
        double my = 0.0, mp = 0.0;
        if (mission_hold(my, mp)) { //任务提示优先
          game_yaw = my;
          game_pitch = mp;
        } else {
          search_step(dt, game_yaw, game_pitch);
        }
        block = "搜索";
        state_txt = "搜索";
      } else {
        game_yaw = last_yaw_; //重发上一条指令（游戏 0.5s 超时）
        game_pitch = last_pitch_;
        block = "等目标回来";
        state_txt = "暂丢";
      }
    }
    last_frame_t_ = std::chrono::steady_clock::now();

    send_angles(game_yaw, game_pitch, shoot);

    //RUNE_DEBUG=1 时逐帧打印：#RDBG 采集戳/found/fire/指令角 | 相位/omega | 冷却/放行 | 叶框
    static const bool rune_debug = (std::getenv("RUNE_DEBUG") != nullptr);
    if (rune_debug) {
      const power_rune::RuneDebugInfo ti = power_rune::debug_get_tracked();
      std::printf("#RDBG %ld %d %d %.3f %.3f | %d %.4f %.4f %.4f %.4f %.4f "
                  "%.4f %.4f %.4f | %.3f %.3f %d | %zu",
                  static_cast<long>(stamp_ns(msg.header)), (int)res.found, (int)res.fire,
                  game_yaw, game_pitch, (int)ti.valid, ti.phase, ti.omega,
                  ti.rune_center[0], ti.rune_center[1], ti.rune_center[2],
                  ti.armor_center[0], ti.armor_center[1], ti.armor_center[2],
                  ti.fire_cooldown_left, ti.fire_time_acc, ti.enable_fire, blades.size());
      for (const auto & b : blades) {
        if (b.points.size() < 5) continue;
        const cv::Point2f c = b.blade_center();
        std::printf(" [%d %d %d %d %d]",
                    (int)b.points[2].x, (int)b.points[2].y, (int)c.x, (int)c.y, b.class_id);
      }
      std::printf("\n");
      std::fflush(stdout);
    }

    if (show_window_) {
      draw_rune(frame, blades, big, n_cls0, n_cls1, state_txt, block, res.found, res.fire);
      cv::imshow("auto_aim", frame);
      if ((char)cv::waitKey(1) == 27) rclcpp::shutdown();
    }

    if (++rune_processed_ % 60 == 0) {
      RCLCPP_INFO(get_logger(),
                  "打符[CampusRune %s 第%d次] cls0=%d cls1=%d | found=%d fire=%d "
                  "指令 yaw=%.2f pitch=%.2f | v0=%.1fm/s | %s",
                  big ? "大符" : "小符", rune_counter_.count(), n_cls0, n_cls1,
                  (int)res.found, (int)res.fire, game_yaw, game_pitch, v0, block);
    }
  }

  //画打符检出框（灰=未激活/橙=小符/绿=大符）和 HUD
  void draw_rune(const cv::Mat & frame, const std::list<RuneBlade> & blades, bool big,
                 int n_cls0, int n_cls1, const char * state_txt, const char * block,
                 bool found, bool fire) {
    for (const auto & b : blades) {
      cv::Scalar col(160, 160, 160); //cls0 未激活
      if (b.class_id == 1) col = cv::Scalar(0, 165, 255); //小符亮灯（橙）
      if (b.class_id == 2) col = cv::Scalar(0, 220, 0); //大符亮灯（绿）
      std::vector<cv::Point> poly;
      for (int i : {0, 1, 4, 3})
        poly.emplace_back(cv::Point(b.points[i]));
      cv::polylines(frame, poly, true, col, 2);
      cv::circle(frame, b.rune_center(), 3, cv::Scalar(255, 255, 255), -1);
      cv::putText(frame, "c" + std::to_string(b.class_id),
                  cv::Point(b.blade_center()) + cv::Point(6, -6),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, col, 1);
    }
    char l1[128], l2[128];
    std::snprintf(l1, sizeof(l1), "%s 第%d次  CampusRune  目标叶%d片(未激活) 亮灯%d片",
                  big ? "大符" : "小符", rune_counter_.count(), n_cls0, n_cls1);
    std::snprintf(l2, sizeof(l2), "状态:%s  found:%s  开火:%s  %s",
                  state_txt, found ? "有" : "无", fire ? "许可" : "无",
                  block[0] ? block : "-");
    cv::rectangle(frame, cv::Point(0, 0), cv::Point(640, 56), cv::Scalar(0, 0, 0), -1);
    cv::putText(frame, l1, cv::Point(10, 22), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 255), 2);
    cv::putText(frame, l2, cv::Point(10, 46), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 255, 255), 2);
  }

  //唯一出口：发角度和射击许可；last_*/fire_sent_ 无条件更新，记录的是节点意图
  void send_angles(double yaw, double pitch, bool shoot) {
    //NaN/Inf 游戏整包拒收（规则 7.3），换发上一条好指令
    if (!std::isfinite(yaw) || !std::isfinite(pitch)) {
      if (!nan_warned_) {
        nan_warned_ = true;
        RCLCPP_WARN(get_logger(), "★ 解算出非有限角度 yaw=%.1f pitch=%.1f → 保持上一条好指令",
                    yaw, pitch);
      }
      yaw = last_good_yaw_;
      pitch = last_good_pitch_;
    } else {
      nan_warned_ = false;
      last_good_yaw_ = yaw;
      last_good_pitch_ = pitch;
    }
    last_yaw_ = yaw;
    last_pitch_ = pitch;
    fire_sent_ = shoot;
    if (!publish_angles_) return;

    tdt_interface::msg::SendData sd;
    sd.yaw = static_cast<float>(yaw);
    //限幅防撞机械限位（pitch_limit_deg 默认 60°）
    sd.pitch = static_cast<float>(std::clamp(pitch, -pitch_limit_, pitch_limit_));
    sd.if_shoot = shoot;
    pub_->publish(sd);
  }

  //规则 7.2：帧有问题时主动发 if_shoot=false，但不甩云台；没发过许可就不重复发
  void withdraw_fire() {
    if (fire_sent_) send_angles(last_yaw_, last_pitch_, false);
  }

  //看门狗：超 0.5 s 没成功处理帧就撤销射击许可
  void watchdog() {
    const double idle = std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - last_rx_).count();
    if (idle < kCommandTimeoutSec) {
      stream_warned_ = false; //帧恢复，下次断流重新报
      return;
    }
    if (fire_sent_) {
      send_angles(last_yaw_, last_pitch_, false);
      RCLCPP_WARN(get_logger(), "断流 %.2fs → 已撤销射击许可", idle);
    } else if (!stream_warned_) {
      //裸断流也报一次，区分帧停了和回调卡死
      stream_warned_ = true;
      RCLCPP_WARN(get_logger(), "断流 %.2fs（帧停了？）", idle);
    }
  }

  static const char * state_name(AimState s) {
    switch (s) {
      case AimState::kSearch:    return "搜索";
      case AimState::kDetecting: return "确认";
      case AimState::kTracking:  return "跟踪";
      case AimState::kTempLost:  return "暂丢";
      case AimState::kNoAmmo:    return "无弹";
    }
    return "?";
  }

  //算帧率:帧率取最近 kFpsWindow 帧的滑动窗口
  static constexpr size_t kFpsWindow = 60;
  static double rate_fps(const std::deque<std::chrono::steady_clock::time_point> & q) {
    if (q.size() < 2) return 0.0;
    const double span = std::chrono::duration<double>(q.back() - q.front()).count();
    return span > 1e-6 ? static_cast<double>(q.size() - 1) / span : 0.0;
  }
  std::deque<std::chrono::steady_clock::time_point> rx_times_, proc_times_;
  double ema_ms_ = 0;
  int processed_ = 0;

  //成员；cfg_ 是配置副本（enemy_color 已覆盖），换局重建 Tracker 时复用
  YAML::Node cfg_;
  Color enemy_ = blue;
  int last_locked_ = -1; //上一帧锁的谁
  uint64_t last_seq_ = 0; //上一帧 frame_sequence，回退即换局（7.5）
  int frame_id_ = 0; //可视化帧号
  bool intr_ready_ = false, warned_fb_ = false; //首次日志用
  bool publish_angles_ = false, auto_fire_ = false, show_window_ = false;
  bool fire_sent_ = false; //当前是否发过射击许可
  char last_fire_block_[64] = ""; //本帧开火否决原因；64 字节防中文截断
  bool stream_warned_ = false; //裸断流只报一次
  bool nan_warned_ = false; //非有限角告警只报一次
  std::chrono::steady_clock::time_point last_slow_warn_{}; //慢帧告警节流
  double last_good_yaw_ = 0.0, last_good_pitch_ = 0.0; //上一条有限的好指令

  //开火决策状态
  bool have_prev_cmd_ = false;
  double prev_cmd_yaw_ = 0.0;
  double first_tol_ = 3.0, second_tol_ = 2.0, judge_dist_ = 2.0; //度 / 米
  //最大开火水平距离（米）；<=0 不限
  double max_fire_distance_ = 4.0;

  //底盘速度门（Unity 单位/s，3 单位=1m；<=0 关闭）
  double fire_max_chassis_speed_ = 1.0;
  double chassis_speed_ = 0.0; //每帧从 state.twist 更新

  //前哨站收敛门：锁定后 outpost_settle_s 秒内不开火（dz 还在学）
  double outpost_settle_s_ = 4.0;
  double outpost_lock_t0_ = -1e9; //锁上前哨站的时刻

  //换板门（只作用于车）：上次板号与换板时刻
  int    last_aim_plate_id_ = -1;
  double plate_switch_t_ = -1e9;
  double plate_switch_settle_s_ = 0.0; //秒；0=关闭
  //ammo_remaining <= 它视为打空，进 kNoAmmo
  int min_ammo_to_fire_ = 0; //发
  //空弹告警只报一次
  bool warned_ammo_ = false;

  //上一条前哨站旋向日志值；-2 哨兵（-1 是合法顺时针）
  int last_outpost_dir_ = -2;

  //本帧是否基地直瞄（日志与可视化用）
  bool aiming_base_ = false;

  //打符成员与参数
  std::unique_ptr<RuneDetector>    rune_detector_;
  std::unique_ptr<RuneCampus>        rune_campus_; //CampusRune 对接层（上游 RP-26Rune）
  RuneActivationCounter rune_counter_{22.0};

  //自动叫符（到点重试直到 accepted）
  int    rune_req_retry_ms_ = 2000; //重试间隔（config: rune.request_retry_s）
  uint64_t rune_req_id_ = 0; //下一条 request_id
  std::atomic<uint64_t> rune_req_last_id_{0}; //最后发出的 id
  std::atomic<bool>     rune_req_ok_{false}; //本轮已受理
  uint64_t rune_req_last_accepted_id_ = 0; //已计数的 id（防重复计数）
  bool     rune_req_active_ = false; //在到点叫符循环里
  std::chrono::steady_clock::time_point rune_req_next_t_{};
  double   rune_req_reject_log_t_ = -1e9; //被拒日志限流（5s 一条）

  //每帧相机内参（要求同帧 camera_info）
  cv::Mat intr_K_, intr_D_;
  int  rune_my_color_ = 1; //符灯颜色：0 红 / 1 蓝
  bool rune_warned_exc_ = false; //异常只报一次

  bool rune_enabled_ = false; //模型加载成功才为 true
  bool mode_forced_ = false; //config rune.mode 明确写了 armor/rune
  Mode forced_mode_ = Mode::kArmor;
  double rune_active_s_ = 22.0;
  int    max_small_runes_ = 2;

  //参数在 third_party/campus-rune/config/power_rune.json

  //搜索：俯仰按累计转角的余弦在 [min,max] 间来回
  double search_pitch_min_ = 0.0, search_pitch_max_ = -35.0, search_pitch_cycles_ = 3.0;
  double search_travel_ = 0.0; //累计转角（度），俯仰相位用

  //打印节流计数
  int rune_processed_ = 0;
  bool rune_started_ = false; //本次开符是否已初始化

  //识别回调与控制回调分组隔离；mission_mu_ 护提示角快照，flag 用 atomic，st_* 单写单读不加锁
  mutable std::mutex mission_mu_;
  double mission_yaw_ = 0.0, mission_pitch_ = 0.0;
  double mission_stamp_ = -1e9; //steady_seconds() 时间戳；超 2s 无提示
  std::atomic<bool> mission_rune_{false}; //任务要求打符
  std::atomic<bool> mission_rune_big_{false}; //"hold … rune big"：强制大符

  //全自动任务（mission:=true 启用）
  static constexpr double kLookahead = 3.0; //pure pursuit 前视距离（场景单位）
  static constexpr double kGoalTol = 2.0; //到点容差
  static constexpr double kStopDist = 4.5; //hp 站停车距离
  static constexpr double kResumeDist = 6.0; //hp 站重新追击距离（迟滞）
  static constexpr double kReplanDist = 3.0; //目标移动超过它就重规划
  static constexpr double kPlanGiveUp = 10.0; //规划连续失败上限（秒）
  //卡死累计次数上限（全任务累计），超过放弃任务
  static constexpr int kStuckGiveUp = 15;
  //打符站到点停留上限（秒），未激活也推进
  static constexpr double kRuneStageTimeout = 70.0;
  struct Stage {
    std::string name, done; //done: arrive / hp / rune
    double gx = 0, gz = 0, aim_pitch = -4.0;
    double wait = 0.0; //到点后停留 N 秒（补弹等）
    double stop = 0.0; //距目标多远停车打（0=用全局 kStopDist）
    //定点打：不跟实时位置，只开到配置点停住打
    bool fixed = false;
    int number = 0; //hp 站打的是几号
    std::string aim_type; //"" / "rune" / "enemy"
    int aim_number = 0;
  };
  NavGrid grid_;
  bool mission_on_ = false;
  double mission_vmax_ = 6.0; //最大速度（Unity 单位/s，6=2 m/s）
  std::vector<Stage> stages_;
  int stage_i_ = 0;
  std::pair<double, double> goal_{0, 0};
  std::vector<std::pair<double, double>> path_;
  size_t path_i_ = 0;
  bool path_valid_ = false;
  //实时目标可达性缓存：同位置只探一次；挪出 1.0 单位重探
  std::pair<double, double> live_probe_{0, 0};
  bool live_probe_done_ = false;
  bool live_reachable_ = false;
  //本体不可驶入时的前方射击位（朝车一侧退 stop_d）及可达性
  std::pair<double, double> standoff_{0, 0};
  bool standoff_reachable_ = false;
  //规划失败重试计时 / 日志节流
  std::chrono::steady_clock::time_point plan_fail_t0_{};
  double last_plan_warn_s_ = 0.0;
  double rune_yaw_ = 45.05, rune_pitch_ = -15.04; //打符预设角（json 可覆盖）
  std::chrono::steady_clock::time_point prewarm_t0_{}, pause_until_{}, stuck_since_{}, dwell_t0_{};
  std::chrono::steady_clock::time_point rune_arrive_t0_{}; //打符站到点时刻
  bool ever_moved_ = false;
  int stuck_count_ = 0;
  bool waiting_ = false; //到点后等"打掉"条件
  bool wait_logged_ = false, mission_finished_ = false;
  std::string learned_path_;
  std::vector<std::pair<double, double>> learned_;
  std::atomic<uint32_t> mission_filter_mask_{0}; //任务期允许开火的兵种位图（0=都不许）
  std::atomic<bool> mission_rune_done_{false}; //符已激活（rune 站判据）
  int rune_count_at_stage_entry_ = 0; //进站时开符计数快照
  std::unordered_map<int, int> enemy_hp_;
  std::unordered_map<int, std::pair<double, double>> enemy_pos_;
  std::set<int> enemy_seen_;
  std::chrono::steady_clock::time_point last_aim_pub_{};
  double st_ux_ = 0, st_uz_ = 0, st_uvx_ = 0, st_uvz_ = 0;
  double cmd_vx_ = 0, cmd_vz_ = 0;
  bool st_seen_ = false;
  std::chrono::steady_clock::time_point st_stamp_{};
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr speed_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_enemies_;

  //控制回调组（底盘/任务）；识别回调走默认组
  rclcpp::CallbackGroup::SharedPtr ctrl_cb_group_;
  bool rune_warned_v_ = false; //projectile_speed 异常只警告一次
  bool rune_finished_ = false; //收到 activated：本轮打完，立即回车自瞄
  bool rune_seen_ = false; //本次开符跟到过符，之后丢检只保持
  int  rune_lost_ = 0; //暂丢连续帧数
  int  rune_lost_tolerance_ = 45; //暂丢容忍帧数

  //状态机
  AimState state_ = AimState::kSearch;
  int lost_ = 0; //kDetecting / kTempLost 连续丢帧计数
  int detect_count_ = 0; //kDetecting 连续看到帧数
  int min_detect_count_ = 5; //连续看到几帧才进跟踪
  double search_rate_ = 120.0; //搜索 yaw 角速度（度/秒）
  int max_temp_lost_ = 15; //暂丢容忍帧数
  bool started_ = false; //首帧对齐 last_yaw_
  std::chrono::steady_clock::time_point last_frame_t_ = std::chrono::steady_clock::now();
                                          //搜索 dt 用

  //kDetecting 连续丢这么多帧回搜索
  static constexpr int kDetectGiveUp = 3;
  double pitch_limit_ = 60.0;
  double last_yaw_ = 0, last_pitch_ = 0;
  std::chrono::steady_clock::time_point last_rx_;

  std::unique_ptr<Detector> detector_;
  std::unique_ptr<Solver> solver_;
  std::unique_ptr<Tracker> tracker_;
  std::unique_ptr<Aimer> aimer_;
  std::unique_ptr<Visualizer> visualizer_;

  rclcpp::Subscription<tdt_interface::msg::AutoAimFrame>::SharedPtr sub_;
  rclcpp::Subscription<tdt_interface::msg::RuneResult>::SharedPtr sub_rune_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mission_;
  rclcpp::Publisher<tdt_interface::msg::SendData>::SharedPtr pub_;
  rclcpp::Publisher<tdt_interface::msg::RuneRequest>::SharedPtr pub_rune_req_; //叫符出口
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  //单线程执行器；回调分组和锁保留，可换 MultiThreadedExecutor
  rclcpp::spin(std::make_shared<AutoAimNode>());
  rclcpp::shutdown();
  return 0;
}
