// ============================================================================
// auto_aim_node —— 把 4.4 的自瞄工程接进 tdt-campus-game
//
//   /autoaim/frame ──► 校验+解码 ──► Detector(TensorRT) ──► Solver(PnP)
//                          │                                    │
//        state.yaw/pitch_degrees ─────────────────────────► Tracker(EKF) ─► Aimer
//                                                                │
//                                                                ▼
//                                              /target_angles_player_N (SendData)
//
// 规则对应：
//   7.1  内参优先取同帧 camera_info，取不到才用手册表 12 标称值
//   7.2  嵌套时间戳必须一致、帧龄超限丢帧；断流超 0.5 s 主动撤销射击许可
//   7.5  frame_sequence 必须单调；回退视为换局 → 清滤波器重来
//
// 坐标系（★ 读代码前先看这段，它是整个节点最容易搞错的地方）：
//
//   检测/PnP 在【相机系】里做（solvePnP 的输出就是这个系）；
//   但进 EKF 之前，每一帧的观测都会用 world_from_cam(云台 yaw, 云台 pitch)
//   旋转到【世界系】—— 只转不平移，原点是相机光心。这么做的原因是：
//   否则云台一转，目标在跟踪器眼里就跟着整体转，被当成目标的真实运动
//   （最难受的是"指令发出、云台正在转过去"那一段）。
//
//   于是 EKF 的状态、Aimer 的 aim_3d 全都在世界系，输出就是直读：
//     world_yaw   = rad2deg(atan2(aim_3d(0), aim_3d(1)))   ← atan2(右, 前)
//     world_pitch = -rad2deg(aim.pitch)
//   ★【不要再加"当前云台角度"】—— 姿态已经烘进观测旋转里了。
//     4.3 那版没做世界系，输出必须写成 `云台 yaw + atan2(前, 右)` 而且 pitch
//     要落在光轴上；两个约定都写反了（详见工程根目录说明文档 8.3 节）。
//   ★ 游戏约定（Unity）：yaw 以 +Z 为 0、顺时针为正；pitch 向下为正。
//     aim_3d 的三分量是 (右, 前, 上)（pseudo 系，见 target.hpp 顶部），
//     aim.pitch 是相对水平面的发射仰角、向上为正 —— 所以才有上面那两个负号。
//
// 精简取舍（相对 4.3 那版）：不发底盘速度、不录视频、不做按键交互。
// ============================================================================

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

// 打符（kRune 模式）—— 算法为 RP-26Rune 移植（2026-10-04 换装）：
//   检测仍然是我们的 RuneDetector（0624 模型，和 RP26 原版同一个），
//   后面的精修/位姿/相位/火控弹道整套走 third_party/rp26_rune（经 rune_rp26 对接层）。
//   老链路（rune_aimer/rune_target/rune_picker/rune_solver）保留在顶层 src/ 里，
//   只给离线对拍工具 rune_aim 用，节点不再引用。
#include "rune_activation.hpp"
#include "rune_detector.hpp"
#include "rune_rp26.hpp"
#include "power_rune_interface.hpp"   // debug_get_tracked()（RUNE_DEBUG=1 时的逐帧调试口）

using namespace std::chrono_literals;

namespace {

// 规则 7.2：帧龄窗口 与 允许的时钟抖动
constexpr int64_t kMaxFrameAgeNs = 250'000'000;  // 250 ms
constexpr int64_t kMaxFutureNs   = 20'000'000;   // 20 ms

// 规则 7.2：Unity 侧超时是 0.5 s，本节点自己也按这个判断流
constexpr double kCommandTimeoutSec = 0.5;

// 手册表 12 的 ROS 图像标称内参（960×720，无畸变），只作回落
constexpr double kFallbackFx = 1159.420, kFallbackFy = 1159.420;
constexpr double kFallbackCx = 480.0,    kFallbackCy = 360.0;

int64_t stamp_ns(const std_msgs::msg::Header & h) {
  return static_cast<int64_t>(h.stamp.sec) * 1'000'000'000LL +
         static_cast<int64_t>(h.stamp.nanosec);
}
double rad2deg(double r) { return r * 180.0 / M_PI; }
double deg2rad(double d) { return d * M_PI / 180.0; }

// 把角度收进 (-180, 180]
// （和 types.hpp 的 limit_rad 同一套防巨值处理：巨值直接 while 会卡死节点）
double wrap180(double d) {
  d = std::fmod(d, 360.0);
  while (d > 180.0) d -= 360.0;
  while (d <= -180.0) d += 360.0;
  return d;
}

// 云台姿态 → 世界系旋转（相机排版，可以直接乘 xyz_in_camera）。
//   ψ = state.yaw_degrees   （Unity 约定：+Z 为 0、顺时针为正）
//   θ = state.pitch_degrees （向下为正）
// 在 pseudo 基（右/前/上）下就是 R = Rz(-ψ)·Rx(-θ)：
// 先绕自身"右"轴把俯仰摆平，再绕世界"上"轴转到世界朝向。
// 相机排版(右/下/前)与 pseudo 差一个换基 M，两头夹一下即可。
Eigen::Matrix3d world_from_cam(double yaw_deg, double pitch_deg) {
  Eigen::Matrix3d M;                      // pseudo = M · cam
  M << 1, 0, 0,
       0, 0, 1,
       0,-1, 0;
  const Eigen::Matrix3d R_pseudo =
      (Eigen::AngleAxisd(-deg2rad(yaw_deg), Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(-deg2rad(pitch_deg), Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  return M.transpose() * R_pseudo * M;
}

// 基地有上下两块板（新模型的 9 类里第 7、8 类分别对应底部/顶部，detector
// 把它们都映射成 base），两块同时可见时挑【离画面中心最近】的那块打。
// 选最近的而不是"最高的"或"任一块"：画面中心≈枪口轴线，打它云台几乎不用转，
// 弹道和 PnP 误差也最小。
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

// 把本帧的观测从相机系搬进世界系：位置用旋转矩阵，板法线角跟着一起转。
// 法线不能只"加个 yaw 偏移"：相机有俯仰时它不再是水平向量，所以还原成
// 单位向量 → 旋转 → 再取回角度（Solver 那边定义 a.yaw = atan2(nz, nx)）。
// ★ 不转法线的话，观测里的 face_yaw 和状态里的车朝向不在同一个系，
//   数据关联（Target::update 里比 face_yaw 那一步）会成片地配错板。
// ★ 只动 xyz_in_camera 和 yaw；rvec/tvec/points 保持相机系不变 ——
//   它们要给 PnP 重投影画框用（Visualizer 依赖这一点）。
void armors_to_world(std::list<Armor> & armors, const Eigen::Matrix3d & R) {
  for (auto & a : armors) {
    a.xyz_in_camera = R * a.xyz_in_camera;
    Eigen::Vector3d n(std::cos(a.yaw), 0.0, std::sin(a.yaw));
    n = R * n;
    a.yaw = std::atan2(n(2), n(0));
  }
}

}  // namespace

class AutoAimNode : public rclcpp::Node {
public:
  // 状态机（照 sp_vision_25 的 tracker 状态机简化）
  //   kSearch    没锁定 —— 云台匀速转，扫一圈找目标
  //   kDetecting 看到目标但还没锁上 —— 停住别动，让跟踪器把它锁住
  //   kTracking  有锁定 —— 正常瞄准 + 开火决策
  //   kTempLost  锁定目标丢了（被遮挡/转走了）—— 保持不动等它回来，
  //              超过 max_temp_lost_count 帧还回不来就回 kSearch
  //   kNoAmmo    弹量打空（ammo_remaining <= 0）—— 保持不动、不开火，
  //              等补给。它是个【覆盖全部瞄准状态】的暂停态：只进不出，
  //              弹药一恢复就按"还锁着没"决定回 kTracking 还是 kSearch
  enum class AimState { kSearch, kDetecting, kTracking, kTempLost, kNoAmmo };

  // 工作模式（同一时刻只能瞄一个东西 —— 一个话题只有一个发布者）：
  //   kArmor 车自瞄（默认，和以前完全一样）
  //   kRune  打符：任务要求或自动叫符 accepted 触发，
  //          单次开符窗口 rune_active_s 秒，窗口结束自动切回车模式
  // config 的 rune.mode 可以强制 "armor"/"rune"（调试用），默认 "auto"
  enum class Mode { kArmor, kRune };

  AutoAimNode() : Node("auto_aim_node") {
    // ---------------- 参数 ----------------
    const int player_n = declare_parameter("player_n", 1);   // 1 蓝方 / 2 红方
    const std::string enemy_color =
        declare_parameter("enemy_color", std::string("red"));
    publish_angles_ = declare_parameter("publish_angles", false);  // false = 只观察
    auto_fire_      = declare_parameter("auto_fire", false);
    show_window_    = declare_parameter("show_window", false);
    pitch_limit_    = declare_parameter("pitch_limit_deg", 60.0);
    const std::string cfg_path =
        declare_parameter("config_file", std::string("config_contest.yaml"));

    // ---------------- 算法 ----------------
    cfg_ = YAML::LoadFile(cfg_path);
    cfg_["enemy_color"] = enemy_color;
    enemy_ = (enemy_color == "blue") ? blue : red;

    // 开火决策的三个阈值
    if (cfg_["first_tolerance"])  first_tol_  = cfg_["first_tolerance"].as<double>();
    if (cfg_["second_tolerance"]) second_tol_ = cfg_["second_tolerance"].as<double>();
    if (cfg_["judge_distance"])   judge_dist_ = cfg_["judge_distance"].as<double>();

    // 状态机参数
    if (cfg_["search_yaw_rate"])     search_rate_   = cfg_["search_yaw_rate"].as<double>();
    if (cfg_["max_temp_lost_count"]) max_temp_lost_ = cfg_["max_temp_lost_count"].as<int>();
    if (cfg_["min_detect_count"])    min_detect_count_ = cfg_["min_detect_count"].as<int>();

    // 距离限制：水平距离超过 max_fire_distance_ 就不发 if_shoot。
    // ★ 只挡开火、不挡瞄准 —— 云台照常跟住目标，只是不给射击许可。
    // ★ 给 0（或负）表示不限距离，方便临时关掉这个门。
    if (cfg_["max_fire_distance"]) max_fire_distance_ = cfg_["max_fire_distance"].as<double>();
    // ★ 2026-10-17 用户："把锁定限制距离删掉，只保留开火限制距离" —— 原先的
    //   max_lock_distance 锁定距离门（10-16 定 3.5、10-17 上午改 4.5）已整个
    //   删除：锁定/跟踪不再有任何距离限制，多远都锁、只是不开火。
    // 弹量门槛：ammo_remaining <= min_ammo_to_fire_ 视为"打空了"（默认 0，
    // 即严格按"0 发打不出来"的规则语义）
    if (cfg_["min_ammo_to_fire"]) min_ammo_to_fire_ = cfg_["min_ammo_to_fire"].as<int>();

    // 底盘停稳门：车没停稳不给射击许可（"打车的时候停下来再打"）。
    // 单位 = Unity 场景单位/s（3 单位 = 1 m）；<= 0 关掉这道门。
    if (cfg_["fire_max_chassis_speed"])
      fire_max_chassis_speed_ = cfg_["fire_max_chassis_speed"].as<double>();

    // 前哨站收敛门：锁定后的前 N 秒不开火（等 dz1/dz2 学完；见 kTracking 分支）
    if (cfg_["outpost_settle_s"]) outpost_settle_s_ = cfg_["outpost_settle_s"].as<double>();

    // 换板开火门：aim.plate_id 变化后 N 秒内不给射击许可（见 decide_fire）。
    // 只作用于车（前哨站有单独的 outpost_settle_s；它的选板没有迟滞，见检测处）。
    if (cfg_["plate_switch_settle_s"])
      plate_switch_settle_s_ = cfg_["plate_switch_settle_s"].as<double>();

    cv::Mat K0 = cv::Mat::eye(3, 3, CV_64F), D0 = cv::Mat::zeros(1, 5, CV_64F);
    K0.at<double>(0, 0) = kFallbackFx; K0.at<double>(1, 1) = kFallbackFy;
    K0.at<double>(0, 2) = kFallbackCx; K0.at<double>(1, 2) = kFallbackCy;

    detector_  = std::make_unique<Detector>(cfg_);
    solver_    = std::make_unique<Solver>(K0, D0);
    tracker_   = std::make_unique<Tracker>(cfg_, *solver_);
    aimer_     = std::make_unique<Aimer>(25.0);   // 初速每帧按 state 更新
    visualizer_ = std::make_unique<Visualizer>(*solver_);

    // ---------------- 打符（kRune 模式，config 的 rune: 段）----------------
    // 打得开就开着；engine 缺失/配置没有 rune 段 → 只警告不报错，
    // 车自瞄照常工作（打符只是这套节点的第二个模式，不是必需品）
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
        // ★ 2026-10-05：到打符位自动叫符的重试间隔（秒 → ms），见 service_rune_requests
        if (rc["request_retry_s"])
          rune_req_retry_ms_ = static_cast<int>(rc["request_retry_s"].as<double>() * 1000.0);

        // ★ 2026-10-04：打符算法已换成 RP-26Rune（third_party/rp26_rune）。
        //   老链路的开火节奏/提前量/相位噪声等旋钮全部作废 —— RP26 自己的弹道
        //   （含空气阻力/马格努斯）、开火冷却、二值化阈值都在
        //   third_party/rp26_rune/config/power_rune.json 里调，别在这边找。
        //   下面只留节点级参数（自动叫符/次数/暂丢容忍/搜索扫掠/目标颜色）。

        // 符的目标颜色 = 符灯颜色。规则「红蓝双方能量机关独立」：看到的是自己方的
        // 符，灯就是自己方的颜色（RP26 官方红方素材、我们两卷蓝方素材都印证了）。
        // 默认跟敌方颜色反推：敌人红 → 我们蓝 → 1。rune.target_color 可以强制
        // "red"/"blue" 覆盖 —— 万一哪天红方视角实测灯色不是红的，改配置即可。
        rune_my_color_ = (enemy_color == "red") ? 1 : 0;
        if (rc["target_color"]) {
          const std::string tc = rc["target_color"].as<std::string>();
          if (tc == "red")  rune_my_color_ = 0;
          if (tc == "blue") rune_my_color_ = 1;
        }

        // 搜索（两种模式共用）：俯仰按累计转角的余弦在 [min,max] 之间平滑来回
        if (rc["search_pitch_min_deg"]) search_pitch_min_ = rc["search_pitch_min_deg"].as<double>();
        if (rc["search_pitch_max_deg"]) search_pitch_max_ = rc["search_pitch_max_deg"].as<double>();
        if (rc["search_pitch_cycles"])  search_pitch_cycles_ = rc["search_pitch_cycles"].as<double>();

        try {
          rune_detector_ = std::make_unique<RuneDetector>(cfg_);
          rune_rp26_     = std::make_unique<RuneRp26>();
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
    // 叫符 request_id：用毫秒时间戳播种（非零、单调，且远大于手工测试用的号），
    // 之后每发一条 +1（见 service_rune_requests）。
    rune_req_id_ = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());

    if (rune_enabled_) {
      RCLCPP_INFO(get_logger(),
                  "打符就绪: 算法=RP-26Rune 模式=%s 单次窗口=%.0fs "
                  "小符最多=%d 次 目标颜色=%s",
                  forced_mode_ == Mode::kRune ? "强制rune" :
                  (forced_mode_ == Mode::kArmor ? "强制armor" : "自动(任务/叫符)"),
                  rune_active_s_, max_small_runes_,
                  rune_my_color_ == 0 ? "红" : "蓝");
    }

    // ---------------- 话题 ----------------
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    sub_ = create_subscription<tdt_interface::msg::AutoAimFrame>(
        "/autoaim/frame", qos,
        [this](tdt_interface::msg::AutoAimFrame::SharedPtr m) { on_frame(std::move(m)); });

    pub_ = create_publisher<tdt_interface::msg::SendData>(
        "/target_angles_player_" + std::to_string(player_n), qos);

    // ★ 2026-10-05：叫符请求出口。任务打符站到点后由节点自动往这儿发（每
    //   request_retry_s 一条，直到 accepted），不再依赖手动开符。游戏按
    //   request_id 回 /game/rune/result（accepted=受理 / activated=增益激活）。
    pub_rune_req_ = create_publisher<tdt_interface::msg::RuneRequest>(
        "/game/rune/request", qos);

    // 符的增益回执（游戏在"这一轮符打完了"时发 activated=true）。
    // ★ 用 BEST_EFFORT 订阅：能匹配任何发布者（发布端 reliable/best_effort 都行），
    //   而 RELIABLE 订阅配不上 BEST_EFFORT 发布端 —— 这条消息丢了也不影响安全。
    sub_rune_ = create_subscription<tdt_interface::msg::RuneResult>(
        "/game/rune/result", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        [this](tdt_interface::msg::RuneResult::SharedPtr m) {
          on_rune_result(std::move(m));
        });

    // 控制回调组：底盘/任务（A* 规划、看门狗、enemies 解析）挂在这组里。
    // 当前 main 是单线程执行器（分组无副作用）；若要治"云台滞后"再换
    // MultiThreadedExecutor，两组就会各跑一根线程（成员已按并发加锁/atomic）。
    ctrl_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions ctrl_opts;
    ctrl_opts.callback_group = ctrl_cb_group_;

    // 外部云台提示（可选，mission_node.py 也往这儿发；节点内建任务直接写成员，不依赖它）。
    sub_mission_ = create_subscription<std_msgs::msg::String>(
        "/mission/cmd", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        [this](std_msgs::msg::String::SharedPtr m) { on_mission(std::move(m)); }, ctrl_opts);

    // ---------------- 全自动任务（底盘 + 站点推进，默认关）----------------
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
      if (mission_on_) mission_tick();      // 全自动任务：底盘 + 站点推进
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
  // ==========================================================================
  // 收帧：校验 → 入队（单线程直接算，2~3 ms/帧，不需要额外线程）
  // ==========================================================================
  void on_frame(tdt_interface::msg::AutoAimFrame::SharedPtr msg) {
    const int64_t stamp = stamp_ns(msg->header);
    const int64_t now = static_cast<int64_t>(get_clock()->now().nanoseconds());

    // 有且仅有一种图像载荷：raw 和 compressed 必须一个有一个空。
    // 两个都有说明发帧方行为不明确（该听谁的？），两个都空说明没图可算。
    // 这里用 `raw == jpg` 一次判掉两种坏情况，别拆成两个 if 写成 ||
    const bool raw = !msg->image.data.empty();
    const bool jpg = !msg->compressed_image.data.empty();
    if (raw == jpg) { withdraw_fire(); return; }

    // 嵌套 header 必须同源（7.2：不得用最新位姿解释旧图像）。
    // state（云台位姿/初速）、camera_info（内参）、image 三者必须和外层
    // header 的时间戳完全一致 —— 只要有一个不是同帧的，这一帧就不能用：
    // 拿旧位姿去转新图像，出来的角度会差一整个云台转角。
    const int64_t h_img = raw ? stamp_ns(msg->image.header)
                              : stamp_ns(msg->compressed_image.header);
    if (stamp_ns(msg->state.header) != stamp ||
        stamp_ns(msg->camera_info.header) != stamp || h_img != stamp) {
      withdraw_fire();
      return;
    }

    // 帧龄：太旧的（>250ms，说明积压了）和"来自未来"的（>20ms，说明两边
    // 时钟不同步）都丢。后者只给 20ms 的宽容，因为 Unity 和 ROS 是两个时钟，
    // 完全对齐不可能，但差出几十毫秒就说明有问题了
    if (now - stamp > kMaxFrameAgeNs || now - stamp < -kMaxFutureNs) {
      withdraw_fire();
      return;
    }

    // 采集帧率：只统计通过校验、真正送进算法的帧。
    // （被上面任何一条拦下的帧都不计 —— 否则帧率统计会把垃圾帧也算进去，
    //   掩盖"其实一直在丢帧"这件事）
    rx_times_.push_back(std::chrono::steady_clock::now());
    if (rx_times_.size() > kFpsWindow) rx_times_.pop_front();

    const auto cb_t0 = std::chrono::steady_clock::now();
    decode_and_aim(*msg, raw);

    // ★ last_rx_ 在【算完之后】才更新，不是收到就更新：
    //   看门狗判的是"多久没成功处理过一帧"，不是"多久没收到消息"。
    //   把这两件事分开，卡死/一直丢帧时看门狗才能正常触发撤射击许可。
    last_rx_ = std::chrono::steady_clock::now();

    // 单帧耗时异常时告警（正常 5~10 ms，每秒最多一条）—— "节点像卡死但没崩溃"
    // 类事故的第一手证据：巨值/无穷进循环、或某层解算被带飞，都会先在这里露头
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

  // ==========================================================================
  void decode_and_aim(const tdt_interface::msg::AutoAimFrame & msg, bool raw) {
    const auto t0 = std::chrono::steady_clock::now();

    cv::Mat frame;
    if (raw) {
      // 原始图（测试时用）：只认 rgb8，其余编码直接拒 —— 猜编码比报错更危险
      const auto & im = msg.image;
      if (im.encoding != "rgb8" || im.width == 0 || im.height == 0) return;
      // 零拷贝包装 ROS 的 buffer（不复制像素），下面 cvtColor 会写进 frame 自己的内存
      cv::Mat rgb(static_cast<int>(im.height), static_cast<int>(im.width), CV_8UC3,
                  const_cast<uint8_t *>(im.data.data()), static_cast<size_t>(im.step));
      // ★ 转成 BGR：整条链路（检测模型、OpenCV 画图）都按 BGR 工作。
      //   漏了这一步会得到"红蓝互换"的图 —— 检测出的颜色全反，敌我过滤直接废掉
      cv::cvtColor(rgb, frame, cv::COLOR_RGB2BGR);
    } else {
      // 压缩图（游戏实际发的）：整块 JPEG 交给 imdecode
      cv::Mat buf(1, static_cast<int>(msg.compressed_image.data.size()), CV_8UC1,
                  const_cast<uint8_t *>(msg.compressed_image.data.data()));
      frame = cv::imdecode(buf, cv::IMREAD_COLOR);
      if (frame.empty()) return;   // 坏 JPEG，直接丢
    }

    // 换局检测（7.5）：序号回退 → 清滤波器与锁定状态重来
    //
    // 做法是【整个 Tracker 重建】而不是遍历 tracks_ 清空 —— 重建顺带把
    // pinned 的 locked_id_/lost_frames_ 全体归零，比挨个清干净且不会漏。
    // Aimer 的"锁定同一块板"迟滞也要一起清，否则会拿着上一局的板编号继续用。
    // ★ 注意 last_seq_ 之外的成员没清（prev_cmd_yaw_ 等开火状态）——
    //   换局后第一帧 have_prev_cmd_ 仍是 true，理论上会拿上一局的指令做比较；
    //   实际影响极小（一帧之后就正常了），记录在此备案。
    const uint64_t seq = msg.state.frame_sequence;
    if (seq == 0) return;
    if (seq <= last_seq_) {
      const uint64_t prev = last_seq_;
      tracker_ = std::make_unique<Tracker>(cfg_, *solver_);
      aimer_->reset_lock();
      last_locked_ = -1;
      last_seq_ = seq;
      // 打符那套也要清（换局 = 新的两次小符机会）。RP26 的内部状态不用手动清：
      // 它自己按时间窗重建（调用间隔超 prepare_time/max_life_time 就重来），
      // 这里只清节点自己的标记。
      rune_counter_.reset();
      rune_count_at_stage_entry_ = 0;   // 计数清零，快照也要跟零（否则"叫过符"门卡死）
      rune_started_ = false;
      rune_finished_ = false;
      RCLCPP_WARN(get_logger(), "frame_sequence 回退(%lu → %lu)，按换局处理，状态已清",
                  static_cast<unsigned long>(prev),
                  static_cast<unsigned long>(seq));
    } else {
      last_seq_ = seq;
    }

    // 内参必须在检测/解算【之前】更新：Solver 里 K_ 是共享的，
    // 这一帧算的 PnP 用的就是刚设进去的这份
    apply_intrinsics(msg.camera_info, frame.size());

    const double yaw_now = msg.state.yaw_degrees;
    const double pitch_now = msg.state.pitch_degrees;

    // 底盘速度（Unity 场景单位/s，3 单位 = 1 m）—— 打车开火门用（见 decide_fire）
    chassis_speed_ = std::hypot(msg.state.twist.linear.x, msg.state.twist.linear.y);

    // 给底盘控制器留最新位姿/速度（全自动任务用；map FLU → Unity (x,z)）
    if (mission_on_) {
      st_ux_ = -msg.state.position.y;
      st_uz_ = msg.state.position.x;
      st_uvx_ = -msg.state.twist.linear.y;
      st_uvz_ = msg.state.twist.linear.x;
      st_stamp_ = std::chrono::steady_clock::now();
      st_seen_ = true;
    }

    // ---------------- 模式选择：这一帧打符还是打车 ----------------
    if (should_run_rune()) {
      rune_aim_step(msg, frame, yaw_now, pitch_now);
      return;
    }
    if (rune_started_) {           // 打符窗口结束：清标记，下次开符重新初始化
      rune_started_ = false;
      RCLCPP_INFO(get_logger(), "打符窗口结束 -> 回到车自瞄");
    }

    // ---------------- 检测 → PnP（都在相机系里做）----------------
    auto armors = detector_->detect(frame);
    armors.remove_if([&](const Armor & a) { return a.color != enemy_; });
    // ★ 任务期目标纪律（用户 2026-10-02 定）：非当前站目标的板**不进跟踪器**
    //   —— 连识别锁定都不给（不是锁了不打）。mask=0 的站（路过/打符）全部丢。
    //   改掉这一层要连带改下面 send_angles 前的兜底（换站瞬间的残留锁定）。
    if (mission_on_) {
      armors.remove_if([&](const Armor & a) {
        return mission_filter_mask_ == 0 ||
               ((1u << static_cast<int>(a.name)) & mission_filter_mask_) == 0;
      });
    }
    for (auto & a : armors) solver_->solve(a);

    // ---------------- 观测搬进世界系，再进 EKF ----------------
    // 这样云台怎么转，EKF 都在一个不动的系里工作；否则跟踪器会把云台自身的
    // 转动当成目标在动（尤其"指令发出、云台正在转过去"那一段）。
    // 注意 rvec/tvec/points 仍留在相机系 —— 画框和 PnP 要用，不能动。
    const Eigen::Matrix3d R_world_from_cam = world_from_cam(yaw_now, pitch_now);
    armors_to_world(armors, R_world_from_cam);
    solver_->set_frame_rotation(R_world_from_cam.transpose());  // 投影像素时转回去

    tracker_->update(armors, std::chrono::steady_clock::now());

    // ---- 处理帧率：到 EKF 更新完成为止（模型推理 + PnP + EKF 这一段）----////////////////////////////
    // 只统计【算法这段】，不含后面的可视化 —— 否则 show_window 时帧率会被
    // 显示拖低，看起来像"算法太慢"
    const auto t_ekf = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(t_ekf - t0).count();
    ema_ms_ = ema_ms_ * 0.9 + ms * 0.1;
    proc_times_.push_back(t_ekf);
    if (proc_times_.size() > kFpsWindow) proc_times_.pop_front();

    // ---------------- 状态机迁移 ----------------
    const bool seen = !armors.empty();                       // 本帧有没有检出敌方板
    const bool locked = tracker_->has_lock();
    const int locked_now = locked ? tracker_->locked_id() : -1;

    // 锁定换目标是个关键事件（换板/抢锁/丢锁），单独打一条，不用等那 30 帧
    if (locked_now != last_locked_) {
      aimer_->reset_lock();
      // 前哨站收敛计时起点（见下面的"收敛期先不开火"）：锁上/重锁都从 0 起算 //////////////////////////////////
      outpost_lock_t0_ = (locked_now == (int)outpost) ? steady_seconds() : -1e9;
      // 换板门状态也清零：换目标后第一次有效瞄准不算"换板"，不用白等 N 秒
      last_aim_plate_id_ = -1;
      plate_switch_t_ = -1e9;
      RCLCPP_INFO(get_logger(), "锁定切换: %s → %s",
                  last_locked_ < 0 ? "-" : ARMOR_NAMES[last_locked_].c_str(),
                  locked_now < 0 ? "-" : ARMOR_NAMES[locked_now].c_str());
    }
    last_locked_ = locked_now;

    // 前哨站：旋转方向（顺/逆）定下来的那一刻单独打一条，带票数 ——//////////////////////
    // 这个只在开局投一次，是值得单独看见的事件
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

    // 迁移规则见类顶部的状态机图。要点：
    //   · 进 kTracking 需要【连续看到 + 已锁定】两个条件同时满足 ——
    //     只看到（seen）不够，跟踪器自己也得认，防的是"检测抖了一下"
    //   · kDetecting 的"看到"和技术上的 locked 是两回事：前者是本帧有框，
    //     后者是 EKF 已经稳定跟上（丢 max_temp_lost_count 帧才解锁）
    // 弹量：打空了就进 kNoAmmo 停住不开火，等补给（规则 5.4）。
    // ★ 字段没被正确填充时的自保：如果游戏一直报 0，云台会永久停在"无弹"、
    //   连瞄准都不做 —— 所以开局头几帧先警告一次，免得当成算法坏了去查半天。
    const bool ammo_empty = msg.state.ammo_remaining <= min_ammo_to_fire_;
    if (ammo_empty && !warned_ammo_ && processed_ < 10) {
      RCLCPP_WARN(get_logger(),
                  "ammo_remaining=%d 判为空弹 → 进入无弹状态。若游戏侧本就填 0，"
                  "请把 config 的 min_ammo_to_fire 设为负数关掉这个门",
                  msg.state.ammo_remaining);
      warned_ammo_ = true;
    }

    const AimState prev_state = state_;
    // ① 弹空 —— 最高优先级，从任何瞄准状态都要让位
    if (ammo_empty && state_ != AimState::kNoAmmo) {
      state_ = AimState::kNoAmmo;
      lost_ = 0;
      detect_count_ = 0;
    } else switch (state_) {
      case AimState::kSearch:                 // 转着找
        if (seen)       { state_ = AimState::kDetecting; detect_count_ = 1; lost_ = 0; }
        break;

      case AimState::kDetecting:              // 看到了，先停住确认几帧
        if (seen) {
          lost_ = 0;
          // 连续 min_detect_count_ 帧都看到 + 跟踪器锁上了，才敢进跟踪//////////////////
          if (++detect_count_ >= min_detect_count_ && locked) {
            state_ = AimState::kTracking;
            detect_count_ = 0;
          }
        } else if (++lost_ >= kDetectGiveUp) {
          state_ = AimState::kSearch;
          lost_ = 0;
        }
        break;

      case AimState::kTracking:               // 锁着，正常打
        if (!locked)    { state_ = AimState::kTempLost;  lost_ = 0; }
        break;

      case AimState::kTempLost:               // 丢了，先别放弃
        if (locked)     { state_ = AimState::kTracking;  lost_ = 0; }
        else if (++lost_ >= max_temp_lost_) { state_ = AimState::kSearch; lost_ = 0; }
        break;

      case AimState::kNoAmmo:                 // 弹空停住；弹药一回来就恢复
        if (!ammo_empty) {
          // 还锁着 → 直接回跟踪接着打；锁已经丢了 → 回搜索重新找
          // （无弹期间云台不动、也没在瞄准，锁过一会儿就会被 Tracker 解掉）
          state_ = locked ? AimState::kTracking : AimState::kSearch;
          lost_ = 0;
        }
        break;
    }
    // 状态变化单独打一条日志 —— 排查"为什么云台不动""为什么不开枪"时，
    // 第一件事就是看它卡在哪个状态
    if (state_ != prev_state)
      RCLCPP_INFO(get_logger(), "状态: %s → %s", state_name(prev_state), state_name(state_));

    // ---------------- 解算 ----------------
    // 初速：规则附录 B 说这是"最近成功一发的实际初速"，没出过弹可能是 0
    const double v0 = msg.state.projectile_speed;
    if (v0 > 20.0 && v0 < 30.0) aimer_->set_bullet_speed(v0);

    // 第一帧把"上一条指令"对齐到云台实际位置，免得搜索/保持状态一上来就甩云台
    if (!started_) {
      last_yaw_ = yaw_now;
      last_pitch_ = pitch_now;
      started_ = true;
    }

    // 各状态的默认动作 = 保持上一条指令不动（kDetecting / kTempLost / kNoAmmo
    // 都是这样，它们不满足下面任何一个 if，于是 world_yaw/world_pitch 原样保留）。
    // ★ "保持不动"不是什么都不发，而是【重发上一条指令】—— 游戏侧有 0.5 s
    //   指令超时，不发的话云台会被游戏清成"无指令"状态。
    //   这也是为什么下面无论走哪个分支，最后都会走到 send_angles()
    double world_yaw = last_yaw_, world_pitch = last_pitch_;
    bool shoot = false;
    Aimer::AimResult aim;

    // 距上一帧的实际时间：搜索转速按它积分（顺带限幅，防卡顿后一次转一大截）/////////////////////////////
    const auto now_tp = std::chrono::steady_clock::now();
    const double dt = std::clamp(
        std::chrono::duration<double>(now_tp - last_frame_t_).count(), 0.0, 0.5);
    last_frame_t_ = now_tp;

    aiming_base_ = false;      // 本帧是不是在直瞄基地，日志用
    last_fire_block_[0] = '\0';   // 开火否决原因（decide_fire 里填，日志用）

    // 下面三个分支就是状态机的"动作"部分（上面那段 switch 只管迁移）。
    // 每个状态在这里决定这一帧要发给游戏什么角度、允不允许开火。
    if (state_ == AimState::kSearch) {
      // 任务提示优先：按住任务要求的角（对准敌方/前哨/基地，见 /mission/cmd），
      // 提示超时或没有提示才走原来的搜索。
      double my = 0.0, mp = 0.0;
      if (mission_hold(my, mp)) {
        world_yaw = my;
        world_pitch = mp;
      } else {
        // 匀速转着扫 + 俯仰余弦扫描（和打符模式共用一套，见 search_step）：
        // 从【上次发出的角】继续转（不是从实际角 —— 用实际角会在云台还没转到位
        // 时"跳"），俯仰在 [min,max] 之间平滑地抬上去再放下来，扫描架子上的符
        search_step(dt, world_yaw, world_pitch);
      }
    } else if (state_ == AimState::kTracking &&
               tracker_->locked_id() == static_cast<int>(base)) {
      // 【基地】锁定照旧（它在跟踪器里就是个普通目标，框和锁定圈照画），
      // 但【瞄准这一步不用 EKF、不做预测】—— 基地是静止结构，直接拿这一帧
      // 观测到的板中心解弹道 + 重力补偿就完了。
      // 顺带一个好处：基地的 EKF 发散也不影响开火（以前会被 !diverged 挡死）。
      const Armor * plate = pick_base_plate(armors, frame.size());
      aiming_base_ = (plate != nullptr);
      if (plate) {
        // ★ 换系：armors 里的 xyz_in_camera 是【相机排版】(右/下/前)，
        //   而 best_aim_direct 要的是【状态系】(右/前/上，也就是世界系)，
        //   两者差一次轴重排：pseudo = (x, z, -y)。
        //   漏了这一步会把"前"当成"上"—— 弹道解出一支近乎垂直的解、
        //   抬高点也加到错的分量上，准星和角度会一起飞掉。
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
          // 换板检测（只给车）：板号一变就记时刻，decide_fire 里用它挡
          // 换板瞬态。前哨站不适用——它的 argmin/argmax 没有迟滞，两块板
          // 等距时可能来回切，会让"换板等待"门一直关着。
          if (!it->second.is_outpost_ &&
              last_aim_plate_id_ >= 0 && aim.plate_id != last_aim_plate_id_)
            plate_switch_t_ = steady_seconds();
          last_aim_plate_id_ = aim.plate_id;

          // tracker/aimer 的坐标现在都在世界系，所以 aim_3d 是世界系的点，
          // 直接取方位角就是游戏的 yaw —— 不用再加云台当前角（姿态已经烘进
          // 观测旋转里了）。
          //   aim_3d = (右, 前, 上)；游戏 yaw 是 Unity +Z 为 0、顺时针为正，
          //   也就是从"前"轴量起，所以是 atan2(右, 前)（别写成 atan2(前, 右)）。
          world_yaw = rad2deg(std::atan2(aim.aim_3d(0), aim.aim_3d(1)));

          // aim.pitch 是世界系里的发射仰角（向上为正），游戏要向下为正。
          world_pitch = -rad2deg(aim.pitch);

          // ★ 前哨站收敛期先不开火（用户 2026-10-04 定："这个收敛时间先不打"）：
          //   锁定后的前 outpost_settle_s 秒（144°/s 下 4s ≈ 转 1.6 圈）里，三块板的
          //   高度差 dz1/dz2 还在从观测里学（模型只能让 z 逐块贴，瞄高会差 0.1~0.2m），
          //   这几秒打了也白打。只挡开火、不挡瞄准——云台照常跟，学稳了立刻放行。
          const bool outpost_settling =
              it->second.is_outpost_ &&
              (steady_seconds() - outpost_lock_t0_) < outpost_settle_s_;
          if (outpost_settling) {
            shoot = false;
            std::snprintf(last_fire_block_, sizeof(last_fire_block_),
                          "前哨站收敛中(%.1f/%.0fs)",
                          steady_seconds() - outpost_lock_t0_, outpost_settle_s_);
          } else {
            // 开火决策（照 sp_vision_25 的 Shooter，见 decide_fire）；
            // 末参 = 弹道迭代有没有收敛（没收敛不开火，2026-10-04）
            const Eigen::VectorXd & x = it->second.ekf_.x;   // 状态：右/前/上
            shoot = decide_fire(world_yaw, yaw_now,
                                std::sqrt(x(0) * x(0) + x(2) * x(2)), aim.converged);
          }
        }
      }
    }

    // ★ 任务期目标纪律：只打当前站目标；锁到别的也一枪不打、云台继续对着
    //   待击打目标（提示角）。不在任务里（mission_on_=false）完全不影响原行为。//////////////////////////
    if (mission_on_) {
      const bool is_target = tracker_->has_lock() && mission_filter_mask_ != 0 &&
                             ((1u << tracker_->locked_id()) & mission_filter_mask_) != 0;
      if (!is_target) {
        shoot = false;
        double my = 0.0, mp = 0.0;
        if (mission_hold(my, mp)) {          // 提示新鲜才夺云台（超时自动让回）
          world_yaw = my;
          world_pitch = mp;
        }
      }
    }

    send_angles(world_yaw, world_pitch, shoot);

    if (show_window_) {
      visualizer_->draw(frame, armors, *tracker_, frame_id_,
                        aim.valid ? &aim : nullptr,
                        !aiming_base_);      // 直瞄基地时不画 EKF 预测箭头
      cv::imshow("auto_aim", frame);
      if ((char)cv::waitKey(1) == 27) rclcpp::shutdown();
    }
    ++frame_id_;

    // 每 30 帧打一条：帧率 + 锁定的车 + 本帧观测到的车。
    //   帧率：处理 vs 采集接近且稳定 = 没有积压；采集 > 处理 = 每帧都在丢、
    //         实际算的是旧图，提前量会偏（该降分辨率/降帧率或换小模型）。
    //   锁定：名字 + 车中心距离 + 角速度（小陀螺能到几百 °/s，一眼看得出）
    //   观测：名字/颜色/置信度/距离/大小板 —— 和锁定对不上就是关联出问题了
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
        lock = "base(直瞄)";           // 基地不走跟踪器，锁的是"这一帧看到的那块板"
      } else if (tracker_->has_lock()) {
        auto it = tracker_->tracks().find(tracker_->locked_id());
        if (it != tracker_->tracks().end()) {
          const Eigen::VectorXd & x = it->second.ekf_.x;
          const double d = std::sqrt(x(0) * x(0) + x(2) * x(2) + x(4) * x(4));
          char b[128];
          // 车模型：r/dr 是发散判据看的量；前哨站是另一套状态（无 r），
          // 打 dz1/dz2 和旋转方向（0=没定/+1 逆/-1 顺）
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

      // 开火一栏带上否决原因（超距/收敛中/指令突变/云台未到位）—— 排查
      // "瞄准很稳却一枪不放"时直接看这里
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

  // ==========================================================================
  // 开火决策
  //
  //   ① 指令不突变：本帧指令 vs 上一条指令的 yaw 差 < 2×容差
  //   ② 云台已到位：云台实际 yaw vs 【上一条】指令的 yaw 差 < 容差
  //      （比上一条而不是本帧 —— 指令刚发出去云台还在转，比本帧会假到位）
  //   ③ 瞄准点有效（调用方已经判过 aim.valid 才会进来）
  //   ④ 距离限制：水平距离 <= max_fire_distance_（我们自己加的，不是 sp_vision 的）
  //
  //   容差按距离分档：远的更严（和 sp_vision 的配置一致：近 3°、远 2°）。
  //   prev_cmd_yaw_ 每帧都更新（不管开不开火），所以 ① 实际是"连续两帧指令一致"；
  //   丢锁时不会走到这里，于是重新锁上要重新等稳定，不会一锁上就开枪。
  //
  //   注意它只看 yaw 不看 pitch —— 游戏侧还有 1.2° 的伺服到位门兜底。
  // ==========================================================================
  bool decide_fire(double cmd_yaw, double gimbal_yaw, double horizon_dist, bool aim_converged) {
    last_fire_block_[0] = '\0';
    if (!auto_fire_) { std::snprintf(last_fire_block_, sizeof(last_fire_block_), "未开自动开火"); return false; }

    // ★ 底盘没停稳不开火（用户 2026-10-04 定："打车的时候停下来再打"）。
    //   车在动时游戏弹道会带上底盘速度/车身晃动，移动中开火实机全散。
    //   阈值 fire_max_chassis_speed（Unity 单位/s，3 单位 = 1 m；<=0 关掉这道门）。
    //   ★ 2026-10-05 用户"还是先不要停车再打"→ config 已设 0 关闭此门，代码保留。
    //   注意这只挡【开火】，云台照常跟——停稳的那一刻天然满足其它门，马上就能打。
    if (fire_max_chassis_speed_ > 0.0 && chassis_speed_ > fire_max_chassis_speed_) {
      std::snprintf(last_fire_block_, sizeof(last_fire_block_), "车在动(%.1f等停稳)",
                    chassis_speed_);
      return false;
    }

    const double tol = (horizon_dist > judge_dist_) ? second_tol_ : first_tol_;

    const bool no_jump   = have_prev_cmd_ && std::abs(prev_cmd_yaw_ - cmd_yaw) < tol * 2.0;
    const bool on_target = have_prev_cmd_ && std::abs(gimbal_yaw - prev_cmd_yaw_) < tol;

    // ★ 这句必须在距离门【前面】：prev_cmd_yaw_ 要代表"最近一次决定发出的指令"，
    //   而超距期间云台仍在跟目标（指令照发，只是不给射击许可），所以那段时间的
    //   指令也要记进来。否则目标从远处回到 4m 内时，① 会拿一个几秒前的陈旧值
    //   去比，可能平白多卡几帧才允许开枪。
    prev_cmd_yaw_ = cmd_yaw;     // 无论开不开火都要更新（sp_vision 的 last_command_）
    have_prev_cmd_ = true;

    // ④ 距离限制：太远就不给射击许可。放在最后是因为它是唯一一条
    //   【与指令历史无关】的判据 —— 打不着就是打不着。
    //   max_fire_distance_ <= 0 表示不限距离（临时关掉这个门用）
    if (max_fire_distance_ > 0.0 && horizon_dist > max_fire_distance_) {
      std::snprintf(last_fire_block_, sizeof(last_fire_block_), "超距");
      return false;
    }

    // ⑤ 弹道收敛（2026-10-04 用户要求"收敛了再开火"）：不动点迭代没收敛时，
    //   aim 返回的是最后一次迭代值 —— 瞄准照发，但这一枪不开（目标机动猛 /
    //   板正在切换，那个角本来就不可信）。
    if (!aim_converged) {
      std::snprintf(last_fire_block_, sizeof(last_fire_block_), "收敛中");
      return false;
    }

    // ⑥ 换板后等待：aim.plate_id 刚变过，指令才从旧板跳到新板。
    //   只挡开火、不挡瞄准；N 秒后云台已到位，再按下面的 ①/② 判。
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

  // ==========================================================================
  // 内参（7.1）：优先同帧 camera_info；无效才回落表 12 标称值
  // ==========================================================================
  // 每帧设一次内参（Solver::set_intrinsics 只是赋值，开销可忽略）。
  // 判据用 k[0]（fx）> 0 —— 全 0 的 camera_info 是"没填"的常见表现，
  // 而 fx=0 会让 PnP 直接除零
  void apply_intrinsics(const sensor_msgs::msg::CameraInfo & ci, const cv::Size & sz) {
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat D = cv::Mat::zeros(1, 5, CV_64F);   // 接口是无畸变针孔模型

    if (ci.width > 0 && ci.height > 0 && ci.k[0] > 0.0) {
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) K.at<double>(r, c) = ci.k[r * 3 + c];
      if (!intr_ready_)
        RCLCPP_INFO(get_logger(), "内参来自 camera_info: fx=%.3f fy=%.3f cx=%.3f cy=%.3f (%ux%u)",
                    ci.k[0], ci.k[4], ci.k[2], ci.k[5], ci.width, ci.height);
      intr_ready_ = true;
    } else {
      // 回落值是按 960×720 给的；图像尺寸不同就按比例缩放，否则 PnP 比例会错
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
    // 打符（RP26 对接层）用同一份内参（规则 7.1：必须同帧 camera_info），
    // 每帧在 rune_aim_step 里作为参数传进 process()
    intr_K_ = K;
    intr_D_ = D;
  }

  // ==========================================================================
  // 打符（kRune 模式）—— 流程和 Python 版 tdt-campus-test 一致：
  //   检测(26 模型) → 选片(picker) → PnP(R 标) → 7/10 维 EKF → 判稳+confirm 开火
  //   没解出就搜索（共用俯仰扫描）并停火。
  // 状态机复用 kSearch/kTracking/kTempLost —— 目标换成"符"而已。
  // ==========================================================================

  static double steady_seconds() {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  // 这一帧该不该走打符：config 强制优先，否则任务要求，
  // 或自动叫符 accepted 后仍在单次开符窗口内。
  // ★ 收到 /game/rune/result 的 activated=true（增益已激活 = 这轮符打完了）
  //   就立刻退出，不等窗口耗完。
  bool should_run_rune() const {
    if (!rune_enabled_) return false;
    if (mission_rune_) return true;          // 任务提示要求打符
    if (mode_forced_) return forced_mode_ == Mode::kRune;
    if (rune_finished_) return false;
    return rune_counter_.active(steady_seconds());
  }

  // /game/rune/result：accepted=受理、activated=增益激活（这一轮打完）。
  // 收到 activated 就立即切回车自瞄。
  void on_rune_result(tdt_interface::msg::RuneResult::SharedPtr msg) {
    if (msg->activated) {
      // ★ 本站必须"叫过符"才认这条 activated：
      //   叫符 = 自动叫符被受理（accepted），计入 rune_counter_。
      //   ★ 2026-10-05 用户定：不再仅凭
      //   "任务要求打符（mission_rune_）"就认 activated —— 必须 accepted 在前。
      //   防的是：上一轮的 activated 迟到 / 游戏哪天改成重发或"状态式"发，
      //   恰好落在进了下一站之后 —— 会把还没叫符的新站误判成"已打过"、直接跳过。
      //   （忽略时打 WARN：顺便当"游戏会不会重发 activated"的探针。）
      const bool engaged = rune_req_ok_.load() ||
                           rune_counter_.count() > rune_count_at_stage_entry_;
      if (!engaged) {
        RCLCPP_WARN(get_logger(),
                    "收到 activated=true 但本站还没叫过符（request_id=%lu）→ 忽略"
                    "（上一轮的迟到/重发？）",
                    static_cast<unsigned long>(msg->request_id));
        return;
      }
      mission_rune_done_ = true;              // 任务判据：符这一站可以过了
      RCLCPP_INFO(get_logger(),
                  "★ 增益已激活（request_id=%lu）—— 这轮符打完了，立即切回车自瞄",
                  static_cast<unsigned long>(msg->request_id));
      rune_finished_ = true;
      // RP26 这边不用收尾：下一次开符的间隔远超它的时间窗（prepare_time/
      // max_life_time），内部状态会自动重建。
    } else if (msg->accepted) {
      // ★ 2026-10-05：ROS 叫符被受理 = 开启本轮符。
      //   只认当前正在等的那条（request_id 对上 rune_req_last_id_）：开符次数
      //   由这里 +1 —— 它同时管两件事：第 3 次起按大符打（rune_aim_step 的 big）、
      //   以及"本站叫过符"的 engaged 判据（rune_counter_ 计数上涨）。
      if (msg->request_id == rune_req_last_accepted_id_) return;   // 同一 id 重发：防重复计数
      if (msg->request_id != rune_req_last_id_.load()) {
        RCLCPP_WARN(get_logger(),
                    "收到 accepted=true 但不是当前请求（id=%lu，当前 %lu）→ 忽略",
                    static_cast<unsigned long>(msg->request_id),
                    static_cast<unsigned long>(rune_req_last_id_.load()));
        return;
      }
      rune_req_last_accepted_id_ = msg->request_id;
      rune_req_ok_.store(true);               // 叫符循环就此停下
      rune_finished_ = false;                 // 同一次开符的收尾（清上一轮“已激活”标记）
      const int c = rune_counter_.mark_opened(steady_seconds());
      RCLCPP_INFO(get_logger(),
                  "★ 叫符成功（request_id=%lu）—— 进入打符模式（本局第 %d 次开符）",
                  static_cast<unsigned long>(msg->request_id), c);
    } else {
      // 被拒：循环每 rune_req_retry_ms 还会再发；日志限流到 5s 一条
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

  // ---- 任务提示（/mission/cmd，来自 mission_node.py）----
  //   "hold <yaw> <pitch> [rune] [big]"：任务要求的云台角（kSearch 时按住；见到
  //      目标进入跟踪/开火后自然让位）。超时 2 s 自动失效 —— 任务节点崩了也不会
  //      把云台卡在提示角上。★ 手控底盘调符：用 tools/rune_hold.py 持续重发这条，
  //      云台就"暂时固定"在打符预设角上。
  //   "rune" 尾标记 = 任务要求开打符模式（activated 后自动退回）。
  //   ★ "big" 尾标记 = 强制按大符打（隐含 rune）——手控底盘调大符用，不必先真开
  //      两次小符把"第 3 次起大符"的计数顶上去。每次 hold 重算，不带就退回计数口径。
  //   "auto"：交回自动搜索。
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
      // ★ 2026-10-05 扩展：尾标记可带 "rune" 和/或 "big"（顺序随意）。
      //   "big" 隐含 rune（不写 rune 也进打符模式）。每次 hold 都重置 ——
      //   要退回"小符/按计数"的口径，发一条不带 big 的 hold 即可。
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

  // ==========================================================================
  // 全自动任务（底盘 + 站点推进）—— 和 Python 的 mission_node.py/nav_node.py 同口径。
  //   参数：mission:=true 启用；mission_vmax 限速；站点表 mission_stages.json
  //   （yaml-cpp 直接吃 JSON）。底盘指令走 /target_speed_player_N，单位=Unity
  //   世界 X/Z 目标速度（1:1，场景单位/s）。publish_angles=false 时连底盘也不动。
  // ==========================================================================
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
        if (s["wait"]) st.wait = s["wait"].as<double>();   // 到点后再停留 N 秒（补弹等）
        if (s["stop"]) st.stop = s["stop"].as<double>();   // 距目标多远停车打（0=默认 4.5）
        if (s["fixed"]) st.fixed = s["fixed"].as<bool>();  // 定点打：不跟实时位置，到点停车打
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
      // 文件不存在 = 还没学过，正常
    }
  }

  // ★ 2026-10-05 起不再调用（幽灵块教训：只在当局内存标障碍，不落盘）。
  //   保留函数以便哪天要恢复"跨局学障碍"。
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
      // 解析不动就忽略这一帧
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
    dwell_t0_ = {};                            // 停留计时（arrive+wait 的站用）
    rune_arrive_t0_ = {};                      // 打符站"到点"计时（45 s 超时用）
    rune_req_active_ = false;                  // 自动叫符循环（到点才开始）
    rune_req_ok_ = false;
    rune_req_last_id_ = 0;
    live_probe_done_ = false;                  // 实时目标可达性缓存（每站重探）
    standoff_reachable_ = false;
    plan_fail_t0_ = {};                        // 规划失败重试计时
    last_plan_warn_s_ = 0.0;
    rune_count_at_stage_entry_ = rune_counter_.count();   // "本站叫过符"判据的快照
    // ★ 目标纪律（用户 2026-10-02 定）：任务期只打当前站指定目标，锁到别的
    //   也一枪不打、云台继续对着待击打目标。mask=0 → 这一站谁都不许打。
    //   映射：2工程=two / 3=three / 4=four / 5哨兵=sentry(也兼容 five) /
    //   6前哨=outpost / 7基地=base
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
      mission_stamp_ = -1e9;                 // 交回自动搜索
      mission_rune_ = false;
    } else {
      push_aim(true);
    }
  }

  bool check_condition() const {
    const Stage & st = stages_[stage_i_];
    if (st.done == "rune") return mission_rune_done_;
    if (st.done == "hp") {
      auto it = enemy_hp_.find(st.number);
      if (it != enemy_hp_.end() && it->second <= 0) return true;
      return enemy_seen_.count(st.number) > 0 && enemy_hp_.count(st.number) == 0;
    }
    return false;                              // "arrive" 到点就推进
  }

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
      if (it == enemy_pos_.end()) return;      // 还不知道它在哪，先不夺云台
      std::lock_guard<std::mutex> lk(mission_mu_);
      mission_yaw_ = rad2deg(std::atan2(it->second.first - st_ux_, it->second.second - st_uz_));
      mission_pitch_ = st.aim_pitch;
      mission_rune_ = false;
      mission_stamp_ = steady_seconds();
    }
    last_aim_pub_ = now;
  }

  void advance_stage() {
    if (stage_i_ + 1 >= static_cast<int>(stages_.size())) {
      mission_finished_ = true;
      publish_speed(0, 0);
      RCLCPP_INFO(get_logger(), "★ 全任务走完");
      return;
    }
    // ★ 先 enter_stage 再设停顿：enter_stage 会清 pause_until_（进站初始化的一部分），
    //   反过来写的话这个 0.8 s 站间停顿会被立刻清掉、等于没有（旧写法就是如此）。
    enter_stage(stage_i_ + 1);
    pause_until_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(800);
  }

  void publish_speed(double vx, double vz) {
    cmd_vx_ = vx;
    cmd_vz_ = vz;
    if (!speed_pub_ || !publish_angles_) return;   // 观察模式：连底盘也不动
    geometry_msgs::msg::TwistStamped m;
    m.header.stamp = now();
    m.twist.linear.x = vx;
    m.twist.linear.y = vz;
    speed_pub_->publish(m);
  }

  // pure pursuit 前瞻点：先把车投影到路径线上，再从投影点沿路径前进 kLookahead。
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
      publish_speed(0, 0);                     // 断流：停车等恢复
      return;
    }
    if (mission_finished_) { publish_speed(0, 0); return; }
    if (prewarm_t0_.time_since_epoch().count() == 0) prewarm_t0_ = now_tp;
    if (std::chrono::duration<double>(now_tp - prewarm_t0_).count() < 2.0) {
      publish_speed(0, 0);                     // DDS 预热（首发指令会被匹配延迟吃掉）
      return;
    }
    push_aim();
    service_rune_requests();     // ★ 到打符位后的自动叫符（内部有 active/间隔门）
    if (now_tp < pause_until_) { publish_speed(0, 0); return; }

    if (!path_valid_) {
      auto p = grid_.plan(st_ux_, st_uz_, goal_.first, goal_.second, 0.4);
      if (!p) {
        // ★ 规划失败【不判死】：先停车自救再重试，持续失败 kPlanGiveUp 秒才放弃。
        //   第一次失败时把车自己周围一圈放开 —— 可能被"学到的障碍"块围死
        //   （和 Python 版 nav_node.py 的自救同口径）。
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

    // ---- hp 站【实时规划】：行驶目标点跟低频实时位置走（1 Hz 的 /navigation/enemies）----
    //   目标移动 > kReplanDist → 换目标点重规划；进射程 kStopDist → 停车打；
    //   跑出 kResumeDist → 再追。拿不到实时位置（还没广播）就回落配置的活动区域点。
    if (st.done == "hp" && st.fixed) {
      // ★ 定点打：不跟实时位置、不按 stop 圈停车 —— 就开到配置点，到点由下面的
      //   "已到"分支停住打到条件满足。路上目标就被打掉了也别白跑一趟，直接推进。
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
        // 每站可覆盖停车距离（静目标如前哨站要站远点：st.stop）；迟滞 1.5 单位
        const double stop_d = st.stop > 0.0 ? st.stop : kStopDist;
        if (std::hypot(live.first - goal_.first, live.second - goal_.second) > kReplanDist) {
          // 目标挪了：跟实时位置 —— 但【只跟真能规划到的点】。
          // 本体坐标可能不可驶入：基地/前哨站的本体在围墙结构里，A* 无路
          // （实测第 10 站拿基地本体当目标当场"没有通路"把任务判死）。
          // 这种情况改为开到【它前方 stop_d 单位的射击位】：基地本来就过不去，
          // 到它跟前 4.5 单位停车打就是需求本身；射击位 = 本体朝车这一侧退 stop_d。
          // ★ 同一个位置只试探一次：基地一动不动，不能每拍跑一次 A* 刷日志。
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
              if (n > 1e-6) { dx /= n; dz /= n; } else { dx = 0.0; dz = 1.0; }  // 重合兜底
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
            waiting_ = false;                  // 跑出射程：继续追
            RCLCPP_INFO(get_logger(), "%d 号跑出射程（%.1f）→ 追", st.number, d_live);
          } else {
            publish_speed(0, 0);               // 射程内：停下打
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
          return;                              // 停留中：保持零速
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
      // ★ 打符站停留上限：到点起最多待 kRuneStageTimeout 秒，无论激活没激活都推进。
      //   （激活了走上面 check_condition 那条，立刻走；这里管"一直没激活"的兜底。）
      if (st.done == "rune") {
        if (rune_arrive_t0_.time_since_epoch().count() == 0) {
          rune_arrive_t0_ = now_tp;
          // ★ 2026-10-05：到打符位即开始 ROS 叫符（不再手动开符）。下一拍发第一条
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

    // 卡死检测：有命令但没动（撞墙/掉坑数据上一样）；块心沿指令方向前移，防围死
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
          // ★ 2026-10-05 用户定：只在【当局内存】标障碍（下一拍重规划绕开），
          //   不再写盘 —— "幽灵块"（卡死误判种下的假障碍）会永久污染后续所有
          //   任务，宁可每局重新学。learned_/save_learned 保留但不再调用。
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

    // pure pursuit：速度 = vmax 封顶，方向朝前瞻点（默认全速、不减速）
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

  // ==========================================================================
  // ★ 2026-10-05 新：到打符位后的 ROS 叫符（替代手动开符）
  //
  // 到点后每 rune_req_retry_ms 往 /game/rune/request 发一条 RuneRequest，直到
  // 游戏回 accepted=true（受理并开启本轮符）。受理在
  // on_rune_result 里处理：开符次数 +1（管大/小符轮次）、窗开、本循环停。
  // 离开打符站（换站/换局）由 enter_stage 复位，循环自然结束。
  // request_id：ctor 里按毫秒时间戳播种，之后逐条 +1（非零且单调）。
  // ==========================================================================
  void service_rune_requests() {
    if (!rune_req_active_ || rune_req_ok_.load() || !pub_rune_req_) return;

    const auto now_tp = std::chrono::steady_clock::now();
    if (now_tp < rune_req_next_t_) return;
    rune_req_next_t_ = now_tp + std::chrono::milliseconds(rune_req_retry_ms_);

    tdt_interface::msg::RuneRequest req;
    req.header.stamp = now();                 // 请求创建时间（UTC）
    req.request_id = ++rune_req_id_;          // 非零且单调
    rune_req_last_id_.store(req.request_id);
    pub_rune_req_->publish(req);
    RCLCPP_INFO(get_logger(), "叫符: 已发 RuneRequest #%lu（%.1fs 没受理就重发）",
                static_cast<unsigned long>(req.request_id), rune_req_retry_ms_ / 1000.0);
  }

  // 搜索（两种模式共用）：yaw 匀速转 + 俯仰按累计转角的余弦"抬上去再放下来"。
  // 能量机关装在架子上，平视经常扫不到 —— 只往上扫、不看地面（负 = 往上看）。
  // 和 Python 版一致：不是"转一圈跳一档"，是连续的螺旋式扫天。
  void search_step(double dt, double & yaw, double & pitch) {
    const double step = search_rate_ * dt;
    search_travel_ += step;
    yaw = wrap180(last_yaw_ + step);   // 从上次【发出的】角继续转（不是实际角）
    const double lo = search_pitch_min_, hi = search_pitch_max_;
    pitch = lo + (hi - lo) *
            (1.0 - std::cos(search_pitch_cycles_ * deg2rad(search_travel_))) / 2.0;
  }

  // 游戏给的 projectile_speed 是"场景单位/s"（3 单位 = 1 m），弹道要用 m/s。
  // 按量级自动判（和 Python 版同款）：15–35 当 m/s；45–105 当场景单位 ÷3；
  // 别的情况退回 23.5（规则表 20 的标称值）并告警一次。
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
    // 第 3 次起是大符（规则：须先用完 2 次小符机会才能开大符）。
    // ★ 2026-10-05：/mission/cmd 的 "big" 标记可强制按大符打 —— 手控底盘调大符时
    //   不用真开两次小符把计数顶上去（见 tools/rune_hold.py）。
    const bool big = mission_rune_big_.load() ||
                     rune_counter_.count() > max_small_runes_;

    if (!rune_started_) {
      search_travel_ = 0.0;
      rune_lost_ = 0;
      rune_seen_ = false;            // 新的一次开符：从"没跟到过"开始
      rune_started_ = true;
      RCLCPP_INFO(get_logger(), "-> 打符模式（RP26 链路：%s，第 %d 次开符）",
                  big ? "大符" : "小符", rune_counter_.count());
    }

    // ---------------- 检测 → RP-26Rune 全套 ----------------
    // 检测仍是我们的 RuneDetector（0624 模型，和 RP26 原版同一个）；后面的
    // 轮廓精修/位姿重建/相位与运动估计/火控弹道由 rp26 库一次调用串完。
    auto blades = rune_detector_->detect(frame);
    int n_cls0 = 0, n_cls1 = 0;
    for (const auto & b : blades) {
      if (b.class_id == 0) n_cls0++;
      else if (b.class_id == 1) n_cls1++;
    }

    // 云台姿态 → 世界旋转。RP26 的 car 系 = 世界对齐的 (右/下/前)，正是
    // world_from_cam 的语义（离线对拍工具传的是同一个函数，见 rune_rp26.hpp
    // 顶部的坐标系说明）。
    const Eigen::Matrix3d R_world_from_cam = world_from_cam(yaw_now, pitch_now);
    const uint64_t stamp = static_cast<uint64_t>(stamp_ns(msg.header));
    const double v0 = rune_bullet_speed(msg.state.projectile_speed);

    RuneRp26::Result res;
    try {
      res = rune_rp26_->process(frame, blades, big, intr_K_, intr_D_,
                                R_world_from_cam, stamp, rune_my_color_, v0);
    } catch (const std::exception & e) {
      // 移植代码里有几处 throw（配置错/内部状态异常）——兜住，绝不让节点崩：
      // 这一帧按"没找到"处理，下一次调用照常。
      if (!rune_warned_exc_) {
        rune_warned_exc_ = true;
        RCLCPP_ERROR(get_logger(),
                     "RP-26Rune 抛异常（已兜住，本帧按未找到处理）：%s", e.what());
      }
    }

    // ---------------- 瞄 + 开火 ----------------
    double game_yaw = last_yaw_, game_pitch = last_pitch_;
    bool shoot = false;
    const char * block = "搜索";
    const char * state_txt = "搜索";

    if (res.found) {
      rune_lost_ = 0;                // 跟住就把"暂丢"计数清了
      rune_seen_ = true;             // 这次开符跟到过符：之后丢检只保持、不搜索
      // RP26 输出：yaw 已是"游戏约定"的方位角（弧度，直接转度就行）；
      // pitch 是抬头为正的发射仰角 → 游戏 pitch 向下为正，取负。
      game_yaw = rad2deg(res.yaw);
      game_pitch = -rad2deg(res.pitch);
      shoot = res.fire && auto_fire_;
      block = res.fire ? "" : "RP26未放行";
      state_txt = "跟随";
    } else {
      // 没跟住：先"暂丢"（冻结云台等它回来），连续丢够才退回搜索 —— 与老链路
      // 同一套节点级行为（RP26 内部有自己的超时/重建，不用我们操心）。
      //
      // ★ 暂丢这一态必须保留：打中一片后它要从检出里消失几帧、换下一片时检出
      //   也常闪断 —— 这几帧直接跳搜索的话云台当场扫走，下一片出来已经指偏。
      const auto now_tp = std::chrono::steady_clock::now();
      const double dt = std::clamp(
          std::chrono::duration<double>(now_tp - last_frame_t_).count(), 0.0, 0.5);
      if (!rune_seen_ && ++rune_lost_ > rune_lost_tolerance_) {
        double my = 0.0, mp = 0.0;
        if (mission_hold(my, mp)) {          // 任务提示：先按住任务给的正对符角
          game_yaw = my;
          game_pitch = mp;
        } else {
          search_step(dt, game_yaw, game_pitch);
        }
        block = "搜索";
        state_txt = "搜索";
      } else {
        game_yaw = last_yaw_;        // 保持上一条指令不动（游戏 0.5s 超时，要重发）
        game_pitch = last_pitch_;
        block = "等目标回来";
        state_txt = "暂丢";
      }
    }
    last_frame_t_ = std::chrono::steady_clock::now();

    send_angles(game_yaw, game_pitch, shoot);

    // 逐帧调试（RUNE_DEBUG=1 时开；排查"打几片突然不打"——冷却被谁重置/饿死）。
    // 格式：#RDBG 采集戳 found fire 指令yaw 指令pitch | tvalid phase omega
    //        rc(3) | ac(3) | 冷却剩余 连续开火累计 放行位 | 各叶[hubx huby cx cy cls]
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
                  "打符[RP26 %s 第%d次] cls0=%d cls1=%d | found=%d fire=%d "
                  "指令 yaw=%.2f pitch=%.2f | v0=%.1fm/s | %s",
                  big ? "大符" : "小符", rune_counter_.count(), n_cls0, n_cls1,
                  (int)res.found, (int)res.fire, game_yaw, game_pitch, v0, block);
    }
  }

  // 打符窗口：检测框（灰=未激活/橙=小符亮灯/绿=大符亮灯）+ HUD。
  // （RP26 链路不往外吐 3D 瞄点，所以不再画"预测瞄点十字"，只画检出和状态。）
  void draw_rune(const cv::Mat & frame, const std::list<RuneBlade> & blades, bool big,
                 int n_cls0, int n_cls1, const char * state_txt, const char * block,
                 bool found, bool fire) {
    for (const auto & b : blades) {
      cv::Scalar col(160, 160, 160);                       // cls0 未激活
      if (b.class_id == 1) col = cv::Scalar(0, 165, 255);  // 小符亮灯（橙）
      if (b.class_id == 2) col = cv::Scalar(0, 220, 0);    // 大符亮灯（绿）
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
    std::snprintf(l1, sizeof(l1), "%s 第%d次  RP26  目标叶%d片(未激活) 亮灯%d片",
                  big ? "大符" : "小符", rune_counter_.count(), n_cls0, n_cls1);
    std::snprintf(l2, sizeof(l2), "状态:%s  found:%s  开火:%s  %s",
                  state_txt, found ? "有" : "无", fire ? "许可" : "无",
                  block[0] ? block : "-");
    cv::rectangle(frame, cv::Point(0, 0), cv::Point(560, 56), cv::Scalar(0, 0, 0), -1);
    cv::putText(frame, l1, cv::Point(10, 22), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 255), 2);
    cv::putText(frame, l2, cv::Point(10, 46), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 255, 255), 2);
  }

  // ==========================================================================
  // 发布
  // ==========================================================================
  // 唯一的出口：把这一帧要发的角度和射击许可发给游戏。
  // ★ last_yaw_/last_pitch_/fire_sent_ 是【无条件】更新的，publish_angles_=false
  //   （只观察模式）时也一样 —— 因为状态机要靠 last_* 当"上一条指令"，看门狗
  //   也要靠 fire_sent_ 判断"现在有没有许可需要撤"。它们记录的是【节点的意图】，
  //   不是"实际发出去的东西"
  void send_angles(double yaw, double pitch, bool shoot) {
    // ★ 非有限值兜底：NaN/Inf 游戏会【整包拒收】（规则 7.3），表现为云台
    //   "冻住"（0.5s 超时被清指令）。打顶上的符片、抬头看前哨站这种大仰角
    //   几何最容易解出 NaN。这里换成上一条好指令发出去，绝不发脏包。
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
    // pitch 限幅保护：解算出离谱值（比如基板换系出错那种 79° 垂直解）时
    // 别让云台撞机械限位。构造参数 pitch_limit_deg 默认 60°
    sd.pitch = static_cast<float>(std::clamp(pitch, -pitch_limit_, pitch_limit_));
    sd.if_shoot = shoot;
    pub_->publish(sd);
  }

  // 撤射击许可但不甩云台（7.2：帧有问题时要主动发 if_shoot=false，不能沉默）。
  // ★ fire_sent_ 的短路判断有实际意义：没发过许可就不必发撤销，
  //   否则每收到一帧坏数据都要往话题上写一条，白白打扰游戏侧
  void withdraw_fire() {
    if (fire_sent_) send_angles(last_yaw_, last_pitch_, false);
  }

  // ==========================================================================
  // 看门狗：超 0.5 s 没帧 → 撤销射击许可
  // ==========================================================================
  void watchdog() {
    const double idle = std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - last_rx_).count();
    if (idle < kCommandTimeoutSec) {
      stream_warned_ = false;                 // 帧回来了，下次断流重新报
      return;
    }
    if (fire_sent_) {
      send_angles(last_yaw_, last_pitch_, false);
      RCLCPP_WARN(get_logger(), "断流 %.2fs → 已撤销射击许可", idle);
    } else if (!stream_warned_) {
      // ★ 裸断流也报一次：区分"帧停了（定时器还活着 → 能报）"和
      //   "识别回调卡死（定时器都跑不了 → 什么日志都没有）"
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

  // ---------------- 帧率统计 ----------------
  // 采集帧率 = 游戏发过来的帧率；处理帧率 = "模型+PnP+EKF 跑完"的速率。
  // 都取最近 kFpsWindow 帧的滑动窗口，避免被开局那几帧拖偏。
  static constexpr size_t kFpsWindow = 60;
  static double rate_fps(const std::deque<std::chrono::steady_clock::time_point> & q) {
    if (q.size() < 2) return 0.0;
    const double span = std::chrono::duration<double>(q.back() - q.front()).count();
    return span > 1e-6 ? static_cast<double>(q.size() - 1) / span : 0.0;
  }
  std::deque<std::chrono::steady_clock::time_point> rx_times_, proc_times_;
  double ema_ms_ = 0;
  int processed_ = 0;

  // ---------------- 成员 ----------------
  // cfg_ 是节点自己持有的一份配置副本（enemy_color 已被参数覆盖），
  // 换局重建 Tracker 时直接拿它再传一遍
  YAML::Node cfg_;
  Color enemy_ = blue;
  int last_locked_ = -1;              // 上一帧锁的谁，用来检测"锁定切换"事件
  uint64_t last_seq_ = 0;             // 上一帧的 frame_sequence，回退即换局（7.5）
  int frame_id_ = 0;                  // 进可视化用的帧号（只在 draw 里用）
  bool intr_ready_ = false, warned_fb_ = false;   // 各只用于"首次"打一条日志
  bool publish_angles_ = false, auto_fire_ = false, show_window_ = false;
  bool fire_sent_ = false;            // 当前是否已发过射击许可（撤许可时判它）
  char last_fire_block_[64] = "";     // 本帧开火被否的原因（决定链里填，日志用）
                                      // 64 而不是 24：中文原因（如"前哨站收敛中(1.2/3s)"）
                                      // 一个汉字 3 字节，24 装不下会截尾（编译警告也会报）
  bool stream_warned_ = false;        // 裸断流只报一次（见 watchdog）
  bool nan_warned_ = false;           // 非有限角告警只报一次一轮（见 send_angles）
  std::chrono::steady_clock::time_point last_slow_warn_{};   // 慢帧告警节流（见 on_frame）
  double last_good_yaw_ = 0.0, last_good_pitch_ = 0.0;   // 上一条有限的好指令

  // 开火决策的状态（照 sp_vision 的 last_command_）
  bool have_prev_cmd_ = false;
  double prev_cmd_yaw_ = 0.0;
  double first_tol_ = 3.0, second_tol_ = 2.0, judge_dist_ = 2.0;   // 度 / 米
  // 距离限制：水平距离超过它就不发 if_shoot；<= 0 表示不限
  double max_fire_distance_ = 4.0;                                 // 米

  // 底盘停稳门：底盘速度超过它就不给射击许可（Unity 单位/s，3 单位 = 1 m；
  // <=0 关掉）。见 decide_fire 顶部。
  double fire_max_chassis_speed_ = 1.0;
  double chassis_speed_ = 0.0;      // 每帧从 state.twist 更新（decode_and_aim）

  // 前哨站收敛门：锁定后的前 outpost_settle_s 秒不给射击许可（dz 还在学）。
  double outpost_settle_s_ = 4.0;
  double outpost_lock_t0_ = -1e9;   // 锁上前哨站的时刻（steady_seconds）

  // 换板开火门（只作用于车）：上一次有效瞄准的板号 / 最近一次换板时刻。
  // 见 kTracking 分支的检测和 decide_fire 里的门。
  int    last_aim_plate_id_ = -1;
  double plate_switch_t_ = -1e9;
  double plate_switch_settle_s_ = 0.0;   // 秒；0 = 关闭
  // 弹量门槛：ammo_remaining 低于它（含等于）就当"打空了"，进 kNoAmmo
  int min_ammo_to_fire_ = 0;                                       // 发
  // 弹量字段看起来没被正确填充时只警告一次（见 decode_and_aim 里那次判断）
  bool warned_ammo_ = false;

  // 前哨站旋转方向的上一条日志值（-2 = 还没见过前哨站）
  // ★ 用 -2 而不是 -1 当哨兵：-1 是"顺时针"的合法值，拿它当初始值会让
  //   第一次投出顺时针时不打日志
  int last_outpost_dir_ = -2;

  // 本帧是不是在对基地直瞄（基地不走 EKF，日志里单独标，
  // 同时它也决定可视化要不要画 EKF 预测箭头）
  bool aiming_base_ = false;

  // ==========================================================================
  // 打符（kRune 模式）—— 成员与参数
  // ==========================================================================
  std::unique_ptr<RuneDetector>    rune_detector_;
  std::unique_ptr<RuneRp26>        rune_rp26_;      // RP-26Rune 对接层（算法全在里面）
  RuneActivationCounter rune_counter_{22.0};

  // ---- ★ 2026-10-05 自动叫符（到打符位重试直到 accepted）----
  int    rune_req_retry_ms_ = 2000;                  // 重试间隔（config: rune.request_retry_s）
  uint64_t rune_req_id_ = 0;                         // 下一条 request_id（ctor 按毫秒时间戳播种）
  std::atomic<uint64_t> rune_req_last_id_{0};        // 最后发出的 id（等它的回执）
  std::atomic<bool>     rune_req_ok_{false};         // 本轮已受理（循环停）
  uint64_t rune_req_last_accepted_id_ = 0;           // 已计过数的 id（防同一 id 重发重复计数）
  bool     rune_req_active_ = false;                 // 在"到点叫符"循环里（enter_stage/到点改）
  std::chrono::steady_clock::time_point rune_req_next_t_{};
  double   rune_req_reject_log_t_ = -1e9;            // 被拒日志限流（5s 一条）

  // 每帧相机内参（apply_intrinsics 里更新；RP26 要求同帧 camera_info）
  cv::Mat intr_K_, intr_D_;
  int  rune_my_color_ = 1;          // 符的目标颜色 = 符灯颜色：0 红 / 1 蓝
  bool rune_warned_exc_ = false;    // RP26 抛异常只在日志里报一次

  bool rune_enabled_ = false;      // 模型加载成功才为 true
  bool mode_forced_ = false;       // config rune.mode 明确写了 armor/rune
  Mode forced_mode_ = Mode::kArmor;
  double rune_active_s_ = 22.0;
  int    max_small_runes_ = 2;

  // 开火节奏/瞄点半径/相位噪声/大符丢帧确认等参数已随老链路退役 ——
  // RP26 对应的旋钮在 third_party/rp26_rune/config/power_rune.json
  // （弹道模型/开火冷却/二值化阈值），不要再在 config_contest.yaml 里找。

  // 搜索（两种模式共用）：俯仰按累计转角的余弦在 [min,max] 间平滑来回
  double search_pitch_min_ = 0.0, search_pitch_max_ = -35.0, search_pitch_cycles_ = 3.0;
  double search_travel_ = 0.0;      // 累计转过的角度（度），俯仰的相位用它

  // 打印用的节流计数
  int rune_processed_ = 0;
  bool rune_started_ = false;      // 本次进入打符模式后有没有做过初始化

  // ---- 任务提示（/mission/cmd）：见 on_mission / mission_hold ----
  // ★ 线程：识别回调（默认组）和控制回调（ctrl_cb_group_）并发跑（见 main）。
  //   mission_mu_ 护住这三个值的"快照一致性"；两个 flag 用 atomic；
  //   st_* 那组 double 是"控制线程读、识别线程写"的对齐 8 字节量 —— 单写单读，
  //   在 x86-64 上不会撕裂，最多差一帧（8ms），对底盘控制无影响，不再加锁。
  mutable std::mutex mission_mu_;
  double mission_yaw_ = 0.0, mission_pitch_ = 0.0;
  double mission_stamp_ = -1e9;    // steady_seconds() 时间戳；超 2s 视为无提示
  std::atomic<bool> mission_rune_{false};   // 任务要求打符模式
  std::atomic<bool> mission_rune_big_{false}; // ★ "hold … rune big"：强制大符（手控调符）

  // ---- 全自动任务（mission:=true 启用）：见 mission_tick ----
  static constexpr double kLookahead = 3.0;    // pure pursuit 前视距离
  static constexpr double kGoalTol = 2.0;      // 到点容差
  static constexpr double kStopDist = 4.5;     // hp 站：进这个距离就停车打
  static constexpr double kResumeDist = 6.0;   // hp 站：目标跑出这个距离再追（迟滞）
  static constexpr double kReplanDist = 3.0;   // hp 站：目标挪了这么多就重规划
  static constexpr double kPlanGiveUp = 10.0;  // 规划连续失败这么久才放弃任务（秒）
  // ★ 2026-10-05 用户改：卡死放弃门槛 5 → 15 次（第 15 次卡住时放弃；计数全任务
  //   累计、不按站重置）。每次卡住照旧"标障碍（只进内存）+ 重规划"自救。
  static constexpr int kStuckGiveUp = 15;
  // ★ 打符站停留上限：到点后最多待这么久，无论符激没激活都按顺序去下一站。
  //   计时从【第一次到点】起算（路上开多久不算）。2026-10-04 定 45 s →
  //   2026-10-16 用户改 70 s（1 分 10 秒）；激活了仍然立刻走，不用等到超时。
  static constexpr double kRuneStageTimeout = 70.0;
  struct Stage {
    std::string name, done;                    // done: arrive / hp / rune
    double gx = 0, gz = 0, aim_pitch = -4.0;
    double wait = 0.0;                         // 到点后停留 N 秒（补弹，如出生点·回）
    double stop = 0.0;                         // 距目标多远停车打（0=用全局 kStopDist）
    // ★ 定点打（用户 2026-10-04 加）：hp 站不跟实时位置、也不按 stop 圈停车 ——
    //   就开到配置点，到点停住打到条件满足。用于"在实测选好的位置打"（如四号·大坡旁：
    //   追目标的路径和选好的射击位不是同一条路，只有定点才能站到那个点）。
    bool fixed = false;
    int number = 0;                            // hp 站打的是几号
    std::string aim_type;                      // "" / "rune" / "enemy"
    int aim_number = 0;
  };
  NavGrid grid_;
  bool mission_on_ = false;
  double mission_vmax_ = 6.0;
  std::vector<Stage> stages_;
  int stage_i_ = 0;
  std::pair<double, double> goal_{0, 0};
  std::vector<std::pair<double, double>> path_;
  size_t path_i_ = 0;
  bool path_valid_ = false;
  // hp 站实时目标的可达性缓存：同一个"实时位置"只 probe 一次（基地一动不动，
  // 不能每拍跑 A*)；挪出 1.0 单位就重探。见 mission_tick 的 hp 段。
  std::pair<double, double> live_probe_{0, 0};
  bool live_probe_done_ = false;
  bool live_reachable_ = false;
  // 本体不可驶入时的"前方射击位"（朝车这一侧退 stop_d）及其可达性
  std::pair<double, double> standoff_{0, 0};
  bool standoff_reachable_ = false;
  // 规划失败的重试计时 / 日志节流（失败不判死，见 mission_tick 的 plan 段）
  std::chrono::steady_clock::time_point plan_fail_t0_{};
  double last_plan_warn_s_ = 0.0;
  double rune_yaw_ = 45.05, rune_pitch_ = -15.04;   // 打符位实测预设（json 可覆盖）
  std::chrono::steady_clock::time_point prewarm_t0_{}, pause_until_{}, stuck_since_{}, dwell_t0_{};
  std::chrono::steady_clock::time_point rune_arrive_t0_{};   // 打符站到点时刻（45s 超时）
  bool ever_moved_ = false;
  int stuck_count_ = 0;
  bool waiting_ = false;                     // 到点后在等"打掉"条件
  bool wait_logged_ = false, mission_finished_ = false;
  std::string learned_path_;
  std::vector<std::pair<double, double>> learned_;
  std::atomic<uint32_t> mission_filter_mask_{0};   // 任务期允许开火的兵种位图（0=都不许）
  std::atomic<bool> mission_rune_done_{false};     // 符激活（rune 站判据）
  int rune_count_at_stage_entry_ = 0;              // 进站时的开符计数快照（"本站叫过符"门）
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

  // 控制回调组（底盘/任务线程）；识别回调走默认组。见 ctor 与 main。
  rclcpp::CallbackGroup::SharedPtr ctrl_cb_group_;
  bool rune_warned_v_ = false;     // projectile_speed 异常只警告一次
  bool rune_finished_ = false;     // 收到 activated=true：这轮打完了，立即回车自瞄
  bool rune_seen_ = false;         // 这次开符跟到过符：之后丢检只保持、不搜索（符不会跑）
  int  rune_lost_ = 0;             // "暂丢"连续帧数（解出来就清零）
  int  rune_lost_tolerance_ = 45;  // 暂丢容忍：连续丢这么多帧才退回搜索

  // 状态机（迁移规则见类顶部注释）
  AimState state_ = AimState::kSearch;
  int lost_ = 0;                          // kDetecting / kTempLost 里的连续丢帧计数
  int detect_count_ = 0;                  // kDetecting 里的连续"看到"帧数
  int min_detect_count_ = 5;              // 连续看到几帧才进跟踪（同 sp_vision）
  double search_rate_ = 120.0;            // 搜索时云台 yaw 角速度（度/秒）
  int max_temp_lost_ = 15;                // 暂丢容忍帧数（和 Tracker 用同一个配置键）
  bool started_ = false;                  // 首帧把 last_yaw_ 对齐到云台实际角
  std::chrono::steady_clock::time_point last_frame_t_ = std::chrono::steady_clock::now();
                                          // 搜索转速按它积分得 dt（见 decode_and_aim）

  // kDetecting 里连续多少帧看不到目标就放弃、回去搜索
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
  rclcpp::Publisher<tdt_interface::msg::RuneRequest>::SharedPtr pub_rune_req_;  // 叫符请求出口
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  // ★ 单线程执行器（2026-10-02 晚按用户要求改回）。
  //   多线程版曾用于治"上全自动后云台滞后"，但前哨站处出现卡死/崩溃，
  //   先回归单线程稳定基线。回调分组/锁都保留着（单线程下无副作用），
  //   要再启多线程只需换回 MultiThreadedExecutor。
  rclcpp::spin(std::make_shared<AutoAimNode>());
  rclcpp::shutdown();
  return 0;
}
