// ============================================================================
// main.cpp —— 离线自瞄流水线（跑视频或摄像头，不开 ROS）
//
// 这是【单线程一条线跑到底】的最简形态，比赛那套在 contest_node 里：
//   读帧 ─► Detector(TensorRT 检测) ─► 按敌方颜色过滤 ─► Solver(PnP 解位姿)
//        ─► Tracker(EKF 跟踪 + 锁定) ─► Aimer(选板 + 提前量) ─► Visualizer(画)
//
// 和 ROS 节点比，这里少的东西（都是有意的）：
//   · 没有世界系旋转 —— 云台不动，相机系就是世界系，所以坐标一路用相机系
//   · 没有开火决策、没有状态机、没有规则校验（那些只在比赛中有意义）
//   · 内参来自 config.yaml，不像节点那样逐帧取 camera_info
//
// 用法：
//   ./auto_aim                    # 用 exe 同级的 ../config.yaml
//   ./auto_aim /tmp/other.yaml    # 换一份配置（验收时就是这么跑无窗口批量的）
// ============================================================================

#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include "detector.hpp"
#include "solver.hpp"
#include "tracker.hpp"
#include "visualizer.hpp"
#include "aimer.hpp"

namespace {

// 把 [a b c; d e f; g h i] 这种行优先的 9 元数组读成 cv::Mat
cv::Mat read_3x3(const YAML::Node & n) {
  auto v = n.as<std::vector<double>>();
  cv::Mat m(3, 3, CV_64F);
  std::memcpy(m.data, v.data(), 9 * sizeof(double));
  return m;
}

cv::Mat read_dist(const YAML::Node & n) {
  auto v = n.as<std::vector<double>>();
  cv::Mat m(1, (int)v.size(), CV_64F);
  std::memcpy(m.data, v.data(), v.size() * sizeof(double));
  return m;
}

}  // namespace

int main(int argc, char * argv[]) {
  // 可执行文件所在目录。用 /proc/self/exe 而不是 argv[0]，是为了在
  // 软链/任意 cwd 下调用时也能定位到真实路径（config.yaml 和模型都相对它找）。
  std::string exe_dir = std::filesystem::canonical("/proc/self/exe").parent_path().string();

  std::string config_path = exe_dir + "/../config.yaml";
  if (argc > 1) config_path = argv[1];

  auto cfg = YAML::LoadFile(config_path);

  // 配置里的相对路径按【可执行文件所在目录】解析（不是 cwd），
  // 这样从任何地方 ./build/auto_aim 都能找到 ../models/*.engine
  auto resolve = [&](const std::string & p) {
    if (!p.empty() && p[0] != '/')
      return exe_dir + "/" + p;
    return p;
  };

  cfg["model_path"] = resolve(cfg["model_path"].as<std::string>());

  // ------------------------------------------------------------------------
  // 相机内参：跑视频 / 跑比赛用两套
  //
  //   顶层 camera_matrix / dist_coeffs  = 规则手册 7.1 的比赛标称值（960×720）
  //   video.camera_matrix / dist_coeffs = 仓库测试视频那台相机的真值（1440×1080）
  //
  // 两者不能混用：拿 960×720 的内参去解 1440×1080 的图，PnP 出来的距离、
  // 以及反投影回去的框都会整体偏掉。有 video 块（=正在跑离线视频）就用 video 的。
  // ------------------------------------------------------------------------
  YAML::Node video_cfg = cfg["video"];
  YAML::Node cam_node = video_cfg ? video_cfg : cfg;

  cv::Mat K = read_3x3(cam_node["camera_matrix"]);
  cv::Mat D = read_dist(cam_node["dist_coeffs"]);

  // 要打哪一方。注意这是【敌方】颜色：enemy_color=blue 表示我们打蓝方，
  // 所以后面过滤时保留的是 color == enemy 的板（tracker 里的 enemy_color_
  // 同理，只是那个成员其实没人读，过滤在下面 armors.remove_if 里做）。
  Color enemy = (cfg["enemy_color"].as<std::string>() == "blue") ? blue : red;

  // 图像源：video 块里有 path 就用那个视频；否则看顶层 video_path；都没有就开摄像头 0
  cv::VideoCapture cap;
  std::string vid;
  if (video_cfg) vid = video_cfg["path"].as<std::string>();
  else if (cfg["video_path"]) vid = resolve(cfg["video_path"].as<std::string>());

  if (vid.empty())
    cap.open(0);
  else
    cap.open(vid);

  if (!cap.isOpened()) {
    std::cerr << "Cannot open video: " << (vid.empty() ? "(camera 0)" : vid) << "\n";
    return 1;
  }

  int wait_ms = cfg["wait_ms"] ? cfg["wait_ms"].as<int>() : 1;
  bool show_window = cfg["show_window"] ? cfg["show_window"].as<bool>() : true;

  // 离线复核用：把标注帧写出来
  std::string save_dir = cfg["save_frames_dir"] ? cfg["save_frames_dir"].as<std::string>() : "";
  int save_every_n = cfg["save_every_n"] ? cfg["save_every_n"].as<int>() : 0;
  if (!save_dir.empty()) std::filesystem::create_directories(save_dir);

  // 四个模块在这里各建一次，然后每帧按固定顺序调用（顺序不能换，见文件头）
  Detector detector(cfg);        // TensorRT 引擎在这里加载（失败会抛异常）
  Solver solver(K, D);           // 内参定死，离线跑没有云台旋转，不带世界系
  Tracker tracker(cfg, solver);
  Visualizer visualizer(solver);

  Aimer aimer(25.0);             // 初速写死 25 m/s（比赛里由 state 每帧更新）

  std::cout << "内参来源: " << (video_cfg ? "video 块(离线测试视频)" : "顶层(比赛标称值)")
            << "  fx=" << K.at<double>(0, 0) << " cx=" << K.at<double>(0, 2)
            << " cy=" << K.at<double>(1, 2) << "\n";

  cv::Mat frame;
  int fid = 0;                  // 帧号，用于日志和存图
  int prev_locked_id = -1;      // 上一帧锁的是谁，用来检测"换目标"事件
  double ema_ms = 0;            // 每帧耗时的指数滑动平均

  while (true) {
    cap >> frame;
    if (frame.empty()) break;   // 视频读完 / 摄像头断开

    // t0 就是这一帧的时间戳，一路传给 Tracker 当 EKF 的 dt 基准
    auto t0 = std::chrono::steady_clock::now();

    // ── ① 检测：TensorRT 出 2D 结果（颜色/兵种/角点），还带着自己的颜色分类 ──
    auto armors = detector.detect(frame);

    // ── ② 敌我过滤：只留敌方颜色的板。放在 PnP 之前是为了省算力 ──
    armors.remove_if([&](const Armor & a) { return a.color != enemy; });

    // ── ③ PnP：把 2D 角点解成 3D 位姿，填 rvec/tvec/xyz_in_camera/yaw ──
    // ★ 必须在 tracker.update 之前做完（Tracker 假设位姿已经算好）
    for (auto & a : armors) solver.solve(a);

    // ── ④ 跟踪：多目标关联 + 每 track 一个 EKF + 决定锁谁 ──
    tracker.update(armors, t0);

    // 锁定的目标换人了 → 清掉 Aimer 的"锁定同一块板"迟滞，
    // 否则会拿着上一辆车的板编号去新车上找，选板要等好几帧才正常
    if (tracker.has_lock() && tracker.locked_id() != prev_locked_id) {
      aimer.reset_lock();
    }
    prev_locked_id = tracker.has_lock() ? tracker.locked_id() : -1;

    // ── ⑤ 解算：只在"有锁定 且 该 track 没发散"时做 ──
    // 发散的目标状态已经在乱跑，算了也不能用（ROS 节点那边还会连带禁开火）
    Aimer::AimResult aim_res;
    if (tracker.has_lock()) {
      auto it = tracker.tracks().find(tracker.locked_id());
      if (it != tracker.tracks().end() && !it->second.diverged()) {
        cv::Point2f img_center(frame.cols / 2.0f, frame.rows / 2.0f);
        aim_res = aimer.best_aim(it->second, img_center, solver);
      }
    }

    // ── ⑥ 画图：解算无效就传 nullptr，Visualizer 就不会画准星 ──
    visualizer.draw(frame, armors, tracker, fid,
                  aim_res.valid ? &aim_res : nullptr);

    // 每帧耗时（指数滑动平均），换 TensorRT 后主要看这个数
    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - t0).count();
    ema_ms = ema_ms * 0.9 + ms * 0.1;

    // 检出摘要：帧号 / 耗时 / 每块板的 兵种+颜色+置信度+距离
    if (fid % 30 == 0) {
      std::string det;
      for (auto & a : armors) {
        char b[96];
        std::snprintf(b, sizeof(b), "%s/%s/%.2f/d%.2f ",
                      ARMOR_NAMES[a.name].c_str(), COLORS[a.color].c_str(),
                      a.confidence, a.xyz_in_camera.norm());
        det += b;
      }
      std::cout << "#" << fid << " " << cv::format("%.1fms", ema_ms)
                << " 锁" << (tracker.has_lock() ? std::to_string(tracker.locked_id()) : "-")
                << " 检出[" << (det.empty() ? "无" : det) << "]" << std::endl;
    }

    // 离线复核用：每 save_every_n 帧把【已标注】的图写一张。
    // 验收时的用法就是关掉窗口、把这批 jpg 存到 /tmp/frames 再逐张看。
    if (!save_dir.empty() && save_every_n > 0 && fid % save_every_n == 0) {
      char name[64];
      std::snprintf(name, sizeof(name), "/f%05d.jpg", fid);
      cv::imwrite(save_dir + name, frame);
    }

    // wait_ms 兼作"播放速度控制"：跑视频时它决定每帧等多久，
    // 按 ESC(27) 提前退出。show_window=false 时完全不进这个分支，
    // 也就没有 waitKey —— 所以无窗口批量跑是最快的。
    if (show_window) {
      cv::imshow("auto_aim", frame);
      if ((char)cv::waitKey(wait_ms) == 27) break;
    }

    fid++;
  }

  std::cout << "结束，共处理 " << fid << " 帧，平均 " << cv::format("%.1fms", ema_ms) << "/帧" << std::endl;
  return 0;
}
