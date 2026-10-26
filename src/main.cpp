//离线自瞄：视频/摄像头 → 检测 → 敌我过滤 → PnP → 跟踪 → 瞄准 → 显示
//单线程运行，相机系即世界系；默认读取可执行文件同级 ../config.yaml
//用法：./auto_aim [config.yaml]

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

//行优先 9 元数组转 3×3 Mat
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

} //namespace

int main(int argc, char * argv[]) {
  //可执行文件所在目录，用于解析配置和模型路径
  std::string exe_dir = std::filesystem::canonical("/proc/self/exe").parent_path().string();

  std::string config_path = exe_dir + "/../config.yaml";
  if (argc > 1) config_path = argv[1];

  auto cfg = YAML::LoadFile(config_path);

  //配置中的相对路径按可执行文件目录解析，不按 cwd
  auto resolve = [&](const std::string & p) {
    if (!p.empty() && p[0] != '/')
      return exe_dir + "/" + p;
    return p;
  };

  cfg["model_path"] = resolve(cfg["model_path"].as<std::string>());

  //顶层内参为 960×720 比赛标称值，video 块为 1440×1080 测试视频内参
  //两者不能混用，有 video 块时优先使用
  YAML::Node video_cfg = cfg["video"];
  YAML::Node cam_node = video_cfg ? video_cfg : cfg;

  cv::Mat K = read_3x3(cam_node["camera_matrix"]);
  cv::Mat D = read_dist(cam_node["dist_coeffs"]);

  //enemy_color 表示要打的一方
  Color enemy = (cfg["enemy_color"].as<std::string>() == "blue") ? blue : red;

  //有 video.path 用视频，否则使用顶层 video_path，都为空则打开摄像头 0
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

  //可将标注帧写入磁盘
  std::string save_dir = cfg["save_frames_dir"] ? cfg["save_frames_dir"].as<std::string>() : "";
  int save_every_n = cfg["save_every_n"] ? cfg["save_every_n"].as<int>() : 0;
  if (!save_dir.empty()) std::filesystem::create_directories(save_dir);

  //模块只构造一次，每帧按固定顺序调用
  Detector detector(cfg); //加载 TensorRT 引擎，失败会抛异常
  Solver solver(K, D); //离线无云台旋转，使用相机系
  Tracker tracker(cfg, solver);
  Visualizer visualizer(solver);

  Aimer aimer(25.0); //初速 25 m/s

  std::cout << "内参来源: " << (video_cfg ? "video 块(离线测试视频)" : "顶层(比赛标称值)")
            << "  fx=" << K.at<double>(0, 0) << " cx=" << K.at<double>(0, 2)
            << " cy=" << K.at<double>(1, 2) << "\n";

  cv::Mat frame;
  int fid = 0; //帧号
  int prev_locked_id = -1; //上一帧锁定编号，用于检测换目标
  double ema_ms = 0; //帧耗时指数滑动平均

  while (true) {
    cap >> frame;
    if (frame.empty()) break;

    //本帧时间戳，作为 Tracker 的 dt 基准
    auto t0 = std::chrono::steady_clock::now();

    auto armors = detector.detect(frame);

    //只保留敌方颜色，在 PnP 前过滤以减少计算
    armors.remove_if([&](const Armor & a) { return a.color != enemy; });

    //PnP 必须在 tracker.update 之前完成
    for (auto & a : armors) solver.solve(a);

    tracker.update(armors, t0);

    //锁定目标变化时清除 Aimer 的选板迟滞
    if (tracker.has_lock() && tracker.locked_id() != prev_locked_id) {
      aimer.reset_lock();
    }
    prev_locked_id = tracker.has_lock() ? tracker.locked_id() : -1;

    //仅在已锁定且未发散时解算
    Aimer::AimResult aim_res;
    if (tracker.has_lock()) {
      auto it = tracker.tracks().find(tracker.locked_id());
      if (it != tracker.tracks().end() && !it->second.diverged()) {
        cv::Point2f img_center(frame.cols / 2.0f, frame.rows / 2.0f);
        aim_res = aimer.best_aim(it->second, img_center, solver);
      }
    }

    visualizer.draw(frame, armors, tracker, fid,
                  aim_res.valid ? &aim_res : nullptr);

    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - t0).count();
    ema_ms = ema_ms * 0.9 + ms * 0.1;

    //每 30 帧输出一次兵种、颜色、置信度和距离
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

    //按间隔保存标注帧
    if (!save_dir.empty() && save_every_n > 0 && fid % save_every_n == 0) {
      char name[64];
      std::snprintf(name, sizeof(name), "/f%05d.jpg", fid);
      cv::imwrite(save_dir + name, frame);
    }

    //wait_ms 同时控制播放速度，ESC 退出；无窗口时不调用 waitKey
    if (show_window) {
      cv::imshow("auto_aim", frame);
      if ((char)cv::waitKey(wait_ms) == 27) break;
    }

    fid++;
  }

  std::cout << "结束，共处理 " << fid << " 帧，平均 " << cv::format("%.1fms", ema_ms) << "/帧" << std::endl;
  return 0;
}
