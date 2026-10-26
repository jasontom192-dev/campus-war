//CampusRune 离线验证工具：检测器 -> RuneCampus 对接层 -> 上游 RP-26Rune
//输入视频和 meta.csv(t,yaw,pitch,v)，逐帧输出 SEND，便于和 rune_main 对拍

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

#include <Eigen/Geometry>

#include "rune_detector.hpp"
#include "rune_campus.hpp"

//--dbg 用：观测精修与内部跟踪快照
#include "RuneObservationRefiner.hpp"
#include "power_rune_interface.hpp" //debug_get_tracked()
#include "function.hpp" //set_time_offset_ns()，离线回放时钟对齐
#include "json.hpp" //J_POWER_RUNE，读取 extand_rotated_rect

namespace {

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

struct MetaRow { double t = 0, yaw = 0, pitch = 0, v = 23.5; };

std::vector<MetaRow> load_meta(const std::string & path) {
  std::vector<MetaRow> rows;
  std::ifstream f(path);
  if (!f) throw std::runtime_error("打不开 meta csv: " + path);
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    MetaRow r;
    if (std::sscanf(line.c_str(), "%lf,%lf,%lf,%lf", &r.t, &r.yaw, &r.pitch, &r.v) == 4)
      rows.push_back(r);
  }
  return rows;
}

//游戏角(度) -> 相机排版到世界排版的旋转，与 auto_aim_node 的 world_from_cam 一致
Eigen::Matrix3d world_from_cam(double yaw_deg, double pitch_deg) {
  const double d2r = M_PI / 180.0;
  Eigen::Matrix3d M;
  M << 1, 0, 0,
       0, 0, 1,
       0, -1, 0;
  const Eigen::Matrix3d R_pseudo =
      (Eigen::AngleAxisd(-yaw_deg * d2r, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(-pitch_deg * d2r, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  return M.transpose() * R_pseudo * M;
}

}

int main(int argc, char * argv[]) {
  if (argc < 3) {
    std::cout << "用法: " << argv[0]
              << " <video.mp4> <meta.csv> [--big] [--color red|blue]"
                 " [--start N] [--end M]\n";
    return 1;
  }
  const std::string video_path = argv[1];
  const std::string meta_path = argv[2];
  bool big = false;
  int start = 0, end = -1;
  //默认蓝方：颜色反了会得到 0 个轮廓，found 恒 0 且无报错
  int my_color = 1; //约定：0 红 / 1 蓝
  bool dbg = false; //打印每片符叶的轮廓/可用性
  for (int i = 3; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--big")) big = true;
    else if (!std::strcmp(argv[i], "--dbg")) dbg = true;
    else if (!std::strcmp(argv[i], "--start") && i + 1 < argc) start = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--end") && i + 1 < argc) end = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--color") && i + 1 < argc)
      my_color = std::strcmp(argv[++i], "blue") == 0 ? 1 : 0;
  }

  //配置取可执行文件同级的 ../config.yaml
  const std::string exe_dir =
      std::filesystem::canonical("/proc/self/exe").parent_path().string();
  auto cfg = YAML::LoadFile(exe_dir + "/../config.yaml");
  if (!cfg["rune"]) cfg["rune"] = YAML::Node(YAML::NodeType::Map);
  if (!cfg["rune"]["model_path"]) {
    const char * fallback = "/home/king/tdt-campus-test-build/models/rune_v8n.engine";
    if (!std::filesystem::exists(fallback))
      throw std::runtime_error(std::string("没有 rune.model_path，回退也不在: ") + fallback);
    cfg["rune"]["model_path"] = std::string(fallback);
  }

  const cv::Mat K = read_3x3(cfg["camera_matrix"]);
  const cv::Mat D = read_dist(cfg["dist_coeffs"]);

  auto meta = load_meta(meta_path);
  cv::VideoCapture cap(video_path);
  if (!cap.isOpened()) {
    std::cerr << "打不开视频: " << video_path << "\n";
    return 1;
  }

  RuneDetector detector(cfg);
  RuneCampus campus;

  std::printf("# CampusRune 离线验证：%s（%s，我方=%s）内参 fx=%.1f cx=%.1f cy=%.1f\n",
              std::filesystem::path(video_path).filename().string().c_str(),
              big ? "大符" : "小符", my_color == 0 ? "红" : "蓝",
              K.at<double>(0, 0), K.at<double>(0, 2), K.at<double>(1, 2));

  cv::Mat frame;
  int k = 0, frames = 0, found_n = 0, fire_n = 0;
  double yaw_min = 1e9, yaw_max = -1e9, pitch_min = 1e9, pitch_max = -1e9;
  for (;; ++k) {
    cap >> frame;
    if (frame.empty()) break;
    if (k < start) { if (meta.empty()) break; continue; }
    if (end >= 0 && k >= end) break;
    if (meta.empty()) break;
    const MetaRow & m = meta[std::min<size_t>(k, meta.size() - 1)];

    auto blades = detector.detect(frame);

    //--dbg：只跑观测精修，看轮廓分类/可用性
    if (dbg && !blades.empty()) {
      RuneObservation obs;
      obs.is_big_rune = big;
      obs.ori_img = frame;
      obs.timestamp = foxglove::Time(0, 0); //精修不用时间戳，占位
      const double kExtand = (double)J_POWER_RUNE.config_["detect"]["extand_rotated_rect"];
      for (const auto &b : blades) {
        RuneInfo ri;
        ri.top = b.points[0]; ri.left = b.points[1]; ri.point_R = b.points[2];
        ri.right = b.points[3]; ri.bottom = b.points[4];
        ri.class_id = b.class_id;
        std::vector<cv::Point2i> pts{ri.top, ri.left, ri.point_R, ri.right, ri.bottom};
        cv::RotatedRect rr = cv::minAreaRect(pts);
        rr.size = rr.size * (float)kExtand;
        ri.rotated_rect = rr;
        ri.view_rect = rr.boundingRect() & cv::Rect(0, 0, frame.cols, frame.rows);
        ri.view = frame(ri.view_rect).clone();
        ri.color = (my_color == 0) ? RuneInfo::RED : RuneInfo::BLUE;
        obs.rune_infos.push_back(std::move(ri));
      }
      //保存视图裁块和标注图
      if (!obs.rune_infos.empty()) {
        cv::imwrite("/tmp/campus_rune_view.png", obs.rune_infos[0].view);
        cv::Mat anno = frame.clone();
        const RuneInfo &r0 = obs.rune_infos[0];
        cv::rectangle(anno, r0.view_rect, {0, 255, 0}, 1);
        cv::circle(anno, r0.top, 3, {0, 0, 255}, -1);
        cv::circle(anno, r0.left, 3, {0, 255, 0}, -1);
        cv::circle(anno, r0.bottom, 3, {255, 0, 0}, -1);
        cv::circle(anno, r0.right, 3, {255, 255, 0}, -1);
        cv::circle(anno, r0.point_R, 4, {255, 0, 255}, -1);
        cv::imwrite("/tmp/campus_rune_anno.png", anno);
      }
      //保存两种颜色的差分掩膜，检查轮廓提取
      if (!obs.rune_infos.empty()) {
        const cv::Mat &v = obs.rune_infos[0].view;
        std::vector<cv::Mat> bgr; cv::split(v, bgr);
        for (int c = 0; c < 2; ++c) {
          cv::Mat diff = c == 0 ? bgr[2] - bgr[0] : bgr[0] - bgr[2];
          cv::GaussianBlur(diff, diff, {5,5}, 0);
          cv::Mat mask;
          cv::threshold(diff, mask, c == 0 ? 50 : 62, 255, cv::THRESH_BINARY);
          std::vector<std::vector<cv::Point>> cs;
          cv::findContours(mask, cs, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
          std::printf("#MASK %s: 轮廓%zu个 面积[", c == 0 ? "R-B" : "B-R", cs.size());
          for (size_t i = 0; i < std::min<size_t>(cs.size(), 6); ++i)
            std::printf("%.0f ", cv::contourArea(cs[i]));
          std::printf("] ");
          cv::imwrite(c == 0 ? "/tmp/campus_rune_mask_RB.png" : "/tmp/campus_rune_mask_BR.png", mask);
        }
        std::printf("\n"); std::fflush(stdout);
      }
      RuneObservationRefiner refiner;
      RefinedRuneObservation ref = refiner.refine(obs);
      std::printf("#DBG t=%.3f 检出%zu片: ", m.t, blades.size());
      for (const auto &b : blades)
        std::printf("{R(%d,%d) 4角(%d,%d)(%d,%d)(%d,%d)(%d,%d)} ", (int)b.points[2].x, (int)b.points[2].y,
                    (int)b.points[0].x, (int)b.points[0].y, (int)b.points[1].x, (int)b.points[1].y,
                    (int)b.points[3].x, (int)b.points[3].y, (int)b.points[4].x, (int)b.points[4].y);
      for (const auto &sb : ref.rune_blade_2D) {
        const auto &cc = sb.constrained_contours;
        std::printf("[cls=%d armor=%s%s light=%s%s cR=%s%s] ",
                    sb.rune_info.class_id,
                    cc.armor_module_opt ? "有" : "无",
                    cc.armor_module_opt ? (sb.is_armor_module_usable ? "+" : "-") : "",
                    cc.light_arm_opt ? "有" : "无",
                    cc.light_arm_opt ? (sb.is_light_arm_usable ? "+" : "-") : "",
                    cc.center_R_opt ? "有" : "无",
                    cc.center_R_opt ? (sb.is_center_R_usable ? "+" : "-") : "");
      }
      std::printf("\n");
      std::fflush(stdout);
    }

    const Eigen::Matrix3d R = world_from_cam(m.yaw, m.pitch);
    //采集戳按录像节奏推进，并把算法的"现在"对齐到该帧；否则离线快跑时 dt 漂移

    static const double t_first = meta.empty() ? 0.0 : meta[0].t;
    static const uint64_t base_now_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
    const uint64_t stamp_ns =
        base_now_ns + static_cast<uint64_t>((m.t - t_first) * 1e9);
    const int64_t real_now_ns = static_cast<int64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
    function::set_time_offset_ns(static_cast<int64_t>(stamp_ns) - real_now_ns);
    //v 是场景单位/s，3 单位=1m；弹道用 m/s
    const RuneCampus::Result r =
        campus.process(frame, blades, big, K, D, R, stamp_ns, my_color, m.v / 3.0);

    //--dbg：内部状态快照(符心/靶心/相位)
    if (dbg) {
      const power_rune::RuneDebugInfo ti = power_rune::debug_get_tracked();
      if (ti.valid)
        std::printf("#TRK %.4f %d %.5f %.5f %.4f %.4f %.4f %.4f %.4f %.4f "
                    "%.4f %.4f %.4f %.4f %.4f %.4f\n",
                    m.t, (int)ti.is_big, ti.phase, ti.omega,
                    ti.rune_center[0], ti.rune_center[1], ti.rune_center[2],
                    ti.armor_center[0], ti.armor_center[1], ti.armor_center[2],
                    ti.start_vector[0], ti.start_vector[1], ti.start_vector[2],
                    ti.plane_normal[0], ti.plane_normal[1], ti.plane_normal[2]);
    }

    frames++;
    if (r.found) ++found_n;
    if (r.fire) ++fire_n;
    const double yaw_deg = r.yaw * 180.0 / M_PI;
    const double pitch_deg = r.pitch * 180.0 / M_PI;
    if (r.found) {
      yaw_min = std::min(yaw_min, yaw_deg);   yaw_max = std::max(yaw_max, yaw_deg);
      pitch_min = std::min(pitch_min, pitch_deg); pitch_max = std::max(pitch_max, pitch_deg);
    }
    //对拍口径：和老链路逐帧比这两行就行
    std::printf("SEND %.4f yaw=%.4f pitch=%.4f found=%d fire=%d n_det=%zu\n",
                m.t, yaw_deg, pitch_deg, (int)r.found, (int)r.fire, blades.size());
    std::fflush(stdout);

    if (frames % 200 == 0)
      std::cerr << "# 第 " << k << " 帧  检出 " << blades.size() << " 片  "
                << (r.found ? "found" : "-") << (r.fire ? " FIRE" : "") << std::endl;
  }

  std::printf("\n===== CampusRune 离线汇总 =====\n");
  std::printf("总帧 %d  找到符 %d（%.0f%%）  开火许可 %d 次\n", frames, found_n,
              frames ? 100.0 * found_n / frames : 0.0, fire_n);
  if (found_n > 0)
    std::printf("yaw 范围 [%.2f, %.2f]°  pitch 范围 [%.2f, %.2f]°\n", yaw_min, yaw_max,
                pitch_min, pitch_max);
  return 0;
}
