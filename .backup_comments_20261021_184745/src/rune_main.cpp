// ============================================================================
// rune_main.cpp —— 打符离线流水线（跑 tdt-campus-test 录下来的素材，不开 ROS）
//
// 用途：把 C++ 移植版和 Python 版对照着验 —— 同一段录像、同一组云台角，
// 两边的数字应该对得上（这是移植正确性的主要证据）：
//   · 检出：类别分布、每帧片数
//   · PnP：成功率、重投影误差中位数（Python 版在 records_big 上是 ~10 px）
//
// 素材格式（和 Python 的 replay.py 对应）：
//   <video.mp4>   原始画面（record_passive.py 录的 frames_ep1.mp4）
//   <meta.csv>   每帧一行 t,yaw,pitch,v —— 用 tools/meta_to_csv.py 从
//                meta_ep1.jsonl 转出来（C++ 侧不引 JSON 库，转换在 Python 做）
//
// 用法：
//   ./rune_aim <video.mp4> <meta.csv> [--start N] [--end M] [--out 标注视频.mp4]
//
// 注意：离线跑没有云台（云台角从 meta 读、当固定值用），所以这里只验
// 【检测 + PnP + 世界系】这三步；EKF/开火那些要在节点里验。
// ============================================================================

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

#include "rune_detector.hpp"
#include "rune_solver.hpp"
#include "rune_geometry.hpp"
#include "rune_target.hpp"
#include "rune_picker.hpp"
#include "rune_aimer.hpp"

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

// meta.csv 的一行：t, yaw, pitch, v（yaw/pitch 是【游戏角度】，和 Python 一致）
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

// 用的时间戳在 meta 里是"节点单调秒"，这里只当帧序对齐用，不做时间差

// --diag：把"明明解出来了、目标却判未解出"的帧按原因归类（排查识别到却不瞄）。
// 原因只有三种（见 target.cpp / rune_target.cpp 的 get_target）：
//   reset_lostcn：累计丢检 >6 帧后的复位帧（丢检不清零，攒够就强制复位一次）
//   diverge     ：转速估计偏离标称 ±10°/s 的发散复位帧
//   other       ：理论上到不了（用来兜底提示移植漏了分支）
struct RuneDiag {
  int unsolved_sol = 0, reset_lostcn = 0, diverge = 0, other = 0;
  std::vector<double> spd_out;     // 发散帧的 x(6)（度/秒）
  std::vector<int>    events;      // 前 40 个事件的帧号

  template <class T>
  void note(const T & t, bool sol_valid, int lostcn_before, int frame) {
    if (!t.is_unsolved() || !sol_valid) return;
    unsolved_sol++;
    if (!t.first_in()) { other++; return; }
    if (lostcn_before > 6) reset_lostcn++;
    else {
      diverge++;
      spd_out.push_back(t.ekf_x()(6) * 180.0 / M_PI);
    }
    if (events.size() < 40) events.push_back(frame);
  }

  void report() const {
    if (!unsolved_sol) return;
    std::cout << cv::format("\n【诊断】有解却判未解出的帧: %d（丢检复位 %d，"
                            "发散复位 %d，其它 %d）\n",
                            unsolved_sol, reset_lostcn, diverge, other);
    if (!spd_out.empty()) {
      auto v = spd_out;
      std::sort(v.begin(), v.end());
      std::cout << cv::format("  发散帧 x(6): 最小 %.1f° 中位 %.1f° 最大 %.1f° /s"
                              "（标称 60，判据 ±10）\n",
                              v.front(), v[v.size() / 2], v.back());
    }
    std::cout << "  前若干事件帧号: ";
    for (int f : events) std::cout << f << " ";
    std::cout << "\n";
  }
};

}  // namespace

int main(int argc, char * argv[]) {
  if (argc < 3) {
    std::cout << "用法: " << argv[0]
              << " <video.mp4> <meta.csv> [--start N] [--end M] [--out out.mp4]"
                 " [--big] [--diag] [--send] [--phase-noise X]\n";
    return 1;
  }

  const std::string video_path = argv[1];
  const std::string meta_path  = argv[2];
  int start = 0, end = -1;
  bool big = false;
  bool diag_on = false;           // --diag：归类"有解却未解出"的帧
  bool send_on = false;           // --send：逐帧打印 SEND（和 rune_rp26 对拍）
  double phase_noise = -1.0;      // <0 = 用模型默认（小符 0.1 / 大符 0.5）
  std::string out_path;
  for (int i = 3; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--start") && i + 1 < argc) start = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--end") && i + 1 < argc) end = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--out") && i + 1 < argc) out_path = argv[++i];
    else if (!std::strcmp(argv[i], "--big")) big = true;
    else if (!std::strcmp(argv[i], "--diag")) diag_on = true;
    else if (!std::strcmp(argv[i], "--send")) send_on = true;
    else if (!std::strcmp(argv[i], "--phase-noise") && i + 1 < argc)
      phase_noise = std::atof(argv[++i]);
  }

  // 配置：和离线车自瞄一样，用可执行文件同级的 ../config.yaml（里面 camera_matrix
  // 是规则手册的 960×720 标称值，正好和录制素材一致）
  const std::string exe_dir =
      std::filesystem::canonical("/proc/self/exe").parent_path().string();
  const std::string config_path = exe_dir + "/../config.yaml";
  auto cfg = YAML::LoadFile(config_path);

  // 打符 engine：优先用 config 里的 rune.model_path；没有就回退到 tdt-campus-test
  // 已经生成好的那个（同机同 GPU，可以直接复用的现成产物）
  if (!cfg["rune"]) cfg["rune"] = YAML::Node(YAML::NodeType::Map);
  if (!cfg["rune"]["model_path"]) {
    const char * fallback = "/home/king/tdt-campus-test-build/models/rune_v8n.engine";
    if (!std::filesystem::exists(fallback))
      throw std::runtime_error("配置里没有 rune.model_path，回退路径也不存在: " +
                               std::string(fallback));
    cfg["rune"]["model_path"] = std::string(fallback);
  }

  cv::Mat K = read_3x3(cfg["camera_matrix"]);
  cv::Mat D = read_dist(cfg["dist_coeffs"]);

  auto meta = load_meta(meta_path);
  cv::VideoCapture cap(video_path);
  if (!cap.isOpened()) {
    std::cerr << "打不开视频: " << video_path << "\n";
    return 1;
  }

  RuneDetector detector(cfg);
  RuneSolver   solver(K, D);

  // 两个模型按 --big 选一个（小符 7 维匀速 / 大符 10 维正弦）。
  // 相位量测噪声可以命令行覆盖 —— 和 Python 版对照时必须两边一致，
  // 否则"多信模型/多信量测"不同，相位残差和 a/w 估计都会有系统性差别。
  BigTarget   big_target(0.7, phase_noise < 0 ? 0.5 : phase_noise);
  SmallTarget small_target(0.7, phase_noise < 0 ? 0.1 : phase_noise);
  RuneBigPairPicker big_picker;
  RuneTargetPicker  small_picker;

  // 打符的开火决策（confirm 模式）：离线只为看它跑得对不对（冻结/切叶/脱靶补枪）
  RuneAimer rune_aimer;
  int fire_frames = 0;
  RuneDiag diag;                  // --diag 的统计（见文件头 RuneDiag）

  int ekf_frames = 0;
  int cls1_frames = 0;       // 小符命中确认用：亮灯叶数量比上一帧多 = 命中
  int prev_cls1 = 0;
  std::vector<double> phase_res, a_hist, w_hist;

  cv::VideoWriter writer;
  cv::Mat frame;
  int fid = 0;
  int frames = 0, frames_with_blade = 0, pnp_ok = 0;
  int cls_count[3] = {0, 0, 0};
  std::vector<double> reproj;
  std::vector<int> corner_order_seen;

  for (int k = 0; !meta.empty(); ++k) {
    cap >> frame;
    if (frame.empty()) break;
    if (k < start) continue;
    if (end >= 0 && k >= end) break;
    const MetaRow & m = meta[std::min<size_t>(k, meta.size() - 1)];

    // 云台角 → 世界系旋转（和 Python replay 用同一套换算）
    const Eigen::Matrix3d R_g2w = rune::R_gimbal2world(m.yaw, m.pitch);

    auto blades = detector.detect(frame);
    frames++;
    for (const auto & b : blades) {
      if (b.class_id >= 0 && b.class_id < 3) cls_count[b.class_id]++;
    }
    if (!blades.empty()) frames_with_blade++;

    // 瞄准只挑 class 0（未激活=待击打，小符/大符都一样）；
    // 选片用 picker 跟住同一片（大符另有"连续丢 N 帧才算打掉"的规则）
    std::list<RuneBlade> lit;
    for (const auto & b : blades)
      if (b.class_id == 0) lit.push_back(b);
    const RuneBlade * target = big ? big_picker.pick(lit) : small_picker.pick(lit);

    RuneSolution sol;
    if (target) sol = solver.solve(*target, R_g2w);
    if (sol.valid) {
      pnp_ok++;
      reproj.push_back(sol.reproj);
      corner_order_seen = sol.corner_order;   // 自标定锁定后每帧都一样，记最后一帧的
    }

    // 亮灯叶数量（小符的命中确认：+1 = 刚打中一片）
    int cls1 = 0;
    for (const auto & b : blades)
      if (b.class_id == 1) cls1++;

    // EKF：这一帧解出来就喂解，解不出来喂 nullptr（当丢帧）
    const int lc_before = big ? big_target.lost_cn() : small_target.lost_cn();
    const RuneSolution * feed = sol.valid ? &sol : nullptr;
    const bool unsolved = big ? (big_target.get_target(feed, m.t), big_target.is_unsolved())
                              : (small_target.get_target(feed, m.t), small_target.is_unsolved());
    if (diag_on) {
      if (big) diag.note(big_target, sol.valid, lc_before, k);
      else     diag.note(small_target, sol.valid, lc_before, k);
      // 逐帧明细（重定向到文件，用脚本分析；格式：k t 实测相位 x5 x6 omega_est 相位样本数 sol unsolved）
      // 大小符都打（大符的 x5/x6 是 10 维模型的状态 5/6）
      if ((big && big_target.has_ekf()) || (!big && small_target.has_ekf())) {
        const Eigen::VectorXd & xv = big ? big_target.ekf_x() : small_target.ekf_x();
        std::printf("D %d %.4f %.2f %.2f %.2f %.2f %d %d %d\n", k, m.t,
                    sol.valid ? sol.ypr(2) * 180.0 / M_PI : 1e9,
                    xv.rows() > 5 ? xv(5) * 180.0 / M_PI : 0.0,
                    xv.rows() > 6 ? xv(6) * 180.0 / M_PI : 0.0,
                    (big ? big_target.omega_est() : small_target.omega_est()) * 180.0 / M_PI,
                    big ? -1 : small_target.phase_samples(),
                    sol.valid ? 1 : 0, unsolved ? 1 : 0);
      }
    }

    // 瞄 + 开火（传副本 —— aimer 会连续 predict 原地改滤波器）
    double send_yaw = 0.0, send_pitch = 0.0;   // --send：和 rune_rp26 同口径逐帧对拍
    bool send_found = false, send_shoot = false;
    if (sol.valid && !unsolved) {
      const bool hit_confirmed = big ? big_picker.vanished() : (cls1 > prev_cls1);
      RuneAimer::Result r = big
          ? rune_aimer.aim(big_target, 0.0, m.v / 3.0, m.t, hit_confirmed)
          : rune_aimer.aim(small_target, 0.0, m.v / 3.0, m.t, hit_confirmed);
      if (r.shoot) fire_frames++;
      send_found = true;
      send_shoot = r.shoot;
      // 输出【游戏角度】（yaw 直读，pitch = -仰角），和发出去的口径一致
      const Eigen::Vector2d gp = rune::to_game_angles(r.yaw, r.pitch);
      send_yaw = gp(0);
      send_pitch = gp(1);
    }
    if (send_on)
      std::printf("SEND %.4f yaw=%.4f pitch=%.4f found=%d fire=%d n_det=%zu\n",
                  m.t, send_yaw, send_pitch, (int)send_found, (int)send_shoot,
                  blades.size());
    prev_cls1 = cls1;

    if (sol.valid && !unsolved) {
      ekf_frames++;
      const Eigen::VectorXd & xv = big ? big_target.ekf_x() : small_target.ekf_x();
      // 相位残差：EKF 相位 vs 本帧量测相位，按 72°（五片叶的间隔）取模
      double d = (xv(5) - sol.ypr(2)) * 180.0 / M_PI;
      d = std::fmod(d + 36.0, 72.0);
      if (d < 0) d += 72.0;
      phase_res.push_back(std::abs(d - 36.0));
      if (big) {
        a_hist.push_back(xv(7));
        w_hist.push_back(xv(8));
      }
    }

    // 标注视频（可选）：画检出框 + 解出的叶心重投影（和检测的四角均值重合=解对了）
    if (!out_path.empty()) {
      if (!writer.isOpened()) {
        // avc1（H.264）优先：Ubuntu 默认播放器能播；mp4v 只是兜底
        writer.open(out_path, cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 60.0,
                    {frame.cols, frame.rows});
        if (!writer.isOpened())
          writer.open(out_path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 60.0,
                      {frame.cols, frame.rows});
      }
      for (const auto & b : blades) {
        std::vector<cv::Point> poly;
        for (int i : {0, 1, 4, 3}) poly.emplace_back((int)b.points[i].x, (int)b.points[i].y);
        cv::polylines(frame, poly, true,
                      b.class_id == 0 ? cv::Scalar(160, 160, 160) : cv::Scalar(0, 220, 0), 2);
        cv::circle(frame, b.rune_center(), 4, cv::Scalar(255, 255, 255), -1);
      }
      if (sol.valid) {
        // 解出的叶心投回图像（黄点）—— 它应该压在检测的四角均值上
        cv::Point2f bc = solver.project_world(sol.blade_xyz, R_g2w);
        cv::circle(frame, bc, 6, cv::Scalar(0, 255, 255), 2);
        char txt[64];
        std::snprintf(txt, sizeof(txt), "reproj %.1fpx", sol.reproj);
        cv::putText(frame, txt, {12, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0, 255, 255), 2);
      }
      writer.write(frame);
    }

    if (fid % 200 == 0)
      std::cout << "#" << k << " 检出 " << blades.size() << " 片  "
                << (sol.valid ? cv::format("reproj %.1fpx 距离 %.2fm", sol.reproj,
                                           sol.xyz.norm())
                              : std::string("未解出")) << std::endl;
    fid++;
  }

  std::cout << "\n===== 打符离线统计 =====\n";
  std::cout << "处理 " << frames << " 帧，有检出的 " << frames_with_blade
            << " 帧，PnP 成功 " << pnp_ok << " 帧\n";
  std::cout << "类别帧次：cls0=" << cls_count[0] << "  cls1=" << cls_count[1]
            << "  cls2=" << cls_count[2] << "\n";
  if (!reproj.empty()) {
    std::sort(reproj.begin(), reproj.end());
    std::cout << cv::format("重投影：中位 %.1fpx  p90 %.1fpx  最大 %.1fpx\n",
                            reproj[reproj.size() / 2],
                            reproj[(size_t)(reproj.size() * 0.9)],
                            reproj.back());
  }
  std::cout << "自标定角点排列（26 模型下标）: ";
  for (int v : corner_order_seen) std::cout << v << " ";
  std::cout << "\n";

  if (!phase_res.empty()) {
    std::sort(phase_res.begin(), phase_res.end());
    std::cout << cv::format("EKF 相位残差（mod 72°，%d 帧追踪中）: 中位 %.2f°  p90 %.2f°\n",
                            ekf_frames, phase_res[phase_res.size() / 2],
                            phase_res[static_cast<size_t>(phase_res.size() * 0.9)]);
  }
  std::cout << cv::format("开火决策：开火 %d 发  命中 %d  脱靶 %d  冻结帧 %d（共 %d 帧）\n",
                          rune_aimer.shots(), rune_aimer.hits(), rune_aimer.misses(),
                          fire_frames, frames);
  if (big && !a_hist.empty()) {
    std::sort(a_hist.begin(), a_hist.end());
    std::sort(w_hist.begin(), w_hist.end());
    std::cout << cv::format("大符正弦参数估计: a 中位 %.4f（标称 0.78–1.045）  "
                            "w 中位 %.4f（标称 1.884–2.0）\n",
                            a_hist[a_hist.size() / 2], w_hist[w_hist.size() / 2]);
  }
  if (writer.isOpened()) writer.release();
  diag.report();
  return 0;
}
