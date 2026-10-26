//打符离线流水线：视频 + meta.csv，跑检测、PnP、EKF 和开火决策
//meta 每行 t,yaw,pitch,v；yaw/pitch 为游戏角，v 为场景单位/s
//离线无云台，云台角取自 meta；检测/PnP/世界系可验，EKF/开火需在线节点验证

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

//meta.csv 一行：t,yaw,pitch,v，角度为游戏角
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

//时间戳只用于帧序对齐，不计算时间差

//--diag：统计有解但判未解出的帧：连续丢检复位、转速发散复位、其它
struct RuneDiag {
  int unsolved_sol = 0, reset_lostcn = 0, diverge = 0, other = 0;
  std::vector<double> spd_out; //发散帧的 x(6)(度/s)
  std::vector<int>    events; //前 40 个事件帧号

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

}

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
  bool diag_on = false; //归类有解却未解出的帧
  bool send_on = false; //逐帧打印 SEND，和 rune_campus 对拍
  double phase_noise = -1.0; //<0 用模型默认(小符 0.1 / 大符 0.5)
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

  //配置取可执行文件同级的 ../config.yaml
  const std::string exe_dir =
      std::filesystem::canonical("/proc/self/exe").parent_path().string();
  const std::string config_path = exe_dir + "/../config.yaml";
  auto cfg = YAML::LoadFile(config_path);

  //engine 优先用 rune.model_path，缺失时回退到预生成的 engine
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

  //--big 选模型；--phase-noise 可覆盖相位量测噪声
  BigTarget   big_target(0.7, phase_noise < 0 ? 0.5 : phase_noise);
  SmallTarget small_target(0.7, phase_noise < 0 ? 0.1 : phase_noise);
  RuneBigPairPicker big_picker;
  RuneTargetPicker  small_picker;

  //confirm 开火决策
  RuneAimer rune_aimer;
  int fire_frames = 0;
  RuneDiag diag;

  int ekf_frames = 0;
  int cls1_frames = 0; //小符命中确认：亮灯叶数量增加
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

    //云台游戏角 -> 世界系旋转
    const Eigen::Matrix3d R_g2w = rune::R_gimbal2world(m.yaw, m.pitch);

    auto blades = detector.detect(frame);
    frames++;
    for (const auto & b : blades) {
      if (b.class_id >= 0 && b.class_id < 3) cls_count[b.class_id]++;
    }
    if (!blades.empty()) frames_with_blade++;

    //只打 class 0(未激活)；picker 跟住同一片
    std::list<RuneBlade> lit;
    for (const auto & b : blades)
      if (b.class_id == 0) lit.push_back(b);
    const RuneBlade * target = big ? big_picker.pick(lit) : small_picker.pick(lit);

    RuneSolution sol;
    if (target) sol = solver.solve(*target, R_g2w);
    if (sol.valid) {
      pnp_ok++;
      reproj.push_back(sol.reproj);
      corner_order_seen = sol.corner_order; //锁定后每帧相同，记最后一帧
    }

    //亮灯叶数量，+1=刚打中(小符)
    int cls1 = 0;
    for (const auto & b : blades)
      if (b.class_id == 1) cls1++;

    //解出来就喂解，否则喂 nullptr 当丢帧
    const int lc_before = big ? big_target.lost_cn() : small_target.lost_cn();
    const RuneSolution * feed = sol.valid ? &sol : nullptr;
    const bool unsolved = big ? (big_target.get_target(feed, m.t), big_target.is_unsolved())
                              : (small_target.get_target(feed, m.t), small_target.is_unsolved());
    if (diag_on) {
      if (big) diag.note(big_target, sol.valid, lc_before, k);
      else     diag.note(small_target, sol.valid, lc_before, k);
      //逐帧明细：k t 相位量测 x5 x6 omega_est 相位样本数 sol unsolved
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

    //瞄 + 开火；传副本，aimer 会连续 predict
    double send_yaw = 0.0, send_pitch = 0.0; //和 rune_campus 同口径逐帧对拍
    bool send_found = false, send_shoot = false;
    if (sol.valid && !unsolved) {
      const bool hit_confirmed = big ? big_picker.vanished() : (cls1 > prev_cls1);
      RuneAimer::Result r = big
          ? rune_aimer.aim(big_target, 0.0, m.v / 3.0, m.t, hit_confirmed)
          : rune_aimer.aim(small_target, 0.0, m.v / 3.0, m.t, hit_confirmed);
      if (r.shoot) fire_frames++;
      send_found = true;
      send_shoot = r.shoot;
      //输出游戏角：yaw 直读，pitch=-仰角
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
      //相位残差：EKF 相位对量测相位，按 72°(五片叶间隔)取模
      double d = (xv(5) - sol.ypr(2)) * 180.0 / M_PI;
      d = std::fmod(d + 36.0, 72.0);
      if (d < 0) d += 72.0;
      phase_res.push_back(std::abs(d - 36.0));
      if (big) {
        a_hist.push_back(xv(7));
        w_hist.push_back(xv(8));
      }
    }

    //标注视频：画检出框和解出的叶心重投影
    if (!out_path.empty()) {
      if (!writer.isOpened()) {
        //avc1(H.264)优先，mp4v 兜底
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
        //叶心投回图像，应压在检测的四角均值上
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
