// ============================================================================
// rune_detector.cpp —— 符检测的实现（和车的 detector.cpp 一样薄）
//
// 三件事：
//   ① 从 config 的 rune 段读 engine 路径和阈值，建部署库的 RuneModel
//   ② 调 netProcess() 拿 NetRuneResult（坐标已还原到原图）
//   ③ 逐条翻译成工程的 RuneBlade
//
// ★ 换打符模型时只需要动第 ③ 步：输出格式一变，关键点顺序的映射就得跟着改。
// ★ engine 与 GPU 架构 / TensorRT 版本绑定，换机器要重新用 trtexec 生成。
//   生成命令（和 tdt-campus-test 的 scripts/build_trt.sh 同款）：
//     trtexec --onnx=Rune-v8n-fp16-20260624.onnx --saveEngine=rune_v8n.engine --fp16
//   本机可以直接复用 tdt-campus-test 已生成的 engine（同机同 GPU）。
// ============================================================================

#include "rune_detector.hpp"
#include "network_deployment_interface.hpp"
#include <yaml-cpp/yaml.h>
#include <stdexcept>

RuneDetector::RuneDetector(const YAML::Node & cfg) {

  // config 里打符的所有参数都在 rune: 段下（和车参数区分开）
  const YAML::Node rc = cfg["rune"];
  if (!rc || !rc["model_path"])
    throw std::runtime_error("配置里缺 rune.model_path（打符 engine 的绝对路径）");

  if (rc["score_threshold"])
    score_th_ = rc["score_threshold"].as<float>();

  const std::string model_path = rc["model_path"].as<std::string>();

  // 构造签名与车的 ArmorModel 完全一致；库会在构造时校验"模型输出形状 vs 后处理"，
  // 拿车模型（[21,6300]）来构造会直接抛异常 —— 这正是我们想要的失败方式。
  model_ = std::make_unique<RuneModel>(
      model_path,
      "sync",           // TensorRT 后端里 sync/async 是同一段代码，写 sync 更贴合实际
      "tensorrt",
      "GPU",            // TRT 后端忽略这个参数
      score_th_,
      "auto_detect");   // [18,6300] → RunePostProcessor
}

RuneDetector::~RuneDetector() = default;

std::list<RuneBlade> RuneDetector::detect(const cv::Mat & img) {

  std::list<RuneBlade> blades;
  if (img.empty()) return blades;

  // 一次调用背后：resize → 归一化 → TensorRT → 解码（argmax 3 类 + 关键点门限）
  // → 中心距 NMS。返回的坐标已经是【原图】像素（库负责还原）。
  const std::vector<NetRuneResult> results = model_->netProcess(img);

  for (const NetRuneResult & r : results) {

    // 5 个关键点缺一个就没法做 PnP / 定中心，防御性丢掉
    if (r.points.size() < 5) continue;
    if (r.class_id < 0 || r.class_id > 2) continue;

    RuneBlade b;
    b.class_id   = r.class_id;
    b.confidence = static_cast<float>(r.score);

    // 库给的点序就是 top/left/R/right/bottom（见 NetRuneResult 注释），
    // 和本工程 RuneBlade::points 的定义一致，直接按序搬，不用重排。
    // （车那边要 [0,3,2,1] 重排是因为模型给的是"左上左下右下右上"；符这边库已经对了。）
    b.points.reserve(5);
    for (const cv::Point2d & p : r.points)
      b.points.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y));

    blades.push_back(std::move(b));
  }

  return blades;
}
