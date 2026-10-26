//符检测实现：读配置、调用部署库 netProcess、翻译成 RuneBlade
//解码和 NMS 在部署库；engine 与 TensorRT/GPU 绑定
//模型输出格式变化时检查关键点顺序映射

#include "rune_detector.hpp"
#include "network_deployment_interface.hpp"
#include <yaml-cpp/yaml.h>
#include <stdexcept>

RuneDetector::RuneDetector(const YAML::Node & cfg) {

  //打符参数在 config 的 rune 段下
  const YAML::Node rc = cfg["rune"];
  if (!rc || !rc["model_path"])
    throw std::runtime_error("配置里缺 rune.model_path（打符 engine 的绝对路径）");

  if (rc["score_threshold"])
    score_th_ = rc["score_threshold"].as<float>();

  const std::string model_path = rc["model_path"].as<std::string>();

  //库会校验模型输出形状，与后处理不匹配会直接抛异常
  model_ = std::make_unique<RuneModel>(
      model_path,
      "sync", //TensorRT 后端
      "tensorrt",
      "GPU", //TRT 后端忽略该参数
      score_th_,
      "auto_detect"); //[18,6300] -> RunePostProcessor
}

RuneDetector::~RuneDetector() = default;

std::list<RuneBlade> RuneDetector::detect(const cv::Mat & img) {

  std::list<RuneBlade> blades;
  if (img.empty()) return blades;

  //库内完成 resize/归一化/推理/解码/NMS，返回原图像素坐标
  const std::vector<NetRuneResult> results = model_->netProcess(img);

  for (const NetRuneResult & r : results) {

    //不足 5 点无法定中心，丢弃
    if (r.points.size() < 5) continue;
    if (r.class_id < 0 || r.class_id > 2) continue;

    RuneBlade b;
    b.class_id   = r.class_id;
    b.confidence = static_cast<float>(r.score);

    //库输出已是 top/left/R/right/bottom，直接按序搬
    b.points.reserve(5);
    for (const cv::Point2d & p : r.points)
      b.points.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y));

    blades.push_back(std::move(b));
  }

  return blades;
}
