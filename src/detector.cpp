//检测、后处理和结果映射
//网络输出格式见 detector.hpp

#include "detector.hpp"
#include "network_deployment_interface.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>

namespace {

//模型 armor_id 0..8 到 ArmorName 的映射
//基地顶部/底部都映射到 base，因此两块板会进入同一个 track
const ArmorName kArmorNames[9] = {
  sentry, one, two, three, four, five, outpost, base, base
};

} //namespace

Detector::Detector(const YAML::Node & cfg) {

  //置信度阈值，低于此值的检测框由部署库后处理丢弃
  if (cfg["score_threshold"])
    score_th_ = cfg["score_threshold"].as<float>();

  //配置中必须是可用路径，本类不做相对路径解析
  const std::string model_path = cfg["model_path"].as<std::string>();

  //auto_detect 会根据 engine 输出形状选择后处理，[21,6300] 对应 V8_21InfantryPostProcessor
  model_ = std::make_unique<ArmorModel>(
      model_path,
      "sync",
      "tensorrt",
      "GPU",
      score_th_,
      "auto_detect");
}

Detector::~Detector() = default;

std::list<Armor> Detector::detect(const cv::Mat & img) {

  std::list<Armor> armors;
  if (img.empty()) return armors;

  //坐标已由部署库还原到原图尺度
  const std::vector<NetArmorResult> results = model_->netProcess(img, kKeepBothColors);

  for (const NetArmorResult & r : results) {

    //少于 4 个关键点无法做 PnP
    if (r.points.size() < 4) continue;

    //color_id：0=蓝，1=红，2=白，3=紫；只保留红蓝
    if (r.color_id != 0 && r.color_id != 1) continue;

    if (r.armor_id < 0 || r.armor_id > 8) continue;

    Armor a;
    a.confidence = static_cast<float>(r.score);
    a.color      = (r.color_id == 0) ? blue : red;
    a.name       = kArmorNames[r.armor_id];
    //部署库 size：1=大板，与 big=0/small=1 的枚举顺序相反
    a.type       = (r.size == 1) ? big : small;

    //模型点序为 左上 左下 右下 右上，PnP 需要 左上 右上 右下 左下，按 [0,3,2,1] 重排
    a.points = {
      {(float)r.points[0].x, (float)r.points[0].y}, //左上
      {(float)r.points[3].x, (float)r.points[3].y}, //右上
      {(float)r.points[2].x, (float)r.points[2].y}, //右下
      {(float)r.points[1].x, (float)r.points[1].y}, //左下
    };

    //仅用于显示，距离使用 PnP 的 tvec
    a.box = cv::boundingRect(a.points);

    armors.push_back(a);
  }

  //只填了 2D 字段，调用方还需逐块调用 Solver::solve()
  return armors;
}
