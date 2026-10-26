// ============================================================================
// detector.cpp —— 检测环节的全部实现（很薄，因为脏活都在部署库里）
//
// 本文件只做三件事：
//   ① 从 config 读模型路径和阈值，把部署库的 ArmorModel 建起来
//   ② 调 netProcess() 拿原始结果
//   ③ 把 NetArmorResult 逐条翻译成工程的 Armor（颜色/兵种/大小板/关键点重排）
//
// ★ 换模型时最需要动的地方就是第 ③ 步：输出格式一变，映射表就得跟着变。
//   （4.4 从 OpenVINO + tiny_resnet 换成端到端 v8n 时，这里从 204 行缩到 95 行，
//     因为"认数字"那一步被网络自己消化了 —— 见 detector.hpp 顶部的说明。）
// ============================================================================

#include "detector.hpp"
#include "network_deployment_interface.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>

namespace {

// 模型的 9 个兵种类别 → 工程的 ArmorName。
// 顺序来自部署库 V8_21InfantryPostProcessor::m_armor_names：
//   0 哨兵  1 英雄  2 工程  3 3号步兵  4 4号步兵  5 5号步兵  6 前哨站
//   7 基地（底部）  8 基地（顶部）
//
// ★ 这 9 类和工程原有的分类器语义是同一回事：工程旧版让 tiny_resnet 去认
//   装甲板上的数字（1 英雄 / 2 工程 / 3·4·5 步兵 / …），新模型直接给出了。
//   所以 one/two/three/four/five 的含义没变 —— 仍然是"这块板属于哪台车"，
//   Tracker 拿它当 track 的 key 依旧成立。
//
// ★ 基地拆成了顶部/底部两类，工程只有一个 base，先都映射过去。
//   代价：同一个基地的两块板会进同一个 track（tracker 的 key 是 (int)name），
//   两块板同时可见时可能把 EKF 拽偏。先这样，出问题再把 base 拆开。
const ArmorName kArmorNames[9] = {
  sentry, one, two, three, four, five, outpost, base, base
};

}  // namespace

Detector::Detector(const YAML::Node & cfg) {

  // 置信度阈值：低于它的检测框在部署库的后处理里就被滤掉了
  if (cfg["score_threshold"])
    score_th_ = cfg["score_threshold"].as<float>();

  // ★ 这里不做路径解析：配置里存的必须是可用的路径。
  //   离线版由 main.cpp 把相对路径解析成绝对路径后再塞进 cfg；
  //   ROS 节点的 config_contest.yaml 里直接写绝对路径。
  const std::string model_path = cfg["model_path"].as<std::string>();

  // 部署库的构造签名：(模型路径, 推理模式, 部署后端, device, 置信度阈值, 后处理模式)
  //
  // infer_mode 给 "sync" 而不是默认的 "async"：TensorRT 后端里这两个走的是
  // 同一段代码（enqueueV3 + cudaStreamSynchronize），"async" 并不真的异步，
  // 只是名字容易误导，这里写 sync 更符合实际行为。
  //
  // postprocess_mode 给 "auto_detect"：库会读 engine 的输出形状自动挑后处理，
  // [21, 6300] 会命中 V8_21InfantryPostProcessor（见 network_deployment_lib.cpp）。
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

  // 图像尺寸不用管：模型输入 640×480，比赛图 960×720、数据集 1440×1080 都是 4:3，
  // 部署库内部做的是纯 resize（同比、无 padding），关键点会还原回原图坐标。
  // 这一次调用背后是：resize → 归一化 → TensorRT 推理 → 解码 → NMS，
  // 返回的坐标已经还原回【原图】尺度（所以下面可以直接当原图像素用）
  const std::vector<NetArmorResult> results = model_->netProcess(img, kKeepBothColors);

  for (const NetArmorResult & r : results) {

    // 四个关键点缺一个就没法做 PnP，直接丢（正常不会发生，防御性检查）
    if (r.points.size() < 4) continue;

    // 颜色：0 蓝、1 红、2 白、3 紫。
    // 白/紫都不是要打的目标（白是装甲板未激活/无敌态的显示色），直接丢
    if (r.color_id != 0 && r.color_id != 1) continue;

    // 兵种 id 越界保护。9 类见上面 kArmorNames 那张表
    if (r.armor_id < 0 || r.armor_id > 8) continue;

    Armor a;
    a.confidence = static_cast<float>(r.score);
    a.color      = (r.color_id == 0) ? blue : red;
    a.name       = kArmorNames[r.armor_id];
    // 部署库的 r.size：1 = 大板。注意这里的三元判断方向和"big=0/small=1"
    // 的枚举顺序是反的，别顺手"化简"成 static_cast
    a.type       = (r.size == 1) ? big : small;

    // ★ 关键点顺序要转：模型的顺序是 左上 左下 右下 右上，
    //   而 Solver::pnp_points() 的 3D 点顺序是 左上 右上 右下 左下
    //   —— 也就是绕一圈的方向相反，按 [0,3,2,1] 重排。
    //   顺序错了的话画出来的四边框会是"蝴蝶结"（对角线交叉）。
    a.points = {
      {(float)r.points[0].x, (float)r.points[0].y},   // 左上
      {(float)r.points[3].x, (float)r.points[3].y},   // 右上
      {(float)r.points[2].x, (float)r.points[2].y},   // 右下
      {(float)r.points[1].x, (float)r.points[1].y},   // 左下
    };

    // 外接矩形由角点现算（可视化/调试用，下游算距离用的是 PnP 的 tvec）
    a.box = cv::boundingRect(a.points);

    armors.push_back(a);
  }

  // ★ 这里返回的 Armor 只有【2D 部分】有效：rvec/tvec/xyz_in_camera/yaw 都还是
  //   默认值。调用方必须接着 for(auto& a : armors) solver.solve(a) 才算完。
  return armors;
}
