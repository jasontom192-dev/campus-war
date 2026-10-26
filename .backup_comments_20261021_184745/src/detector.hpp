#pragma once
#include <list>
#include <memory>
#include <opencv2/opencv.hpp>
#include "types.hpp"

namespace YAML { class Node; }   // 只用来传引用，不需要完整定义（避免传染 yaml-cpp 头文件）
class ArmorModel;                // 部署库的模型类，同上：前向声明 + 下面的 unique_ptr

// ============================================================================
// Detector —— 装甲板检测（TensorRT 后端）
//
// 用的是 26_NNDeployment 那套统一部署库里的 ArmorModel：
//   模型 Infantry-v8n-fp16-20260726-D1.8w-B16
//   输入 images [1, 3, 480, 640] f32 (RGB, 0~1)
//   输出 output0 [1, 21, 6300] f32
//        [0:4]   颜色 4 类：蓝 / 红 / 白 / 紫
//        [4:13]  兵种 9 类：哨兵 英雄 工程 3号 4号 5号 前哨站 基地(底) 基地(顶)
//        [13:21] 4 个关键点 (x, y)，顺序：左上 左下 右下 右上
//
// ★ 和旧版（OpenVINO yolov8.xml + tiny_resnet 数字分类）最大的区别：
//   这个模型是【端到端】的，颜色和兵种都由网络直接给出，
//   所以 tiny_resnet 分类器、分类用的 classify()、以及 ok_name/ok_type 那套
//   两级过滤全部删掉了。检测这一环只剩"网络出结果 → 填 Armor"。
//
// ★ 本类不接受空模型路径：engine 文件不存在时 ArmorModel 构造会抛异常
//   （throws std::runtime_error），由调用方决定怎么报错。
// ============================================================================
class Detector {
public:

  Detector(const YAML::Node & cfg);

  // 析构放 .cpp 里定义：ArmorModel 在这里只是个前向声明，
  // unique_ptr 的默认析构需要完整类型，写在这儿会编译不过
  ~Detector();

  // 返回的 Armor 只填好了【2D 部分】(confidence/box/points/name/type/color)，
  // rvec/tvec/xyz_in_camera/yaw 由调用方接着调 Solver::solve() 填
  std::list<Armor> detect(const cv::Mat & img);

private:

  std::unique_ptr<ArmorModel> model_;

  // 置信度阈值，传给部署库的 postprocess
  float score_th_ = 0.5f;

  // 传进 netProcess 的 my_color：0=我方红、1=我方蓝、2=两边都留。
  // 固定给 2 —— 敌我过滤统一由 main.cpp 按 enemy_color 做（旧版也是这么分工的）
  static constexpr int kKeepBothColors = 2;
};
