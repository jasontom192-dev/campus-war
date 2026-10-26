#pragma once
#include <list>
#include <memory>
#include <opencv2/opencv.hpp>
#include "types.hpp"

//Detector：TensorRT 装甲板检测
//输入 [1,3,480,640] f32 RGB 0~1，输出 [1,21,6300]
//输出 0:4 颜色、4:13 兵种、13:21 四点 (x,y)，点序 左上 左下 右下 右上

namespace YAML { class Node; } //仅传引用，避免引入 yaml-cpp 头文件
class ArmorModel; //部署库模型类，配合下面的 unique_ptr 使用

class Detector {
public:

  //engine 文件不可用时构造函数抛异常
  Detector(const YAML::Node & cfg);

  //析构需完整类型，只能在 .cpp 中定义
  ~Detector();

  //返回的 Armor 只有 2D 字段，rvec/tvec/xyz_in_camera/yaw 需再调用 Solver::solve()
  std::list<Armor> detect(const cv::Mat & img);

private:

  std::unique_ptr<ArmorModel> model_;

  //置信度阈值，传给部署库后处理
  float score_th_ = 0.5f;

  //my_color：0=我方红，1=我方蓝，2=双方都保留；敌我过滤在外部完成
  static constexpr int kKeepBothColors = 2;
};
