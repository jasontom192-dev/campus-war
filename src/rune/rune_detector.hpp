#pragma once
#include <list>
#include <memory>
#include <opencv2/opencv.hpp>

namespace YAML { class Node; } //只传引用，不需要完整定义
class RuneModel; //部署库模型类，配合 unique_ptr 前向声明

//能量机关检测：输入 [1,3,480,640] RGB f32，输出 [1,18,6300]
//3 类分数 0 未激活/1 小符已激活/2 大符已激活，sigmoid 已烘焙
//关键点顺序 top/left/R/right/bottom，第 2 点 R 是机关中心

//一片符叶的 2D 检测结果；3D 由 RuneSolver 填
struct RuneBlade {
  int   class_id   = -1; //0 未激活 / 1 小符已激活 / 2 大符已激活
  float confidence = 0.f;
  //5 个关键点，原图像素坐标，顺序 top/left/R/right/bottom
  std::vector<cv::Point2f> points;

  //叶心=四角均值(跳过第 2 点)，对应 buff 系 (0,0,0.7)
  cv::Point2f blade_center() const {
    return {(points[0].x + points[1].x + points[3].x + points[4].x) / 4.0f,
            (points[0].y + points[1].y + points[3].y + points[4].y) / 4.0f};
  }
  //机关旋转中心(第 2 点)
  cv::Point2f rune_center() const { return points[2]; }
};

//后处理在部署库：关键点 conf>0.8 且至少 3 点，中心距 NMS 100px，大符阈值 1/3
class RuneDetector {
public:

  //读 rune.model_path / rune.score_threshold
  explicit RuneDetector(const YAML::Node & cfg);

  //析构放 .cpp：RuneModel 不完整，unique_ptr 默认析构编不过
  ~RuneDetector();

  //返回全部类别，调用方自行过滤/画图
  std::list<RuneBlade> detect(const cv::Mat & img);

private:

  std::unique_ptr<RuneModel> model_;
  float score_th_ = 0.5f; //类别分数阈值
};
