#pragma once
#include <list>
#include <memory>
#include <opencv2/opencv.hpp>

namespace YAML { class Node; }   // 只用来传引用，不需要完整定义
class RuneModel;                 // 部署库的神符模型类，同上：前向声明 + unique_ptr

// ============================================================================
// rune_detector —— 能量机关（符）检测（TensorRT 后端）
//
// 和车的 Detector 是【同一个部署库】里的另一支：
//   RuneModel  ←→ 26 仓库的打符模型 Rune-v8n-fp16-20260624
//   输入 [1, 3, 480, 640] f32 (RGB, 0~1)      和车模型同形状，4:3 纯 resize 无 padding
//   输出 [1, 18, 6300] f32
//        [0:3]   3 类分数：0 未激活 / 1 小符已激活 / 2 大符已激活（sigmoid 已烘进图）
//        [3:18]  5×(x, y, conf) 关键点，顺序：top, left, R, right, bottom
//
// ★ 解码和 NMS 都在部署库里（RunePostProcessor），本类只做"翻译"：
//   后处理逻辑与 tdt-campus-test 的 detector.py 逐条对应 ——
//   3 类 argmax + 关键点 conf>0.8 至少 3 个 + 中心距 NMS（跳过 R 点、四角均值当中心，
//   默认 100px；大符组阈值 1/3）。所以 C++ 侧不需要重写任何解码。
// ★ 构造会抛异常（路径错 / engine 过期 / 模型类型不匹配 —— 拿车模型构造 RuneModel
//   会直接抛），调用方自己兜底。
//
// ★ 关键点语义（和车模型完全不同的地方，容易踩）：
//     0 top / 1 left / 3 right / 4 bottom  = 符叶 R 标的四个角（图像空间逆时针 0→1→4→3）
//     2 = **整个能量机关的旋转中心**（不是符叶中心！同一帧所有符叶的第 2 点重合）
//   打符策略里"叶心"= 四个角的均值（blade_center()），"机关中心"= 第 2 点。
// ============================================================================

// 一片符叶的检测结果（只含 2D 部分；3D 由 RuneSolver 填）
struct RuneBlade {
  int   class_id   = -1;     // 0 未激活 / 1 小符已激活 / 2 大符已激活
  float confidence = 0.f;
  // 5 个关键点，原图像素坐标，顺序 top/left/R/right/bottom。
  // 和 RuneSolver 的模型点顺序一一对应（R 点是机关中心，不是叶心）。
  std::vector<cv::Point2f> points;

  // 叶心 = 四个角的均值（跳过 R 点）。这就是 PnP 里"叶心 = buff 系 (0,0,0.7)"那个点，
  // 也是可视化里的黄十字。别拿 points[2] 当叶心 —— 那是机关中心。
  cv::Point2f blade_center() const {
    return {(points[0].x + points[1].x + points[3].x + points[4].x) / 4.0f,
            (points[0].y + points[1].y + points[3].y + points[4].y) / 4.0f};
  }
  // 机关旋转中心（第 2 点）
  cv::Point2f rune_center() const { return points[2]; }
};

class RuneDetector {
public:

  // cfg 读 rune.model_path / rune.score_threshold（见 contest_node/config_contest.yaml）
  explicit RuneDetector(const YAML::Node & cfg);

  // 析构放 .cpp：RuneModel 在前向声明下不完整，unique_ptr 的默认析构编不过
  ~RuneDetector();

  // 全部类别都返回（调用方自己按类别过滤/画图 —— 和 Python 版一致：
  // 瞄准只挑目标类，但可视化要看到所有类别，含"类别对不上"的告警场景）
  std::list<RuneBlade> detect(const cv::Mat & img);

private:

  std::unique_ptr<RuneModel> model_;
  float score_th_ = 0.5f;
};
