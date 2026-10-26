#pragma once

#include <list>
#include <opencv2/opencv.hpp>

#include "rune_detector.hpp"

//符叶选择器：小符单目标，大符一对两片
//小符按亮灯数跟踪；大符连续丢检 vanish_frames 帧才确认打掉
//只处理调用方已按类别过滤的列表；返回指针在调用期间有效

//小符：单目标 + 多亮一片事件
class RuneTargetPicker {
public:
  //返回选中那片；没有可选的返回 nullptr
  const RuneBlade * pick(const std::list<RuneBlade> & dets);
  void reset();

  bool new_blade() const { return new_blade_; } //本帧比上一帧多一片(命中确认用)
  int  light_num() const { return light_num_; }

private:
  bool has_last_centers_ = false;
  std::vector<cv::Point2f> last_centers_;
  bool has_last_target_ = false;
  cv::Point2f last_target_{0.f, 0.f};
  int lost_ = 0;
  bool new_blade_ = false;
  int light_num_ = 0;
};

//大符：一对两片，连续丢 4 帧确认打掉
class RuneBigPairPicker {
public:
  RuneBigPairPicker(double vanish_px = 80.0, int vanish_frames = 4);

  const RuneBlade * pick(const std::list<RuneBlade> & dets);
  void reset();

  //本帧确认目标消失(confirm 模式的命中信号)
  bool vanished() const { return vanished_; }

private:
  void adopt(const RuneBlade & b);

  double vanish_px_;
  int vanish_frames_;

  bool has_target_ = false;
  cv::Point2f last_center_{0.f, 0.f};
  bool vanished_ = false;
  int lost_ = 0; //连续丢检帧数
  int idle_ = 0; //完全无目标时的连续空帧数
};
