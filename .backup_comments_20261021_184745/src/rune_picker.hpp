#pragma once

#include <list>
#include <opencv2/opencv.hpp>

#include "rune_detector.hpp"

// ============================================================================
// rune_picker.hpp —— 符的"打哪一片"选择器（照 tdt-campus-test 的 rune.py 移植）
//
// 两个选择器，对应两种开符：
//
//   RuneTargetPicker（小符）：每次开符只有【一片】待击打（扇形不发光、只剩 R 标暗环，
//     模型给 class 0）。选片逻辑是"看灯数变化"：
//       本帧目标类只有 1 片      → 就是它（绝大多数时候）
//       和上一帧片数相同         → 跟住上一帧那片（取中心最近的）
//       比上一帧多一片           → 新出现的那片 = 离上一帧所有目标最远的（new_blade）
//
//   RuneBigPairPicker（大符）：一次【两片】，共 5 对。要打的两片和小符待击打
//     长得一模一样（class 0）；打中一片后另一片仍是待击打，要在 1 s 内打完。
//     命中判据 = **打中的那片从检出里消失**（打中变亮后模型不再出框），
//     但必须【连续丢够 vanish_frames 帧】才算数 —— 单帧丢检（叶片转成侧影、
//     模型闪断）非常常见，见到一帧就切的话第一片还没打中就会跳到第二片
//     （2026-10-01 实机反馈）。丢检期间目标保持不变、pick 返回 nullptr。
//
// ★ 两个 picker 都只吃【已经按目标类别过滤过】的检测列表（调用方过滤），
//   返回的指针指向传入列表里的元素 —— 调用期间列表不能被改。
// ============================================================================

// 小符：单目标 + "多亮一片"事件
class RuneTargetPicker {
public:
  // 返回选中那片；没有可选的返回 nullptr
  const RuneBlade * pick(const std::list<RuneBlade> & dets);
  void reset();

  bool new_blade() const { return new_blade_; }   // 本帧"多亮了一片"（命中确认用）
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

// 大符：一对两片 + "确认打掉"事件
class RuneBigPairPicker {
public:
  RuneBigPairPicker(double vanish_px = 80.0, int vanish_frames = 4);

  const RuneBlade * pick(const std::list<RuneBlade> & dets);
  void reset();

  // 本帧"打的那片确认没了"（confirm 开火模式的命中信号）
  bool vanished() const { return vanished_; }

private:
  void adopt(const RuneBlade & b);

  double vanish_px_;
  int vanish_frames_;

  bool has_target_ = false;
  cv::Point2f last_center_{0.f, 0.f};
  bool vanished_ = false;
  int lost_ = 0;    // 连续丢检帧数（目标还在跟，只是这几帧没检出）
  int idle_ = 0;    // 完全没有目标时的连续空帧数
};
