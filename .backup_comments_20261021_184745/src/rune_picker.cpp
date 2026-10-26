// ============================================================================
// rune_picker.cpp —— 两个选择器的实现（逻辑细节见 rune_picker.hpp 顶部）
// ============================================================================

#include "rune_picker.hpp"

#include <algorithm>
#include <limits>

namespace {

float dist2(const cv::Point2f & a, const cv::Point2f & b) {
  const float dx = a.x - b.x, dy = a.y - b.y;
  return dx * dx + dy * dy;
}

// centers 里离 ref 最近的下标
size_t nearest_idx(const std::vector<cv::Point2f> & centers, const cv::Point2f & ref) {
  size_t best = 0;
  float best_d = std::numeric_limits<float>::max();
  for (size_t i = 0; i < centers.size(); ++i) {
    const float d = dist2(centers[i], ref);
    if (d < best_d) { best_d = d; best = i; }
  }
  return best;
}

// centers 里"离 refs 全体最远"的那个（= 新出现的那片）
size_t farthest_from_all(const std::vector<cv::Point2f> & centers,
                         const std::vector<cv::Point2f> & refs) {
  size_t best = 0;
  float best_min = -1.f;
  for (size_t i = 0; i < centers.size(); ++i) {
    float mn = std::numeric_limits<float>::max();
    for (const auto & r : refs) mn = std::min(mn, dist2(centers[i], r));
    if (mn > best_min) { best_min = mn; best = i; }
  }
  return best;
}

}  // namespace

// ============================================================================
// 小符：单目标 + "多亮一片"
// ============================================================================

void RuneTargetPicker::reset() {
  has_last_centers_ = false;
  last_centers_.clear();
  has_last_target_ = false;
  lost_ = 0;
  new_blade_ = false;
  light_num_ = 0;
}

const RuneBlade * RuneTargetPicker::pick(const std::list<RuneBlade> & dets) {
  new_blade_ = false;

  if (dets.empty()) {
    light_num_ = 0;
    if (++lost_ > 6) reset();          // 连丢 6 帧就当序列断了，重新开始
    return nullptr;
  }
  lost_ = 0;
  light_num_ = static_cast<int>(dets.size());

  std::vector<cv::Point2f> centers;
  centers.reserve(dets.size());
  for (const auto & d : dets) centers.push_back(d.blade_center());

  size_t idx = 0;
  if (dets.size() == 1) {
    idx = 0;                                        // 只有一片：就是它
  } else if (!has_last_centers_ || last_centers_.empty()) {
    idx = 0;                                        // 没有历史：先拿第一个
  } else if (dets.size() == last_centers_.size()) {
    // 亮灯数没变：还是刚才那片，取离上一帧目标最近的
    const cv::Point2f ref = has_last_target_ ? last_target_ : last_centers_[0];
    idx = nearest_idx(centers, ref);
  } else if (dets.size() == last_centers_.size() + 1) {
    // 多亮了一片：新亮起来的那片 = 离上一帧所有亮灯最远的
    idx = farthest_from_all(centers, last_centers_);
    new_blade_ = true;
  } else {
    // 灯数跳变（丢检/误检/序列重置）：取离上一帧目标最近的兜底
    const cv::Point2f ref = has_last_target_ ? last_target_ : centers[0];
    idx = nearest_idx(centers, ref);
  }

  last_centers_ = centers;
  has_last_centers_ = true;
  last_target_ = centers[idx];
  has_last_target_ = true;

  auto it = dets.begin();
  std::advance(it, static_cast<long>(idx));
  return &(*it);
}

// ============================================================================
// 大符：一对两片 + "连续丢 N 帧才算打掉"
// ============================================================================

RuneBigPairPicker::RuneBigPairPicker(double vanish_px, int vanish_frames)
  : vanish_px_(vanish_px), vanish_frames_(std::max(1, vanish_frames)) {}

void RuneBigPairPicker::reset() {
  has_target_ = false;
  vanished_ = false;
  lost_ = 0;
  idle_ = 0;
}

void RuneBigPairPicker::adopt(const RuneBlade & b) {
  has_target_ = true;
  last_center_ = b.blade_center();
}

const RuneBlade * RuneBigPairPicker::pick(const std::list<RuneBlade> & dets) {
  vanished_ = false;

  // 还没锁目标：拿一个（有检出就取第一个）
  if (!has_target_) {
    if (dets.empty()) {
      if (++idle_ > 6) reset();
      return nullptr;
    }
    idle_ = 0;
    adopt(dets.front());
    return &dets.front();
  }

  // 找离上一帧目标最近的检出
  const RuneBlade * near_b = nullptr;
  double near_d = 1e18;
  for (const auto & b : dets) {
    const double d = std::sqrt(static_cast<double>(dist2(b.blade_center(), last_center_)));
    if (d < near_d) { near_d = d; near_b = &b; }
  }
  if (near_b && near_d <= vanish_px_) {     // 还在：继续跟它
    lost_ = 0;
    adopt(*near_b);
    return near_b;
  }

  // 这一帧没找到正在跟的那片 —— 先别切，很可能只是丢检
  if (++lost_ < vanish_frames_) return nullptr;

  // 连续丢够 = 这片确认没了（打中变亮 / 转出去）：记命中，切到剩下那一片
  vanished_ = true;
  lost_ = 0;
  if (!near_b) {                            // 一片都不剩
    has_target_ = false;
    return nullptr;
  }
  adopt(*near_b);
  return near_b;
}
