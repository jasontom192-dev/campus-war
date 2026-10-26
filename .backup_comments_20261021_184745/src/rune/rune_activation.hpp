#pragma once

// ============================================================================
// rune_activation.hpp —— 开符计数（自动叫符 / 任务开符的窗口）
//
// 现在进入打符只有两条路：
//   1. 任务提示 "hold ... rune" / "big"（mission_rune_）；
//   2. 到打符站自动发 RuneRequest，收到 accepted=true 后 mark_opened()。
// 本类只保留计数和单次开符窗口。
//
// 用途：
//   · count()：本局开符次数，第 3 次起按大符打；
//   · active(now)：本次开符是否还在 rune_active_s 窗口内；
//   · reset()：换局清零。
// ============================================================================

class RuneActivationCounter {
public:
  explicit RuneActivationCounter(double active_s = 22.0) : active_s_(active_s) {}

  void reset() {
    count_ = 0;
    opened_at_ = -1.0;
  }

  int mark_opened(double now) {
    count_++;
    opened_at_ = now;
    return count_;
  }

  bool active(double now) const {
    return opened_at_ >= 0.0 && (now - opened_at_) <= active_s_;
  }

  int count() const { return count_; }

private:
  double active_s_;
  int count_ = 0;
  double opened_at_ = -1.0;
};
