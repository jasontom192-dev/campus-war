#pragma once

//开符计数：记录本局开符次数和单次开符窗口
//count()>=3 按大符处理；active(now) 判断是否还在 active_s 窗口内；reset() 换局清零
//收到 accepted 后调用 mark_opened()

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
