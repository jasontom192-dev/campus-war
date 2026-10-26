#pragma once

#include <Eigen/Dense>
#include <functional>

// ============================================================================
// ExtendedKalmanFilter —— 通用 EKF 骨架（本工程只用在 Target 里）
//
// 这是个【不带模型的】通用实现：状态转移 f、观测 h、雅可比 F/H、噪声 Q/R
// 全部由调用方传进来，本类只负责那几行矩阵公式。
// 真正的"物理"在 target.cpp：那边定义了 11 维车模型 / 10 维前哨站模型，
// 以及 h_armor_xyz()（观测模型）和 h_jacobian()（线性化）。
//
// ★ 两个"钩子"是这个实现唯一特别的地方 —— 因为状态里有【角度】，
//   而角度的加减不能直接做（179° + 2° 应该是 -179°，不是 181°）：
//     x_add      : 状态相加时怎么处理  → Target::init 里传的那个 lambda 只回卷 x(6)
//     z_subtract : 观测相减时怎么处理  → Target::update 里传的 lambda 回卷 yaw/pitch/face_yaw
//   默认参数是朴素的 + 和 -，给"状态里没有角度"的普通滤波用。
//   （这两个钩子在角度状态上是必须的，漏一个就会出现"目标明明没动、
//     状态却在 ±π 附近来回跳 2π"的经典现象。）
// ============================================================================
class ExtendedKalmanFilter {
public:
  Eigen::VectorXd x;   // 状态均值向量
  Eigen::MatrixXd P;   // 状态协方差矩阵

  ExtendedKalmanFilter() = default;

  ExtendedKalmanFilter(
    const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a + b; })
  : x(x0), P(P0), I_(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add_(x_add) {}

  // 预测步（时间更新）
  //   P = F·P·Fᵀ + Q      协方差先膨胀（过程噪声 Q 就是"模型有多不可信"）
  //   x = f(x)            均值按状态转移往前走
  // ★ 顺序无关紧要：P 只用到 F，x 只用到 f；这里先算 P 纯粹是历史写法。
  // ★ 本工程里 f 和 F 其实是一回事（F·x 再加一次角度回卷），传两个参数是因为
  //   这个骨架是给通用 EKF 写的，非线性场景下 f 和 F 不同。
  void predict(
    const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
  {
    P = F * P * F.transpose() + Q;
    x = f(x);
  }

  // 更新步（观测更新）
  //   S = H·P·Hᵀ + R     创新协方差
  //   K = P·Hᵀ·S⁻¹       卡尔曼增益
  //   P = (I−K·H)·P·(I−K·H)ᵀ + K·R·Kᵀ     ← Joseph 形式
  //   x = x ⊕ K·(z ⊖ h(x))                ← ⊕/⊖ 就是上面那两个钩子
  //
  // ★ 协方差用 Joseph 形式而不是教科书里那个简化式 P = (I−K·H)·P：
  //   两者数学上等价，但 Joseph 形式在数值上是"对称正定保持型"的 ——
  //   浮点误差下也不会把 P 算成非正定矩阵。这个滤波器要连续跑几千帧
  //   （验收记录里是 1546 帧不崩），用简化式迟早会飘。
  // ★ z ⊖ h(x) 是【创新】：观测减去"按当前状态预测出来的观测"，
  //   在目标这边由 z_subtract 负责把角度分量回卷。
  void update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; })
  {
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();

    P = (I_ - K * H) * P * (I_ - K * H).transpose() + K * R * K.transpose();

    x = x_add_(x, K * z_subtract(z, h(x)));
  }

private:
  Eigen::MatrixXd I_;   // 单位阵，构造时按状态维数定死（避免每帧重复分配）

  // 见类顶部的说明：状态相加时的角度回卷钩子
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add_;
};
