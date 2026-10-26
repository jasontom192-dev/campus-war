#pragma once

#include <Eigen/Dense>
#include <functional>

//通用 EKF，状态转移、观测模型、雅可比和噪声均由调用方传入
//含角度状态时通过 x_add 和 z_subtract 做角度回卷

class ExtendedKalmanFilter {
public:
  Eigen::VectorXd x; //状态均值
  Eigen::MatrixXd P; //状态协方差

  ExtendedKalmanFilter() = default;

  ExtendedKalmanFilter(
    const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a + b; })
  : x(x0), P(P0), I_(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add_(x_add) {}

  //预测步：P=F·P·Fᵀ+Q，x=f(x)；非线性场景下 f 与 F 可以不同
  void predict(
    const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
  {
    P = F * P * F.transpose() + Q;
    x = f(x);
  }

  //更新步：S=H·P·Hᵀ+R，K=P·Hᵀ·S⁻¹，P 用 Joseph 形式；角度创新由 z_subtract 回卷
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
  Eigen::MatrixXd I_; //单位阵，按状态维数预分配

  //状态相加时的角度回卷函数
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add_;
};
