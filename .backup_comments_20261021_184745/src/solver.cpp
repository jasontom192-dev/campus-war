// ============================================================================
// solver.cpp —— PnP 位姿解算 + 板几何 + 投影工具
//
// 这个类做三件事，理清这三件事，整个文件就没难度了：
//   ① solve()      ：2D 角点 → 3D 位姿（solvePnP），再修掉平面板 PnP 的二义性
//   ② 板几何       ：pnp_points / left_lightbar / right_lightbar / digit_box
//                    都是在【板坐标系】里描述"装甲板长什么样"的一堆 3D 点
//   ③ 投影工具     ：project()（3D 点 + 位姿 → 像素，走完整相机模型）
//                    project_world()（状态系一点 → 像素，给准星/预测箭头用）
//
// ★ 板坐标系约定（整份文件都建立在这上面）：
//     x 轴 = 板法线，指向【板 → 车中心】（所以看正面时它指向场景深处）
//     y 轴 = 板宽方向（左右）
//     z 轴 = 板高方向，注意【从上往下】为正（p0 是左上角，z = -HH）
//   原点在板中心。装甲板的物理尺寸（HW_BIG 等）见 solver.hpp 底部。
// ============================================================================

#include "solver.hpp"
#include <Eigen/Geometry>

Solver::Solver(const cv::Mat & K, const cv::Mat & D) : K_(K), D_(D) {}

// 装甲板四个角点在【板坐标系】下的 3D 坐标，作为 solvePnP 的 objectPoints。
//
// ★ 点序：左上 右上 右下 左下（顺时针绕一圈），必须和 Armor::points 的
//   顺序严格对应。detector.cpp 里专门做过一次 [0,3,2,1] 重排就是为了它 ——
//   顺序错开会让 PnP 解出一个"翻转"的位姿，画出来的框是对角线交叉的蝴蝶结。
//
// ★ 高度用的是 HH_LB（灯条半高 = 28mm）而不是整块板的高度：四个角点取的是
//   【灯条的外沿】，不是数字区。用错尺寸会让解出的距离系统性偏大/偏小。
std::vector<cv::Point3f> Solver::pnp_points(ArmorType t) {
  float hw = (t == big) ? HW_BIG : HW_SMALL;
  return {
    {0.f, -hw, -HH_LB},
    {0.f,  hw, -HH_LB},
    {0.f,  hw,  HH_LB},
    {0.f, -hw,  HH_LB}
  };
}

// 一块装甲板的完整位姿解算：solvePnP + 法线角 + 二义性修正。
// 输入是 a.points（检测到的 4 个角点）和 a.type（决定用哪套 3D 尺寸），
// 输出填进 a.rvec / a.tvec / a.xyz_in_camera / a.yaw。
void Solver::solve(Armor & a) {
  auto pts3d = pnp_points(a.type);

  // SOLVEPNP_IPPE：专为【平面四点】设计的解法。用它是必须的 ——
  // 通用的迭代法在 4 个共面点上不稳定，会解出跳变的位姿。
  // 代价是它有"正反面二义"，所以下面紧接着要修（见 optimize_yaw）。
  cv::solvePnP(
    pts3d, a.points, K_, D_,
    a.rvec, a.tvec, false, cv::SOLVEPNP_IPPE);

  // tvec 就是板中心在相机系的位置，直接当 xyz_in_camera 用
  cv::cv2eigen(a.tvec, a.xyz_in_camera);

  cv::Mat R;
  cv::Rodrigues(a.rvec, R);

  // 旋转矩阵的第 0 列 = 板坐标系的 x 轴 = 板法线（在相机系下）。
  // a.yaw = atan2(nz, nx) 表示法线在【水平面内】的方位角，
  // 方向是"板 → 车中心"（见文件头），下游 Tracker/Target 都按这个约定用。
  double nx = R.at<double>(0,0);
  double nz = R.at<double>(2,0);

  a.yaw = std::atan2(nz, nx);

  // ★ PnP 的二义性就在这里修：IPPE 给的是"其中一支"，不保证是正面朝我
  //   那一支。前哨站上错了会让方向投票和 EKF 的顺/逆时针整个反掉。
  //   （参考 sp_vision_25 的 Solver::optimize_yaw）
  optimize_yaw(a);
}

// ============================================================================
// yaw 暴力优化 —— 消掉平面板 PnP 的二义性
//
// 一块平板（4 个角点共面）的 PnP 天生有两支解：板"正面朝我"和"背面朝我"，
// 两者的 4 个角投影几乎一样，IPPE 挑哪支不保证对。
//
// 判据是"装甲板只有正面能被看到"这一条先验：
//   · 模型坐标里板的法线 = 板坐标系的 x 轴（pnp_points 里 4 个角的 x 都是 0）
//   · 它指向【板 → 车中心】（Target 里 a.yaw 就是这个方向，见 target.cpp 注释）
//   · 所以我们看到正面时，它一定指向【远离相机】的那一侧：n · p > 0
//     （p 是板在相机系下的位置）
//
// 做法：位置 tvec 固定，只扫 yaw（绕相机竖直轴、也就是相机系 y 轴转），
// 粗扫 5° + 细扫 1°，每个候选手工投影 4 个角、和观测到的 4 个点比距离和，
// 取误差最小的那支。不用 cv::projectPoints 是因为它每次都要建一堆矩阵，
// 每帧几块板 × 几十次候选太贵；手工投影只有两次乘加。
// （投影里没加畸变：这只是个打分函数，最终位姿还是由完整 PnP 给的）
// ============================================================================
void Solver::optimize_yaw(Armor & a) const {
  const auto pts3d = pnp_points(a.type);
  const double px = a.tvec[0], py = a.tvec[1], pz = a.tvec[2];
  if (pz <= 0) return;                       // 板在相机后面，不折腾

  const double fx = K_.at<double>(0, 0), fy = K_.at<double>(1, 1);
  const double cx = K_.at<double>(0, 2), cy = K_.at<double>(1, 2);

  // ★ 候选姿态是【由 yaw 直接重建】的，不是把 PnP 的解转一转 ——
  //   镜像那支和真值之间差的是反射不是旋转，绕着竖直轴怎么转都到不了，
  //   这一点在第一版实现里踩过（全被约束淘汰，退回错解）。
  //
  //   重建假设：装甲板是竖直的（法线水平、板高方向竖直）。于是位姿只剩
  //   一个自由度 yaw。sp_vision 也是这么干的（在世界系里重建），只是它们
  //   的搜索中心取云台 yaw；我们直接扫一整圈，不需要初始猜测。
  //
  //   世界系（= 去旋转后的相机系，x=右 y=下 z=前）里：
  //     法线 n = (cosδ, 0, sinδ)   —— 和 a.yaw = atan2(nz, nx) 的定义一致
  //     板高方向 = (0, 1, 0)       —— 模型 z 轴是从上往下（p0=左上 z=-HH）
  //     宽度方向 w = u × n，凑成右手系
  //   再乘 cam_from_world_ 转回相机系投影。离线视频那条路 cam_from_world_
  //   是单位阵，等价于"板在相机系里竖直"。
  const Eigen::Matrix3d R_cw = cam_from_world_;

  auto eval = [&](double delta, cv::Mat & R_out) -> double {
    const double c = std::cos(delta), s = std::sin(delta);
    // R_W(δ) = [n | w | u]，列向量
    Eigen::Matrix3d R_w;
    R_w << c,  s, 0.0,
           0.0, 0.0, 1.0,
           s, -c, 0.0;
    const Eigen::Matrix3d R_cam = R_cw * R_w;

    // 约束：法线（第一列）必须朝远离相机那一侧 —— 装甲板只有正面能被看到
    const Eigen::Vector3d n = R_cam.col(0);
    if (n(0) * px + n(1) * py + n(2) * pz <= 0.0) return 1e18;

    double err = 0.0;
    for (int i = 0; i < 4; ++i) {
      const Eigen::Vector3d X(pts3d[i].x, pts3d[i].y, pts3d[i].z);
      const Eigen::Vector3d pc = R_cam * X + Eigen::Vector3d(px, py, pz);
      if (pc(2) <= 1e-6) return 1e18;
      const double u = fx * pc(0) / pc(2) + cx;
      const double v = fy * pc(1) / pc(2) + cy;
      err += std::hypot(u - a.points[i].x, v - a.points[i].y);
    }
    R_out = (cv::Mat_<double>(3, 3) <<
      R_cam(0,0), R_cam(0,1), R_cam(0,2),
      R_cam(1,0), R_cam(1,1), R_cam(1,2),
      R_cam(2,0), R_cam(2,1), R_cam(2,2));
    return err;
  };

  double best_err = 1e18, best_delta = 0.0;
  cv::Mat R_cand;
  for (int d = -180; d < 180; d += 5) {                  // 粗扫：一圈 5° 步长
    const double e = eval(d * CV_PI / 180.0, R_cand);
    if (e < best_err) { best_err = e; best_delta = d; }
  }
  for (int d = -4; d <= 4; ++d) {                        // 细扫：±4° 内 1° 步长
    const double e = eval((best_delta + d) * CV_PI / 180.0, R_cand);
    if (e < best_err) { best_err = e; best_delta += d; }
  }
  for (int d = -4; d <= 4; ++d) {                        // 再细一层：±1° 内 0.25°
    const double e = eval((best_delta + d * 0.25) * CV_PI / 180.0, R_cand);
    if (e < best_err) { best_err = e; best_delta += d * 0.25; }
  }
  if (best_err >= 1e18) return;                          // 一个合法候选都没有

  cv::Mat R_best;
  eval(best_delta * CV_PI / 180.0, R_best);
  cv::Rodrigues(R_best, a.rvec);
  a.yaw = std::atan2(R_best.at<double>(2, 0), R_best.at<double>(0, 0));
}

// 下面三个都是"可视化用的 3D 方框"，都定义在板坐标系里（x=0 平面），
// 画的时候由 Visualizer 用 PnP 位姿重投影到图像上。
// 它们【不参与解算】——只画。所以尺寸改错了只影响观感，不影响精度。
//
// 点序统一是：左上 右上 右下 左下（和 pnp_points 一致，方便对照）

// 左灯条：在 y = -hw 处的一条竖窄条（±HW_LB 宽，±HH_LB 高）
Box3D Solver::left_lightbar(ArmorType t) const {
  float hw  = (t == big) ? HW_BIG : HW_SMALL;
  float ctr = -hw;

  return {{
    {0.f, ctr - HW_LB, +HH_LB},
    {0.f, ctr + HW_LB, +HH_LB},
    {0.f, ctr + HW_LB, -HH_LB},
    {0.f, ctr - HW_LB, -HH_LB}
  }};
}

// 右灯条：同上，移到 y = +hw
Box3D Solver::right_lightbar(ArmorType t) const {
  float hw  = (t == big) ? HW_BIG : HW_SMALL;
  float ctr = +hw;

  return {{
    {0.f, ctr - HW_LB, +HH_LB},
    {0.f, ctr + HW_LB, +HH_LB},
    {0.f, ctr + HW_LB, -HH_LB},
    {0.f, ctr - HW_LB, -HH_LB}
  }};
}

// 数字区：两条灯条【之间】的那块区域（比灯条矮一些，HH_ARMOR < HH_LB）。
// 画面上就是框住装甲板上那个数字的框 —— 装甲板上写"2"就框住它。
Box3D Solver::digit_box(ArmorType t) const {
  float hw = (t == big) ? HW_BIG : HW_SMALL;
  float y0 = -hw + HW_LB;   // 左灯条内沿
  float y1 = +hw - HW_LB;   // 右灯条内沿

  return {{
    {0.f, y0, +HH_ARMOR},
    {0.f, y1, +HH_ARMOR},
    {0.f, y1, -HH_ARMOR},
    {0.f, y0, -HH_ARMOR}
  }};
}

//世界系点->像素系，用来画预测点
cv::Point2f Solver::project_world(const Eigen::Vector3d & p) const {
  // p 是【状态所在的系】里的一点（离线视频 = 相机系本身，默认旋转是单位阵；
  // ROS 节点接管云台时状态在世界系，这里先转回相机系再投影）
  const Eigen::Vector3d pc = cam_from_world_ * p;
  if (pc(2) <= 0) return {-1, -1};

  double fx = K_.at<double>(0,0);
  double fy = K_.at<double>(1,1);
  double cx = K_.at<double>(0,2);
  double cy = K_.at<double>(1,2);

  return {
    (float)(fx * pc(0) / pc(2) + cx),
    (float)(fy * pc(1) / pc(2) + cy)
  };
}

// 板系->像素系
std::vector<cv::Point2f> Solver::project(
  const std::vector<cv::Point3f> & pts3d,
  const cv::Vec3d & rvec,
  const cv::Vec3d & tvec) const {
  std::vector<cv::Point2f> out;
  cv::projectPoints(pts3d, rvec, tvec, K_, D_, out);
  return out;
}
