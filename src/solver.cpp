//PnP 位姿解算、装甲板几何和投影
//板坐标系：x=法线（板→车中心），y=板宽，z=板高向下，原点在板中心；单位米
//角点顺序为 左上 右上 右下 左下，必须与 Armor::points 一致

#include "solver.hpp"
#include <Eigen/Geometry>

Solver::Solver(const cv::Mat & K, const cv::Mat & D) : K_(K), D_(D) {}

//solvePnP 的四点，z 取灯条外沿 HH_LB，不是整块板高度
std::vector<cv::Point3f> Solver::pnp_points(ArmorType t) {
  float hw = (t == big) ? HW_BIG : HW_SMALL;
  return {
    {0.f, -hw, -HH_LB},
    {0.f,  hw, -HH_LB},
    {0.f,  hw,  HH_LB},
    {0.f, -hw,  HH_LB}
  };
}

//解算单块装甲板：solvePnP，填 rvec/tvec/xyz_in_camera/yaw，再修正二义性
void Solver::solve(Armor & a) {
  auto pts3d = pnp_points(a.type);

  //IPPE 适合共面四点，但结果存在正反面二义性
  cv::solvePnP(
    pts3d, a.points, K_, D_,
    a.rvec, a.tvec, false, cv::SOLVEPNP_IPPE);

  //tvec 为板中心在相机系的位置
  cv::cv2eigen(a.tvec, a.xyz_in_camera);

  cv::Mat R;
  cv::Rodrigues(a.rvec, R);

  //第 0 列为板法线，yaw=atan2(nz,nx)，方向为板→车中心
  double nx = R.at<double>(0,0);
  double nz = R.at<double>(2,0);

  a.yaw = std::atan2(nz, nx);

  optimize_yaw(a);
}

//固定 tvec，扫描 yaw 并重投影打分，消除平面板 PnP 的正反面二义性
void Solver::optimize_yaw(Armor & a) const {
  const auto pts3d = pnp_points(a.type);
  const double px = a.tvec[0], py = a.tvec[1], pz = a.tvec[2];
  if (pz <= 0) return; //板在相机后方

  const double fx = K_.at<double>(0, 0), fy = K_.at<double>(1, 1);
  const double cx = K_.at<double>(0, 2), cy = K_.at<double>(1, 2);

  //假设装甲板竖直、法线水平，位姿只剩 yaw；按 yaw 重建旋转，不能用镜像解旋转得到
  //世界系中 n=(cosδ,0,sinδ)，板高方向为 (0,1,0)，再用 cam_from_world_ 转回相机系
  const Eigen::Matrix3d R_cw = cam_from_world_;

  auto eval = [&](double delta, cv::Mat & R_out) -> double {
    const double c = std::cos(delta), s = std::sin(delta);
    //R_W(δ) 的列向量为 n、板宽方向、板高方向
    Eigen::Matrix3d R_w;
    R_w << c,  s, 0.0,
           0.0, 0.0, 1.0,
           s, -c, 0.0;
    const Eigen::Matrix3d R_cam = R_cw * R_w;

    //法线必须指向远离相机的一侧，即只能看到装甲板正面
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
  for (int d = -180; d < 180; d += 5) { //粗扫一圈，步长 5°
    const double e = eval(d * CV_PI / 180.0, R_cand);
    if (e < best_err) { best_err = e; best_delta = d; }
  }
  for (int d = -4; d <= 4; ++d) { //±4° 内细扫 1°
    const double e = eval((best_delta + d) * CV_PI / 180.0, R_cand);
    if (e < best_err) { best_err = e; best_delta += d; }
  }
  for (int d = -4; d <= 4; ++d) { //±1° 内再扫 0.25°
    const double e = eval((best_delta + d * 0.25) * CV_PI / 180.0, R_cand);
    if (e < best_err) { best_err = e; best_delta += d * 0.25; }
  }
  if (best_err >= 1e18) return; //无合法候选

  cv::Mat R_best;
  eval(best_delta * CV_PI / 180.0, R_best);
  cv::Rodrigues(R_best, a.rvec);
  a.yaw = std::atan2(R_best.at<double>(2, 0), R_best.at<double>(0, 0));
}

//以下三个方框定义在板坐标系 x=0 平面，仅供显示；点序为 左上 右上 右下 左下

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

Box3D Solver::digit_box(ArmorType t) const {
  float hw = (t == big) ? HW_BIG : HW_SMALL;
  float y0 = -hw + HW_LB; //左灯条内沿
  float y1 = +hw - HW_LB; //右灯条内沿

  return {{
    {0.f, y0, +HH_ARMOR},
    {0.f, y1, +HH_ARMOR},
    {0.f, y1, -HH_ARMOR},
    {0.f, y0, -HH_ARMOR}
  }};
}

//状态系点投影到像素，用于预测点和准星
cv::Point2f Solver::project_world(const Eigen::Vector3d & p) const {
  //p 先由世界系转到相机系；相机后方返回 (-1,-1)
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

//板坐标系点按给定位姿投影到像素，包含畸变
std::vector<cv::Point2f> Solver::project(
  const std::vector<cv::Point3f> & pts3d,
  const cv::Vec3d & rvec,
  const cv::Vec3d & tvec) const {
  std::vector<cv::Point2f> out;
  cv::projectPoints(pts3d, rvec, tvec, K_, D_, out);
  return out;
}
