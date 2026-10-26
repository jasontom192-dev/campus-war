//符叶 PnP 解算实现
//26 模型 5 点顺序 top/left/R/right/bottom，R 为机关中心，四角环序 0->1->4->3
//26 环序与 OBJECT_POINTS 相反，镜像解重投影相同，故用 8 排列+法线朝相机自标定

#include "rune_solver.hpp"
#include "rune_geometry.hpp"

#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp> //cv::cv2eigen
#include <cmath>

namespace {

//buff 系模型点(m)：0..3 为上/右/下/左四角，4 叶心，5 臂内侧，6 机关中心
const std::array<cv::Point3f, 7> kObjectPoints = {{
    {0.f,  0.f,    0.827f},
    {0.f,  0.127f, 0.700f},
    {0.f,  0.f,    0.573f},
    {0.f, -0.127f, 0.700f},
    {0.f,  0.f,    0.700f},
    {0.f,  0.f,    0.220f},
    {0.f,  0.f,    0.000f},
}};

//26 模型四角下标，环序 0->1->4->3
constexpr int kCornerIdx[4] = {0, 1, 4, 3};

//4 个起点 x 2 个方向共 8 个候选排列
const std::array<std::array<int, 4>, 8> kPerms = [] {
  std::array<std::array<int, 4>, 8> p{};
  int n = 0;
  for (int start = 0; start < 4; ++start) {
    const std::array<int, 4> base = {start, (start + 1) % 4, (start + 2) % 4, (start + 3) % 4};
    p[n++] = base;
    p[n++] = {base[3], base[2], base[1], base[0]};
  }
  return p;
}();

//自标定接受门槛：重投影误差<60px 才锁定排列
constexpr double kCalibAcceptPx = 60.0;

}

RuneSolver::RuneSolver(const cv::Mat & K, const cv::Mat & D) : K_(K), D_(D) {}

//perm[k] 是模型第 k 角在 26 四角中的序号，再经 kCornerIdx 映射
std::vector<cv::Point2f> RuneSolver::order_pts(
    const RuneBlade & blade, const std::array<int, 4> & perm) const {
  std::vector<cv::Point2f> out(4);
  for (int k = 0; k < 4; ++k)
    out[k] = blade.points[kCornerIdx[perm[k]]];
  return out;
}

bool RuneSolver::solve_pnp(const std::vector<cv::Point2f> & pts4,
                           cv::Mat & rvec, cv::Mat & tvec) const {
  //SOLVEPNP_IPPE 专为平面四点设计，通用迭代法在共面点上会跳变
  std::vector<cv::Point3f> obj(kObjectPoints.begin(), kObjectPoints.begin() + 4);
  return cv::solvePnP(obj, pts4, K_, D_, rvec, tvec, false, cv::SOLVEPNP_IPPE);
}

double RuneSolver::reproj_err(const cv::Mat & rvec, const cv::Mat & tvec,
                              const RuneBlade & blade) const {
  //叶心对四角均值，机关中心对第 2 点，后者权重 0.5
  const std::vector<cv::Point3f> obj = {kObjectPoints[4], kObjectPoints[6]};
  std::vector<cv::Point2f> proj;
  cv::projectPoints(obj, rvec, tvec, K_, D_, proj);

  const cv::Point2f blade_det = blade.blade_center();
  const cv::Point2f r_det     = blade.rune_center();

  const double d0 = std::hypot(proj[0].x - blade_det.x, proj[0].y - blade_det.y);
  const double d1 = std::hypot(proj[1].x - r_det.x,     proj[1].y - r_det.y);
  return d0 + 0.5 * d1;
}

std::array<int, 4> RuneSolver::corner_order_for(const RuneBlade & blade) const {
  if (locked_valid_) return locked_;

  double best_err = 1e18;
  std::array<int, 4> best{0, 1, 2, 3};
  double best_nz = 0.0;
  bool found = false;

  //8 个候选各解一次，收集重投影误差和法线 z 分量
  struct Cand { double err, nz; std::array<int, 4> perm; };
  std::vector<Cand> cands;
  cands.reserve(kPerms.size());

  for (const auto & perm : kPerms) {
    const std::vector<cv::Point2f> pts4 = order_pts(blade, perm);
    cv::Mat rvec, tvec;
    if (!solve_pnp(pts4, rvec, tvec)) continue;

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    //buff x 轴(符面法线)在相机系的 z 分量，<0 表示朝向相机
    const double nz = R.at<double>(2, 0);

    cands.push_back({reproj_err(rvec, tvec, blade), nz, perm});
  }
  if (cands.empty()) return {0, 1, 2, 3};
  (void)best_nz;

  //正面优先取误差最小；无正面候选则退回全部候选
  const Cand * pick = nullptr;
  for (const Cand & c : cands) {
    if (c.nz >= 0.0) continue;
    if (!pick || c.err < pick->err) pick = &c;
  }
  if (!pick) {
    for (const Cand & c : cands)
      if (!pick || c.err < pick->err) pick = &c;
  }
  best_err = pick->err;
  best     = pick->perm;

  //误差<60px 才锁定
  if (best_err < kCalibAcceptPx) {
    locked_       = best;
    locked_valid_ = true;
  }
  (void)found;
  return best;
}

RuneSolution RuneSolver::solve(const RuneBlade & blade,
                               const Eigen::Matrix3d & R_gimbal2world) {
  RuneSolution sol;

  const std::array<int, 4> perm = corner_order_for(blade);
  const std::vector<cv::Point2f> pts4 = order_pts(blade, perm);

  cv::Mat rvec, tvec;
  if (!solve_pnp(pts4, rvec, tvec)) return sol;

  const double err = reproj_err(rvec, tvec, blade);

  //未锁定且本帧误差>60px：按丢帧返回
  if (!locked_valid_ && err > kCalibAcceptPx) return sol;
  last_reproj_ = err;

  cv::Mat R_b2c_cv;
  cv::Rodrigues(rvec, R_b2c_cv);
  Eigen::Matrix3d R_b2c;
  cv::cv2eigen(R_b2c_cv, R_b2c);
  const Eigen::Vector3d t_b2c(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

  //相机与云台同轴(R=I,t=0)，世界位姿 = R_gimbal2world * 相机位姿
  const Eigen::Matrix3d R_b2w = R_gimbal2world * R_b2c;
  const Eigen::Vector3d t_b2w = R_gimbal2world * t_b2c;

  const Eigen::Vector3d blade_buff(kObjectPoints[4].x, kObjectPoints[4].y,
                                   kObjectPoints[4].z);
  const Eigen::Vector3d blade_xyz = R_b2w * blade_buff + t_b2w;

  sol.valid        = true;
  sol.xyz          = t_b2w;
  sol.ypd          = rune::xyz2ypd(t_b2w);
  sol.ypr          = rune::eulers_zyx(R_b2w);
  sol.blade_xyz    = blade_xyz;
  sol.blade_ypd    = rune::xyz2ypd(blade_xyz);
  sol.R_buff2world = R_b2w;
  sol.reproj       = err;
  sol.corner_order = {kCornerIdx[perm[0]], kCornerIdx[perm[1]],
                      kCornerIdx[perm[2]], kCornerIdx[perm[3]]};
  return sol;
}

cv::Point2f RuneSolver::project_world(const Eigen::Vector3d & p_world,
                                      const Eigen::Matrix3d & R_gimbal2world) const {
  //相机与云台同轴，世界->相机的旋转是 R_gimbal2world 的转置
  const Eigen::Vector3d pc = R_gimbal2world.transpose() * p_world;
  if (pc(2) <= 1e-6) return {-1.f, -1.f};

  const double fx = K_.at<double>(0, 0), fy = K_.at<double>(1, 1);
  const double cx = K_.at<double>(0, 2), cy = K_.at<double>(1, 2);
  return {static_cast<float>(fx * pc(0) / pc(2) + cx),
          static_cast<float>(fy * pc(1) / pc(2) + cy)};
}
