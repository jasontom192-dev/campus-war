// ============================================================================
// rune_solver.cpp —— 符叶位姿解算的实现（照 tdt-campus-test 的 solver.py 移植）
//
// 核心难点只有一个：【26 模型四角的环序和 sp_vision 模型点环序相反】。
//   26 模型（本文件）：top / left / R / right / bottom，四角环序 0→1→4→3
//   OBJECT_POINTS     ：上 → 右 → 下 → 左（buff 系里是顺时针）
//   两个环序一正一反，直接对映会解出【镜像位姿】—— 而平面四点 PnP 的镜像解
//   重投影误差完全相等（实测 11.04 vs 11.04 px），光看误差分辨不出来。
//
//   所以用两个手段：
//     ① 8 个候选排列（4 个起点 × 2 个方向）各解一次，取重投影最小的；
//     ② tiebreak：符面法线（buff 系 x 轴）必须朝向相机 —— 能量机关只能从正面看到，
//        法线背对相机的解物理上不存在。镜像解恰好就是法线反号的那支。
//   锁定一次就复用（corner_order 固定），之后每帧只解一次 PnP。
//
//   附带一个巧合但重要的事实：叶心在 buff 系是 (0,0,0.7)，x 分量为 0，
//   而镜像解只把 x 轴翻号 —— 所以两支解算出的【叶心 3D 坐标完全相同】
//   （实测 292 帧差 0.000 m）。挑正面解只是让姿态量（ypr 里的 yaw/roll）干净些。
// ============================================================================

#include "rune_solver.hpp"
#include "rune_geometry.hpp"

#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>   // cv::cv2eigen（车的 solver.hpp 里也是这么引的）
#include <cmath>

namespace {

// buff 系模型点（米）。索引含义（和 solver.py 的 OBJECT_POINTS 一致）：
//   0..3 四角（上 / 右 / 下 / 左，环序）  4 叶心 = R 标中心（半径 0.7）
//   5 臂内侧点   6 机关中心
// ★ 四角只用来解 PnP；4/6 用来做重投影打分；5 目前只给可视化留的。
const std::array<cv::Point3f, 7> kObjectPoints = {{
    {0.f,  0.f,    0.827f},
    {0.f,  0.127f, 0.700f},
    {0.f,  0.f,    0.573f},
    {0.f, -0.127f, 0.700f},
    {0.f,  0.f,    0.700f},
    {0.f,  0.f,    0.220f},
    {0.f,  0.f,    0.000f},
}};

// 26 模型四角点在自身数组里的下标（环序 0->1->4->3）
constexpr int kCornerIdx[4] = {0, 1, 4, 3};

// 候选排列：4 个循环起点 × 2 个方向 = 8 个
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

// 自标定接受门槛：重投影误差小于它才锁定排列（和 Python 版一致，60px）
constexpr double kCalibAcceptPx = 60.0;

}  // namespace

RuneSolver::RuneSolver(const cv::Mat & K, const cv::Mat & D) : K_(K), D_(D) {}

// 按排列把 5 点检测重排成 PnP 要的 4 点。
// perm[k] 表示"模型第 k 个角点"对应 26 模型的第几个角点（再经 kCornerIdx 映射）。
std::vector<cv::Point2f> RuneSolver::order_pts(
    const RuneBlade & blade, const std::array<int, 4> & perm) const {
  std::vector<cv::Point2f> out(4);
  for (int k = 0; k < 4; ++k)
    out[k] = blade.points[kCornerIdx[perm[k]]];
  return out;
}

bool RuneSolver::solve_pnp(const std::vector<cv::Point2f> & pts4,
                           cv::Mat & rvec, cv::Mat & tvec) const {
  // SOLVEPNP_IPPE：专为平面四点设计；通用迭代法在共面点上会跳变
  // （和车的 Solver 用 IPPE 是同一个理由）
  std::vector<cv::Point3f> obj(kObjectPoints.begin(), kObjectPoints.begin() + 4);
  return cv::solvePnP(obj, pts4, K_, D_, rvec, tvec, false, cv::SOLVEPNP_IPPE);
}

double RuneSolver::reproj_err(const cv::Mat & rvec, const cv::Mat & tvec,
                              const RuneBlade & blade) const {
  // 打分口径（和 Python 一致）：叶心 (0,0,0.7) 对"四角均值"，机关中心 (0,0,0)
  // 对"第 2 点"，后者权重 0.5 —— 机关中心离得远、像素误差天然大，压一压。
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

  // 先收集 8 个候选，再把"法线朝相机"的那几个放进候选池
  struct Cand { double err, nz; std::array<int, 4> perm; };
  std::vector<Cand> cands;
  cands.reserve(kPerms.size());

  for (const auto & perm : kPerms) {
    const std::vector<cv::Point2f> pts4 = order_pts(blade, perm);
    cv::Mat rvec, tvec;
    if (!solve_pnp(pts4, rvec, tvec)) continue;

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    // buff x 轴（符面法线）在相机系里的 z 分量：< 0 表示指向相机（正面）
    const double nz = R.at<double>(2, 0);

    cands.push_back({reproj_err(rvec, tvec, blade), nz, perm});
  }
  if (cands.empty()) return {0, 1, 2, 3};
  (void)best_nz;

  // 正面优先；一个正面候选都没有（理论上不该发生）就退回全部候选
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

  // 误差够小才认（否则这一帧的姿态量先不定，等更清楚的帧）
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

  // 还没自标定成功、这一帧又解得离谱 → 当丢帧（调用方按没解出处理）
  if (!locked_valid_ && err > kCalibAcceptPx) return sol;
  last_reproj_ = err;

  cv::Mat R_b2c_cv;
  cv::Rodrigues(rvec, R_b2c_cv);
  Eigen::Matrix3d R_b2c;
  cv::cv2eigen(R_b2c_cv, R_b2c);
  const Eigen::Vector3d t_b2c(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

  // 相机 → 云台 → 世界。本工程相机与云台同轴（外参单位阵、零平移），
  // 所以 R_cam2gimbal = I、t_cam2gimbal = 0，直接乘云台旋转即可。
  // （tdt-campus-test 里留了外参配置，我们这台机器用不上，先不引入。）
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
  // 相机 = 云台（同轴），所以 世界→相机 的旋转就是 R_gimbal2world 的转置
  const Eigen::Vector3d pc = R_gimbal2world.transpose() * p_world;
  if (pc(2) <= 1e-6) return {-1.f, -1.f};

  const double fx = K_.at<double>(0, 0), fy = K_.at<double>(1, 1);
  const double cx = K_.at<double>(0, 2), cy = K_.at<double>(1, 2);
  return {static_cast<float>(fx * pc(0) / pc(2) + cx),
          static_cast<float>(fy * pc(1) / pc(2) + cy)};
}
