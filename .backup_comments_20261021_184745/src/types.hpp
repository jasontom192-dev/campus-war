#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>

// ============================================================================
// types.hpp —— 全工程共用的基础类型
//
// 这里只放"所有人都要用"的东西：颜色 / 兵种 / 板大小三个枚举 + Armor 结构体。
// 没有类的实现，也没有坐标系约定（坐标系见 target.hpp 顶部那张表）。
//
// ★ 三个枚举的【整数值都会被当作 track 的身份】用：
//   Tracker 用 (int)a.name 当 std::map 的 key，所以这些数值一旦改了，
//   等于换了跟踪器的目标身份体系。不要随便调顺序。
// ============================================================================

// 装甲板的颜色。数值顺序是【模型输出的约定】（见 detector.cpp 的映射），
// 不是我们随便定的：网络的 color_id = 0 蓝 / 1 红 / 2 白 / 3 紫，
// 前两个才对应这里的 red/blue（白紫是未激活态，detector 里直接丢掉）。
enum Color { red = 0, blue = 1};
const std::vector<std::string> COLORS = {"red", "blue"};

// 兵种（= 这块板属于哪台车）。
// ★ 数值同样是对外约定：detector.cpp 里那张 kArmorNames 表就是按模型输出的
//   armor_id（0..8）直接索引到这里来的，顺序必须和模型的 9 个类别一一对应。
// ★ 它同时是 Tracker 的 track key —— 所以"编号为 three 的板"永远落在同一个
//   track 里，这也是 tracker.cpp 那个"易看漏"注释说的事。
enum ArmorName {
  one = 0, two, three, four, five, sentry, outpost, base, not_armor
};
const std::vector<std::string> ARMOR_NAMES = {
  "one", "two", "three", "four", "five",
  "sentry", "outpost", "base", "not_armor"};

// 大小装甲板。只影响 PnP 用的那组 3D 点尺寸（solver.hpp 的 HW_BIG/HW_SMALL），
// 大板 230mm 宽、小板 135mm 宽 —— 用错尺寸会让解出来的距离系统性偏掉。
enum ArmorType { big = 0, small = 1 };
const std::vector<std::string> ARMOR_TYPES = {"big", "small"};



// 一块装甲板的全部信息。字段是【分两个阶段】填的，别指望刚检测完就都有效：
//
//   阶段一 —— Detector::detect() 填 2D 部分：
//       confidence / box / points(四个角点，左上→右上→右下→左下) / name / type / color
//   阶段二 —— Solver::solve() 填 3D 部分：
//       rvec / tvec / xyz_in_camera / yaw
//
// ★ 下游（Tracker）假设阶段二已经做完了，所以 main.cpp 和 ROS 节点都在
//   调用 tracker.update() 之前先 for(auto& a : armors) solver.solve(a)。
//   漏掉这一步的话 EKF 拿到的是全 0 的位置，会直接把 track 建在相机原点。
struct Armor {

  float confidence = 0;                      // 网络给的置信度（0~1）
  cv::Rect box;                              // 四角点的外接矩形（可视化/调试用）
  std::vector<cv::Point2f> points;           // 四个角点，顺序：左上 右上 右下 左下
                                             // ★ 顺序必须和 Solver::pnp_points() 的
                                             //   3D 点顺序一一对应，detector 里为此
                                             //   专门做过 [0,3,2,1] 重排
  ArmorName name = not_armor;
  ArmorType type = small;
  Color color = blue;

  cv::Vec3d rvec, tvec;                      // PnP 的旋转/平移（相机系，tvec 即板中心）
  Eigen::Vector3d xyz_in_camera = {0, 0, 0}; // 板中心在【相机系】(右/下/前)
                                             // ★ 名字叫 in_camera，但在 ROS 节点里
                                             //   会被 armors_to_world() 原地改写成世界系
  double yaw = 0;                            // 板法线的方位角 = atan2(nz, nx)，
                                             //   方向是【板 → 车中心】（不是板朝外的方向）
};

// 锁定目标的优先级：数字小的先打。
// 排序依据是"打它的收益/紧迫度"：前哨站会持续掉血所以最先打，
// 基地是胜负手排第二，普通机器人里三/四/五号板（步兵）比哨兵/英雄更值得打。
// Tracker::lock_target() 用它排序；优先级相同再按距离近的优先。
inline int get_armor_priority(ArmorName name) {
  switch (name) {
    case outpost: return 1;
    case base:    return 2;
    case three:
    case four:
    case five:    return 3;
    case sentry:  return 4;
    case one:     return 5;
    case two:     return 6;
    default:      return 99;
  }
}

// 把角度折回 (-π, π]。
//
// ★ 这个函数在整个工程里被调了几十次，是角度相关 bug 的总闸门：凡是
//   "两个角度相减"（EKF 的创新、数据关联的误差、板法线与车朝向的夹角）
//   都必须先过它一遍，否则 179° 和 -179° 相减会算出 358° 的假误差。
// ★ 注意边界是左开右闭：-π 会被折成 +π，保证同一个角度只有一个表示。
//
// ★ 第一行 fmod 是防"巨值/无穷"的：如果状态被卡尔曼增益带飞成 1e9 这种量级，
//   下面两个 while 要转上亿圈 —— 每帧卡几百毫秒、整个节点看起来像"卡死"
//   （实测：全自动跑前哨站时节点突然变哑巴、日志停更，就是这么来的）。
//   fmod 是 O(1)；±inf 经 fmod 变 NaN 后两个 while 条件恒假、原样返回 NaN，
//   下游判 !valid 就不瞄不打，安全落地。有限正常值的结果和原来逐位一致。
inline double limit_rad(double a) {
  a = std::fmod(a, 2.0 * CV_PI);
  while (a > CV_PI) a -= 2 * CV_PI;
  while (a <= -CV_PI) a += 2 * CV_PI;
  return a;
}
