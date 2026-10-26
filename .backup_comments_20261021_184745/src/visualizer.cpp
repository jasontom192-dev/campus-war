#include "visualizer.hpp"
#include <sstream>
#include <iomanip>

// ============================================================================
// Visualizer —— 只负责把算法状态画到图像上，不参与任何解算
//
// 画面上有六种元素，看的时候按这个对照：
//   绿框 ×2      左右灯条的 3D 框（由 PnP 位姿重投影，不是检测框）
//   红框(粗)     当前【准备打】的那块板的数字区
//   蓝框(细)     其他板的数字区
//   蓝字         兵种名（one/two/outpost/base…），画在数字框左下角
//   黄实心点     板中心 = 四个角点的平均（纯 2D，直接用检测点算的）
//   红圈         锁定圈，画在锁定目标的板中心上；正在打的那块画得更粗
//   红箭头       EKF 预测的 0.04 s 后板位置（含重力下坠补偿），只有被锁定的车才画
//   黄准星       最终瞄准点 aim_2d（子弹到达时刻应该在的地方）
//
// ★ 一个容易搞混的地方：板中心那个黄点用的是【检测出来的 2D 角点平均】，
//   而绿框/数字框是用【PnP 位姿投影】出来的。两者不重合说明 PnP 解偏了
//   （或者关键的 3D 尺寸给错了），这是最直观的排查手段。
// ============================================================================

Visualizer::Visualizer(Solver & solver) : solver_(solver) {}

// frame 参数目前只用于将来在角上写帧号，现在没用到
void Visualizer::draw(cv::Mat & img, const std::list<Armor> & armors,
                      const Tracker & tracker, int frame,
                      const Aimer::AimResult * aim_result,
                      bool draw_prediction) {

  for (auto & a : armors) {

    // 锁定是按【兵种编号】比的，和 Tracker 内部 track 的 key 一致
    bool is_locked = tracker.has_lock() && tracker.locked_id() == (int)a.name;

    // 这块板是不是"这一帧准备打的那块"？
    // 判据：它在被锁定的车内部被数据关联判成了 aim_result->plate_id。
    // 绕这一圈（而不是直接比 plate_id）是因为 plate_id 是"车模型里的第几块板"，
    // 而画面里的 Armor 是"检测到的板"，两者要靠 get_armor_plate_id 对应起来。
    bool is_aim_plate = false;
    if (is_locked && aim_result && aim_result->valid) {
      auto it = tracker.tracks().find((int)a.name);
      if (it != tracker.tracks().end() && !it->second.diverged()) {
        Eigen::Vector4d obs = Target::camera_to_obs(a.xyz_in_camera, a.yaw);
        int armor_id = tracker.get_armor_plate_id(it->second, obs);
        if (armor_id == aim_result->plate_id)
          is_aim_plate = true;
      }
    }

    // 左右灯条用绿框（灯条在模型里是装甲板两侧那两条竖直的窄条）
    draw_box(img, solver_.left_lightbar(a.type),  a.rvec, a.tvec, {0,255,0});
    draw_box(img, solver_.right_lightbar(a.type), a.rvec, a.tvec, {0,255,0});

    // 数字区：正在打的那块用粗红框，其余用细蓝框
    if (is_aim_plate) {
      draw_box(img, solver_.digit_box(a.type), a.rvec, a.tvec, {0, 0, 255}, 3);
    } else {
      draw_box(img, solver_.digit_box(a.type), a.rvec, a.tvec, {255, 0, 0}, 1);
    }

    // 兵种名写在数字框左下角的外面一点。corners[3] 是左下角
    // （digit_box 的点序：左上 右上 右下 左下）
    Box3D db = solver_.digit_box(a.type);
    auto corners = solver_.project(db.corners, a.rvec, a.tvec);
    if (!corners.empty())
      cv::putText(img, ARMOR_NAMES[a.name], corners[3] + cv::Point2f(0, 15),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, {255,0,0}, 2);

    // 板中心 = 四个检测角点的平均（注意：是 2D 检测点，不是 PnP 投影点，见文件头）
    cv::Point2f ct = (a.points[0] + a.points[1] + a.points[2] + a.points[3]) * 0.25f;
    cv::circle(img, ct, 4, {0, 255, 255}, -1);

    if (is_locked) {
      cv::circle(img, ct, 18, {0, 0, 255}, is_aim_plate ? 3 : 2);
    }

    // 预测箭头只给锁定目标画；没锁定的板到此为止
    if (!is_locked) continue;

    // 找到它对应的 track（key = 兵种编号，和 Tracker 内部一致）
    int vid = (int)a.name;
    auto it = tracker.tracks().find(vid);
    if (it == tracker.tracks().end()) continue;

    auto & t = it->second;
    // 发散的 track 不画预测 —— 状态已经在乱跑，画出来只会误导排查
    if (t.diverged()) continue;

    Eigen::Vector4d obs = Target::camera_to_obs(a.xyz_in_camera, a.yaw);
    int armor_id = tracker.get_armor_plate_id(t, obs);

    // 基地是直瞄（不用 EKF），这条"预测点"箭头就没意义了 —— 只留黄准星
    if (draw_prediction) {
      // predict_armor 内部已经把状态系的点转成【相机排版(右/下/前)】了
      // （tracker.cpp 末尾那句 pseudo_to_camera），所以这里直接当相机系用：
      //   (0)=右 (1)=下 (2)=前，水平距离取 右&前，重力下坠减在"下"上（箭头画高一点）
      // 世界系那版运行时这个向量是世界排版，交给 project_world 里的旋转转回相机系。
      Eigen::Vector3d pred_cam = tracker.predict_armor(t, armor_id, 0.04);

      // 下坠 = ½gt²，飞行时间用 dist/25 粗估。这里只是个示意箭头，不像 Aimer
      // 那样做不动点迭代。
      // ⚠ 25.0（初速）和 9.81（重力）都是硬编码 —— 和 Aimer 里的 g_=9.7833
      //   不同步，且初速变了这里不会跟着变（影响很小，知道就行）。
      double dh = std::sqrt(pred_cam(0)*pred_cam(0) + pred_cam(2)*pred_cam(2));
      pred_cam(1) -= 0.5 * 9.81 * (dh / 25.0) * (dh / 25.0);

      cv::Point2f pred_pt = solver_.project_world(pred_cam);

      // project_world 在点在相机背后（z ≤ 0）时返回 (-1,-1) 作哨兵值，
      // 靠这个判掉"箭头指向画面背后"的情况，否则会画出一条横穿画面的长线
      if (pred_pt.x >= 0 && pred_pt.y >= 0)
        cv::arrowedLine(img, ct, pred_pt, {0, 0, 255}, 2, cv::LINE_AA, 0, 0.5);
    }
  }

  // 最终瞄准点：黄圈 + 十字。它是 Aimer 解出的 aim_2d —— 子弹【到达时刻】
  // 板应该在的位置（已含提前量和重力补偿），所以通常落在实际板的前方一点。
  // 准星与板上黄点（当前位置）的差距，就是这一帧的提前量。
  if (aim_result && aim_result->valid) {
    cv::Point2f ap = aim_result->aim_2d;

    cv::circle(img, ap, 8, {0, 255, 255}, 2);
    cv::circle(img, ap, 2, {0, 255, 255}, -1);
    cv::line(img, cv::Point{int(ap.x - 12), int(ap.y)}, cv::Point{int(ap.x + 12), int(ap.y)}, {0, 255, 255}, 2);
    cv::line(img, cv::Point{int(ap.x), int(ap.y - 12)}, cv::Point{int(ap.x), int(ap.y + 12)}, {0, 255, 255}, 2);
  }
}

// b3 是一个 3D 盒子的四个角（如 left_lightbar/digit_box），
// 用 PnP 解出的 rvec/tvec 重投影成像素再画四边形。
// ★ 它用的是 cv::projectPoints（带畸变），而不是 Solver::project_world ——
//   前者才和 PnP 完全配套，框才能贴住板；后者是给"状态系的点"用的简化投影。
void Visualizer::draw_box(cv::Mat & img, const Box3D & b3,
                          const cv::Vec3d & rvec, const cv::Vec3d & tvec,
                          const cv::Scalar & color, int thickness) {
  auto pts = solver_.project(b3.corners, rvec, tvec);
  if (pts.size() < 4) return;
  std::vector<cv::Point> ipts(pts.begin(), pts.end());
  std::vector<std::vector<cv::Point>> contours = {{ipts[0], ipts[1], ipts[2], ipts[3]}};
  cv::drawContours(img, contours, -1, color, thickness);
}
