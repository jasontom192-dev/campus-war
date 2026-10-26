#include "visualizer.hpp"
#include <sstream>
#include <iomanip>

//可视化：灯条/数字区框、锁定圈、预测箭头和瞄准点
//绿框为灯条，红粗框为当前瞄准板，蓝细框为其他板，黄准星为 aim_2d
//黄点为检测角点均值，与 PnP 投影框不重合说明位姿解算异常

Visualizer::Visualizer(Solver & solver) : solver_(solver) {}

//frame 当前未使用
void Visualizer::draw(cv::Mat & img, const std::list<Armor> & armors,
                      const Tracker & tracker, int frame,
                      const Aimer::AimResult * aim_result,
                      bool draw_prediction) {

  for (auto & a : armors) {

    //锁定按兵种编号判断，与 track key 一致
    bool is_locked = tracker.has_lock() && tracker.locked_id() == (int)a.name;

    //通过数据关联把检测板映射到车模型板编号，再与 aim_result 比较
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

    draw_box(img, solver_.left_lightbar(a.type),  a.rvec, a.tvec, {0,255,0});
    draw_box(img, solver_.right_lightbar(a.type), a.rvec, a.tvec, {0,255,0});

    //数字区：瞄准板用粗红框，其余用细蓝框
    if (is_aim_plate) {
      draw_box(img, solver_.digit_box(a.type), a.rvec, a.tvec, {0, 0, 255}, 3);
    } else {
      draw_box(img, solver_.digit_box(a.type), a.rvec, a.tvec, {255, 0, 0}, 1);
    }

    //兵种名画在数字框左下角外侧，corners[3] 为左下角
    Box3D db = solver_.digit_box(a.type);
    auto corners = solver_.project(db.corners, a.rvec, a.tvec);
    if (!corners.empty())
      cv::putText(img, ARMOR_NAMES[a.name], corners[3] + cv::Point2f(0, 15),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, {255,0,0}, 2);

    //板中心取四个检测角点均值，不是 PnP 投影
    cv::Point2f ct = (a.points[0] + a.points[1] + a.points[2] + a.points[3]) * 0.25f;
    cv::circle(img, ct, 4, {0, 255, 255}, -1);

    if (is_locked) {
      cv::circle(img, ct, 18, {0, 0, 255}, is_aim_plate ? 3 : 2);
    }

    if (!is_locked) continue;

    int vid = (int)a.name;
    auto it = tracker.tracks().find(vid);
    if (it == tracker.tracks().end()) continue;

    auto & t = it->second;
    if (t.diverged()) continue;

    Eigen::Vector4d obs = Target::camera_to_obs(a.xyz_in_camera, a.yaw);
    int armor_id = tracker.get_armor_plate_id(t, obs);

    if (draw_prediction) {
      //离线时该向量即相机系；世界系下由 project_world 的旋转转回相机系
      Eigen::Vector3d pred_cam = tracker.predict_armor(t, armor_id, 0.04);

      //下坠按 ½gt² 估计，飞行时间取水平距离/25；此处重力和初速为硬编码
      double dh = std::sqrt(pred_cam(0)*pred_cam(0) + pred_cam(2)*pred_cam(2));
      pred_cam(1) -= 0.5 * 9.81 * (dh / 25.0) * (dh / 25.0);

      cv::Point2f pred_pt = solver_.project_world(pred_cam);

      //相机后方返回 (-1,-1)
      if (pred_pt.x >= 0 && pred_pt.y >= 0)
        cv::arrowedLine(img, ct, pred_pt, {0, 0, 255}, 2, cv::LINE_AA, 0, 0.5);
    }
  }

  //黄圈和十字为子弹到达时刻的瞄准点，已含提前量和重力补偿
  if (aim_result && aim_result->valid) {
    cv::Point2f ap = aim_result->aim_2d;

    cv::circle(img, ap, 8, {0, 255, 255}, 2);
    cv::circle(img, ap, 2, {0, 255, 255}, -1);
    cv::line(img, cv::Point{int(ap.x - 12), int(ap.y)}, cv::Point{int(ap.x + 12), int(ap.y)}, {0, 255, 255}, 2);
    cv::line(img, cv::Point{int(ap.x), int(ap.y - 12)}, cv::Point{int(ap.x), int(ap.y + 12)}, {0, 255, 255}, 2);
  }
}

//用 PnP 位姿重投影 3D 框并绘制，projectPoints 包含畸变
void Visualizer::draw_box(cv::Mat & img, const Box3D & b3,
                          const cv::Vec3d & rvec, const cv::Vec3d & tvec,
                          const cv::Scalar & color, int thickness) {
  auto pts = solver_.project(b3.corners, rvec, tvec);
  if (pts.size() < 4) return;
  std::vector<cv::Point> ipts(pts.begin(), pts.end());
  std::vector<std::vector<cv::Point>> contours = {{ipts[0], ipts[1], ipts[2], ipts[3]}};
  cv::drawContours(img, contours, -1, color, thickness);
}
