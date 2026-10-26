#include "rune_campus.hpp"

#include "power_rune_interface.hpp" //RuneInput/RuneSendData
#include "PnPVariable.hpp" //全局 CAM/DIS
#include "rune_runtime_override.hpp" //弹速运行时覆盖
#include "common/power_rune_function.hpp" //PRF::timestamp_from_nanoseconds
#include "transform_tools/transform_tools.h"
#include "transform_tools/frame_id.h"

//power_rune.json 二值化阈值 180/180；颜色传错时轮廓为 0
RuneCampus::Result RuneCampus::process(const cv::Mat & frame, const std::list<RuneBlade> & blades,
                                   bool is_big,
                                   const cv::Mat & K, const cv::Mat & D,
                                   const Eigen::Matrix3d & R_world_from_cam,
                                   uint64_t stamp_ns, int my_color,
                                   double bullet_speed_mps) {
  Result out;

  //每帧覆盖弹速；<=0 清除覆盖，使用配置值
  power_rune::set_bullet_speed_override(bullet_speed_mps > 0.0 ? bullet_speed_mps : 0.0);

  //每帧写入同帧相机内参
  if (K.rows == 3 && K.cols == 3 && K.type() == CV_64F) {
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) CAM(r, c) = K.at<double>(r, c);
  }
  if (D.total() >= 5 && D.type() == CV_64F) {
    for (int c = 0; c < 5; ++c) DIS(0, c) = D.at<double>(0, c);
  }

  //TF：car(世界对齐) <- camera，平移取 0
  transform_tools::TFTree tf_tree;
  tf_tree.add_TF(transform_tools::TF(Eigen::Vector3d::Zero()), car_frame, camera_frame);
  tf_tree[camera_frame].set_rotation(R_world_from_cam);

  power_rune::RuneInput in;
  in.is_big_rune = is_big;
  in.ori_mat = frame; //必传，观测精修要裁图提轮廓
  in.tf_tree = tf_tree;
  in.timestamp = PRF::timestamp_from_nanoseconds(stamp_ns);
  in.cd_my_color = my_color; //0=红 1=蓝

  in.nn_rune_infos.reserve(blades.size());
  for (const RuneBlade & b : blades) {
    if (b.points.size() < 5) continue;
    power_rune::RuneInput::NNRuneInfo info;
    //点序与 RP-26Rune 的 NNRuneInfo 一致
    info.top = b.points[0];
    info.left = b.points[1];
    info.point_R = b.points[2];
    info.right = b.points[3];
    info.bottom = b.points[4];
    //RP-26Rune 只区分 0 与非 0(未击打/已击打)
    info.class_id = b.class_id;
    in.nn_rune_infos.push_back(info);
  }

  power_rune::process_power_rune(in);
  const power_rune::RuneSendData send = power_rune::get_rune_data(is_big);

  out.found = send.is_find_buff != 0;
  out.fire = send.is_enable_fire != 0;
  out.yaw = send.yaw; //rad，世界系，即游戏 yaw
  out.pitch = send.pitch; //rad，仰角(抬头正)
  return out;
}
