#include "rune_rp26.hpp"

#include "power_rune_interface.hpp"     // RP-26Rune 公开接口（RuneInput/RuneSendData）
#include "PnPVariable.hpp"              // 全局 CAM / DIS
#include "rune_runtime_override.hpp"    // 【移植扩展】弹速运行时覆盖
#include "common/power_rune_function.hpp"   // PRF::timestamp_from_nanoseconds
#include "transform_tools/transform_tools.h"
#include "transform_tools/frame_id.h"

RuneRp26::Result RuneRp26::process(const cv::Mat & frame, const std::list<RuneBlade> & blades,
                                   bool is_big,
                                   const cv::Mat & K, const cv::Mat & D,
                                   const Eigen::Matrix3d & R_world_from_cam,
                                   uint64_t stamp_ns, int my_color,
                                   double bullet_speed_mps) {
  Result out;

  // ---- 弹速：每帧覆盖（游戏的 projectile_speed / meta 里的 v/3）----
  // 不给值（<=0）就清掉覆盖，弹道回落配置里的 bullet_flying_speed。
  power_rune::set_bullet_speed_override(bullet_speed_mps > 0.0 ? bullet_speed_mps : 0.0);

  // ---- 相机内外参：每帧写进 RP-26Rune 的全局（规则 7.1 同帧 camera_info）----
  if (K.rows == 3 && K.cols == 3 && K.type() == CV_64F) {
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) CAM(r, c) = K.at<double>(r, c);
  }
  if (D.total() >= 5 && D.type() == CV_64F) {
    for (int c = 0; c < 5; ++c) DIS(0, c) = D.at<double>(0, c);
  }

  // ---- TF 树：car（世界对齐定系）← camera，只这一条边 ----
  // 旋转语义 = "相机排版向量 → 世界排版向量"，正是我们节点的 world_from_cam。
  // 平移取 0：题目前提 2 —— 枪口与相机原点重合（游戏里相机就在云台上）。
  transform_tools::TFTree tf_tree;
  tf_tree.add_TF(transform_tools::TF(Eigen::Vector3d::Zero()), car_frame, camera_frame);
  tf_tree[camera_frame].set_rotation(R_world_from_cam);

  // ---- 填 RuneInput ----
  power_rune::RuneInput in;
  in.is_big_rune = is_big;
  in.ori_mat = frame;              // ★ 必传：观测精修要在这张图上裁 view 提轮廓
  in.tf_tree = tf_tree;
  in.timestamp = PRF::timestamp_from_nanoseconds(stamp_ns);
  in.cd_my_color = my_color;       // 0=红 1=蓝（RP26 约定，与我们的 Color 枚举一致）

  in.nn_rune_infos.reserve(blades.size());
  for (const RuneBlade & b : blades) {
    if (b.points.size() < 5) continue;
    power_rune::RuneInput::NNRuneInfo info;
    // 我们的点序 top/left/R/right/bottom 和 26 模型契约一致（rune_detector.hpp），
    // 和 RP-26Rune 的 NNRuneInfo 逐位对应。
    info.top = b.points[0];
    info.left = b.points[1];
    info.point_R = b.points[2];
    info.right = b.points[3];
    info.bottom = b.points[4];
    // class_id：0=未激活（待击打）1=小符已激活 2=大符已激活。
    // RP-26Rune 只区分"0 / 非 0"（未击打 / 已击打），直接透传即可。
    info.class_id = b.class_id;
    in.nn_rune_infos.push_back(info);
  }

  // ---- 跑算法 + 取结果 ----
  power_rune::process_power_rune(in);
  const power_rune::RuneSendData send = power_rune::get_rune_data(is_big);

  out.found = send.is_find_buff != 0;
  out.fire = send.is_enable_fire != 0;
  out.yaw = send.yaw;              // 弧度，世界系（= 游戏 yaw 的弧度制）
  out.pitch = send.pitch;          // 弧度，仰角（抬头正）
  return out;
}
