#pragma once

// ============================================================================
// rune_rp26.hpp —— RP-26Rune 打符算法的对接层（2026-10-04 移植）
//
// 干什么：把"我们的检测结果 + 相机内外参 + 云台姿态"翻译成 RP-26Rune 的
// RuneInput，调它的 process_power_rune()，再把 RuneSendData 翻译回我们能用的
// 角度/开火。算法一行没改（third_party/rp26_rune/，只换掉了 foxglove 可视化、
// ImgViz 和 foxglove::Time 三个周边件）。
//
// 坐标系对接（★ 这是整套对接里唯一需要想清楚的地方）：
//   RP-26Rune 的"car 系"= (x 右, y 下, z 前)，且是【世界对齐】的定系 —— 正好和
//   我们节点的世界系（pseudo：右/前/上 → 相机排版就是右/下/前）逐位一致。
//   所以：R_car_from_camera = 我们节点的 world_from_cam(云台 yaw, pitch)
//   （该矩阵的语义就是"相机排版向量 → 世界排版向量"，直接塞进 TF 即可）。
//   输出：out.yaw = atan2(右, 前) = 我们的 world_yaw（游戏 yaw，直接用）；
//         out.pitch = 仰角（抬头正）→ 游戏要发 -pitch。
//
// 相机内参：RP-26Rune 用两个全局 CAM/DIS（原来在静态初始化时从配置读一次）。
// 规则 7.1 要求优先同帧 camera_info —— 所以本类每帧把 K/D 写进那两个全局，
// 再喂帧（单线程调用，安全）。
//
// 移植配置差异（config/power_rune.json，2026-10-04 按本游戏实测改）：
//   feature_extract 两个二值化阈值 50/62 → 180/180。
//   我们游戏里灯臂和装甲板圆环在颜色差分图上【连通】，低阈值下合并成一个
//   轮廓 → 装甲板面积核对不上被滤掉 → 全链路静默返回 found=0。
//   实测：170~200 稳定（差分亮核 200~248，≥210 掩膜全灭）；滤镜相关键别乱动。
// 颜色语义：cd_my_color = 符灯颜色 = 我方阵营色（红蓝双方能量机关独立）。
//   跑错颜色同样是"轮廓 0 个 → found 恒 0"，且无报错 —— 排查时先看这个。
// ============================================================================

#include <Eigen/Dense>
#include <cstdint>
#include <list>
#include <opencv2/opencv.hpp>

#include "rune_detector.hpp"   // 我们的 RuneBlade（5 关键点 + class_id）

class RuneRp26 {
public:
  struct Result {
    bool found = false;        // is_find_buff：算法认为找到了符
    bool fire = false;         // is_enable_fire：射击许可
    double yaw = 0.0;          // 【弧度】世界系方位角 = atan2(右, 前)，可直接当游戏 yaw
    double pitch = 0.0;        // 【弧度】发射仰角（抬头为正）；游戏要发 -pitch
  };

  // 一帧。
  //   frame           : 本帧原图（BGR）。★ 算法真的要用它：观测精修会在检测框
  //                     内裁 view → 颜色差分 → 提轮廓，别传空图
  //   blades          : 本帧检测到的全部符叶（含已激活的 —— 姿态精修也要用）
  //   is_big          : 打大符？
  //   K, D            : 同帧相机内参（规则 7.1）
  //   R_world_from_cam: 云台姿态构造的世界旋转（= auto_aim_node 的 world_from_cam）
  //   stamp_ns        : 该帧的采集时间戳（系统时钟纳秒）
  //   my_color        : 我方颜色 RP26 约定：0 = 红，1 = 蓝（= 符灯颜色：红蓝双方
  //                     能量机关独立，看到的是自己方的符，灯就是自己方的颜色）
  //   bullet_speed_mps: 本帧弹速（m/s）。>0 时覆盖配置里的 bullet_flying_speed
  //                     （规则附录 B：游戏每帧给最近一发的实际初速，23.2~23.8）；
  //                     <=0 表示"没值"，用配置值（23.5/24.5）。
  Result process(const cv::Mat & frame, const std::list<RuneBlade> & blades, bool is_big,
                 const cv::Mat & K, const cv::Mat & D,
                 const Eigen::Matrix3d & R_world_from_cam,
                 uint64_t stamp_ns, int my_color, double bullet_speed_mps = -1.0);
};
