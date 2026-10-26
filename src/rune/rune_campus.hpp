#pragma once

//CampusRune 对接层（上游 RP-26Rune 移植）：检测结果、内外参、云台旋转 -> RuneInput -> RuneSendData
//car 系=世界对齐(右/下/前)，R_car_from_camera 即节点的 world_from_cam
//cd_my_color=符灯颜色=我方阵营色(0 红/1 蓝)；每帧写全局 CAM/DIS，单线程调用

#include <Eigen/Dense>
#include <cstdint>
#include <list>
#include <opencv2/opencv.hpp>

#include "rune_detector.hpp" //RuneBlade

class RuneCampus {
public:
  struct Result {
    bool found = false; //算法认为找到了符
    bool fire = false; //射击许可
    double yaw = 0.0; //rad，世界系 atan2(右,前)，可直接当游戏 yaw
    double pitch = 0.0; //rad，仰角(抬头正)；发给游戏取 -pitch
  };

  //frame 不能为空(轮廓精修要用)；stamp_ns 为采集时间；bullet_speed_mps<=0 时用配置值
  Result process(const cv::Mat & frame, const std::list<RuneBlade> & blades, bool is_big,
                 const cv::Mat & K, const cv::Mat & D,
                 const Eigen::Matrix3d & R_world_from_cam,
                 uint64_t stamp_ns, int my_color, double bullet_speed_mps = -1.0);
};
