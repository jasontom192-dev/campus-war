#pragma once
#include<opencv2/opencv.hpp>
#include <vector>
#include "time/time.hpp"
#include "transform_tools/transform_tools.h"

namespace power_rune
{

//符的输入
struct RuneInput
{
    bool is_big_rune;// 是否是大符
    cv::Mat ori_mat;//原图
    transform_tools::TFTree tf_tree;//tf_tree
    timetool::Timestamp timestamp;//时间戳
    int cd_my_color;
    struct NNRuneInfo
    {
        cv::Point top, left, right, bottom, point_R;
        int class_id; // 0是未击打，1是已击打
    };
    std::vector<NNRuneInfo> nn_rune_infos;//符数据

};


//符的输出
struct RuneSendData
{
    float yaw;              // 需要转到的yaw
    float pitch;            // 需要转到的pitch
    uint8_t is_find_buff;   // 是否发现能量机关
    uint8_t mode;           // aim: 1   small_buuf: 2   big_buff: 3  outpost: 4   hero: 5
    uint8_t is_enable_fire; // 是否允许开火
};


void process_power_rune(const RuneInput& input);
RuneSendData get_rune_data(bool is_big_rune);

// ============================================================================
// 【移植扩展 · 2026-10-04】调试快照：当前跟踪目标（车系三维量 + 相位/转速）
//
// 只给离线对拍 / 在线排查用（"认的符心/靶心/相位对不对"一眼可见），
// 不参与算法行为 —— 与 rune_runtime_override.hpp 同一性质的移植件
// 结构体原样转发 RuneDecisionModule::DebugSnapshot 的字段
// ============================================================================
struct RuneDebugInfo
{
    bool valid = false;
    bool is_big = false;
    double phase = 0.0;               // 相位（rad）
    double omega = 0.0;               // 小符角速度（大符=0）
    double rune_center[3] = {0, 0, 0};  // 车系：右/下/前，米
    double armor_center[3] = {0, 0, 0};
    double start_vector[3] = {0, 0, 0};  // 0 相位方向
    double plane_normal[3] = {0, 0, 0};  // 符平面法向
    double motion_A = 0.0, motion_B = 0.0, motion_b = 0.0, motion_C = 0.0, motion_w = 0.0;
    double fire_cooldown_left = 0.0;   // 冷却剩余（>0 = 在冷却，不放行）
    double fire_time_acc = 0.0;        // 本次连续开火累计（攒到阈值进冷却）
    int    enable_fire = 0;            // 最近一次 is_enable_fire
};
RuneDebugInfo debug_get_tracked();

}
