#pragma once
#include <atomic>
#include <cstdint>
#include <mutex>
#include "PowerRuneBallisticModel.hpp"
#include "common/power_rune_global.hpp"
#include "power_rune_interface.hpp"

//符的火控决策器(小符/大符共用一套决策状态机，弹道模型在 get_send_data 中按模式分流)
class RuneDecisionModule
{
public:
    RuneDecisionModule();

    //更新决策器状态
    void update_decision_module(std::vector<RuneTarget> &&rune_targets);

    //获取火控决策所需的结构体
    power_rune::RuneSendData get_send_data(bool is_big_rune);

    // 【移植扩展 · 2026-10-04】调试快照：当前跟踪目标（车系三维量 + 相位/转速）
    // 只给离线对拍/在线排查用，不影响上游行为（同 rune_runtime_override.hpp 的定位）
    struct DebugSnapshot
    {
        bool valid = false;
        bool is_big = false;
        double phase = 0.0;                                  // 相位（rad，未归一）
        double omega = 0.0;                                  // 小符角速度（大符=0，用运动方程）
        Eigen::Vector3d rune_center = Eigen::Vector3d::Zero();   // 符心（车系：右/下/前，米）
        Eigen::Vector3d armor_center = Eigen::Vector3d::Zero();  // 靶心（同上）
        Eigen::Vector3d start_vector = Eigen::Vector3d::Zero();  // 0 相位方向（相位↔靶心的换算基准）
        Eigen::Vector3d plane_normal = Eigen::Vector3d::Zero();  // 符平面法向
        double motion_A = 0.0, motion_B = 0.0, motion_b = 0.0, motion_C = 0.0, motion_w = 0.0; // 大符
        // 开火状态机（排查"突然不打了"用）：冷却剩余 / 连续开火累计 / 放行位
        double fire_cooldown_left = 0.0;   // m_fire_enable_time_until（>0 = 还在冷却）
        double fire_time_acc = 0.0;        // m_fire_remaining_time（攒到阈值就进冷却）
        int    enable_fire = 0;            // 最近一次 get_send_data 的 is_enable_fire
    };
    DebugSnapshot debug_snapshot();

    //传入符心，起始向量，转轴，半径，相位。得到靶心
    static inline Eigen::Vector3d calculate_armor_module_center(
        const Eigen::Vector3d &rune_center,  // 符心
        const Eigen::Vector3d &start_vector, // 0 相位方向（需在符平面内）
        const Eigen::Vector3d &plane_normal, // 转轴 / 平面法向（单位向量）
        double rune_radius,                  // 半径
        double phase                         // 相位（rad）
    );

private:
    const char *decision_config_key(bool is_big_rune) const;//根据大小符选择对应的决策配置
    uint8_t rune_mode(bool is_big_rune) const;//根据大小符选择发送模式
    const char *log_prefix(bool is_big_rune) const;//日志前缀

    void solve_ballistic(const RuneTarget &rune_target, double algorithmic_time, bool is_big_rune);//统一弹道求解入口
    void solve_small_rune_ballistic(const RuneTarget &rune_target, double algorithmic_time);//小符运动方程
    void solve_big_rune_ballistic(const RuneTarget &rune_target, double algorithmic_time);//大符运动方程
    double compensate_pitch(const Eigen::Vector3d &predict_armor_center);//由于弹道不拟合，所以需要对pitch进行补偿
    void recover2rune_center(const RuneTarget &rune_target, bool is_big_rune,double cap2now_interval);//超时后退化为指向符心
    inline void reset_cooldown();//重置冷却
    bool need_change_target();//是否需要切换目标

    std::mutex m_data_mutex;//用于保护变量的访问，支持get_send_data函数的跨线程使用
    RuneTarget m_rune_target;//滤波器计算出的目标，同时也是上一次的目标
    foxglove::Time m_built_time;//建立这个决策器的时间(即连续追踪的符叶)
    std::deque<RuneTarget> m_pending_targets;//大符切换目标时的缓存
    bool m_is_valid = false;//是否是有效的决策器，即是否是通过数据构建且数据没有严重超时
    bool m_is_big_rune = false;//当前跟踪目标对应的大/小符模式
    power_rune::RuneSendData m_send_data;//发送的数据(不需要保护,因为只有get_send_data会进行读写)
    foxglove::Time m_send_time;//发送数据的时间
    std::atomic<double> m_fire_remaining_time;//已经连续开火的时间
    std::atomic<double> m_fire_enable_time_until;//距离fire_enable还有多久(冷却)
};
