#pragma once

// ============================================================================
// 【移植扩展 · 2026-10-04】运行时可覆盖量
//
// RP-26Rune 的弹道初速原本只从配置（rune_ballistic_model.bullet_flying_speed）
// 读一次；而比赛里游戏每帧通过 AutoAimState.projectile_speed 给当前实际初速
// （规则附录 D：23.2–23.8 m/s，且会变）。这里给一个进程内的覆盖值：
// 调用方（我们的节点适配层）每帧 set 一次，算法侧读 effective_bullet_speed()
// 不设（或设 <=0）时行为与上游完全一致 —— 用配置值
// ============================================================================

namespace power_rune
{

inline double g_bullet_speed_override = 0.0;

inline void set_bullet_speed_override(double v)
{
    g_bullet_speed_override = v;
}

// config_value：配置里的 bullet_flying_speed
inline double effective_bullet_speed(double config_value)
{
    return g_bullet_speed_override > 0.0 ? g_bullet_speed_override : config_value;
}

}  // namespace power_rune
