// PowerRuneInterface.cpp：内部实现
#include "PowerRuneProcessor.hpp"
#include "RuneDecisionModule.hpp"
#include "power_rune_interface.hpp"
#include "json.hpp"
namespace power_rune
{

// ============================================================================
// 【移植修复 · 2026-10-04】全局对象改成"首次使用时构造"（Meyers 单例）
//
// 原版是文件作用域的两个 static 全局：
//     static RuneDecisionModule rune_decision_module;
//     static PowerRuneProcessor rune_processor(rune_decision_module);
// 而 Projector 的构造函数会【拷贝】points_world.cpp 里的模型点/锚点全局
// （m_inactive_points_model_* 等）。两个 TU 的全局初始化顺序 C++ 不保证 ——
// 上游的 app 靠链接顺序侥幸躲过；我们以静态库方式接进来时顺序反了：
// 接口的全局先构造 → 拷贝到【空表】→ 锚点数量不足、Chamfer 优化没模型点，
// 整条链路静默返回（表现为 found 永远 0，且只有一句"未激活目标锚点数量不足"）
//
// 改成函数内静态：第一次调用时（进程里所有全局都初始化完了）才构造，
// 顺序问题消失。行为与原来完全一致
// ============================================================================
namespace
{
RuneDecisionModule &decision_module()
{
    static RuneDecisionModule module;
    return module;
}

PowerRuneProcessor &processor()
{
    static PowerRuneProcessor p(decision_module());
    return p;
}
}  // namespace

void process_power_rune(const RuneInput &input)
{
    processor().process_power_rune(input);
}

RuneSendData get_rune_data(bool is_big_rune)
{
    return decision_module().get_send_data(is_big_rune);
}

// 【移植扩展 · 2026-10-04】调试快照：见头文件说明
RuneDebugInfo debug_get_tracked()
{
    const RuneDecisionModule::DebugSnapshot s = decision_module().debug_snapshot();
    RuneDebugInfo out;
    out.valid = s.valid;
    out.is_big = s.is_big;
    out.phase = s.phase;
    out.omega = s.omega;
    for (int i = 0; i < 3; ++i)
    {
        out.rune_center[i] = s.rune_center(i);
        out.armor_center[i] = s.armor_center(i);
        out.start_vector[i] = s.start_vector(i);
        out.plane_normal[i] = s.plane_normal(i);
    }
    out.motion_A = s.motion_A;
    out.motion_B = s.motion_B;
    out.motion_b = s.motion_b;
    out.motion_C = s.motion_C;
    out.motion_w = s.motion_w;
    out.fire_cooldown_left = s.fire_cooldown_left;
    out.fire_time_acc = s.fire_time_acc;
    out.enable_fire = s.enable_fire;
    return out;
}

}
