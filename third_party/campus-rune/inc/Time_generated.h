#pragma once

// ============================================================================
// 【移植替换件 · 2026-10-04】foxglove::Time 的最小实现
//
// 原版是 foxglove SDK 用 flatbuffers 生成的类型（Time_generated.h，静态断言
// 卡死 flatbuffers 25.2.10）。RP-26Rune 里 Time 只做一件事：秒 + 纳秒的时间戳
// （construct(sec, nsec) + 取值 sec()/nsec()），不涉及任何序列化
// 移植时用这个等价的最小类型顶替，丢掉 flatbuffers/SDK 依赖
// 用法点：function.cpp 的 getNowTimestamp/timestampMinus/to_nanoseconds_since_epoch
// 以及各结构体里的 capture_timestamp / reference_timestamp
// ============================================================================

#include <cstdint>

namespace foxglove
{

struct Time
{
    Time() = default;
    Time(std::uint32_t sec, std::uint32_t nsec) : sec_(sec), nsec_(nsec) {}

    std::uint32_t sec() const { return sec_; }
    std::uint32_t nsec() const { return nsec_; }

private:
    std::uint32_t sec_ = 0;
    std::uint32_t nsec_ = 0;
};

}  // namespace foxglove
