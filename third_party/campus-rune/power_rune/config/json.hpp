#pragma once

// ============================================================================
// 【移植改动 · 2026-10-04】配置路径
//
// 原版写死"可执行文件往上两层 / config / power_rune.json"（他们的 app 在
// output/ 下，config 在仓库根）。移植进 tdt-learning4.5 后离线工具和 ROS 节点
// 的可执行文件位置各不相同，改成：
//   ① 环境变量 CAMPUS_RUNE_CONFIG 指定（调试/对拍时切配置用）；
//   ② 否则用仓库里的固定路径
// ============================================================================

#include <cstdlib>
#include <filesystem>
#include <string>

#include "json/ReJson.hpp"

inline std::string campus_rune_config_path()
{
    if (const char *env = std::getenv("CAMPUS_RUNE_CONFIG"); env != nullptr && env[0] != '\0')
    {
        return env;
    }
    return "/home/king/tdt-learning4.5/third_party/campus-rune/config/power_rune.json";
}

inline ReJson J_POWER_RUNE(campus_rune_config_path());
