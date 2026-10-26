#pragma once

// ============================================================================
// foxglove_viz 替换件（2026-10-04）：空实现的同名单头
//
// 上游的 foxglove_viz 是"调试数据 → Foxglove Studio"的推送层，依赖 SDK 的
// 编译产物 libfoxglove.a（没随仓库开源）。本文件提供完全相同的对外 API，
// 内部全是空操作：
//   · foxglove::schemas::{Vector3, Quaternion, Color, Pose, SpherePrimitive,
//     ArrowPrimitive} —— power_rune 实际用到的 6 个类型/字段（原 SDK 的
//     schemas.hpp 有 4000+ 行，这里按用到的字段最小化）
//   · foxglove_viz::DataPublisher<Ts...>（publish / publish_with_time）
//   · foxglove_viz::EntityPublisher（publish_spheres / publish_arrows）
//   · foxglove_viz::FoxgloveServer + global_foxglove_server()
//
// power_rune 的调用方式：Viz::log<主题>(...) / Viz::publish_spheres<主题>(...)
// —— 全部经 PowerRuneVisualizeManager 中转，本桩只需覆盖那一层的 API 面
// 原版实现留在 ../../foxglove_viz.orig/ 备查
// ============================================================================

#include <cstdint>
#include <optional>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

// ---- foxglove::schemas 的最小桩（只保留被用到的字段）----
namespace foxglove::schemas
{

struct Vector3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Quaternion
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
};

struct Color
{
    double r = 0.0;
    double g = 0.0;
    double b = 0.0;
    double a = 0.0;
};

struct Pose
{
    // ★ 和真 SDK 一致：嵌套字段都是 std::optional（调用方写
    //   `arrow.pose->position = ...`，靠 optional::operator->）
    std::optional<Vector3> position;
    std::optional<Quaternion> orientation;
};

struct SpherePrimitive
{
    std::optional<Pose> pose;
    std::optional<Vector3> size;
    std::optional<Color> color;
};

struct ArrowPrimitive
{
    std::optional<Pose> pose;
    double shaft_length = 0.0;
    double shaft_diameter = 0.0;
    double head_length = 0.0;
    double head_diameter = 0.0;
    std::optional<Color> color;
};

}  // namespace foxglove::schemas

namespace foxglove_viz
{

class FoxgloveServer;

/// @brief 场景实体发布器（空实现）：接受球/箭头图元，什么也不做
class EntityPublisher
{
public:
    using Ptr = std::shared_ptr<EntityPublisher>;

    void publish_spheres(std::span<const foxglove::schemas::SpherePrimitive>) {}
    void publish_arrows(std::span<const foxglove::schemas::ArrowPrimitive>) {}
};

/// @brief 全局 Foxglove 服务器（空实现）
class FoxgloveServer
{
public:
    EntityPublisher::Ptr create_entity_publisher(
        std::string_view, std::string_view, std::string_view)
    {
        return std::make_shared<EntityPublisher>();
    }
};

inline FoxgloveServer &global_foxglove_server()
{
    static FoxgloveServer server;
    return server;
}

/// @brief 数据主题发布器（空实现）：publish/publish_with_time 全部返回 true 不做事
template <typename... Ts>
class DataPublisher
{
public:
    using Ptr = std::shared_ptr<DataPublisher<Ts...>>;

    DataPublisher(FoxgloveServer &, std::string_view /*topic*/,
                  std::vector<std::string> /*fields*/)
    {
    }

    template <typename... Us>
    bool publish(Us &&...)
    {
        return true;
    }

    template <typename Time, typename... Us>
    bool publish_with_time(Time &&, Us &&...)
    {
        return true;
    }
};

}  // namespace foxglove_viz
