// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tdt_interface:msg/AutoAimState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tdt_interface/msg/auto_aim_state.hpp"


#ifndef TDT_INTERFACE__MSG__DETAIL__AUTO_AIM_STATE__BUILDER_HPP_
#define TDT_INTERFACE__MSG__DETAIL__AUTO_AIM_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tdt_interface/msg/detail/auto_aim_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tdt_interface
{

namespace msg
{

namespace builder
{

class Init_AutoAimState_ammo_capacity
{
public:
  explicit Init_AutoAimState_ammo_capacity(::tdt_interface::msg::AutoAimState & msg)
  : msg_(msg)
  {}
  ::tdt_interface::msg::AutoAimState ammo_capacity(::tdt_interface::msg::AutoAimState::_ammo_capacity_type arg)
  {
    msg_.ammo_capacity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tdt_interface::msg::AutoAimState msg_;
};

class Init_AutoAimState_ammo_remaining
{
public:
  explicit Init_AutoAimState_ammo_remaining(::tdt_interface::msg::AutoAimState & msg)
  : msg_(msg)
  {}
  Init_AutoAimState_ammo_capacity ammo_remaining(::tdt_interface::msg::AutoAimState::_ammo_remaining_type arg)
  {
    msg_.ammo_remaining = std::move(arg);
    return Init_AutoAimState_ammo_capacity(msg_);
  }

private:
  ::tdt_interface::msg::AutoAimState msg_;
};

class Init_AutoAimState_enemy_healths
{
public:
  explicit Init_AutoAimState_enemy_healths(::tdt_interface::msg::AutoAimState & msg)
  : msg_(msg)
  {}
  Init_AutoAimState_ammo_remaining enemy_healths(::tdt_interface::msg::AutoAimState::_enemy_healths_type arg)
  {
    msg_.enemy_healths = std::move(arg);
    return Init_AutoAimState_ammo_remaining(msg_);
  }

private:
  ::tdt_interface::msg::AutoAimState msg_;
};

class Init_AutoAimState_remaining_seconds
{
public:
  explicit Init_AutoAimState_remaining_seconds(::tdt_interface::msg::AutoAimState & msg)
  : msg_(msg)
  {}
  Init_AutoAimState_enemy_healths remaining_seconds(::tdt_interface::msg::AutoAimState::_remaining_seconds_type arg)
  {
    msg_.remaining_seconds = std::move(arg);
    return Init_AutoAimState_enemy_healths(msg_);
  }

private:
  ::tdt_interface::msg::AutoAimState msg_;
};

class Init_AutoAimState_projectile_speed
{
public:
  explicit Init_AutoAimState_projectile_speed(::tdt_interface::msg::AutoAimState & msg)
  : msg_(msg)
  {}
  Init_AutoAimState_remaining_seconds projectile_speed(::tdt_interface::msg::AutoAimState::_projectile_speed_type arg)
  {
    msg_.projectile_speed = std::move(arg);
    return Init_AutoAimState_remaining_seconds(msg_);
  }

private:
  ::tdt_interface::msg::AutoAimState msg_;
};

class Init_AutoAimState_is_weakened
{
public:
  explicit Init_AutoAimState_is_weakened(::tdt_interface::msg::AutoAimState & msg)
  : msg_(msg)
  {}
  Init_AutoAimState_projectile_speed is_weakened(::tdt_interface::msg::AutoAimState::_is_weakened_type arg)
  {
    msg_.is_weakened = std::move(arg);
    return Init_AutoAimState_projectile_speed(msg_);
  }

private:
  ::tdt_interface::msg::AutoAimState msg_;
};

class Init_AutoAimState_max_health
{
public:
  explicit Init_AutoAimState_max_health(::tdt_interface::msg::AutoAimState & msg)
  : msg_(msg)
  {}
  Init_AutoAimState_is_weakened max_health(::tdt_interface::msg::AutoAimState::_max_health_type arg)
  {
    msg_.max_health = std::move(arg);
    return Init_AutoAimState_is_weakened(msg_);
  }

private:
  ::tdt_interface::msg::AutoAimState msg_;
};

class Init_AutoAimState_health
{
public:
  explicit Init_AutoAimState_health(::tdt_interface::msg::AutoAimState & msg)
  : msg_(msg)
  {}
  Init_AutoAimState_max_health health(::tdt_interface::msg::AutoAimState::_health_type arg)
  {
    msg_.health = std::move(arg);
    return Init_AutoAimState_max_health(msg_);
  }

private:
  ::tdt_interface::msg::AutoAimState msg_;
};

class Init_AutoAimState_pitch_degrees
{
public:
  explicit Init_AutoAimState_pitch_degrees(::tdt_interface::msg::AutoAimState & msg)
  : msg_(msg)
  {}
  Init_AutoAimState_health pitch_degrees(::tdt_interface::msg::AutoAimState::_pitch_degrees_type arg)
  {
    msg_.pitch_degrees = std::move(arg);
    return Init_AutoAimState_health(msg_);
  }

private:
  ::tdt_interface::msg::AutoAimState msg_;
};

class Init_AutoAimState_yaw_degrees
{
public:
  explicit Init_AutoAimState_yaw_degrees(::tdt_interface::msg::AutoAimState & msg)
  : msg_(msg)
  {}
  Init_AutoAimState_pitch_degrees yaw_degrees(::tdt_interface::msg::AutoAimState::_yaw_degrees_type arg)
  {
    msg_.yaw_degrees = std::move(arg);
    return Init_AutoAimState_pitch_degrees(msg_);
  }

private:
  ::tdt_interface::msg::AutoAimState msg_;
};

class Init_AutoAimState_twist
{
public:
  explicit Init_AutoAimState_twist(::tdt_interface::msg::AutoAimState & msg)
  : msg_(msg)
  {}
  Init_AutoAimState_yaw_degrees twist(::tdt_interface::msg::AutoAimState::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return Init_AutoAimState_yaw_degrees(msg_);
  }

private:
  ::tdt_interface::msg::AutoAimState msg_;
};

class Init_AutoAimState_position
{
public:
  explicit Init_AutoAimState_position(::tdt_interface::msg::AutoAimState & msg)
  : msg_(msg)
  {}
  Init_AutoAimState_twist position(::tdt_interface::msg::AutoAimState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_AutoAimState_twist(msg_);
  }

private:
  ::tdt_interface::msg::AutoAimState msg_;
};

class Init_AutoAimState_frame_sequence
{
public:
  explicit Init_AutoAimState_frame_sequence(::tdt_interface::msg::AutoAimState & msg)
  : msg_(msg)
  {}
  Init_AutoAimState_position frame_sequence(::tdt_interface::msg::AutoAimState::_frame_sequence_type arg)
  {
    msg_.frame_sequence = std::move(arg);
    return Init_AutoAimState_position(msg_);
  }

private:
  ::tdt_interface::msg::AutoAimState msg_;
};

class Init_AutoAimState_header
{
public:
  Init_AutoAimState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AutoAimState_frame_sequence header(::tdt_interface::msg::AutoAimState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_AutoAimState_frame_sequence(msg_);
  }

private:
  ::tdt_interface::msg::AutoAimState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tdt_interface::msg::AutoAimState>()
{
  return tdt_interface::msg::builder::Init_AutoAimState_header();
}

}  // namespace tdt_interface

#endif  // TDT_INTERFACE__MSG__DETAIL__AUTO_AIM_STATE__BUILDER_HPP_
