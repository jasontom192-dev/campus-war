// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tdt_interface:msg/AutoAimState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tdt_interface/msg/auto_aim_state.hpp"


#ifndef TDT_INTERFACE__MSG__DETAIL__AUTO_AIM_STATE__TRAITS_HPP_
#define TDT_INTERFACE__MSG__DETAIL__AUTO_AIM_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tdt_interface/msg/detail/auto_aim_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'position'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__traits.hpp"

namespace tdt_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const AutoAimState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: frame_sequence
  {
    out << "frame_sequence: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_sequence, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
    out << ", ";
  }

  // member: twist
  {
    out << "twist: ";
    to_flow_style_yaml(msg.twist, out);
    out << ", ";
  }

  // member: yaw_degrees
  {
    out << "yaw_degrees: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_degrees, out);
    out << ", ";
  }

  // member: pitch_degrees
  {
    out << "pitch_degrees: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_degrees, out);
    out << ", ";
  }

  // member: health
  {
    out << "health: ";
    rosidl_generator_traits::value_to_yaml(msg.health, out);
    out << ", ";
  }

  // member: max_health
  {
    out << "max_health: ";
    rosidl_generator_traits::value_to_yaml(msg.max_health, out);
    out << ", ";
  }

  // member: is_weakened
  {
    out << "is_weakened: ";
    rosidl_generator_traits::value_to_yaml(msg.is_weakened, out);
    out << ", ";
  }

  // member: projectile_speed
  {
    out << "projectile_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.projectile_speed, out);
    out << ", ";
  }

  // member: remaining_seconds
  {
    out << "remaining_seconds: ";
    rosidl_generator_traits::value_to_yaml(msg.remaining_seconds, out);
    out << ", ";
  }

  // member: enemy_healths
  {
    if (msg.enemy_healths.size() == 0) {
      out << "enemy_healths: []";
    } else {
      out << "enemy_healths: [";
      size_t pending_items = msg.enemy_healths.size();
      for (auto item : msg.enemy_healths) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: ammo_remaining
  {
    out << "ammo_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.ammo_remaining, out);
    out << ", ";
  }

  // member: ammo_capacity
  {
    out << "ammo_capacity: ";
    rosidl_generator_traits::value_to_yaml(msg.ammo_capacity, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AutoAimState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: frame_sequence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frame_sequence: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_sequence, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }

  // member: twist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "twist:\n";
    to_block_style_yaml(msg.twist, out, indentation + 2);
  }

  // member: yaw_degrees
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_degrees: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_degrees, out);
    out << "\n";
  }

  // member: pitch_degrees
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_degrees: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_degrees, out);
    out << "\n";
  }

  // member: health
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "health: ";
    rosidl_generator_traits::value_to_yaml(msg.health, out);
    out << "\n";
  }

  // member: max_health
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_health: ";
    rosidl_generator_traits::value_to_yaml(msg.max_health, out);
    out << "\n";
  }

  // member: is_weakened
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_weakened: ";
    rosidl_generator_traits::value_to_yaml(msg.is_weakened, out);
    out << "\n";
  }

  // member: projectile_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "projectile_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.projectile_speed, out);
    out << "\n";
  }

  // member: remaining_seconds
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "remaining_seconds: ";
    rosidl_generator_traits::value_to_yaml(msg.remaining_seconds, out);
    out << "\n";
  }

  // member: enemy_healths
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.enemy_healths.size() == 0) {
      out << "enemy_healths: []\n";
    } else {
      out << "enemy_healths:\n";
      for (auto item : msg.enemy_healths) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: ammo_remaining
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ammo_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.ammo_remaining, out);
    out << "\n";
  }

  // member: ammo_capacity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ammo_capacity: ";
    rosidl_generator_traits::value_to_yaml(msg.ammo_capacity, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AutoAimState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace tdt_interface

namespace rosidl_generator_traits
{

[[deprecated("use tdt_interface::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tdt_interface::msg::AutoAimState & msg,
  std::ostream & out, size_t indentation = 0)
{
  tdt_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tdt_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const tdt_interface::msg::AutoAimState & msg)
{
  return tdt_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tdt_interface::msg::AutoAimState>()
{
  return "tdt_interface::msg::AutoAimState";
}

template<>
inline const char * name<tdt_interface::msg::AutoAimState>()
{
  return "tdt_interface/msg/AutoAimState";
}

template<>
struct has_fixed_size<tdt_interface::msg::AutoAimState>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<geometry_msgs::msg::Twist>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<tdt_interface::msg::AutoAimState>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<geometry_msgs::msg::Twist>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<tdt_interface::msg::AutoAimState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TDT_INTERFACE__MSG__DETAIL__AUTO_AIM_STATE__TRAITS_HPP_
