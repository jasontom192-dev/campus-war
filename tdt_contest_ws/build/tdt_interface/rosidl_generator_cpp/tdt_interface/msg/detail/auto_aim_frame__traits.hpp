// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tdt_interface:msg/AutoAimFrame.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tdt_interface/msg/auto_aim_frame.hpp"


#ifndef TDT_INTERFACE__MSG__DETAIL__AUTO_AIM_FRAME__TRAITS_HPP_
#define TDT_INTERFACE__MSG__DETAIL__AUTO_AIM_FRAME__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tdt_interface/msg/detail/auto_aim_frame__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'state'
#include "tdt_interface/msg/detail/auto_aim_state__traits.hpp"
// Member 'camera_info'
#include "sensor_msgs/msg/detail/camera_info__traits.hpp"
// Member 'image'
#include "sensor_msgs/msg/detail/image__traits.hpp"
// Member 'compressed_image'
#include "sensor_msgs/msg/detail/compressed_image__traits.hpp"

namespace tdt_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const AutoAimFrame & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: state
  {
    out << "state: ";
    to_flow_style_yaml(msg.state, out);
    out << ", ";
  }

  // member: camera_info
  {
    out << "camera_info: ";
    to_flow_style_yaml(msg.camera_info, out);
    out << ", ";
  }

  // member: image
  {
    out << "image: ";
    to_flow_style_yaml(msg.image, out);
    out << ", ";
  }

  // member: compressed_image
  {
    out << "compressed_image: ";
    to_flow_style_yaml(msg.compressed_image, out);
    out << ", ";
  }

  // member: capture_to_publish_ms
  {
    out << "capture_to_publish_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.capture_to_publish_ms, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AutoAimFrame & msg,
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

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state:\n";
    to_block_style_yaml(msg.state, out, indentation + 2);
  }

  // member: camera_info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "camera_info:\n";
    to_block_style_yaml(msg.camera_info, out, indentation + 2);
  }

  // member: image
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "image:\n";
    to_block_style_yaml(msg.image, out, indentation + 2);
  }

  // member: compressed_image
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "compressed_image:\n";
    to_block_style_yaml(msg.compressed_image, out, indentation + 2);
  }

  // member: capture_to_publish_ms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "capture_to_publish_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.capture_to_publish_ms, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AutoAimFrame & msg, bool use_flow_style = false)
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
  const tdt_interface::msg::AutoAimFrame & msg,
  std::ostream & out, size_t indentation = 0)
{
  tdt_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tdt_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const tdt_interface::msg::AutoAimFrame & msg)
{
  return tdt_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tdt_interface::msg::AutoAimFrame>()
{
  return "tdt_interface::msg::AutoAimFrame";
}

template<>
inline const char * name<tdt_interface::msg::AutoAimFrame>()
{
  return "tdt_interface/msg/AutoAimFrame";
}

template<>
struct has_fixed_size<tdt_interface::msg::AutoAimFrame>
  : std::integral_constant<bool, has_fixed_size<sensor_msgs::msg::CameraInfo>::value && has_fixed_size<sensor_msgs::msg::CompressedImage>::value && has_fixed_size<sensor_msgs::msg::Image>::value && has_fixed_size<std_msgs::msg::Header>::value && has_fixed_size<tdt_interface::msg::AutoAimState>::value> {};

template<>
struct has_bounded_size<tdt_interface::msg::AutoAimFrame>
  : std::integral_constant<bool, has_bounded_size<sensor_msgs::msg::CameraInfo>::value && has_bounded_size<sensor_msgs::msg::CompressedImage>::value && has_bounded_size<sensor_msgs::msg::Image>::value && has_bounded_size<std_msgs::msg::Header>::value && has_bounded_size<tdt_interface::msg::AutoAimState>::value> {};

template<>
struct is_message<tdt_interface::msg::AutoAimFrame>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TDT_INTERFACE__MSG__DETAIL__AUTO_AIM_FRAME__TRAITS_HPP_
