// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tdt_interface:msg/SendData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tdt_interface/msg/send_data.hpp"


#ifndef TDT_INTERFACE__MSG__DETAIL__SEND_DATA__TRAITS_HPP_
#define TDT_INTERFACE__MSG__DETAIL__SEND_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tdt_interface/msg/detail/send_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace tdt_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const SendData & msg,
  std::ostream & out)
{
  out << "{";
  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: if_shoot
  {
    out << "if_shoot: ";
    rosidl_generator_traits::value_to_yaml(msg.if_shoot, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SendData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: if_shoot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "if_shoot: ";
    rosidl_generator_traits::value_to_yaml(msg.if_shoot, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SendData & msg, bool use_flow_style = false)
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
  const tdt_interface::msg::SendData & msg,
  std::ostream & out, size_t indentation = 0)
{
  tdt_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tdt_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const tdt_interface::msg::SendData & msg)
{
  return tdt_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tdt_interface::msg::SendData>()
{
  return "tdt_interface::msg::SendData";
}

template<>
inline const char * name<tdt_interface::msg::SendData>()
{
  return "tdt_interface/msg/SendData";
}

template<>
struct has_fixed_size<tdt_interface::msg::SendData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<tdt_interface::msg::SendData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<tdt_interface::msg::SendData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TDT_INTERFACE__MSG__DETAIL__SEND_DATA__TRAITS_HPP_
