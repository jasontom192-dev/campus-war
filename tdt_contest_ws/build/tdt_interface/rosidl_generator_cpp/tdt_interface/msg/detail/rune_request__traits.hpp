// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tdt_interface:msg/RuneRequest.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tdt_interface/msg/rune_request.hpp"


#ifndef TDT_INTERFACE__MSG__DETAIL__RUNE_REQUEST__TRAITS_HPP_
#define TDT_INTERFACE__MSG__DETAIL__RUNE_REQUEST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tdt_interface/msg/detail/rune_request__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace tdt_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const RuneRequest & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: request_id
  {
    out << "request_id: ";
    rosidl_generator_traits::value_to_yaml(msg.request_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RuneRequest & msg,
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

  // member: request_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "request_id: ";
    rosidl_generator_traits::value_to_yaml(msg.request_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RuneRequest & msg, bool use_flow_style = false)
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
  const tdt_interface::msg::RuneRequest & msg,
  std::ostream & out, size_t indentation = 0)
{
  tdt_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tdt_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const tdt_interface::msg::RuneRequest & msg)
{
  return tdt_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tdt_interface::msg::RuneRequest>()
{
  return "tdt_interface::msg::RuneRequest";
}

template<>
inline const char * name<tdt_interface::msg::RuneRequest>()
{
  return "tdt_interface/msg/RuneRequest";
}

template<>
struct has_fixed_size<tdt_interface::msg::RuneRequest>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<tdt_interface::msg::RuneRequest>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<tdt_interface::msg::RuneRequest>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TDT_INTERFACE__MSG__DETAIL__RUNE_REQUEST__TRAITS_HPP_
