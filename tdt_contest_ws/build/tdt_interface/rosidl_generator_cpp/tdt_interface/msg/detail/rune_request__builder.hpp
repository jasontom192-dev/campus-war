// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tdt_interface:msg/RuneRequest.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tdt_interface/msg/rune_request.hpp"


#ifndef TDT_INTERFACE__MSG__DETAIL__RUNE_REQUEST__BUILDER_HPP_
#define TDT_INTERFACE__MSG__DETAIL__RUNE_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tdt_interface/msg/detail/rune_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tdt_interface
{

namespace msg
{

namespace builder
{

class Init_RuneRequest_request_id
{
public:
  explicit Init_RuneRequest_request_id(::tdt_interface::msg::RuneRequest & msg)
  : msg_(msg)
  {}
  ::tdt_interface::msg::RuneRequest request_id(::tdt_interface::msg::RuneRequest::_request_id_type arg)
  {
    msg_.request_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tdt_interface::msg::RuneRequest msg_;
};

class Init_RuneRequest_header
{
public:
  Init_RuneRequest_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RuneRequest_request_id header(::tdt_interface::msg::RuneRequest::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RuneRequest_request_id(msg_);
  }

private:
  ::tdt_interface::msg::RuneRequest msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tdt_interface::msg::RuneRequest>()
{
  return tdt_interface::msg::builder::Init_RuneRequest_header();
}

}  // namespace tdt_interface

#endif  // TDT_INTERFACE__MSG__DETAIL__RUNE_REQUEST__BUILDER_HPP_
