// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tdt_interface:msg/RuneResult.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tdt_interface/msg/rune_result.hpp"


#ifndef TDT_INTERFACE__MSG__DETAIL__RUNE_RESULT__BUILDER_HPP_
#define TDT_INTERFACE__MSG__DETAIL__RUNE_RESULT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tdt_interface/msg/detail/rune_result__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tdt_interface
{

namespace msg
{

namespace builder
{

class Init_RuneResult_activated
{
public:
  explicit Init_RuneResult_activated(::tdt_interface::msg::RuneResult & msg)
  : msg_(msg)
  {}
  ::tdt_interface::msg::RuneResult activated(::tdt_interface::msg::RuneResult::_activated_type arg)
  {
    msg_.activated = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tdt_interface::msg::RuneResult msg_;
};

class Init_RuneResult_accepted
{
public:
  explicit Init_RuneResult_accepted(::tdt_interface::msg::RuneResult & msg)
  : msg_(msg)
  {}
  Init_RuneResult_activated accepted(::tdt_interface::msg::RuneResult::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_RuneResult_activated(msg_);
  }

private:
  ::tdt_interface::msg::RuneResult msg_;
};

class Init_RuneResult_request_id
{
public:
  explicit Init_RuneResult_request_id(::tdt_interface::msg::RuneResult & msg)
  : msg_(msg)
  {}
  Init_RuneResult_accepted request_id(::tdt_interface::msg::RuneResult::_request_id_type arg)
  {
    msg_.request_id = std::move(arg);
    return Init_RuneResult_accepted(msg_);
  }

private:
  ::tdt_interface::msg::RuneResult msg_;
};

class Init_RuneResult_header
{
public:
  Init_RuneResult_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RuneResult_request_id header(::tdt_interface::msg::RuneResult::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RuneResult_request_id(msg_);
  }

private:
  ::tdt_interface::msg::RuneResult msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tdt_interface::msg::RuneResult>()
{
  return tdt_interface::msg::builder::Init_RuneResult_header();
}

}  // namespace tdt_interface

#endif  // TDT_INTERFACE__MSG__DETAIL__RUNE_RESULT__BUILDER_HPP_
