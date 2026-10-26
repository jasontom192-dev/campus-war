// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tdt_interface:msg/AutoAimFrame.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tdt_interface/msg/auto_aim_frame.hpp"


#ifndef TDT_INTERFACE__MSG__DETAIL__AUTO_AIM_FRAME__BUILDER_HPP_
#define TDT_INTERFACE__MSG__DETAIL__AUTO_AIM_FRAME__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tdt_interface/msg/detail/auto_aim_frame__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tdt_interface
{

namespace msg
{

namespace builder
{

class Init_AutoAimFrame_capture_to_publish_ms
{
public:
  explicit Init_AutoAimFrame_capture_to_publish_ms(::tdt_interface::msg::AutoAimFrame & msg)
  : msg_(msg)
  {}
  ::tdt_interface::msg::AutoAimFrame capture_to_publish_ms(::tdt_interface::msg::AutoAimFrame::_capture_to_publish_ms_type arg)
  {
    msg_.capture_to_publish_ms = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tdt_interface::msg::AutoAimFrame msg_;
};

class Init_AutoAimFrame_compressed_image
{
public:
  explicit Init_AutoAimFrame_compressed_image(::tdt_interface::msg::AutoAimFrame & msg)
  : msg_(msg)
  {}
  Init_AutoAimFrame_capture_to_publish_ms compressed_image(::tdt_interface::msg::AutoAimFrame::_compressed_image_type arg)
  {
    msg_.compressed_image = std::move(arg);
    return Init_AutoAimFrame_capture_to_publish_ms(msg_);
  }

private:
  ::tdt_interface::msg::AutoAimFrame msg_;
};

class Init_AutoAimFrame_image
{
public:
  explicit Init_AutoAimFrame_image(::tdt_interface::msg::AutoAimFrame & msg)
  : msg_(msg)
  {}
  Init_AutoAimFrame_compressed_image image(::tdt_interface::msg::AutoAimFrame::_image_type arg)
  {
    msg_.image = std::move(arg);
    return Init_AutoAimFrame_compressed_image(msg_);
  }

private:
  ::tdt_interface::msg::AutoAimFrame msg_;
};

class Init_AutoAimFrame_camera_info
{
public:
  explicit Init_AutoAimFrame_camera_info(::tdt_interface::msg::AutoAimFrame & msg)
  : msg_(msg)
  {}
  Init_AutoAimFrame_image camera_info(::tdt_interface::msg::AutoAimFrame::_camera_info_type arg)
  {
    msg_.camera_info = std::move(arg);
    return Init_AutoAimFrame_image(msg_);
  }

private:
  ::tdt_interface::msg::AutoAimFrame msg_;
};

class Init_AutoAimFrame_state
{
public:
  explicit Init_AutoAimFrame_state(::tdt_interface::msg::AutoAimFrame & msg)
  : msg_(msg)
  {}
  Init_AutoAimFrame_camera_info state(::tdt_interface::msg::AutoAimFrame::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_AutoAimFrame_camera_info(msg_);
  }

private:
  ::tdt_interface::msg::AutoAimFrame msg_;
};

class Init_AutoAimFrame_header
{
public:
  Init_AutoAimFrame_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AutoAimFrame_state header(::tdt_interface::msg::AutoAimFrame::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_AutoAimFrame_state(msg_);
  }

private:
  ::tdt_interface::msg::AutoAimFrame msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tdt_interface::msg::AutoAimFrame>()
{
  return tdt_interface::msg::builder::Init_AutoAimFrame_header();
}

}  // namespace tdt_interface

#endif  // TDT_INTERFACE__MSG__DETAIL__AUTO_AIM_FRAME__BUILDER_HPP_
