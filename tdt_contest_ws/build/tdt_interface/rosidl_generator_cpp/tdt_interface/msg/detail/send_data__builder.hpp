// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tdt_interface:msg/SendData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tdt_interface/msg/send_data.hpp"


#ifndef TDT_INTERFACE__MSG__DETAIL__SEND_DATA__BUILDER_HPP_
#define TDT_INTERFACE__MSG__DETAIL__SEND_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tdt_interface/msg/detail/send_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tdt_interface
{

namespace msg
{

namespace builder
{

class Init_SendData_if_shoot
{
public:
  explicit Init_SendData_if_shoot(::tdt_interface::msg::SendData & msg)
  : msg_(msg)
  {}
  ::tdt_interface::msg::SendData if_shoot(::tdt_interface::msg::SendData::_if_shoot_type arg)
  {
    msg_.if_shoot = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tdt_interface::msg::SendData msg_;
};

class Init_SendData_pitch
{
public:
  explicit Init_SendData_pitch(::tdt_interface::msg::SendData & msg)
  : msg_(msg)
  {}
  Init_SendData_if_shoot pitch(::tdt_interface::msg::SendData::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_SendData_if_shoot(msg_);
  }

private:
  ::tdt_interface::msg::SendData msg_;
};

class Init_SendData_yaw
{
public:
  Init_SendData_yaw()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SendData_pitch yaw(::tdt_interface::msg::SendData::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_SendData_pitch(msg_);
  }

private:
  ::tdt_interface::msg::SendData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tdt_interface::msg::SendData>()
{
  return tdt_interface::msg::builder::Init_SendData_yaw();
}

}  // namespace tdt_interface

#endif  // TDT_INTERFACE__MSG__DETAIL__SEND_DATA__BUILDER_HPP_
