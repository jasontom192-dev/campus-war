// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tdt_interface:msg/SendData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tdt_interface/msg/send_data.hpp"


#ifndef TDT_INTERFACE__MSG__DETAIL__SEND_DATA__STRUCT_HPP_
#define TDT_INTERFACE__MSG__DETAIL__SEND_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__tdt_interface__msg__SendData __attribute__((deprecated))
#else
# define DEPRECATED__tdt_interface__msg__SendData __declspec(deprecated)
#endif

namespace tdt_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SendData_
{
  using Type = SendData_<ContainerAllocator>;

  explicit SendData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yaw = 0.0f;
      this->pitch = 0.0f;
      this->if_shoot = false;
    }
  }

  explicit SendData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yaw = 0.0f;
      this->pitch = 0.0f;
      this->if_shoot = false;
    }
  }

  // field types and members
  using _yaw_type =
    float;
  _yaw_type yaw;
  using _pitch_type =
    float;
  _pitch_type pitch;
  using _if_shoot_type =
    bool;
  _if_shoot_type if_shoot;

  // setters for named parameter idiom
  Type & set__yaw(
    const float & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__pitch(
    const float & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__if_shoot(
    const bool & _arg)
  {
    this->if_shoot = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tdt_interface::msg::SendData_<ContainerAllocator> *;
  using ConstRawPtr =
    const tdt_interface::msg::SendData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tdt_interface::msg::SendData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tdt_interface::msg::SendData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tdt_interface::msg::SendData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tdt_interface::msg::SendData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tdt_interface::msg::SendData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tdt_interface::msg::SendData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tdt_interface::msg::SendData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tdt_interface::msg::SendData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tdt_interface__msg__SendData
    std::shared_ptr<tdt_interface::msg::SendData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tdt_interface__msg__SendData
    std::shared_ptr<tdt_interface::msg::SendData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SendData_ & other) const
  {
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->if_shoot != other.if_shoot) {
      return false;
    }
    return true;
  }
  bool operator!=(const SendData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SendData_

// alias to use template instance with default allocator
using SendData =
  tdt_interface::msg::SendData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tdt_interface

#endif  // TDT_INTERFACE__MSG__DETAIL__SEND_DATA__STRUCT_HPP_
