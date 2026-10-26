// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tdt_interface:msg/RuneResult.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tdt_interface/msg/rune_result.hpp"


#ifndef TDT_INTERFACE__MSG__DETAIL__RUNE_RESULT__STRUCT_HPP_
#define TDT_INTERFACE__MSG__DETAIL__RUNE_RESULT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__tdt_interface__msg__RuneResult __attribute__((deprecated))
#else
# define DEPRECATED__tdt_interface__msg__RuneResult __declspec(deprecated)
#endif

namespace tdt_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RuneResult_
{
  using Type = RuneResult_<ContainerAllocator>;

  explicit RuneResult_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request_id = 0ull;
      this->accepted = false;
      this->activated = false;
    }
  }

  explicit RuneResult_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request_id = 0ull;
      this->accepted = false;
      this->activated = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _request_id_type =
    uint64_t;
  _request_id_type request_id;
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _activated_type =
    bool;
  _activated_type activated;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__request_id(
    const uint64_t & _arg)
  {
    this->request_id = _arg;
    return *this;
  }
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__activated(
    const bool & _arg)
  {
    this->activated = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tdt_interface::msg::RuneResult_<ContainerAllocator> *;
  using ConstRawPtr =
    const tdt_interface::msg::RuneResult_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tdt_interface::msg::RuneResult_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tdt_interface::msg::RuneResult_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tdt_interface::msg::RuneResult_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tdt_interface::msg::RuneResult_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tdt_interface::msg::RuneResult_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tdt_interface::msg::RuneResult_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tdt_interface::msg::RuneResult_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tdt_interface::msg::RuneResult_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tdt_interface__msg__RuneResult
    std::shared_ptr<tdt_interface::msg::RuneResult_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tdt_interface__msg__RuneResult
    std::shared_ptr<tdt_interface::msg::RuneResult_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RuneResult_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->request_id != other.request_id) {
      return false;
    }
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->activated != other.activated) {
      return false;
    }
    return true;
  }
  bool operator!=(const RuneResult_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RuneResult_

// alias to use template instance with default allocator
using RuneResult =
  tdt_interface::msg::RuneResult_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tdt_interface

#endif  // TDT_INTERFACE__MSG__DETAIL__RUNE_RESULT__STRUCT_HPP_
