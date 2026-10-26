// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tdt_interface:msg/AutoAimState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tdt_interface/msg/auto_aim_state.hpp"


#ifndef TDT_INTERFACE__MSG__DETAIL__AUTO_AIM_STATE__STRUCT_HPP_
#define TDT_INTERFACE__MSG__DETAIL__AUTO_AIM_STATE__STRUCT_HPP_

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
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__tdt_interface__msg__AutoAimState __attribute__((deprecated))
#else
# define DEPRECATED__tdt_interface__msg__AutoAimState __declspec(deprecated)
#endif

namespace tdt_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AutoAimState_
{
  using Type = AutoAimState_<ContainerAllocator>;

  explicit AutoAimState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    position(_init),
    twist(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_sequence = 0ull;
      this->yaw_degrees = 0.0f;
      this->pitch_degrees = 0.0f;
      this->health = 0l;
      this->max_health = 0l;
      this->is_weakened = false;
      this->projectile_speed = 0.0f;
      this->remaining_seconds = 0.0f;
      std::fill<typename std::array<int32_t, 7>::iterator, int32_t>(this->enemy_healths.begin(), this->enemy_healths.end(), 0l);
      this->ammo_remaining = 0l;
      this->ammo_capacity = 0l;
    }
  }

  explicit AutoAimState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    position(_alloc, _init),
    twist(_alloc, _init),
    enemy_healths(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_sequence = 0ull;
      this->yaw_degrees = 0.0f;
      this->pitch_degrees = 0.0f;
      this->health = 0l;
      this->max_health = 0l;
      this->is_weakened = false;
      this->projectile_speed = 0.0f;
      this->remaining_seconds = 0.0f;
      std::fill<typename std::array<int32_t, 7>::iterator, int32_t>(this->enemy_healths.begin(), this->enemy_healths.end(), 0l);
      this->ammo_remaining = 0l;
      this->ammo_capacity = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _frame_sequence_type =
    uint64_t;
  _frame_sequence_type frame_sequence;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _twist_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _twist_type twist;
  using _yaw_degrees_type =
    float;
  _yaw_degrees_type yaw_degrees;
  using _pitch_degrees_type =
    float;
  _pitch_degrees_type pitch_degrees;
  using _health_type =
    int32_t;
  _health_type health;
  using _max_health_type =
    int32_t;
  _max_health_type max_health;
  using _is_weakened_type =
    bool;
  _is_weakened_type is_weakened;
  using _projectile_speed_type =
    float;
  _projectile_speed_type projectile_speed;
  using _remaining_seconds_type =
    float;
  _remaining_seconds_type remaining_seconds;
  using _enemy_healths_type =
    std::array<int32_t, 7>;
  _enemy_healths_type enemy_healths;
  using _ammo_remaining_type =
    int32_t;
  _ammo_remaining_type ammo_remaining;
  using _ammo_capacity_type =
    int32_t;
  _ammo_capacity_type ammo_capacity;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__frame_sequence(
    const uint64_t & _arg)
  {
    this->frame_sequence = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__twist(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->twist = _arg;
    return *this;
  }
  Type & set__yaw_degrees(
    const float & _arg)
  {
    this->yaw_degrees = _arg;
    return *this;
  }
  Type & set__pitch_degrees(
    const float & _arg)
  {
    this->pitch_degrees = _arg;
    return *this;
  }
  Type & set__health(
    const int32_t & _arg)
  {
    this->health = _arg;
    return *this;
  }
  Type & set__max_health(
    const int32_t & _arg)
  {
    this->max_health = _arg;
    return *this;
  }
  Type & set__is_weakened(
    const bool & _arg)
  {
    this->is_weakened = _arg;
    return *this;
  }
  Type & set__projectile_speed(
    const float & _arg)
  {
    this->projectile_speed = _arg;
    return *this;
  }
  Type & set__remaining_seconds(
    const float & _arg)
  {
    this->remaining_seconds = _arg;
    return *this;
  }
  Type & set__enemy_healths(
    const std::array<int32_t, 7> & _arg)
  {
    this->enemy_healths = _arg;
    return *this;
  }
  Type & set__ammo_remaining(
    const int32_t & _arg)
  {
    this->ammo_remaining = _arg;
    return *this;
  }
  Type & set__ammo_capacity(
    const int32_t & _arg)
  {
    this->ammo_capacity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tdt_interface::msg::AutoAimState_<ContainerAllocator> *;
  using ConstRawPtr =
    const tdt_interface::msg::AutoAimState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tdt_interface::msg::AutoAimState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tdt_interface::msg::AutoAimState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tdt_interface::msg::AutoAimState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tdt_interface::msg::AutoAimState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tdt_interface::msg::AutoAimState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tdt_interface::msg::AutoAimState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tdt_interface::msg::AutoAimState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tdt_interface::msg::AutoAimState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tdt_interface__msg__AutoAimState
    std::shared_ptr<tdt_interface::msg::AutoAimState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tdt_interface__msg__AutoAimState
    std::shared_ptr<tdt_interface::msg::AutoAimState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AutoAimState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->frame_sequence != other.frame_sequence) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->twist != other.twist) {
      return false;
    }
    if (this->yaw_degrees != other.yaw_degrees) {
      return false;
    }
    if (this->pitch_degrees != other.pitch_degrees) {
      return false;
    }
    if (this->health != other.health) {
      return false;
    }
    if (this->max_health != other.max_health) {
      return false;
    }
    if (this->is_weakened != other.is_weakened) {
      return false;
    }
    if (this->projectile_speed != other.projectile_speed) {
      return false;
    }
    if (this->remaining_seconds != other.remaining_seconds) {
      return false;
    }
    if (this->enemy_healths != other.enemy_healths) {
      return false;
    }
    if (this->ammo_remaining != other.ammo_remaining) {
      return false;
    }
    if (this->ammo_capacity != other.ammo_capacity) {
      return false;
    }
    return true;
  }
  bool operator!=(const AutoAimState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AutoAimState_

// alias to use template instance with default allocator
using AutoAimState =
  tdt_interface::msg::AutoAimState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tdt_interface

#endif  // TDT_INTERFACE__MSG__DETAIL__AUTO_AIM_STATE__STRUCT_HPP_
