// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tdt_interface:msg/AutoAimFrame.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tdt_interface/msg/auto_aim_frame.hpp"


#ifndef TDT_INTERFACE__MSG__DETAIL__AUTO_AIM_FRAME__STRUCT_HPP_
#define TDT_INTERFACE__MSG__DETAIL__AUTO_AIM_FRAME__STRUCT_HPP_

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
// Member 'state'
#include "tdt_interface/msg/detail/auto_aim_state__struct.hpp"
// Member 'camera_info'
#include "sensor_msgs/msg/detail/camera_info__struct.hpp"
// Member 'image'
#include "sensor_msgs/msg/detail/image__struct.hpp"
// Member 'compressed_image'
#include "sensor_msgs/msg/detail/compressed_image__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__tdt_interface__msg__AutoAimFrame __attribute__((deprecated))
#else
# define DEPRECATED__tdt_interface__msg__AutoAimFrame __declspec(deprecated)
#endif

namespace tdt_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AutoAimFrame_
{
  using Type = AutoAimFrame_<ContainerAllocator>;

  explicit AutoAimFrame_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    state(_init),
    camera_info(_init),
    image(_init),
    compressed_image(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->capture_to_publish_ms = 0.0f;
    }
  }

  explicit AutoAimFrame_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    state(_alloc, _init),
    camera_info(_alloc, _init),
    image(_alloc, _init),
    compressed_image(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->capture_to_publish_ms = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _state_type =
    tdt_interface::msg::AutoAimState_<ContainerAllocator>;
  _state_type state;
  using _camera_info_type =
    sensor_msgs::msg::CameraInfo_<ContainerAllocator>;
  _camera_info_type camera_info;
  using _image_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _image_type image;
  using _compressed_image_type =
    sensor_msgs::msg::CompressedImage_<ContainerAllocator>;
  _compressed_image_type compressed_image;
  using _capture_to_publish_ms_type =
    float;
  _capture_to_publish_ms_type capture_to_publish_ms;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__state(
    const tdt_interface::msg::AutoAimState_<ContainerAllocator> & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__camera_info(
    const sensor_msgs::msg::CameraInfo_<ContainerAllocator> & _arg)
  {
    this->camera_info = _arg;
    return *this;
  }
  Type & set__image(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->image = _arg;
    return *this;
  }
  Type & set__compressed_image(
    const sensor_msgs::msg::CompressedImage_<ContainerAllocator> & _arg)
  {
    this->compressed_image = _arg;
    return *this;
  }
  Type & set__capture_to_publish_ms(
    const float & _arg)
  {
    this->capture_to_publish_ms = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tdt_interface::msg::AutoAimFrame_<ContainerAllocator> *;
  using ConstRawPtr =
    const tdt_interface::msg::AutoAimFrame_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tdt_interface::msg::AutoAimFrame_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tdt_interface::msg::AutoAimFrame_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tdt_interface::msg::AutoAimFrame_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tdt_interface::msg::AutoAimFrame_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tdt_interface::msg::AutoAimFrame_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tdt_interface::msg::AutoAimFrame_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tdt_interface::msg::AutoAimFrame_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tdt_interface::msg::AutoAimFrame_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tdt_interface__msg__AutoAimFrame
    std::shared_ptr<tdt_interface::msg::AutoAimFrame_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tdt_interface__msg__AutoAimFrame
    std::shared_ptr<tdt_interface::msg::AutoAimFrame_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AutoAimFrame_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->camera_info != other.camera_info) {
      return false;
    }
    if (this->image != other.image) {
      return false;
    }
    if (this->compressed_image != other.compressed_image) {
      return false;
    }
    if (this->capture_to_publish_ms != other.capture_to_publish_ms) {
      return false;
    }
    return true;
  }
  bool operator!=(const AutoAimFrame_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AutoAimFrame_

// alias to use template instance with default allocator
using AutoAimFrame =
  tdt_interface::msg::AutoAimFrame_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tdt_interface

#endif  // TDT_INTERFACE__MSG__DETAIL__AUTO_AIM_FRAME__STRUCT_HPP_
