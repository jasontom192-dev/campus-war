// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tdt_interface:msg/AutoAimFrame.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tdt_interface/msg/auto_aim_frame.h"


#ifndef TDT_INTERFACE__MSG__DETAIL__AUTO_AIM_FRAME__STRUCT_H_
#define TDT_INTERFACE__MSG__DETAIL__AUTO_AIM_FRAME__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'state'
#include "tdt_interface/msg/detail/auto_aim_state__struct.h"
// Member 'camera_info'
#include "sensor_msgs/msg/detail/camera_info__struct.h"
// Member 'image'
#include "sensor_msgs/msg/detail/image__struct.h"
// Member 'compressed_image'
#include "sensor_msgs/msg/detail/compressed_image__struct.h"

/// Struct defined in msg/AutoAimFrame in the package tdt_interface.
typedef struct tdt_interface__msg__AutoAimFrame
{
  std_msgs__msg__Header header;
  tdt_interface__msg__AutoAimState state;
  sensor_msgs__msg__CameraInfo camera_info;
  sensor_msgs__msg__Image image;
  sensor_msgs__msg__CompressedImage compressed_image;
  float capture_to_publish_ms;
} tdt_interface__msg__AutoAimFrame;

// Struct for a sequence of tdt_interface__msg__AutoAimFrame.
typedef struct tdt_interface__msg__AutoAimFrame__Sequence
{
  tdt_interface__msg__AutoAimFrame * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tdt_interface__msg__AutoAimFrame__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TDT_INTERFACE__MSG__DETAIL__AUTO_AIM_FRAME__STRUCT_H_
