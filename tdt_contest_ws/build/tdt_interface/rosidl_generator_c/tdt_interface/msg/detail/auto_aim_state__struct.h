// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tdt_interface:msg/AutoAimState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tdt_interface/msg/auto_aim_state.h"


#ifndef TDT_INTERFACE__MSG__DETAIL__AUTO_AIM_STATE__STRUCT_H_
#define TDT_INTERFACE__MSG__DETAIL__AUTO_AIM_STATE__STRUCT_H_

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
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.h"

/// Struct defined in msg/AutoAimState in the package tdt_interface.
typedef struct tdt_interface__msg__AutoAimState
{
  std_msgs__msg__Header header;
  uint64_t frame_sequence;
  geometry_msgs__msg__Point position;
  geometry_msgs__msg__Twist twist;
  float yaw_degrees;
  float pitch_degrees;
  int32_t health;
  int32_t max_health;
  bool is_weakened;
  float projectile_speed;
  float remaining_seconds;
  int32_t enemy_healths[7];
  int32_t ammo_remaining;
  int32_t ammo_capacity;
} tdt_interface__msg__AutoAimState;

// Struct for a sequence of tdt_interface__msg__AutoAimState.
typedef struct tdt_interface__msg__AutoAimState__Sequence
{
  tdt_interface__msg__AutoAimState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tdt_interface__msg__AutoAimState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TDT_INTERFACE__MSG__DETAIL__AUTO_AIM_STATE__STRUCT_H_
