// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tdt_interface:msg/RuneResult.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tdt_interface/msg/rune_result.h"


#ifndef TDT_INTERFACE__MSG__DETAIL__RUNE_RESULT__STRUCT_H_
#define TDT_INTERFACE__MSG__DETAIL__RUNE_RESULT__STRUCT_H_

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

/// Struct defined in msg/RuneResult in the package tdt_interface.
typedef struct tdt_interface__msg__RuneResult
{
  std_msgs__msg__Header header;
  uint64_t request_id;
  bool accepted;
  bool activated;
} tdt_interface__msg__RuneResult;

// Struct for a sequence of tdt_interface__msg__RuneResult.
typedef struct tdt_interface__msg__RuneResult__Sequence
{
  tdt_interface__msg__RuneResult * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tdt_interface__msg__RuneResult__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TDT_INTERFACE__MSG__DETAIL__RUNE_RESULT__STRUCT_H_
