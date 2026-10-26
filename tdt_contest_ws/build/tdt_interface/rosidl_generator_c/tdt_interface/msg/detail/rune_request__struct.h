// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tdt_interface:msg/RuneRequest.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tdt_interface/msg/rune_request.h"


#ifndef TDT_INTERFACE__MSG__DETAIL__RUNE_REQUEST__STRUCT_H_
#define TDT_INTERFACE__MSG__DETAIL__RUNE_REQUEST__STRUCT_H_

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

/// Struct defined in msg/RuneRequest in the package tdt_interface.
typedef struct tdt_interface__msg__RuneRequest
{
  std_msgs__msg__Header header;
  uint64_t request_id;
} tdt_interface__msg__RuneRequest;

// Struct for a sequence of tdt_interface__msg__RuneRequest.
typedef struct tdt_interface__msg__RuneRequest__Sequence
{
  tdt_interface__msg__RuneRequest * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tdt_interface__msg__RuneRequest__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TDT_INTERFACE__MSG__DETAIL__RUNE_REQUEST__STRUCT_H_
