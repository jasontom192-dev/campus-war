// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from tdt_interface:msg/RuneResult.idl
// generated code does not contain a copyright notice
#ifndef TDT_INTERFACE__MSG__DETAIL__RUNE_RESULT__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define TDT_INTERFACE__MSG__DETAIL__RUNE_RESULT__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "tdt_interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "tdt_interface/msg/detail/rune_result__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tdt_interface
bool cdr_serialize_tdt_interface__msg__RuneResult(
  const tdt_interface__msg__RuneResult * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tdt_interface
bool cdr_deserialize_tdt_interface__msg__RuneResult(
  eprosima::fastcdr::Cdr &,
  tdt_interface__msg__RuneResult * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tdt_interface
size_t get_serialized_size_tdt_interface__msg__RuneResult(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tdt_interface
size_t max_serialized_size_tdt_interface__msg__RuneResult(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tdt_interface
bool cdr_serialize_key_tdt_interface__msg__RuneResult(
  const tdt_interface__msg__RuneResult * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tdt_interface
size_t get_serialized_size_key_tdt_interface__msg__RuneResult(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tdt_interface
size_t max_serialized_size_key_tdt_interface__msg__RuneResult(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tdt_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, tdt_interface, msg, RuneResult)();

#ifdef __cplusplus
}
#endif

#endif  // TDT_INTERFACE__MSG__DETAIL__RUNE_RESULT__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
