// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from tdt_interface:msg/AutoAimState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "tdt_interface/msg/detail/auto_aim_state__rosidl_typesupport_introspection_c.h"
#include "tdt_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "tdt_interface/msg/detail/auto_aim_state__functions.h"
#include "tdt_interface/msg/detail/auto_aim_state__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `position`
#include "geometry_msgs/msg/point.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"
// Member `twist`
#include "geometry_msgs/msg/twist.h"
// Member `twist`
#include "geometry_msgs/msg/detail/twist__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__AutoAimState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  tdt_interface__msg__AutoAimState__init(message_memory);
}

void tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__AutoAimState_fini_function(void * message_memory)
{
  tdt_interface__msg__AutoAimState__fini(message_memory);
}

size_t tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__size_function__AutoAimState__enemy_healths(
  const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__get_const_function__AutoAimState__enemy_healths(
  const void * untyped_member, size_t index)
{
  const int32_t * member =
    (const int32_t *)(untyped_member);
  return &member[index];
}

void * tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__get_function__AutoAimState__enemy_healths(
  void * untyped_member, size_t index)
{
  int32_t * member =
    (int32_t *)(untyped_member);
  return &member[index];
}

void tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__fetch_function__AutoAimState__enemy_healths(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__get_const_function__AutoAimState__enemy_healths(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__assign_function__AutoAimState__enemy_healths(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__get_function__AutoAimState__enemy_healths(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__AutoAimState_message_member_array[14] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface__msg__AutoAimState, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "frame_sequence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface__msg__AutoAimState, frame_sequence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface__msg__AutoAimState, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "twist",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface__msg__AutoAimState, twist),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yaw_degrees",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface__msg__AutoAimState, yaw_degrees),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pitch_degrees",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface__msg__AutoAimState, pitch_degrees),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "health",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface__msg__AutoAimState, health),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_health",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface__msg__AutoAimState, max_health),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_weakened",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface__msg__AutoAimState, is_weakened),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "projectile_speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface__msg__AutoAimState, projectile_speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "remaining_seconds",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface__msg__AutoAimState, remaining_seconds),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "enemy_healths",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(tdt_interface__msg__AutoAimState, enemy_healths),  // bytes offset in struct
    NULL,  // default value
    tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__size_function__AutoAimState__enemy_healths,  // size() function pointer
    tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__get_const_function__AutoAimState__enemy_healths,  // get_const(index) function pointer
    tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__get_function__AutoAimState__enemy_healths,  // get(index) function pointer
    tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__fetch_function__AutoAimState__enemy_healths,  // fetch(index, &value) function pointer
    tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__assign_function__AutoAimState__enemy_healths,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ammo_remaining",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface__msg__AutoAimState, ammo_remaining),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ammo_capacity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface__msg__AutoAimState, ammo_capacity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__AutoAimState_message_members = {
  "tdt_interface__msg",  // message namespace
  "AutoAimState",  // message name
  14,  // number of fields
  sizeof(tdt_interface__msg__AutoAimState),
  false,  // has_any_key_member_
  tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__AutoAimState_message_member_array,  // message members
  tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__AutoAimState_init_function,  // function to initialize message memory (memory has to be allocated)
  tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__AutoAimState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__AutoAimState_message_type_support_handle = {
  0,
  &tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__AutoAimState_message_members,
  get_message_typesupport_handle_function,
  &tdt_interface__msg__AutoAimState__get_type_hash,
  &tdt_interface__msg__AutoAimState__get_type_description,
  &tdt_interface__msg__AutoAimState__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_tdt_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tdt_interface, msg, AutoAimState)() {
  tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__AutoAimState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__AutoAimState_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__AutoAimState_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Twist)();
  if (!tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__AutoAimState_message_type_support_handle.typesupport_identifier) {
    tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__AutoAimState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &tdt_interface__msg__AutoAimState__rosidl_typesupport_introspection_c__AutoAimState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
