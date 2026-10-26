// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from tdt_interface:msg/AutoAimFrame.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "tdt_interface/msg/detail/auto_aim_frame__rosidl_typesupport_introspection_c.h"
#include "tdt_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "tdt_interface/msg/detail/auto_aim_frame__functions.h"
#include "tdt_interface/msg/detail/auto_aim_frame__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `state`
#include "tdt_interface/msg/auto_aim_state.h"
// Member `state`
#include "tdt_interface/msg/detail/auto_aim_state__rosidl_typesupport_introspection_c.h"
// Member `camera_info`
#include "sensor_msgs/msg/camera_info.h"
// Member `camera_info`
#include "sensor_msgs/msg/detail/camera_info__rosidl_typesupport_introspection_c.h"
// Member `image`
#include "sensor_msgs/msg/image.h"
// Member `image`
#include "sensor_msgs/msg/detail/image__rosidl_typesupport_introspection_c.h"
// Member `compressed_image`
#include "sensor_msgs/msg/compressed_image.h"
// Member `compressed_image`
#include "sensor_msgs/msg/detail/compressed_image__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void tdt_interface__msg__AutoAimFrame__rosidl_typesupport_introspection_c__AutoAimFrame_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  tdt_interface__msg__AutoAimFrame__init(message_memory);
}

void tdt_interface__msg__AutoAimFrame__rosidl_typesupport_introspection_c__AutoAimFrame_fini_function(void * message_memory)
{
  tdt_interface__msg__AutoAimFrame__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember tdt_interface__msg__AutoAimFrame__rosidl_typesupport_introspection_c__AutoAimFrame_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface__msg__AutoAimFrame, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface__msg__AutoAimFrame, state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "camera_info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface__msg__AutoAimFrame, camera_info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "image",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface__msg__AutoAimFrame, image),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "compressed_image",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface__msg__AutoAimFrame, compressed_image),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "capture_to_publish_ms",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface__msg__AutoAimFrame, capture_to_publish_ms),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers tdt_interface__msg__AutoAimFrame__rosidl_typesupport_introspection_c__AutoAimFrame_message_members = {
  "tdt_interface__msg",  // message namespace
  "AutoAimFrame",  // message name
  6,  // number of fields
  sizeof(tdt_interface__msg__AutoAimFrame),
  false,  // has_any_key_member_
  tdt_interface__msg__AutoAimFrame__rosidl_typesupport_introspection_c__AutoAimFrame_message_member_array,  // message members
  tdt_interface__msg__AutoAimFrame__rosidl_typesupport_introspection_c__AutoAimFrame_init_function,  // function to initialize message memory (memory has to be allocated)
  tdt_interface__msg__AutoAimFrame__rosidl_typesupport_introspection_c__AutoAimFrame_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t tdt_interface__msg__AutoAimFrame__rosidl_typesupport_introspection_c__AutoAimFrame_message_type_support_handle = {
  0,
  &tdt_interface__msg__AutoAimFrame__rosidl_typesupport_introspection_c__AutoAimFrame_message_members,
  get_message_typesupport_handle_function,
  &tdt_interface__msg__AutoAimFrame__get_type_hash,
  &tdt_interface__msg__AutoAimFrame__get_type_description,
  &tdt_interface__msg__AutoAimFrame__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_tdt_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tdt_interface, msg, AutoAimFrame)() {
  tdt_interface__msg__AutoAimFrame__rosidl_typesupport_introspection_c__AutoAimFrame_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  tdt_interface__msg__AutoAimFrame__rosidl_typesupport_introspection_c__AutoAimFrame_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tdt_interface, msg, AutoAimState)();
  tdt_interface__msg__AutoAimFrame__rosidl_typesupport_introspection_c__AutoAimFrame_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, CameraInfo)();
  tdt_interface__msg__AutoAimFrame__rosidl_typesupport_introspection_c__AutoAimFrame_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Image)();
  tdt_interface__msg__AutoAimFrame__rosidl_typesupport_introspection_c__AutoAimFrame_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, CompressedImage)();
  if (!tdt_interface__msg__AutoAimFrame__rosidl_typesupport_introspection_c__AutoAimFrame_message_type_support_handle.typesupport_identifier) {
    tdt_interface__msg__AutoAimFrame__rosidl_typesupport_introspection_c__AutoAimFrame_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &tdt_interface__msg__AutoAimFrame__rosidl_typesupport_introspection_c__AutoAimFrame_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
