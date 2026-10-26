// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from tdt_interface:msg/SendData.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "tdt_interface/msg/detail/send_data__functions.h"
#include "tdt_interface/msg/detail/send_data__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace tdt_interface
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void SendData_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) tdt_interface::msg::SendData(_init);
}

void SendData_fini_function(void * message_memory)
{
  auto typed_message = static_cast<tdt_interface::msg::SendData *>(message_memory);
  typed_message->~SendData();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SendData_message_member_array[3] = {
  {
    "yaw",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface::msg::SendData, yaw),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pitch",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface::msg::SendData, pitch),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "if_shoot",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tdt_interface::msg::SendData, if_shoot),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SendData_message_members = {
  "tdt_interface::msg",  // message namespace
  "SendData",  // message name
  3,  // number of fields
  sizeof(tdt_interface::msg::SendData),
  false,  // has_any_key_member_
  SendData_message_member_array,  // message members
  SendData_init_function,  // function to initialize message memory (memory has to be allocated)
  SendData_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SendData_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SendData_message_members,
  get_message_typesupport_handle_function,
  &tdt_interface__msg__SendData__get_type_hash,
  &tdt_interface__msg__SendData__get_type_description,
  &tdt_interface__msg__SendData__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace tdt_interface


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<tdt_interface::msg::SendData>()
{
  return &::tdt_interface::msg::rosidl_typesupport_introspection_cpp::SendData_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, tdt_interface, msg, SendData)() {
  return &::tdt_interface::msg::rosidl_typesupport_introspection_cpp::SendData_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
