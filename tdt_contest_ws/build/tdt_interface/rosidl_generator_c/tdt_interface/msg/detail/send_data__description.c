// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from tdt_interface:msg/SendData.idl
// generated code does not contain a copyright notice

#include "tdt_interface/msg/detail/send_data__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_tdt_interface
const rosidl_type_hash_t *
tdt_interface__msg__SendData__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8f, 0x69, 0x67, 0x7a, 0x9a, 0x21, 0xdf, 0x99,
      0x6c, 0xf5, 0xe4, 0x71, 0x31, 0xf2, 0xec, 0x52,
      0xd8, 0x0e, 0x03, 0x11, 0x69, 0x71, 0x22, 0xfa,
      0x34, 0x41, 0x5f, 0x05, 0x6e, 0x4b, 0x03, 0x8b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char tdt_interface__msg__SendData__TYPE_NAME[] = "tdt_interface/msg/SendData";

// Define type names, field names, and default values
static char tdt_interface__msg__SendData__FIELD_NAME__yaw[] = "yaw";
static char tdt_interface__msg__SendData__FIELD_NAME__pitch[] = "pitch";
static char tdt_interface__msg__SendData__FIELD_NAME__if_shoot[] = "if_shoot";

static rosidl_runtime_c__type_description__Field tdt_interface__msg__SendData__FIELDS[] = {
  {
    {tdt_interface__msg__SendData__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tdt_interface__msg__SendData__FIELD_NAME__pitch, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tdt_interface__msg__SendData__FIELD_NAME__if_shoot, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
tdt_interface__msg__SendData__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {tdt_interface__msg__SendData__TYPE_NAME, 26, 26},
      {tdt_interface__msg__SendData__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 yaw       # World optical yaw in degrees: Unity +Z = 0, clockwise positive; chassis-decoupled.\n"
  "float32 pitch     # World optical pitch in degrees: downward positive, range [-60, 60].\n"
  "bool if_shoot     # Fire permission paired with these angles; actuator alignment gates actual shots.";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
tdt_interface__msg__SendData__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {tdt_interface__msg__SendData__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 292, 292},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
tdt_interface__msg__SendData__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *tdt_interface__msg__SendData__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
