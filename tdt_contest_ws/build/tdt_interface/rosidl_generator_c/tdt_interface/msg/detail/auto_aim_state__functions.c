// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tdt_interface:msg/AutoAimState.idl
// generated code does not contain a copyright notice
#include "tdt_interface/msg/detail/auto_aim_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `twist`
#include "geometry_msgs/msg/detail/twist__functions.h"

bool
tdt_interface__msg__AutoAimState__init(tdt_interface__msg__AutoAimState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    tdt_interface__msg__AutoAimState__fini(msg);
    return false;
  }
  // frame_sequence
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    tdt_interface__msg__AutoAimState__fini(msg);
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__init(&msg->twist)) {
    tdt_interface__msg__AutoAimState__fini(msg);
    return false;
  }
  // yaw_degrees
  // pitch_degrees
  // health
  // max_health
  // is_weakened
  // projectile_speed
  // remaining_seconds
  // enemy_healths
  // ammo_remaining
  // ammo_capacity
  return true;
}

void
tdt_interface__msg__AutoAimState__fini(tdt_interface__msg__AutoAimState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // frame_sequence
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // twist
  geometry_msgs__msg__Twist__fini(&msg->twist);
  // yaw_degrees
  // pitch_degrees
  // health
  // max_health
  // is_weakened
  // projectile_speed
  // remaining_seconds
  // enemy_healths
  // ammo_remaining
  // ammo_capacity
}

bool
tdt_interface__msg__AutoAimState__are_equal(const tdt_interface__msg__AutoAimState * lhs, const tdt_interface__msg__AutoAimState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // frame_sequence
  if (lhs->frame_sequence != rhs->frame_sequence) {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->twist), &(rhs->twist)))
  {
    return false;
  }
  // yaw_degrees
  if (lhs->yaw_degrees != rhs->yaw_degrees) {
    return false;
  }
  // pitch_degrees
  if (lhs->pitch_degrees != rhs->pitch_degrees) {
    return false;
  }
  // health
  if (lhs->health != rhs->health) {
    return false;
  }
  // max_health
  if (lhs->max_health != rhs->max_health) {
    return false;
  }
  // is_weakened
  if (lhs->is_weakened != rhs->is_weakened) {
    return false;
  }
  // projectile_speed
  if (lhs->projectile_speed != rhs->projectile_speed) {
    return false;
  }
  // remaining_seconds
  if (lhs->remaining_seconds != rhs->remaining_seconds) {
    return false;
  }
  // enemy_healths
  for (size_t i = 0; i < 7; ++i) {
    if (lhs->enemy_healths[i] != rhs->enemy_healths[i]) {
      return false;
    }
  }
  // ammo_remaining
  if (lhs->ammo_remaining != rhs->ammo_remaining) {
    return false;
  }
  // ammo_capacity
  if (lhs->ammo_capacity != rhs->ammo_capacity) {
    return false;
  }
  return true;
}

bool
tdt_interface__msg__AutoAimState__copy(
  const tdt_interface__msg__AutoAimState * input,
  tdt_interface__msg__AutoAimState * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // frame_sequence
  output->frame_sequence = input->frame_sequence;
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__copy(
      &(input->twist), &(output->twist)))
  {
    return false;
  }
  // yaw_degrees
  output->yaw_degrees = input->yaw_degrees;
  // pitch_degrees
  output->pitch_degrees = input->pitch_degrees;
  // health
  output->health = input->health;
  // max_health
  output->max_health = input->max_health;
  // is_weakened
  output->is_weakened = input->is_weakened;
  // projectile_speed
  output->projectile_speed = input->projectile_speed;
  // remaining_seconds
  output->remaining_seconds = input->remaining_seconds;
  // enemy_healths
  for (size_t i = 0; i < 7; ++i) {
    output->enemy_healths[i] = input->enemy_healths[i];
  }
  // ammo_remaining
  output->ammo_remaining = input->ammo_remaining;
  // ammo_capacity
  output->ammo_capacity = input->ammo_capacity;
  return true;
}

tdt_interface__msg__AutoAimState *
tdt_interface__msg__AutoAimState__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tdt_interface__msg__AutoAimState * msg = (tdt_interface__msg__AutoAimState *)allocator.allocate(sizeof(tdt_interface__msg__AutoAimState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tdt_interface__msg__AutoAimState));
  bool success = tdt_interface__msg__AutoAimState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tdt_interface__msg__AutoAimState__destroy(tdt_interface__msg__AutoAimState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tdt_interface__msg__AutoAimState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tdt_interface__msg__AutoAimState__Sequence__init(tdt_interface__msg__AutoAimState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tdt_interface__msg__AutoAimState * data = NULL;

  if (size) {
    if (size > SIZE_MAX / sizeof(tdt_interface__msg__AutoAimState)) {
      return false;
    }
    data = (tdt_interface__msg__AutoAimState *)allocator.zero_allocate(size, sizeof(tdt_interface__msg__AutoAimState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tdt_interface__msg__AutoAimState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tdt_interface__msg__AutoAimState__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
tdt_interface__msg__AutoAimState__Sequence__fini(tdt_interface__msg__AutoAimState__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      tdt_interface__msg__AutoAimState__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

tdt_interface__msg__AutoAimState__Sequence *
tdt_interface__msg__AutoAimState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tdt_interface__msg__AutoAimState__Sequence * array = (tdt_interface__msg__AutoAimState__Sequence *)allocator.allocate(sizeof(tdt_interface__msg__AutoAimState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tdt_interface__msg__AutoAimState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tdt_interface__msg__AutoAimState__Sequence__destroy(tdt_interface__msg__AutoAimState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tdt_interface__msg__AutoAimState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tdt_interface__msg__AutoAimState__Sequence__are_equal(const tdt_interface__msg__AutoAimState__Sequence * lhs, const tdt_interface__msg__AutoAimState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tdt_interface__msg__AutoAimState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tdt_interface__msg__AutoAimState__Sequence__copy(
  const tdt_interface__msg__AutoAimState__Sequence * input,
  tdt_interface__msg__AutoAimState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    if (input->size > SIZE_MAX / sizeof(tdt_interface__msg__AutoAimState)) {
      return false;
    }
    const size_t allocation_size =
      input->size * sizeof(tdt_interface__msg__AutoAimState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tdt_interface__msg__AutoAimState * data =
      (tdt_interface__msg__AutoAimState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tdt_interface__msg__AutoAimState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tdt_interface__msg__AutoAimState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tdt_interface__msg__AutoAimState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
