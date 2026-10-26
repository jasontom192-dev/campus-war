// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tdt_interface:msg/AutoAimFrame.idl
// generated code does not contain a copyright notice
#include "tdt_interface/msg/detail/auto_aim_frame__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `state`
#include "tdt_interface/msg/detail/auto_aim_state__functions.h"
// Member `camera_info`
#include "sensor_msgs/msg/detail/camera_info__functions.h"
// Member `image`
#include "sensor_msgs/msg/detail/image__functions.h"
// Member `compressed_image`
#include "sensor_msgs/msg/detail/compressed_image__functions.h"

bool
tdt_interface__msg__AutoAimFrame__init(tdt_interface__msg__AutoAimFrame * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    tdt_interface__msg__AutoAimFrame__fini(msg);
    return false;
  }
  // state
  if (!tdt_interface__msg__AutoAimState__init(&msg->state)) {
    tdt_interface__msg__AutoAimFrame__fini(msg);
    return false;
  }
  // camera_info
  if (!sensor_msgs__msg__CameraInfo__init(&msg->camera_info)) {
    tdt_interface__msg__AutoAimFrame__fini(msg);
    return false;
  }
  // image
  if (!sensor_msgs__msg__Image__init(&msg->image)) {
    tdt_interface__msg__AutoAimFrame__fini(msg);
    return false;
  }
  // compressed_image
  if (!sensor_msgs__msg__CompressedImage__init(&msg->compressed_image)) {
    tdt_interface__msg__AutoAimFrame__fini(msg);
    return false;
  }
  // capture_to_publish_ms
  return true;
}

void
tdt_interface__msg__AutoAimFrame__fini(tdt_interface__msg__AutoAimFrame * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // state
  tdt_interface__msg__AutoAimState__fini(&msg->state);
  // camera_info
  sensor_msgs__msg__CameraInfo__fini(&msg->camera_info);
  // image
  sensor_msgs__msg__Image__fini(&msg->image);
  // compressed_image
  sensor_msgs__msg__CompressedImage__fini(&msg->compressed_image);
  // capture_to_publish_ms
}

bool
tdt_interface__msg__AutoAimFrame__are_equal(const tdt_interface__msg__AutoAimFrame * lhs, const tdt_interface__msg__AutoAimFrame * rhs)
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
  // state
  if (!tdt_interface__msg__AutoAimState__are_equal(
      &(lhs->state), &(rhs->state)))
  {
    return false;
  }
  // camera_info
  if (!sensor_msgs__msg__CameraInfo__are_equal(
      &(lhs->camera_info), &(rhs->camera_info)))
  {
    return false;
  }
  // image
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->image), &(rhs->image)))
  {
    return false;
  }
  // compressed_image
  if (!sensor_msgs__msg__CompressedImage__are_equal(
      &(lhs->compressed_image), &(rhs->compressed_image)))
  {
    return false;
  }
  // capture_to_publish_ms
  if (lhs->capture_to_publish_ms != rhs->capture_to_publish_ms) {
    return false;
  }
  return true;
}

bool
tdt_interface__msg__AutoAimFrame__copy(
  const tdt_interface__msg__AutoAimFrame * input,
  tdt_interface__msg__AutoAimFrame * output)
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
  // state
  if (!tdt_interface__msg__AutoAimState__copy(
      &(input->state), &(output->state)))
  {
    return false;
  }
  // camera_info
  if (!sensor_msgs__msg__CameraInfo__copy(
      &(input->camera_info), &(output->camera_info)))
  {
    return false;
  }
  // image
  if (!sensor_msgs__msg__Image__copy(
      &(input->image), &(output->image)))
  {
    return false;
  }
  // compressed_image
  if (!sensor_msgs__msg__CompressedImage__copy(
      &(input->compressed_image), &(output->compressed_image)))
  {
    return false;
  }
  // capture_to_publish_ms
  output->capture_to_publish_ms = input->capture_to_publish_ms;
  return true;
}

tdt_interface__msg__AutoAimFrame *
tdt_interface__msg__AutoAimFrame__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tdt_interface__msg__AutoAimFrame * msg = (tdt_interface__msg__AutoAimFrame *)allocator.allocate(sizeof(tdt_interface__msg__AutoAimFrame), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tdt_interface__msg__AutoAimFrame));
  bool success = tdt_interface__msg__AutoAimFrame__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tdt_interface__msg__AutoAimFrame__destroy(tdt_interface__msg__AutoAimFrame * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tdt_interface__msg__AutoAimFrame__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tdt_interface__msg__AutoAimFrame__Sequence__init(tdt_interface__msg__AutoAimFrame__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tdt_interface__msg__AutoAimFrame * data = NULL;

  if (size) {
    if (size > SIZE_MAX / sizeof(tdt_interface__msg__AutoAimFrame)) {
      return false;
    }
    data = (tdt_interface__msg__AutoAimFrame *)allocator.zero_allocate(size, sizeof(tdt_interface__msg__AutoAimFrame), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tdt_interface__msg__AutoAimFrame__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tdt_interface__msg__AutoAimFrame__fini(&data[i - 1]);
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
tdt_interface__msg__AutoAimFrame__Sequence__fini(tdt_interface__msg__AutoAimFrame__Sequence * array)
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
      tdt_interface__msg__AutoAimFrame__fini(&array->data[i]);
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

tdt_interface__msg__AutoAimFrame__Sequence *
tdt_interface__msg__AutoAimFrame__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tdt_interface__msg__AutoAimFrame__Sequence * array = (tdt_interface__msg__AutoAimFrame__Sequence *)allocator.allocate(sizeof(tdt_interface__msg__AutoAimFrame__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tdt_interface__msg__AutoAimFrame__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tdt_interface__msg__AutoAimFrame__Sequence__destroy(tdt_interface__msg__AutoAimFrame__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tdt_interface__msg__AutoAimFrame__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tdt_interface__msg__AutoAimFrame__Sequence__are_equal(const tdt_interface__msg__AutoAimFrame__Sequence * lhs, const tdt_interface__msg__AutoAimFrame__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tdt_interface__msg__AutoAimFrame__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tdt_interface__msg__AutoAimFrame__Sequence__copy(
  const tdt_interface__msg__AutoAimFrame__Sequence * input,
  tdt_interface__msg__AutoAimFrame__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    if (input->size > SIZE_MAX / sizeof(tdt_interface__msg__AutoAimFrame)) {
      return false;
    }
    const size_t allocation_size =
      input->size * sizeof(tdt_interface__msg__AutoAimFrame);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tdt_interface__msg__AutoAimFrame * data =
      (tdt_interface__msg__AutoAimFrame *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tdt_interface__msg__AutoAimFrame__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tdt_interface__msg__AutoAimFrame__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tdt_interface__msg__AutoAimFrame__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
