// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tdt_interface:msg/RuneResult.idl
// generated code does not contain a copyright notice
#include "tdt_interface/msg/detail/rune_result__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
tdt_interface__msg__RuneResult__init(tdt_interface__msg__RuneResult * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    tdt_interface__msg__RuneResult__fini(msg);
    return false;
  }
  // request_id
  // accepted
  // activated
  return true;
}

void
tdt_interface__msg__RuneResult__fini(tdt_interface__msg__RuneResult * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // request_id
  // accepted
  // activated
}

bool
tdt_interface__msg__RuneResult__are_equal(const tdt_interface__msg__RuneResult * lhs, const tdt_interface__msg__RuneResult * rhs)
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
  // request_id
  if (lhs->request_id != rhs->request_id) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // activated
  if (lhs->activated != rhs->activated) {
    return false;
  }
  return true;
}

bool
tdt_interface__msg__RuneResult__copy(
  const tdt_interface__msg__RuneResult * input,
  tdt_interface__msg__RuneResult * output)
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
  // request_id
  output->request_id = input->request_id;
  // accepted
  output->accepted = input->accepted;
  // activated
  output->activated = input->activated;
  return true;
}

tdt_interface__msg__RuneResult *
tdt_interface__msg__RuneResult__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tdt_interface__msg__RuneResult * msg = (tdt_interface__msg__RuneResult *)allocator.allocate(sizeof(tdt_interface__msg__RuneResult), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tdt_interface__msg__RuneResult));
  bool success = tdt_interface__msg__RuneResult__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tdt_interface__msg__RuneResult__destroy(tdt_interface__msg__RuneResult * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tdt_interface__msg__RuneResult__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tdt_interface__msg__RuneResult__Sequence__init(tdt_interface__msg__RuneResult__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tdt_interface__msg__RuneResult * data = NULL;

  if (size) {
    if (size > SIZE_MAX / sizeof(tdt_interface__msg__RuneResult)) {
      return false;
    }
    data = (tdt_interface__msg__RuneResult *)allocator.zero_allocate(size, sizeof(tdt_interface__msg__RuneResult), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tdt_interface__msg__RuneResult__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tdt_interface__msg__RuneResult__fini(&data[i - 1]);
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
tdt_interface__msg__RuneResult__Sequence__fini(tdt_interface__msg__RuneResult__Sequence * array)
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
      tdt_interface__msg__RuneResult__fini(&array->data[i]);
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

tdt_interface__msg__RuneResult__Sequence *
tdt_interface__msg__RuneResult__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tdt_interface__msg__RuneResult__Sequence * array = (tdt_interface__msg__RuneResult__Sequence *)allocator.allocate(sizeof(tdt_interface__msg__RuneResult__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tdt_interface__msg__RuneResult__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tdt_interface__msg__RuneResult__Sequence__destroy(tdt_interface__msg__RuneResult__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tdt_interface__msg__RuneResult__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tdt_interface__msg__RuneResult__Sequence__are_equal(const tdt_interface__msg__RuneResult__Sequence * lhs, const tdt_interface__msg__RuneResult__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tdt_interface__msg__RuneResult__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tdt_interface__msg__RuneResult__Sequence__copy(
  const tdt_interface__msg__RuneResult__Sequence * input,
  tdt_interface__msg__RuneResult__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    if (input->size > SIZE_MAX / sizeof(tdt_interface__msg__RuneResult)) {
      return false;
    }
    const size_t allocation_size =
      input->size * sizeof(tdt_interface__msg__RuneResult);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tdt_interface__msg__RuneResult * data =
      (tdt_interface__msg__RuneResult *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tdt_interface__msg__RuneResult__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tdt_interface__msg__RuneResult__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tdt_interface__msg__RuneResult__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
