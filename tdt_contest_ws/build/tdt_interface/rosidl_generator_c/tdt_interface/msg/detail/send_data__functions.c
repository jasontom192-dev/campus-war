// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tdt_interface:msg/SendData.idl
// generated code does not contain a copyright notice
#include "tdt_interface/msg/detail/send_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
tdt_interface__msg__SendData__init(tdt_interface__msg__SendData * msg)
{
  if (!msg) {
    return false;
  }
  // yaw
  // pitch
  // if_shoot
  return true;
}

void
tdt_interface__msg__SendData__fini(tdt_interface__msg__SendData * msg)
{
  if (!msg) {
    return;
  }
  // yaw
  // pitch
  // if_shoot
}

bool
tdt_interface__msg__SendData__are_equal(const tdt_interface__msg__SendData * lhs, const tdt_interface__msg__SendData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // if_shoot
  if (lhs->if_shoot != rhs->if_shoot) {
    return false;
  }
  return true;
}

bool
tdt_interface__msg__SendData__copy(
  const tdt_interface__msg__SendData * input,
  tdt_interface__msg__SendData * output)
{
  if (!input || !output) {
    return false;
  }
  // yaw
  output->yaw = input->yaw;
  // pitch
  output->pitch = input->pitch;
  // if_shoot
  output->if_shoot = input->if_shoot;
  return true;
}

tdt_interface__msg__SendData *
tdt_interface__msg__SendData__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tdt_interface__msg__SendData * msg = (tdt_interface__msg__SendData *)allocator.allocate(sizeof(tdt_interface__msg__SendData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tdt_interface__msg__SendData));
  bool success = tdt_interface__msg__SendData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tdt_interface__msg__SendData__destroy(tdt_interface__msg__SendData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tdt_interface__msg__SendData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tdt_interface__msg__SendData__Sequence__init(tdt_interface__msg__SendData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tdt_interface__msg__SendData * data = NULL;

  if (size) {
    if (size > SIZE_MAX / sizeof(tdt_interface__msg__SendData)) {
      return false;
    }
    data = (tdt_interface__msg__SendData *)allocator.zero_allocate(size, sizeof(tdt_interface__msg__SendData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tdt_interface__msg__SendData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tdt_interface__msg__SendData__fini(&data[i - 1]);
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
tdt_interface__msg__SendData__Sequence__fini(tdt_interface__msg__SendData__Sequence * array)
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
      tdt_interface__msg__SendData__fini(&array->data[i]);
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

tdt_interface__msg__SendData__Sequence *
tdt_interface__msg__SendData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tdt_interface__msg__SendData__Sequence * array = (tdt_interface__msg__SendData__Sequence *)allocator.allocate(sizeof(tdt_interface__msg__SendData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tdt_interface__msg__SendData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tdt_interface__msg__SendData__Sequence__destroy(tdt_interface__msg__SendData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tdt_interface__msg__SendData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tdt_interface__msg__SendData__Sequence__are_equal(const tdt_interface__msg__SendData__Sequence * lhs, const tdt_interface__msg__SendData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tdt_interface__msg__SendData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tdt_interface__msg__SendData__Sequence__copy(
  const tdt_interface__msg__SendData__Sequence * input,
  tdt_interface__msg__SendData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    if (input->size > SIZE_MAX / sizeof(tdt_interface__msg__SendData)) {
      return false;
    }
    const size_t allocation_size =
      input->size * sizeof(tdt_interface__msg__SendData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tdt_interface__msg__SendData * data =
      (tdt_interface__msg__SendData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tdt_interface__msg__SendData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tdt_interface__msg__SendData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tdt_interface__msg__SendData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
