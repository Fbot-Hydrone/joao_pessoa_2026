// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from biguasim_interfaces:msg/DVLSensorRange.idl
// generated code does not contain a copyright notice
#include "biguasim_interfaces/msg/detail/dvl_sensor_range__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
biguasim_interfaces__msg__DVLSensorRange__init(biguasim_interfaces__msg__DVLSensorRange * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    biguasim_interfaces__msg__DVLSensorRange__fini(msg);
    return false;
  }
  // range
  return true;
}

void
biguasim_interfaces__msg__DVLSensorRange__fini(biguasim_interfaces__msg__DVLSensorRange * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // range
}

bool
biguasim_interfaces__msg__DVLSensorRange__are_equal(const biguasim_interfaces__msg__DVLSensorRange * lhs, const biguasim_interfaces__msg__DVLSensorRange * rhs)
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
  // range
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->range[i] != rhs->range[i]) {
      return false;
    }
  }
  return true;
}

bool
biguasim_interfaces__msg__DVLSensorRange__copy(
  const biguasim_interfaces__msg__DVLSensorRange * input,
  biguasim_interfaces__msg__DVLSensorRange * output)
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
  // range
  for (size_t i = 0; i < 4; ++i) {
    output->range[i] = input->range[i];
  }
  return true;
}

biguasim_interfaces__msg__DVLSensorRange *
biguasim_interfaces__msg__DVLSensorRange__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  biguasim_interfaces__msg__DVLSensorRange * msg = (biguasim_interfaces__msg__DVLSensorRange *)allocator.allocate(sizeof(biguasim_interfaces__msg__DVLSensorRange), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(biguasim_interfaces__msg__DVLSensorRange));
  bool success = biguasim_interfaces__msg__DVLSensorRange__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
biguasim_interfaces__msg__DVLSensorRange__destroy(biguasim_interfaces__msg__DVLSensorRange * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    biguasim_interfaces__msg__DVLSensorRange__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
biguasim_interfaces__msg__DVLSensorRange__Sequence__init(biguasim_interfaces__msg__DVLSensorRange__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  biguasim_interfaces__msg__DVLSensorRange * data = NULL;

  if (size) {
    data = (biguasim_interfaces__msg__DVLSensorRange *)allocator.zero_allocate(size, sizeof(biguasim_interfaces__msg__DVLSensorRange), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = biguasim_interfaces__msg__DVLSensorRange__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        biguasim_interfaces__msg__DVLSensorRange__fini(&data[i - 1]);
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
biguasim_interfaces__msg__DVLSensorRange__Sequence__fini(biguasim_interfaces__msg__DVLSensorRange__Sequence * array)
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
      biguasim_interfaces__msg__DVLSensorRange__fini(&array->data[i]);
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

biguasim_interfaces__msg__DVLSensorRange__Sequence *
biguasim_interfaces__msg__DVLSensorRange__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  biguasim_interfaces__msg__DVLSensorRange__Sequence * array = (biguasim_interfaces__msg__DVLSensorRange__Sequence *)allocator.allocate(sizeof(biguasim_interfaces__msg__DVLSensorRange__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = biguasim_interfaces__msg__DVLSensorRange__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
biguasim_interfaces__msg__DVLSensorRange__Sequence__destroy(biguasim_interfaces__msg__DVLSensorRange__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    biguasim_interfaces__msg__DVLSensorRange__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
biguasim_interfaces__msg__DVLSensorRange__Sequence__are_equal(const biguasim_interfaces__msg__DVLSensorRange__Sequence * lhs, const biguasim_interfaces__msg__DVLSensorRange__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!biguasim_interfaces__msg__DVLSensorRange__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
biguasim_interfaces__msg__DVLSensorRange__Sequence__copy(
  const biguasim_interfaces__msg__DVLSensorRange__Sequence * input,
  biguasim_interfaces__msg__DVLSensorRange__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(biguasim_interfaces__msg__DVLSensorRange);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    biguasim_interfaces__msg__DVLSensorRange * data =
      (biguasim_interfaces__msg__DVLSensorRange *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!biguasim_interfaces__msg__DVLSensorRange__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          biguasim_interfaces__msg__DVLSensorRange__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!biguasim_interfaces__msg__DVLSensorRange__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
