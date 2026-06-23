// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hydrone_msgs:msg/LandingBase.idl
// generated code does not contain a copyright notice
#include "hydrone_msgs/msg/detail/landing_base__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
hydrone_msgs__msg__LandingBase__init(hydrone_msgs__msg__LandingBase * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    hydrone_msgs__msg__LandingBase__fini(msg);
    return false;
  }
  // base_id
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    hydrone_msgs__msg__LandingBase__fini(msg);
    return false;
  }
  // is_suspended
  // is_visited
  // confidence
  // height
  return true;
}

void
hydrone_msgs__msg__LandingBase__fini(hydrone_msgs__msg__LandingBase * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // base_id
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // is_suspended
  // is_visited
  // confidence
  // height
}

bool
hydrone_msgs__msg__LandingBase__are_equal(const hydrone_msgs__msg__LandingBase * lhs, const hydrone_msgs__msg__LandingBase * rhs)
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
  // base_id
  if (lhs->base_id != rhs->base_id) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // is_suspended
  if (lhs->is_suspended != rhs->is_suspended) {
    return false;
  }
  // is_visited
  if (lhs->is_visited != rhs->is_visited) {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  return true;
}

bool
hydrone_msgs__msg__LandingBase__copy(
  const hydrone_msgs__msg__LandingBase * input,
  hydrone_msgs__msg__LandingBase * output)
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
  // base_id
  output->base_id = input->base_id;
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // is_suspended
  output->is_suspended = input->is_suspended;
  // is_visited
  output->is_visited = input->is_visited;
  // confidence
  output->confidence = input->confidence;
  // height
  output->height = input->height;
  return true;
}

hydrone_msgs__msg__LandingBase *
hydrone_msgs__msg__LandingBase__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hydrone_msgs__msg__LandingBase * msg = (hydrone_msgs__msg__LandingBase *)allocator.allocate(sizeof(hydrone_msgs__msg__LandingBase), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hydrone_msgs__msg__LandingBase));
  bool success = hydrone_msgs__msg__LandingBase__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hydrone_msgs__msg__LandingBase__destroy(hydrone_msgs__msg__LandingBase * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hydrone_msgs__msg__LandingBase__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hydrone_msgs__msg__LandingBase__Sequence__init(hydrone_msgs__msg__LandingBase__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hydrone_msgs__msg__LandingBase * data = NULL;

  if (size) {
    data = (hydrone_msgs__msg__LandingBase *)allocator.zero_allocate(size, sizeof(hydrone_msgs__msg__LandingBase), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hydrone_msgs__msg__LandingBase__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hydrone_msgs__msg__LandingBase__fini(&data[i - 1]);
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
hydrone_msgs__msg__LandingBase__Sequence__fini(hydrone_msgs__msg__LandingBase__Sequence * array)
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
      hydrone_msgs__msg__LandingBase__fini(&array->data[i]);
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

hydrone_msgs__msg__LandingBase__Sequence *
hydrone_msgs__msg__LandingBase__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hydrone_msgs__msg__LandingBase__Sequence * array = (hydrone_msgs__msg__LandingBase__Sequence *)allocator.allocate(sizeof(hydrone_msgs__msg__LandingBase__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hydrone_msgs__msg__LandingBase__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hydrone_msgs__msg__LandingBase__Sequence__destroy(hydrone_msgs__msg__LandingBase__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hydrone_msgs__msg__LandingBase__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hydrone_msgs__msg__LandingBase__Sequence__are_equal(const hydrone_msgs__msg__LandingBase__Sequence * lhs, const hydrone_msgs__msg__LandingBase__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hydrone_msgs__msg__LandingBase__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hydrone_msgs__msg__LandingBase__Sequence__copy(
  const hydrone_msgs__msg__LandingBase__Sequence * input,
  hydrone_msgs__msg__LandingBase__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hydrone_msgs__msg__LandingBase);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hydrone_msgs__msg__LandingBase * data =
      (hydrone_msgs__msg__LandingBase *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hydrone_msgs__msg__LandingBase__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hydrone_msgs__msg__LandingBase__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hydrone_msgs__msg__LandingBase__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
