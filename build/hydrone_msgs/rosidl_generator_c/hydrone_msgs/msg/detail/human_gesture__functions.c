// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hydrone_msgs:msg/HumanGesture.idl
// generated code does not contain a copyright notice
#include "hydrone_msgs/msg/detail/human_gesture__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `gesture_name`
#include "rosidl_runtime_c/string_functions.h"
// Member `human_position`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `skeleton_keypoints`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
hydrone_msgs__msg__HumanGesture__init(hydrone_msgs__msg__HumanGesture * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    hydrone_msgs__msg__HumanGesture__fini(msg);
    return false;
  }
  // gesture_name
  if (!rosidl_runtime_c__String__init(&msg->gesture_name)) {
    hydrone_msgs__msg__HumanGesture__fini(msg);
    return false;
  }
  // confidence
  // human_position
  if (!geometry_msgs__msg__Point__init(&msg->human_position)) {
    hydrone_msgs__msg__HumanGesture__fini(msg);
    return false;
  }
  // skeleton_keypoints
  if (!rosidl_runtime_c__float__Sequence__init(&msg->skeleton_keypoints, 0)) {
    hydrone_msgs__msg__HumanGesture__fini(msg);
    return false;
  }
  return true;
}

void
hydrone_msgs__msg__HumanGesture__fini(hydrone_msgs__msg__HumanGesture * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // gesture_name
  rosidl_runtime_c__String__fini(&msg->gesture_name);
  // confidence
  // human_position
  geometry_msgs__msg__Point__fini(&msg->human_position);
  // skeleton_keypoints
  rosidl_runtime_c__float__Sequence__fini(&msg->skeleton_keypoints);
}

bool
hydrone_msgs__msg__HumanGesture__are_equal(const hydrone_msgs__msg__HumanGesture * lhs, const hydrone_msgs__msg__HumanGesture * rhs)
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
  // gesture_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->gesture_name), &(rhs->gesture_name)))
  {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // human_position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->human_position), &(rhs->human_position)))
  {
    return false;
  }
  // skeleton_keypoints
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->skeleton_keypoints), &(rhs->skeleton_keypoints)))
  {
    return false;
  }
  return true;
}

bool
hydrone_msgs__msg__HumanGesture__copy(
  const hydrone_msgs__msg__HumanGesture * input,
  hydrone_msgs__msg__HumanGesture * output)
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
  // gesture_name
  if (!rosidl_runtime_c__String__copy(
      &(input->gesture_name), &(output->gesture_name)))
  {
    return false;
  }
  // confidence
  output->confidence = input->confidence;
  // human_position
  if (!geometry_msgs__msg__Point__copy(
      &(input->human_position), &(output->human_position)))
  {
    return false;
  }
  // skeleton_keypoints
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->skeleton_keypoints), &(output->skeleton_keypoints)))
  {
    return false;
  }
  return true;
}

hydrone_msgs__msg__HumanGesture *
hydrone_msgs__msg__HumanGesture__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hydrone_msgs__msg__HumanGesture * msg = (hydrone_msgs__msg__HumanGesture *)allocator.allocate(sizeof(hydrone_msgs__msg__HumanGesture), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hydrone_msgs__msg__HumanGesture));
  bool success = hydrone_msgs__msg__HumanGesture__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hydrone_msgs__msg__HumanGesture__destroy(hydrone_msgs__msg__HumanGesture * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hydrone_msgs__msg__HumanGesture__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hydrone_msgs__msg__HumanGesture__Sequence__init(hydrone_msgs__msg__HumanGesture__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hydrone_msgs__msg__HumanGesture * data = NULL;

  if (size) {
    data = (hydrone_msgs__msg__HumanGesture *)allocator.zero_allocate(size, sizeof(hydrone_msgs__msg__HumanGesture), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hydrone_msgs__msg__HumanGesture__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hydrone_msgs__msg__HumanGesture__fini(&data[i - 1]);
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
hydrone_msgs__msg__HumanGesture__Sequence__fini(hydrone_msgs__msg__HumanGesture__Sequence * array)
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
      hydrone_msgs__msg__HumanGesture__fini(&array->data[i]);
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

hydrone_msgs__msg__HumanGesture__Sequence *
hydrone_msgs__msg__HumanGesture__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hydrone_msgs__msg__HumanGesture__Sequence * array = (hydrone_msgs__msg__HumanGesture__Sequence *)allocator.allocate(sizeof(hydrone_msgs__msg__HumanGesture__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hydrone_msgs__msg__HumanGesture__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hydrone_msgs__msg__HumanGesture__Sequence__destroy(hydrone_msgs__msg__HumanGesture__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hydrone_msgs__msg__HumanGesture__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hydrone_msgs__msg__HumanGesture__Sequence__are_equal(const hydrone_msgs__msg__HumanGesture__Sequence * lhs, const hydrone_msgs__msg__HumanGesture__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hydrone_msgs__msg__HumanGesture__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hydrone_msgs__msg__HumanGesture__Sequence__copy(
  const hydrone_msgs__msg__HumanGesture__Sequence * input,
  hydrone_msgs__msg__HumanGesture__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hydrone_msgs__msg__HumanGesture);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hydrone_msgs__msg__HumanGesture * data =
      (hydrone_msgs__msg__HumanGesture *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hydrone_msgs__msg__HumanGesture__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hydrone_msgs__msg__HumanGesture__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hydrone_msgs__msg__HumanGesture__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
