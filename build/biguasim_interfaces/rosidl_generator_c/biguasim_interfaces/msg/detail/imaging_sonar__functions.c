// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from biguasim_interfaces:msg/ImagingSonar.idl
// generated code does not contain a copyright notice
#include "biguasim_interfaces/msg/detail/imaging_sonar__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `raw_image`
// Member `intensity`
// Member `elevation`
#include "sensor_msgs/msg/detail/image__functions.h"
// Member `point_cloud`
#include "sensor_msgs/msg/detail/point_cloud2__functions.h"

bool
biguasim_interfaces__msg__ImagingSonar__init(biguasim_interfaces__msg__ImagingSonar * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    biguasim_interfaces__msg__ImagingSonar__fini(msg);
    return false;
  }
  // bins_azimuth
  // bins_range
  // raw_image
  if (!sensor_msgs__msg__Image__init(&msg->raw_image)) {
    biguasim_interfaces__msg__ImagingSonar__fini(msg);
    return false;
  }
  // intensity
  if (!sensor_msgs__msg__Image__init(&msg->intensity)) {
    biguasim_interfaces__msg__ImagingSonar__fini(msg);
    return false;
  }
  // elevation
  if (!sensor_msgs__msg__Image__init(&msg->elevation)) {
    biguasim_interfaces__msg__ImagingSonar__fini(msg);
    return false;
  }
  // point_cloud
  if (!sensor_msgs__msg__PointCloud2__init(&msg->point_cloud)) {
    biguasim_interfaces__msg__ImagingSonar__fini(msg);
    return false;
  }
  return true;
}

void
biguasim_interfaces__msg__ImagingSonar__fini(biguasim_interfaces__msg__ImagingSonar * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // bins_azimuth
  // bins_range
  // raw_image
  sensor_msgs__msg__Image__fini(&msg->raw_image);
  // intensity
  sensor_msgs__msg__Image__fini(&msg->intensity);
  // elevation
  sensor_msgs__msg__Image__fini(&msg->elevation);
  // point_cloud
  sensor_msgs__msg__PointCloud2__fini(&msg->point_cloud);
}

bool
biguasim_interfaces__msg__ImagingSonar__are_equal(const biguasim_interfaces__msg__ImagingSonar * lhs, const biguasim_interfaces__msg__ImagingSonar * rhs)
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
  // bins_azimuth
  if (lhs->bins_azimuth != rhs->bins_azimuth) {
    return false;
  }
  // bins_range
  if (lhs->bins_range != rhs->bins_range) {
    return false;
  }
  // raw_image
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->raw_image), &(rhs->raw_image)))
  {
    return false;
  }
  // intensity
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->intensity), &(rhs->intensity)))
  {
    return false;
  }
  // elevation
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->elevation), &(rhs->elevation)))
  {
    return false;
  }
  // point_cloud
  if (!sensor_msgs__msg__PointCloud2__are_equal(
      &(lhs->point_cloud), &(rhs->point_cloud)))
  {
    return false;
  }
  return true;
}

bool
biguasim_interfaces__msg__ImagingSonar__copy(
  const biguasim_interfaces__msg__ImagingSonar * input,
  biguasim_interfaces__msg__ImagingSonar * output)
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
  // bins_azimuth
  output->bins_azimuth = input->bins_azimuth;
  // bins_range
  output->bins_range = input->bins_range;
  // raw_image
  if (!sensor_msgs__msg__Image__copy(
      &(input->raw_image), &(output->raw_image)))
  {
    return false;
  }
  // intensity
  if (!sensor_msgs__msg__Image__copy(
      &(input->intensity), &(output->intensity)))
  {
    return false;
  }
  // elevation
  if (!sensor_msgs__msg__Image__copy(
      &(input->elevation), &(output->elevation)))
  {
    return false;
  }
  // point_cloud
  if (!sensor_msgs__msg__PointCloud2__copy(
      &(input->point_cloud), &(output->point_cloud)))
  {
    return false;
  }
  return true;
}

biguasim_interfaces__msg__ImagingSonar *
biguasim_interfaces__msg__ImagingSonar__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  biguasim_interfaces__msg__ImagingSonar * msg = (biguasim_interfaces__msg__ImagingSonar *)allocator.allocate(sizeof(biguasim_interfaces__msg__ImagingSonar), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(biguasim_interfaces__msg__ImagingSonar));
  bool success = biguasim_interfaces__msg__ImagingSonar__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
biguasim_interfaces__msg__ImagingSonar__destroy(biguasim_interfaces__msg__ImagingSonar * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    biguasim_interfaces__msg__ImagingSonar__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
biguasim_interfaces__msg__ImagingSonar__Sequence__init(biguasim_interfaces__msg__ImagingSonar__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  biguasim_interfaces__msg__ImagingSonar * data = NULL;

  if (size) {
    data = (biguasim_interfaces__msg__ImagingSonar *)allocator.zero_allocate(size, sizeof(biguasim_interfaces__msg__ImagingSonar), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = biguasim_interfaces__msg__ImagingSonar__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        biguasim_interfaces__msg__ImagingSonar__fini(&data[i - 1]);
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
biguasim_interfaces__msg__ImagingSonar__Sequence__fini(biguasim_interfaces__msg__ImagingSonar__Sequence * array)
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
      biguasim_interfaces__msg__ImagingSonar__fini(&array->data[i]);
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

biguasim_interfaces__msg__ImagingSonar__Sequence *
biguasim_interfaces__msg__ImagingSonar__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  biguasim_interfaces__msg__ImagingSonar__Sequence * array = (biguasim_interfaces__msg__ImagingSonar__Sequence *)allocator.allocate(sizeof(biguasim_interfaces__msg__ImagingSonar__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = biguasim_interfaces__msg__ImagingSonar__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
biguasim_interfaces__msg__ImagingSonar__Sequence__destroy(biguasim_interfaces__msg__ImagingSonar__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    biguasim_interfaces__msg__ImagingSonar__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
biguasim_interfaces__msg__ImagingSonar__Sequence__are_equal(const biguasim_interfaces__msg__ImagingSonar__Sequence * lhs, const biguasim_interfaces__msg__ImagingSonar__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!biguasim_interfaces__msg__ImagingSonar__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
biguasim_interfaces__msg__ImagingSonar__Sequence__copy(
  const biguasim_interfaces__msg__ImagingSonar__Sequence * input,
  biguasim_interfaces__msg__ImagingSonar__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(biguasim_interfaces__msg__ImagingSonar);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    biguasim_interfaces__msg__ImagingSonar * data =
      (biguasim_interfaces__msg__ImagingSonar *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!biguasim_interfaces__msg__ImagingSonar__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          biguasim_interfaces__msg__ImagingSonar__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!biguasim_interfaces__msg__ImagingSonar__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
