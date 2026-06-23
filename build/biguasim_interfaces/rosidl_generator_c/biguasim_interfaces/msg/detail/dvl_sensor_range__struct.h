// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from biguasim_interfaces:msg/DVLSensorRange.idl
// generated code does not contain a copyright notice

#ifndef BIGUASIM_INTERFACES__MSG__DETAIL__DVL_SENSOR_RANGE__STRUCT_H_
#define BIGUASIM_INTERFACES__MSG__DETAIL__DVL_SENSOR_RANGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/DVLSensorRange in the package biguasim_interfaces.
/**
  * DVLSensor message
  * Contains header, velocity, and range measurements
 */
typedef struct biguasim_interfaces__msg__DVLSensorRange
{
  std_msgs__msg__Header header;
  /// Range measurements in meters from the 4 sonar beams
  float range[4];
} biguasim_interfaces__msg__DVLSensorRange;

// Struct for a sequence of biguasim_interfaces__msg__DVLSensorRange.
typedef struct biguasim_interfaces__msg__DVLSensorRange__Sequence
{
  biguasim_interfaces__msg__DVLSensorRange * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} biguasim_interfaces__msg__DVLSensorRange__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BIGUASIM_INTERFACES__MSG__DETAIL__DVL_SENSOR_RANGE__STRUCT_H_
