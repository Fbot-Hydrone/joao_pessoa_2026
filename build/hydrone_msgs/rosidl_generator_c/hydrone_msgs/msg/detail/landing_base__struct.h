// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hydrone_msgs:msg/LandingBase.idl
// generated code does not contain a copyright notice

#ifndef HYDRONE_MSGS__MSG__DETAIL__LANDING_BASE__STRUCT_H_
#define HYDRONE_MSGS__MSG__DETAIL__LANDING_BASE__STRUCT_H_

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
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/LandingBase in the package hydrone_msgs.
/**
  * Represents a detected landing base in the arena
 */
typedef struct hydrone_msgs__msg__LandingBase
{
  std_msgs__msg__Header header;
  /// 0-5 (6 bases total)
  uint8_t base_id;
  /// Position and orientation of the base
  geometry_msgs__msg__Pose pose;
  /// True if base is elevated (suspended)
  bool is_suspended;
  /// Whether drone has already landed here
  bool is_visited;
  /// Detection confidence 0.0-1.0
  float confidence;
  /// Height above ground in meters
  float height;
} hydrone_msgs__msg__LandingBase;

// Struct for a sequence of hydrone_msgs__msg__LandingBase.
typedef struct hydrone_msgs__msg__LandingBase__Sequence
{
  hydrone_msgs__msg__LandingBase * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hydrone_msgs__msg__LandingBase__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HYDRONE_MSGS__MSG__DETAIL__LANDING_BASE__STRUCT_H_
