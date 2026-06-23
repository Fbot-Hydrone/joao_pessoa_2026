// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hydrone_msgs:msg/HumanGesture.idl
// generated code does not contain a copyright notice

#ifndef HYDRONE_MSGS__MSG__DETAIL__HUMAN_GESTURE__STRUCT_H_
#define HYDRONE_MSGS__MSG__DETAIL__HUMAN_GESTURE__STRUCT_H_

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
// Member 'gesture_name'
#include "rosidl_runtime_c/string.h"
// Member 'human_position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'skeleton_keypoints'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/HumanGesture in the package hydrone_msgs.
/**
  * Human gesture/command detected by vision system (Phase 3)
 */
typedef struct hydrone_msgs__msg__HumanGesture
{
  std_msgs__msg__Header header;
  /// e.g. "arms_up", "point_left", "point_right", etc.
  rosidl_runtime_c__String gesture_name;
  /// Detection confidence 0.0-1.0
  float confidence;
  /// Position of the human in the world
  geometry_msgs__msg__Point human_position;
  /// Flat array of skeleton keypoints (x,y,z per joint)
  rosidl_runtime_c__float__Sequence skeleton_keypoints;
} hydrone_msgs__msg__HumanGesture;

// Struct for a sequence of hydrone_msgs__msg__HumanGesture.
typedef struct hydrone_msgs__msg__HumanGesture__Sequence
{
  hydrone_msgs__msg__HumanGesture * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hydrone_msgs__msg__HumanGesture__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HYDRONE_MSGS__MSG__DETAIL__HUMAN_GESTURE__STRUCT_H_
