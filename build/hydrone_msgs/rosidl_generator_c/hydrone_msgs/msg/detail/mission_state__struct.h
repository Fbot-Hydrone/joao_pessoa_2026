// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hydrone_msgs:msg/MissionState.idl
// generated code does not contain a copyright notice

#ifndef HYDRONE_MSGS__MSG__DETAIL__MISSION_STATE__STRUCT_H_
#define HYDRONE_MSGS__MSG__DETAIL__MISSION_STATE__STRUCT_H_

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
// Member 'state_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/MissionState in the package hydrone_msgs.
/**
  * Mission state for the state machine
 */
typedef struct hydrone_msgs__msg__MissionState
{
  std_msgs__msg__Header header;
  /// Competition phase: 1, 2, 3 or 4
  uint8_t phase;
  /// Current state within the mission
  uint8_t state;
  /// Human-readable state name
  rosidl_runtime_c__String state_name;
  /// Accumulated score for current attempt
  float score;
  /// True if using open hardware (2x multiplier)
  bool open_hardware;
} hydrone_msgs__msg__MissionState;

// Struct for a sequence of hydrone_msgs__msg__MissionState.
typedef struct hydrone_msgs__msg__MissionState__Sequence
{
  hydrone_msgs__msg__MissionState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hydrone_msgs__msg__MissionState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HYDRONE_MSGS__MSG__DETAIL__MISSION_STATE__STRUCT_H_
