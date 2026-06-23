// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hydrone_msgs:srv/SetPhase.idl
// generated code does not contain a copyright notice

#ifndef HYDRONE_MSGS__SRV__DETAIL__SET_PHASE__STRUCT_H_
#define HYDRONE_MSGS__SRV__DETAIL__SET_PHASE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetPhase in the package hydrone_msgs.
typedef struct hydrone_msgs__srv__SetPhase_Request
{
  /// 1, 2, 3 or 4
  uint8_t phase;
  /// Using open hardware drone?
  bool open_hardware;
  /// Phase 3 only: use two drones simultaneously?
  bool use_two_drones;
} hydrone_msgs__srv__SetPhase_Request;

// Struct for a sequence of hydrone_msgs__srv__SetPhase_Request.
typedef struct hydrone_msgs__srv__SetPhase_Request__Sequence
{
  hydrone_msgs__srv__SetPhase_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hydrone_msgs__srv__SetPhase_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetPhase in the package hydrone_msgs.
typedef struct hydrone_msgs__srv__SetPhase_Response
{
  bool success;
  rosidl_runtime_c__String message;
} hydrone_msgs__srv__SetPhase_Response;

// Struct for a sequence of hydrone_msgs__srv__SetPhase_Response.
typedef struct hydrone_msgs__srv__SetPhase_Response__Sequence
{
  hydrone_msgs__srv__SetPhase_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hydrone_msgs__srv__SetPhase_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HYDRONE_MSGS__SRV__DETAIL__SET_PHASE__STRUCT_H_
