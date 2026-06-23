// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hydrone_msgs:msg/QRCode.idl
// generated code does not contain a copyright notice

#ifndef HYDRONE_MSGS__MSG__DETAIL__QR_CODE__STRUCT_H_
#define HYDRONE_MSGS__MSG__DETAIL__QR_CODE__STRUCT_H_

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
// Member 'qr_id'
#include "rosidl_runtime_c/string.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/QRCode in the package hydrone_msgs.
/**
  * QR Code detection result (Phase 4)
 */
typedef struct hydrone_msgs__msg__QRCode
{
  std_msgs__msg__Header header;
  /// Letter identifier: A, B, C, D or E
  rosidl_runtime_c__String qr_id;
  /// Estimated pose of the QR code in the world
  geometry_msgs__msg__Pose pose;
  /// True if this is the first time this QR was detected
  bool is_new;
} hydrone_msgs__msg__QRCode;

// Struct for a sequence of hydrone_msgs__msg__QRCode.
typedef struct hydrone_msgs__msg__QRCode__Sequence
{
  hydrone_msgs__msg__QRCode * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hydrone_msgs__msg__QRCode__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HYDRONE_MSGS__MSG__DETAIL__QR_CODE__STRUCT_H_
