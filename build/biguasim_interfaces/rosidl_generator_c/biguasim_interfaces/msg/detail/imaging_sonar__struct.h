// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from biguasim_interfaces:msg/ImagingSonar.idl
// generated code does not contain a copyright notice

#ifndef BIGUASIM_INTERFACES__MSG__DETAIL__IMAGING_SONAR__STRUCT_H_
#define BIGUASIM_INTERFACES__MSG__DETAIL__IMAGING_SONAR__STRUCT_H_

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
// Member 'raw_image'
// Member 'intensity'
// Member 'elevation'
#include "sensor_msgs/msg/detail/image__struct.h"
// Member 'point_cloud'
#include "sensor_msgs/msg/detail/point_cloud2__struct.h"

/// Struct defined in msg/ImagingSonar in the package biguasim_interfaces.
/**
  * ImagingSonar message
  * Contains timestamp, bins information, and image data
 */
typedef struct biguasim_interfaces__msg__ImagingSonar
{
  std_msgs__msg__Header header;
  /// Number of azimuth bins
  int32_t bins_azimuth;
  /// Number of range bins
  int32_t bins_range;
  /// Raw sonar image (as received)
  sensor_msgs__msg__Image raw_image;
  /// Ground-truth intensity (float, same size as raw)
  sensor_msgs__msg__Image intensity;
  /// Ground-truth elevation (per-pixel angle or height)
  sensor_msgs__msg__Image elevation;
  /// Ground-truth 3D points
  sensor_msgs__msg__PointCloud2 point_cloud;
} biguasim_interfaces__msg__ImagingSonar;

// Struct for a sequence of biguasim_interfaces__msg__ImagingSonar.
typedef struct biguasim_interfaces__msg__ImagingSonar__Sequence
{
  biguasim_interfaces__msg__ImagingSonar * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} biguasim_interfaces__msg__ImagingSonar__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BIGUASIM_INTERFACES__MSG__DETAIL__IMAGING_SONAR__STRUCT_H_
