// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from biguasim_interfaces:msg/ImagingSonar.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "biguasim_interfaces/msg/detail/imaging_sonar__rosidl_typesupport_introspection_c.h"
#include "biguasim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "biguasim_interfaces/msg/detail/imaging_sonar__functions.h"
#include "biguasim_interfaces/msg/detail/imaging_sonar__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `raw_image`
// Member `intensity`
// Member `elevation`
#include "sensor_msgs/msg/image.h"
// Member `raw_image`
// Member `intensity`
// Member `elevation`
#include "sensor_msgs/msg/detail/image__rosidl_typesupport_introspection_c.h"
// Member `point_cloud`
#include "sensor_msgs/msg/point_cloud2.h"
// Member `point_cloud`
#include "sensor_msgs/msg/detail/point_cloud2__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void biguasim_interfaces__msg__ImagingSonar__rosidl_typesupport_introspection_c__ImagingSonar_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  biguasim_interfaces__msg__ImagingSonar__init(message_memory);
}

void biguasim_interfaces__msg__ImagingSonar__rosidl_typesupport_introspection_c__ImagingSonar_fini_function(void * message_memory)
{
  biguasim_interfaces__msg__ImagingSonar__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember biguasim_interfaces__msg__ImagingSonar__rosidl_typesupport_introspection_c__ImagingSonar_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(biguasim_interfaces__msg__ImagingSonar, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bins_azimuth",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(biguasim_interfaces__msg__ImagingSonar, bins_azimuth),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bins_range",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(biguasim_interfaces__msg__ImagingSonar, bins_range),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "raw_image",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(biguasim_interfaces__msg__ImagingSonar, raw_image),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "intensity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(biguasim_interfaces__msg__ImagingSonar, intensity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "elevation",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(biguasim_interfaces__msg__ImagingSonar, elevation),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "point_cloud",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(biguasim_interfaces__msg__ImagingSonar, point_cloud),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers biguasim_interfaces__msg__ImagingSonar__rosidl_typesupport_introspection_c__ImagingSonar_message_members = {
  "biguasim_interfaces__msg",  // message namespace
  "ImagingSonar",  // message name
  7,  // number of fields
  sizeof(biguasim_interfaces__msg__ImagingSonar),
  biguasim_interfaces__msg__ImagingSonar__rosidl_typesupport_introspection_c__ImagingSonar_message_member_array,  // message members
  biguasim_interfaces__msg__ImagingSonar__rosidl_typesupport_introspection_c__ImagingSonar_init_function,  // function to initialize message memory (memory has to be allocated)
  biguasim_interfaces__msg__ImagingSonar__rosidl_typesupport_introspection_c__ImagingSonar_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t biguasim_interfaces__msg__ImagingSonar__rosidl_typesupport_introspection_c__ImagingSonar_message_type_support_handle = {
  0,
  &biguasim_interfaces__msg__ImagingSonar__rosidl_typesupport_introspection_c__ImagingSonar_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_biguasim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, biguasim_interfaces, msg, ImagingSonar)() {
  biguasim_interfaces__msg__ImagingSonar__rosidl_typesupport_introspection_c__ImagingSonar_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  biguasim_interfaces__msg__ImagingSonar__rosidl_typesupport_introspection_c__ImagingSonar_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Image)();
  biguasim_interfaces__msg__ImagingSonar__rosidl_typesupport_introspection_c__ImagingSonar_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Image)();
  biguasim_interfaces__msg__ImagingSonar__rosidl_typesupport_introspection_c__ImagingSonar_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Image)();
  biguasim_interfaces__msg__ImagingSonar__rosidl_typesupport_introspection_c__ImagingSonar_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, PointCloud2)();
  if (!biguasim_interfaces__msg__ImagingSonar__rosidl_typesupport_introspection_c__ImagingSonar_message_type_support_handle.typesupport_identifier) {
    biguasim_interfaces__msg__ImagingSonar__rosidl_typesupport_introspection_c__ImagingSonar_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &biguasim_interfaces__msg__ImagingSonar__rosidl_typesupport_introspection_c__ImagingSonar_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
