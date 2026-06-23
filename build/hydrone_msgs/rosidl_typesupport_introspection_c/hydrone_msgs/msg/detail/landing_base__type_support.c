// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hydrone_msgs:msg/LandingBase.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hydrone_msgs/msg/detail/landing_base__rosidl_typesupport_introspection_c.h"
#include "hydrone_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hydrone_msgs/msg/detail/landing_base__functions.h"
#include "hydrone_msgs/msg/detail/landing_base__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `pose`
#include "geometry_msgs/msg/pose.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void hydrone_msgs__msg__LandingBase__rosidl_typesupport_introspection_c__LandingBase_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hydrone_msgs__msg__LandingBase__init(message_memory);
}

void hydrone_msgs__msg__LandingBase__rosidl_typesupport_introspection_c__LandingBase_fini_function(void * message_memory)
{
  hydrone_msgs__msg__LandingBase__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember hydrone_msgs__msg__LandingBase__rosidl_typesupport_introspection_c__LandingBase_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hydrone_msgs__msg__LandingBase, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "base_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hydrone_msgs__msg__LandingBase, base_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hydrone_msgs__msg__LandingBase, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_suspended",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hydrone_msgs__msg__LandingBase, is_suspended),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_visited",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hydrone_msgs__msg__LandingBase, is_visited),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "confidence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hydrone_msgs__msg__LandingBase, confidence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "height",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hydrone_msgs__msg__LandingBase, height),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers hydrone_msgs__msg__LandingBase__rosidl_typesupport_introspection_c__LandingBase_message_members = {
  "hydrone_msgs__msg",  // message namespace
  "LandingBase",  // message name
  7,  // number of fields
  sizeof(hydrone_msgs__msg__LandingBase),
  hydrone_msgs__msg__LandingBase__rosidl_typesupport_introspection_c__LandingBase_message_member_array,  // message members
  hydrone_msgs__msg__LandingBase__rosidl_typesupport_introspection_c__LandingBase_init_function,  // function to initialize message memory (memory has to be allocated)
  hydrone_msgs__msg__LandingBase__rosidl_typesupport_introspection_c__LandingBase_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t hydrone_msgs__msg__LandingBase__rosidl_typesupport_introspection_c__LandingBase_message_type_support_handle = {
  0,
  &hydrone_msgs__msg__LandingBase__rosidl_typesupport_introspection_c__LandingBase_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hydrone_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hydrone_msgs, msg, LandingBase)() {
  hydrone_msgs__msg__LandingBase__rosidl_typesupport_introspection_c__LandingBase_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  hydrone_msgs__msg__LandingBase__rosidl_typesupport_introspection_c__LandingBase_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!hydrone_msgs__msg__LandingBase__rosidl_typesupport_introspection_c__LandingBase_message_type_support_handle.typesupport_identifier) {
    hydrone_msgs__msg__LandingBase__rosidl_typesupport_introspection_c__LandingBase_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &hydrone_msgs__msg__LandingBase__rosidl_typesupport_introspection_c__LandingBase_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
