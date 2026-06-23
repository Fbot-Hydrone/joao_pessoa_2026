// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hydrone_msgs:msg/HumanGesture.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hydrone_msgs/msg/detail/human_gesture__rosidl_typesupport_introspection_c.h"
#include "hydrone_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hydrone_msgs/msg/detail/human_gesture__functions.h"
#include "hydrone_msgs/msg/detail/human_gesture__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `gesture_name`
#include "rosidl_runtime_c/string_functions.h"
// Member `human_position`
#include "geometry_msgs/msg/point.h"
// Member `human_position`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"
// Member `skeleton_keypoints`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__HumanGesture_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hydrone_msgs__msg__HumanGesture__init(message_memory);
}

void hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__HumanGesture_fini_function(void * message_memory)
{
  hydrone_msgs__msg__HumanGesture__fini(message_memory);
}

size_t hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__size_function__HumanGesture__skeleton_keypoints(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__get_const_function__HumanGesture__skeleton_keypoints(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__get_function__HumanGesture__skeleton_keypoints(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__fetch_function__HumanGesture__skeleton_keypoints(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__get_const_function__HumanGesture__skeleton_keypoints(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__assign_function__HumanGesture__skeleton_keypoints(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__get_function__HumanGesture__skeleton_keypoints(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__resize_function__HumanGesture__skeleton_keypoints(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__HumanGesture_message_member_array[5] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hydrone_msgs__msg__HumanGesture, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gesture_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hydrone_msgs__msg__HumanGesture, gesture_name),  // bytes offset in struct
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
    offsetof(hydrone_msgs__msg__HumanGesture, confidence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "human_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hydrone_msgs__msg__HumanGesture, human_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "skeleton_keypoints",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hydrone_msgs__msg__HumanGesture, skeleton_keypoints),  // bytes offset in struct
    NULL,  // default value
    hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__size_function__HumanGesture__skeleton_keypoints,  // size() function pointer
    hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__get_const_function__HumanGesture__skeleton_keypoints,  // get_const(index) function pointer
    hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__get_function__HumanGesture__skeleton_keypoints,  // get(index) function pointer
    hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__fetch_function__HumanGesture__skeleton_keypoints,  // fetch(index, &value) function pointer
    hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__assign_function__HumanGesture__skeleton_keypoints,  // assign(index, value) function pointer
    hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__resize_function__HumanGesture__skeleton_keypoints  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__HumanGesture_message_members = {
  "hydrone_msgs__msg",  // message namespace
  "HumanGesture",  // message name
  5,  // number of fields
  sizeof(hydrone_msgs__msg__HumanGesture),
  hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__HumanGesture_message_member_array,  // message members
  hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__HumanGesture_init_function,  // function to initialize message memory (memory has to be allocated)
  hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__HumanGesture_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__HumanGesture_message_type_support_handle = {
  0,
  &hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__HumanGesture_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hydrone_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hydrone_msgs, msg, HumanGesture)() {
  hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__HumanGesture_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__HumanGesture_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__HumanGesture_message_type_support_handle.typesupport_identifier) {
    hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__HumanGesture_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &hydrone_msgs__msg__HumanGesture__rosidl_typesupport_introspection_c__HumanGesture_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
