// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from biguasim_interfaces:msg/DVLSensorRange.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "biguasim_interfaces/msg/detail/dvl_sensor_range__rosidl_typesupport_introspection_c.h"
#include "biguasim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "biguasim_interfaces/msg/detail/dvl_sensor_range__functions.h"
#include "biguasim_interfaces/msg/detail/dvl_sensor_range__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__DVLSensorRange_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  biguasim_interfaces__msg__DVLSensorRange__init(message_memory);
}

void biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__DVLSensorRange_fini_function(void * message_memory)
{
  biguasim_interfaces__msg__DVLSensorRange__fini(message_memory);
}

size_t biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__size_function__DVLSensorRange__range(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__get_const_function__DVLSensorRange__range(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__get_function__DVLSensorRange__range(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__fetch_function__DVLSensorRange__range(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__get_const_function__DVLSensorRange__range(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__assign_function__DVLSensorRange__range(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__get_function__DVLSensorRange__range(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__DVLSensorRange_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(biguasim_interfaces__msg__DVLSensorRange, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "range",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(biguasim_interfaces__msg__DVLSensorRange, range),  // bytes offset in struct
    NULL,  // default value
    biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__size_function__DVLSensorRange__range,  // size() function pointer
    biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__get_const_function__DVLSensorRange__range,  // get_const(index) function pointer
    biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__get_function__DVLSensorRange__range,  // get(index) function pointer
    biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__fetch_function__DVLSensorRange__range,  // fetch(index, &value) function pointer
    biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__assign_function__DVLSensorRange__range,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__DVLSensorRange_message_members = {
  "biguasim_interfaces__msg",  // message namespace
  "DVLSensorRange",  // message name
  2,  // number of fields
  sizeof(biguasim_interfaces__msg__DVLSensorRange),
  biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__DVLSensorRange_message_member_array,  // message members
  biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__DVLSensorRange_init_function,  // function to initialize message memory (memory has to be allocated)
  biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__DVLSensorRange_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__DVLSensorRange_message_type_support_handle = {
  0,
  &biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__DVLSensorRange_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_biguasim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, biguasim_interfaces, msg, DVLSensorRange)() {
  biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__DVLSensorRange_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__DVLSensorRange_message_type_support_handle.typesupport_identifier) {
    biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__DVLSensorRange_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &biguasim_interfaces__msg__DVLSensorRange__rosidl_typesupport_introspection_c__DVLSensorRange_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
