// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from biguasim_interfaces:msg/ImagingSonar.idl
// generated code does not contain a copyright notice

#ifndef BIGUASIM_INTERFACES__MSG__DETAIL__IMAGING_SONAR__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define BIGUASIM_INTERFACES__MSG__DETAIL__IMAGING_SONAR__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "biguasim_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "biguasim_interfaces/msg/detail/imaging_sonar__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace biguasim_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_biguasim_interfaces
cdr_serialize(
  const biguasim_interfaces::msg::ImagingSonar & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_biguasim_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  biguasim_interfaces::msg::ImagingSonar & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_biguasim_interfaces
get_serialized_size(
  const biguasim_interfaces::msg::ImagingSonar & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_biguasim_interfaces
max_serialized_size_ImagingSonar(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace biguasim_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_biguasim_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, biguasim_interfaces, msg, ImagingSonar)();

#ifdef __cplusplus
}
#endif

#endif  // BIGUASIM_INTERFACES__MSG__DETAIL__IMAGING_SONAR__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
