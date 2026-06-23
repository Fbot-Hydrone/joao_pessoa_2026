// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hydrone_msgs:srv/SetPhase.idl
// generated code does not contain a copyright notice

#ifndef HYDRONE_MSGS__SRV__DETAIL__SET_PHASE__TRAITS_HPP_
#define HYDRONE_MSGS__SRV__DETAIL__SET_PHASE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hydrone_msgs/srv/detail/set_phase__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace hydrone_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetPhase_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: phase
  {
    out << "phase: ";
    rosidl_generator_traits::value_to_yaml(msg.phase, out);
    out << ", ";
  }

  // member: open_hardware
  {
    out << "open_hardware: ";
    rosidl_generator_traits::value_to_yaml(msg.open_hardware, out);
    out << ", ";
  }

  // member: use_two_drones
  {
    out << "use_two_drones: ";
    rosidl_generator_traits::value_to_yaml(msg.use_two_drones, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetPhase_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: phase
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "phase: ";
    rosidl_generator_traits::value_to_yaml(msg.phase, out);
    out << "\n";
  }

  // member: open_hardware
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "open_hardware: ";
    rosidl_generator_traits::value_to_yaml(msg.open_hardware, out);
    out << "\n";
  }

  // member: use_two_drones
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "use_two_drones: ";
    rosidl_generator_traits::value_to_yaml(msg.use_two_drones, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetPhase_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace hydrone_msgs

namespace rosidl_generator_traits
{

[[deprecated("use hydrone_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const hydrone_msgs::srv::SetPhase_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  hydrone_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hydrone_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const hydrone_msgs::srv::SetPhase_Request & msg)
{
  return hydrone_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<hydrone_msgs::srv::SetPhase_Request>()
{
  return "hydrone_msgs::srv::SetPhase_Request";
}

template<>
inline const char * name<hydrone_msgs::srv::SetPhase_Request>()
{
  return "hydrone_msgs/srv/SetPhase_Request";
}

template<>
struct has_fixed_size<hydrone_msgs::srv::SetPhase_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hydrone_msgs::srv::SetPhase_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hydrone_msgs::srv::SetPhase_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace hydrone_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetPhase_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetPhase_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetPhase_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace hydrone_msgs

namespace rosidl_generator_traits
{

[[deprecated("use hydrone_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const hydrone_msgs::srv::SetPhase_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  hydrone_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hydrone_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const hydrone_msgs::srv::SetPhase_Response & msg)
{
  return hydrone_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<hydrone_msgs::srv::SetPhase_Response>()
{
  return "hydrone_msgs::srv::SetPhase_Response";
}

template<>
inline const char * name<hydrone_msgs::srv::SetPhase_Response>()
{
  return "hydrone_msgs/srv/SetPhase_Response";
}

template<>
struct has_fixed_size<hydrone_msgs::srv::SetPhase_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<hydrone_msgs::srv::SetPhase_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<hydrone_msgs::srv::SetPhase_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hydrone_msgs::srv::SetPhase>()
{
  return "hydrone_msgs::srv::SetPhase";
}

template<>
inline const char * name<hydrone_msgs::srv::SetPhase>()
{
  return "hydrone_msgs/srv/SetPhase";
}

template<>
struct has_fixed_size<hydrone_msgs::srv::SetPhase>
  : std::integral_constant<
    bool,
    has_fixed_size<hydrone_msgs::srv::SetPhase_Request>::value &&
    has_fixed_size<hydrone_msgs::srv::SetPhase_Response>::value
  >
{
};

template<>
struct has_bounded_size<hydrone_msgs::srv::SetPhase>
  : std::integral_constant<
    bool,
    has_bounded_size<hydrone_msgs::srv::SetPhase_Request>::value &&
    has_bounded_size<hydrone_msgs::srv::SetPhase_Response>::value
  >
{
};

template<>
struct is_service<hydrone_msgs::srv::SetPhase>
  : std::true_type
{
};

template<>
struct is_service_request<hydrone_msgs::srv::SetPhase_Request>
  : std::true_type
{
};

template<>
struct is_service_response<hydrone_msgs::srv::SetPhase_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // HYDRONE_MSGS__SRV__DETAIL__SET_PHASE__TRAITS_HPP_
