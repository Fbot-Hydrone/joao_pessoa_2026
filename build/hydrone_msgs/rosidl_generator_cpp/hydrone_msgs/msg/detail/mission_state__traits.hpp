// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hydrone_msgs:msg/MissionState.idl
// generated code does not contain a copyright notice

#ifndef HYDRONE_MSGS__MSG__DETAIL__MISSION_STATE__TRAITS_HPP_
#define HYDRONE_MSGS__MSG__DETAIL__MISSION_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hydrone_msgs/msg/detail/mission_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace hydrone_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MissionState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: phase
  {
    out << "phase: ";
    rosidl_generator_traits::value_to_yaml(msg.phase, out);
    out << ", ";
  }

  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: state_name
  {
    out << "state_name: ";
    rosidl_generator_traits::value_to_yaml(msg.state_name, out);
    out << ", ";
  }

  // member: score
  {
    out << "score: ";
    rosidl_generator_traits::value_to_yaml(msg.score, out);
    out << ", ";
  }

  // member: open_hardware
  {
    out << "open_hardware: ";
    rosidl_generator_traits::value_to_yaml(msg.open_hardware, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MissionState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: phase
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "phase: ";
    rosidl_generator_traits::value_to_yaml(msg.phase, out);
    out << "\n";
  }

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }

  // member: state_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state_name: ";
    rosidl_generator_traits::value_to_yaml(msg.state_name, out);
    out << "\n";
  }

  // member: score
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "score: ";
    rosidl_generator_traits::value_to_yaml(msg.score, out);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MissionState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace hydrone_msgs

namespace rosidl_generator_traits
{

[[deprecated("use hydrone_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const hydrone_msgs::msg::MissionState & msg,
  std::ostream & out, size_t indentation = 0)
{
  hydrone_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hydrone_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const hydrone_msgs::msg::MissionState & msg)
{
  return hydrone_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<hydrone_msgs::msg::MissionState>()
{
  return "hydrone_msgs::msg::MissionState";
}

template<>
inline const char * name<hydrone_msgs::msg::MissionState>()
{
  return "hydrone_msgs/msg/MissionState";
}

template<>
struct has_fixed_size<hydrone_msgs::msg::MissionState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<hydrone_msgs::msg::MissionState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<hydrone_msgs::msg::MissionState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HYDRONE_MSGS__MSG__DETAIL__MISSION_STATE__TRAITS_HPP_
