// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from biguasim_interfaces:msg/DVLSensorRange.idl
// generated code does not contain a copyright notice

#ifndef BIGUASIM_INTERFACES__MSG__DETAIL__DVL_SENSOR_RANGE__TRAITS_HPP_
#define BIGUASIM_INTERFACES__MSG__DETAIL__DVL_SENSOR_RANGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "biguasim_interfaces/msg/detail/dvl_sensor_range__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace biguasim_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const DVLSensorRange & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: range
  {
    if (msg.range.size() == 0) {
      out << "range: []";
    } else {
      out << "range: [";
      size_t pending_items = msg.range.size();
      for (auto item : msg.range) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DVLSensorRange & msg,
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

  // member: range
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.range.size() == 0) {
      out << "range: []\n";
    } else {
      out << "range:\n";
      for (auto item : msg.range) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DVLSensorRange & msg, bool use_flow_style = false)
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

}  // namespace biguasim_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use biguasim_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const biguasim_interfaces::msg::DVLSensorRange & msg,
  std::ostream & out, size_t indentation = 0)
{
  biguasim_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use biguasim_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const biguasim_interfaces::msg::DVLSensorRange & msg)
{
  return biguasim_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<biguasim_interfaces::msg::DVLSensorRange>()
{
  return "biguasim_interfaces::msg::DVLSensorRange";
}

template<>
inline const char * name<biguasim_interfaces::msg::DVLSensorRange>()
{
  return "biguasim_interfaces/msg/DVLSensorRange";
}

template<>
struct has_fixed_size<biguasim_interfaces::msg::DVLSensorRange>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<biguasim_interfaces::msg::DVLSensorRange>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<biguasim_interfaces::msg::DVLSensorRange>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BIGUASIM_INTERFACES__MSG__DETAIL__DVL_SENSOR_RANGE__TRAITS_HPP_
