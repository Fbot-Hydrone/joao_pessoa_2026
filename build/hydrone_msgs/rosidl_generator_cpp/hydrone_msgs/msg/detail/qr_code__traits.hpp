// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hydrone_msgs:msg/QRCode.idl
// generated code does not contain a copyright notice

#ifndef HYDRONE_MSGS__MSG__DETAIL__QR_CODE__TRAITS_HPP_
#define HYDRONE_MSGS__MSG__DETAIL__QR_CODE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hydrone_msgs/msg/detail/qr_code__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace hydrone_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const QRCode & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: qr_id
  {
    out << "qr_id: ";
    rosidl_generator_traits::value_to_yaml(msg.qr_id, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: is_new
  {
    out << "is_new: ";
    rosidl_generator_traits::value_to_yaml(msg.is_new, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const QRCode & msg,
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

  // member: qr_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "qr_id: ";
    rosidl_generator_traits::value_to_yaml(msg.qr_id, out);
    out << "\n";
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: is_new
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_new: ";
    rosidl_generator_traits::value_to_yaml(msg.is_new, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const QRCode & msg, bool use_flow_style = false)
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
  const hydrone_msgs::msg::QRCode & msg,
  std::ostream & out, size_t indentation = 0)
{
  hydrone_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hydrone_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const hydrone_msgs::msg::QRCode & msg)
{
  return hydrone_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<hydrone_msgs::msg::QRCode>()
{
  return "hydrone_msgs::msg::QRCode";
}

template<>
inline const char * name<hydrone_msgs::msg::QRCode>()
{
  return "hydrone_msgs/msg/QRCode";
}

template<>
struct has_fixed_size<hydrone_msgs::msg::QRCode>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<hydrone_msgs::msg::QRCode>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<hydrone_msgs::msg::QRCode>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HYDRONE_MSGS__MSG__DETAIL__QR_CODE__TRAITS_HPP_
