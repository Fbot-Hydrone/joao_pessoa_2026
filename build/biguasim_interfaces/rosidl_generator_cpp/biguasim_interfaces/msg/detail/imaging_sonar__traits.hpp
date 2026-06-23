// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from biguasim_interfaces:msg/ImagingSonar.idl
// generated code does not contain a copyright notice

#ifndef BIGUASIM_INTERFACES__MSG__DETAIL__IMAGING_SONAR__TRAITS_HPP_
#define BIGUASIM_INTERFACES__MSG__DETAIL__IMAGING_SONAR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "biguasim_interfaces/msg/detail/imaging_sonar__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'raw_image'
// Member 'intensity'
// Member 'elevation'
#include "sensor_msgs/msg/detail/image__traits.hpp"
// Member 'point_cloud'
#include "sensor_msgs/msg/detail/point_cloud2__traits.hpp"

namespace biguasim_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ImagingSonar & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: bins_azimuth
  {
    out << "bins_azimuth: ";
    rosidl_generator_traits::value_to_yaml(msg.bins_azimuth, out);
    out << ", ";
  }

  // member: bins_range
  {
    out << "bins_range: ";
    rosidl_generator_traits::value_to_yaml(msg.bins_range, out);
    out << ", ";
  }

  // member: raw_image
  {
    out << "raw_image: ";
    to_flow_style_yaml(msg.raw_image, out);
    out << ", ";
  }

  // member: intensity
  {
    out << "intensity: ";
    to_flow_style_yaml(msg.intensity, out);
    out << ", ";
  }

  // member: elevation
  {
    out << "elevation: ";
    to_flow_style_yaml(msg.elevation, out);
    out << ", ";
  }

  // member: point_cloud
  {
    out << "point_cloud: ";
    to_flow_style_yaml(msg.point_cloud, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ImagingSonar & msg,
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

  // member: bins_azimuth
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bins_azimuth: ";
    rosidl_generator_traits::value_to_yaml(msg.bins_azimuth, out);
    out << "\n";
  }

  // member: bins_range
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bins_range: ";
    rosidl_generator_traits::value_to_yaml(msg.bins_range, out);
    out << "\n";
  }

  // member: raw_image
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "raw_image:\n";
    to_block_style_yaml(msg.raw_image, out, indentation + 2);
  }

  // member: intensity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "intensity:\n";
    to_block_style_yaml(msg.intensity, out, indentation + 2);
  }

  // member: elevation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "elevation:\n";
    to_block_style_yaml(msg.elevation, out, indentation + 2);
  }

  // member: point_cloud
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "point_cloud:\n";
    to_block_style_yaml(msg.point_cloud, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ImagingSonar & msg, bool use_flow_style = false)
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
  const biguasim_interfaces::msg::ImagingSonar & msg,
  std::ostream & out, size_t indentation = 0)
{
  biguasim_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use biguasim_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const biguasim_interfaces::msg::ImagingSonar & msg)
{
  return biguasim_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<biguasim_interfaces::msg::ImagingSonar>()
{
  return "biguasim_interfaces::msg::ImagingSonar";
}

template<>
inline const char * name<biguasim_interfaces::msg::ImagingSonar>()
{
  return "biguasim_interfaces/msg/ImagingSonar";
}

template<>
struct has_fixed_size<biguasim_interfaces::msg::ImagingSonar>
  : std::integral_constant<bool, has_fixed_size<sensor_msgs::msg::Image>::value && has_fixed_size<sensor_msgs::msg::PointCloud2>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<biguasim_interfaces::msg::ImagingSonar>
  : std::integral_constant<bool, has_bounded_size<sensor_msgs::msg::Image>::value && has_bounded_size<sensor_msgs::msg::PointCloud2>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<biguasim_interfaces::msg::ImagingSonar>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BIGUASIM_INTERFACES__MSG__DETAIL__IMAGING_SONAR__TRAITS_HPP_
