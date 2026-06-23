// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hydrone_msgs:msg/HumanGesture.idl
// generated code does not contain a copyright notice

#ifndef HYDRONE_MSGS__MSG__DETAIL__HUMAN_GESTURE__TRAITS_HPP_
#define HYDRONE_MSGS__MSG__DETAIL__HUMAN_GESTURE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hydrone_msgs/msg/detail/human_gesture__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'human_position'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace hydrone_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const HumanGesture & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: gesture_name
  {
    out << "gesture_name: ";
    rosidl_generator_traits::value_to_yaml(msg.gesture_name, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << ", ";
  }

  // member: human_position
  {
    out << "human_position: ";
    to_flow_style_yaml(msg.human_position, out);
    out << ", ";
  }

  // member: skeleton_keypoints
  {
    if (msg.skeleton_keypoints.size() == 0) {
      out << "skeleton_keypoints: []";
    } else {
      out << "skeleton_keypoints: [";
      size_t pending_items = msg.skeleton_keypoints.size();
      for (auto item : msg.skeleton_keypoints) {
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
  const HumanGesture & msg,
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

  // member: gesture_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gesture_name: ";
    rosidl_generator_traits::value_to_yaml(msg.gesture_name, out);
    out << "\n";
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }

  // member: human_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "human_position:\n";
    to_block_style_yaml(msg.human_position, out, indentation + 2);
  }

  // member: skeleton_keypoints
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.skeleton_keypoints.size() == 0) {
      out << "skeleton_keypoints: []\n";
    } else {
      out << "skeleton_keypoints:\n";
      for (auto item : msg.skeleton_keypoints) {
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

inline std::string to_yaml(const HumanGesture & msg, bool use_flow_style = false)
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
  const hydrone_msgs::msg::HumanGesture & msg,
  std::ostream & out, size_t indentation = 0)
{
  hydrone_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hydrone_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const hydrone_msgs::msg::HumanGesture & msg)
{
  return hydrone_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<hydrone_msgs::msg::HumanGesture>()
{
  return "hydrone_msgs::msg::HumanGesture";
}

template<>
inline const char * name<hydrone_msgs::msg::HumanGesture>()
{
  return "hydrone_msgs/msg/HumanGesture";
}

template<>
struct has_fixed_size<hydrone_msgs::msg::HumanGesture>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<hydrone_msgs::msg::HumanGesture>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<hydrone_msgs::msg::HumanGesture>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HYDRONE_MSGS__MSG__DETAIL__HUMAN_GESTURE__TRAITS_HPP_
