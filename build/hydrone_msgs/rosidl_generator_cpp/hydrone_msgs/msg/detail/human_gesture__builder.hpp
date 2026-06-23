// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hydrone_msgs:msg/HumanGesture.idl
// generated code does not contain a copyright notice

#ifndef HYDRONE_MSGS__MSG__DETAIL__HUMAN_GESTURE__BUILDER_HPP_
#define HYDRONE_MSGS__MSG__DETAIL__HUMAN_GESTURE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hydrone_msgs/msg/detail/human_gesture__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hydrone_msgs
{

namespace msg
{

namespace builder
{

class Init_HumanGesture_skeleton_keypoints
{
public:
  explicit Init_HumanGesture_skeleton_keypoints(::hydrone_msgs::msg::HumanGesture & msg)
  : msg_(msg)
  {}
  ::hydrone_msgs::msg::HumanGesture skeleton_keypoints(::hydrone_msgs::msg::HumanGesture::_skeleton_keypoints_type arg)
  {
    msg_.skeleton_keypoints = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hydrone_msgs::msg::HumanGesture msg_;
};

class Init_HumanGesture_human_position
{
public:
  explicit Init_HumanGesture_human_position(::hydrone_msgs::msg::HumanGesture & msg)
  : msg_(msg)
  {}
  Init_HumanGesture_skeleton_keypoints human_position(::hydrone_msgs::msg::HumanGesture::_human_position_type arg)
  {
    msg_.human_position = std::move(arg);
    return Init_HumanGesture_skeleton_keypoints(msg_);
  }

private:
  ::hydrone_msgs::msg::HumanGesture msg_;
};

class Init_HumanGesture_confidence
{
public:
  explicit Init_HumanGesture_confidence(::hydrone_msgs::msg::HumanGesture & msg)
  : msg_(msg)
  {}
  Init_HumanGesture_human_position confidence(::hydrone_msgs::msg::HumanGesture::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_HumanGesture_human_position(msg_);
  }

private:
  ::hydrone_msgs::msg::HumanGesture msg_;
};

class Init_HumanGesture_gesture_name
{
public:
  explicit Init_HumanGesture_gesture_name(::hydrone_msgs::msg::HumanGesture & msg)
  : msg_(msg)
  {}
  Init_HumanGesture_confidence gesture_name(::hydrone_msgs::msg::HumanGesture::_gesture_name_type arg)
  {
    msg_.gesture_name = std::move(arg);
    return Init_HumanGesture_confidence(msg_);
  }

private:
  ::hydrone_msgs::msg::HumanGesture msg_;
};

class Init_HumanGesture_header
{
public:
  Init_HumanGesture_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HumanGesture_gesture_name header(::hydrone_msgs::msg::HumanGesture::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_HumanGesture_gesture_name(msg_);
  }

private:
  ::hydrone_msgs::msg::HumanGesture msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hydrone_msgs::msg::HumanGesture>()
{
  return hydrone_msgs::msg::builder::Init_HumanGesture_header();
}

}  // namespace hydrone_msgs

#endif  // HYDRONE_MSGS__MSG__DETAIL__HUMAN_GESTURE__BUILDER_HPP_
