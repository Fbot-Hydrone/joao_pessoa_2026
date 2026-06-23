// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hydrone_msgs:msg/LandingBase.idl
// generated code does not contain a copyright notice

#ifndef HYDRONE_MSGS__MSG__DETAIL__LANDING_BASE__BUILDER_HPP_
#define HYDRONE_MSGS__MSG__DETAIL__LANDING_BASE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hydrone_msgs/msg/detail/landing_base__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hydrone_msgs
{

namespace msg
{

namespace builder
{

class Init_LandingBase_height
{
public:
  explicit Init_LandingBase_height(::hydrone_msgs::msg::LandingBase & msg)
  : msg_(msg)
  {}
  ::hydrone_msgs::msg::LandingBase height(::hydrone_msgs::msg::LandingBase::_height_type arg)
  {
    msg_.height = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hydrone_msgs::msg::LandingBase msg_;
};

class Init_LandingBase_confidence
{
public:
  explicit Init_LandingBase_confidence(::hydrone_msgs::msg::LandingBase & msg)
  : msg_(msg)
  {}
  Init_LandingBase_height confidence(::hydrone_msgs::msg::LandingBase::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_LandingBase_height(msg_);
  }

private:
  ::hydrone_msgs::msg::LandingBase msg_;
};

class Init_LandingBase_is_visited
{
public:
  explicit Init_LandingBase_is_visited(::hydrone_msgs::msg::LandingBase & msg)
  : msg_(msg)
  {}
  Init_LandingBase_confidence is_visited(::hydrone_msgs::msg::LandingBase::_is_visited_type arg)
  {
    msg_.is_visited = std::move(arg);
    return Init_LandingBase_confidence(msg_);
  }

private:
  ::hydrone_msgs::msg::LandingBase msg_;
};

class Init_LandingBase_is_suspended
{
public:
  explicit Init_LandingBase_is_suspended(::hydrone_msgs::msg::LandingBase & msg)
  : msg_(msg)
  {}
  Init_LandingBase_is_visited is_suspended(::hydrone_msgs::msg::LandingBase::_is_suspended_type arg)
  {
    msg_.is_suspended = std::move(arg);
    return Init_LandingBase_is_visited(msg_);
  }

private:
  ::hydrone_msgs::msg::LandingBase msg_;
};

class Init_LandingBase_pose
{
public:
  explicit Init_LandingBase_pose(::hydrone_msgs::msg::LandingBase & msg)
  : msg_(msg)
  {}
  Init_LandingBase_is_suspended pose(::hydrone_msgs::msg::LandingBase::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_LandingBase_is_suspended(msg_);
  }

private:
  ::hydrone_msgs::msg::LandingBase msg_;
};

class Init_LandingBase_base_id
{
public:
  explicit Init_LandingBase_base_id(::hydrone_msgs::msg::LandingBase & msg)
  : msg_(msg)
  {}
  Init_LandingBase_pose base_id(::hydrone_msgs::msg::LandingBase::_base_id_type arg)
  {
    msg_.base_id = std::move(arg);
    return Init_LandingBase_pose(msg_);
  }

private:
  ::hydrone_msgs::msg::LandingBase msg_;
};

class Init_LandingBase_header
{
public:
  Init_LandingBase_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LandingBase_base_id header(::hydrone_msgs::msg::LandingBase::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LandingBase_base_id(msg_);
  }

private:
  ::hydrone_msgs::msg::LandingBase msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hydrone_msgs::msg::LandingBase>()
{
  return hydrone_msgs::msg::builder::Init_LandingBase_header();
}

}  // namespace hydrone_msgs

#endif  // HYDRONE_MSGS__MSG__DETAIL__LANDING_BASE__BUILDER_HPP_
