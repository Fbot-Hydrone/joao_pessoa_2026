// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hydrone_msgs:msg/MissionState.idl
// generated code does not contain a copyright notice

#ifndef HYDRONE_MSGS__MSG__DETAIL__MISSION_STATE__BUILDER_HPP_
#define HYDRONE_MSGS__MSG__DETAIL__MISSION_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hydrone_msgs/msg/detail/mission_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hydrone_msgs
{

namespace msg
{

namespace builder
{

class Init_MissionState_open_hardware
{
public:
  explicit Init_MissionState_open_hardware(::hydrone_msgs::msg::MissionState & msg)
  : msg_(msg)
  {}
  ::hydrone_msgs::msg::MissionState open_hardware(::hydrone_msgs::msg::MissionState::_open_hardware_type arg)
  {
    msg_.open_hardware = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hydrone_msgs::msg::MissionState msg_;
};

class Init_MissionState_score
{
public:
  explicit Init_MissionState_score(::hydrone_msgs::msg::MissionState & msg)
  : msg_(msg)
  {}
  Init_MissionState_open_hardware score(::hydrone_msgs::msg::MissionState::_score_type arg)
  {
    msg_.score = std::move(arg);
    return Init_MissionState_open_hardware(msg_);
  }

private:
  ::hydrone_msgs::msg::MissionState msg_;
};

class Init_MissionState_state_name
{
public:
  explicit Init_MissionState_state_name(::hydrone_msgs::msg::MissionState & msg)
  : msg_(msg)
  {}
  Init_MissionState_score state_name(::hydrone_msgs::msg::MissionState::_state_name_type arg)
  {
    msg_.state_name = std::move(arg);
    return Init_MissionState_score(msg_);
  }

private:
  ::hydrone_msgs::msg::MissionState msg_;
};

class Init_MissionState_state
{
public:
  explicit Init_MissionState_state(::hydrone_msgs::msg::MissionState & msg)
  : msg_(msg)
  {}
  Init_MissionState_state_name state(::hydrone_msgs::msg::MissionState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_MissionState_state_name(msg_);
  }

private:
  ::hydrone_msgs::msg::MissionState msg_;
};

class Init_MissionState_phase
{
public:
  explicit Init_MissionState_phase(::hydrone_msgs::msg::MissionState & msg)
  : msg_(msg)
  {}
  Init_MissionState_state phase(::hydrone_msgs::msg::MissionState::_phase_type arg)
  {
    msg_.phase = std::move(arg);
    return Init_MissionState_state(msg_);
  }

private:
  ::hydrone_msgs::msg::MissionState msg_;
};

class Init_MissionState_header
{
public:
  Init_MissionState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MissionState_phase header(::hydrone_msgs::msg::MissionState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MissionState_phase(msg_);
  }

private:
  ::hydrone_msgs::msg::MissionState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hydrone_msgs::msg::MissionState>()
{
  return hydrone_msgs::msg::builder::Init_MissionState_header();
}

}  // namespace hydrone_msgs

#endif  // HYDRONE_MSGS__MSG__DETAIL__MISSION_STATE__BUILDER_HPP_
