// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hydrone_msgs:srv/SetPhase.idl
// generated code does not contain a copyright notice

#ifndef HYDRONE_MSGS__SRV__DETAIL__SET_PHASE__BUILDER_HPP_
#define HYDRONE_MSGS__SRV__DETAIL__SET_PHASE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hydrone_msgs/srv/detail/set_phase__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hydrone_msgs
{

namespace srv
{

namespace builder
{

class Init_SetPhase_Request_use_two_drones
{
public:
  explicit Init_SetPhase_Request_use_two_drones(::hydrone_msgs::srv::SetPhase_Request & msg)
  : msg_(msg)
  {}
  ::hydrone_msgs::srv::SetPhase_Request use_two_drones(::hydrone_msgs::srv::SetPhase_Request::_use_two_drones_type arg)
  {
    msg_.use_two_drones = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hydrone_msgs::srv::SetPhase_Request msg_;
};

class Init_SetPhase_Request_open_hardware
{
public:
  explicit Init_SetPhase_Request_open_hardware(::hydrone_msgs::srv::SetPhase_Request & msg)
  : msg_(msg)
  {}
  Init_SetPhase_Request_use_two_drones open_hardware(::hydrone_msgs::srv::SetPhase_Request::_open_hardware_type arg)
  {
    msg_.open_hardware = std::move(arg);
    return Init_SetPhase_Request_use_two_drones(msg_);
  }

private:
  ::hydrone_msgs::srv::SetPhase_Request msg_;
};

class Init_SetPhase_Request_phase
{
public:
  Init_SetPhase_Request_phase()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetPhase_Request_open_hardware phase(::hydrone_msgs::srv::SetPhase_Request::_phase_type arg)
  {
    msg_.phase = std::move(arg);
    return Init_SetPhase_Request_open_hardware(msg_);
  }

private:
  ::hydrone_msgs::srv::SetPhase_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hydrone_msgs::srv::SetPhase_Request>()
{
  return hydrone_msgs::srv::builder::Init_SetPhase_Request_phase();
}

}  // namespace hydrone_msgs


namespace hydrone_msgs
{

namespace srv
{

namespace builder
{

class Init_SetPhase_Response_message
{
public:
  explicit Init_SetPhase_Response_message(::hydrone_msgs::srv::SetPhase_Response & msg)
  : msg_(msg)
  {}
  ::hydrone_msgs::srv::SetPhase_Response message(::hydrone_msgs::srv::SetPhase_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hydrone_msgs::srv::SetPhase_Response msg_;
};

class Init_SetPhase_Response_success
{
public:
  Init_SetPhase_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetPhase_Response_message success(::hydrone_msgs::srv::SetPhase_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetPhase_Response_message(msg_);
  }

private:
  ::hydrone_msgs::srv::SetPhase_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hydrone_msgs::srv::SetPhase_Response>()
{
  return hydrone_msgs::srv::builder::Init_SetPhase_Response_success();
}

}  // namespace hydrone_msgs

#endif  // HYDRONE_MSGS__SRV__DETAIL__SET_PHASE__BUILDER_HPP_
