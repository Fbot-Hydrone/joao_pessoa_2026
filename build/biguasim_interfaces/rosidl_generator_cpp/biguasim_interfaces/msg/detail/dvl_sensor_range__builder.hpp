// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from biguasim_interfaces:msg/DVLSensorRange.idl
// generated code does not contain a copyright notice

#ifndef BIGUASIM_INTERFACES__MSG__DETAIL__DVL_SENSOR_RANGE__BUILDER_HPP_
#define BIGUASIM_INTERFACES__MSG__DETAIL__DVL_SENSOR_RANGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "biguasim_interfaces/msg/detail/dvl_sensor_range__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace biguasim_interfaces
{

namespace msg
{

namespace builder
{

class Init_DVLSensorRange_range
{
public:
  explicit Init_DVLSensorRange_range(::biguasim_interfaces::msg::DVLSensorRange & msg)
  : msg_(msg)
  {}
  ::biguasim_interfaces::msg::DVLSensorRange range(::biguasim_interfaces::msg::DVLSensorRange::_range_type arg)
  {
    msg_.range = std::move(arg);
    return std::move(msg_);
  }

private:
  ::biguasim_interfaces::msg::DVLSensorRange msg_;
};

class Init_DVLSensorRange_header
{
public:
  Init_DVLSensorRange_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DVLSensorRange_range header(::biguasim_interfaces::msg::DVLSensorRange::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DVLSensorRange_range(msg_);
  }

private:
  ::biguasim_interfaces::msg::DVLSensorRange msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::biguasim_interfaces::msg::DVLSensorRange>()
{
  return biguasim_interfaces::msg::builder::Init_DVLSensorRange_header();
}

}  // namespace biguasim_interfaces

#endif  // BIGUASIM_INTERFACES__MSG__DETAIL__DVL_SENSOR_RANGE__BUILDER_HPP_
