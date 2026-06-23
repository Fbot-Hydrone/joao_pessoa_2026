// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hydrone_msgs:msg/QRCode.idl
// generated code does not contain a copyright notice

#ifndef HYDRONE_MSGS__MSG__DETAIL__QR_CODE__BUILDER_HPP_
#define HYDRONE_MSGS__MSG__DETAIL__QR_CODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hydrone_msgs/msg/detail/qr_code__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hydrone_msgs
{

namespace msg
{

namespace builder
{

class Init_QRCode_is_new
{
public:
  explicit Init_QRCode_is_new(::hydrone_msgs::msg::QRCode & msg)
  : msg_(msg)
  {}
  ::hydrone_msgs::msg::QRCode is_new(::hydrone_msgs::msg::QRCode::_is_new_type arg)
  {
    msg_.is_new = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hydrone_msgs::msg::QRCode msg_;
};

class Init_QRCode_pose
{
public:
  explicit Init_QRCode_pose(::hydrone_msgs::msg::QRCode & msg)
  : msg_(msg)
  {}
  Init_QRCode_is_new pose(::hydrone_msgs::msg::QRCode::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_QRCode_is_new(msg_);
  }

private:
  ::hydrone_msgs::msg::QRCode msg_;
};

class Init_QRCode_qr_id
{
public:
  explicit Init_QRCode_qr_id(::hydrone_msgs::msg::QRCode & msg)
  : msg_(msg)
  {}
  Init_QRCode_pose qr_id(::hydrone_msgs::msg::QRCode::_qr_id_type arg)
  {
    msg_.qr_id = std::move(arg);
    return Init_QRCode_pose(msg_);
  }

private:
  ::hydrone_msgs::msg::QRCode msg_;
};

class Init_QRCode_header
{
public:
  Init_QRCode_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_QRCode_qr_id header(::hydrone_msgs::msg::QRCode::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_QRCode_qr_id(msg_);
  }

private:
  ::hydrone_msgs::msg::QRCode msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hydrone_msgs::msg::QRCode>()
{
  return hydrone_msgs::msg::builder::Init_QRCode_header();
}

}  // namespace hydrone_msgs

#endif  // HYDRONE_MSGS__MSG__DETAIL__QR_CODE__BUILDER_HPP_
