// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from biguasim_interfaces:msg/ImagingSonar.idl
// generated code does not contain a copyright notice

#ifndef BIGUASIM_INTERFACES__MSG__DETAIL__IMAGING_SONAR__BUILDER_HPP_
#define BIGUASIM_INTERFACES__MSG__DETAIL__IMAGING_SONAR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "biguasim_interfaces/msg/detail/imaging_sonar__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace biguasim_interfaces
{

namespace msg
{

namespace builder
{

class Init_ImagingSonar_point_cloud
{
public:
  explicit Init_ImagingSonar_point_cloud(::biguasim_interfaces::msg::ImagingSonar & msg)
  : msg_(msg)
  {}
  ::biguasim_interfaces::msg::ImagingSonar point_cloud(::biguasim_interfaces::msg::ImagingSonar::_point_cloud_type arg)
  {
    msg_.point_cloud = std::move(arg);
    return std::move(msg_);
  }

private:
  ::biguasim_interfaces::msg::ImagingSonar msg_;
};

class Init_ImagingSonar_elevation
{
public:
  explicit Init_ImagingSonar_elevation(::biguasim_interfaces::msg::ImagingSonar & msg)
  : msg_(msg)
  {}
  Init_ImagingSonar_point_cloud elevation(::biguasim_interfaces::msg::ImagingSonar::_elevation_type arg)
  {
    msg_.elevation = std::move(arg);
    return Init_ImagingSonar_point_cloud(msg_);
  }

private:
  ::biguasim_interfaces::msg::ImagingSonar msg_;
};

class Init_ImagingSonar_intensity
{
public:
  explicit Init_ImagingSonar_intensity(::biguasim_interfaces::msg::ImagingSonar & msg)
  : msg_(msg)
  {}
  Init_ImagingSonar_elevation intensity(::biguasim_interfaces::msg::ImagingSonar::_intensity_type arg)
  {
    msg_.intensity = std::move(arg);
    return Init_ImagingSonar_elevation(msg_);
  }

private:
  ::biguasim_interfaces::msg::ImagingSonar msg_;
};

class Init_ImagingSonar_raw_image
{
public:
  explicit Init_ImagingSonar_raw_image(::biguasim_interfaces::msg::ImagingSonar & msg)
  : msg_(msg)
  {}
  Init_ImagingSonar_intensity raw_image(::biguasim_interfaces::msg::ImagingSonar::_raw_image_type arg)
  {
    msg_.raw_image = std::move(arg);
    return Init_ImagingSonar_intensity(msg_);
  }

private:
  ::biguasim_interfaces::msg::ImagingSonar msg_;
};

class Init_ImagingSonar_bins_range
{
public:
  explicit Init_ImagingSonar_bins_range(::biguasim_interfaces::msg::ImagingSonar & msg)
  : msg_(msg)
  {}
  Init_ImagingSonar_raw_image bins_range(::biguasim_interfaces::msg::ImagingSonar::_bins_range_type arg)
  {
    msg_.bins_range = std::move(arg);
    return Init_ImagingSonar_raw_image(msg_);
  }

private:
  ::biguasim_interfaces::msg::ImagingSonar msg_;
};

class Init_ImagingSonar_bins_azimuth
{
public:
  explicit Init_ImagingSonar_bins_azimuth(::biguasim_interfaces::msg::ImagingSonar & msg)
  : msg_(msg)
  {}
  Init_ImagingSonar_bins_range bins_azimuth(::biguasim_interfaces::msg::ImagingSonar::_bins_azimuth_type arg)
  {
    msg_.bins_azimuth = std::move(arg);
    return Init_ImagingSonar_bins_range(msg_);
  }

private:
  ::biguasim_interfaces::msg::ImagingSonar msg_;
};

class Init_ImagingSonar_header
{
public:
  Init_ImagingSonar_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ImagingSonar_bins_azimuth header(::biguasim_interfaces::msg::ImagingSonar::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ImagingSonar_bins_azimuth(msg_);
  }

private:
  ::biguasim_interfaces::msg::ImagingSonar msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::biguasim_interfaces::msg::ImagingSonar>()
{
  return biguasim_interfaces::msg::builder::Init_ImagingSonar_header();
}

}  // namespace biguasim_interfaces

#endif  // BIGUASIM_INTERFACES__MSG__DETAIL__IMAGING_SONAR__BUILDER_HPP_
