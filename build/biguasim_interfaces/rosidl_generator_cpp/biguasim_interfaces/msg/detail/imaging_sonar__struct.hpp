// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from biguasim_interfaces:msg/ImagingSonar.idl
// generated code does not contain a copyright notice

#ifndef BIGUASIM_INTERFACES__MSG__DETAIL__IMAGING_SONAR__STRUCT_HPP_
#define BIGUASIM_INTERFACES__MSG__DETAIL__IMAGING_SONAR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'raw_image'
// Member 'intensity'
// Member 'elevation'
#include "sensor_msgs/msg/detail/image__struct.hpp"
// Member 'point_cloud'
#include "sensor_msgs/msg/detail/point_cloud2__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__biguasim_interfaces__msg__ImagingSonar __attribute__((deprecated))
#else
# define DEPRECATED__biguasim_interfaces__msg__ImagingSonar __declspec(deprecated)
#endif

namespace biguasim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ImagingSonar_
{
  using Type = ImagingSonar_<ContainerAllocator>;

  explicit ImagingSonar_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    raw_image(_init),
    intensity(_init),
    elevation(_init),
    point_cloud(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->bins_azimuth = 0l;
      this->bins_range = 0l;
    }
  }

  explicit ImagingSonar_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    raw_image(_alloc, _init),
    intensity(_alloc, _init),
    elevation(_alloc, _init),
    point_cloud(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->bins_azimuth = 0l;
      this->bins_range = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _bins_azimuth_type =
    int32_t;
  _bins_azimuth_type bins_azimuth;
  using _bins_range_type =
    int32_t;
  _bins_range_type bins_range;
  using _raw_image_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _raw_image_type raw_image;
  using _intensity_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _intensity_type intensity;
  using _elevation_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _elevation_type elevation;
  using _point_cloud_type =
    sensor_msgs::msg::PointCloud2_<ContainerAllocator>;
  _point_cloud_type point_cloud;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__bins_azimuth(
    const int32_t & _arg)
  {
    this->bins_azimuth = _arg;
    return *this;
  }
  Type & set__bins_range(
    const int32_t & _arg)
  {
    this->bins_range = _arg;
    return *this;
  }
  Type & set__raw_image(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->raw_image = _arg;
    return *this;
  }
  Type & set__intensity(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->intensity = _arg;
    return *this;
  }
  Type & set__elevation(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->elevation = _arg;
    return *this;
  }
  Type & set__point_cloud(
    const sensor_msgs::msg::PointCloud2_<ContainerAllocator> & _arg)
  {
    this->point_cloud = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    biguasim_interfaces::msg::ImagingSonar_<ContainerAllocator> *;
  using ConstRawPtr =
    const biguasim_interfaces::msg::ImagingSonar_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<biguasim_interfaces::msg::ImagingSonar_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<biguasim_interfaces::msg::ImagingSonar_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      biguasim_interfaces::msg::ImagingSonar_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<biguasim_interfaces::msg::ImagingSonar_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      biguasim_interfaces::msg::ImagingSonar_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<biguasim_interfaces::msg::ImagingSonar_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<biguasim_interfaces::msg::ImagingSonar_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<biguasim_interfaces::msg::ImagingSonar_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__biguasim_interfaces__msg__ImagingSonar
    std::shared_ptr<biguasim_interfaces::msg::ImagingSonar_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__biguasim_interfaces__msg__ImagingSonar
    std::shared_ptr<biguasim_interfaces::msg::ImagingSonar_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ImagingSonar_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->bins_azimuth != other.bins_azimuth) {
      return false;
    }
    if (this->bins_range != other.bins_range) {
      return false;
    }
    if (this->raw_image != other.raw_image) {
      return false;
    }
    if (this->intensity != other.intensity) {
      return false;
    }
    if (this->elevation != other.elevation) {
      return false;
    }
    if (this->point_cloud != other.point_cloud) {
      return false;
    }
    return true;
  }
  bool operator!=(const ImagingSonar_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ImagingSonar_

// alias to use template instance with default allocator
using ImagingSonar =
  biguasim_interfaces::msg::ImagingSonar_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace biguasim_interfaces

#endif  // BIGUASIM_INTERFACES__MSG__DETAIL__IMAGING_SONAR__STRUCT_HPP_
