// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from biguasim_interfaces:msg/DVLSensorRange.idl
// generated code does not contain a copyright notice

#ifndef BIGUASIM_INTERFACES__MSG__DETAIL__DVL_SENSOR_RANGE__STRUCT_HPP_
#define BIGUASIM_INTERFACES__MSG__DETAIL__DVL_SENSOR_RANGE__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__biguasim_interfaces__msg__DVLSensorRange __attribute__((deprecated))
#else
# define DEPRECATED__biguasim_interfaces__msg__DVLSensorRange __declspec(deprecated)
#endif

namespace biguasim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DVLSensorRange_
{
  using Type = DVLSensorRange_<ContainerAllocator>;

  explicit DVLSensorRange_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 4>::iterator, float>(this->range.begin(), this->range.end(), 0.0f);
    }
  }

  explicit DVLSensorRange_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    range(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 4>::iterator, float>(this->range.begin(), this->range.end(), 0.0f);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _range_type =
    std::array<float, 4>;
  _range_type range;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__range(
    const std::array<float, 4> & _arg)
  {
    this->range = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    biguasim_interfaces::msg::DVLSensorRange_<ContainerAllocator> *;
  using ConstRawPtr =
    const biguasim_interfaces::msg::DVLSensorRange_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<biguasim_interfaces::msg::DVLSensorRange_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<biguasim_interfaces::msg::DVLSensorRange_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      biguasim_interfaces::msg::DVLSensorRange_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<biguasim_interfaces::msg::DVLSensorRange_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      biguasim_interfaces::msg::DVLSensorRange_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<biguasim_interfaces::msg::DVLSensorRange_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<biguasim_interfaces::msg::DVLSensorRange_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<biguasim_interfaces::msg::DVLSensorRange_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__biguasim_interfaces__msg__DVLSensorRange
    std::shared_ptr<biguasim_interfaces::msg::DVLSensorRange_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__biguasim_interfaces__msg__DVLSensorRange
    std::shared_ptr<biguasim_interfaces::msg::DVLSensorRange_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DVLSensorRange_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->range != other.range) {
      return false;
    }
    return true;
  }
  bool operator!=(const DVLSensorRange_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DVLSensorRange_

// alias to use template instance with default allocator
using DVLSensorRange =
  biguasim_interfaces::msg::DVLSensorRange_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace biguasim_interfaces

#endif  // BIGUASIM_INTERFACES__MSG__DETAIL__DVL_SENSOR_RANGE__STRUCT_HPP_
