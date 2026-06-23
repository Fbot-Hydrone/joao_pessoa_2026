// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hydrone_msgs:msg/LandingBase.idl
// generated code does not contain a copyright notice

#ifndef HYDRONE_MSGS__MSG__DETAIL__LANDING_BASE__STRUCT_HPP_
#define HYDRONE_MSGS__MSG__DETAIL__LANDING_BASE__STRUCT_HPP_

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
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hydrone_msgs__msg__LandingBase __attribute__((deprecated))
#else
# define DEPRECATED__hydrone_msgs__msg__LandingBase __declspec(deprecated)
#endif

namespace hydrone_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LandingBase_
{
  using Type = LandingBase_<ContainerAllocator>;

  explicit LandingBase_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->base_id = 0;
      this->is_suspended = false;
      this->is_visited = false;
      this->confidence = 0.0f;
      this->height = 0.0f;
    }
  }

  explicit LandingBase_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->base_id = 0;
      this->is_suspended = false;
      this->is_visited = false;
      this->confidence = 0.0f;
      this->height = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _base_id_type =
    uint8_t;
  _base_id_type base_id;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _is_suspended_type =
    bool;
  _is_suspended_type is_suspended;
  using _is_visited_type =
    bool;
  _is_visited_type is_visited;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _height_type =
    float;
  _height_type height;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__base_id(
    const uint8_t & _arg)
  {
    this->base_id = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__is_suspended(
    const bool & _arg)
  {
    this->is_suspended = _arg;
    return *this;
  }
  Type & set__is_visited(
    const bool & _arg)
  {
    this->is_visited = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__height(
    const float & _arg)
  {
    this->height = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hydrone_msgs::msg::LandingBase_<ContainerAllocator> *;
  using ConstRawPtr =
    const hydrone_msgs::msg::LandingBase_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hydrone_msgs::msg::LandingBase_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hydrone_msgs::msg::LandingBase_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hydrone_msgs::msg::LandingBase_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hydrone_msgs::msg::LandingBase_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hydrone_msgs::msg::LandingBase_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hydrone_msgs::msg::LandingBase_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hydrone_msgs::msg::LandingBase_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hydrone_msgs::msg::LandingBase_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hydrone_msgs__msg__LandingBase
    std::shared_ptr<hydrone_msgs::msg::LandingBase_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hydrone_msgs__msg__LandingBase
    std::shared_ptr<hydrone_msgs::msg::LandingBase_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LandingBase_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->base_id != other.base_id) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->is_suspended != other.is_suspended) {
      return false;
    }
    if (this->is_visited != other.is_visited) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    return true;
  }
  bool operator!=(const LandingBase_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LandingBase_

// alias to use template instance with default allocator
using LandingBase =
  hydrone_msgs::msg::LandingBase_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hydrone_msgs

#endif  // HYDRONE_MSGS__MSG__DETAIL__LANDING_BASE__STRUCT_HPP_
