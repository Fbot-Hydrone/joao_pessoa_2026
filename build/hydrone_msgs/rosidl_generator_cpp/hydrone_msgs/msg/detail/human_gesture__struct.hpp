// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hydrone_msgs:msg/HumanGesture.idl
// generated code does not contain a copyright notice

#ifndef HYDRONE_MSGS__MSG__DETAIL__HUMAN_GESTURE__STRUCT_HPP_
#define HYDRONE_MSGS__MSG__DETAIL__HUMAN_GESTURE__STRUCT_HPP_

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
// Member 'human_position'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hydrone_msgs__msg__HumanGesture __attribute__((deprecated))
#else
# define DEPRECATED__hydrone_msgs__msg__HumanGesture __declspec(deprecated)
#endif

namespace hydrone_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HumanGesture_
{
  using Type = HumanGesture_<ContainerAllocator>;

  explicit HumanGesture_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    human_position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gesture_name = "";
      this->confidence = 0.0f;
    }
  }

  explicit HumanGesture_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    gesture_name(_alloc),
    human_position(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gesture_name = "";
      this->confidence = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _gesture_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _gesture_name_type gesture_name;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _human_position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _human_position_type human_position;
  using _skeleton_keypoints_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _skeleton_keypoints_type skeleton_keypoints;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__gesture_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->gesture_name = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__human_position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->human_position = _arg;
    return *this;
  }
  Type & set__skeleton_keypoints(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->skeleton_keypoints = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hydrone_msgs::msg::HumanGesture_<ContainerAllocator> *;
  using ConstRawPtr =
    const hydrone_msgs::msg::HumanGesture_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hydrone_msgs::msg::HumanGesture_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hydrone_msgs::msg::HumanGesture_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hydrone_msgs::msg::HumanGesture_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hydrone_msgs::msg::HumanGesture_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hydrone_msgs::msg::HumanGesture_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hydrone_msgs::msg::HumanGesture_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hydrone_msgs::msg::HumanGesture_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hydrone_msgs::msg::HumanGesture_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hydrone_msgs__msg__HumanGesture
    std::shared_ptr<hydrone_msgs::msg::HumanGesture_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hydrone_msgs__msg__HumanGesture
    std::shared_ptr<hydrone_msgs::msg::HumanGesture_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HumanGesture_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->gesture_name != other.gesture_name) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->human_position != other.human_position) {
      return false;
    }
    if (this->skeleton_keypoints != other.skeleton_keypoints) {
      return false;
    }
    return true;
  }
  bool operator!=(const HumanGesture_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HumanGesture_

// alias to use template instance with default allocator
using HumanGesture =
  hydrone_msgs::msg::HumanGesture_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hydrone_msgs

#endif  // HYDRONE_MSGS__MSG__DETAIL__HUMAN_GESTURE__STRUCT_HPP_
