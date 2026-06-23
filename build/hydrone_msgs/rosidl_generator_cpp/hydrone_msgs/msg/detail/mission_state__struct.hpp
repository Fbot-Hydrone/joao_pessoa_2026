// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hydrone_msgs:msg/MissionState.idl
// generated code does not contain a copyright notice

#ifndef HYDRONE_MSGS__MSG__DETAIL__MISSION_STATE__STRUCT_HPP_
#define HYDRONE_MSGS__MSG__DETAIL__MISSION_STATE__STRUCT_HPP_

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
# define DEPRECATED__hydrone_msgs__msg__MissionState __attribute__((deprecated))
#else
# define DEPRECATED__hydrone_msgs__msg__MissionState __declspec(deprecated)
#endif

namespace hydrone_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MissionState_
{
  using Type = MissionState_<ContainerAllocator>;

  explicit MissionState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->phase = 0;
      this->state = 0;
      this->state_name = "";
      this->score = 0.0f;
      this->open_hardware = false;
    }
  }

  explicit MissionState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    state_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->phase = 0;
      this->state = 0;
      this->state_name = "";
      this->score = 0.0f;
      this->open_hardware = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _phase_type =
    uint8_t;
  _phase_type phase;
  using _state_type =
    uint8_t;
  _state_type state;
  using _state_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _state_name_type state_name;
  using _score_type =
    float;
  _score_type score;
  using _open_hardware_type =
    bool;
  _open_hardware_type open_hardware;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__phase(
    const uint8_t & _arg)
  {
    this->phase = _arg;
    return *this;
  }
  Type & set__state(
    const uint8_t & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__state_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->state_name = _arg;
    return *this;
  }
  Type & set__score(
    const float & _arg)
  {
    this->score = _arg;
    return *this;
  }
  Type & set__open_hardware(
    const bool & _arg)
  {
    this->open_hardware = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hydrone_msgs::msg::MissionState_<ContainerAllocator> *;
  using ConstRawPtr =
    const hydrone_msgs::msg::MissionState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hydrone_msgs::msg::MissionState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hydrone_msgs::msg::MissionState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hydrone_msgs::msg::MissionState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hydrone_msgs::msg::MissionState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hydrone_msgs::msg::MissionState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hydrone_msgs::msg::MissionState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hydrone_msgs::msg::MissionState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hydrone_msgs::msg::MissionState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hydrone_msgs__msg__MissionState
    std::shared_ptr<hydrone_msgs::msg::MissionState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hydrone_msgs__msg__MissionState
    std::shared_ptr<hydrone_msgs::msg::MissionState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MissionState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->phase != other.phase) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->state_name != other.state_name) {
      return false;
    }
    if (this->score != other.score) {
      return false;
    }
    if (this->open_hardware != other.open_hardware) {
      return false;
    }
    return true;
  }
  bool operator!=(const MissionState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MissionState_

// alias to use template instance with default allocator
using MissionState =
  hydrone_msgs::msg::MissionState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hydrone_msgs

#endif  // HYDRONE_MSGS__MSG__DETAIL__MISSION_STATE__STRUCT_HPP_
