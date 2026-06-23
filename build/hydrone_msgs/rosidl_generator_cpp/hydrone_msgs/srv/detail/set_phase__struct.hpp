// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hydrone_msgs:srv/SetPhase.idl
// generated code does not contain a copyright notice

#ifndef HYDRONE_MSGS__SRV__DETAIL__SET_PHASE__STRUCT_HPP_
#define HYDRONE_MSGS__SRV__DETAIL__SET_PHASE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__hydrone_msgs__srv__SetPhase_Request __attribute__((deprecated))
#else
# define DEPRECATED__hydrone_msgs__srv__SetPhase_Request __declspec(deprecated)
#endif

namespace hydrone_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetPhase_Request_
{
  using Type = SetPhase_Request_<ContainerAllocator>;

  explicit SetPhase_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->phase = 0;
      this->open_hardware = false;
      this->use_two_drones = false;
    }
  }

  explicit SetPhase_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->phase = 0;
      this->open_hardware = false;
      this->use_two_drones = false;
    }
  }

  // field types and members
  using _phase_type =
    uint8_t;
  _phase_type phase;
  using _open_hardware_type =
    bool;
  _open_hardware_type open_hardware;
  using _use_two_drones_type =
    bool;
  _use_two_drones_type use_two_drones;

  // setters for named parameter idiom
  Type & set__phase(
    const uint8_t & _arg)
  {
    this->phase = _arg;
    return *this;
  }
  Type & set__open_hardware(
    const bool & _arg)
  {
    this->open_hardware = _arg;
    return *this;
  }
  Type & set__use_two_drones(
    const bool & _arg)
  {
    this->use_two_drones = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hydrone_msgs::srv::SetPhase_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const hydrone_msgs::srv::SetPhase_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hydrone_msgs::srv::SetPhase_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hydrone_msgs::srv::SetPhase_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hydrone_msgs::srv::SetPhase_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hydrone_msgs::srv::SetPhase_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hydrone_msgs::srv::SetPhase_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hydrone_msgs::srv::SetPhase_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hydrone_msgs::srv::SetPhase_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hydrone_msgs::srv::SetPhase_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hydrone_msgs__srv__SetPhase_Request
    std::shared_ptr<hydrone_msgs::srv::SetPhase_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hydrone_msgs__srv__SetPhase_Request
    std::shared_ptr<hydrone_msgs::srv::SetPhase_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetPhase_Request_ & other) const
  {
    if (this->phase != other.phase) {
      return false;
    }
    if (this->open_hardware != other.open_hardware) {
      return false;
    }
    if (this->use_two_drones != other.use_two_drones) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetPhase_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetPhase_Request_

// alias to use template instance with default allocator
using SetPhase_Request =
  hydrone_msgs::srv::SetPhase_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hydrone_msgs


#ifndef _WIN32
# define DEPRECATED__hydrone_msgs__srv__SetPhase_Response __attribute__((deprecated))
#else
# define DEPRECATED__hydrone_msgs__srv__SetPhase_Response __declspec(deprecated)
#endif

namespace hydrone_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetPhase_Response_
{
  using Type = SetPhase_Response_<ContainerAllocator>;

  explicit SetPhase_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit SetPhase_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hydrone_msgs::srv::SetPhase_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const hydrone_msgs::srv::SetPhase_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hydrone_msgs::srv::SetPhase_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hydrone_msgs::srv::SetPhase_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hydrone_msgs::srv::SetPhase_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hydrone_msgs::srv::SetPhase_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hydrone_msgs::srv::SetPhase_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hydrone_msgs::srv::SetPhase_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hydrone_msgs::srv::SetPhase_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hydrone_msgs::srv::SetPhase_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hydrone_msgs__srv__SetPhase_Response
    std::shared_ptr<hydrone_msgs::srv::SetPhase_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hydrone_msgs__srv__SetPhase_Response
    std::shared_ptr<hydrone_msgs::srv::SetPhase_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetPhase_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetPhase_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetPhase_Response_

// alias to use template instance with default allocator
using SetPhase_Response =
  hydrone_msgs::srv::SetPhase_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hydrone_msgs

namespace hydrone_msgs
{

namespace srv
{

struct SetPhase
{
  using Request = hydrone_msgs::srv::SetPhase_Request;
  using Response = hydrone_msgs::srv::SetPhase_Response;
};

}  // namespace srv

}  // namespace hydrone_msgs

#endif  // HYDRONE_MSGS__SRV__DETAIL__SET_PHASE__STRUCT_HPP_
