// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hydrone_msgs:msg/QRCode.idl
// generated code does not contain a copyright notice

#ifndef HYDRONE_MSGS__MSG__DETAIL__QR_CODE__STRUCT_HPP_
#define HYDRONE_MSGS__MSG__DETAIL__QR_CODE__STRUCT_HPP_

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
# define DEPRECATED__hydrone_msgs__msg__QRCode __attribute__((deprecated))
#else
# define DEPRECATED__hydrone_msgs__msg__QRCode __declspec(deprecated)
#endif

namespace hydrone_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct QRCode_
{
  using Type = QRCode_<ContainerAllocator>;

  explicit QRCode_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->qr_id = "";
      this->is_new = false;
    }
  }

  explicit QRCode_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    qr_id(_alloc),
    pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->qr_id = "";
      this->is_new = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _qr_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _qr_id_type qr_id;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _is_new_type =
    bool;
  _is_new_type is_new;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__qr_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->qr_id = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__is_new(
    const bool & _arg)
  {
    this->is_new = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hydrone_msgs::msg::QRCode_<ContainerAllocator> *;
  using ConstRawPtr =
    const hydrone_msgs::msg::QRCode_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hydrone_msgs::msg::QRCode_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hydrone_msgs::msg::QRCode_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hydrone_msgs::msg::QRCode_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hydrone_msgs::msg::QRCode_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hydrone_msgs::msg::QRCode_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hydrone_msgs::msg::QRCode_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hydrone_msgs::msg::QRCode_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hydrone_msgs::msg::QRCode_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hydrone_msgs__msg__QRCode
    std::shared_ptr<hydrone_msgs::msg::QRCode_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hydrone_msgs__msg__QRCode
    std::shared_ptr<hydrone_msgs::msg::QRCode_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const QRCode_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->qr_id != other.qr_id) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->is_new != other.is_new) {
      return false;
    }
    return true;
  }
  bool operator!=(const QRCode_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct QRCode_

// alias to use template instance with default allocator
using QRCode =
  hydrone_msgs::msg::QRCode_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hydrone_msgs

#endif  // HYDRONE_MSGS__MSG__DETAIL__QR_CODE__STRUCT_HPP_
