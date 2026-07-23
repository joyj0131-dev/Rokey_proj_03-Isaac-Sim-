// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from parking_robot_interfaces:msg/FormationStop.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__MSG__DETAIL__FORMATION_STOP__STRUCT_HPP_
#define PARKING_ROBOT_INTERFACES__MSG__DETAIL__FORMATION_STOP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__parking_robot_interfaces__msg__FormationStop __attribute__((deprecated))
#else
# define DEPRECATED__parking_robot_interfaces__msg__FormationStop __declspec(deprecated)
#endif

namespace parking_robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FormationStop_
{
  using Type = FormationStop_<ContainerAllocator>;

  explicit FormationStop_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->task_id = "";
      this->source_robot_id = "";
      this->stop = false;
      this->reason = "";
    }
  }

  explicit FormationStop_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : task_id(_alloc),
    source_robot_id(_alloc),
    reason(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->task_id = "";
      this->source_robot_id = "";
      this->stop = false;
      this->reason = "";
    }
  }

  // field types and members
  using _task_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _task_id_type task_id;
  using _source_robot_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _source_robot_id_type source_robot_id;
  using _stop_type =
    bool;
  _stop_type stop;
  using _reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _reason_type reason;

  // setters for named parameter idiom
  Type & set__task_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->task_id = _arg;
    return *this;
  }
  Type & set__source_robot_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->source_robot_id = _arg;
    return *this;
  }
  Type & set__stop(
    const bool & _arg)
  {
    this->stop = _arg;
    return *this;
  }
  Type & set__reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->reason = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    parking_robot_interfaces::msg::FormationStop_<ContainerAllocator> *;
  using ConstRawPtr =
    const parking_robot_interfaces::msg::FormationStop_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<parking_robot_interfaces::msg::FormationStop_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<parking_robot_interfaces::msg::FormationStop_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::msg::FormationStop_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::msg::FormationStop_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::msg::FormationStop_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::msg::FormationStop_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<parking_robot_interfaces::msg::FormationStop_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<parking_robot_interfaces::msg::FormationStop_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__parking_robot_interfaces__msg__FormationStop
    std::shared_ptr<parking_robot_interfaces::msg::FormationStop_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__parking_robot_interfaces__msg__FormationStop
    std::shared_ptr<parking_robot_interfaces::msg::FormationStop_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FormationStop_ & other) const
  {
    if (this->task_id != other.task_id) {
      return false;
    }
    if (this->source_robot_id != other.source_robot_id) {
      return false;
    }
    if (this->stop != other.stop) {
      return false;
    }
    if (this->reason != other.reason) {
      return false;
    }
    return true;
  }
  bool operator!=(const FormationStop_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FormationStop_

// alias to use template instance with default allocator
using FormationStop =
  parking_robot_interfaces::msg::FormationStop_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__MSG__DETAIL__FORMATION_STOP__STRUCT_HPP_
