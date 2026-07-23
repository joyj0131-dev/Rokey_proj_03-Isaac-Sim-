// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from parking_robot_interfaces:msg/FormationAssignment.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__MSG__DETAIL__FORMATION_ASSIGNMENT__STRUCT_HPP_
#define PARKING_ROBOT_INTERFACES__MSG__DETAIL__FORMATION_ASSIGNMENT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__parking_robot_interfaces__msg__FormationAssignment __attribute__((deprecated))
#else
# define DEPRECATED__parking_robot_interfaces__msg__FormationAssignment __declspec(deprecated)
#endif

namespace parking_robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FormationAssignment_
{
  using Type = FormationAssignment_<ContainerAllocator>;

  explicit FormationAssignment_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
      this->task_id = "";
      this->role = "";
      this->partner_robot_id = "";
      this->active = false;
    }
  }

  explicit FormationAssignment_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : robot_id(_alloc),
    task_id(_alloc),
    role(_alloc),
    partner_robot_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
      this->task_id = "";
      this->role = "";
      this->partner_robot_id = "";
      this->active = false;
    }
  }

  // field types and members
  using _robot_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_id_type robot_id;
  using _task_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _task_id_type task_id;
  using _role_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _role_type role;
  using _partner_robot_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _partner_robot_id_type partner_robot_id;
  using _active_type =
    bool;
  _active_type active;

  // setters for named parameter idiom
  Type & set__robot_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__task_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->task_id = _arg;
    return *this;
  }
  Type & set__role(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->role = _arg;
    return *this;
  }
  Type & set__partner_robot_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->partner_robot_id = _arg;
    return *this;
  }
  Type & set__active(
    const bool & _arg)
  {
    this->active = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    parking_robot_interfaces::msg::FormationAssignment_<ContainerAllocator> *;
  using ConstRawPtr =
    const parking_robot_interfaces::msg::FormationAssignment_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<parking_robot_interfaces::msg::FormationAssignment_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<parking_robot_interfaces::msg::FormationAssignment_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::msg::FormationAssignment_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::msg::FormationAssignment_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::msg::FormationAssignment_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::msg::FormationAssignment_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<parking_robot_interfaces::msg::FormationAssignment_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<parking_robot_interfaces::msg::FormationAssignment_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__parking_robot_interfaces__msg__FormationAssignment
    std::shared_ptr<parking_robot_interfaces::msg::FormationAssignment_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__parking_robot_interfaces__msg__FormationAssignment
    std::shared_ptr<parking_robot_interfaces::msg::FormationAssignment_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FormationAssignment_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->task_id != other.task_id) {
      return false;
    }
    if (this->role != other.role) {
      return false;
    }
    if (this->partner_robot_id != other.partner_robot_id) {
      return false;
    }
    if (this->active != other.active) {
      return false;
    }
    return true;
  }
  bool operator!=(const FormationAssignment_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FormationAssignment_

// alias to use template instance with default allocator
using FormationAssignment =
  parking_robot_interfaces::msg::FormationAssignment_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__MSG__DETAIL__FORMATION_ASSIGNMENT__STRUCT_HPP_
