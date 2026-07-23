// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from parking_robot_interfaces:srv/GetTaskStatus.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__SRV__DETAIL__GET_TASK_STATUS__STRUCT_HPP_
#define PARKING_ROBOT_INTERFACES__SRV__DETAIL__GET_TASK_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__parking_robot_interfaces__srv__GetTaskStatus_Request __attribute__((deprecated))
#else
# define DEPRECATED__parking_robot_interfaces__srv__GetTaskStatus_Request __declspec(deprecated)
#endif

namespace parking_robot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetTaskStatus_Request_
{
  using Type = GetTaskStatus_Request_<ContainerAllocator>;

  explicit GetTaskStatus_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->task_id = "";
    }
  }

  explicit GetTaskStatus_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : task_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->task_id = "";
    }
  }

  // field types and members
  using _task_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _task_id_type task_id;

  // setters for named parameter idiom
  Type & set__task_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->task_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    parking_robot_interfaces::srv::GetTaskStatus_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const parking_robot_interfaces::srv::GetTaskStatus_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<parking_robot_interfaces::srv::GetTaskStatus_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<parking_robot_interfaces::srv::GetTaskStatus_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::srv::GetTaskStatus_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::srv::GetTaskStatus_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::srv::GetTaskStatus_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::srv::GetTaskStatus_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<parking_robot_interfaces::srv::GetTaskStatus_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<parking_robot_interfaces::srv::GetTaskStatus_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__parking_robot_interfaces__srv__GetTaskStatus_Request
    std::shared_ptr<parking_robot_interfaces::srv::GetTaskStatus_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__parking_robot_interfaces__srv__GetTaskStatus_Request
    std::shared_ptr<parking_robot_interfaces::srv::GetTaskStatus_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetTaskStatus_Request_ & other) const
  {
    if (this->task_id != other.task_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetTaskStatus_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetTaskStatus_Request_

// alias to use template instance with default allocator
using GetTaskStatus_Request =
  parking_robot_interfaces::srv::GetTaskStatus_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace parking_robot_interfaces


#ifndef _WIN32
# define DEPRECATED__parking_robot_interfaces__srv__GetTaskStatus_Response __attribute__((deprecated))
#else
# define DEPRECATED__parking_robot_interfaces__srv__GetTaskStatus_Response __declspec(deprecated)
#endif

namespace parking_robot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetTaskStatus_Response_
{
  using Type = GetTaskStatus_Response_<ContainerAllocator>;

  explicit GetTaskStatus_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = "";
      this->eta_seconds = 0l;
      this->message = "";
    }
  }

  explicit GetTaskStatus_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : state(_alloc),
    message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = "";
      this->eta_seconds = 0l;
      this->message = "";
    }
  }

  // field types and members
  using _state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _state_type state;
  using _eta_seconds_type =
    int32_t;
  _eta_seconds_type eta_seconds;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__eta_seconds(
    const int32_t & _arg)
  {
    this->eta_seconds = _arg;
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
    parking_robot_interfaces::srv::GetTaskStatus_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const parking_robot_interfaces::srv::GetTaskStatus_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<parking_robot_interfaces::srv::GetTaskStatus_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<parking_robot_interfaces::srv::GetTaskStatus_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::srv::GetTaskStatus_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::srv::GetTaskStatus_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::srv::GetTaskStatus_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::srv::GetTaskStatus_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<parking_robot_interfaces::srv::GetTaskStatus_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<parking_robot_interfaces::srv::GetTaskStatus_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__parking_robot_interfaces__srv__GetTaskStatus_Response
    std::shared_ptr<parking_robot_interfaces::srv::GetTaskStatus_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__parking_robot_interfaces__srv__GetTaskStatus_Response
    std::shared_ptr<parking_robot_interfaces::srv::GetTaskStatus_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetTaskStatus_Response_ & other) const
  {
    if (this->state != other.state) {
      return false;
    }
    if (this->eta_seconds != other.eta_seconds) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetTaskStatus_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetTaskStatus_Response_

// alias to use template instance with default allocator
using GetTaskStatus_Response =
  parking_robot_interfaces::srv::GetTaskStatus_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace parking_robot_interfaces

namespace parking_robot_interfaces
{

namespace srv
{

struct GetTaskStatus
{
  using Request = parking_robot_interfaces::srv::GetTaskStatus_Request;
  using Response = parking_robot_interfaces::srv::GetTaskStatus_Response;
};

}  // namespace srv

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__SRV__DETAIL__GET_TASK_STATUS__STRUCT_HPP_
