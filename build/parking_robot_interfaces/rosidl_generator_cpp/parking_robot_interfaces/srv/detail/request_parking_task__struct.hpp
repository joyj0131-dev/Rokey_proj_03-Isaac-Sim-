// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from parking_robot_interfaces:srv/RequestParkingTask.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__SRV__DETAIL__REQUEST_PARKING_TASK__STRUCT_HPP_
#define PARKING_ROBOT_INTERFACES__SRV__DETAIL__REQUEST_PARKING_TASK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__parking_robot_interfaces__srv__RequestParkingTask_Request __attribute__((deprecated))
#else
# define DEPRECATED__parking_robot_interfaces__srv__RequestParkingTask_Request __declspec(deprecated)
#endif

namespace parking_robot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RequestParkingTask_Request_
{
  using Type = RequestParkingTask_Request_<ContainerAllocator>;

  explicit RequestParkingTask_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request_type = "";
      this->vehicle_id = "";
    }
  }

  explicit RequestParkingTask_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : request_type(_alloc),
    vehicle_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request_type = "";
      this->vehicle_id = "";
    }
  }

  // field types and members
  using _request_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _request_type_type request_type;
  using _vehicle_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _vehicle_id_type vehicle_id;

  // setters for named parameter idiom
  Type & set__request_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->request_type = _arg;
    return *this;
  }
  Type & set__vehicle_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->vehicle_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    parking_robot_interfaces::srv::RequestParkingTask_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const parking_robot_interfaces::srv::RequestParkingTask_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<parking_robot_interfaces::srv::RequestParkingTask_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<parking_robot_interfaces::srv::RequestParkingTask_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::srv::RequestParkingTask_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::srv::RequestParkingTask_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::srv::RequestParkingTask_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::srv::RequestParkingTask_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<parking_robot_interfaces::srv::RequestParkingTask_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<parking_robot_interfaces::srv::RequestParkingTask_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__parking_robot_interfaces__srv__RequestParkingTask_Request
    std::shared_ptr<parking_robot_interfaces::srv::RequestParkingTask_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__parking_robot_interfaces__srv__RequestParkingTask_Request
    std::shared_ptr<parking_robot_interfaces::srv::RequestParkingTask_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RequestParkingTask_Request_ & other) const
  {
    if (this->request_type != other.request_type) {
      return false;
    }
    if (this->vehicle_id != other.vehicle_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const RequestParkingTask_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RequestParkingTask_Request_

// alias to use template instance with default allocator
using RequestParkingTask_Request =
  parking_robot_interfaces::srv::RequestParkingTask_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace parking_robot_interfaces


#ifndef _WIN32
# define DEPRECATED__parking_robot_interfaces__srv__RequestParkingTask_Response __attribute__((deprecated))
#else
# define DEPRECATED__parking_robot_interfaces__srv__RequestParkingTask_Response __declspec(deprecated)
#endif

namespace parking_robot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RequestParkingTask_Response_
{
  using Type = RequestParkingTask_Response_<ContainerAllocator>;

  explicit RequestParkingTask_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
      this->task_id = "";
      this->message = "";
    }
  }

  explicit RequestParkingTask_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : task_id(_alloc),
    message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
      this->task_id = "";
      this->message = "";
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _task_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _task_id_type task_id;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__task_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->task_id = _arg;
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
    parking_robot_interfaces::srv::RequestParkingTask_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const parking_robot_interfaces::srv::RequestParkingTask_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<parking_robot_interfaces::srv::RequestParkingTask_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<parking_robot_interfaces::srv::RequestParkingTask_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::srv::RequestParkingTask_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::srv::RequestParkingTask_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::srv::RequestParkingTask_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::srv::RequestParkingTask_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<parking_robot_interfaces::srv::RequestParkingTask_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<parking_robot_interfaces::srv::RequestParkingTask_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__parking_robot_interfaces__srv__RequestParkingTask_Response
    std::shared_ptr<parking_robot_interfaces::srv::RequestParkingTask_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__parking_robot_interfaces__srv__RequestParkingTask_Response
    std::shared_ptr<parking_robot_interfaces::srv::RequestParkingTask_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RequestParkingTask_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->task_id != other.task_id) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const RequestParkingTask_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RequestParkingTask_Response_

// alias to use template instance with default allocator
using RequestParkingTask_Response =
  parking_robot_interfaces::srv::RequestParkingTask_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace parking_robot_interfaces

namespace parking_robot_interfaces
{

namespace srv
{

struct RequestParkingTask
{
  using Request = parking_robot_interfaces::srv::RequestParkingTask_Request;
  using Response = parking_robot_interfaces::srv::RequestParkingTask_Response;
};

}  // namespace srv

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__SRV__DETAIL__REQUEST_PARKING_TASK__STRUCT_HPP_
