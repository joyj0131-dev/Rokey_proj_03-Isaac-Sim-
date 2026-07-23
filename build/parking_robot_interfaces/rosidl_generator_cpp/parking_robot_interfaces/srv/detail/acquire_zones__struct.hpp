// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from parking_robot_interfaces:srv/AcquireZones.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__SRV__DETAIL__ACQUIRE_ZONES__STRUCT_HPP_
#define PARKING_ROBOT_INTERFACES__SRV__DETAIL__ACQUIRE_ZONES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__parking_robot_interfaces__srv__AcquireZones_Request __attribute__((deprecated))
#else
# define DEPRECATED__parking_robot_interfaces__srv__AcquireZones_Request __declspec(deprecated)
#endif

namespace parking_robot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct AcquireZones_Request_
{
  using Type = AcquireZones_Request_<ContainerAllocator>;

  explicit AcquireZones_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
      this->task_id = "";
    }
  }

  explicit AcquireZones_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : robot_id(_alloc),
    task_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
      this->task_id = "";
    }
  }

  // field types and members
  using _robot_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_id_type robot_id;
  using _task_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _task_id_type task_id;
  using _zone_ids_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _zone_ids_type zone_ids;

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
  Type & set__zone_ids(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->zone_ids = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    parking_robot_interfaces::srv::AcquireZones_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const parking_robot_interfaces::srv::AcquireZones_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<parking_robot_interfaces::srv::AcquireZones_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<parking_robot_interfaces::srv::AcquireZones_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::srv::AcquireZones_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::srv::AcquireZones_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::srv::AcquireZones_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::srv::AcquireZones_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<parking_robot_interfaces::srv::AcquireZones_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<parking_robot_interfaces::srv::AcquireZones_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__parking_robot_interfaces__srv__AcquireZones_Request
    std::shared_ptr<parking_robot_interfaces::srv::AcquireZones_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__parking_robot_interfaces__srv__AcquireZones_Request
    std::shared_ptr<parking_robot_interfaces::srv::AcquireZones_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AcquireZones_Request_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->task_id != other.task_id) {
      return false;
    }
    if (this->zone_ids != other.zone_ids) {
      return false;
    }
    return true;
  }
  bool operator!=(const AcquireZones_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AcquireZones_Request_

// alias to use template instance with default allocator
using AcquireZones_Request =
  parking_robot_interfaces::srv::AcquireZones_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace parking_robot_interfaces


#ifndef _WIN32
# define DEPRECATED__parking_robot_interfaces__srv__AcquireZones_Response __attribute__((deprecated))
#else
# define DEPRECATED__parking_robot_interfaces__srv__AcquireZones_Response __declspec(deprecated)
#endif

namespace parking_robot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct AcquireZones_Response_
{
  using Type = AcquireZones_Response_<ContainerAllocator>;

  explicit AcquireZones_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->granted = false;
      this->retry_after_sec = 0.0f;
    }
  }

  explicit AcquireZones_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->granted = false;
      this->retry_after_sec = 0.0f;
    }
  }

  // field types and members
  using _granted_type =
    bool;
  _granted_type granted;
  using _held_zones_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _held_zones_type held_zones;
  using _retry_after_sec_type =
    float;
  _retry_after_sec_type retry_after_sec;

  // setters for named parameter idiom
  Type & set__granted(
    const bool & _arg)
  {
    this->granted = _arg;
    return *this;
  }
  Type & set__held_zones(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->held_zones = _arg;
    return *this;
  }
  Type & set__retry_after_sec(
    const float & _arg)
  {
    this->retry_after_sec = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    parking_robot_interfaces::srv::AcquireZones_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const parking_robot_interfaces::srv::AcquireZones_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<parking_robot_interfaces::srv::AcquireZones_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<parking_robot_interfaces::srv::AcquireZones_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::srv::AcquireZones_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::srv::AcquireZones_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::srv::AcquireZones_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::srv::AcquireZones_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<parking_robot_interfaces::srv::AcquireZones_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<parking_robot_interfaces::srv::AcquireZones_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__parking_robot_interfaces__srv__AcquireZones_Response
    std::shared_ptr<parking_robot_interfaces::srv::AcquireZones_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__parking_robot_interfaces__srv__AcquireZones_Response
    std::shared_ptr<parking_robot_interfaces::srv::AcquireZones_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AcquireZones_Response_ & other) const
  {
    if (this->granted != other.granted) {
      return false;
    }
    if (this->held_zones != other.held_zones) {
      return false;
    }
    if (this->retry_after_sec != other.retry_after_sec) {
      return false;
    }
    return true;
  }
  bool operator!=(const AcquireZones_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AcquireZones_Response_

// alias to use template instance with default allocator
using AcquireZones_Response =
  parking_robot_interfaces::srv::AcquireZones_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace parking_robot_interfaces

namespace parking_robot_interfaces
{

namespace srv
{

struct AcquireZones
{
  using Request = parking_robot_interfaces::srv::AcquireZones_Request;
  using Response = parking_robot_interfaces::srv::AcquireZones_Response;
};

}  // namespace srv

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__SRV__DETAIL__ACQUIRE_ZONES__STRUCT_HPP_
