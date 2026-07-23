// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from parking_robot_interfaces:msg/ObstacleAlert.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__MSG__DETAIL__OBSTACLE_ALERT__STRUCT_HPP_
#define PARKING_ROBOT_INTERFACES__MSG__DETAIL__OBSTACLE_ALERT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'location'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__parking_robot_interfaces__msg__ObstacleAlert __attribute__((deprecated))
#else
# define DEPRECATED__parking_robot_interfaces__msg__ObstacleAlert __declspec(deprecated)
#endif

namespace parking_robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObstacleAlert_
{
  using Type = ObstacleAlert_<ContainerAllocator>;

  explicit ObstacleAlert_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : location(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->obstacle_detected = false;
      this->description = "";
    }
  }

  explicit ObstacleAlert_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : description(_alloc),
    location(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->obstacle_detected = false;
      this->description = "";
    }
  }

  // field types and members
  using _obstacle_detected_type =
    bool;
  _obstacle_detected_type obstacle_detected;
  using _description_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _description_type description;
  using _location_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _location_type location;

  // setters for named parameter idiom
  Type & set__obstacle_detected(
    const bool & _arg)
  {
    this->obstacle_detected = _arg;
    return *this;
  }
  Type & set__description(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->description = _arg;
    return *this;
  }
  Type & set__location(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->location = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    parking_robot_interfaces::msg::ObstacleAlert_<ContainerAllocator> *;
  using ConstRawPtr =
    const parking_robot_interfaces::msg::ObstacleAlert_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<parking_robot_interfaces::msg::ObstacleAlert_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<parking_robot_interfaces::msg::ObstacleAlert_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::msg::ObstacleAlert_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::msg::ObstacleAlert_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::msg::ObstacleAlert_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::msg::ObstacleAlert_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<parking_robot_interfaces::msg::ObstacleAlert_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<parking_robot_interfaces::msg::ObstacleAlert_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__parking_robot_interfaces__msg__ObstacleAlert
    std::shared_ptr<parking_robot_interfaces::msg::ObstacleAlert_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__parking_robot_interfaces__msg__ObstacleAlert
    std::shared_ptr<parking_robot_interfaces::msg::ObstacleAlert_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObstacleAlert_ & other) const
  {
    if (this->obstacle_detected != other.obstacle_detected) {
      return false;
    }
    if (this->description != other.description) {
      return false;
    }
    if (this->location != other.location) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObstacleAlert_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObstacleAlert_

// alias to use template instance with default allocator
using ObstacleAlert =
  parking_robot_interfaces::msg::ObstacleAlert_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__MSG__DETAIL__OBSTACLE_ALERT__STRUCT_HPP_
