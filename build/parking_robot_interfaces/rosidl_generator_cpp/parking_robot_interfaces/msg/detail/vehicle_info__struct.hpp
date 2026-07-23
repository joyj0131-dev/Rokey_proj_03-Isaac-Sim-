// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from parking_robot_interfaces:msg/VehicleInfo.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__MSG__DETAIL__VEHICLE_INFO__STRUCT_HPP_
#define PARKING_ROBOT_INTERFACES__MSG__DETAIL__VEHICLE_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__parking_robot_interfaces__msg__VehicleInfo __attribute__((deprecated))
#else
# define DEPRECATED__parking_robot_interfaces__msg__VehicleInfo __declspec(deprecated)
#endif

namespace parking_robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VehicleInfo_
{
  using Type = VehicleInfo_<ContainerAllocator>;

  explicit VehicleInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->length = 0.0;
      this->width = 0.0;
      this->height = 0.0;
    }
  }

  explicit VehicleInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->length = 0.0;
      this->width = 0.0;
      this->height = 0.0;
    }
  }

  // field types and members
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _length_type =
    double;
  _length_type length;
  using _width_type =
    double;
  _width_type width;
  using _height_type =
    double;
  _height_type height;

  // setters for named parameter idiom
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__length(
    const double & _arg)
  {
    this->length = _arg;
    return *this;
  }
  Type & set__width(
    const double & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__height(
    const double & _arg)
  {
    this->height = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    parking_robot_interfaces::msg::VehicleInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const parking_robot_interfaces::msg::VehicleInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<parking_robot_interfaces::msg::VehicleInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<parking_robot_interfaces::msg::VehicleInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::msg::VehicleInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::msg::VehicleInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::msg::VehicleInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::msg::VehicleInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<parking_robot_interfaces::msg::VehicleInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<parking_robot_interfaces::msg::VehicleInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__parking_robot_interfaces__msg__VehicleInfo
    std::shared_ptr<parking_robot_interfaces::msg::VehicleInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__parking_robot_interfaces__msg__VehicleInfo
    std::shared_ptr<parking_robot_interfaces::msg::VehicleInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VehicleInfo_ & other) const
  {
    if (this->pose != other.pose) {
      return false;
    }
    if (this->length != other.length) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    return true;
  }
  bool operator!=(const VehicleInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VehicleInfo_

// alias to use template instance with default allocator
using VehicleInfo =
  parking_robot_interfaces::msg::VehicleInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__MSG__DETAIL__VEHICLE_INFO__STRUCT_HPP_
