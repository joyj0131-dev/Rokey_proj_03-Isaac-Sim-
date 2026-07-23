// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from parking_robot_interfaces:srv/FindEmptySlot.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__SRV__DETAIL__FIND_EMPTY_SLOT__STRUCT_HPP_
#define PARKING_ROBOT_INTERFACES__SRV__DETAIL__FIND_EMPTY_SLOT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__parking_robot_interfaces__srv__FindEmptySlot_Request __attribute__((deprecated))
#else
# define DEPRECATED__parking_robot_interfaces__srv__FindEmptySlot_Request __declspec(deprecated)
#endif

namespace parking_robot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FindEmptySlot_Request_
{
  using Type = FindEmptySlot_Request_<ContainerAllocator>;

  explicit FindEmptySlot_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vehicle_length = 0.0;
      this->vehicle_width = 0.0;
    }
  }

  explicit FindEmptySlot_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vehicle_length = 0.0;
      this->vehicle_width = 0.0;
    }
  }

  // field types and members
  using _vehicle_length_type =
    double;
  _vehicle_length_type vehicle_length;
  using _vehicle_width_type =
    double;
  _vehicle_width_type vehicle_width;

  // setters for named parameter idiom
  Type & set__vehicle_length(
    const double & _arg)
  {
    this->vehicle_length = _arg;
    return *this;
  }
  Type & set__vehicle_width(
    const double & _arg)
  {
    this->vehicle_width = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    parking_robot_interfaces::srv::FindEmptySlot_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const parking_robot_interfaces::srv::FindEmptySlot_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<parking_robot_interfaces::srv::FindEmptySlot_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<parking_robot_interfaces::srv::FindEmptySlot_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::srv::FindEmptySlot_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::srv::FindEmptySlot_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::srv::FindEmptySlot_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::srv::FindEmptySlot_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<parking_robot_interfaces::srv::FindEmptySlot_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<parking_robot_interfaces::srv::FindEmptySlot_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__parking_robot_interfaces__srv__FindEmptySlot_Request
    std::shared_ptr<parking_robot_interfaces::srv::FindEmptySlot_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__parking_robot_interfaces__srv__FindEmptySlot_Request
    std::shared_ptr<parking_robot_interfaces::srv::FindEmptySlot_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FindEmptySlot_Request_ & other) const
  {
    if (this->vehicle_length != other.vehicle_length) {
      return false;
    }
    if (this->vehicle_width != other.vehicle_width) {
      return false;
    }
    return true;
  }
  bool operator!=(const FindEmptySlot_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FindEmptySlot_Request_

// alias to use template instance with default allocator
using FindEmptySlot_Request =
  parking_robot_interfaces::srv::FindEmptySlot_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace parking_robot_interfaces


// Include directives for member types
// Member 'slot_pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__parking_robot_interfaces__srv__FindEmptySlot_Response __attribute__((deprecated))
#else
# define DEPRECATED__parking_robot_interfaces__srv__FindEmptySlot_Response __declspec(deprecated)
#endif

namespace parking_robot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FindEmptySlot_Response_
{
  using Type = FindEmptySlot_Response_<ContainerAllocator>;

  explicit FindEmptySlot_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : slot_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->slot_id = "";
    }
  }

  explicit FindEmptySlot_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : slot_id(_alloc),
    slot_pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->slot_id = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _slot_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _slot_id_type slot_id;
  using _slot_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _slot_pose_type slot_pose;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__slot_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->slot_id = _arg;
    return *this;
  }
  Type & set__slot_pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->slot_pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    parking_robot_interfaces::srv::FindEmptySlot_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const parking_robot_interfaces::srv::FindEmptySlot_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<parking_robot_interfaces::srv::FindEmptySlot_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<parking_robot_interfaces::srv::FindEmptySlot_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::srv::FindEmptySlot_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::srv::FindEmptySlot_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::srv::FindEmptySlot_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::srv::FindEmptySlot_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<parking_robot_interfaces::srv::FindEmptySlot_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<parking_robot_interfaces::srv::FindEmptySlot_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__parking_robot_interfaces__srv__FindEmptySlot_Response
    std::shared_ptr<parking_robot_interfaces::srv::FindEmptySlot_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__parking_robot_interfaces__srv__FindEmptySlot_Response
    std::shared_ptr<parking_robot_interfaces::srv::FindEmptySlot_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FindEmptySlot_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->slot_id != other.slot_id) {
      return false;
    }
    if (this->slot_pose != other.slot_pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const FindEmptySlot_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FindEmptySlot_Response_

// alias to use template instance with default allocator
using FindEmptySlot_Response =
  parking_robot_interfaces::srv::FindEmptySlot_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace parking_robot_interfaces

namespace parking_robot_interfaces
{

namespace srv
{

struct FindEmptySlot
{
  using Request = parking_robot_interfaces::srv::FindEmptySlot_Request;
  using Response = parking_robot_interfaces::srv::FindEmptySlot_Response;
};

}  // namespace srv

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__SRV__DETAIL__FIND_EMPTY_SLOT__STRUCT_HPP_
