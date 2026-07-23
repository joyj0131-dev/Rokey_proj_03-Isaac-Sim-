// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from parking_robot_interfaces:srv/GetSlotInfo.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__SRV__DETAIL__GET_SLOT_INFO__STRUCT_HPP_
#define PARKING_ROBOT_INTERFACES__SRV__DETAIL__GET_SLOT_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__parking_robot_interfaces__srv__GetSlotInfo_Request __attribute__((deprecated))
#else
# define DEPRECATED__parking_robot_interfaces__srv__GetSlotInfo_Request __declspec(deprecated)
#endif

namespace parking_robot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetSlotInfo_Request_
{
  using Type = GetSlotInfo_Request_<ContainerAllocator>;

  explicit GetSlotInfo_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->slot_id = "";
    }
  }

  explicit GetSlotInfo_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : slot_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->slot_id = "";
    }
  }

  // field types and members
  using _slot_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _slot_id_type slot_id;

  // setters for named parameter idiom
  Type & set__slot_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->slot_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    parking_robot_interfaces::srv::GetSlotInfo_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const parking_robot_interfaces::srv::GetSlotInfo_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<parking_robot_interfaces::srv::GetSlotInfo_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<parking_robot_interfaces::srv::GetSlotInfo_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::srv::GetSlotInfo_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::srv::GetSlotInfo_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::srv::GetSlotInfo_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::srv::GetSlotInfo_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<parking_robot_interfaces::srv::GetSlotInfo_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<parking_robot_interfaces::srv::GetSlotInfo_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__parking_robot_interfaces__srv__GetSlotInfo_Request
    std::shared_ptr<parking_robot_interfaces::srv::GetSlotInfo_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__parking_robot_interfaces__srv__GetSlotInfo_Request
    std::shared_ptr<parking_robot_interfaces::srv::GetSlotInfo_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetSlotInfo_Request_ & other) const
  {
    if (this->slot_id != other.slot_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetSlotInfo_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetSlotInfo_Request_

// alias to use template instance with default allocator
using GetSlotInfo_Request =
  parking_robot_interfaces::srv::GetSlotInfo_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace parking_robot_interfaces


// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__parking_robot_interfaces__srv__GetSlotInfo_Response __attribute__((deprecated))
#else
# define DEPRECATED__parking_robot_interfaces__srv__GetSlotInfo_Response __declspec(deprecated)
#endif

namespace parking_robot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetSlotInfo_Response_
{
  using Type = GetSlotInfo_Response_<ContainerAllocator>;

  explicit GetSlotInfo_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data_ready = false;
      this->exists = false;
      this->occupied = false;
      this->is_accessible = false;
    }
  }

  explicit GetSlotInfo_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data_ready = false;
      this->exists = false;
      this->occupied = false;
      this->is_accessible = false;
    }
  }

  // field types and members
  using _data_ready_type =
    bool;
  _data_ready_type data_ready;
  using _exists_type =
    bool;
  _exists_type exists;
  using _occupied_type =
    bool;
  _occupied_type occupied;
  using _is_accessible_type =
    bool;
  _is_accessible_type is_accessible;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;

  // setters for named parameter idiom
  Type & set__data_ready(
    const bool & _arg)
  {
    this->data_ready = _arg;
    return *this;
  }
  Type & set__exists(
    const bool & _arg)
  {
    this->exists = _arg;
    return *this;
  }
  Type & set__occupied(
    const bool & _arg)
  {
    this->occupied = _arg;
    return *this;
  }
  Type & set__is_accessible(
    const bool & _arg)
  {
    this->is_accessible = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    parking_robot_interfaces::srv::GetSlotInfo_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const parking_robot_interfaces::srv::GetSlotInfo_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<parking_robot_interfaces::srv::GetSlotInfo_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<parking_robot_interfaces::srv::GetSlotInfo_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::srv::GetSlotInfo_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::srv::GetSlotInfo_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::srv::GetSlotInfo_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::srv::GetSlotInfo_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<parking_robot_interfaces::srv::GetSlotInfo_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<parking_robot_interfaces::srv::GetSlotInfo_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__parking_robot_interfaces__srv__GetSlotInfo_Response
    std::shared_ptr<parking_robot_interfaces::srv::GetSlotInfo_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__parking_robot_interfaces__srv__GetSlotInfo_Response
    std::shared_ptr<parking_robot_interfaces::srv::GetSlotInfo_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetSlotInfo_Response_ & other) const
  {
    if (this->data_ready != other.data_ready) {
      return false;
    }
    if (this->exists != other.exists) {
      return false;
    }
    if (this->occupied != other.occupied) {
      return false;
    }
    if (this->is_accessible != other.is_accessible) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetSlotInfo_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetSlotInfo_Response_

// alias to use template instance with default allocator
using GetSlotInfo_Response =
  parking_robot_interfaces::srv::GetSlotInfo_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace parking_robot_interfaces

namespace parking_robot_interfaces
{

namespace srv
{

struct GetSlotInfo
{
  using Request = parking_robot_interfaces::srv::GetSlotInfo_Request;
  using Response = parking_robot_interfaces::srv::GetSlotInfo_Response;
};

}  // namespace srv

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__SRV__DETAIL__GET_SLOT_INFO__STRUCT_HPP_
