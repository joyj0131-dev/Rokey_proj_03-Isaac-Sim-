// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from parking_robot_interfaces:action/AlignVehicle.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__ACTION__DETAIL__ALIGN_VEHICLE__STRUCT_HPP_
#define PARKING_ROBOT_INTERFACES__ACTION__DETAIL__ALIGN_VEHICLE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__parking_robot_interfaces__action__AlignVehicle_Goal __attribute__((deprecated))
#else
# define DEPRECATED__parking_robot_interfaces__action__AlignVehicle_Goal __declspec(deprecated)
#endif

namespace parking_robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct AlignVehicle_Goal_
{
  using Type = AlignVehicle_Goal_<ContainerAllocator>;

  explicit AlignVehicle_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_pose(_init)
  {
    (void)_init;
  }

  explicit AlignVehicle_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_pose(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _target_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _target_pose_type target_pose;

  // setters for named parameter idiom
  Type & set__target_pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->target_pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    parking_robot_interfaces::action::AlignVehicle_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const parking_robot_interfaces::action::AlignVehicle_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::action::AlignVehicle_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::action::AlignVehicle_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::action::AlignVehicle_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::action::AlignVehicle_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<parking_robot_interfaces::action::AlignVehicle_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<parking_robot_interfaces::action::AlignVehicle_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__parking_robot_interfaces__action__AlignVehicle_Goal
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__parking_robot_interfaces__action__AlignVehicle_Goal
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AlignVehicle_Goal_ & other) const
  {
    if (this->target_pose != other.target_pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const AlignVehicle_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AlignVehicle_Goal_

// alias to use template instance with default allocator
using AlignVehicle_Goal =
  parking_robot_interfaces::action::AlignVehicle_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace parking_robot_interfaces


#ifndef _WIN32
# define DEPRECATED__parking_robot_interfaces__action__AlignVehicle_Result __attribute__((deprecated))
#else
# define DEPRECATED__parking_robot_interfaces__action__AlignVehicle_Result __declspec(deprecated)
#endif

namespace parking_robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct AlignVehicle_Result_
{
  using Type = AlignVehicle_Result_<ContainerAllocator>;

  explicit AlignVehicle_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->final_error = 0.0f;
    }
  }

  explicit AlignVehicle_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->final_error = 0.0f;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _final_error_type =
    float;
  _final_error_type final_error;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__final_error(
    const float & _arg)
  {
    this->final_error = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    parking_robot_interfaces::action::AlignVehicle_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const parking_robot_interfaces::action::AlignVehicle_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::action::AlignVehicle_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::action::AlignVehicle_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::action::AlignVehicle_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::action::AlignVehicle_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<parking_robot_interfaces::action::AlignVehicle_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<parking_robot_interfaces::action::AlignVehicle_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__parking_robot_interfaces__action__AlignVehicle_Result
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__parking_robot_interfaces__action__AlignVehicle_Result
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AlignVehicle_Result_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->final_error != other.final_error) {
      return false;
    }
    return true;
  }
  bool operator!=(const AlignVehicle_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AlignVehicle_Result_

// alias to use template instance with default allocator
using AlignVehicle_Result =
  parking_robot_interfaces::action::AlignVehicle_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace parking_robot_interfaces


#ifndef _WIN32
# define DEPRECATED__parking_robot_interfaces__action__AlignVehicle_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__parking_robot_interfaces__action__AlignVehicle_Feedback __declspec(deprecated)
#endif

namespace parking_robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct AlignVehicle_Feedback_
{
  using Type = AlignVehicle_Feedback_<ContainerAllocator>;

  explicit AlignVehicle_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_error = 0.0f;
    }
  }

  explicit AlignVehicle_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_error = 0.0f;
    }
  }

  // field types and members
  using _current_error_type =
    float;
  _current_error_type current_error;

  // setters for named parameter idiom
  Type & set__current_error(
    const float & _arg)
  {
    this->current_error = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    parking_robot_interfaces::action::AlignVehicle_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const parking_robot_interfaces::action::AlignVehicle_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::action::AlignVehicle_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::action::AlignVehicle_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::action::AlignVehicle_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::action::AlignVehicle_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<parking_robot_interfaces::action::AlignVehicle_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<parking_robot_interfaces::action::AlignVehicle_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__parking_robot_interfaces__action__AlignVehicle_Feedback
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__parking_robot_interfaces__action__AlignVehicle_Feedback
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AlignVehicle_Feedback_ & other) const
  {
    if (this->current_error != other.current_error) {
      return false;
    }
    return true;
  }
  bool operator!=(const AlignVehicle_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AlignVehicle_Feedback_

// alias to use template instance with default allocator
using AlignVehicle_Feedback =
  parking_robot_interfaces::action::AlignVehicle_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace parking_robot_interfaces


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "parking_robot_interfaces/action/detail/align_vehicle__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__parking_robot_interfaces__action__AlignVehicle_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__parking_robot_interfaces__action__AlignVehicle_SendGoal_Request __declspec(deprecated)
#endif

namespace parking_robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct AlignVehicle_SendGoal_Request_
{
  using Type = AlignVehicle_SendGoal_Request_<ContainerAllocator>;

  explicit AlignVehicle_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit AlignVehicle_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    parking_robot_interfaces::action::AlignVehicle_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const parking_robot_interfaces::action::AlignVehicle_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    parking_robot_interfaces::action::AlignVehicle_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const parking_robot_interfaces::action::AlignVehicle_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::action::AlignVehicle_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::action::AlignVehicle_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::action::AlignVehicle_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::action::AlignVehicle_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<parking_robot_interfaces::action::AlignVehicle_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<parking_robot_interfaces::action::AlignVehicle_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__parking_robot_interfaces__action__AlignVehicle_SendGoal_Request
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__parking_robot_interfaces__action__AlignVehicle_SendGoal_Request
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AlignVehicle_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const AlignVehicle_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AlignVehicle_SendGoal_Request_

// alias to use template instance with default allocator
using AlignVehicle_SendGoal_Request =
  parking_robot_interfaces::action::AlignVehicle_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace parking_robot_interfaces


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__parking_robot_interfaces__action__AlignVehicle_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__parking_robot_interfaces__action__AlignVehicle_SendGoal_Response __declspec(deprecated)
#endif

namespace parking_robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct AlignVehicle_SendGoal_Response_
{
  using Type = AlignVehicle_SendGoal_Response_<ContainerAllocator>;

  explicit AlignVehicle_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit AlignVehicle_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    parking_robot_interfaces::action::AlignVehicle_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const parking_robot_interfaces::action::AlignVehicle_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::action::AlignVehicle_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::action::AlignVehicle_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::action::AlignVehicle_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::action::AlignVehicle_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<parking_robot_interfaces::action::AlignVehicle_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<parking_robot_interfaces::action::AlignVehicle_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__parking_robot_interfaces__action__AlignVehicle_SendGoal_Response
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__parking_robot_interfaces__action__AlignVehicle_SendGoal_Response
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AlignVehicle_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const AlignVehicle_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AlignVehicle_SendGoal_Response_

// alias to use template instance with default allocator
using AlignVehicle_SendGoal_Response =
  parking_robot_interfaces::action::AlignVehicle_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace parking_robot_interfaces

namespace parking_robot_interfaces
{

namespace action
{

struct AlignVehicle_SendGoal
{
  using Request = parking_robot_interfaces::action::AlignVehicle_SendGoal_Request;
  using Response = parking_robot_interfaces::action::AlignVehicle_SendGoal_Response;
};

}  // namespace action

}  // namespace parking_robot_interfaces


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__parking_robot_interfaces__action__AlignVehicle_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__parking_robot_interfaces__action__AlignVehicle_GetResult_Request __declspec(deprecated)
#endif

namespace parking_robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct AlignVehicle_GetResult_Request_
{
  using Type = AlignVehicle_GetResult_Request_<ContainerAllocator>;

  explicit AlignVehicle_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit AlignVehicle_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    parking_robot_interfaces::action::AlignVehicle_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const parking_robot_interfaces::action::AlignVehicle_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::action::AlignVehicle_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::action::AlignVehicle_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::action::AlignVehicle_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::action::AlignVehicle_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<parking_robot_interfaces::action::AlignVehicle_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<parking_robot_interfaces::action::AlignVehicle_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__parking_robot_interfaces__action__AlignVehicle_GetResult_Request
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__parking_robot_interfaces__action__AlignVehicle_GetResult_Request
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AlignVehicle_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const AlignVehicle_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AlignVehicle_GetResult_Request_

// alias to use template instance with default allocator
using AlignVehicle_GetResult_Request =
  parking_robot_interfaces::action::AlignVehicle_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace parking_robot_interfaces


// Include directives for member types
// Member 'result'
// already included above
// #include "parking_robot_interfaces/action/detail/align_vehicle__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__parking_robot_interfaces__action__AlignVehicle_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__parking_robot_interfaces__action__AlignVehicle_GetResult_Response __declspec(deprecated)
#endif

namespace parking_robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct AlignVehicle_GetResult_Response_
{
  using Type = AlignVehicle_GetResult_Response_<ContainerAllocator>;

  explicit AlignVehicle_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit AlignVehicle_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    parking_robot_interfaces::action::AlignVehicle_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const parking_robot_interfaces::action::AlignVehicle_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    parking_robot_interfaces::action::AlignVehicle_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const parking_robot_interfaces::action::AlignVehicle_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::action::AlignVehicle_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::action::AlignVehicle_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::action::AlignVehicle_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::action::AlignVehicle_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<parking_robot_interfaces::action::AlignVehicle_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<parking_robot_interfaces::action::AlignVehicle_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__parking_robot_interfaces__action__AlignVehicle_GetResult_Response
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__parking_robot_interfaces__action__AlignVehicle_GetResult_Response
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AlignVehicle_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const AlignVehicle_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AlignVehicle_GetResult_Response_

// alias to use template instance with default allocator
using AlignVehicle_GetResult_Response =
  parking_robot_interfaces::action::AlignVehicle_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace parking_robot_interfaces

namespace parking_robot_interfaces
{

namespace action
{

struct AlignVehicle_GetResult
{
  using Request = parking_robot_interfaces::action::AlignVehicle_GetResult_Request;
  using Response = parking_robot_interfaces::action::AlignVehicle_GetResult_Response;
};

}  // namespace action

}  // namespace parking_robot_interfaces


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "parking_robot_interfaces/action/detail/align_vehicle__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__parking_robot_interfaces__action__AlignVehicle_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__parking_robot_interfaces__action__AlignVehicle_FeedbackMessage __declspec(deprecated)
#endif

namespace parking_robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct AlignVehicle_FeedbackMessage_
{
  using Type = AlignVehicle_FeedbackMessage_<ContainerAllocator>;

  explicit AlignVehicle_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit AlignVehicle_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    parking_robot_interfaces::action::AlignVehicle_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const parking_robot_interfaces::action::AlignVehicle_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    parking_robot_interfaces::action::AlignVehicle_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const parking_robot_interfaces::action::AlignVehicle_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::action::AlignVehicle_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::action::AlignVehicle_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      parking_robot_interfaces::action::AlignVehicle_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<parking_robot_interfaces::action::AlignVehicle_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<parking_robot_interfaces::action::AlignVehicle_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<parking_robot_interfaces::action::AlignVehicle_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__parking_robot_interfaces__action__AlignVehicle_FeedbackMessage
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__parking_robot_interfaces__action__AlignVehicle_FeedbackMessage
    std::shared_ptr<parking_robot_interfaces::action::AlignVehicle_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AlignVehicle_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const AlignVehicle_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AlignVehicle_FeedbackMessage_

// alias to use template instance with default allocator
using AlignVehicle_FeedbackMessage =
  parking_robot_interfaces::action::AlignVehicle_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace parking_robot_interfaces

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace parking_robot_interfaces
{

namespace action
{

struct AlignVehicle
{
  /// The goal message defined in the action definition.
  using Goal = parking_robot_interfaces::action::AlignVehicle_Goal;
  /// The result message defined in the action definition.
  using Result = parking_robot_interfaces::action::AlignVehicle_Result;
  /// The feedback message defined in the action definition.
  using Feedback = parking_robot_interfaces::action::AlignVehicle_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = parking_robot_interfaces::action::AlignVehicle_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = parking_robot_interfaces::action::AlignVehicle_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = parking_robot_interfaces::action::AlignVehicle_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct AlignVehicle AlignVehicle;

}  // namespace action

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__ACTION__DETAIL__ALIGN_VEHICLE__STRUCT_HPP_
