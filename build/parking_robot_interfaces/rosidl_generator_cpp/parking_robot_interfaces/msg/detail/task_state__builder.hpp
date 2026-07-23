// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from parking_robot_interfaces:msg/TaskState.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__MSG__DETAIL__TASK_STATE__BUILDER_HPP_
#define PARKING_ROBOT_INTERFACES__MSG__DETAIL__TASK_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "parking_robot_interfaces/msg/detail/task_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace parking_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_TaskState_current_step
{
public:
  explicit Init_TaskState_current_step(::parking_robot_interfaces::msg::TaskState & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::msg::TaskState current_step(::parking_robot_interfaces::msg::TaskState::_current_step_type arg)
  {
    msg_.current_step = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::msg::TaskState msg_;
};

class Init_TaskState_state
{
public:
  explicit Init_TaskState_state(::parking_robot_interfaces::msg::TaskState & msg)
  : msg_(msg)
  {}
  Init_TaskState_current_step state(::parking_robot_interfaces::msg::TaskState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_TaskState_current_step(msg_);
  }

private:
  ::parking_robot_interfaces::msg::TaskState msg_;
};

class Init_TaskState_task_id
{
public:
  explicit Init_TaskState_task_id(::parking_robot_interfaces::msg::TaskState & msg)
  : msg_(msg)
  {}
  Init_TaskState_state task_id(::parking_robot_interfaces::msg::TaskState::_task_id_type arg)
  {
    msg_.task_id = std::move(arg);
    return Init_TaskState_state(msg_);
  }

private:
  ::parking_robot_interfaces::msg::TaskState msg_;
};

class Init_TaskState_robot_id
{
public:
  Init_TaskState_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TaskState_task_id robot_id(::parking_robot_interfaces::msg::TaskState::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_TaskState_task_id(msg_);
  }

private:
  ::parking_robot_interfaces::msg::TaskState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::msg::TaskState>()
{
  return parking_robot_interfaces::msg::builder::Init_TaskState_robot_id();
}

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__MSG__DETAIL__TASK_STATE__BUILDER_HPP_
