// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from parking_robot_interfaces:srv/GetTaskStatus.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__SRV__DETAIL__GET_TASK_STATUS__BUILDER_HPP_
#define PARKING_ROBOT_INTERFACES__SRV__DETAIL__GET_TASK_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "parking_robot_interfaces/srv/detail/get_task_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace parking_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetTaskStatus_Request_task_id
{
public:
  Init_GetTaskStatus_Request_task_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::parking_robot_interfaces::srv::GetTaskStatus_Request task_id(::parking_robot_interfaces::srv::GetTaskStatus_Request::_task_id_type arg)
  {
    msg_.task_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::srv::GetTaskStatus_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::srv::GetTaskStatus_Request>()
{
  return parking_robot_interfaces::srv::builder::Init_GetTaskStatus_Request_task_id();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetTaskStatus_Response_message
{
public:
  explicit Init_GetTaskStatus_Response_message(::parking_robot_interfaces::srv::GetTaskStatus_Response & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::srv::GetTaskStatus_Response message(::parking_robot_interfaces::srv::GetTaskStatus_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::srv::GetTaskStatus_Response msg_;
};

class Init_GetTaskStatus_Response_eta_seconds
{
public:
  explicit Init_GetTaskStatus_Response_eta_seconds(::parking_robot_interfaces::srv::GetTaskStatus_Response & msg)
  : msg_(msg)
  {}
  Init_GetTaskStatus_Response_message eta_seconds(::parking_robot_interfaces::srv::GetTaskStatus_Response::_eta_seconds_type arg)
  {
    msg_.eta_seconds = std::move(arg);
    return Init_GetTaskStatus_Response_message(msg_);
  }

private:
  ::parking_robot_interfaces::srv::GetTaskStatus_Response msg_;
};

class Init_GetTaskStatus_Response_state
{
public:
  Init_GetTaskStatus_Response_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetTaskStatus_Response_eta_seconds state(::parking_robot_interfaces::srv::GetTaskStatus_Response::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_GetTaskStatus_Response_eta_seconds(msg_);
  }

private:
  ::parking_robot_interfaces::srv::GetTaskStatus_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::srv::GetTaskStatus_Response>()
{
  return parking_robot_interfaces::srv::builder::Init_GetTaskStatus_Response_state();
}

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__SRV__DETAIL__GET_TASK_STATUS__BUILDER_HPP_
