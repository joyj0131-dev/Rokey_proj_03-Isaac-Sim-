// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from parking_robot_interfaces:srv/RequestParkingTask.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__SRV__DETAIL__REQUEST_PARKING_TASK__BUILDER_HPP_
#define PARKING_ROBOT_INTERFACES__SRV__DETAIL__REQUEST_PARKING_TASK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "parking_robot_interfaces/srv/detail/request_parking_task__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace parking_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_RequestParkingTask_Request_vehicle_id
{
public:
  explicit Init_RequestParkingTask_Request_vehicle_id(::parking_robot_interfaces::srv::RequestParkingTask_Request & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::srv::RequestParkingTask_Request vehicle_id(::parking_robot_interfaces::srv::RequestParkingTask_Request::_vehicle_id_type arg)
  {
    msg_.vehicle_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::srv::RequestParkingTask_Request msg_;
};

class Init_RequestParkingTask_Request_request_type
{
public:
  Init_RequestParkingTask_Request_request_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RequestParkingTask_Request_vehicle_id request_type(::parking_robot_interfaces::srv::RequestParkingTask_Request::_request_type_type arg)
  {
    msg_.request_type = std::move(arg);
    return Init_RequestParkingTask_Request_vehicle_id(msg_);
  }

private:
  ::parking_robot_interfaces::srv::RequestParkingTask_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::srv::RequestParkingTask_Request>()
{
  return parking_robot_interfaces::srv::builder::Init_RequestParkingTask_Request_request_type();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_RequestParkingTask_Response_message
{
public:
  explicit Init_RequestParkingTask_Response_message(::parking_robot_interfaces::srv::RequestParkingTask_Response & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::srv::RequestParkingTask_Response message(::parking_robot_interfaces::srv::RequestParkingTask_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::srv::RequestParkingTask_Response msg_;
};

class Init_RequestParkingTask_Response_task_id
{
public:
  explicit Init_RequestParkingTask_Response_task_id(::parking_robot_interfaces::srv::RequestParkingTask_Response & msg)
  : msg_(msg)
  {}
  Init_RequestParkingTask_Response_message task_id(::parking_robot_interfaces::srv::RequestParkingTask_Response::_task_id_type arg)
  {
    msg_.task_id = std::move(arg);
    return Init_RequestParkingTask_Response_message(msg_);
  }

private:
  ::parking_robot_interfaces::srv::RequestParkingTask_Response msg_;
};

class Init_RequestParkingTask_Response_accepted
{
public:
  Init_RequestParkingTask_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RequestParkingTask_Response_task_id accepted(::parking_robot_interfaces::srv::RequestParkingTask_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_RequestParkingTask_Response_task_id(msg_);
  }

private:
  ::parking_robot_interfaces::srv::RequestParkingTask_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::srv::RequestParkingTask_Response>()
{
  return parking_robot_interfaces::srv::builder::Init_RequestParkingTask_Response_accepted();
}

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__SRV__DETAIL__REQUEST_PARKING_TASK__BUILDER_HPP_
