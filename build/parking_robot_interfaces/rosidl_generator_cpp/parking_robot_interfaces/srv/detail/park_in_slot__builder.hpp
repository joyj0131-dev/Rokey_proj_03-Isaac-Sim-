// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from parking_robot_interfaces:srv/ParkInSlot.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__SRV__DETAIL__PARK_IN_SLOT__BUILDER_HPP_
#define PARKING_ROBOT_INTERFACES__SRV__DETAIL__PARK_IN_SLOT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "parking_robot_interfaces/srv/detail/park_in_slot__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace parking_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_ParkInSlot_Request_slot_id
{
public:
  Init_ParkInSlot_Request_slot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::parking_robot_interfaces::srv::ParkInSlot_Request slot_id(::parking_robot_interfaces::srv::ParkInSlot_Request::_slot_id_type arg)
  {
    msg_.slot_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::srv::ParkInSlot_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::srv::ParkInSlot_Request>()
{
  return parking_robot_interfaces::srv::builder::Init_ParkInSlot_Request_slot_id();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_ParkInSlot_Response_message
{
public:
  explicit Init_ParkInSlot_Response_message(::parking_robot_interfaces::srv::ParkInSlot_Response & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::srv::ParkInSlot_Response message(::parking_robot_interfaces::srv::ParkInSlot_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::srv::ParkInSlot_Response msg_;
};

class Init_ParkInSlot_Response_task_id
{
public:
  explicit Init_ParkInSlot_Response_task_id(::parking_robot_interfaces::srv::ParkInSlot_Response & msg)
  : msg_(msg)
  {}
  Init_ParkInSlot_Response_message task_id(::parking_robot_interfaces::srv::ParkInSlot_Response::_task_id_type arg)
  {
    msg_.task_id = std::move(arg);
    return Init_ParkInSlot_Response_message(msg_);
  }

private:
  ::parking_robot_interfaces::srv::ParkInSlot_Response msg_;
};

class Init_ParkInSlot_Response_accepted
{
public:
  Init_ParkInSlot_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ParkInSlot_Response_task_id accepted(::parking_robot_interfaces::srv::ParkInSlot_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_ParkInSlot_Response_task_id(msg_);
  }

private:
  ::parking_robot_interfaces::srv::ParkInSlot_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::srv::ParkInSlot_Response>()
{
  return parking_robot_interfaces::srv::builder::Init_ParkInSlot_Response_accepted();
}

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__SRV__DETAIL__PARK_IN_SLOT__BUILDER_HPP_
