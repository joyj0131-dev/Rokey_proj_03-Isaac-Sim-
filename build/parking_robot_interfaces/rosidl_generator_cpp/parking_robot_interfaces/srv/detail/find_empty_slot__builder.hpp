// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from parking_robot_interfaces:srv/FindEmptySlot.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__SRV__DETAIL__FIND_EMPTY_SLOT__BUILDER_HPP_
#define PARKING_ROBOT_INTERFACES__SRV__DETAIL__FIND_EMPTY_SLOT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "parking_robot_interfaces/srv/detail/find_empty_slot__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace parking_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_FindEmptySlot_Request_vehicle_width
{
public:
  explicit Init_FindEmptySlot_Request_vehicle_width(::parking_robot_interfaces::srv::FindEmptySlot_Request & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::srv::FindEmptySlot_Request vehicle_width(::parking_robot_interfaces::srv::FindEmptySlot_Request::_vehicle_width_type arg)
  {
    msg_.vehicle_width = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::srv::FindEmptySlot_Request msg_;
};

class Init_FindEmptySlot_Request_vehicle_length
{
public:
  Init_FindEmptySlot_Request_vehicle_length()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FindEmptySlot_Request_vehicle_width vehicle_length(::parking_robot_interfaces::srv::FindEmptySlot_Request::_vehicle_length_type arg)
  {
    msg_.vehicle_length = std::move(arg);
    return Init_FindEmptySlot_Request_vehicle_width(msg_);
  }

private:
  ::parking_robot_interfaces::srv::FindEmptySlot_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::srv::FindEmptySlot_Request>()
{
  return parking_robot_interfaces::srv::builder::Init_FindEmptySlot_Request_vehicle_length();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_FindEmptySlot_Response_slot_pose
{
public:
  explicit Init_FindEmptySlot_Response_slot_pose(::parking_robot_interfaces::srv::FindEmptySlot_Response & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::srv::FindEmptySlot_Response slot_pose(::parking_robot_interfaces::srv::FindEmptySlot_Response::_slot_pose_type arg)
  {
    msg_.slot_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::srv::FindEmptySlot_Response msg_;
};

class Init_FindEmptySlot_Response_slot_id
{
public:
  explicit Init_FindEmptySlot_Response_slot_id(::parking_robot_interfaces::srv::FindEmptySlot_Response & msg)
  : msg_(msg)
  {}
  Init_FindEmptySlot_Response_slot_pose slot_id(::parking_robot_interfaces::srv::FindEmptySlot_Response::_slot_id_type arg)
  {
    msg_.slot_id = std::move(arg);
    return Init_FindEmptySlot_Response_slot_pose(msg_);
  }

private:
  ::parking_robot_interfaces::srv::FindEmptySlot_Response msg_;
};

class Init_FindEmptySlot_Response_success
{
public:
  Init_FindEmptySlot_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FindEmptySlot_Response_slot_id success(::parking_robot_interfaces::srv::FindEmptySlot_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_FindEmptySlot_Response_slot_id(msg_);
  }

private:
  ::parking_robot_interfaces::srv::FindEmptySlot_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::srv::FindEmptySlot_Response>()
{
  return parking_robot_interfaces::srv::builder::Init_FindEmptySlot_Response_success();
}

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__SRV__DETAIL__FIND_EMPTY_SLOT__BUILDER_HPP_
