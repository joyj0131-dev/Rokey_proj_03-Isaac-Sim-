// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from parking_robot_interfaces:srv/GetSlotInfo.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__SRV__DETAIL__GET_SLOT_INFO__BUILDER_HPP_
#define PARKING_ROBOT_INTERFACES__SRV__DETAIL__GET_SLOT_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "parking_robot_interfaces/srv/detail/get_slot_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace parking_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetSlotInfo_Request_slot_id
{
public:
  Init_GetSlotInfo_Request_slot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::parking_robot_interfaces::srv::GetSlotInfo_Request slot_id(::parking_robot_interfaces::srv::GetSlotInfo_Request::_slot_id_type arg)
  {
    msg_.slot_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::srv::GetSlotInfo_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::srv::GetSlotInfo_Request>()
{
  return parking_robot_interfaces::srv::builder::Init_GetSlotInfo_Request_slot_id();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetSlotInfo_Response_pose
{
public:
  explicit Init_GetSlotInfo_Response_pose(::parking_robot_interfaces::srv::GetSlotInfo_Response & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::srv::GetSlotInfo_Response pose(::parking_robot_interfaces::srv::GetSlotInfo_Response::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::srv::GetSlotInfo_Response msg_;
};

class Init_GetSlotInfo_Response_is_accessible
{
public:
  explicit Init_GetSlotInfo_Response_is_accessible(::parking_robot_interfaces::srv::GetSlotInfo_Response & msg)
  : msg_(msg)
  {}
  Init_GetSlotInfo_Response_pose is_accessible(::parking_robot_interfaces::srv::GetSlotInfo_Response::_is_accessible_type arg)
  {
    msg_.is_accessible = std::move(arg);
    return Init_GetSlotInfo_Response_pose(msg_);
  }

private:
  ::parking_robot_interfaces::srv::GetSlotInfo_Response msg_;
};

class Init_GetSlotInfo_Response_occupied
{
public:
  explicit Init_GetSlotInfo_Response_occupied(::parking_robot_interfaces::srv::GetSlotInfo_Response & msg)
  : msg_(msg)
  {}
  Init_GetSlotInfo_Response_is_accessible occupied(::parking_robot_interfaces::srv::GetSlotInfo_Response::_occupied_type arg)
  {
    msg_.occupied = std::move(arg);
    return Init_GetSlotInfo_Response_is_accessible(msg_);
  }

private:
  ::parking_robot_interfaces::srv::GetSlotInfo_Response msg_;
};

class Init_GetSlotInfo_Response_exists
{
public:
  explicit Init_GetSlotInfo_Response_exists(::parking_robot_interfaces::srv::GetSlotInfo_Response & msg)
  : msg_(msg)
  {}
  Init_GetSlotInfo_Response_occupied exists(::parking_robot_interfaces::srv::GetSlotInfo_Response::_exists_type arg)
  {
    msg_.exists = std::move(arg);
    return Init_GetSlotInfo_Response_occupied(msg_);
  }

private:
  ::parking_robot_interfaces::srv::GetSlotInfo_Response msg_;
};

class Init_GetSlotInfo_Response_data_ready
{
public:
  Init_GetSlotInfo_Response_data_ready()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetSlotInfo_Response_exists data_ready(::parking_robot_interfaces::srv::GetSlotInfo_Response::_data_ready_type arg)
  {
    msg_.data_ready = std::move(arg);
    return Init_GetSlotInfo_Response_exists(msg_);
  }

private:
  ::parking_robot_interfaces::srv::GetSlotInfo_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::srv::GetSlotInfo_Response>()
{
  return parking_robot_interfaces::srv::builder::Init_GetSlotInfo_Response_data_ready();
}

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__SRV__DETAIL__GET_SLOT_INFO__BUILDER_HPP_
