// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from parking_robot_interfaces:srv/AcquireZones.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__SRV__DETAIL__ACQUIRE_ZONES__BUILDER_HPP_
#define PARKING_ROBOT_INTERFACES__SRV__DETAIL__ACQUIRE_ZONES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "parking_robot_interfaces/srv/detail/acquire_zones__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace parking_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_AcquireZones_Request_zone_ids
{
public:
  explicit Init_AcquireZones_Request_zone_ids(::parking_robot_interfaces::srv::AcquireZones_Request & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::srv::AcquireZones_Request zone_ids(::parking_robot_interfaces::srv::AcquireZones_Request::_zone_ids_type arg)
  {
    msg_.zone_ids = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::srv::AcquireZones_Request msg_;
};

class Init_AcquireZones_Request_task_id
{
public:
  explicit Init_AcquireZones_Request_task_id(::parking_robot_interfaces::srv::AcquireZones_Request & msg)
  : msg_(msg)
  {}
  Init_AcquireZones_Request_zone_ids task_id(::parking_robot_interfaces::srv::AcquireZones_Request::_task_id_type arg)
  {
    msg_.task_id = std::move(arg);
    return Init_AcquireZones_Request_zone_ids(msg_);
  }

private:
  ::parking_robot_interfaces::srv::AcquireZones_Request msg_;
};

class Init_AcquireZones_Request_robot_id
{
public:
  Init_AcquireZones_Request_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AcquireZones_Request_task_id robot_id(::parking_robot_interfaces::srv::AcquireZones_Request::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_AcquireZones_Request_task_id(msg_);
  }

private:
  ::parking_robot_interfaces::srv::AcquireZones_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::srv::AcquireZones_Request>()
{
  return parking_robot_interfaces::srv::builder::Init_AcquireZones_Request_robot_id();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_AcquireZones_Response_retry_after_sec
{
public:
  explicit Init_AcquireZones_Response_retry_after_sec(::parking_robot_interfaces::srv::AcquireZones_Response & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::srv::AcquireZones_Response retry_after_sec(::parking_robot_interfaces::srv::AcquireZones_Response::_retry_after_sec_type arg)
  {
    msg_.retry_after_sec = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::srv::AcquireZones_Response msg_;
};

class Init_AcquireZones_Response_held_zones
{
public:
  explicit Init_AcquireZones_Response_held_zones(::parking_robot_interfaces::srv::AcquireZones_Response & msg)
  : msg_(msg)
  {}
  Init_AcquireZones_Response_retry_after_sec held_zones(::parking_robot_interfaces::srv::AcquireZones_Response::_held_zones_type arg)
  {
    msg_.held_zones = std::move(arg);
    return Init_AcquireZones_Response_retry_after_sec(msg_);
  }

private:
  ::parking_robot_interfaces::srv::AcquireZones_Response msg_;
};

class Init_AcquireZones_Response_granted
{
public:
  Init_AcquireZones_Response_granted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AcquireZones_Response_held_zones granted(::parking_robot_interfaces::srv::AcquireZones_Response::_granted_type arg)
  {
    msg_.granted = std::move(arg);
    return Init_AcquireZones_Response_held_zones(msg_);
  }

private:
  ::parking_robot_interfaces::srv::AcquireZones_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::srv::AcquireZones_Response>()
{
  return parking_robot_interfaces::srv::builder::Init_AcquireZones_Response_granted();
}

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__SRV__DETAIL__ACQUIRE_ZONES__BUILDER_HPP_
