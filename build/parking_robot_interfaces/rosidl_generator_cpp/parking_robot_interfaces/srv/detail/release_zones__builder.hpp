// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from parking_robot_interfaces:srv/ReleaseZones.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__SRV__DETAIL__RELEASE_ZONES__BUILDER_HPP_
#define PARKING_ROBOT_INTERFACES__SRV__DETAIL__RELEASE_ZONES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "parking_robot_interfaces/srv/detail/release_zones__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace parking_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_ReleaseZones_Request_zone_ids
{
public:
  explicit Init_ReleaseZones_Request_zone_ids(::parking_robot_interfaces::srv::ReleaseZones_Request & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::srv::ReleaseZones_Request zone_ids(::parking_robot_interfaces::srv::ReleaseZones_Request::_zone_ids_type arg)
  {
    msg_.zone_ids = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::srv::ReleaseZones_Request msg_;
};

class Init_ReleaseZones_Request_task_id
{
public:
  explicit Init_ReleaseZones_Request_task_id(::parking_robot_interfaces::srv::ReleaseZones_Request & msg)
  : msg_(msg)
  {}
  Init_ReleaseZones_Request_zone_ids task_id(::parking_robot_interfaces::srv::ReleaseZones_Request::_task_id_type arg)
  {
    msg_.task_id = std::move(arg);
    return Init_ReleaseZones_Request_zone_ids(msg_);
  }

private:
  ::parking_robot_interfaces::srv::ReleaseZones_Request msg_;
};

class Init_ReleaseZones_Request_robot_id
{
public:
  Init_ReleaseZones_Request_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ReleaseZones_Request_task_id robot_id(::parking_robot_interfaces::srv::ReleaseZones_Request::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_ReleaseZones_Request_task_id(msg_);
  }

private:
  ::parking_robot_interfaces::srv::ReleaseZones_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::srv::ReleaseZones_Request>()
{
  return parking_robot_interfaces::srv::builder::Init_ReleaseZones_Request_robot_id();
}

}  // namespace parking_robot_interfaces


namespace parking_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_ReleaseZones_Response_success
{
public:
  Init_ReleaseZones_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::parking_robot_interfaces::srv::ReleaseZones_Response success(::parking_robot_interfaces::srv::ReleaseZones_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::srv::ReleaseZones_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::srv::ReleaseZones_Response>()
{
  return parking_robot_interfaces::srv::builder::Init_ReleaseZones_Response_success();
}

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__SRV__DETAIL__RELEASE_ZONES__BUILDER_HPP_
