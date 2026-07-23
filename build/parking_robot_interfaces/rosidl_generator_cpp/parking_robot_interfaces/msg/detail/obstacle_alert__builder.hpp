// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from parking_robot_interfaces:msg/ObstacleAlert.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__MSG__DETAIL__OBSTACLE_ALERT__BUILDER_HPP_
#define PARKING_ROBOT_INTERFACES__MSG__DETAIL__OBSTACLE_ALERT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "parking_robot_interfaces/msg/detail/obstacle_alert__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace parking_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_ObstacleAlert_location
{
public:
  explicit Init_ObstacleAlert_location(::parking_robot_interfaces::msg::ObstacleAlert & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::msg::ObstacleAlert location(::parking_robot_interfaces::msg::ObstacleAlert::_location_type arg)
  {
    msg_.location = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::msg::ObstacleAlert msg_;
};

class Init_ObstacleAlert_description
{
public:
  explicit Init_ObstacleAlert_description(::parking_robot_interfaces::msg::ObstacleAlert & msg)
  : msg_(msg)
  {}
  Init_ObstacleAlert_location description(::parking_robot_interfaces::msg::ObstacleAlert::_description_type arg)
  {
    msg_.description = std::move(arg);
    return Init_ObstacleAlert_location(msg_);
  }

private:
  ::parking_robot_interfaces::msg::ObstacleAlert msg_;
};

class Init_ObstacleAlert_obstacle_detected
{
public:
  Init_ObstacleAlert_obstacle_detected()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObstacleAlert_description obstacle_detected(::parking_robot_interfaces::msg::ObstacleAlert::_obstacle_detected_type arg)
  {
    msg_.obstacle_detected = std::move(arg);
    return Init_ObstacleAlert_description(msg_);
  }

private:
  ::parking_robot_interfaces::msg::ObstacleAlert msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::msg::ObstacleAlert>()
{
  return parking_robot_interfaces::msg::builder::Init_ObstacleAlert_obstacle_detected();
}

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__MSG__DETAIL__OBSTACLE_ALERT__BUILDER_HPP_
