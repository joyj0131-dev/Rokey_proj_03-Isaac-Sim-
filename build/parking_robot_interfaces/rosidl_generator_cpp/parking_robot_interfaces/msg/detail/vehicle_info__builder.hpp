// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from parking_robot_interfaces:msg/VehicleInfo.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__MSG__DETAIL__VEHICLE_INFO__BUILDER_HPP_
#define PARKING_ROBOT_INTERFACES__MSG__DETAIL__VEHICLE_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "parking_robot_interfaces/msg/detail/vehicle_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace parking_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_VehicleInfo_height
{
public:
  explicit Init_VehicleInfo_height(::parking_robot_interfaces::msg::VehicleInfo & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::msg::VehicleInfo height(::parking_robot_interfaces::msg::VehicleInfo::_height_type arg)
  {
    msg_.height = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::msg::VehicleInfo msg_;
};

class Init_VehicleInfo_width
{
public:
  explicit Init_VehicleInfo_width(::parking_robot_interfaces::msg::VehicleInfo & msg)
  : msg_(msg)
  {}
  Init_VehicleInfo_height width(::parking_robot_interfaces::msg::VehicleInfo::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_VehicleInfo_height(msg_);
  }

private:
  ::parking_robot_interfaces::msg::VehicleInfo msg_;
};

class Init_VehicleInfo_length
{
public:
  explicit Init_VehicleInfo_length(::parking_robot_interfaces::msg::VehicleInfo & msg)
  : msg_(msg)
  {}
  Init_VehicleInfo_width length(::parking_robot_interfaces::msg::VehicleInfo::_length_type arg)
  {
    msg_.length = std::move(arg);
    return Init_VehicleInfo_width(msg_);
  }

private:
  ::parking_robot_interfaces::msg::VehicleInfo msg_;
};

class Init_VehicleInfo_pose
{
public:
  Init_VehicleInfo_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VehicleInfo_length pose(::parking_robot_interfaces::msg::VehicleInfo::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_VehicleInfo_length(msg_);
  }

private:
  ::parking_robot_interfaces::msg::VehicleInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::msg::VehicleInfo>()
{
  return parking_robot_interfaces::msg::builder::Init_VehicleInfo_pose();
}

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__MSG__DETAIL__VEHICLE_INFO__BUILDER_HPP_
