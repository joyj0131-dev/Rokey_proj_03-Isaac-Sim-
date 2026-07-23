// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from parking_robot_interfaces:msg/FormationStop.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__MSG__DETAIL__FORMATION_STOP__BUILDER_HPP_
#define PARKING_ROBOT_INTERFACES__MSG__DETAIL__FORMATION_STOP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "parking_robot_interfaces/msg/detail/formation_stop__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace parking_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_FormationStop_reason
{
public:
  explicit Init_FormationStop_reason(::parking_robot_interfaces::msg::FormationStop & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::msg::FormationStop reason(::parking_robot_interfaces::msg::FormationStop::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::msg::FormationStop msg_;
};

class Init_FormationStop_stop
{
public:
  explicit Init_FormationStop_stop(::parking_robot_interfaces::msg::FormationStop & msg)
  : msg_(msg)
  {}
  Init_FormationStop_reason stop(::parking_robot_interfaces::msg::FormationStop::_stop_type arg)
  {
    msg_.stop = std::move(arg);
    return Init_FormationStop_reason(msg_);
  }

private:
  ::parking_robot_interfaces::msg::FormationStop msg_;
};

class Init_FormationStop_source_robot_id
{
public:
  explicit Init_FormationStop_source_robot_id(::parking_robot_interfaces::msg::FormationStop & msg)
  : msg_(msg)
  {}
  Init_FormationStop_stop source_robot_id(::parking_robot_interfaces::msg::FormationStop::_source_robot_id_type arg)
  {
    msg_.source_robot_id = std::move(arg);
    return Init_FormationStop_stop(msg_);
  }

private:
  ::parking_robot_interfaces::msg::FormationStop msg_;
};

class Init_FormationStop_task_id
{
public:
  Init_FormationStop_task_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FormationStop_source_robot_id task_id(::parking_robot_interfaces::msg::FormationStop::_task_id_type arg)
  {
    msg_.task_id = std::move(arg);
    return Init_FormationStop_source_robot_id(msg_);
  }

private:
  ::parking_robot_interfaces::msg::FormationStop msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::msg::FormationStop>()
{
  return parking_robot_interfaces::msg::builder::Init_FormationStop_task_id();
}

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__MSG__DETAIL__FORMATION_STOP__BUILDER_HPP_
