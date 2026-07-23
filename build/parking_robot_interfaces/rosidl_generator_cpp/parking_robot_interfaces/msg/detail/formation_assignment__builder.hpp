// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from parking_robot_interfaces:msg/FormationAssignment.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__MSG__DETAIL__FORMATION_ASSIGNMENT__BUILDER_HPP_
#define PARKING_ROBOT_INTERFACES__MSG__DETAIL__FORMATION_ASSIGNMENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "parking_robot_interfaces/msg/detail/formation_assignment__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace parking_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_FormationAssignment_active
{
public:
  explicit Init_FormationAssignment_active(::parking_robot_interfaces::msg::FormationAssignment & msg)
  : msg_(msg)
  {}
  ::parking_robot_interfaces::msg::FormationAssignment active(::parking_robot_interfaces::msg::FormationAssignment::_active_type arg)
  {
    msg_.active = std::move(arg);
    return std::move(msg_);
  }

private:
  ::parking_robot_interfaces::msg::FormationAssignment msg_;
};

class Init_FormationAssignment_partner_robot_id
{
public:
  explicit Init_FormationAssignment_partner_robot_id(::parking_robot_interfaces::msg::FormationAssignment & msg)
  : msg_(msg)
  {}
  Init_FormationAssignment_active partner_robot_id(::parking_robot_interfaces::msg::FormationAssignment::_partner_robot_id_type arg)
  {
    msg_.partner_robot_id = std::move(arg);
    return Init_FormationAssignment_active(msg_);
  }

private:
  ::parking_robot_interfaces::msg::FormationAssignment msg_;
};

class Init_FormationAssignment_role
{
public:
  explicit Init_FormationAssignment_role(::parking_robot_interfaces::msg::FormationAssignment & msg)
  : msg_(msg)
  {}
  Init_FormationAssignment_partner_robot_id role(::parking_robot_interfaces::msg::FormationAssignment::_role_type arg)
  {
    msg_.role = std::move(arg);
    return Init_FormationAssignment_partner_robot_id(msg_);
  }

private:
  ::parking_robot_interfaces::msg::FormationAssignment msg_;
};

class Init_FormationAssignment_task_id
{
public:
  explicit Init_FormationAssignment_task_id(::parking_robot_interfaces::msg::FormationAssignment & msg)
  : msg_(msg)
  {}
  Init_FormationAssignment_role task_id(::parking_robot_interfaces::msg::FormationAssignment::_task_id_type arg)
  {
    msg_.task_id = std::move(arg);
    return Init_FormationAssignment_role(msg_);
  }

private:
  ::parking_robot_interfaces::msg::FormationAssignment msg_;
};

class Init_FormationAssignment_robot_id
{
public:
  Init_FormationAssignment_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FormationAssignment_task_id robot_id(::parking_robot_interfaces::msg::FormationAssignment::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_FormationAssignment_task_id(msg_);
  }

private:
  ::parking_robot_interfaces::msg::FormationAssignment msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::parking_robot_interfaces::msg::FormationAssignment>()
{
  return parking_robot_interfaces::msg::builder::Init_FormationAssignment_robot_id();
}

}  // namespace parking_robot_interfaces

#endif  // PARKING_ROBOT_INTERFACES__MSG__DETAIL__FORMATION_ASSIGNMENT__BUILDER_HPP_
