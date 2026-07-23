// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from parking_robot_interfaces:msg/TaskState.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__MSG__DETAIL__TASK_STATE__TRAITS_HPP_
#define PARKING_ROBOT_INTERFACES__MSG__DETAIL__TASK_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "parking_robot_interfaces/msg/detail/task_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace parking_robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TaskState & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << ", ";
  }

  // member: task_id
  {
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
    out << ", ";
  }

  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: current_step
  {
    out << "current_step: ";
    rosidl_generator_traits::value_to_yaml(msg.current_step, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TaskState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: robot_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << "\n";
  }

  // member: task_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
    out << "\n";
  }

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }

  // member: current_step
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_step: ";
    rosidl_generator_traits::value_to_yaml(msg.current_step, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TaskState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace parking_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use parking_robot_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const parking_robot_interfaces::msg::TaskState & msg,
  std::ostream & out, size_t indentation = 0)
{
  parking_robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use parking_robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const parking_robot_interfaces::msg::TaskState & msg)
{
  return parking_robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<parking_robot_interfaces::msg::TaskState>()
{
  return "parking_robot_interfaces::msg::TaskState";
}

template<>
inline const char * name<parking_robot_interfaces::msg::TaskState>()
{
  return "parking_robot_interfaces/msg/TaskState";
}

template<>
struct has_fixed_size<parking_robot_interfaces::msg::TaskState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<parking_robot_interfaces::msg::TaskState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<parking_robot_interfaces::msg::TaskState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PARKING_ROBOT_INTERFACES__MSG__DETAIL__TASK_STATE__TRAITS_HPP_
