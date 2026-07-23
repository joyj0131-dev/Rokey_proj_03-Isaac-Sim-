// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from parking_robot_interfaces:msg/FormationAssignment.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__MSG__DETAIL__FORMATION_ASSIGNMENT__TRAITS_HPP_
#define PARKING_ROBOT_INTERFACES__MSG__DETAIL__FORMATION_ASSIGNMENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "parking_robot_interfaces/msg/detail/formation_assignment__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace parking_robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const FormationAssignment & msg,
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

  // member: role
  {
    out << "role: ";
    rosidl_generator_traits::value_to_yaml(msg.role, out);
    out << ", ";
  }

  // member: partner_robot_id
  {
    out << "partner_robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.partner_robot_id, out);
    out << ", ";
  }

  // member: active
  {
    out << "active: ";
    rosidl_generator_traits::value_to_yaml(msg.active, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FormationAssignment & msg,
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

  // member: role
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "role: ";
    rosidl_generator_traits::value_to_yaml(msg.role, out);
    out << "\n";
  }

  // member: partner_robot_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "partner_robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.partner_robot_id, out);
    out << "\n";
  }

  // member: active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "active: ";
    rosidl_generator_traits::value_to_yaml(msg.active, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FormationAssignment & msg, bool use_flow_style = false)
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
  const parking_robot_interfaces::msg::FormationAssignment & msg,
  std::ostream & out, size_t indentation = 0)
{
  parking_robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use parking_robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const parking_robot_interfaces::msg::FormationAssignment & msg)
{
  return parking_robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<parking_robot_interfaces::msg::FormationAssignment>()
{
  return "parking_robot_interfaces::msg::FormationAssignment";
}

template<>
inline const char * name<parking_robot_interfaces::msg::FormationAssignment>()
{
  return "parking_robot_interfaces/msg/FormationAssignment";
}

template<>
struct has_fixed_size<parking_robot_interfaces::msg::FormationAssignment>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<parking_robot_interfaces::msg::FormationAssignment>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<parking_robot_interfaces::msg::FormationAssignment>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PARKING_ROBOT_INTERFACES__MSG__DETAIL__FORMATION_ASSIGNMENT__TRAITS_HPP_
