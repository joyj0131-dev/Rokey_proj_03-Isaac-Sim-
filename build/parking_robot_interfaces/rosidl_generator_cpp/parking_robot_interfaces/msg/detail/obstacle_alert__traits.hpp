// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from parking_robot_interfaces:msg/ObstacleAlert.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__MSG__DETAIL__OBSTACLE_ALERT__TRAITS_HPP_
#define PARKING_ROBOT_INTERFACES__MSG__DETAIL__OBSTACLE_ALERT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "parking_robot_interfaces/msg/detail/obstacle_alert__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'location'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace parking_robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ObstacleAlert & msg,
  std::ostream & out)
{
  out << "{";
  // member: obstacle_detected
  {
    out << "obstacle_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle_detected, out);
    out << ", ";
  }

  // member: description
  {
    out << "description: ";
    rosidl_generator_traits::value_to_yaml(msg.description, out);
    out << ", ";
  }

  // member: location
  {
    out << "location: ";
    to_flow_style_yaml(msg.location, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ObstacleAlert & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: obstacle_detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "obstacle_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle_detected, out);
    out << "\n";
  }

  // member: description
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "description: ";
    rosidl_generator_traits::value_to_yaml(msg.description, out);
    out << "\n";
  }

  // member: location
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "location:\n";
    to_block_style_yaml(msg.location, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ObstacleAlert & msg, bool use_flow_style = false)
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
  const parking_robot_interfaces::msg::ObstacleAlert & msg,
  std::ostream & out, size_t indentation = 0)
{
  parking_robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use parking_robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const parking_robot_interfaces::msg::ObstacleAlert & msg)
{
  return parking_robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<parking_robot_interfaces::msg::ObstacleAlert>()
{
  return "parking_robot_interfaces::msg::ObstacleAlert";
}

template<>
inline const char * name<parking_robot_interfaces::msg::ObstacleAlert>()
{
  return "parking_robot_interfaces/msg/ObstacleAlert";
}

template<>
struct has_fixed_size<parking_robot_interfaces::msg::ObstacleAlert>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<parking_robot_interfaces::msg::ObstacleAlert>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<parking_robot_interfaces::msg::ObstacleAlert>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PARKING_ROBOT_INTERFACES__MSG__DETAIL__OBSTACLE_ALERT__TRAITS_HPP_
