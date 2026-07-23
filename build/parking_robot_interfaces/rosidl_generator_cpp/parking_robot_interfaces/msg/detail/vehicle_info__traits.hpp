// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from parking_robot_interfaces:msg/VehicleInfo.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__MSG__DETAIL__VEHICLE_INFO__TRAITS_HPP_
#define PARKING_ROBOT_INTERFACES__MSG__DETAIL__VEHICLE_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "parking_robot_interfaces/msg/detail/vehicle_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace parking_robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const VehicleInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: length
  {
    out << "length: ";
    rosidl_generator_traits::value_to_yaml(msg.length, out);
    out << ", ";
  }

  // member: width
  {
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << ", ";
  }

  // member: height
  {
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VehicleInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "length: ";
    rosidl_generator_traits::value_to_yaml(msg.length, out);
    out << "\n";
  }

  // member: width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << "\n";
  }

  // member: height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VehicleInfo & msg, bool use_flow_style = false)
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
  const parking_robot_interfaces::msg::VehicleInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  parking_robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use parking_robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const parking_robot_interfaces::msg::VehicleInfo & msg)
{
  return parking_robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<parking_robot_interfaces::msg::VehicleInfo>()
{
  return "parking_robot_interfaces::msg::VehicleInfo";
}

template<>
inline const char * name<parking_robot_interfaces::msg::VehicleInfo>()
{
  return "parking_robot_interfaces/msg/VehicleInfo";
}

template<>
struct has_fixed_size<parking_robot_interfaces::msg::VehicleInfo>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct has_bounded_size<parking_robot_interfaces::msg::VehicleInfo>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct is_message<parking_robot_interfaces::msg::VehicleInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PARKING_ROBOT_INTERFACES__MSG__DETAIL__VEHICLE_INFO__TRAITS_HPP_
