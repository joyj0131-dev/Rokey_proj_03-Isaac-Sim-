// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from parking_robot_interfaces:srv/FindEmptySlot.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__SRV__DETAIL__FIND_EMPTY_SLOT__TRAITS_HPP_
#define PARKING_ROBOT_INTERFACES__SRV__DETAIL__FIND_EMPTY_SLOT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "parking_robot_interfaces/srv/detail/find_empty_slot__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace parking_robot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const FindEmptySlot_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: vehicle_length
  {
    out << "vehicle_length: ";
    rosidl_generator_traits::value_to_yaml(msg.vehicle_length, out);
    out << ", ";
  }

  // member: vehicle_width
  {
    out << "vehicle_width: ";
    rosidl_generator_traits::value_to_yaml(msg.vehicle_width, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FindEmptySlot_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: vehicle_length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vehicle_length: ";
    rosidl_generator_traits::value_to_yaml(msg.vehicle_length, out);
    out << "\n";
  }

  // member: vehicle_width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vehicle_width: ";
    rosidl_generator_traits::value_to_yaml(msg.vehicle_width, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FindEmptySlot_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace parking_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use parking_robot_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const parking_robot_interfaces::srv::FindEmptySlot_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  parking_robot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use parking_robot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const parking_robot_interfaces::srv::FindEmptySlot_Request & msg)
{
  return parking_robot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<parking_robot_interfaces::srv::FindEmptySlot_Request>()
{
  return "parking_robot_interfaces::srv::FindEmptySlot_Request";
}

template<>
inline const char * name<parking_robot_interfaces::srv::FindEmptySlot_Request>()
{
  return "parking_robot_interfaces/srv/FindEmptySlot_Request";
}

template<>
struct has_fixed_size<parking_robot_interfaces::srv::FindEmptySlot_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<parking_robot_interfaces::srv::FindEmptySlot_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<parking_robot_interfaces::srv::FindEmptySlot_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'slot_pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace parking_robot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const FindEmptySlot_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: slot_id
  {
    out << "slot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.slot_id, out);
    out << ", ";
  }

  // member: slot_pose
  {
    out << "slot_pose: ";
    to_flow_style_yaml(msg.slot_pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FindEmptySlot_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: slot_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "slot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.slot_id, out);
    out << "\n";
  }

  // member: slot_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "slot_pose:\n";
    to_block_style_yaml(msg.slot_pose, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FindEmptySlot_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace parking_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use parking_robot_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const parking_robot_interfaces::srv::FindEmptySlot_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  parking_robot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use parking_robot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const parking_robot_interfaces::srv::FindEmptySlot_Response & msg)
{
  return parking_robot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<parking_robot_interfaces::srv::FindEmptySlot_Response>()
{
  return "parking_robot_interfaces::srv::FindEmptySlot_Response";
}

template<>
inline const char * name<parking_robot_interfaces::srv::FindEmptySlot_Response>()
{
  return "parking_robot_interfaces/srv/FindEmptySlot_Response";
}

template<>
struct has_fixed_size<parking_robot_interfaces::srv::FindEmptySlot_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<parking_robot_interfaces::srv::FindEmptySlot_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<parking_robot_interfaces::srv::FindEmptySlot_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<parking_robot_interfaces::srv::FindEmptySlot>()
{
  return "parking_robot_interfaces::srv::FindEmptySlot";
}

template<>
inline const char * name<parking_robot_interfaces::srv::FindEmptySlot>()
{
  return "parking_robot_interfaces/srv/FindEmptySlot";
}

template<>
struct has_fixed_size<parking_robot_interfaces::srv::FindEmptySlot>
  : std::integral_constant<
    bool,
    has_fixed_size<parking_robot_interfaces::srv::FindEmptySlot_Request>::value &&
    has_fixed_size<parking_robot_interfaces::srv::FindEmptySlot_Response>::value
  >
{
};

template<>
struct has_bounded_size<parking_robot_interfaces::srv::FindEmptySlot>
  : std::integral_constant<
    bool,
    has_bounded_size<parking_robot_interfaces::srv::FindEmptySlot_Request>::value &&
    has_bounded_size<parking_robot_interfaces::srv::FindEmptySlot_Response>::value
  >
{
};

template<>
struct is_service<parking_robot_interfaces::srv::FindEmptySlot>
  : std::true_type
{
};

template<>
struct is_service_request<parking_robot_interfaces::srv::FindEmptySlot_Request>
  : std::true_type
{
};

template<>
struct is_service_response<parking_robot_interfaces::srv::FindEmptySlot_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PARKING_ROBOT_INTERFACES__SRV__DETAIL__FIND_EMPTY_SLOT__TRAITS_HPP_
