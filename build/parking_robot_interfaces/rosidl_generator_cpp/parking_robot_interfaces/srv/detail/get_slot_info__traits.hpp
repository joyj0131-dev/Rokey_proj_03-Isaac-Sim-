// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from parking_robot_interfaces:srv/GetSlotInfo.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__SRV__DETAIL__GET_SLOT_INFO__TRAITS_HPP_
#define PARKING_ROBOT_INTERFACES__SRV__DETAIL__GET_SLOT_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "parking_robot_interfaces/srv/detail/get_slot_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace parking_robot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetSlotInfo_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: slot_id
  {
    out << "slot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.slot_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetSlotInfo_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: slot_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "slot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.slot_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetSlotInfo_Request & msg, bool use_flow_style = false)
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
  const parking_robot_interfaces::srv::GetSlotInfo_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  parking_robot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use parking_robot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const parking_robot_interfaces::srv::GetSlotInfo_Request & msg)
{
  return parking_robot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<parking_robot_interfaces::srv::GetSlotInfo_Request>()
{
  return "parking_robot_interfaces::srv::GetSlotInfo_Request";
}

template<>
inline const char * name<parking_robot_interfaces::srv::GetSlotInfo_Request>()
{
  return "parking_robot_interfaces/srv/GetSlotInfo_Request";
}

template<>
struct has_fixed_size<parking_robot_interfaces::srv::GetSlotInfo_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<parking_robot_interfaces::srv::GetSlotInfo_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<parking_robot_interfaces::srv::GetSlotInfo_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace parking_robot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetSlotInfo_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: data_ready
  {
    out << "data_ready: ";
    rosidl_generator_traits::value_to_yaml(msg.data_ready, out);
    out << ", ";
  }

  // member: exists
  {
    out << "exists: ";
    rosidl_generator_traits::value_to_yaml(msg.exists, out);
    out << ", ";
  }

  // member: occupied
  {
    out << "occupied: ";
    rosidl_generator_traits::value_to_yaml(msg.occupied, out);
    out << ", ";
  }

  // member: is_accessible
  {
    out << "is_accessible: ";
    rosidl_generator_traits::value_to_yaml(msg.is_accessible, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetSlotInfo_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data_ready
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data_ready: ";
    rosidl_generator_traits::value_to_yaml(msg.data_ready, out);
    out << "\n";
  }

  // member: exists
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "exists: ";
    rosidl_generator_traits::value_to_yaml(msg.exists, out);
    out << "\n";
  }

  // member: occupied
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "occupied: ";
    rosidl_generator_traits::value_to_yaml(msg.occupied, out);
    out << "\n";
  }

  // member: is_accessible
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_accessible: ";
    rosidl_generator_traits::value_to_yaml(msg.is_accessible, out);
    out << "\n";
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetSlotInfo_Response & msg, bool use_flow_style = false)
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
  const parking_robot_interfaces::srv::GetSlotInfo_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  parking_robot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use parking_robot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const parking_robot_interfaces::srv::GetSlotInfo_Response & msg)
{
  return parking_robot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<parking_robot_interfaces::srv::GetSlotInfo_Response>()
{
  return "parking_robot_interfaces::srv::GetSlotInfo_Response";
}

template<>
inline const char * name<parking_robot_interfaces::srv::GetSlotInfo_Response>()
{
  return "parking_robot_interfaces/srv/GetSlotInfo_Response";
}

template<>
struct has_fixed_size<parking_robot_interfaces::srv::GetSlotInfo_Response>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct has_bounded_size<parking_robot_interfaces::srv::GetSlotInfo_Response>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct is_message<parking_robot_interfaces::srv::GetSlotInfo_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<parking_robot_interfaces::srv::GetSlotInfo>()
{
  return "parking_robot_interfaces::srv::GetSlotInfo";
}

template<>
inline const char * name<parking_robot_interfaces::srv::GetSlotInfo>()
{
  return "parking_robot_interfaces/srv/GetSlotInfo";
}

template<>
struct has_fixed_size<parking_robot_interfaces::srv::GetSlotInfo>
  : std::integral_constant<
    bool,
    has_fixed_size<parking_robot_interfaces::srv::GetSlotInfo_Request>::value &&
    has_fixed_size<parking_robot_interfaces::srv::GetSlotInfo_Response>::value
  >
{
};

template<>
struct has_bounded_size<parking_robot_interfaces::srv::GetSlotInfo>
  : std::integral_constant<
    bool,
    has_bounded_size<parking_robot_interfaces::srv::GetSlotInfo_Request>::value &&
    has_bounded_size<parking_robot_interfaces::srv::GetSlotInfo_Response>::value
  >
{
};

template<>
struct is_service<parking_robot_interfaces::srv::GetSlotInfo>
  : std::true_type
{
};

template<>
struct is_service_request<parking_robot_interfaces::srv::GetSlotInfo_Request>
  : std::true_type
{
};

template<>
struct is_service_response<parking_robot_interfaces::srv::GetSlotInfo_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PARKING_ROBOT_INTERFACES__SRV__DETAIL__GET_SLOT_INFO__TRAITS_HPP_
