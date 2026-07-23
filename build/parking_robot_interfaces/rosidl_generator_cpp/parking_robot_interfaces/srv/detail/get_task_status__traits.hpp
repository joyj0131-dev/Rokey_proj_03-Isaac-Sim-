// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from parking_robot_interfaces:srv/GetTaskStatus.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__SRV__DETAIL__GET_TASK_STATUS__TRAITS_HPP_
#define PARKING_ROBOT_INTERFACES__SRV__DETAIL__GET_TASK_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "parking_robot_interfaces/srv/detail/get_task_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace parking_robot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetTaskStatus_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: task_id
  {
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetTaskStatus_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: task_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetTaskStatus_Request & msg, bool use_flow_style = false)
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
  const parking_robot_interfaces::srv::GetTaskStatus_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  parking_robot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use parking_robot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const parking_robot_interfaces::srv::GetTaskStatus_Request & msg)
{
  return parking_robot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<parking_robot_interfaces::srv::GetTaskStatus_Request>()
{
  return "parking_robot_interfaces::srv::GetTaskStatus_Request";
}

template<>
inline const char * name<parking_robot_interfaces::srv::GetTaskStatus_Request>()
{
  return "parking_robot_interfaces/srv/GetTaskStatus_Request";
}

template<>
struct has_fixed_size<parking_robot_interfaces::srv::GetTaskStatus_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<parking_robot_interfaces::srv::GetTaskStatus_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<parking_robot_interfaces::srv::GetTaskStatus_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace parking_robot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetTaskStatus_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: eta_seconds
  {
    out << "eta_seconds: ";
    rosidl_generator_traits::value_to_yaml(msg.eta_seconds, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetTaskStatus_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }

  // member: eta_seconds
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "eta_seconds: ";
    rosidl_generator_traits::value_to_yaml(msg.eta_seconds, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetTaskStatus_Response & msg, bool use_flow_style = false)
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
  const parking_robot_interfaces::srv::GetTaskStatus_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  parking_robot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use parking_robot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const parking_robot_interfaces::srv::GetTaskStatus_Response & msg)
{
  return parking_robot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<parking_robot_interfaces::srv::GetTaskStatus_Response>()
{
  return "parking_robot_interfaces::srv::GetTaskStatus_Response";
}

template<>
inline const char * name<parking_robot_interfaces::srv::GetTaskStatus_Response>()
{
  return "parking_robot_interfaces/srv/GetTaskStatus_Response";
}

template<>
struct has_fixed_size<parking_robot_interfaces::srv::GetTaskStatus_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<parking_robot_interfaces::srv::GetTaskStatus_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<parking_robot_interfaces::srv::GetTaskStatus_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<parking_robot_interfaces::srv::GetTaskStatus>()
{
  return "parking_robot_interfaces::srv::GetTaskStatus";
}

template<>
inline const char * name<parking_robot_interfaces::srv::GetTaskStatus>()
{
  return "parking_robot_interfaces/srv/GetTaskStatus";
}

template<>
struct has_fixed_size<parking_robot_interfaces::srv::GetTaskStatus>
  : std::integral_constant<
    bool,
    has_fixed_size<parking_robot_interfaces::srv::GetTaskStatus_Request>::value &&
    has_fixed_size<parking_robot_interfaces::srv::GetTaskStatus_Response>::value
  >
{
};

template<>
struct has_bounded_size<parking_robot_interfaces::srv::GetTaskStatus>
  : std::integral_constant<
    bool,
    has_bounded_size<parking_robot_interfaces::srv::GetTaskStatus_Request>::value &&
    has_bounded_size<parking_robot_interfaces::srv::GetTaskStatus_Response>::value
  >
{
};

template<>
struct is_service<parking_robot_interfaces::srv::GetTaskStatus>
  : std::true_type
{
};

template<>
struct is_service_request<parking_robot_interfaces::srv::GetTaskStatus_Request>
  : std::true_type
{
};

template<>
struct is_service_response<parking_robot_interfaces::srv::GetTaskStatus_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PARKING_ROBOT_INTERFACES__SRV__DETAIL__GET_TASK_STATUS__TRAITS_HPP_
