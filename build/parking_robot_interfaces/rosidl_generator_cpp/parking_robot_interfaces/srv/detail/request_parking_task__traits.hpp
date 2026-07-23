// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from parking_robot_interfaces:srv/RequestParkingTask.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__SRV__DETAIL__REQUEST_PARKING_TASK__TRAITS_HPP_
#define PARKING_ROBOT_INTERFACES__SRV__DETAIL__REQUEST_PARKING_TASK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "parking_robot_interfaces/srv/detail/request_parking_task__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace parking_robot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const RequestParkingTask_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: request_type
  {
    out << "request_type: ";
    rosidl_generator_traits::value_to_yaml(msg.request_type, out);
    out << ", ";
  }

  // member: vehicle_id
  {
    out << "vehicle_id: ";
    rosidl_generator_traits::value_to_yaml(msg.vehicle_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RequestParkingTask_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: request_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "request_type: ";
    rosidl_generator_traits::value_to_yaml(msg.request_type, out);
    out << "\n";
  }

  // member: vehicle_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vehicle_id: ";
    rosidl_generator_traits::value_to_yaml(msg.vehicle_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RequestParkingTask_Request & msg, bool use_flow_style = false)
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
  const parking_robot_interfaces::srv::RequestParkingTask_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  parking_robot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use parking_robot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const parking_robot_interfaces::srv::RequestParkingTask_Request & msg)
{
  return parking_robot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<parking_robot_interfaces::srv::RequestParkingTask_Request>()
{
  return "parking_robot_interfaces::srv::RequestParkingTask_Request";
}

template<>
inline const char * name<parking_robot_interfaces::srv::RequestParkingTask_Request>()
{
  return "parking_robot_interfaces/srv/RequestParkingTask_Request";
}

template<>
struct has_fixed_size<parking_robot_interfaces::srv::RequestParkingTask_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<parking_robot_interfaces::srv::RequestParkingTask_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<parking_robot_interfaces::srv::RequestParkingTask_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace parking_robot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const RequestParkingTask_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: task_id
  {
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
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
  const RequestParkingTask_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
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

inline std::string to_yaml(const RequestParkingTask_Response & msg, bool use_flow_style = false)
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
  const parking_robot_interfaces::srv::RequestParkingTask_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  parking_robot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use parking_robot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const parking_robot_interfaces::srv::RequestParkingTask_Response & msg)
{
  return parking_robot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<parking_robot_interfaces::srv::RequestParkingTask_Response>()
{
  return "parking_robot_interfaces::srv::RequestParkingTask_Response";
}

template<>
inline const char * name<parking_robot_interfaces::srv::RequestParkingTask_Response>()
{
  return "parking_robot_interfaces/srv/RequestParkingTask_Response";
}

template<>
struct has_fixed_size<parking_robot_interfaces::srv::RequestParkingTask_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<parking_robot_interfaces::srv::RequestParkingTask_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<parking_robot_interfaces::srv::RequestParkingTask_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<parking_robot_interfaces::srv::RequestParkingTask>()
{
  return "parking_robot_interfaces::srv::RequestParkingTask";
}

template<>
inline const char * name<parking_robot_interfaces::srv::RequestParkingTask>()
{
  return "parking_robot_interfaces/srv/RequestParkingTask";
}

template<>
struct has_fixed_size<parking_robot_interfaces::srv::RequestParkingTask>
  : std::integral_constant<
    bool,
    has_fixed_size<parking_robot_interfaces::srv::RequestParkingTask_Request>::value &&
    has_fixed_size<parking_robot_interfaces::srv::RequestParkingTask_Response>::value
  >
{
};

template<>
struct has_bounded_size<parking_robot_interfaces::srv::RequestParkingTask>
  : std::integral_constant<
    bool,
    has_bounded_size<parking_robot_interfaces::srv::RequestParkingTask_Request>::value &&
    has_bounded_size<parking_robot_interfaces::srv::RequestParkingTask_Response>::value
  >
{
};

template<>
struct is_service<parking_robot_interfaces::srv::RequestParkingTask>
  : std::true_type
{
};

template<>
struct is_service_request<parking_robot_interfaces::srv::RequestParkingTask_Request>
  : std::true_type
{
};

template<>
struct is_service_response<parking_robot_interfaces::srv::RequestParkingTask_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PARKING_ROBOT_INTERFACES__SRV__DETAIL__REQUEST_PARKING_TASK__TRAITS_HPP_
