// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from parking_robot_interfaces:srv/AcquireZones.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__SRV__DETAIL__ACQUIRE_ZONES__TRAITS_HPP_
#define PARKING_ROBOT_INTERFACES__SRV__DETAIL__ACQUIRE_ZONES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "parking_robot_interfaces/srv/detail/acquire_zones__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace parking_robot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const AcquireZones_Request & msg,
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

  // member: zone_ids
  {
    if (msg.zone_ids.size() == 0) {
      out << "zone_ids: []";
    } else {
      out << "zone_ids: [";
      size_t pending_items = msg.zone_ids.size();
      for (auto item : msg.zone_ids) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AcquireZones_Request & msg,
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

  // member: zone_ids
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.zone_ids.size() == 0) {
      out << "zone_ids: []\n";
    } else {
      out << "zone_ids:\n";
      for (auto item : msg.zone_ids) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AcquireZones_Request & msg, bool use_flow_style = false)
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
  const parking_robot_interfaces::srv::AcquireZones_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  parking_robot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use parking_robot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const parking_robot_interfaces::srv::AcquireZones_Request & msg)
{
  return parking_robot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<parking_robot_interfaces::srv::AcquireZones_Request>()
{
  return "parking_robot_interfaces::srv::AcquireZones_Request";
}

template<>
inline const char * name<parking_robot_interfaces::srv::AcquireZones_Request>()
{
  return "parking_robot_interfaces/srv/AcquireZones_Request";
}

template<>
struct has_fixed_size<parking_robot_interfaces::srv::AcquireZones_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<parking_robot_interfaces::srv::AcquireZones_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<parking_robot_interfaces::srv::AcquireZones_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace parking_robot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const AcquireZones_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: granted
  {
    out << "granted: ";
    rosidl_generator_traits::value_to_yaml(msg.granted, out);
    out << ", ";
  }

  // member: held_zones
  {
    if (msg.held_zones.size() == 0) {
      out << "held_zones: []";
    } else {
      out << "held_zones: [";
      size_t pending_items = msg.held_zones.size();
      for (auto item : msg.held_zones) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: retry_after_sec
  {
    out << "retry_after_sec: ";
    rosidl_generator_traits::value_to_yaml(msg.retry_after_sec, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AcquireZones_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: granted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "granted: ";
    rosidl_generator_traits::value_to_yaml(msg.granted, out);
    out << "\n";
  }

  // member: held_zones
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.held_zones.size() == 0) {
      out << "held_zones: []\n";
    } else {
      out << "held_zones:\n";
      for (auto item : msg.held_zones) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: retry_after_sec
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "retry_after_sec: ";
    rosidl_generator_traits::value_to_yaml(msg.retry_after_sec, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AcquireZones_Response & msg, bool use_flow_style = false)
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
  const parking_robot_interfaces::srv::AcquireZones_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  parking_robot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use parking_robot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const parking_robot_interfaces::srv::AcquireZones_Response & msg)
{
  return parking_robot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<parking_robot_interfaces::srv::AcquireZones_Response>()
{
  return "parking_robot_interfaces::srv::AcquireZones_Response";
}

template<>
inline const char * name<parking_robot_interfaces::srv::AcquireZones_Response>()
{
  return "parking_robot_interfaces/srv/AcquireZones_Response";
}

template<>
struct has_fixed_size<parking_robot_interfaces::srv::AcquireZones_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<parking_robot_interfaces::srv::AcquireZones_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<parking_robot_interfaces::srv::AcquireZones_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<parking_robot_interfaces::srv::AcquireZones>()
{
  return "parking_robot_interfaces::srv::AcquireZones";
}

template<>
inline const char * name<parking_robot_interfaces::srv::AcquireZones>()
{
  return "parking_robot_interfaces/srv/AcquireZones";
}

template<>
struct has_fixed_size<parking_robot_interfaces::srv::AcquireZones>
  : std::integral_constant<
    bool,
    has_fixed_size<parking_robot_interfaces::srv::AcquireZones_Request>::value &&
    has_fixed_size<parking_robot_interfaces::srv::AcquireZones_Response>::value
  >
{
};

template<>
struct has_bounded_size<parking_robot_interfaces::srv::AcquireZones>
  : std::integral_constant<
    bool,
    has_bounded_size<parking_robot_interfaces::srv::AcquireZones_Request>::value &&
    has_bounded_size<parking_robot_interfaces::srv::AcquireZones_Response>::value
  >
{
};

template<>
struct is_service<parking_robot_interfaces::srv::AcquireZones>
  : std::true_type
{
};

template<>
struct is_service_request<parking_robot_interfaces::srv::AcquireZones_Request>
  : std::true_type
{
};

template<>
struct is_service_response<parking_robot_interfaces::srv::AcquireZones_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PARKING_ROBOT_INTERFACES__SRV__DETAIL__ACQUIRE_ZONES__TRAITS_HPP_
