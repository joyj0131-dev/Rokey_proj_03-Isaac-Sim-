// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from parking_robot_interfaces:action/CarryToSlot.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__ACTION__DETAIL__CARRY_TO_SLOT__TRAITS_HPP_
#define PARKING_ROBOT_INTERFACES__ACTION__DETAIL__CARRY_TO_SLOT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "parking_robot_interfaces/action/detail/carry_to_slot__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace parking_robot_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const CarryToSlot_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: lead_robot_id
  {
    out << "lead_robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.lead_robot_id, out);
    out << ", ";
  }

  // member: follow_robot_id
  {
    out << "follow_robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.follow_robot_id, out);
    out << ", ";
  }

  // member: target_x
  {
    out << "target_x: ";
    rosidl_generator_traits::value_to_yaml(msg.target_x, out);
    out << ", ";
  }

  // member: target_z
  {
    out << "target_z: ";
    rosidl_generator_traits::value_to_yaml(msg.target_z, out);
    out << ", ";
  }

  // member: target_yaw_deg
  {
    out << "target_yaw_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.target_yaw_deg, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CarryToSlot_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: lead_robot_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lead_robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.lead_robot_id, out);
    out << "\n";
  }

  // member: follow_robot_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "follow_robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.follow_robot_id, out);
    out << "\n";
  }

  // member: target_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_x: ";
    rosidl_generator_traits::value_to_yaml(msg.target_x, out);
    out << "\n";
  }

  // member: target_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_z: ";
    rosidl_generator_traits::value_to_yaml(msg.target_z, out);
    out << "\n";
  }

  // member: target_yaw_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_yaw_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.target_yaw_deg, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CarryToSlot_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace parking_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use parking_robot_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const parking_robot_interfaces::action::CarryToSlot_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  parking_robot_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use parking_robot_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const parking_robot_interfaces::action::CarryToSlot_Goal & msg)
{
  return parking_robot_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<parking_robot_interfaces::action::CarryToSlot_Goal>()
{
  return "parking_robot_interfaces::action::CarryToSlot_Goal";
}

template<>
inline const char * name<parking_robot_interfaces::action::CarryToSlot_Goal>()
{
  return "parking_robot_interfaces/action/CarryToSlot_Goal";
}

template<>
struct has_fixed_size<parking_robot_interfaces::action::CarryToSlot_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<parking_robot_interfaces::action::CarryToSlot_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<parking_robot_interfaces::action::CarryToSlot_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace parking_robot_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const CarryToSlot_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << ", ";
  }

  // member: final_x
  {
    out << "final_x: ";
    rosidl_generator_traits::value_to_yaml(msg.final_x, out);
    out << ", ";
  }

  // member: final_z
  {
    out << "final_z: ";
    rosidl_generator_traits::value_to_yaml(msg.final_z, out);
    out << ", ";
  }

  // member: final_yaw_deg
  {
    out << "final_yaw_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.final_yaw_deg, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CarryToSlot_Result & msg,
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

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }

  // member: final_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_x: ";
    rosidl_generator_traits::value_to_yaml(msg.final_x, out);
    out << "\n";
  }

  // member: final_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_z: ";
    rosidl_generator_traits::value_to_yaml(msg.final_z, out);
    out << "\n";
  }

  // member: final_yaw_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_yaw_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.final_yaw_deg, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CarryToSlot_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace parking_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use parking_robot_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const parking_robot_interfaces::action::CarryToSlot_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  parking_robot_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use parking_robot_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const parking_robot_interfaces::action::CarryToSlot_Result & msg)
{
  return parking_robot_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<parking_robot_interfaces::action::CarryToSlot_Result>()
{
  return "parking_robot_interfaces::action::CarryToSlot_Result";
}

template<>
inline const char * name<parking_robot_interfaces::action::CarryToSlot_Result>()
{
  return "parking_robot_interfaces/action/CarryToSlot_Result";
}

template<>
struct has_fixed_size<parking_robot_interfaces::action::CarryToSlot_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<parking_robot_interfaces::action::CarryToSlot_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<parking_robot_interfaces::action::CarryToSlot_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace parking_robot_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const CarryToSlot_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: phase
  {
    out << "phase: ";
    rosidl_generator_traits::value_to_yaml(msg.phase, out);
    out << ", ";
  }

  // member: dist_remaining
  {
    out << "dist_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.dist_remaining, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CarryToSlot_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: phase
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "phase: ";
    rosidl_generator_traits::value_to_yaml(msg.phase, out);
    out << "\n";
  }

  // member: dist_remaining
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dist_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.dist_remaining, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CarryToSlot_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace parking_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use parking_robot_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const parking_robot_interfaces::action::CarryToSlot_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  parking_robot_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use parking_robot_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const parking_robot_interfaces::action::CarryToSlot_Feedback & msg)
{
  return parking_robot_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<parking_robot_interfaces::action::CarryToSlot_Feedback>()
{
  return "parking_robot_interfaces::action::CarryToSlot_Feedback";
}

template<>
inline const char * name<parking_robot_interfaces::action::CarryToSlot_Feedback>()
{
  return "parking_robot_interfaces/action/CarryToSlot_Feedback";
}

template<>
struct has_fixed_size<parking_robot_interfaces::action::CarryToSlot_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<parking_robot_interfaces::action::CarryToSlot_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<parking_robot_interfaces::action::CarryToSlot_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "parking_robot_interfaces/action/detail/carry_to_slot__traits.hpp"

namespace parking_robot_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const CarryToSlot_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CarryToSlot_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CarryToSlot_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace parking_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use parking_robot_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const parking_robot_interfaces::action::CarryToSlot_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  parking_robot_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use parking_robot_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const parking_robot_interfaces::action::CarryToSlot_SendGoal_Request & msg)
{
  return parking_robot_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<parking_robot_interfaces::action::CarryToSlot_SendGoal_Request>()
{
  return "parking_robot_interfaces::action::CarryToSlot_SendGoal_Request";
}

template<>
inline const char * name<parking_robot_interfaces::action::CarryToSlot_SendGoal_Request>()
{
  return "parking_robot_interfaces/action/CarryToSlot_SendGoal_Request";
}

template<>
struct has_fixed_size<parking_robot_interfaces::action::CarryToSlot_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<parking_robot_interfaces::action::CarryToSlot_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<parking_robot_interfaces::action::CarryToSlot_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<parking_robot_interfaces::action::CarryToSlot_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<parking_robot_interfaces::action::CarryToSlot_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace parking_robot_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const CarryToSlot_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CarryToSlot_SendGoal_Response & msg,
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

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CarryToSlot_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace parking_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use parking_robot_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const parking_robot_interfaces::action::CarryToSlot_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  parking_robot_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use parking_robot_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const parking_robot_interfaces::action::CarryToSlot_SendGoal_Response & msg)
{
  return parking_robot_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<parking_robot_interfaces::action::CarryToSlot_SendGoal_Response>()
{
  return "parking_robot_interfaces::action::CarryToSlot_SendGoal_Response";
}

template<>
inline const char * name<parking_robot_interfaces::action::CarryToSlot_SendGoal_Response>()
{
  return "parking_robot_interfaces/action/CarryToSlot_SendGoal_Response";
}

template<>
struct has_fixed_size<parking_robot_interfaces::action::CarryToSlot_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<parking_robot_interfaces::action::CarryToSlot_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<parking_robot_interfaces::action::CarryToSlot_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<parking_robot_interfaces::action::CarryToSlot_SendGoal>()
{
  return "parking_robot_interfaces::action::CarryToSlot_SendGoal";
}

template<>
inline const char * name<parking_robot_interfaces::action::CarryToSlot_SendGoal>()
{
  return "parking_robot_interfaces/action/CarryToSlot_SendGoal";
}

template<>
struct has_fixed_size<parking_robot_interfaces::action::CarryToSlot_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<parking_robot_interfaces::action::CarryToSlot_SendGoal_Request>::value &&
    has_fixed_size<parking_robot_interfaces::action::CarryToSlot_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<parking_robot_interfaces::action::CarryToSlot_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<parking_robot_interfaces::action::CarryToSlot_SendGoal_Request>::value &&
    has_bounded_size<parking_robot_interfaces::action::CarryToSlot_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<parking_robot_interfaces::action::CarryToSlot_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<parking_robot_interfaces::action::CarryToSlot_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<parking_robot_interfaces::action::CarryToSlot_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace parking_robot_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const CarryToSlot_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CarryToSlot_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CarryToSlot_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace parking_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use parking_robot_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const parking_robot_interfaces::action::CarryToSlot_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  parking_robot_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use parking_robot_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const parking_robot_interfaces::action::CarryToSlot_GetResult_Request & msg)
{
  return parking_robot_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<parking_robot_interfaces::action::CarryToSlot_GetResult_Request>()
{
  return "parking_robot_interfaces::action::CarryToSlot_GetResult_Request";
}

template<>
inline const char * name<parking_robot_interfaces::action::CarryToSlot_GetResult_Request>()
{
  return "parking_robot_interfaces/action/CarryToSlot_GetResult_Request";
}

template<>
struct has_fixed_size<parking_robot_interfaces::action::CarryToSlot_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<parking_robot_interfaces::action::CarryToSlot_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<parking_robot_interfaces::action::CarryToSlot_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "parking_robot_interfaces/action/detail/carry_to_slot__traits.hpp"

namespace parking_robot_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const CarryToSlot_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CarryToSlot_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CarryToSlot_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace parking_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use parking_robot_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const parking_robot_interfaces::action::CarryToSlot_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  parking_robot_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use parking_robot_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const parking_robot_interfaces::action::CarryToSlot_GetResult_Response & msg)
{
  return parking_robot_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<parking_robot_interfaces::action::CarryToSlot_GetResult_Response>()
{
  return "parking_robot_interfaces::action::CarryToSlot_GetResult_Response";
}

template<>
inline const char * name<parking_robot_interfaces::action::CarryToSlot_GetResult_Response>()
{
  return "parking_robot_interfaces/action/CarryToSlot_GetResult_Response";
}

template<>
struct has_fixed_size<parking_robot_interfaces::action::CarryToSlot_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<parking_robot_interfaces::action::CarryToSlot_Result>::value> {};

template<>
struct has_bounded_size<parking_robot_interfaces::action::CarryToSlot_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<parking_robot_interfaces::action::CarryToSlot_Result>::value> {};

template<>
struct is_message<parking_robot_interfaces::action::CarryToSlot_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<parking_robot_interfaces::action::CarryToSlot_GetResult>()
{
  return "parking_robot_interfaces::action::CarryToSlot_GetResult";
}

template<>
inline const char * name<parking_robot_interfaces::action::CarryToSlot_GetResult>()
{
  return "parking_robot_interfaces/action/CarryToSlot_GetResult";
}

template<>
struct has_fixed_size<parking_robot_interfaces::action::CarryToSlot_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<parking_robot_interfaces::action::CarryToSlot_GetResult_Request>::value &&
    has_fixed_size<parking_robot_interfaces::action::CarryToSlot_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<parking_robot_interfaces::action::CarryToSlot_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<parking_robot_interfaces::action::CarryToSlot_GetResult_Request>::value &&
    has_bounded_size<parking_robot_interfaces::action::CarryToSlot_GetResult_Response>::value
  >
{
};

template<>
struct is_service<parking_robot_interfaces::action::CarryToSlot_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<parking_robot_interfaces::action::CarryToSlot_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<parking_robot_interfaces::action::CarryToSlot_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "parking_robot_interfaces/action/detail/carry_to_slot__traits.hpp"

namespace parking_robot_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const CarryToSlot_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CarryToSlot_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CarryToSlot_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace parking_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use parking_robot_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const parking_robot_interfaces::action::CarryToSlot_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  parking_robot_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use parking_robot_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const parking_robot_interfaces::action::CarryToSlot_FeedbackMessage & msg)
{
  return parking_robot_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<parking_robot_interfaces::action::CarryToSlot_FeedbackMessage>()
{
  return "parking_robot_interfaces::action::CarryToSlot_FeedbackMessage";
}

template<>
inline const char * name<parking_robot_interfaces::action::CarryToSlot_FeedbackMessage>()
{
  return "parking_robot_interfaces/action/CarryToSlot_FeedbackMessage";
}

template<>
struct has_fixed_size<parking_robot_interfaces::action::CarryToSlot_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<parking_robot_interfaces::action::CarryToSlot_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<parking_robot_interfaces::action::CarryToSlot_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<parking_robot_interfaces::action::CarryToSlot_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<parking_robot_interfaces::action::CarryToSlot_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<parking_robot_interfaces::action::CarryToSlot>
  : std::true_type
{
};

template<>
struct is_action_goal<parking_robot_interfaces::action::CarryToSlot_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<parking_robot_interfaces::action::CarryToSlot_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<parking_robot_interfaces::action::CarryToSlot_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // PARKING_ROBOT_INTERFACES__ACTION__DETAIL__CARRY_TO_SLOT__TRAITS_HPP_
