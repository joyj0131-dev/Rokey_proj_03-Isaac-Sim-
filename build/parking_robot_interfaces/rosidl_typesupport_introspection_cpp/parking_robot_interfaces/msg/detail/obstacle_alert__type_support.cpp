// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from parking_robot_interfaces:msg/ObstacleAlert.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "parking_robot_interfaces/msg/detail/obstacle_alert__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace parking_robot_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ObstacleAlert_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) parking_robot_interfaces::msg::ObstacleAlert(_init);
}

void ObstacleAlert_fini_function(void * message_memory)
{
  auto typed_message = static_cast<parking_robot_interfaces::msg::ObstacleAlert *>(message_memory);
  typed_message->~ObstacleAlert();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ObstacleAlert_message_member_array[3] = {
  {
    "obstacle_detected",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(parking_robot_interfaces::msg::ObstacleAlert, obstacle_detected),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "description",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(parking_robot_interfaces::msg::ObstacleAlert, description),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "location",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Point>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(parking_robot_interfaces::msg::ObstacleAlert, location),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ObstacleAlert_message_members = {
  "parking_robot_interfaces::msg",  // message namespace
  "ObstacleAlert",  // message name
  3,  // number of fields
  sizeof(parking_robot_interfaces::msg::ObstacleAlert),
  ObstacleAlert_message_member_array,  // message members
  ObstacleAlert_init_function,  // function to initialize message memory (memory has to be allocated)
  ObstacleAlert_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ObstacleAlert_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ObstacleAlert_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace parking_robot_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<parking_robot_interfaces::msg::ObstacleAlert>()
{
  return &::parking_robot_interfaces::msg::rosidl_typesupport_introspection_cpp::ObstacleAlert_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, parking_robot_interfaces, msg, ObstacleAlert)() {
  return &::parking_robot_interfaces::msg::rosidl_typesupport_introspection_cpp::ObstacleAlert_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
