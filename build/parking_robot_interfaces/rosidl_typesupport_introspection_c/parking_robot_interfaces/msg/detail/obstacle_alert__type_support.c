// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from parking_robot_interfaces:msg/ObstacleAlert.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "parking_robot_interfaces/msg/detail/obstacle_alert__rosidl_typesupport_introspection_c.h"
#include "parking_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "parking_robot_interfaces/msg/detail/obstacle_alert__functions.h"
#include "parking_robot_interfaces/msg/detail/obstacle_alert__struct.h"


// Include directives for member types
// Member `description`
#include "rosidl_runtime_c/string_functions.h"
// Member `location`
#include "geometry_msgs/msg/point.h"
// Member `location`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void parking_robot_interfaces__msg__ObstacleAlert__rosidl_typesupport_introspection_c__ObstacleAlert_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  parking_robot_interfaces__msg__ObstacleAlert__init(message_memory);
}

void parking_robot_interfaces__msg__ObstacleAlert__rosidl_typesupport_introspection_c__ObstacleAlert_fini_function(void * message_memory)
{
  parking_robot_interfaces__msg__ObstacleAlert__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember parking_robot_interfaces__msg__ObstacleAlert__rosidl_typesupport_introspection_c__ObstacleAlert_message_member_array[3] = {
  {
    "obstacle_detected",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(parking_robot_interfaces__msg__ObstacleAlert, obstacle_detected),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "description",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(parking_robot_interfaces__msg__ObstacleAlert, description),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "location",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(parking_robot_interfaces__msg__ObstacleAlert, location),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers parking_robot_interfaces__msg__ObstacleAlert__rosidl_typesupport_introspection_c__ObstacleAlert_message_members = {
  "parking_robot_interfaces__msg",  // message namespace
  "ObstacleAlert",  // message name
  3,  // number of fields
  sizeof(parking_robot_interfaces__msg__ObstacleAlert),
  parking_robot_interfaces__msg__ObstacleAlert__rosidl_typesupport_introspection_c__ObstacleAlert_message_member_array,  // message members
  parking_robot_interfaces__msg__ObstacleAlert__rosidl_typesupport_introspection_c__ObstacleAlert_init_function,  // function to initialize message memory (memory has to be allocated)
  parking_robot_interfaces__msg__ObstacleAlert__rosidl_typesupport_introspection_c__ObstacleAlert_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t parking_robot_interfaces__msg__ObstacleAlert__rosidl_typesupport_introspection_c__ObstacleAlert_message_type_support_handle = {
  0,
  &parking_robot_interfaces__msg__ObstacleAlert__rosidl_typesupport_introspection_c__ObstacleAlert_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_parking_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, parking_robot_interfaces, msg, ObstacleAlert)() {
  parking_robot_interfaces__msg__ObstacleAlert__rosidl_typesupport_introspection_c__ObstacleAlert_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!parking_robot_interfaces__msg__ObstacleAlert__rosidl_typesupport_introspection_c__ObstacleAlert_message_type_support_handle.typesupport_identifier) {
    parking_robot_interfaces__msg__ObstacleAlert__rosidl_typesupport_introspection_c__ObstacleAlert_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &parking_robot_interfaces__msg__ObstacleAlert__rosidl_typesupport_introspection_c__ObstacleAlert_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
