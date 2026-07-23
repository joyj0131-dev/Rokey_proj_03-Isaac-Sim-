// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from parking_robot_interfaces:msg/VehicleInfo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "parking_robot_interfaces/msg/detail/vehicle_info__rosidl_typesupport_introspection_c.h"
#include "parking_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "parking_robot_interfaces/msg/detail/vehicle_info__functions.h"
#include "parking_robot_interfaces/msg/detail/vehicle_info__struct.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/pose.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void parking_robot_interfaces__msg__VehicleInfo__rosidl_typesupport_introspection_c__VehicleInfo_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  parking_robot_interfaces__msg__VehicleInfo__init(message_memory);
}

void parking_robot_interfaces__msg__VehicleInfo__rosidl_typesupport_introspection_c__VehicleInfo_fini_function(void * message_memory)
{
  parking_robot_interfaces__msg__VehicleInfo__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember parking_robot_interfaces__msg__VehicleInfo__rosidl_typesupport_introspection_c__VehicleInfo_message_member_array[4] = {
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(parking_robot_interfaces__msg__VehicleInfo, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "length",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(parking_robot_interfaces__msg__VehicleInfo, length),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "width",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(parking_robot_interfaces__msg__VehicleInfo, width),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "height",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(parking_robot_interfaces__msg__VehicleInfo, height),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers parking_robot_interfaces__msg__VehicleInfo__rosidl_typesupport_introspection_c__VehicleInfo_message_members = {
  "parking_robot_interfaces__msg",  // message namespace
  "VehicleInfo",  // message name
  4,  // number of fields
  sizeof(parking_robot_interfaces__msg__VehicleInfo),
  parking_robot_interfaces__msg__VehicleInfo__rosidl_typesupport_introspection_c__VehicleInfo_message_member_array,  // message members
  parking_robot_interfaces__msg__VehicleInfo__rosidl_typesupport_introspection_c__VehicleInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  parking_robot_interfaces__msg__VehicleInfo__rosidl_typesupport_introspection_c__VehicleInfo_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t parking_robot_interfaces__msg__VehicleInfo__rosidl_typesupport_introspection_c__VehicleInfo_message_type_support_handle = {
  0,
  &parking_robot_interfaces__msg__VehicleInfo__rosidl_typesupport_introspection_c__VehicleInfo_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_parking_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, parking_robot_interfaces, msg, VehicleInfo)() {
  parking_robot_interfaces__msg__VehicleInfo__rosidl_typesupport_introspection_c__VehicleInfo_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!parking_robot_interfaces__msg__VehicleInfo__rosidl_typesupport_introspection_c__VehicleInfo_message_type_support_handle.typesupport_identifier) {
    parking_robot_interfaces__msg__VehicleInfo__rosidl_typesupport_introspection_c__VehicleInfo_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &parking_robot_interfaces__msg__VehicleInfo__rosidl_typesupport_introspection_c__VehicleInfo_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
