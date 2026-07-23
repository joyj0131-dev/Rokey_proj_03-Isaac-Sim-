// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from parking_robot_interfaces:srv/GetSlotInfo.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__SRV__DETAIL__GET_SLOT_INFO__STRUCT_H_
#define PARKING_ROBOT_INTERFACES__SRV__DETAIL__GET_SLOT_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'slot_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetSlotInfo in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__srv__GetSlotInfo_Request
{
  rosidl_runtime_c__String slot_id;
} parking_robot_interfaces__srv__GetSlotInfo_Request;

// Struct for a sequence of parking_robot_interfaces__srv__GetSlotInfo_Request.
typedef struct parking_robot_interfaces__srv__GetSlotInfo_Request__Sequence
{
  parking_robot_interfaces__srv__GetSlotInfo_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__srv__GetSlotInfo_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in srv/GetSlotInfo in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__srv__GetSlotInfo_Response
{
  /// false = /parking_slots 캐시 비어있음(runner 미기동)
  bool data_ready;
  bool exists;
  bool occupied;
  bool is_accessible;
  /// map 프레임, orientation = 목표 yaw(quaternion z/w)
  geometry_msgs__msg__Pose pose;
} parking_robot_interfaces__srv__GetSlotInfo_Response;

// Struct for a sequence of parking_robot_interfaces__srv__GetSlotInfo_Response.
typedef struct parking_robot_interfaces__srv__GetSlotInfo_Response__Sequence
{
  parking_robot_interfaces__srv__GetSlotInfo_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__srv__GetSlotInfo_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PARKING_ROBOT_INTERFACES__SRV__DETAIL__GET_SLOT_INFO__STRUCT_H_
