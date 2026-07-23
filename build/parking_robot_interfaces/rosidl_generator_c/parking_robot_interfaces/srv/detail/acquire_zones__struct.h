// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from parking_robot_interfaces:srv/AcquireZones.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__SRV__DETAIL__ACQUIRE_ZONES__STRUCT_H_
#define PARKING_ROBOT_INTERFACES__SRV__DETAIL__ACQUIRE_ZONES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'robot_id'
// Member 'task_id'
// Member 'zone_ids'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/AcquireZones in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__srv__AcquireZones_Request
{
  rosidl_runtime_c__String robot_id;
  rosidl_runtime_c__String task_id;
  rosidl_runtime_c__String__Sequence zone_ids;
} parking_robot_interfaces__srv__AcquireZones_Request;

// Struct for a sequence of parking_robot_interfaces__srv__AcquireZones_Request.
typedef struct parking_robot_interfaces__srv__AcquireZones_Request__Sequence
{
  parking_robot_interfaces__srv__AcquireZones_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__srv__AcquireZones_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'held_zones'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/AcquireZones in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__srv__AcquireZones_Response
{
  bool granted;
  /// 이 로봇이 현재 보유한 존 전체
  rosidl_runtime_c__String__Sequence held_zones;
  /// 거부 시 재시도 권고 간격
  float retry_after_sec;
} parking_robot_interfaces__srv__AcquireZones_Response;

// Struct for a sequence of parking_robot_interfaces__srv__AcquireZones_Response.
typedef struct parking_robot_interfaces__srv__AcquireZones_Response__Sequence
{
  parking_robot_interfaces__srv__AcquireZones_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__srv__AcquireZones_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PARKING_ROBOT_INTERFACES__SRV__DETAIL__ACQUIRE_ZONES__STRUCT_H_
