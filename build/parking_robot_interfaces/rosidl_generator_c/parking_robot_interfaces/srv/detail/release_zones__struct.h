// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from parking_robot_interfaces:srv/ReleaseZones.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__SRV__DETAIL__RELEASE_ZONES__STRUCT_H_
#define PARKING_ROBOT_INTERFACES__SRV__DETAIL__RELEASE_ZONES__STRUCT_H_

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

/// Struct defined in srv/ReleaseZones in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__srv__ReleaseZones_Request
{
  rosidl_runtime_c__String robot_id;
  rosidl_runtime_c__String task_id;
  rosidl_runtime_c__String__Sequence zone_ids;
} parking_robot_interfaces__srv__ReleaseZones_Request;

// Struct for a sequence of parking_robot_interfaces__srv__ReleaseZones_Request.
typedef struct parking_robot_interfaces__srv__ReleaseZones_Request__Sequence
{
  parking_robot_interfaces__srv__ReleaseZones_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__srv__ReleaseZones_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ReleaseZones in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__srv__ReleaseZones_Response
{
  bool success;
} parking_robot_interfaces__srv__ReleaseZones_Response;

// Struct for a sequence of parking_robot_interfaces__srv__ReleaseZones_Response.
typedef struct parking_robot_interfaces__srv__ReleaseZones_Response__Sequence
{
  parking_robot_interfaces__srv__ReleaseZones_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__srv__ReleaseZones_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PARKING_ROBOT_INTERFACES__SRV__DETAIL__RELEASE_ZONES__STRUCT_H_
