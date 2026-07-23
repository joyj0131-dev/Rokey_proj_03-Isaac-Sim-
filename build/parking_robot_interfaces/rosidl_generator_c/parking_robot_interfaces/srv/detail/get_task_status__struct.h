// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from parking_robot_interfaces:srv/GetTaskStatus.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__SRV__DETAIL__GET_TASK_STATUS__STRUCT_H_
#define PARKING_ROBOT_INTERFACES__SRV__DETAIL__GET_TASK_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'task_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetTaskStatus in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__srv__GetTaskStatus_Request
{
  rosidl_runtime_c__String task_id;
} parking_robot_interfaces__srv__GetTaskStatus_Request;

// Struct for a sequence of parking_robot_interfaces__srv__GetTaskStatus_Request.
typedef struct parking_robot_interfaces__srv__GetTaskStatus_Request__Sequence
{
  parking_robot_interfaces__srv__GetTaskStatus_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__srv__GetTaskStatus_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'state'
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetTaskStatus in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__srv__GetTaskStatus_Response
{
  /// WAITING, PROCESSING, DONE, FAILED
  rosidl_runtime_c__String state;
  int32_t eta_seconds;
  rosidl_runtime_c__String message;
} parking_robot_interfaces__srv__GetTaskStatus_Response;

// Struct for a sequence of parking_robot_interfaces__srv__GetTaskStatus_Response.
typedef struct parking_robot_interfaces__srv__GetTaskStatus_Response__Sequence
{
  parking_robot_interfaces__srv__GetTaskStatus_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__srv__GetTaskStatus_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PARKING_ROBOT_INTERFACES__SRV__DETAIL__GET_TASK_STATUS__STRUCT_H_
