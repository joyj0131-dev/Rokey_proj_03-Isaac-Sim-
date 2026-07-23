// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from parking_robot_interfaces:srv/ParkInSlot.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__SRV__DETAIL__PARK_IN_SLOT__STRUCT_H_
#define PARKING_ROBOT_INTERFACES__SRV__DETAIL__PARK_IN_SLOT__STRUCT_H_

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

/// Struct defined in srv/ParkInSlot in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__srv__ParkInSlot_Request
{
  rosidl_runtime_c__String slot_id;
} parking_robot_interfaces__srv__ParkInSlot_Request;

// Struct for a sequence of parking_robot_interfaces__srv__ParkInSlot_Request.
typedef struct parking_robot_interfaces__srv__ParkInSlot_Request__Sequence
{
  parking_robot_interfaces__srv__ParkInSlot_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__srv__ParkInSlot_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'task_id'
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ParkInSlot in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__srv__ParkInSlot_Response
{
  bool accepted;
  rosidl_runtime_c__String task_id;
  rosidl_runtime_c__String message;
} parking_robot_interfaces__srv__ParkInSlot_Response;

// Struct for a sequence of parking_robot_interfaces__srv__ParkInSlot_Response.
typedef struct parking_robot_interfaces__srv__ParkInSlot_Response__Sequence
{
  parking_robot_interfaces__srv__ParkInSlot_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__srv__ParkInSlot_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PARKING_ROBOT_INTERFACES__SRV__DETAIL__PARK_IN_SLOT__STRUCT_H_
