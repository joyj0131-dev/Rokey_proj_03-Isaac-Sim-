// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from parking_robot_interfaces:srv/FindEmptySlot.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__SRV__DETAIL__FIND_EMPTY_SLOT__STRUCT_H_
#define PARKING_ROBOT_INTERFACES__SRV__DETAIL__FIND_EMPTY_SLOT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/FindEmptySlot in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__srv__FindEmptySlot_Request
{
  double vehicle_length;
  double vehicle_width;
} parking_robot_interfaces__srv__FindEmptySlot_Request;

// Struct for a sequence of parking_robot_interfaces__srv__FindEmptySlot_Request.
typedef struct parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence
{
  parking_robot_interfaces__srv__FindEmptySlot_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'slot_id'
#include "rosidl_runtime_c/string.h"
// Member 'slot_pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in srv/FindEmptySlot in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__srv__FindEmptySlot_Response
{
  bool success;
  rosidl_runtime_c__String slot_id;
  geometry_msgs__msg__Pose slot_pose;
} parking_robot_interfaces__srv__FindEmptySlot_Response;

// Struct for a sequence of parking_robot_interfaces__srv__FindEmptySlot_Response.
typedef struct parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence
{
  parking_robot_interfaces__srv__FindEmptySlot_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PARKING_ROBOT_INTERFACES__SRV__DETAIL__FIND_EMPTY_SLOT__STRUCT_H_
