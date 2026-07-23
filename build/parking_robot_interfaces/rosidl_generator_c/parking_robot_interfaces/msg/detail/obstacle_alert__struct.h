// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from parking_robot_interfaces:msg/ObstacleAlert.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__MSG__DETAIL__OBSTACLE_ALERT__STRUCT_H_
#define PARKING_ROBOT_INTERFACES__MSG__DETAIL__OBSTACLE_ALERT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'description'
#include "rosidl_runtime_c/string.h"
// Member 'location'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/ObstacleAlert in the package parking_robot_interfaces.
/**
  * safety_monitor가 발행하는 긴급정지 신호. obstacle_alert 토픽에서 사용.
 */
typedef struct parking_robot_interfaces__msg__ObstacleAlert
{
  bool obstacle_detected;
  rosidl_runtime_c__String description;
  geometry_msgs__msg__Point location;
} parking_robot_interfaces__msg__ObstacleAlert;

// Struct for a sequence of parking_robot_interfaces__msg__ObstacleAlert.
typedef struct parking_robot_interfaces__msg__ObstacleAlert__Sequence
{
  parking_robot_interfaces__msg__ObstacleAlert * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__msg__ObstacleAlert__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PARKING_ROBOT_INTERFACES__MSG__DETAIL__OBSTACLE_ALERT__STRUCT_H_
