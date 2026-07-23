// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from parking_robot_interfaces:msg/VehicleInfo.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__MSG__DETAIL__VEHICLE_INFO__STRUCT_H_
#define PARKING_ROBOT_INTERFACES__MSG__DETAIL__VEHICLE_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/VehicleInfo in the package parking_robot_interfaces.
/**
  * vehicle_detection_node가 인식한 차량 정보. 담당자 확정 전 임시 정의.
 */
typedef struct parking_robot_interfaces__msg__VehicleInfo
{
  geometry_msgs__msg__Pose pose;
  double length;
  double width;
  double height;
} parking_robot_interfaces__msg__VehicleInfo;

// Struct for a sequence of parking_robot_interfaces__msg__VehicleInfo.
typedef struct parking_robot_interfaces__msg__VehicleInfo__Sequence
{
  parking_robot_interfaces__msg__VehicleInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__msg__VehicleInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PARKING_ROBOT_INTERFACES__MSG__DETAIL__VEHICLE_INFO__STRUCT_H_
