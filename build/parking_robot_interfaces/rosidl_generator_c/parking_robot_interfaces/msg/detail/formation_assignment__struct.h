// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from parking_robot_interfaces:msg/FormationAssignment.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__MSG__DETAIL__FORMATION_ASSIGNMENT__STRUCT_H_
#define PARKING_ROBOT_INTERFACES__MSG__DETAIL__FORMATION_ASSIGNMENT__STRUCT_H_

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
// Member 'role'
// Member 'partner_robot_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/FormationAssignment in the package parking_robot_interfaces.
/**
  * [초안, Team A 2026-07-21] task_dispatcher -> 로봇별 formation_gap_controller 배정.
  * formation_assignment 토픽(공용, 네임스페이스 없음)에서 사용한다.
  *
  * formation_gap_controller는 로봇마다 하나씩 항상 떠 있는 상시 노드다(실제
  * 로봇의 온보드 소프트웨어처럼). 작업이 없을 때는 idle 상태로 아무것도
  * 발행하지 않고 대기하다가, 이 메시지로 자기 robot_id에 해당하는 배정이
  * 오면 그 순간부터 active=true인 동안만 간격유지/공동정지 로직을 켠다.
  *
  * 파트너 odom 토픽 이름은 "/<partner_robot_id>/odom" 컨벤션을 그대로
  * 따른다고 가정한다 — 로봇마다 자기 네임스페이스 아래 odom/cmd_vel을
  * 갖는다는 게 이 프로젝트의 다중로봇 토픽 규칙이다.
 */
typedef struct parking_robot_interfaces__msg__FormationAssignment
{
  /// 이 배정의 대상 로봇
  rosidl_runtime_c__String robot_id;
  /// active=false일 때는 의미 없음
  rosidl_runtime_c__String task_id;
  /// leader | follower
  rosidl_runtime_c__String role;
  rosidl_runtime_c__String partner_robot_id;
  /// false면 이 로봇은 idle로 복귀
  bool active;
} parking_robot_interfaces__msg__FormationAssignment;

// Struct for a sequence of parking_robot_interfaces__msg__FormationAssignment.
typedef struct parking_robot_interfaces__msg__FormationAssignment__Sequence
{
  parking_robot_interfaces__msg__FormationAssignment * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__msg__FormationAssignment__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PARKING_ROBOT_INTERFACES__MSG__DETAIL__FORMATION_ASSIGNMENT__STRUCT_H_
