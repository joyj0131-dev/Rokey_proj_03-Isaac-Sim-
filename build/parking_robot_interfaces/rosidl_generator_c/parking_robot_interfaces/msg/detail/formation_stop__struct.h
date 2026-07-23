// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from parking_robot_interfaces:msg/FormationStop.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__MSG__DETAIL__FORMATION_STOP__STRUCT_H_
#define PARKING_ROBOT_INTERFACES__MSG__DETAIL__FORMATION_STOP__STRUCT_H_

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
// Member 'source_robot_id'
// Member 'reason'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/FormationStop in the package parking_robot_interfaces.
/**
  * [초안, Team A 2026-07-21] 로봇 2대(leader/follower) 공동 정지 방송.
  * formation_stop 토픽(공용, 네임스페이스 없음)에서 사용한다.
  *
  * 배경: 차량 하나를 로봇 2대가 붙잡고 옮기는 중에는 "한쪽만 멈춤"이 가장
  * 위험하다(차가 뒤틀림). 어느 한쪽이 정지해야 하는 상황(자기 이상 감지,
  * 파트너 신호 두절 등)이 생기면 이 메시지로 즉시 상대에게 알려서 같이
  * 멈추게 한다. task_id로 어느 팀(로봇 2대 편성)의 메시지인지 구분한다 —
  * 여러 팀이 동시에 작업 중이어도 서로 신호가 안 섞이도록.
 */
typedef struct parking_robot_interfaces__msg__FormationStop
{
  rosidl_runtime_c__String task_id;
  /// 이 신호를 보낸 로봇
  rosidl_runtime_c__String source_robot_id;
  bool stop;
  rosidl_runtime_c__String reason;
} parking_robot_interfaces__msg__FormationStop;

// Struct for a sequence of parking_robot_interfaces__msg__FormationStop.
typedef struct parking_robot_interfaces__msg__FormationStop__Sequence
{
  parking_robot_interfaces__msg__FormationStop * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__msg__FormationStop__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PARKING_ROBOT_INTERFACES__MSG__DETAIL__FORMATION_STOP__STRUCT_H_
