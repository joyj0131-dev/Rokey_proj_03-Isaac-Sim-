// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from parking_robot_interfaces:action/IngressUnderTruck.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__ACTION__DETAIL__INGRESS_UNDER_TRUCK__STRUCT_H_
#define PARKING_ROBOT_INTERFACES__ACTION__DETAIL__INGRESS_UNDER_TRUCK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/IngressUnderTruck in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__IngressUnderTruck_Goal
{
  /// 0=첫 트로프(후축), 1=둘째 트로프(전축) -- axle_detector_node 의
  ///   /robot_<id>/axle_index 와 동일한 0-based 규약(그 노드가 이
  ///   목표와 같은 세션에서 0부터 새로 세기 시작한다고 가정)
  int32_t trough_index;
  /// m/s. 0.0(생략) => ingress_node 파라미터 기본값 사용
  float forward_speed;
  /// m/s. 0.0(생략) => ingress_node 파라미터 기본값 사용
  float return_speed;
} parking_robot_interfaces__action__IngressUnderTruck_Goal;

// Struct for a sequence of parking_robot_interfaces__action__IngressUnderTruck_Goal.
typedef struct parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence
{
  parking_robot_interfaces__action__IngressUnderTruck_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stop_reason'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/IngressUnderTruck in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__IngressUnderTruck_Result
{
  bool success;
  /// midpoint_reached | timeout | pose_stale | depth_lost | canceled
  rosidl_runtime_c__String stop_reason;
  /// 최종 정지 좌표(주행좌표계 world x)
  float stop_x;
  /// axle_detector_node 가 보고한 목표 트로프 중심 x(정렬 목표였던 값)
  float target_axle_x;
  /// 진단용: 명령 vy 의 시간적분 기반 추정치. GT 실측이 아님(이 노드는
  ///   지면진실 좌표를 모른다) -- 실제 횡편차 검증은 외부 스모크가 GT로 한다
  float est_max_lateral_dev_m;
} parking_robot_interfaces__action__IngressUnderTruck_Result;

// Struct for a sequence of parking_robot_interfaces__action__IngressUnderTruck_Result.
typedef struct parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence
{
  parking_robot_interfaces__action__IngressUnderTruck_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'phase'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/IngressUnderTruck in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__IngressUnderTruck_Feedback
{
  /// SEEK | RETURN | SETTLING
  rosidl_runtime_c__String phase;
  float current_x;
  int32_t troughs_seen;
  float vy_cmd;
} parking_robot_interfaces__action__IngressUnderTruck_Feedback;

// Struct for a sequence of parking_robot_interfaces__action__IngressUnderTruck_Feedback.
typedef struct parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence
{
  parking_robot_interfaces__action__IngressUnderTruck_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "parking_robot_interfaces/action/detail/ingress_under_truck__struct.h"

/// Struct defined in action/IngressUnderTruck in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  parking_robot_interfaces__action__IngressUnderTruck_Goal goal;
} parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request;

// Struct for a sequence of parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request.
typedef struct parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence
{
  parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/IngressUnderTruck in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response;

// Struct for a sequence of parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response.
typedef struct parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence
{
  parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/IngressUnderTruck in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request;

// Struct for a sequence of parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request.
typedef struct parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence
{
  parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "parking_robot_interfaces/action/detail/ingress_under_truck__struct.h"

/// Struct defined in action/IngressUnderTruck in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response
{
  int8_t status;
  parking_robot_interfaces__action__IngressUnderTruck_Result result;
} parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response;

// Struct for a sequence of parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response.
typedef struct parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence
{
  parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "parking_robot_interfaces/action/detail/ingress_under_truck__struct.h"

/// Struct defined in action/IngressUnderTruck in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  parking_robot_interfaces__action__IngressUnderTruck_Feedback feedback;
} parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage;

// Struct for a sequence of parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage.
typedef struct parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence
{
  parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PARKING_ROBOT_INTERFACES__ACTION__DETAIL__INGRESS_UNDER_TRUCK__STRUCT_H_
