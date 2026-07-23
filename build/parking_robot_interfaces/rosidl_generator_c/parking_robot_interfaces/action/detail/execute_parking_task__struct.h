// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from parking_robot_interfaces:action/ExecuteParkingTask.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__ACTION__DETAIL__EXECUTE_PARKING_TASK__STRUCT_H_
#define PARKING_ROBOT_INTERFACES__ACTION__DETAIL__EXECUTE_PARKING_TASK__STRUCT_H_

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
// Member 'request_type'
// Member 'vehicle_id'
// Member 'slot_id'
// Member 'leader_robot_id'
// Member 'follower_robot_id'
#include "rosidl_runtime_c/string.h"
// Member 'slot_pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in action/ExecuteParkingTask in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__ExecuteParkingTask_Goal
{
  rosidl_runtime_c__String task_id;
  /// ENTRY, EXIT
  rosidl_runtime_c__String request_type;
  rosidl_runtime_c__String vehicle_id;
  /// 목표 주차면 (예: 'B1')
  rosidl_runtime_c__String slot_id;
  /// 목표 주차면 좌표 (map 프레임)
  geometry_msgs__msg__Pose slot_pose;
  rosidl_runtime_c__String leader_robot_id;
  /// 로봇 1대 MVP 흐름에서는 빈 문자열
  rosidl_runtime_c__String follower_robot_id;
} parking_robot_interfaces__action__ExecuteParkingTask_Goal;

// Struct for a sequence of parking_robot_interfaces__action__ExecuteParkingTask_Goal.
typedef struct parking_robot_interfaces__action__ExecuteParkingTask_Goal__Sequence
{
  parking_robot_interfaces__action__ExecuteParkingTask_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__ExecuteParkingTask_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/ExecuteParkingTask in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__ExecuteParkingTask_Result
{
  bool success;
  rosidl_runtime_c__String message;
} parking_robot_interfaces__action__ExecuteParkingTask_Result;

// Struct for a sequence of parking_robot_interfaces__action__ExecuteParkingTask_Result.
typedef struct parking_robot_interfaces__action__ExecuteParkingTask_Result__Sequence
{
  parking_robot_interfaces__action__ExecuteParkingTask_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__ExecuteParkingTask_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'current_step'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/ExecuteParkingTask in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__ExecuteParkingTask_Feedback
{
  rosidl_runtime_c__String current_step;
  float progress;
} parking_robot_interfaces__action__ExecuteParkingTask_Feedback;

// Struct for a sequence of parking_robot_interfaces__action__ExecuteParkingTask_Feedback.
typedef struct parking_robot_interfaces__action__ExecuteParkingTask_Feedback__Sequence
{
  parking_robot_interfaces__action__ExecuteParkingTask_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__ExecuteParkingTask_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "parking_robot_interfaces/action/detail/execute_parking_task__struct.h"

/// Struct defined in action/ExecuteParkingTask in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  parking_robot_interfaces__action__ExecuteParkingTask_Goal goal;
} parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Request;

// Struct for a sequence of parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Request.
typedef struct parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Request__Sequence
{
  parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/ExecuteParkingTask in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Response;

// Struct for a sequence of parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Response.
typedef struct parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Response__Sequence
{
  parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__ExecuteParkingTask_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/ExecuteParkingTask in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Request;

// Struct for a sequence of parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Request.
typedef struct parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Request__Sequence
{
  parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "parking_robot_interfaces/action/detail/execute_parking_task__struct.h"

/// Struct defined in action/ExecuteParkingTask in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Response
{
  int8_t status;
  parking_robot_interfaces__action__ExecuteParkingTask_Result result;
} parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Response;

// Struct for a sequence of parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Response.
typedef struct parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Response__Sequence
{
  parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__ExecuteParkingTask_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "parking_robot_interfaces/action/detail/execute_parking_task__struct.h"

/// Struct defined in action/ExecuteParkingTask in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__ExecuteParkingTask_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  parking_robot_interfaces__action__ExecuteParkingTask_Feedback feedback;
} parking_robot_interfaces__action__ExecuteParkingTask_FeedbackMessage;

// Struct for a sequence of parking_robot_interfaces__action__ExecuteParkingTask_FeedbackMessage.
typedef struct parking_robot_interfaces__action__ExecuteParkingTask_FeedbackMessage__Sequence
{
  parking_robot_interfaces__action__ExecuteParkingTask_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__ExecuteParkingTask_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PARKING_ROBOT_INTERFACES__ACTION__DETAIL__EXECUTE_PARKING_TASK__STRUCT_H_
