// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from parking_robot_interfaces:action/AlignVehicle.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__ACTION__DETAIL__ALIGN_VEHICLE__STRUCT_H_
#define PARKING_ROBOT_INTERFACES__ACTION__DETAIL__ALIGN_VEHICLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in action/AlignVehicle in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__AlignVehicle_Goal
{
  geometry_msgs__msg__Pose target_pose;
} parking_robot_interfaces__action__AlignVehicle_Goal;

// Struct for a sequence of parking_robot_interfaces__action__AlignVehicle_Goal.
typedef struct parking_robot_interfaces__action__AlignVehicle_Goal__Sequence
{
  parking_robot_interfaces__action__AlignVehicle_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__AlignVehicle_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/AlignVehicle in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__AlignVehicle_Result
{
  bool success;
  float final_error;
} parking_robot_interfaces__action__AlignVehicle_Result;

// Struct for a sequence of parking_robot_interfaces__action__AlignVehicle_Result.
typedef struct parking_robot_interfaces__action__AlignVehicle_Result__Sequence
{
  parking_robot_interfaces__action__AlignVehicle_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__AlignVehicle_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/AlignVehicle in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__AlignVehicle_Feedback
{
  float current_error;
} parking_robot_interfaces__action__AlignVehicle_Feedback;

// Struct for a sequence of parking_robot_interfaces__action__AlignVehicle_Feedback.
typedef struct parking_robot_interfaces__action__AlignVehicle_Feedback__Sequence
{
  parking_robot_interfaces__action__AlignVehicle_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__AlignVehicle_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "parking_robot_interfaces/action/detail/align_vehicle__struct.h"

/// Struct defined in action/AlignVehicle in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__AlignVehicle_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  parking_robot_interfaces__action__AlignVehicle_Goal goal;
} parking_robot_interfaces__action__AlignVehicle_SendGoal_Request;

// Struct for a sequence of parking_robot_interfaces__action__AlignVehicle_SendGoal_Request.
typedef struct parking_robot_interfaces__action__AlignVehicle_SendGoal_Request__Sequence
{
  parking_robot_interfaces__action__AlignVehicle_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__AlignVehicle_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/AlignVehicle in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__AlignVehicle_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} parking_robot_interfaces__action__AlignVehicle_SendGoal_Response;

// Struct for a sequence of parking_robot_interfaces__action__AlignVehicle_SendGoal_Response.
typedef struct parking_robot_interfaces__action__AlignVehicle_SendGoal_Response__Sequence
{
  parking_robot_interfaces__action__AlignVehicle_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__AlignVehicle_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/AlignVehicle in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__AlignVehicle_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} parking_robot_interfaces__action__AlignVehicle_GetResult_Request;

// Struct for a sequence of parking_robot_interfaces__action__AlignVehicle_GetResult_Request.
typedef struct parking_robot_interfaces__action__AlignVehicle_GetResult_Request__Sequence
{
  parking_robot_interfaces__action__AlignVehicle_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__AlignVehicle_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "parking_robot_interfaces/action/detail/align_vehicle__struct.h"

/// Struct defined in action/AlignVehicle in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__AlignVehicle_GetResult_Response
{
  int8_t status;
  parking_robot_interfaces__action__AlignVehicle_Result result;
} parking_robot_interfaces__action__AlignVehicle_GetResult_Response;

// Struct for a sequence of parking_robot_interfaces__action__AlignVehicle_GetResult_Response.
typedef struct parking_robot_interfaces__action__AlignVehicle_GetResult_Response__Sequence
{
  parking_robot_interfaces__action__AlignVehicle_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__AlignVehicle_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "parking_robot_interfaces/action/detail/align_vehicle__struct.h"

/// Struct defined in action/AlignVehicle in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__AlignVehicle_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  parking_robot_interfaces__action__AlignVehicle_Feedback feedback;
} parking_robot_interfaces__action__AlignVehicle_FeedbackMessage;

// Struct for a sequence of parking_robot_interfaces__action__AlignVehicle_FeedbackMessage.
typedef struct parking_robot_interfaces__action__AlignVehicle_FeedbackMessage__Sequence
{
  parking_robot_interfaces__action__AlignVehicle_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__AlignVehicle_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PARKING_ROBOT_INTERFACES__ACTION__DETAIL__ALIGN_VEHICLE__STRUCT_H_
