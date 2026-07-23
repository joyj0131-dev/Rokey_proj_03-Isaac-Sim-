// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from parking_robot_interfaces:action/DetectVehicle.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__ACTION__DETAIL__DETECT_VEHICLE__STRUCT_H_
#define PARKING_ROBOT_INTERFACES__ACTION__DETAIL__DETECT_VEHICLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/DetectVehicle in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__DetectVehicle_Goal
{
  bool trigger;
} parking_robot_interfaces__action__DetectVehicle_Goal;

// Struct for a sequence of parking_robot_interfaces__action__DetectVehicle_Goal.
typedef struct parking_robot_interfaces__action__DetectVehicle_Goal__Sequence
{
  parking_robot_interfaces__action__DetectVehicle_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__DetectVehicle_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'vehicle_info'
#include "parking_robot_interfaces/msg/detail/vehicle_info__struct.h"

/// Struct defined in action/DetectVehicle in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__DetectVehicle_Result
{
  bool success;
  parking_robot_interfaces__msg__VehicleInfo vehicle_info;
} parking_robot_interfaces__action__DetectVehicle_Result;

// Struct for a sequence of parking_robot_interfaces__action__DetectVehicle_Result.
typedef struct parking_robot_interfaces__action__DetectVehicle_Result__Sequence
{
  parking_robot_interfaces__action__DetectVehicle_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__DetectVehicle_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'status'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/DetectVehicle in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__DetectVehicle_Feedback
{
  rosidl_runtime_c__String status;
} parking_robot_interfaces__action__DetectVehicle_Feedback;

// Struct for a sequence of parking_robot_interfaces__action__DetectVehicle_Feedback.
typedef struct parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence
{
  parking_robot_interfaces__action__DetectVehicle_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "parking_robot_interfaces/action/detail/detect_vehicle__struct.h"

/// Struct defined in action/DetectVehicle in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__DetectVehicle_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  parking_robot_interfaces__action__DetectVehicle_Goal goal;
} parking_robot_interfaces__action__DetectVehicle_SendGoal_Request;

// Struct for a sequence of parking_robot_interfaces__action__DetectVehicle_SendGoal_Request.
typedef struct parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence
{
  parking_robot_interfaces__action__DetectVehicle_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/DetectVehicle in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__DetectVehicle_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} parking_robot_interfaces__action__DetectVehicle_SendGoal_Response;

// Struct for a sequence of parking_robot_interfaces__action__DetectVehicle_SendGoal_Response.
typedef struct parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence
{
  parking_robot_interfaces__action__DetectVehicle_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/DetectVehicle in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__DetectVehicle_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} parking_robot_interfaces__action__DetectVehicle_GetResult_Request;

// Struct for a sequence of parking_robot_interfaces__action__DetectVehicle_GetResult_Request.
typedef struct parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence
{
  parking_robot_interfaces__action__DetectVehicle_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "parking_robot_interfaces/action/detail/detect_vehicle__struct.h"

/// Struct defined in action/DetectVehicle in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__DetectVehicle_GetResult_Response
{
  int8_t status;
  parking_robot_interfaces__action__DetectVehicle_Result result;
} parking_robot_interfaces__action__DetectVehicle_GetResult_Response;

// Struct for a sequence of parking_robot_interfaces__action__DetectVehicle_GetResult_Response.
typedef struct parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence
{
  parking_robot_interfaces__action__DetectVehicle_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "parking_robot_interfaces/action/detail/detect_vehicle__struct.h"

/// Struct defined in action/DetectVehicle in the package parking_robot_interfaces.
typedef struct parking_robot_interfaces__action__DetectVehicle_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  parking_robot_interfaces__action__DetectVehicle_Feedback feedback;
} parking_robot_interfaces__action__DetectVehicle_FeedbackMessage;

// Struct for a sequence of parking_robot_interfaces__action__DetectVehicle_FeedbackMessage.
typedef struct parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence
{
  parking_robot_interfaces__action__DetectVehicle_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PARKING_ROBOT_INTERFACES__ACTION__DETAIL__DETECT_VEHICLE__STRUCT_H_
