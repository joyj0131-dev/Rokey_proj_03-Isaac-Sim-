// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from parking_robot_interfaces:msg/FormationAssignment.idl
// generated code does not contain a copyright notice

#ifndef PARKING_ROBOT_INTERFACES__MSG__DETAIL__FORMATION_ASSIGNMENT__FUNCTIONS_H_
#define PARKING_ROBOT_INTERFACES__MSG__DETAIL__FORMATION_ASSIGNMENT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "parking_robot_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "parking_robot_interfaces/msg/detail/formation_assignment__struct.h"

/// Initialize msg/FormationAssignment message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * parking_robot_interfaces__msg__FormationAssignment
 * )) before or use
 * parking_robot_interfaces__msg__FormationAssignment__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_parking_robot_interfaces
bool
parking_robot_interfaces__msg__FormationAssignment__init(parking_robot_interfaces__msg__FormationAssignment * msg);

/// Finalize msg/FormationAssignment message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_parking_robot_interfaces
void
parking_robot_interfaces__msg__FormationAssignment__fini(parking_robot_interfaces__msg__FormationAssignment * msg);

/// Create msg/FormationAssignment message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * parking_robot_interfaces__msg__FormationAssignment__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_parking_robot_interfaces
parking_robot_interfaces__msg__FormationAssignment *
parking_robot_interfaces__msg__FormationAssignment__create();

/// Destroy msg/FormationAssignment message.
/**
 * It calls
 * parking_robot_interfaces__msg__FormationAssignment__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_parking_robot_interfaces
void
parking_robot_interfaces__msg__FormationAssignment__destroy(parking_robot_interfaces__msg__FormationAssignment * msg);

/// Check for msg/FormationAssignment message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_parking_robot_interfaces
bool
parking_robot_interfaces__msg__FormationAssignment__are_equal(const parking_robot_interfaces__msg__FormationAssignment * lhs, const parking_robot_interfaces__msg__FormationAssignment * rhs);

/// Copy a msg/FormationAssignment message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_parking_robot_interfaces
bool
parking_robot_interfaces__msg__FormationAssignment__copy(
  const parking_robot_interfaces__msg__FormationAssignment * input,
  parking_robot_interfaces__msg__FormationAssignment * output);

/// Initialize array of msg/FormationAssignment messages.
/**
 * It allocates the memory for the number of elements and calls
 * parking_robot_interfaces__msg__FormationAssignment__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_parking_robot_interfaces
bool
parking_robot_interfaces__msg__FormationAssignment__Sequence__init(parking_robot_interfaces__msg__FormationAssignment__Sequence * array, size_t size);

/// Finalize array of msg/FormationAssignment messages.
/**
 * It calls
 * parking_robot_interfaces__msg__FormationAssignment__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_parking_robot_interfaces
void
parking_robot_interfaces__msg__FormationAssignment__Sequence__fini(parking_robot_interfaces__msg__FormationAssignment__Sequence * array);

/// Create array of msg/FormationAssignment messages.
/**
 * It allocates the memory for the array and calls
 * parking_robot_interfaces__msg__FormationAssignment__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_parking_robot_interfaces
parking_robot_interfaces__msg__FormationAssignment__Sequence *
parking_robot_interfaces__msg__FormationAssignment__Sequence__create(size_t size);

/// Destroy array of msg/FormationAssignment messages.
/**
 * It calls
 * parking_robot_interfaces__msg__FormationAssignment__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_parking_robot_interfaces
void
parking_robot_interfaces__msg__FormationAssignment__Sequence__destroy(parking_robot_interfaces__msg__FormationAssignment__Sequence * array);

/// Check for msg/FormationAssignment message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_parking_robot_interfaces
bool
parking_robot_interfaces__msg__FormationAssignment__Sequence__are_equal(const parking_robot_interfaces__msg__FormationAssignment__Sequence * lhs, const parking_robot_interfaces__msg__FormationAssignment__Sequence * rhs);

/// Copy an array of msg/FormationAssignment messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_parking_robot_interfaces
bool
parking_robot_interfaces__msg__FormationAssignment__Sequence__copy(
  const parking_robot_interfaces__msg__FormationAssignment__Sequence * input,
  parking_robot_interfaces__msg__FormationAssignment__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // PARKING_ROBOT_INTERFACES__MSG__DETAIL__FORMATION_ASSIGNMENT__FUNCTIONS_H_
