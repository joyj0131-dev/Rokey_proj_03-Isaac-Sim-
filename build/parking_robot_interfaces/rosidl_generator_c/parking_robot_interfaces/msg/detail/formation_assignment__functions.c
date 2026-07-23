// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from parking_robot_interfaces:msg/FormationAssignment.idl
// generated code does not contain a copyright notice
#include "parking_robot_interfaces/msg/detail/formation_assignment__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `robot_id`
// Member `task_id`
// Member `role`
// Member `partner_robot_id`
#include "rosidl_runtime_c/string_functions.h"

bool
parking_robot_interfaces__msg__FormationAssignment__init(parking_robot_interfaces__msg__FormationAssignment * msg)
{
  if (!msg) {
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__init(&msg->robot_id)) {
    parking_robot_interfaces__msg__FormationAssignment__fini(msg);
    return false;
  }
  // task_id
  if (!rosidl_runtime_c__String__init(&msg->task_id)) {
    parking_robot_interfaces__msg__FormationAssignment__fini(msg);
    return false;
  }
  // role
  if (!rosidl_runtime_c__String__init(&msg->role)) {
    parking_robot_interfaces__msg__FormationAssignment__fini(msg);
    return false;
  }
  // partner_robot_id
  if (!rosidl_runtime_c__String__init(&msg->partner_robot_id)) {
    parking_robot_interfaces__msg__FormationAssignment__fini(msg);
    return false;
  }
  // active
  return true;
}

void
parking_robot_interfaces__msg__FormationAssignment__fini(parking_robot_interfaces__msg__FormationAssignment * msg)
{
  if (!msg) {
    return;
  }
  // robot_id
  rosidl_runtime_c__String__fini(&msg->robot_id);
  // task_id
  rosidl_runtime_c__String__fini(&msg->task_id);
  // role
  rosidl_runtime_c__String__fini(&msg->role);
  // partner_robot_id
  rosidl_runtime_c__String__fini(&msg->partner_robot_id);
  // active
}

bool
parking_robot_interfaces__msg__FormationAssignment__are_equal(const parking_robot_interfaces__msg__FormationAssignment * lhs, const parking_robot_interfaces__msg__FormationAssignment * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->robot_id), &(rhs->robot_id)))
  {
    return false;
  }
  // task_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->task_id), &(rhs->task_id)))
  {
    return false;
  }
  // role
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->role), &(rhs->role)))
  {
    return false;
  }
  // partner_robot_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->partner_robot_id), &(rhs->partner_robot_id)))
  {
    return false;
  }
  // active
  if (lhs->active != rhs->active) {
    return false;
  }
  return true;
}

bool
parking_robot_interfaces__msg__FormationAssignment__copy(
  const parking_robot_interfaces__msg__FormationAssignment * input,
  parking_robot_interfaces__msg__FormationAssignment * output)
{
  if (!input || !output) {
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__copy(
      &(input->robot_id), &(output->robot_id)))
  {
    return false;
  }
  // task_id
  if (!rosidl_runtime_c__String__copy(
      &(input->task_id), &(output->task_id)))
  {
    return false;
  }
  // role
  if (!rosidl_runtime_c__String__copy(
      &(input->role), &(output->role)))
  {
    return false;
  }
  // partner_robot_id
  if (!rosidl_runtime_c__String__copy(
      &(input->partner_robot_id), &(output->partner_robot_id)))
  {
    return false;
  }
  // active
  output->active = input->active;
  return true;
}

parking_robot_interfaces__msg__FormationAssignment *
parking_robot_interfaces__msg__FormationAssignment__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__msg__FormationAssignment * msg = (parking_robot_interfaces__msg__FormationAssignment *)allocator.allocate(sizeof(parking_robot_interfaces__msg__FormationAssignment), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parking_robot_interfaces__msg__FormationAssignment));
  bool success = parking_robot_interfaces__msg__FormationAssignment__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parking_robot_interfaces__msg__FormationAssignment__destroy(parking_robot_interfaces__msg__FormationAssignment * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    parking_robot_interfaces__msg__FormationAssignment__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
parking_robot_interfaces__msg__FormationAssignment__Sequence__init(parking_robot_interfaces__msg__FormationAssignment__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__msg__FormationAssignment * data = NULL;

  if (size) {
    data = (parking_robot_interfaces__msg__FormationAssignment *)allocator.zero_allocate(size, sizeof(parking_robot_interfaces__msg__FormationAssignment), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = parking_robot_interfaces__msg__FormationAssignment__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        parking_robot_interfaces__msg__FormationAssignment__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
parking_robot_interfaces__msg__FormationAssignment__Sequence__fini(parking_robot_interfaces__msg__FormationAssignment__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      parking_robot_interfaces__msg__FormationAssignment__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

parking_robot_interfaces__msg__FormationAssignment__Sequence *
parking_robot_interfaces__msg__FormationAssignment__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__msg__FormationAssignment__Sequence * array = (parking_robot_interfaces__msg__FormationAssignment__Sequence *)allocator.allocate(sizeof(parking_robot_interfaces__msg__FormationAssignment__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = parking_robot_interfaces__msg__FormationAssignment__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
parking_robot_interfaces__msg__FormationAssignment__Sequence__destroy(parking_robot_interfaces__msg__FormationAssignment__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    parking_robot_interfaces__msg__FormationAssignment__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
parking_robot_interfaces__msg__FormationAssignment__Sequence__are_equal(const parking_robot_interfaces__msg__FormationAssignment__Sequence * lhs, const parking_robot_interfaces__msg__FormationAssignment__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!parking_robot_interfaces__msg__FormationAssignment__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
parking_robot_interfaces__msg__FormationAssignment__Sequence__copy(
  const parking_robot_interfaces__msg__FormationAssignment__Sequence * input,
  parking_robot_interfaces__msg__FormationAssignment__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(parking_robot_interfaces__msg__FormationAssignment);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    parking_robot_interfaces__msg__FormationAssignment * data =
      (parking_robot_interfaces__msg__FormationAssignment *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!parking_robot_interfaces__msg__FormationAssignment__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          parking_robot_interfaces__msg__FormationAssignment__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!parking_robot_interfaces__msg__FormationAssignment__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
