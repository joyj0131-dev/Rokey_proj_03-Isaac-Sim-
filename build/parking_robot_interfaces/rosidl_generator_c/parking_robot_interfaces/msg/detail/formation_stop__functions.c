// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from parking_robot_interfaces:msg/FormationStop.idl
// generated code does not contain a copyright notice
#include "parking_robot_interfaces/msg/detail/formation_stop__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `task_id`
// Member `source_robot_id`
// Member `reason`
#include "rosidl_runtime_c/string_functions.h"

bool
parking_robot_interfaces__msg__FormationStop__init(parking_robot_interfaces__msg__FormationStop * msg)
{
  if (!msg) {
    return false;
  }
  // task_id
  if (!rosidl_runtime_c__String__init(&msg->task_id)) {
    parking_robot_interfaces__msg__FormationStop__fini(msg);
    return false;
  }
  // source_robot_id
  if (!rosidl_runtime_c__String__init(&msg->source_robot_id)) {
    parking_robot_interfaces__msg__FormationStop__fini(msg);
    return false;
  }
  // stop
  // reason
  if (!rosidl_runtime_c__String__init(&msg->reason)) {
    parking_robot_interfaces__msg__FormationStop__fini(msg);
    return false;
  }
  return true;
}

void
parking_robot_interfaces__msg__FormationStop__fini(parking_robot_interfaces__msg__FormationStop * msg)
{
  if (!msg) {
    return;
  }
  // task_id
  rosidl_runtime_c__String__fini(&msg->task_id);
  // source_robot_id
  rosidl_runtime_c__String__fini(&msg->source_robot_id);
  // stop
  // reason
  rosidl_runtime_c__String__fini(&msg->reason);
}

bool
parking_robot_interfaces__msg__FormationStop__are_equal(const parking_robot_interfaces__msg__FormationStop * lhs, const parking_robot_interfaces__msg__FormationStop * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // task_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->task_id), &(rhs->task_id)))
  {
    return false;
  }
  // source_robot_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->source_robot_id), &(rhs->source_robot_id)))
  {
    return false;
  }
  // stop
  if (lhs->stop != rhs->stop) {
    return false;
  }
  // reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->reason), &(rhs->reason)))
  {
    return false;
  }
  return true;
}

bool
parking_robot_interfaces__msg__FormationStop__copy(
  const parking_robot_interfaces__msg__FormationStop * input,
  parking_robot_interfaces__msg__FormationStop * output)
{
  if (!input || !output) {
    return false;
  }
  // task_id
  if (!rosidl_runtime_c__String__copy(
      &(input->task_id), &(output->task_id)))
  {
    return false;
  }
  // source_robot_id
  if (!rosidl_runtime_c__String__copy(
      &(input->source_robot_id), &(output->source_robot_id)))
  {
    return false;
  }
  // stop
  output->stop = input->stop;
  // reason
  if (!rosidl_runtime_c__String__copy(
      &(input->reason), &(output->reason)))
  {
    return false;
  }
  return true;
}

parking_robot_interfaces__msg__FormationStop *
parking_robot_interfaces__msg__FormationStop__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__msg__FormationStop * msg = (parking_robot_interfaces__msg__FormationStop *)allocator.allocate(sizeof(parking_robot_interfaces__msg__FormationStop), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parking_robot_interfaces__msg__FormationStop));
  bool success = parking_robot_interfaces__msg__FormationStop__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parking_robot_interfaces__msg__FormationStop__destroy(parking_robot_interfaces__msg__FormationStop * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    parking_robot_interfaces__msg__FormationStop__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
parking_robot_interfaces__msg__FormationStop__Sequence__init(parking_robot_interfaces__msg__FormationStop__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__msg__FormationStop * data = NULL;

  if (size) {
    data = (parking_robot_interfaces__msg__FormationStop *)allocator.zero_allocate(size, sizeof(parking_robot_interfaces__msg__FormationStop), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = parking_robot_interfaces__msg__FormationStop__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        parking_robot_interfaces__msg__FormationStop__fini(&data[i - 1]);
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
parking_robot_interfaces__msg__FormationStop__Sequence__fini(parking_robot_interfaces__msg__FormationStop__Sequence * array)
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
      parking_robot_interfaces__msg__FormationStop__fini(&array->data[i]);
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

parking_robot_interfaces__msg__FormationStop__Sequence *
parking_robot_interfaces__msg__FormationStop__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__msg__FormationStop__Sequence * array = (parking_robot_interfaces__msg__FormationStop__Sequence *)allocator.allocate(sizeof(parking_robot_interfaces__msg__FormationStop__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = parking_robot_interfaces__msg__FormationStop__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
parking_robot_interfaces__msg__FormationStop__Sequence__destroy(parking_robot_interfaces__msg__FormationStop__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    parking_robot_interfaces__msg__FormationStop__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
parking_robot_interfaces__msg__FormationStop__Sequence__are_equal(const parking_robot_interfaces__msg__FormationStop__Sequence * lhs, const parking_robot_interfaces__msg__FormationStop__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!parking_robot_interfaces__msg__FormationStop__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
parking_robot_interfaces__msg__FormationStop__Sequence__copy(
  const parking_robot_interfaces__msg__FormationStop__Sequence * input,
  parking_robot_interfaces__msg__FormationStop__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(parking_robot_interfaces__msg__FormationStop);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    parking_robot_interfaces__msg__FormationStop * data =
      (parking_robot_interfaces__msg__FormationStop *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!parking_robot_interfaces__msg__FormationStop__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          parking_robot_interfaces__msg__FormationStop__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!parking_robot_interfaces__msg__FormationStop__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
