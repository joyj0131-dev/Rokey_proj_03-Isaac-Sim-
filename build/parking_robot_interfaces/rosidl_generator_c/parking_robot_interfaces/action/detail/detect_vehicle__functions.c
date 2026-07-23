// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from parking_robot_interfaces:action/DetectVehicle.idl
// generated code does not contain a copyright notice
#include "parking_robot_interfaces/action/detail/detect_vehicle__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
parking_robot_interfaces__action__DetectVehicle_Goal__init(parking_robot_interfaces__action__DetectVehicle_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // trigger
  return true;
}

void
parking_robot_interfaces__action__DetectVehicle_Goal__fini(parking_robot_interfaces__action__DetectVehicle_Goal * msg)
{
  if (!msg) {
    return;
  }
  // trigger
}

bool
parking_robot_interfaces__action__DetectVehicle_Goal__are_equal(const parking_robot_interfaces__action__DetectVehicle_Goal * lhs, const parking_robot_interfaces__action__DetectVehicle_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // trigger
  if (lhs->trigger != rhs->trigger) {
    return false;
  }
  return true;
}

bool
parking_robot_interfaces__action__DetectVehicle_Goal__copy(
  const parking_robot_interfaces__action__DetectVehicle_Goal * input,
  parking_robot_interfaces__action__DetectVehicle_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // trigger
  output->trigger = input->trigger;
  return true;
}

parking_robot_interfaces__action__DetectVehicle_Goal *
parking_robot_interfaces__action__DetectVehicle_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__DetectVehicle_Goal * msg = (parking_robot_interfaces__action__DetectVehicle_Goal *)allocator.allocate(sizeof(parking_robot_interfaces__action__DetectVehicle_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parking_robot_interfaces__action__DetectVehicle_Goal));
  bool success = parking_robot_interfaces__action__DetectVehicle_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parking_robot_interfaces__action__DetectVehicle_Goal__destroy(parking_robot_interfaces__action__DetectVehicle_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    parking_robot_interfaces__action__DetectVehicle_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
parking_robot_interfaces__action__DetectVehicle_Goal__Sequence__init(parking_robot_interfaces__action__DetectVehicle_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__DetectVehicle_Goal * data = NULL;

  if (size) {
    data = (parking_robot_interfaces__action__DetectVehicle_Goal *)allocator.zero_allocate(size, sizeof(parking_robot_interfaces__action__DetectVehicle_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = parking_robot_interfaces__action__DetectVehicle_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        parking_robot_interfaces__action__DetectVehicle_Goal__fini(&data[i - 1]);
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
parking_robot_interfaces__action__DetectVehicle_Goal__Sequence__fini(parking_robot_interfaces__action__DetectVehicle_Goal__Sequence * array)
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
      parking_robot_interfaces__action__DetectVehicle_Goal__fini(&array->data[i]);
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

parking_robot_interfaces__action__DetectVehicle_Goal__Sequence *
parking_robot_interfaces__action__DetectVehicle_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__DetectVehicle_Goal__Sequence * array = (parking_robot_interfaces__action__DetectVehicle_Goal__Sequence *)allocator.allocate(sizeof(parking_robot_interfaces__action__DetectVehicle_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = parking_robot_interfaces__action__DetectVehicle_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
parking_robot_interfaces__action__DetectVehicle_Goal__Sequence__destroy(parking_robot_interfaces__action__DetectVehicle_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    parking_robot_interfaces__action__DetectVehicle_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
parking_robot_interfaces__action__DetectVehicle_Goal__Sequence__are_equal(const parking_robot_interfaces__action__DetectVehicle_Goal__Sequence * lhs, const parking_robot_interfaces__action__DetectVehicle_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!parking_robot_interfaces__action__DetectVehicle_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
parking_robot_interfaces__action__DetectVehicle_Goal__Sequence__copy(
  const parking_robot_interfaces__action__DetectVehicle_Goal__Sequence * input,
  parking_robot_interfaces__action__DetectVehicle_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(parking_robot_interfaces__action__DetectVehicle_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    parking_robot_interfaces__action__DetectVehicle_Goal * data =
      (parking_robot_interfaces__action__DetectVehicle_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!parking_robot_interfaces__action__DetectVehicle_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          parking_robot_interfaces__action__DetectVehicle_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!parking_robot_interfaces__action__DetectVehicle_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `vehicle_info`
#include "parking_robot_interfaces/msg/detail/vehicle_info__functions.h"

bool
parking_robot_interfaces__action__DetectVehicle_Result__init(parking_robot_interfaces__action__DetectVehicle_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // vehicle_info
  if (!parking_robot_interfaces__msg__VehicleInfo__init(&msg->vehicle_info)) {
    parking_robot_interfaces__action__DetectVehicle_Result__fini(msg);
    return false;
  }
  return true;
}

void
parking_robot_interfaces__action__DetectVehicle_Result__fini(parking_robot_interfaces__action__DetectVehicle_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
  // vehicle_info
  parking_robot_interfaces__msg__VehicleInfo__fini(&msg->vehicle_info);
}

bool
parking_robot_interfaces__action__DetectVehicle_Result__are_equal(const parking_robot_interfaces__action__DetectVehicle_Result * lhs, const parking_robot_interfaces__action__DetectVehicle_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // vehicle_info
  if (!parking_robot_interfaces__msg__VehicleInfo__are_equal(
      &(lhs->vehicle_info), &(rhs->vehicle_info)))
  {
    return false;
  }
  return true;
}

bool
parking_robot_interfaces__action__DetectVehicle_Result__copy(
  const parking_robot_interfaces__action__DetectVehicle_Result * input,
  parking_robot_interfaces__action__DetectVehicle_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // vehicle_info
  if (!parking_robot_interfaces__msg__VehicleInfo__copy(
      &(input->vehicle_info), &(output->vehicle_info)))
  {
    return false;
  }
  return true;
}

parking_robot_interfaces__action__DetectVehicle_Result *
parking_robot_interfaces__action__DetectVehicle_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__DetectVehicle_Result * msg = (parking_robot_interfaces__action__DetectVehicle_Result *)allocator.allocate(sizeof(parking_robot_interfaces__action__DetectVehicle_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parking_robot_interfaces__action__DetectVehicle_Result));
  bool success = parking_robot_interfaces__action__DetectVehicle_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parking_robot_interfaces__action__DetectVehicle_Result__destroy(parking_robot_interfaces__action__DetectVehicle_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    parking_robot_interfaces__action__DetectVehicle_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
parking_robot_interfaces__action__DetectVehicle_Result__Sequence__init(parking_robot_interfaces__action__DetectVehicle_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__DetectVehicle_Result * data = NULL;

  if (size) {
    data = (parking_robot_interfaces__action__DetectVehicle_Result *)allocator.zero_allocate(size, sizeof(parking_robot_interfaces__action__DetectVehicle_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = parking_robot_interfaces__action__DetectVehicle_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        parking_robot_interfaces__action__DetectVehicle_Result__fini(&data[i - 1]);
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
parking_robot_interfaces__action__DetectVehicle_Result__Sequence__fini(parking_robot_interfaces__action__DetectVehicle_Result__Sequence * array)
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
      parking_robot_interfaces__action__DetectVehicle_Result__fini(&array->data[i]);
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

parking_robot_interfaces__action__DetectVehicle_Result__Sequence *
parking_robot_interfaces__action__DetectVehicle_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__DetectVehicle_Result__Sequence * array = (parking_robot_interfaces__action__DetectVehicle_Result__Sequence *)allocator.allocate(sizeof(parking_robot_interfaces__action__DetectVehicle_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = parking_robot_interfaces__action__DetectVehicle_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
parking_robot_interfaces__action__DetectVehicle_Result__Sequence__destroy(parking_robot_interfaces__action__DetectVehicle_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    parking_robot_interfaces__action__DetectVehicle_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
parking_robot_interfaces__action__DetectVehicle_Result__Sequence__are_equal(const parking_robot_interfaces__action__DetectVehicle_Result__Sequence * lhs, const parking_robot_interfaces__action__DetectVehicle_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!parking_robot_interfaces__action__DetectVehicle_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
parking_robot_interfaces__action__DetectVehicle_Result__Sequence__copy(
  const parking_robot_interfaces__action__DetectVehicle_Result__Sequence * input,
  parking_robot_interfaces__action__DetectVehicle_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(parking_robot_interfaces__action__DetectVehicle_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    parking_robot_interfaces__action__DetectVehicle_Result * data =
      (parking_robot_interfaces__action__DetectVehicle_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!parking_robot_interfaces__action__DetectVehicle_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          parking_robot_interfaces__action__DetectVehicle_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!parking_robot_interfaces__action__DetectVehicle_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `status`
#include "rosidl_runtime_c/string_functions.h"

bool
parking_robot_interfaces__action__DetectVehicle_Feedback__init(parking_robot_interfaces__action__DetectVehicle_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__init(&msg->status)) {
    parking_robot_interfaces__action__DetectVehicle_Feedback__fini(msg);
    return false;
  }
  return true;
}

void
parking_robot_interfaces__action__DetectVehicle_Feedback__fini(parking_robot_interfaces__action__DetectVehicle_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // status
  rosidl_runtime_c__String__fini(&msg->status);
}

bool
parking_robot_interfaces__action__DetectVehicle_Feedback__are_equal(const parking_robot_interfaces__action__DetectVehicle_Feedback * lhs, const parking_robot_interfaces__action__DetectVehicle_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->status), &(rhs->status)))
  {
    return false;
  }
  return true;
}

bool
parking_robot_interfaces__action__DetectVehicle_Feedback__copy(
  const parking_robot_interfaces__action__DetectVehicle_Feedback * input,
  parking_robot_interfaces__action__DetectVehicle_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__copy(
      &(input->status), &(output->status)))
  {
    return false;
  }
  return true;
}

parking_robot_interfaces__action__DetectVehicle_Feedback *
parking_robot_interfaces__action__DetectVehicle_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__DetectVehicle_Feedback * msg = (parking_robot_interfaces__action__DetectVehicle_Feedback *)allocator.allocate(sizeof(parking_robot_interfaces__action__DetectVehicle_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parking_robot_interfaces__action__DetectVehicle_Feedback));
  bool success = parking_robot_interfaces__action__DetectVehicle_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parking_robot_interfaces__action__DetectVehicle_Feedback__destroy(parking_robot_interfaces__action__DetectVehicle_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    parking_robot_interfaces__action__DetectVehicle_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence__init(parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__DetectVehicle_Feedback * data = NULL;

  if (size) {
    data = (parking_robot_interfaces__action__DetectVehicle_Feedback *)allocator.zero_allocate(size, sizeof(parking_robot_interfaces__action__DetectVehicle_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = parking_robot_interfaces__action__DetectVehicle_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        parking_robot_interfaces__action__DetectVehicle_Feedback__fini(&data[i - 1]);
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
parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence__fini(parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence * array)
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
      parking_robot_interfaces__action__DetectVehicle_Feedback__fini(&array->data[i]);
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

parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence *
parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence * array = (parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence *)allocator.allocate(sizeof(parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence__destroy(parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence__are_equal(const parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence * lhs, const parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!parking_robot_interfaces__action__DetectVehicle_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence__copy(
  const parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence * input,
  parking_robot_interfaces__action__DetectVehicle_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(parking_robot_interfaces__action__DetectVehicle_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    parking_robot_interfaces__action__DetectVehicle_Feedback * data =
      (parking_robot_interfaces__action__DetectVehicle_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!parking_robot_interfaces__action__DetectVehicle_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          parking_robot_interfaces__action__DetectVehicle_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!parking_robot_interfaces__action__DetectVehicle_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "parking_robot_interfaces/action/detail/detect_vehicle__functions.h"

bool
parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__init(parking_robot_interfaces__action__DetectVehicle_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!parking_robot_interfaces__action__DetectVehicle_Goal__init(&msg->goal)) {
    parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__fini(parking_robot_interfaces__action__DetectVehicle_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  parking_robot_interfaces__action__DetectVehicle_Goal__fini(&msg->goal);
}

bool
parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__are_equal(const parking_robot_interfaces__action__DetectVehicle_SendGoal_Request * lhs, const parking_robot_interfaces__action__DetectVehicle_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!parking_robot_interfaces__action__DetectVehicle_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__copy(
  const parking_robot_interfaces__action__DetectVehicle_SendGoal_Request * input,
  parking_robot_interfaces__action__DetectVehicle_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!parking_robot_interfaces__action__DetectVehicle_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

parking_robot_interfaces__action__DetectVehicle_SendGoal_Request *
parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__DetectVehicle_SendGoal_Request * msg = (parking_robot_interfaces__action__DetectVehicle_SendGoal_Request *)allocator.allocate(sizeof(parking_robot_interfaces__action__DetectVehicle_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parking_robot_interfaces__action__DetectVehicle_SendGoal_Request));
  bool success = parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__destroy(parking_robot_interfaces__action__DetectVehicle_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence__init(parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__DetectVehicle_SendGoal_Request * data = NULL;

  if (size) {
    data = (parking_robot_interfaces__action__DetectVehicle_SendGoal_Request *)allocator.zero_allocate(size, sizeof(parking_robot_interfaces__action__DetectVehicle_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__fini(&data[i - 1]);
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
parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence__fini(parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence * array)
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
      parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__fini(&array->data[i]);
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

parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence *
parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence * array = (parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence *)allocator.allocate(sizeof(parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence__destroy(parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence__are_equal(const parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence * lhs, const parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence__copy(
  const parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence * input,
  parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(parking_robot_interfaces__action__DetectVehicle_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    parking_robot_interfaces__action__DetectVehicle_SendGoal_Request * data =
      (parking_robot_interfaces__action__DetectVehicle_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!parking_robot_interfaces__action__DetectVehicle_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__init(parking_robot_interfaces__action__DetectVehicle_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__fini(parking_robot_interfaces__action__DetectVehicle_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__are_equal(const parking_robot_interfaces__action__DetectVehicle_SendGoal_Response * lhs, const parking_robot_interfaces__action__DetectVehicle_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__copy(
  const parking_robot_interfaces__action__DetectVehicle_SendGoal_Response * input,
  parking_robot_interfaces__action__DetectVehicle_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

parking_robot_interfaces__action__DetectVehicle_SendGoal_Response *
parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__DetectVehicle_SendGoal_Response * msg = (parking_robot_interfaces__action__DetectVehicle_SendGoal_Response *)allocator.allocate(sizeof(parking_robot_interfaces__action__DetectVehicle_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parking_robot_interfaces__action__DetectVehicle_SendGoal_Response));
  bool success = parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__destroy(parking_robot_interfaces__action__DetectVehicle_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence__init(parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__DetectVehicle_SendGoal_Response * data = NULL;

  if (size) {
    data = (parking_robot_interfaces__action__DetectVehicle_SendGoal_Response *)allocator.zero_allocate(size, sizeof(parking_robot_interfaces__action__DetectVehicle_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__fini(&data[i - 1]);
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
parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence__fini(parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence * array)
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
      parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__fini(&array->data[i]);
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

parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence *
parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence * array = (parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence *)allocator.allocate(sizeof(parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence__destroy(parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence__are_equal(const parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence * lhs, const parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence__copy(
  const parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence * input,
  parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(parking_robot_interfaces__action__DetectVehicle_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    parking_robot_interfaces__action__DetectVehicle_SendGoal_Response * data =
      (parking_robot_interfaces__action__DetectVehicle_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!parking_robot_interfaces__action__DetectVehicle_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
parking_robot_interfaces__action__DetectVehicle_GetResult_Request__init(parking_robot_interfaces__action__DetectVehicle_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    parking_robot_interfaces__action__DetectVehicle_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
parking_robot_interfaces__action__DetectVehicle_GetResult_Request__fini(parking_robot_interfaces__action__DetectVehicle_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
parking_robot_interfaces__action__DetectVehicle_GetResult_Request__are_equal(const parking_robot_interfaces__action__DetectVehicle_GetResult_Request * lhs, const parking_robot_interfaces__action__DetectVehicle_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
parking_robot_interfaces__action__DetectVehicle_GetResult_Request__copy(
  const parking_robot_interfaces__action__DetectVehicle_GetResult_Request * input,
  parking_robot_interfaces__action__DetectVehicle_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

parking_robot_interfaces__action__DetectVehicle_GetResult_Request *
parking_robot_interfaces__action__DetectVehicle_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__DetectVehicle_GetResult_Request * msg = (parking_robot_interfaces__action__DetectVehicle_GetResult_Request *)allocator.allocate(sizeof(parking_robot_interfaces__action__DetectVehicle_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parking_robot_interfaces__action__DetectVehicle_GetResult_Request));
  bool success = parking_robot_interfaces__action__DetectVehicle_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parking_robot_interfaces__action__DetectVehicle_GetResult_Request__destroy(parking_robot_interfaces__action__DetectVehicle_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    parking_robot_interfaces__action__DetectVehicle_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence__init(parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__DetectVehicle_GetResult_Request * data = NULL;

  if (size) {
    data = (parking_robot_interfaces__action__DetectVehicle_GetResult_Request *)allocator.zero_allocate(size, sizeof(parking_robot_interfaces__action__DetectVehicle_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = parking_robot_interfaces__action__DetectVehicle_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        parking_robot_interfaces__action__DetectVehicle_GetResult_Request__fini(&data[i - 1]);
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
parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence__fini(parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence * array)
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
      parking_robot_interfaces__action__DetectVehicle_GetResult_Request__fini(&array->data[i]);
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

parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence *
parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence * array = (parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence *)allocator.allocate(sizeof(parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence__destroy(parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence__are_equal(const parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence * lhs, const parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!parking_robot_interfaces__action__DetectVehicle_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence__copy(
  const parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence * input,
  parking_robot_interfaces__action__DetectVehicle_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(parking_robot_interfaces__action__DetectVehicle_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    parking_robot_interfaces__action__DetectVehicle_GetResult_Request * data =
      (parking_robot_interfaces__action__DetectVehicle_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!parking_robot_interfaces__action__DetectVehicle_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          parking_robot_interfaces__action__DetectVehicle_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!parking_robot_interfaces__action__DetectVehicle_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "parking_robot_interfaces/action/detail/detect_vehicle__functions.h"

bool
parking_robot_interfaces__action__DetectVehicle_GetResult_Response__init(parking_robot_interfaces__action__DetectVehicle_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!parking_robot_interfaces__action__DetectVehicle_Result__init(&msg->result)) {
    parking_robot_interfaces__action__DetectVehicle_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
parking_robot_interfaces__action__DetectVehicle_GetResult_Response__fini(parking_robot_interfaces__action__DetectVehicle_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  parking_robot_interfaces__action__DetectVehicle_Result__fini(&msg->result);
}

bool
parking_robot_interfaces__action__DetectVehicle_GetResult_Response__are_equal(const parking_robot_interfaces__action__DetectVehicle_GetResult_Response * lhs, const parking_robot_interfaces__action__DetectVehicle_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!parking_robot_interfaces__action__DetectVehicle_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
parking_robot_interfaces__action__DetectVehicle_GetResult_Response__copy(
  const parking_robot_interfaces__action__DetectVehicle_GetResult_Response * input,
  parking_robot_interfaces__action__DetectVehicle_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!parking_robot_interfaces__action__DetectVehicle_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

parking_robot_interfaces__action__DetectVehicle_GetResult_Response *
parking_robot_interfaces__action__DetectVehicle_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__DetectVehicle_GetResult_Response * msg = (parking_robot_interfaces__action__DetectVehicle_GetResult_Response *)allocator.allocate(sizeof(parking_robot_interfaces__action__DetectVehicle_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parking_robot_interfaces__action__DetectVehicle_GetResult_Response));
  bool success = parking_robot_interfaces__action__DetectVehicle_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parking_robot_interfaces__action__DetectVehicle_GetResult_Response__destroy(parking_robot_interfaces__action__DetectVehicle_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    parking_robot_interfaces__action__DetectVehicle_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence__init(parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__DetectVehicle_GetResult_Response * data = NULL;

  if (size) {
    data = (parking_robot_interfaces__action__DetectVehicle_GetResult_Response *)allocator.zero_allocate(size, sizeof(parking_robot_interfaces__action__DetectVehicle_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = parking_robot_interfaces__action__DetectVehicle_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        parking_robot_interfaces__action__DetectVehicle_GetResult_Response__fini(&data[i - 1]);
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
parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence__fini(parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence * array)
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
      parking_robot_interfaces__action__DetectVehicle_GetResult_Response__fini(&array->data[i]);
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

parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence *
parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence * array = (parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence *)allocator.allocate(sizeof(parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence__destroy(parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence__are_equal(const parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence * lhs, const parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!parking_robot_interfaces__action__DetectVehicle_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence__copy(
  const parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence * input,
  parking_robot_interfaces__action__DetectVehicle_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(parking_robot_interfaces__action__DetectVehicle_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    parking_robot_interfaces__action__DetectVehicle_GetResult_Response * data =
      (parking_robot_interfaces__action__DetectVehicle_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!parking_robot_interfaces__action__DetectVehicle_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          parking_robot_interfaces__action__DetectVehicle_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!parking_robot_interfaces__action__DetectVehicle_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "parking_robot_interfaces/action/detail/detect_vehicle__functions.h"

bool
parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__init(parking_robot_interfaces__action__DetectVehicle_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!parking_robot_interfaces__action__DetectVehicle_Feedback__init(&msg->feedback)) {
    parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__fini(parking_robot_interfaces__action__DetectVehicle_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  parking_robot_interfaces__action__DetectVehicle_Feedback__fini(&msg->feedback);
}

bool
parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__are_equal(const parking_robot_interfaces__action__DetectVehicle_FeedbackMessage * lhs, const parking_robot_interfaces__action__DetectVehicle_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!parking_robot_interfaces__action__DetectVehicle_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__copy(
  const parking_robot_interfaces__action__DetectVehicle_FeedbackMessage * input,
  parking_robot_interfaces__action__DetectVehicle_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!parking_robot_interfaces__action__DetectVehicle_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

parking_robot_interfaces__action__DetectVehicle_FeedbackMessage *
parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__DetectVehicle_FeedbackMessage * msg = (parking_robot_interfaces__action__DetectVehicle_FeedbackMessage *)allocator.allocate(sizeof(parking_robot_interfaces__action__DetectVehicle_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parking_robot_interfaces__action__DetectVehicle_FeedbackMessage));
  bool success = parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__destroy(parking_robot_interfaces__action__DetectVehicle_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence__init(parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__DetectVehicle_FeedbackMessage * data = NULL;

  if (size) {
    data = (parking_robot_interfaces__action__DetectVehicle_FeedbackMessage *)allocator.zero_allocate(size, sizeof(parking_robot_interfaces__action__DetectVehicle_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__fini(&data[i - 1]);
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
parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence__fini(parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence * array)
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
      parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__fini(&array->data[i]);
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

parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence *
parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence * array = (parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence *)allocator.allocate(sizeof(parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence__destroy(parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence__are_equal(const parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence * lhs, const parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence__copy(
  const parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence * input,
  parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(parking_robot_interfaces__action__DetectVehicle_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    parking_robot_interfaces__action__DetectVehicle_FeedbackMessage * data =
      (parking_robot_interfaces__action__DetectVehicle_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!parking_robot_interfaces__action__DetectVehicle_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
