// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from parking_robot_interfaces:srv/FindEmptySlot.idl
// generated code does not contain a copyright notice
#include "parking_robot_interfaces/srv/detail/find_empty_slot__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
parking_robot_interfaces__srv__FindEmptySlot_Request__init(parking_robot_interfaces__srv__FindEmptySlot_Request * msg)
{
  if (!msg) {
    return false;
  }
  // vehicle_length
  // vehicle_width
  return true;
}

void
parking_robot_interfaces__srv__FindEmptySlot_Request__fini(parking_robot_interfaces__srv__FindEmptySlot_Request * msg)
{
  if (!msg) {
    return;
  }
  // vehicle_length
  // vehicle_width
}

bool
parking_robot_interfaces__srv__FindEmptySlot_Request__are_equal(const parking_robot_interfaces__srv__FindEmptySlot_Request * lhs, const parking_robot_interfaces__srv__FindEmptySlot_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // vehicle_length
  if (lhs->vehicle_length != rhs->vehicle_length) {
    return false;
  }
  // vehicle_width
  if (lhs->vehicle_width != rhs->vehicle_width) {
    return false;
  }
  return true;
}

bool
parking_robot_interfaces__srv__FindEmptySlot_Request__copy(
  const parking_robot_interfaces__srv__FindEmptySlot_Request * input,
  parking_robot_interfaces__srv__FindEmptySlot_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // vehicle_length
  output->vehicle_length = input->vehicle_length;
  // vehicle_width
  output->vehicle_width = input->vehicle_width;
  return true;
}

parking_robot_interfaces__srv__FindEmptySlot_Request *
parking_robot_interfaces__srv__FindEmptySlot_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__srv__FindEmptySlot_Request * msg = (parking_robot_interfaces__srv__FindEmptySlot_Request *)allocator.allocate(sizeof(parking_robot_interfaces__srv__FindEmptySlot_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parking_robot_interfaces__srv__FindEmptySlot_Request));
  bool success = parking_robot_interfaces__srv__FindEmptySlot_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parking_robot_interfaces__srv__FindEmptySlot_Request__destroy(parking_robot_interfaces__srv__FindEmptySlot_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    parking_robot_interfaces__srv__FindEmptySlot_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence__init(parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__srv__FindEmptySlot_Request * data = NULL;

  if (size) {
    data = (parking_robot_interfaces__srv__FindEmptySlot_Request *)allocator.zero_allocate(size, sizeof(parking_robot_interfaces__srv__FindEmptySlot_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = parking_robot_interfaces__srv__FindEmptySlot_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        parking_robot_interfaces__srv__FindEmptySlot_Request__fini(&data[i - 1]);
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
parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence__fini(parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence * array)
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
      parking_robot_interfaces__srv__FindEmptySlot_Request__fini(&array->data[i]);
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

parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence *
parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence * array = (parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence *)allocator.allocate(sizeof(parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence__destroy(parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence__are_equal(const parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence * lhs, const parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!parking_robot_interfaces__srv__FindEmptySlot_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence__copy(
  const parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence * input,
  parking_robot_interfaces__srv__FindEmptySlot_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(parking_robot_interfaces__srv__FindEmptySlot_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    parking_robot_interfaces__srv__FindEmptySlot_Request * data =
      (parking_robot_interfaces__srv__FindEmptySlot_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!parking_robot_interfaces__srv__FindEmptySlot_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          parking_robot_interfaces__srv__FindEmptySlot_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!parking_robot_interfaces__srv__FindEmptySlot_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `slot_id`
#include "rosidl_runtime_c/string_functions.h"
// Member `slot_pose`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
parking_robot_interfaces__srv__FindEmptySlot_Response__init(parking_robot_interfaces__srv__FindEmptySlot_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // slot_id
  if (!rosidl_runtime_c__String__init(&msg->slot_id)) {
    parking_robot_interfaces__srv__FindEmptySlot_Response__fini(msg);
    return false;
  }
  // slot_pose
  if (!geometry_msgs__msg__Pose__init(&msg->slot_pose)) {
    parking_robot_interfaces__srv__FindEmptySlot_Response__fini(msg);
    return false;
  }
  return true;
}

void
parking_robot_interfaces__srv__FindEmptySlot_Response__fini(parking_robot_interfaces__srv__FindEmptySlot_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // slot_id
  rosidl_runtime_c__String__fini(&msg->slot_id);
  // slot_pose
  geometry_msgs__msg__Pose__fini(&msg->slot_pose);
}

bool
parking_robot_interfaces__srv__FindEmptySlot_Response__are_equal(const parking_robot_interfaces__srv__FindEmptySlot_Response * lhs, const parking_robot_interfaces__srv__FindEmptySlot_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // slot_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->slot_id), &(rhs->slot_id)))
  {
    return false;
  }
  // slot_pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->slot_pose), &(rhs->slot_pose)))
  {
    return false;
  }
  return true;
}

bool
parking_robot_interfaces__srv__FindEmptySlot_Response__copy(
  const parking_robot_interfaces__srv__FindEmptySlot_Response * input,
  parking_robot_interfaces__srv__FindEmptySlot_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // slot_id
  if (!rosidl_runtime_c__String__copy(
      &(input->slot_id), &(output->slot_id)))
  {
    return false;
  }
  // slot_pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->slot_pose), &(output->slot_pose)))
  {
    return false;
  }
  return true;
}

parking_robot_interfaces__srv__FindEmptySlot_Response *
parking_robot_interfaces__srv__FindEmptySlot_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__srv__FindEmptySlot_Response * msg = (parking_robot_interfaces__srv__FindEmptySlot_Response *)allocator.allocate(sizeof(parking_robot_interfaces__srv__FindEmptySlot_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parking_robot_interfaces__srv__FindEmptySlot_Response));
  bool success = parking_robot_interfaces__srv__FindEmptySlot_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parking_robot_interfaces__srv__FindEmptySlot_Response__destroy(parking_robot_interfaces__srv__FindEmptySlot_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    parking_robot_interfaces__srv__FindEmptySlot_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence__init(parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__srv__FindEmptySlot_Response * data = NULL;

  if (size) {
    data = (parking_robot_interfaces__srv__FindEmptySlot_Response *)allocator.zero_allocate(size, sizeof(parking_robot_interfaces__srv__FindEmptySlot_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = parking_robot_interfaces__srv__FindEmptySlot_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        parking_robot_interfaces__srv__FindEmptySlot_Response__fini(&data[i - 1]);
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
parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence__fini(parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence * array)
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
      parking_robot_interfaces__srv__FindEmptySlot_Response__fini(&array->data[i]);
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

parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence *
parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence * array = (parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence *)allocator.allocate(sizeof(parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence__destroy(parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence__are_equal(const parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence * lhs, const parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!parking_robot_interfaces__srv__FindEmptySlot_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence__copy(
  const parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence * input,
  parking_robot_interfaces__srv__FindEmptySlot_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(parking_robot_interfaces__srv__FindEmptySlot_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    parking_robot_interfaces__srv__FindEmptySlot_Response * data =
      (parking_robot_interfaces__srv__FindEmptySlot_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!parking_robot_interfaces__srv__FindEmptySlot_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          parking_robot_interfaces__srv__FindEmptySlot_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!parking_robot_interfaces__srv__FindEmptySlot_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
