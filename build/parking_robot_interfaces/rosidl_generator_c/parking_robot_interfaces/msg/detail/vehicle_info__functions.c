// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from parking_robot_interfaces:msg/VehicleInfo.idl
// generated code does not contain a copyright notice
#include "parking_robot_interfaces/msg/detail/vehicle_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
parking_robot_interfaces__msg__VehicleInfo__init(parking_robot_interfaces__msg__VehicleInfo * msg)
{
  if (!msg) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    parking_robot_interfaces__msg__VehicleInfo__fini(msg);
    return false;
  }
  // length
  // width
  // height
  return true;
}

void
parking_robot_interfaces__msg__VehicleInfo__fini(parking_robot_interfaces__msg__VehicleInfo * msg)
{
  if (!msg) {
    return;
  }
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // length
  // width
  // height
}

bool
parking_robot_interfaces__msg__VehicleInfo__are_equal(const parking_robot_interfaces__msg__VehicleInfo * lhs, const parking_robot_interfaces__msg__VehicleInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // length
  if (lhs->length != rhs->length) {
    return false;
  }
  // width
  if (lhs->width != rhs->width) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  return true;
}

bool
parking_robot_interfaces__msg__VehicleInfo__copy(
  const parking_robot_interfaces__msg__VehicleInfo * input,
  parking_robot_interfaces__msg__VehicleInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // length
  output->length = input->length;
  // width
  output->width = input->width;
  // height
  output->height = input->height;
  return true;
}

parking_robot_interfaces__msg__VehicleInfo *
parking_robot_interfaces__msg__VehicleInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__msg__VehicleInfo * msg = (parking_robot_interfaces__msg__VehicleInfo *)allocator.allocate(sizeof(parking_robot_interfaces__msg__VehicleInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parking_robot_interfaces__msg__VehicleInfo));
  bool success = parking_robot_interfaces__msg__VehicleInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parking_robot_interfaces__msg__VehicleInfo__destroy(parking_robot_interfaces__msg__VehicleInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    parking_robot_interfaces__msg__VehicleInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
parking_robot_interfaces__msg__VehicleInfo__Sequence__init(parking_robot_interfaces__msg__VehicleInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__msg__VehicleInfo * data = NULL;

  if (size) {
    data = (parking_robot_interfaces__msg__VehicleInfo *)allocator.zero_allocate(size, sizeof(parking_robot_interfaces__msg__VehicleInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = parking_robot_interfaces__msg__VehicleInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        parking_robot_interfaces__msg__VehicleInfo__fini(&data[i - 1]);
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
parking_robot_interfaces__msg__VehicleInfo__Sequence__fini(parking_robot_interfaces__msg__VehicleInfo__Sequence * array)
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
      parking_robot_interfaces__msg__VehicleInfo__fini(&array->data[i]);
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

parking_robot_interfaces__msg__VehicleInfo__Sequence *
parking_robot_interfaces__msg__VehicleInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__msg__VehicleInfo__Sequence * array = (parking_robot_interfaces__msg__VehicleInfo__Sequence *)allocator.allocate(sizeof(parking_robot_interfaces__msg__VehicleInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = parking_robot_interfaces__msg__VehicleInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
parking_robot_interfaces__msg__VehicleInfo__Sequence__destroy(parking_robot_interfaces__msg__VehicleInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    parking_robot_interfaces__msg__VehicleInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
parking_robot_interfaces__msg__VehicleInfo__Sequence__are_equal(const parking_robot_interfaces__msg__VehicleInfo__Sequence * lhs, const parking_robot_interfaces__msg__VehicleInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!parking_robot_interfaces__msg__VehicleInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
parking_robot_interfaces__msg__VehicleInfo__Sequence__copy(
  const parking_robot_interfaces__msg__VehicleInfo__Sequence * input,
  parking_robot_interfaces__msg__VehicleInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(parking_robot_interfaces__msg__VehicleInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    parking_robot_interfaces__msg__VehicleInfo * data =
      (parking_robot_interfaces__msg__VehicleInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!parking_robot_interfaces__msg__VehicleInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          parking_robot_interfaces__msg__VehicleInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!parking_robot_interfaces__msg__VehicleInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
