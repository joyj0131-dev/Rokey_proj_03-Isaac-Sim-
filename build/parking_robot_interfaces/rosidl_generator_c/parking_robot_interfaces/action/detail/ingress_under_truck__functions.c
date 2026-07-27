// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from parking_robot_interfaces:action/IngressUnderTruck.idl
// generated code does not contain a copyright notice
#include "parking_robot_interfaces/action/detail/ingress_under_truck__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
parking_robot_interfaces__action__IngressUnderTruck_Goal__init(parking_robot_interfaces__action__IngressUnderTruck_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // trough_index
  // forward_speed
  // return_speed
  return true;
}

void
parking_robot_interfaces__action__IngressUnderTruck_Goal__fini(parking_robot_interfaces__action__IngressUnderTruck_Goal * msg)
{
  if (!msg) {
    return;
  }
  // trough_index
  // forward_speed
  // return_speed
}

bool
parking_robot_interfaces__action__IngressUnderTruck_Goal__are_equal(const parking_robot_interfaces__action__IngressUnderTruck_Goal * lhs, const parking_robot_interfaces__action__IngressUnderTruck_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // trough_index
  if (lhs->trough_index != rhs->trough_index) {
    return false;
  }
  // forward_speed
  if (lhs->forward_speed != rhs->forward_speed) {
    return false;
  }
  // return_speed
  if (lhs->return_speed != rhs->return_speed) {
    return false;
  }
  return true;
}

bool
parking_robot_interfaces__action__IngressUnderTruck_Goal__copy(
  const parking_robot_interfaces__action__IngressUnderTruck_Goal * input,
  parking_robot_interfaces__action__IngressUnderTruck_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // trough_index
  output->trough_index = input->trough_index;
  // forward_speed
  output->forward_speed = input->forward_speed;
  // return_speed
  output->return_speed = input->return_speed;
  return true;
}

parking_robot_interfaces__action__IngressUnderTruck_Goal *
parking_robot_interfaces__action__IngressUnderTruck_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__IngressUnderTruck_Goal * msg = (parking_robot_interfaces__action__IngressUnderTruck_Goal *)allocator.allocate(sizeof(parking_robot_interfaces__action__IngressUnderTruck_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parking_robot_interfaces__action__IngressUnderTruck_Goal));
  bool success = parking_robot_interfaces__action__IngressUnderTruck_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parking_robot_interfaces__action__IngressUnderTruck_Goal__destroy(parking_robot_interfaces__action__IngressUnderTruck_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    parking_robot_interfaces__action__IngressUnderTruck_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence__init(parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__IngressUnderTruck_Goal * data = NULL;

  if (size) {
    data = (parking_robot_interfaces__action__IngressUnderTruck_Goal *)allocator.zero_allocate(size, sizeof(parking_robot_interfaces__action__IngressUnderTruck_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = parking_robot_interfaces__action__IngressUnderTruck_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        parking_robot_interfaces__action__IngressUnderTruck_Goal__fini(&data[i - 1]);
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
parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence__fini(parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence * array)
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
      parking_robot_interfaces__action__IngressUnderTruck_Goal__fini(&array->data[i]);
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

parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence *
parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence * array = (parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence *)allocator.allocate(sizeof(parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence__destroy(parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence__are_equal(const parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence * lhs, const parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!parking_robot_interfaces__action__IngressUnderTruck_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence__copy(
  const parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence * input,
  parking_robot_interfaces__action__IngressUnderTruck_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(parking_robot_interfaces__action__IngressUnderTruck_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    parking_robot_interfaces__action__IngressUnderTruck_Goal * data =
      (parking_robot_interfaces__action__IngressUnderTruck_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!parking_robot_interfaces__action__IngressUnderTruck_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          parking_robot_interfaces__action__IngressUnderTruck_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!parking_robot_interfaces__action__IngressUnderTruck_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stop_reason`
#include "rosidl_runtime_c/string_functions.h"

bool
parking_robot_interfaces__action__IngressUnderTruck_Result__init(parking_robot_interfaces__action__IngressUnderTruck_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // stop_reason
  if (!rosidl_runtime_c__String__init(&msg->stop_reason)) {
    parking_robot_interfaces__action__IngressUnderTruck_Result__fini(msg);
    return false;
  }
  // stop_x
  // target_axle_x
  // est_max_lateral_dev_m
  return true;
}

void
parking_robot_interfaces__action__IngressUnderTruck_Result__fini(parking_robot_interfaces__action__IngressUnderTruck_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
  // stop_reason
  rosidl_runtime_c__String__fini(&msg->stop_reason);
  // stop_x
  // target_axle_x
  // est_max_lateral_dev_m
}

bool
parking_robot_interfaces__action__IngressUnderTruck_Result__are_equal(const parking_robot_interfaces__action__IngressUnderTruck_Result * lhs, const parking_robot_interfaces__action__IngressUnderTruck_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // stop_reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->stop_reason), &(rhs->stop_reason)))
  {
    return false;
  }
  // stop_x
  if (lhs->stop_x != rhs->stop_x) {
    return false;
  }
  // target_axle_x
  if (lhs->target_axle_x != rhs->target_axle_x) {
    return false;
  }
  // est_max_lateral_dev_m
  if (lhs->est_max_lateral_dev_m != rhs->est_max_lateral_dev_m) {
    return false;
  }
  return true;
}

bool
parking_robot_interfaces__action__IngressUnderTruck_Result__copy(
  const parking_robot_interfaces__action__IngressUnderTruck_Result * input,
  parking_robot_interfaces__action__IngressUnderTruck_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // stop_reason
  if (!rosidl_runtime_c__String__copy(
      &(input->stop_reason), &(output->stop_reason)))
  {
    return false;
  }
  // stop_x
  output->stop_x = input->stop_x;
  // target_axle_x
  output->target_axle_x = input->target_axle_x;
  // est_max_lateral_dev_m
  output->est_max_lateral_dev_m = input->est_max_lateral_dev_m;
  return true;
}

parking_robot_interfaces__action__IngressUnderTruck_Result *
parking_robot_interfaces__action__IngressUnderTruck_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__IngressUnderTruck_Result * msg = (parking_robot_interfaces__action__IngressUnderTruck_Result *)allocator.allocate(sizeof(parking_robot_interfaces__action__IngressUnderTruck_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parking_robot_interfaces__action__IngressUnderTruck_Result));
  bool success = parking_robot_interfaces__action__IngressUnderTruck_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parking_robot_interfaces__action__IngressUnderTruck_Result__destroy(parking_robot_interfaces__action__IngressUnderTruck_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    parking_robot_interfaces__action__IngressUnderTruck_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence__init(parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__IngressUnderTruck_Result * data = NULL;

  if (size) {
    data = (parking_robot_interfaces__action__IngressUnderTruck_Result *)allocator.zero_allocate(size, sizeof(parking_robot_interfaces__action__IngressUnderTruck_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = parking_robot_interfaces__action__IngressUnderTruck_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        parking_robot_interfaces__action__IngressUnderTruck_Result__fini(&data[i - 1]);
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
parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence__fini(parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence * array)
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
      parking_robot_interfaces__action__IngressUnderTruck_Result__fini(&array->data[i]);
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

parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence *
parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence * array = (parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence *)allocator.allocate(sizeof(parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence__destroy(parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence__are_equal(const parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence * lhs, const parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!parking_robot_interfaces__action__IngressUnderTruck_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence__copy(
  const parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence * input,
  parking_robot_interfaces__action__IngressUnderTruck_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(parking_robot_interfaces__action__IngressUnderTruck_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    parking_robot_interfaces__action__IngressUnderTruck_Result * data =
      (parking_robot_interfaces__action__IngressUnderTruck_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!parking_robot_interfaces__action__IngressUnderTruck_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          parking_robot_interfaces__action__IngressUnderTruck_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!parking_robot_interfaces__action__IngressUnderTruck_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `phase`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
parking_robot_interfaces__action__IngressUnderTruck_Feedback__init(parking_robot_interfaces__action__IngressUnderTruck_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // phase
  if (!rosidl_runtime_c__String__init(&msg->phase)) {
    parking_robot_interfaces__action__IngressUnderTruck_Feedback__fini(msg);
    return false;
  }
  // current_x
  // troughs_seen
  // vy_cmd
  return true;
}

void
parking_robot_interfaces__action__IngressUnderTruck_Feedback__fini(parking_robot_interfaces__action__IngressUnderTruck_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // phase
  rosidl_runtime_c__String__fini(&msg->phase);
  // current_x
  // troughs_seen
  // vy_cmd
}

bool
parking_robot_interfaces__action__IngressUnderTruck_Feedback__are_equal(const parking_robot_interfaces__action__IngressUnderTruck_Feedback * lhs, const parking_robot_interfaces__action__IngressUnderTruck_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // phase
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->phase), &(rhs->phase)))
  {
    return false;
  }
  // current_x
  if (lhs->current_x != rhs->current_x) {
    return false;
  }
  // troughs_seen
  if (lhs->troughs_seen != rhs->troughs_seen) {
    return false;
  }
  // vy_cmd
  if (lhs->vy_cmd != rhs->vy_cmd) {
    return false;
  }
  return true;
}

bool
parking_robot_interfaces__action__IngressUnderTruck_Feedback__copy(
  const parking_robot_interfaces__action__IngressUnderTruck_Feedback * input,
  parking_robot_interfaces__action__IngressUnderTruck_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // phase
  if (!rosidl_runtime_c__String__copy(
      &(input->phase), &(output->phase)))
  {
    return false;
  }
  // current_x
  output->current_x = input->current_x;
  // troughs_seen
  output->troughs_seen = input->troughs_seen;
  // vy_cmd
  output->vy_cmd = input->vy_cmd;
  return true;
}

parking_robot_interfaces__action__IngressUnderTruck_Feedback *
parking_robot_interfaces__action__IngressUnderTruck_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__IngressUnderTruck_Feedback * msg = (parking_robot_interfaces__action__IngressUnderTruck_Feedback *)allocator.allocate(sizeof(parking_robot_interfaces__action__IngressUnderTruck_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parking_robot_interfaces__action__IngressUnderTruck_Feedback));
  bool success = parking_robot_interfaces__action__IngressUnderTruck_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parking_robot_interfaces__action__IngressUnderTruck_Feedback__destroy(parking_robot_interfaces__action__IngressUnderTruck_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    parking_robot_interfaces__action__IngressUnderTruck_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence__init(parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__IngressUnderTruck_Feedback * data = NULL;

  if (size) {
    data = (parking_robot_interfaces__action__IngressUnderTruck_Feedback *)allocator.zero_allocate(size, sizeof(parking_robot_interfaces__action__IngressUnderTruck_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = parking_robot_interfaces__action__IngressUnderTruck_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        parking_robot_interfaces__action__IngressUnderTruck_Feedback__fini(&data[i - 1]);
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
parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence__fini(parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence * array)
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
      parking_robot_interfaces__action__IngressUnderTruck_Feedback__fini(&array->data[i]);
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

parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence *
parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence * array = (parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence *)allocator.allocate(sizeof(parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence__destroy(parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence__are_equal(const parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence * lhs, const parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!parking_robot_interfaces__action__IngressUnderTruck_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence__copy(
  const parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence * input,
  parking_robot_interfaces__action__IngressUnderTruck_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(parking_robot_interfaces__action__IngressUnderTruck_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    parking_robot_interfaces__action__IngressUnderTruck_Feedback * data =
      (parking_robot_interfaces__action__IngressUnderTruck_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!parking_robot_interfaces__action__IngressUnderTruck_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          parking_robot_interfaces__action__IngressUnderTruck_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!parking_robot_interfaces__action__IngressUnderTruck_Feedback__copy(
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
// #include "parking_robot_interfaces/action/detail/ingress_under_truck__functions.h"

bool
parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__init(parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!parking_robot_interfaces__action__IngressUnderTruck_Goal__init(&msg->goal)) {
    parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__fini(parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  parking_robot_interfaces__action__IngressUnderTruck_Goal__fini(&msg->goal);
}

bool
parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__are_equal(const parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request * lhs, const parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request * rhs)
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
  if (!parking_robot_interfaces__action__IngressUnderTruck_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__copy(
  const parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request * input,
  parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request * output)
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
  if (!parking_robot_interfaces__action__IngressUnderTruck_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request *
parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request * msg = (parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request *)allocator.allocate(sizeof(parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request));
  bool success = parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__destroy(parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence__init(parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request * data = NULL;

  if (size) {
    data = (parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request *)allocator.zero_allocate(size, sizeof(parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__fini(&data[i - 1]);
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
parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence__fini(parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence * array)
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
      parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__fini(&array->data[i]);
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

parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence *
parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence * array = (parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence *)allocator.allocate(sizeof(parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence__destroy(parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence__are_equal(const parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence * lhs, const parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence__copy(
  const parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence * input,
  parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request * data =
      (parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Request__copy(
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
parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__init(parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__fini(parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__are_equal(const parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response * lhs, const parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response * rhs)
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
parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__copy(
  const parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response * input,
  parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response * output)
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

parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response *
parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response * msg = (parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response *)allocator.allocate(sizeof(parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response));
  bool success = parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__destroy(parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence__init(parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response * data = NULL;

  if (size) {
    data = (parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response *)allocator.zero_allocate(size, sizeof(parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__fini(&data[i - 1]);
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
parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence__fini(parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence * array)
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
      parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__fini(&array->data[i]);
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

parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence *
parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence * array = (parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence *)allocator.allocate(sizeof(parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence__destroy(parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence__are_equal(const parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence * lhs, const parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence__copy(
  const parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence * input,
  parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response * data =
      (parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!parking_robot_interfaces__action__IngressUnderTruck_SendGoal_Response__copy(
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
parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__init(parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__fini(parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__are_equal(const parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request * lhs, const parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request * rhs)
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
parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__copy(
  const parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request * input,
  parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request * output)
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

parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request *
parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request * msg = (parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request *)allocator.allocate(sizeof(parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request));
  bool success = parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__destroy(parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence__init(parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request * data = NULL;

  if (size) {
    data = (parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request *)allocator.zero_allocate(size, sizeof(parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__fini(&data[i - 1]);
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
parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence__fini(parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence * array)
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
      parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__fini(&array->data[i]);
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

parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence *
parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence * array = (parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence *)allocator.allocate(sizeof(parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence__destroy(parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence__are_equal(const parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence * lhs, const parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence__copy(
  const parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence * input,
  parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request * data =
      (parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!parking_robot_interfaces__action__IngressUnderTruck_GetResult_Request__copy(
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
// #include "parking_robot_interfaces/action/detail/ingress_under_truck__functions.h"

bool
parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__init(parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!parking_robot_interfaces__action__IngressUnderTruck_Result__init(&msg->result)) {
    parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__fini(parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  parking_robot_interfaces__action__IngressUnderTruck_Result__fini(&msg->result);
}

bool
parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__are_equal(const parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response * lhs, const parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!parking_robot_interfaces__action__IngressUnderTruck_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__copy(
  const parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response * input,
  parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!parking_robot_interfaces__action__IngressUnderTruck_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response *
parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response * msg = (parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response *)allocator.allocate(sizeof(parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response));
  bool success = parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__destroy(parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence__init(parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response * data = NULL;

  if (size) {
    data = (parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response *)allocator.zero_allocate(size, sizeof(parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__fini(&data[i - 1]);
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
parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence__fini(parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence * array)
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
      parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__fini(&array->data[i]);
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

parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence *
parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence * array = (parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence *)allocator.allocate(sizeof(parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence__destroy(parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence__are_equal(const parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence * lhs, const parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence__copy(
  const parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence * input,
  parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response * data =
      (parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!parking_robot_interfaces__action__IngressUnderTruck_GetResult_Response__copy(
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
// #include "parking_robot_interfaces/action/detail/ingress_under_truck__functions.h"

bool
parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__init(parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!parking_robot_interfaces__action__IngressUnderTruck_Feedback__init(&msg->feedback)) {
    parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__fini(parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  parking_robot_interfaces__action__IngressUnderTruck_Feedback__fini(&msg->feedback);
}

bool
parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__are_equal(const parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage * lhs, const parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage * rhs)
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
  if (!parking_robot_interfaces__action__IngressUnderTruck_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__copy(
  const parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage * input,
  parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage * output)
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
  if (!parking_robot_interfaces__action__IngressUnderTruck_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage *
parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage * msg = (parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage *)allocator.allocate(sizeof(parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage));
  bool success = parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__destroy(parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence__init(parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage * data = NULL;

  if (size) {
    data = (parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage *)allocator.zero_allocate(size, sizeof(parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__fini(&data[i - 1]);
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
parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence__fini(parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence * array)
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
      parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__fini(&array->data[i]);
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

parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence *
parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence * array = (parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence *)allocator.allocate(sizeof(parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence__destroy(parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence__are_equal(const parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence * lhs, const parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence__copy(
  const parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence * input,
  parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage * data =
      (parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!parking_robot_interfaces__action__IngressUnderTruck_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
