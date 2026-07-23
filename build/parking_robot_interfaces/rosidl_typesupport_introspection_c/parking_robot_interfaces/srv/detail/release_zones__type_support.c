// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from parking_robot_interfaces:srv/ReleaseZones.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "parking_robot_interfaces/srv/detail/release_zones__rosidl_typesupport_introspection_c.h"
#include "parking_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "parking_robot_interfaces/srv/detail/release_zones__functions.h"
#include "parking_robot_interfaces/srv/detail/release_zones__struct.h"


// Include directives for member types
// Member `robot_id`
// Member `task_id`
// Member `zone_ids`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__ReleaseZones_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  parking_robot_interfaces__srv__ReleaseZones_Request__init(message_memory);
}

void parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__ReleaseZones_Request_fini_function(void * message_memory)
{
  parking_robot_interfaces__srv__ReleaseZones_Request__fini(message_memory);
}

size_t parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__size_function__ReleaseZones_Request__zone_ids(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__get_const_function__ReleaseZones_Request__zone_ids(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__get_function__ReleaseZones_Request__zone_ids(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__fetch_function__ReleaseZones_Request__zone_ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__get_const_function__ReleaseZones_Request__zone_ids(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__assign_function__ReleaseZones_Request__zone_ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__get_function__ReleaseZones_Request__zone_ids(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__resize_function__ReleaseZones_Request__zone_ids(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__ReleaseZones_Request_message_member_array[3] = {
  {
    "robot_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(parking_robot_interfaces__srv__ReleaseZones_Request, robot_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "task_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(parking_robot_interfaces__srv__ReleaseZones_Request, task_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "zone_ids",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(parking_robot_interfaces__srv__ReleaseZones_Request, zone_ids),  // bytes offset in struct
    NULL,  // default value
    parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__size_function__ReleaseZones_Request__zone_ids,  // size() function pointer
    parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__get_const_function__ReleaseZones_Request__zone_ids,  // get_const(index) function pointer
    parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__get_function__ReleaseZones_Request__zone_ids,  // get(index) function pointer
    parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__fetch_function__ReleaseZones_Request__zone_ids,  // fetch(index, &value) function pointer
    parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__assign_function__ReleaseZones_Request__zone_ids,  // assign(index, value) function pointer
    parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__resize_function__ReleaseZones_Request__zone_ids  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__ReleaseZones_Request_message_members = {
  "parking_robot_interfaces__srv",  // message namespace
  "ReleaseZones_Request",  // message name
  3,  // number of fields
  sizeof(parking_robot_interfaces__srv__ReleaseZones_Request),
  parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__ReleaseZones_Request_message_member_array,  // message members
  parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__ReleaseZones_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__ReleaseZones_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__ReleaseZones_Request_message_type_support_handle = {
  0,
  &parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__ReleaseZones_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_parking_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, parking_robot_interfaces, srv, ReleaseZones_Request)() {
  if (!parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__ReleaseZones_Request_message_type_support_handle.typesupport_identifier) {
    parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__ReleaseZones_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &parking_robot_interfaces__srv__ReleaseZones_Request__rosidl_typesupport_introspection_c__ReleaseZones_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "parking_robot_interfaces/srv/detail/release_zones__rosidl_typesupport_introspection_c.h"
// already included above
// #include "parking_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "parking_robot_interfaces/srv/detail/release_zones__functions.h"
// already included above
// #include "parking_robot_interfaces/srv/detail/release_zones__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void parking_robot_interfaces__srv__ReleaseZones_Response__rosidl_typesupport_introspection_c__ReleaseZones_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  parking_robot_interfaces__srv__ReleaseZones_Response__init(message_memory);
}

void parking_robot_interfaces__srv__ReleaseZones_Response__rosidl_typesupport_introspection_c__ReleaseZones_Response_fini_function(void * message_memory)
{
  parking_robot_interfaces__srv__ReleaseZones_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember parking_robot_interfaces__srv__ReleaseZones_Response__rosidl_typesupport_introspection_c__ReleaseZones_Response_message_member_array[1] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(parking_robot_interfaces__srv__ReleaseZones_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers parking_robot_interfaces__srv__ReleaseZones_Response__rosidl_typesupport_introspection_c__ReleaseZones_Response_message_members = {
  "parking_robot_interfaces__srv",  // message namespace
  "ReleaseZones_Response",  // message name
  1,  // number of fields
  sizeof(parking_robot_interfaces__srv__ReleaseZones_Response),
  parking_robot_interfaces__srv__ReleaseZones_Response__rosidl_typesupport_introspection_c__ReleaseZones_Response_message_member_array,  // message members
  parking_robot_interfaces__srv__ReleaseZones_Response__rosidl_typesupport_introspection_c__ReleaseZones_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  parking_robot_interfaces__srv__ReleaseZones_Response__rosidl_typesupport_introspection_c__ReleaseZones_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t parking_robot_interfaces__srv__ReleaseZones_Response__rosidl_typesupport_introspection_c__ReleaseZones_Response_message_type_support_handle = {
  0,
  &parking_robot_interfaces__srv__ReleaseZones_Response__rosidl_typesupport_introspection_c__ReleaseZones_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_parking_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, parking_robot_interfaces, srv, ReleaseZones_Response)() {
  if (!parking_robot_interfaces__srv__ReleaseZones_Response__rosidl_typesupport_introspection_c__ReleaseZones_Response_message_type_support_handle.typesupport_identifier) {
    parking_robot_interfaces__srv__ReleaseZones_Response__rosidl_typesupport_introspection_c__ReleaseZones_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &parking_robot_interfaces__srv__ReleaseZones_Response__rosidl_typesupport_introspection_c__ReleaseZones_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "parking_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "parking_robot_interfaces/srv/detail/release_zones__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers parking_robot_interfaces__srv__detail__release_zones__rosidl_typesupport_introspection_c__ReleaseZones_service_members = {
  "parking_robot_interfaces__srv",  // service namespace
  "ReleaseZones",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // parking_robot_interfaces__srv__detail__release_zones__rosidl_typesupport_introspection_c__ReleaseZones_Request_message_type_support_handle,
  NULL  // response message
  // parking_robot_interfaces__srv__detail__release_zones__rosidl_typesupport_introspection_c__ReleaseZones_Response_message_type_support_handle
};

static rosidl_service_type_support_t parking_robot_interfaces__srv__detail__release_zones__rosidl_typesupport_introspection_c__ReleaseZones_service_type_support_handle = {
  0,
  &parking_robot_interfaces__srv__detail__release_zones__rosidl_typesupport_introspection_c__ReleaseZones_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, parking_robot_interfaces, srv, ReleaseZones_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, parking_robot_interfaces, srv, ReleaseZones_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_parking_robot_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, parking_robot_interfaces, srv, ReleaseZones)() {
  if (!parking_robot_interfaces__srv__detail__release_zones__rosidl_typesupport_introspection_c__ReleaseZones_service_type_support_handle.typesupport_identifier) {
    parking_robot_interfaces__srv__detail__release_zones__rosidl_typesupport_introspection_c__ReleaseZones_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)parking_robot_interfaces__srv__detail__release_zones__rosidl_typesupport_introspection_c__ReleaseZones_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, parking_robot_interfaces, srv, ReleaseZones_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, parking_robot_interfaces, srv, ReleaseZones_Response)()->data;
  }

  return &parking_robot_interfaces__srv__detail__release_zones__rosidl_typesupport_introspection_c__ReleaseZones_service_type_support_handle;
}
