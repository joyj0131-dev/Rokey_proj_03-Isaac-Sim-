// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from parking_robot_interfaces:srv/FindEmptySlot.idl
// generated code does not contain a copyright notice
#include "parking_robot_interfaces/srv/detail/find_empty_slot__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "parking_robot_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "parking_robot_interfaces/srv/detail/find_empty_slot__struct.h"
#include "parking_robot_interfaces/srv/detail/find_empty_slot__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _FindEmptySlot_Request__ros_msg_type = parking_robot_interfaces__srv__FindEmptySlot_Request;

static bool _FindEmptySlot_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _FindEmptySlot_Request__ros_msg_type * ros_message = static_cast<const _FindEmptySlot_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: vehicle_length
  {
    cdr << ros_message->vehicle_length;
  }

  // Field name: vehicle_width
  {
    cdr << ros_message->vehicle_width;
  }

  return true;
}

static bool _FindEmptySlot_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _FindEmptySlot_Request__ros_msg_type * ros_message = static_cast<_FindEmptySlot_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: vehicle_length
  {
    cdr >> ros_message->vehicle_length;
  }

  // Field name: vehicle_width
  {
    cdr >> ros_message->vehicle_width;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_parking_robot_interfaces
size_t get_serialized_size_parking_robot_interfaces__srv__FindEmptySlot_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _FindEmptySlot_Request__ros_msg_type * ros_message = static_cast<const _FindEmptySlot_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name vehicle_length
  {
    size_t item_size = sizeof(ros_message->vehicle_length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name vehicle_width
  {
    size_t item_size = sizeof(ros_message->vehicle_width);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _FindEmptySlot_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_parking_robot_interfaces__srv__FindEmptySlot_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_parking_robot_interfaces
size_t max_serialized_size_parking_robot_interfaces__srv__FindEmptySlot_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: vehicle_length
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: vehicle_width
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = parking_robot_interfaces__srv__FindEmptySlot_Request;
    is_plain =
      (
      offsetof(DataType, vehicle_width) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _FindEmptySlot_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_parking_robot_interfaces__srv__FindEmptySlot_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_FindEmptySlot_Request = {
  "parking_robot_interfaces::srv",
  "FindEmptySlot_Request",
  _FindEmptySlot_Request__cdr_serialize,
  _FindEmptySlot_Request__cdr_deserialize,
  _FindEmptySlot_Request__get_serialized_size,
  _FindEmptySlot_Request__max_serialized_size
};

static rosidl_message_type_support_t _FindEmptySlot_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_FindEmptySlot_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, parking_robot_interfaces, srv, FindEmptySlot_Request)() {
  return &_FindEmptySlot_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "parking_robot_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "parking_robot_interfaces/srv/detail/find_empty_slot__struct.h"
// already included above
// #include "parking_robot_interfaces/srv/detail/find_empty_slot__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "geometry_msgs/msg/detail/pose__functions.h"  // slot_pose
#include "rosidl_runtime_c/string.h"  // slot_id
#include "rosidl_runtime_c/string_functions.h"  // slot_id

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_parking_robot_interfaces
size_t get_serialized_size_geometry_msgs__msg__Pose(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_parking_robot_interfaces
size_t max_serialized_size_geometry_msgs__msg__Pose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_parking_robot_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Pose)();


using _FindEmptySlot_Response__ros_msg_type = parking_robot_interfaces__srv__FindEmptySlot_Response;

static bool _FindEmptySlot_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _FindEmptySlot_Response__ros_msg_type * ros_message = static_cast<const _FindEmptySlot_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  // Field name: slot_id
  {
    const rosidl_runtime_c__String * str = &ros_message->slot_id;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: slot_pose
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Pose
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->slot_pose, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _FindEmptySlot_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _FindEmptySlot_Response__ros_msg_type * ros_message = static_cast<_FindEmptySlot_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  // Field name: slot_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->slot_id.data) {
      rosidl_runtime_c__String__init(&ros_message->slot_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->slot_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'slot_id'\n");
      return false;
    }
  }

  // Field name: slot_pose
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Pose
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->slot_pose))
    {
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_parking_robot_interfaces
size_t get_serialized_size_parking_robot_interfaces__srv__FindEmptySlot_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _FindEmptySlot_Response__ros_msg_type * ros_message = static_cast<const _FindEmptySlot_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name slot_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->slot_id.size + 1);
  // field.name slot_pose

  current_alignment += get_serialized_size_geometry_msgs__msg__Pose(
    &(ros_message->slot_pose), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _FindEmptySlot_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_parking_robot_interfaces__srv__FindEmptySlot_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_parking_robot_interfaces
size_t max_serialized_size_parking_robot_interfaces__srv__FindEmptySlot_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: success
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: slot_id
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: slot_pose
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Pose(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = parking_robot_interfaces__srv__FindEmptySlot_Response;
    is_plain =
      (
      offsetof(DataType, slot_pose) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _FindEmptySlot_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_parking_robot_interfaces__srv__FindEmptySlot_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_FindEmptySlot_Response = {
  "parking_robot_interfaces::srv",
  "FindEmptySlot_Response",
  _FindEmptySlot_Response__cdr_serialize,
  _FindEmptySlot_Response__cdr_deserialize,
  _FindEmptySlot_Response__get_serialized_size,
  _FindEmptySlot_Response__max_serialized_size
};

static rosidl_message_type_support_t _FindEmptySlot_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_FindEmptySlot_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, parking_robot_interfaces, srv, FindEmptySlot_Response)() {
  return &_FindEmptySlot_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "parking_robot_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "parking_robot_interfaces/srv/find_empty_slot.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t FindEmptySlot__callbacks = {
  "parking_robot_interfaces::srv",
  "FindEmptySlot",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, parking_robot_interfaces, srv, FindEmptySlot_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, parking_robot_interfaces, srv, FindEmptySlot_Response)(),
};

static rosidl_service_type_support_t FindEmptySlot__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &FindEmptySlot__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, parking_robot_interfaces, srv, FindEmptySlot)() {
  return &FindEmptySlot__handle;
}

#if defined(__cplusplus)
}
#endif
