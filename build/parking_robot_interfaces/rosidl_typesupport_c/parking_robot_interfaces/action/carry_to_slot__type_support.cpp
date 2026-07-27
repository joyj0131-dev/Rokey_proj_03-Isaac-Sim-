// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from parking_robot_interfaces:action/CarryToSlot.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "parking_robot_interfaces/action/detail/carry_to_slot__struct.h"
#include "parking_robot_interfaces/action/detail/carry_to_slot__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace parking_robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _CarryToSlot_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CarryToSlot_Goal_type_support_ids_t;

static const _CarryToSlot_Goal_type_support_ids_t _CarryToSlot_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _CarryToSlot_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CarryToSlot_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CarryToSlot_Goal_type_support_symbol_names_t _CarryToSlot_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, parking_robot_interfaces, action, CarryToSlot_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, parking_robot_interfaces, action, CarryToSlot_Goal)),
  }
};

typedef struct _CarryToSlot_Goal_type_support_data_t
{
  void * data[2];
} _CarryToSlot_Goal_type_support_data_t;

static _CarryToSlot_Goal_type_support_data_t _CarryToSlot_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CarryToSlot_Goal_message_typesupport_map = {
  2,
  "parking_robot_interfaces",
  &_CarryToSlot_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_CarryToSlot_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_CarryToSlot_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CarryToSlot_Goal_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CarryToSlot_Goal_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace parking_robot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, parking_robot_interfaces, action, CarryToSlot_Goal)() {
  return &::parking_robot_interfaces::action::rosidl_typesupport_c::CarryToSlot_Goal_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "parking_robot_interfaces/action/detail/carry_to_slot__struct.h"
// already included above
// #include "parking_robot_interfaces/action/detail/carry_to_slot__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace parking_robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _CarryToSlot_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CarryToSlot_Result_type_support_ids_t;

static const _CarryToSlot_Result_type_support_ids_t _CarryToSlot_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _CarryToSlot_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CarryToSlot_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CarryToSlot_Result_type_support_symbol_names_t _CarryToSlot_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, parking_robot_interfaces, action, CarryToSlot_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, parking_robot_interfaces, action, CarryToSlot_Result)),
  }
};

typedef struct _CarryToSlot_Result_type_support_data_t
{
  void * data[2];
} _CarryToSlot_Result_type_support_data_t;

static _CarryToSlot_Result_type_support_data_t _CarryToSlot_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CarryToSlot_Result_message_typesupport_map = {
  2,
  "parking_robot_interfaces",
  &_CarryToSlot_Result_message_typesupport_ids.typesupport_identifier[0],
  &_CarryToSlot_Result_message_typesupport_symbol_names.symbol_name[0],
  &_CarryToSlot_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CarryToSlot_Result_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CarryToSlot_Result_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace parking_robot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, parking_robot_interfaces, action, CarryToSlot_Result)() {
  return &::parking_robot_interfaces::action::rosidl_typesupport_c::CarryToSlot_Result_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "parking_robot_interfaces/action/detail/carry_to_slot__struct.h"
// already included above
// #include "parking_robot_interfaces/action/detail/carry_to_slot__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace parking_robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _CarryToSlot_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CarryToSlot_Feedback_type_support_ids_t;

static const _CarryToSlot_Feedback_type_support_ids_t _CarryToSlot_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _CarryToSlot_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CarryToSlot_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CarryToSlot_Feedback_type_support_symbol_names_t _CarryToSlot_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, parking_robot_interfaces, action, CarryToSlot_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, parking_robot_interfaces, action, CarryToSlot_Feedback)),
  }
};

typedef struct _CarryToSlot_Feedback_type_support_data_t
{
  void * data[2];
} _CarryToSlot_Feedback_type_support_data_t;

static _CarryToSlot_Feedback_type_support_data_t _CarryToSlot_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CarryToSlot_Feedback_message_typesupport_map = {
  2,
  "parking_robot_interfaces",
  &_CarryToSlot_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_CarryToSlot_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_CarryToSlot_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CarryToSlot_Feedback_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CarryToSlot_Feedback_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace parking_robot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, parking_robot_interfaces, action, CarryToSlot_Feedback)() {
  return &::parking_robot_interfaces::action::rosidl_typesupport_c::CarryToSlot_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "parking_robot_interfaces/action/detail/carry_to_slot__struct.h"
// already included above
// #include "parking_robot_interfaces/action/detail/carry_to_slot__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace parking_robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _CarryToSlot_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CarryToSlot_SendGoal_Request_type_support_ids_t;

static const _CarryToSlot_SendGoal_Request_type_support_ids_t _CarryToSlot_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _CarryToSlot_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CarryToSlot_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CarryToSlot_SendGoal_Request_type_support_symbol_names_t _CarryToSlot_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, parking_robot_interfaces, action, CarryToSlot_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, parking_robot_interfaces, action, CarryToSlot_SendGoal_Request)),
  }
};

typedef struct _CarryToSlot_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _CarryToSlot_SendGoal_Request_type_support_data_t;

static _CarryToSlot_SendGoal_Request_type_support_data_t _CarryToSlot_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CarryToSlot_SendGoal_Request_message_typesupport_map = {
  2,
  "parking_robot_interfaces",
  &_CarryToSlot_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_CarryToSlot_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_CarryToSlot_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CarryToSlot_SendGoal_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CarryToSlot_SendGoal_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace parking_robot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, parking_robot_interfaces, action, CarryToSlot_SendGoal_Request)() {
  return &::parking_robot_interfaces::action::rosidl_typesupport_c::CarryToSlot_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "parking_robot_interfaces/action/detail/carry_to_slot__struct.h"
// already included above
// #include "parking_robot_interfaces/action/detail/carry_to_slot__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace parking_robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _CarryToSlot_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CarryToSlot_SendGoal_Response_type_support_ids_t;

static const _CarryToSlot_SendGoal_Response_type_support_ids_t _CarryToSlot_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _CarryToSlot_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CarryToSlot_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CarryToSlot_SendGoal_Response_type_support_symbol_names_t _CarryToSlot_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, parking_robot_interfaces, action, CarryToSlot_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, parking_robot_interfaces, action, CarryToSlot_SendGoal_Response)),
  }
};

typedef struct _CarryToSlot_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _CarryToSlot_SendGoal_Response_type_support_data_t;

static _CarryToSlot_SendGoal_Response_type_support_data_t _CarryToSlot_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CarryToSlot_SendGoal_Response_message_typesupport_map = {
  2,
  "parking_robot_interfaces",
  &_CarryToSlot_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_CarryToSlot_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_CarryToSlot_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CarryToSlot_SendGoal_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CarryToSlot_SendGoal_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace parking_robot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, parking_robot_interfaces, action, CarryToSlot_SendGoal_Response)() {
  return &::parking_robot_interfaces::action::rosidl_typesupport_c::CarryToSlot_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "parking_robot_interfaces/action/detail/carry_to_slot__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace parking_robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _CarryToSlot_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CarryToSlot_SendGoal_type_support_ids_t;

static const _CarryToSlot_SendGoal_type_support_ids_t _CarryToSlot_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _CarryToSlot_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CarryToSlot_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CarryToSlot_SendGoal_type_support_symbol_names_t _CarryToSlot_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, parking_robot_interfaces, action, CarryToSlot_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, parking_robot_interfaces, action, CarryToSlot_SendGoal)),
  }
};

typedef struct _CarryToSlot_SendGoal_type_support_data_t
{
  void * data[2];
} _CarryToSlot_SendGoal_type_support_data_t;

static _CarryToSlot_SendGoal_type_support_data_t _CarryToSlot_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CarryToSlot_SendGoal_service_typesupport_map = {
  2,
  "parking_robot_interfaces",
  &_CarryToSlot_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_CarryToSlot_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_CarryToSlot_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t CarryToSlot_SendGoal_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CarryToSlot_SendGoal_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace parking_robot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, parking_robot_interfaces, action, CarryToSlot_SendGoal)() {
  return &::parking_robot_interfaces::action::rosidl_typesupport_c::CarryToSlot_SendGoal_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "parking_robot_interfaces/action/detail/carry_to_slot__struct.h"
// already included above
// #include "parking_robot_interfaces/action/detail/carry_to_slot__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace parking_robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _CarryToSlot_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CarryToSlot_GetResult_Request_type_support_ids_t;

static const _CarryToSlot_GetResult_Request_type_support_ids_t _CarryToSlot_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _CarryToSlot_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CarryToSlot_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CarryToSlot_GetResult_Request_type_support_symbol_names_t _CarryToSlot_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, parking_robot_interfaces, action, CarryToSlot_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, parking_robot_interfaces, action, CarryToSlot_GetResult_Request)),
  }
};

typedef struct _CarryToSlot_GetResult_Request_type_support_data_t
{
  void * data[2];
} _CarryToSlot_GetResult_Request_type_support_data_t;

static _CarryToSlot_GetResult_Request_type_support_data_t _CarryToSlot_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CarryToSlot_GetResult_Request_message_typesupport_map = {
  2,
  "parking_robot_interfaces",
  &_CarryToSlot_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_CarryToSlot_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_CarryToSlot_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CarryToSlot_GetResult_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CarryToSlot_GetResult_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace parking_robot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, parking_robot_interfaces, action, CarryToSlot_GetResult_Request)() {
  return &::parking_robot_interfaces::action::rosidl_typesupport_c::CarryToSlot_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "parking_robot_interfaces/action/detail/carry_to_slot__struct.h"
// already included above
// #include "parking_robot_interfaces/action/detail/carry_to_slot__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace parking_robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _CarryToSlot_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CarryToSlot_GetResult_Response_type_support_ids_t;

static const _CarryToSlot_GetResult_Response_type_support_ids_t _CarryToSlot_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _CarryToSlot_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CarryToSlot_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CarryToSlot_GetResult_Response_type_support_symbol_names_t _CarryToSlot_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, parking_robot_interfaces, action, CarryToSlot_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, parking_robot_interfaces, action, CarryToSlot_GetResult_Response)),
  }
};

typedef struct _CarryToSlot_GetResult_Response_type_support_data_t
{
  void * data[2];
} _CarryToSlot_GetResult_Response_type_support_data_t;

static _CarryToSlot_GetResult_Response_type_support_data_t _CarryToSlot_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CarryToSlot_GetResult_Response_message_typesupport_map = {
  2,
  "parking_robot_interfaces",
  &_CarryToSlot_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_CarryToSlot_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_CarryToSlot_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CarryToSlot_GetResult_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CarryToSlot_GetResult_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace parking_robot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, parking_robot_interfaces, action, CarryToSlot_GetResult_Response)() {
  return &::parking_robot_interfaces::action::rosidl_typesupport_c::CarryToSlot_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "parking_robot_interfaces/action/detail/carry_to_slot__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace parking_robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _CarryToSlot_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CarryToSlot_GetResult_type_support_ids_t;

static const _CarryToSlot_GetResult_type_support_ids_t _CarryToSlot_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _CarryToSlot_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CarryToSlot_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CarryToSlot_GetResult_type_support_symbol_names_t _CarryToSlot_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, parking_robot_interfaces, action, CarryToSlot_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, parking_robot_interfaces, action, CarryToSlot_GetResult)),
  }
};

typedef struct _CarryToSlot_GetResult_type_support_data_t
{
  void * data[2];
} _CarryToSlot_GetResult_type_support_data_t;

static _CarryToSlot_GetResult_type_support_data_t _CarryToSlot_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CarryToSlot_GetResult_service_typesupport_map = {
  2,
  "parking_robot_interfaces",
  &_CarryToSlot_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_CarryToSlot_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_CarryToSlot_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t CarryToSlot_GetResult_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CarryToSlot_GetResult_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace parking_robot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, parking_robot_interfaces, action, CarryToSlot_GetResult)() {
  return &::parking_robot_interfaces::action::rosidl_typesupport_c::CarryToSlot_GetResult_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "parking_robot_interfaces/action/detail/carry_to_slot__struct.h"
// already included above
// #include "parking_robot_interfaces/action/detail/carry_to_slot__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace parking_robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _CarryToSlot_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CarryToSlot_FeedbackMessage_type_support_ids_t;

static const _CarryToSlot_FeedbackMessage_type_support_ids_t _CarryToSlot_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _CarryToSlot_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CarryToSlot_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CarryToSlot_FeedbackMessage_type_support_symbol_names_t _CarryToSlot_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, parking_robot_interfaces, action, CarryToSlot_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, parking_robot_interfaces, action, CarryToSlot_FeedbackMessage)),
  }
};

typedef struct _CarryToSlot_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _CarryToSlot_FeedbackMessage_type_support_data_t;

static _CarryToSlot_FeedbackMessage_type_support_data_t _CarryToSlot_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CarryToSlot_FeedbackMessage_message_typesupport_map = {
  2,
  "parking_robot_interfaces",
  &_CarryToSlot_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_CarryToSlot_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_CarryToSlot_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CarryToSlot_FeedbackMessage_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CarryToSlot_FeedbackMessage_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace parking_robot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, parking_robot_interfaces, action, CarryToSlot_FeedbackMessage)() {
  return &::parking_robot_interfaces::action::rosidl_typesupport_c::CarryToSlot_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "action_msgs/msg/goal_status_array.h"
#include "action_msgs/srv/cancel_goal.h"
#include "parking_robot_interfaces/action/carry_to_slot.h"
// already included above
// #include "parking_robot_interfaces/action/detail/carry_to_slot__type_support.h"

static rosidl_action_type_support_t _parking_robot_interfaces__action__CarryToSlot__typesupport_c;

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(
  rosidl_typesupport_c, parking_robot_interfaces, action, CarryToSlot)()
{
  // Thread-safe by always writing the same values to the static struct
  _parking_robot_interfaces__action__CarryToSlot__typesupport_c.goal_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, parking_robot_interfaces, action, CarryToSlot_SendGoal)();
  _parking_robot_interfaces__action__CarryToSlot__typesupport_c.result_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, parking_robot_interfaces, action, CarryToSlot_GetResult)();
  _parking_robot_interfaces__action__CarryToSlot__typesupport_c.cancel_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, srv, CancelGoal)();
  _parking_robot_interfaces__action__CarryToSlot__typesupport_c.feedback_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, parking_robot_interfaces, action, CarryToSlot_FeedbackMessage)();
  _parking_robot_interfaces__action__CarryToSlot__typesupport_c.status_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, msg, GoalStatusArray)();

  return &_parking_robot_interfaces__action__CarryToSlot__typesupport_c;
}

#ifdef __cplusplus
}
#endif
