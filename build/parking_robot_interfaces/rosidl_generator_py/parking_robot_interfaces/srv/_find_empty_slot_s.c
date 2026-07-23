// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from parking_robot_interfaces:srv/FindEmptySlot.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "parking_robot_interfaces/srv/detail/find_empty_slot__struct.h"
#include "parking_robot_interfaces/srv/detail/find_empty_slot__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool parking_robot_interfaces__srv__find_empty_slot__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[68];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("parking_robot_interfaces.srv._find_empty_slot.FindEmptySlot_Request", full_classname_dest, 67) == 0);
  }
  parking_robot_interfaces__srv__FindEmptySlot_Request * ros_message = _ros_message;
  {  // vehicle_length
    PyObject * field = PyObject_GetAttrString(_pymsg, "vehicle_length");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vehicle_length = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vehicle_width
    PyObject * field = PyObject_GetAttrString(_pymsg, "vehicle_width");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vehicle_width = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * parking_robot_interfaces__srv__find_empty_slot__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of FindEmptySlot_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("parking_robot_interfaces.srv._find_empty_slot");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "FindEmptySlot_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  parking_robot_interfaces__srv__FindEmptySlot_Request * ros_message = (parking_robot_interfaces__srv__FindEmptySlot_Request *)raw_ros_message;
  {  // vehicle_length
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vehicle_length);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vehicle_length", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vehicle_width
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vehicle_width);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vehicle_width", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "parking_robot_interfaces/srv/detail/find_empty_slot__struct.h"
// already included above
// #include "parking_robot_interfaces/srv/detail/find_empty_slot__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool parking_robot_interfaces__srv__find_empty_slot__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[69];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("parking_robot_interfaces.srv._find_empty_slot.FindEmptySlot_Response", full_classname_dest, 68) == 0);
  }
  parking_robot_interfaces__srv__FindEmptySlot_Response * ros_message = _ros_message;
  {  // success
    PyObject * field = PyObject_GetAttrString(_pymsg, "success");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->success = (Py_True == field);
    Py_DECREF(field);
  }
  {  // slot_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "slot_id");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->slot_id, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // slot_pose
    PyObject * field = PyObject_GetAttrString(_pymsg, "slot_pose");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose__convert_from_py(field, &ros_message->slot_pose)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * parking_robot_interfaces__srv__find_empty_slot__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of FindEmptySlot_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("parking_robot_interfaces.srv._find_empty_slot");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "FindEmptySlot_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  parking_robot_interfaces__srv__FindEmptySlot_Response * ros_message = (parking_robot_interfaces__srv__FindEmptySlot_Response *)raw_ros_message;
  {  // success
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->success ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "success", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // slot_id
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->slot_id.data,
      strlen(ros_message->slot_id.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "slot_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // slot_pose
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose__convert_to_py(&ros_message->slot_pose);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "slot_pose", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
