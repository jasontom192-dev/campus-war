// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from tdt_interface:msg/AutoAimState.idl
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
#include "tdt_interface/msg/detail/auto_aim_state__struct.h"
#include "tdt_interface/msg/detail/auto_aim_state__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__point__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__point__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__twist__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__twist__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool tdt_interface__msg__auto_aim_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[47];
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
    assert(strncmp("tdt_interface.msg._auto_aim_state.AutoAimState", full_classname_dest, 46) == 0);
  }
  tdt_interface__msg__AutoAimState * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // frame_sequence
    PyObject * field = PyObject_GetAttrString(_pymsg, "frame_sequence");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->frame_sequence = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }
  {  // position
    PyObject * field = PyObject_GetAttrString(_pymsg, "position");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__point__convert_from_py(field, &ros_message->position)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // twist
    PyObject * field = PyObject_GetAttrString(_pymsg, "twist");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__twist__convert_from_py(field, &ros_message->twist)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // yaw_degrees
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw_degrees");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->yaw_degrees = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pitch_degrees
    PyObject * field = PyObject_GetAttrString(_pymsg, "pitch_degrees");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pitch_degrees = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // health
    PyObject * field = PyObject_GetAttrString(_pymsg, "health");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->health = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // max_health
    PyObject * field = PyObject_GetAttrString(_pymsg, "max_health");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->max_health = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // is_weakened
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_weakened");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_weakened = (Py_True == field);
    Py_DECREF(field);
  }
  {  // projectile_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "projectile_speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->projectile_speed = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // remaining_seconds
    PyObject * field = PyObject_GetAttrString(_pymsg, "remaining_seconds");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->remaining_seconds = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // enemy_healths
    PyObject * field = PyObject_GetAttrString(_pymsg, "enemy_healths");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_INT32);
      Py_ssize_t size = 7;
      int32_t * dest = ros_message->enemy_healths;
      for (Py_ssize_t i = 0; i < size; ++i) {
        int32_t tmp = *(npy_int32 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(int32_t));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // ammo_remaining
    PyObject * field = PyObject_GetAttrString(_pymsg, "ammo_remaining");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->ammo_remaining = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // ammo_capacity
    PyObject * field = PyObject_GetAttrString(_pymsg, "ammo_capacity");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->ammo_capacity = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * tdt_interface__msg__auto_aim_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of AutoAimState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("tdt_interface.msg._auto_aim_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "AutoAimState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  tdt_interface__msg__AutoAimState * ros_message = (tdt_interface__msg__AutoAimState *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // frame_sequence
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->frame_sequence);
    {
      int rc = PyObject_SetAttrString(_pymessage, "frame_sequence", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // position
    PyObject * field = NULL;
    field = geometry_msgs__msg__point__convert_to_py(&ros_message->position);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // twist
    PyObject * field = NULL;
    field = geometry_msgs__msg__twist__convert_to_py(&ros_message->twist);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "twist", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yaw_degrees
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->yaw_degrees);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yaw_degrees", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pitch_degrees
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pitch_degrees);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pitch_degrees", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // health
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->health);
    {
      int rc = PyObject_SetAttrString(_pymessage, "health", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // max_health
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->max_health);
    {
      int rc = PyObject_SetAttrString(_pymessage, "max_health", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_weakened
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_weakened ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_weakened", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // projectile_speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->projectile_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "projectile_speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // remaining_seconds
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->remaining_seconds);
    {
      int rc = PyObject_SetAttrString(_pymessage, "remaining_seconds", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // enemy_healths
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "enemy_healths");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_INT32);
    assert(sizeof(npy_int32) == sizeof(int32_t));
    npy_int32 * dst = (npy_int32 *)PyArray_GETPTR1(seq_field, 0);
    int32_t * src = &(ros_message->enemy_healths[0]);
    memcpy(dst, src, 7 * sizeof(int32_t));
    Py_DECREF(field);
  }
  {  // ammo_remaining
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->ammo_remaining);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ammo_remaining", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ammo_capacity
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->ammo_capacity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ammo_capacity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
