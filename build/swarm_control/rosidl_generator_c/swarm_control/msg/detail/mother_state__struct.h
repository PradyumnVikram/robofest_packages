// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from swarm_control:msg/MotherState.idl
// generated code does not contain a copyright notice

#ifndef SWARM_CONTROL__MSG__DETAIL__MOTHER_STATE__STRUCT_H_
#define SWARM_CONTROL__MSG__DETAIL__MOTHER_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MotherState in the package swarm_control.
typedef struct swarm_control__msg__MotherState
{
  float x;
  float y;
  float z;
  float yaw;
  float distance;
  int32_t lock;
  int32_t direction;
  bool at_goal;
  uint64_t timestamp;
} swarm_control__msg__MotherState;

// Struct for a sequence of swarm_control__msg__MotherState.
typedef struct swarm_control__msg__MotherState__Sequence
{
  swarm_control__msg__MotherState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} swarm_control__msg__MotherState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SWARM_CONTROL__MSG__DETAIL__MOTHER_STATE__STRUCT_H_
