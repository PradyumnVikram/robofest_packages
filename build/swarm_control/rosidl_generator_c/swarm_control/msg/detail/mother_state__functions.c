// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from swarm_control:msg/MotherState.idl
// generated code does not contain a copyright notice
#include "swarm_control/msg/detail/mother_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
swarm_control__msg__MotherState__init(swarm_control__msg__MotherState * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // z
  // yaw
  // distance
  // lock
  // direction
  // at_goal
  // timestamp
  return true;
}

void
swarm_control__msg__MotherState__fini(swarm_control__msg__MotherState * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // z
  // yaw
  // distance
  // lock
  // direction
  // at_goal
  // timestamp
}

bool
swarm_control__msg__MotherState__are_equal(const swarm_control__msg__MotherState * lhs, const swarm_control__msg__MotherState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  // lock
  if (lhs->lock != rhs->lock) {
    return false;
  }
  // direction
  if (lhs->direction != rhs->direction) {
    return false;
  }
  // at_goal
  if (lhs->at_goal != rhs->at_goal) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  return true;
}

bool
swarm_control__msg__MotherState__copy(
  const swarm_control__msg__MotherState * input,
  swarm_control__msg__MotherState * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  // yaw
  output->yaw = input->yaw;
  // distance
  output->distance = input->distance;
  // lock
  output->lock = input->lock;
  // direction
  output->direction = input->direction;
  // at_goal
  output->at_goal = input->at_goal;
  // timestamp
  output->timestamp = input->timestamp;
  return true;
}

swarm_control__msg__MotherState *
swarm_control__msg__MotherState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarm_control__msg__MotherState * msg = (swarm_control__msg__MotherState *)allocator.allocate(sizeof(swarm_control__msg__MotherState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(swarm_control__msg__MotherState));
  bool success = swarm_control__msg__MotherState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
swarm_control__msg__MotherState__destroy(swarm_control__msg__MotherState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    swarm_control__msg__MotherState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
swarm_control__msg__MotherState__Sequence__init(swarm_control__msg__MotherState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarm_control__msg__MotherState * data = NULL;

  if (size) {
    data = (swarm_control__msg__MotherState *)allocator.zero_allocate(size, sizeof(swarm_control__msg__MotherState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = swarm_control__msg__MotherState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        swarm_control__msg__MotherState__fini(&data[i - 1]);
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
swarm_control__msg__MotherState__Sequence__fini(swarm_control__msg__MotherState__Sequence * array)
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
      swarm_control__msg__MotherState__fini(&array->data[i]);
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

swarm_control__msg__MotherState__Sequence *
swarm_control__msg__MotherState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarm_control__msg__MotherState__Sequence * array = (swarm_control__msg__MotherState__Sequence *)allocator.allocate(sizeof(swarm_control__msg__MotherState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = swarm_control__msg__MotherState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
swarm_control__msg__MotherState__Sequence__destroy(swarm_control__msg__MotherState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    swarm_control__msg__MotherState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
swarm_control__msg__MotherState__Sequence__are_equal(const swarm_control__msg__MotherState__Sequence * lhs, const swarm_control__msg__MotherState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!swarm_control__msg__MotherState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
swarm_control__msg__MotherState__Sequence__copy(
  const swarm_control__msg__MotherState__Sequence * input,
  swarm_control__msg__MotherState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(swarm_control__msg__MotherState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    swarm_control__msg__MotherState * data =
      (swarm_control__msg__MotherState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!swarm_control__msg__MotherState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          swarm_control__msg__MotherState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!swarm_control__msg__MotherState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
