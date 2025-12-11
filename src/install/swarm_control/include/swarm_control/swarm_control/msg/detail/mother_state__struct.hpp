// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from swarm_control:msg/MotherState.idl
// generated code does not contain a copyright notice

#ifndef SWARM_CONTROL__MSG__DETAIL__MOTHER_STATE__STRUCT_HPP_
#define SWARM_CONTROL__MSG__DETAIL__MOTHER_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__swarm_control__msg__MotherState __attribute__((deprecated))
#else
# define DEPRECATED__swarm_control__msg__MotherState __declspec(deprecated)
#endif

namespace swarm_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotherState_
{
  using Type = MotherState_<ContainerAllocator>;

  explicit MotherState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->yaw = 0.0f;
      this->distance = 0.0f;
      this->lock = 0l;
      this->direction = 0l;
      this->at_goal = false;
      this->timestamp = 0ull;
    }
  }

  explicit MotherState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->yaw = 0.0f;
      this->distance = 0.0f;
      this->lock = 0l;
      this->direction = 0l;
      this->at_goal = false;
      this->timestamp = 0ull;
    }
  }

  // field types and members
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _z_type =
    float;
  _z_type z;
  using _yaw_type =
    float;
  _yaw_type yaw;
  using _distance_type =
    float;
  _distance_type distance;
  using _lock_type =
    int32_t;
  _lock_type lock;
  using _direction_type =
    int32_t;
  _direction_type direction;
  using _at_goal_type =
    bool;
  _at_goal_type at_goal;
  using _timestamp_type =
    uint64_t;
  _timestamp_type timestamp;

  // setters for named parameter idiom
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const float & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__yaw(
    const float & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__distance(
    const float & _arg)
  {
    this->distance = _arg;
    return *this;
  }
  Type & set__lock(
    const int32_t & _arg)
  {
    this->lock = _arg;
    return *this;
  }
  Type & set__direction(
    const int32_t & _arg)
  {
    this->direction = _arg;
    return *this;
  }
  Type & set__at_goal(
    const bool & _arg)
  {
    this->at_goal = _arg;
    return *this;
  }
  Type & set__timestamp(
    const uint64_t & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    swarm_control::msg::MotherState_<ContainerAllocator> *;
  using ConstRawPtr =
    const swarm_control::msg::MotherState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<swarm_control::msg::MotherState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<swarm_control::msg::MotherState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      swarm_control::msg::MotherState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<swarm_control::msg::MotherState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      swarm_control::msg::MotherState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<swarm_control::msg::MotherState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<swarm_control::msg::MotherState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<swarm_control::msg::MotherState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__swarm_control__msg__MotherState
    std::shared_ptr<swarm_control::msg::MotherState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__swarm_control__msg__MotherState
    std::shared_ptr<swarm_control::msg::MotherState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotherState_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    if (this->lock != other.lock) {
      return false;
    }
    if (this->direction != other.direction) {
      return false;
    }
    if (this->at_goal != other.at_goal) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotherState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotherState_

// alias to use template instance with default allocator
using MotherState =
  swarm_control::msg::MotherState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace swarm_control

#endif  // SWARM_CONTROL__MSG__DETAIL__MOTHER_STATE__STRUCT_HPP_
