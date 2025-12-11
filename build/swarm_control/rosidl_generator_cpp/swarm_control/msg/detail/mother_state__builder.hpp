// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from swarm_control:msg/MotherState.idl
// generated code does not contain a copyright notice

#ifndef SWARM_CONTROL__MSG__DETAIL__MOTHER_STATE__BUILDER_HPP_
#define SWARM_CONTROL__MSG__DETAIL__MOTHER_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "swarm_control/msg/detail/mother_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace swarm_control
{

namespace msg
{

namespace builder
{

class Init_MotherState_timestamp
{
public:
  explicit Init_MotherState_timestamp(::swarm_control::msg::MotherState & msg)
  : msg_(msg)
  {}
  ::swarm_control::msg::MotherState timestamp(::swarm_control::msg::MotherState::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::swarm_control::msg::MotherState msg_;
};

class Init_MotherState_at_goal
{
public:
  explicit Init_MotherState_at_goal(::swarm_control::msg::MotherState & msg)
  : msg_(msg)
  {}
  Init_MotherState_timestamp at_goal(::swarm_control::msg::MotherState::_at_goal_type arg)
  {
    msg_.at_goal = std::move(arg);
    return Init_MotherState_timestamp(msg_);
  }

private:
  ::swarm_control::msg::MotherState msg_;
};

class Init_MotherState_direction
{
public:
  explicit Init_MotherState_direction(::swarm_control::msg::MotherState & msg)
  : msg_(msg)
  {}
  Init_MotherState_at_goal direction(::swarm_control::msg::MotherState::_direction_type arg)
  {
    msg_.direction = std::move(arg);
    return Init_MotherState_at_goal(msg_);
  }

private:
  ::swarm_control::msg::MotherState msg_;
};

class Init_MotherState_lock
{
public:
  explicit Init_MotherState_lock(::swarm_control::msg::MotherState & msg)
  : msg_(msg)
  {}
  Init_MotherState_direction lock(::swarm_control::msg::MotherState::_lock_type arg)
  {
    msg_.lock = std::move(arg);
    return Init_MotherState_direction(msg_);
  }

private:
  ::swarm_control::msg::MotherState msg_;
};

class Init_MotherState_distance
{
public:
  explicit Init_MotherState_distance(::swarm_control::msg::MotherState & msg)
  : msg_(msg)
  {}
  Init_MotherState_lock distance(::swarm_control::msg::MotherState::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_MotherState_lock(msg_);
  }

private:
  ::swarm_control::msg::MotherState msg_;
};

class Init_MotherState_yaw
{
public:
  explicit Init_MotherState_yaw(::swarm_control::msg::MotherState & msg)
  : msg_(msg)
  {}
  Init_MotherState_distance yaw(::swarm_control::msg::MotherState::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_MotherState_distance(msg_);
  }

private:
  ::swarm_control::msg::MotherState msg_;
};

class Init_MotherState_z
{
public:
  explicit Init_MotherState_z(::swarm_control::msg::MotherState & msg)
  : msg_(msg)
  {}
  Init_MotherState_yaw z(::swarm_control::msg::MotherState::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_MotherState_yaw(msg_);
  }

private:
  ::swarm_control::msg::MotherState msg_;
};

class Init_MotherState_y
{
public:
  explicit Init_MotherState_y(::swarm_control::msg::MotherState & msg)
  : msg_(msg)
  {}
  Init_MotherState_z y(::swarm_control::msg::MotherState::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_MotherState_z(msg_);
  }

private:
  ::swarm_control::msg::MotherState msg_;
};

class Init_MotherState_x
{
public:
  Init_MotherState_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotherState_y x(::swarm_control::msg::MotherState::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_MotherState_y(msg_);
  }

private:
  ::swarm_control::msg::MotherState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::swarm_control::msg::MotherState>()
{
  return swarm_control::msg::builder::Init_MotherState_x();
}

}  // namespace swarm_control

#endif  // SWARM_CONTROL__MSG__DETAIL__MOTHER_STATE__BUILDER_HPP_
