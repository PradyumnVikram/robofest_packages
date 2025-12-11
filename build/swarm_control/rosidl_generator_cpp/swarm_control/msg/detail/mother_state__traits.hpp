// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from swarm_control:msg/MotherState.idl
// generated code does not contain a copyright notice

#ifndef SWARM_CONTROL__MSG__DETAIL__MOTHER_STATE__TRAITS_HPP_
#define SWARM_CONTROL__MSG__DETAIL__MOTHER_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "swarm_control/msg/detail/mother_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace swarm_control
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotherState & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: distance
  {
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << ", ";
  }

  // member: lock
  {
    out << "lock: ";
    rosidl_generator_traits::value_to_yaml(msg.lock, out);
    out << ", ";
  }

  // member: direction
  {
    out << "direction: ";
    rosidl_generator_traits::value_to_yaml(msg.direction, out);
    out << ", ";
  }

  // member: at_goal
  {
    out << "at_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.at_goal, out);
    out << ", ";
  }

  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotherState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << "\n";
  }

  // member: lock
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lock: ";
    rosidl_generator_traits::value_to_yaml(msg.lock, out);
    out << "\n";
  }

  // member: direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "direction: ";
    rosidl_generator_traits::value_to_yaml(msg.direction, out);
    out << "\n";
  }

  // member: at_goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "at_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.at_goal, out);
    out << "\n";
  }

  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotherState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace swarm_control

namespace rosidl_generator_traits
{

[[deprecated("use swarm_control::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const swarm_control::msg::MotherState & msg,
  std::ostream & out, size_t indentation = 0)
{
  swarm_control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use swarm_control::msg::to_yaml() instead")]]
inline std::string to_yaml(const swarm_control::msg::MotherState & msg)
{
  return swarm_control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<swarm_control::msg::MotherState>()
{
  return "swarm_control::msg::MotherState";
}

template<>
inline const char * name<swarm_control::msg::MotherState>()
{
  return "swarm_control/msg/MotherState";
}

template<>
struct has_fixed_size<swarm_control::msg::MotherState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<swarm_control::msg::MotherState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<swarm_control::msg::MotherState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SWARM_CONTROL__MSG__DETAIL__MOTHER_STATE__TRAITS_HPP_
