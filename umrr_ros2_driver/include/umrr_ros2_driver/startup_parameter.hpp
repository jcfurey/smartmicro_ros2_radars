// SPDX-License-Identifier: Apache-2.0
#ifndef UMRR_ROS2_DRIVER__STARTUP_PARAMETER_HPP_
#define UMRR_ROS2_DRIVER__STARTUP_PARAMETER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <string>
#include <type_traits>

namespace smartmicro::drivers::radar
{
inline rcl_interfaces::msg::ParameterDescriptor startup_descriptor()
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = "Startup configuration; restart the node to change this value.";
  descriptor.read_only = true;
  return descriptor;
}

template<typename T>
auto startup_parameter(
  rclcpp::Node & node, const std::string & name, T default_value,
  int64_t minimum = 0, int64_t maximum = UINT32_MAX)
{
  auto descriptor = startup_descriptor();
  if constexpr (std::is_integral_v<T>) {
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = minimum;
    range.to_value = maximum;
    descriptor.integer_range.push_back(range);
    // Validate before assignment to the SDK's unsigned fields; never wrap negatives.
    return node.declare_parameter<int64_t>(name, default_value, descriptor);
  } else {
    return node.declare_parameter<std::string>(name, std::string(default_value), descriptor);
  }
}
}  // namespace smartmicro::drivers::radar
#endif  // UMRR_ROS2_DRIVER__STARTUP_PARAMETER_HPP_
