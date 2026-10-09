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

// diagnostic_updater names statuses "<node name>: <task>", so radars in different
// namespaces would publish identical names. Declares diagnostic_updater.use_fqn with
// default true for a namespaced node; a value set by the user still wins. Call before
// constructing the Updater, which reads an already declared parameter.
inline bool declare_diagnostic_names(rclcpp::Node & node)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description =
    "Prefix diagnostic status names with the node namespace (default: true when namespaced).";
  return node.declare_parameter(
    "diagnostic_updater.use_fqn", std::string(node.get_namespace()) != "/", descriptor);
}
}  // namespace smartmicro::drivers::radar
#endif  // UMRR_ROS2_DRIVER__STARTUP_PARAMETER_HPP_
