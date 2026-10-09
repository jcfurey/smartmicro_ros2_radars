// SPDX-License-Identifier: Apache-2.0
#ifndef UMRR_ROS2_DRIVER__STARTUP_PARAMETER_HPP_
#define UMRR_ROS2_DRIVER__STARTUP_PARAMETER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <stdexcept>
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

// A floating-point startup parameter. An integer is accepted too (`5` for `5.0` in YAML,
// as in the Python nodes); a non-number, NaN or a value outside [minimum, maximum] is
// rejected with the parameter named.
inline double startup_number(
  rclcpp::Node & node, const std::string & name, double default_value, double minimum,
  double maximum, const std::string & description, const std::string & unit = {})
{
  std::ostringstream range;
  range << minimum << ".." << maximum << unit;
  auto descriptor = startup_descriptor();
  descriptor.description = description;
  descriptor.additional_constraints = "Number within [" + range.str() + "]";
  descriptor.dynamic_typing = true;
  const auto value =
    node.declare_parameter(name, rclcpp::ParameterValue(default_value), descriptor);
  double number{};
  if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    number = static_cast<double>(value.get<int64_t>());
  } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    number = value.get<double>();
  } else {
    throw std::invalid_argument(name + " must be a number");
  }
  if (!std::isfinite(number) || number < minimum || number > maximum) {
    throw std::invalid_argument(name + " must be within " + range.str());
  }
  return number;
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
