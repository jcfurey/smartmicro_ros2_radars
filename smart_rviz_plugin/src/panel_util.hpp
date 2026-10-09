// SPDX-License-Identifier: Apache-2.0
// Helpers shared by the generic Smart panels (not installed).
#ifndef SMART_RVIZ_PLUGIN__PANEL_UTIL_HPP_
#define SMART_RVIZ_PLUGIN__PANEL_UTIL_HPP_

#include <QString>
#include <QVariant>
#include <QWidget>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>
#include <unistd.h>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>

namespace smart_rviz_plugin
{
namespace panel_util
{
/// Unique node name per panel instance: the process ID separates RViz processes that
/// load the same configuration, the counter separates copies of a panel in one process.
inline std::string unique_node_name(const std::string & base)
{
  static std::atomic<unsigned> counter{0};
  return base + "_" + std::to_string(::getpid()) + "_" + std::to_string(counter++);
}

/// Options for a panel's own node. Global arguments stay enabled, so the namespace and
/// remappings RViz was started with (a launch file's namespace:=front gives
/// `-r __ns:=/front`) apply to the panel's relative endpoints as to the displays (C39).
/// Local arguments take precedence: the `__node` rule keeps the unique panel name when
/// launch renames RViz's node, and a non-empty `ns` replaces the process namespace.
inline rclcpp::NodeOptions node_options(const std::string & name, const std::string & ns = {})
{
  std::vector<std::string> arguments{"--ros-args", "-r", "__node:=" + name};
  if (!ns.empty()) {
    arguments.insert(arguments.end(), {"-r", "__ns:=" + (ns.front() == '/' ? ns : "/" + ns)});
  }
  return rclcpp::NodeOptions().arguments(arguments);
}

/// Subscription QoS for driver data topics: best effort matches both the default reliable
/// publishers and a best-effort override (qos_overrides...reliability). Ten pending
/// messages leave room for the spin_all catch-up after a GUI stall (C41).
inline rclcpp::QoS data_qos()
{
  return rclcpp::SensorDataQoS(rclcpp::KeepLast(10));
}

/// Strict unsigned parser: whole string, decimal or 0x-prefixed hexadecimal, <= max.
/// Signs, leading/trailing garbage, octal ambiguity and overflow are rejected.
inline std::optional<uint32_t> parse_uint(
  const QString & input, uint32_t max = std::numeric_limits<uint32_t>::max())
{
  const auto text = input.trimmed();
  if (text.isEmpty() || text.startsWith('+') || text.startsWith('-')) {return std::nullopt;}
  bool ok = false;
  const bool hex = text.startsWith("0x", Qt::CaseInsensitive);
  const qulonglong value = hex ? text.mid(2).toULongLong(&ok, 16) : text.toULongLong(&ok, 10);
  if (!ok || (hex && text.size() == 2) || value > max) {return std::nullopt;}
  return static_cast<uint32_t>(value);
}

/// Strict finite float32 parser.
inline std::optional<float> parse_float(const QString & input)
{
  bool ok = false;
  const float value = input.trimmed().toFloat(&ok);
  if (!ok || !std::isfinite(value)) {return std::nullopt;}
  return value;
}

/// Service deadline; tests may shorten it through the "request_timeout_ms" property.
inline std::chrono::milliseconds request_timeout(
  const QWidget * panel, std::chrono::milliseconds fallback)
{
  bool ok = false;
  const auto ms = panel->property("request_timeout_ms").toLongLong(&ok);
  return ok && ms > 0 ? std::chrono::milliseconds(ms) : fallback;
}
}  // namespace panel_util
}  // namespace smart_rviz_plugin

#endif  // SMART_RVIZ_PLUGIN__PANEL_UTIL_HPP_
