// SPDX-License-Identifier: Apache-2.0
#ifndef UMRR_ROS2_DRIVER__UMRR96_TUNING_HPP_
#define UMRR_ROS2_DRIVER__UMRR96_TUNING_HPP_

#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace smartmicro::drivers::radar
{
using TuningValue = std::variant<float, uint8_t>;

// Volatile controls documented by UMRR-96 Type 153 UIF 1.2.2. Network,
// synchronization, output stream removal, EEPROM and reset are outside this API.
inline std::vector<TuningValue> validate_umrr96_tuning(
  const std::vector<std::string> & names, const std::vector<std::string> & strings,
  const std::vector<uint8_t> & types)
{
  if (names.empty() || names.size() > 10 || names.size() != strings.size() ||
    names.size() != types.size())
  {
    throw std::invalid_argument("Supply 1–10 names, values and types with equal lengths");
  }
  static const std::map<std::string, unsigned> limits = {
    {"tx_antenna_idx", 2}, {"frequency_sweep_idx", 2}, {"range_toggle_mode", 3},
    {"output_control_target_list_can", 1}, {"prf_selector_manual", 1},
    {"prf_set_selector", 0}, {"prf_manual_value_idx", 2}};
  std::map<std::string, float> parsed_values;
  std::vector<TuningValue> result;
  for (size_t i = 0; i < names.size(); ++i) {
    const auto & name = names[i];
    if (parsed_values.count(name)) {
      throw std::invalid_argument("Tuning names must be unique");
    }
    const auto bound = limits.find(name);
    bool velocity = false;
    for (unsigned sweep = 0; sweep != 3; ++sweep) {
      velocity |= name == "tv_min_speed_sweep_idx_" + std::to_string(sweep) ||
        name == "tv_max_speed_sweep_idx_" + std::to_string(sweep);
    }
    if ((velocity && types[i] != 0) || (!velocity &&
      (bound == limits.end() || types[i] != 3)))
    {
      throw std::invalid_argument("Unsupported tuning parameter or type: " + name);
    }
    const auto & text = strings[i];
    if (velocity) {
      float value{};
      const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
      if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
        !std::isfinite(value) || value < -150 || value > 150)
      {
        throw std::invalid_argument("Invalid tuning value for " + name);
      }
      parsed_values.emplace(name, value);
      result.emplace_back(value);
    } else {
      unsigned value{};
      const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
      if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
        value > bound->second)
      {
        throw std::invalid_argument("Invalid tuning value for " + name);
      }
      parsed_values.emplace(name, value);
      result.emplace_back(static_cast<uint8_t>(value));
    }
  }
  // Readback is not an atomic transaction with a write: require both bounds to
  // validate the intended window locally, regardless of the previous settings.
  for (unsigned sweep = 0; sweep != 3; ++sweep) {
    const auto low = parsed_values.find("tv_min_speed_sweep_idx_" + std::to_string(sweep));
    const auto high = parsed_values.find("tv_max_speed_sweep_idx_" + std::to_string(sweep));
    if ((low == parsed_values.end()) != (high == parsed_values.end())) {
      throw std::invalid_argument("Supply both velocity bounds for each changed sweep");
    }
    if (low != parsed_values.end() && low->second > high->second) {
      throw std::invalid_argument("Minimum velocity must not exceed maximum velocity");
    }
  }
  return result;
}
}  // namespace smartmicro::drivers::radar
#endif
