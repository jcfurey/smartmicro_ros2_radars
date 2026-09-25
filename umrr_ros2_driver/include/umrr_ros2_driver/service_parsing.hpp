// SPDX-License-Identifier: Apache-2.0
#ifndef UMRR_ROS2_DRIVER__SERVICE_PARSING_HPP_
#define UMRR_ROS2_DRIVER__SERVICE_PARSING_HPP_

#include <charconv>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>
#include <variant>

namespace smartmicro::drivers::radar
{
// Value type codes of SetMode.value_types and GetMode.param_types.
enum class ModeValueType : uint8_t { kFloat32 = 0, kUint32 = 1, kUint16 = 2, kUint8 = 3 };

using ModeValue = std::variant<float, uint32_t, uint16_t, uint8_t>;

namespace detail
{
template<typename T>
T parse_unsigned(const std::string & text, const char * type_name)
{
  // from_chars rejects signs, whitespace and empty input for unsigned types.
  uint64_t value{};
  const auto end = text.data() + text.size();
  const auto parsed = std::from_chars(text.data(), end, value);
  if (parsed.ec == std::errc::result_out_of_range ||
    (parsed.ec == std::errc{} && parsed.ptr == end && value > std::numeric_limits<T>::max()))
  {
    throw std::out_of_range(
            std::string("value '") + text + "' is out of range for " + type_name + " (0.." +
            std::to_string(std::numeric_limits<T>::max()) + ")");
  }
  if (parsed.ec != std::errc{} || parsed.ptr != end) {
    throw std::invalid_argument(
            std::string("value '") + text + "' is not a decimal " + type_name);
  }
  return static_cast<T>(value);
}
}  // namespace detail

// Parses a SetMode value completely: no trailing characters, no sign on unsigned
// types, no wrap-around and only finite floats. Throws std::invalid_argument or
// std::out_of_range with a message suitable for the service response.
inline ModeValue parse_mode_value(const std::string & text, uint8_t type)
{
  switch (static_cast<ModeValueType>(type)) {
    case ModeValueType::kFloat32: {
        float value{};
        const auto end = text.data() + text.size();
        const auto parsed = std::from_chars(text.data(), end, value);
        if (parsed.ec == std::errc::result_out_of_range) {
          throw std::out_of_range("value '" + text + "' is out of range for float32");
        }
        if (parsed.ec != std::errc{} || parsed.ptr != end || !std::isfinite(value)) {
          throw std::invalid_argument("value '" + text + "' is not a finite float32");
        }
        return value;
      }
    case ModeValueType::kUint32:
      return detail::parse_unsigned<uint32_t>(text, "uint32");
    case ModeValueType::kUint16:
      return detail::parse_unsigned<uint16_t>(text, "uint16");
    case ModeValueType::kUint8:
      return detail::parse_unsigned<uint8_t>(text, "uint8");
  }
  throw std::invalid_argument(
          "invalid value type " + std::to_string(type) +
          "; must be 0 (float32), 1 (uint32), 2 (uint16) or 3 (uint8)");
}
}  // namespace smartmicro::drivers::radar
#endif  // UMRR_ROS2_DRIVER__SERVICE_PARSING_HPP_
