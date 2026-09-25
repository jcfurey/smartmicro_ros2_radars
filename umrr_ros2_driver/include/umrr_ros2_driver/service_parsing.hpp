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

// SendCommand.value is float32 but the SDK command argument is uint32 (commands
// include resets and EEPROM saves): accept only finite whole numbers that float32
// represents exactly, 0..2^24. A cast of a negative, NaN or out-of-range float
// is undefined behaviour, and a fraction would be truncated silently.
inline uint32_t parse_command_value(float value)
{
  constexpr float kMaxExact = 16777216.0F;  // 2^24
  if (!std::isfinite(value) || value < 0.0F || value > kMaxExact || std::trunc(value) != value) {
    throw std::invalid_argument(
            "command value must be a whole number within 0..16777216, got " +
            std::to_string(value));
  }
  return static_cast<uint32_t>(value);
}

// SetIp.value_ip (first octet in the most significant byte, as the sensor reads
// it back) is saved to EEPROM at once. Reject addresses the sensor could not be
// reached on after its restart: 0.0.0.0/8, loopback, and multicast, reserved or
// broadcast (224.0.0.0 and above).
inline void validate_sensor_ipv4(uint32_t address)
{
  const auto first = address >> 24;
  if (first == 0 || first == 127 || first >= 224) {
    throw std::invalid_argument(
            "IP address " + std::to_string(first) + "." + std::to_string((address >> 16) & 255) +
            "." + std::to_string((address >> 8) & 255) + "." + std::to_string(address & 255) +
            " is not a usable unicast sensor address");
  }
}
}  // namespace smartmicro::drivers::radar
#endif  // UMRR_ROS2_DRIVER__SERVICE_PARSING_HPP_
