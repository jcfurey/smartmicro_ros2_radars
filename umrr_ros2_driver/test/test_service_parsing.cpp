// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <umrr_ros2_driver/sensor_models.hpp>
#include <umrr_ros2_driver/service_parsing.hpp>

#include <limits>
#include <string>
#include <variant>

using smartmicro::drivers::radar::parse_command_value;
using smartmicro::drivers::radar::parse_mode_value;
using smartmicro::drivers::radar::validate_sensor_ipv4;
using smartmicro::drivers::radar::validate_sensor_config;

TEST(ServiceParsing, AcceptsCompleteInRangeValues)
{
  EXPECT_FLOAT_EQ(std::get<float>(parse_mode_value("-12.5", 0)), -12.5F);
  EXPECT_EQ(std::get<uint32_t>(parse_mode_value("4294967295", 1)), 4294967295U);
  EXPECT_EQ(std::get<uint16_t>(parse_mode_value("65535", 2)), 65535U);
  EXPECT_EQ(std::get<uint8_t>(parse_mode_value("255", 3)), 255U);
  EXPECT_EQ(std::get<uint8_t>(parse_mode_value("0", 3)), 0U);
}

TEST(ServiceParsing, RejectsWrapTruncationAndNonFinite)
{
  for (const auto * text : {"-1", "12abc", "", " 1", "1.5", "+1", "0x10"}) {
    EXPECT_THROW(parse_mode_value(text, 1), std::invalid_argument) << text;
  }
  EXPECT_THROW(parse_mode_value("5000000000", 1), std::out_of_range);
  EXPECT_THROW(parse_mode_value("99999999999999999999999", 1), std::out_of_range);
  EXPECT_THROW(parse_mode_value("65536", 2), std::out_of_range);
  EXPECT_THROW(parse_mode_value("256", 3), std::out_of_range);
  for (const auto * text : {"nan", "inf", "-inf", "1x", "", "1.0 "}) {
    EXPECT_THROW(parse_mode_value(text, 0), std::invalid_argument) << text;
  }
  EXPECT_THROW(parse_mode_value("1e90", 0), std::out_of_range);
  EXPECT_THROW(parse_mode_value("1", 4), std::invalid_argument);
}

TEST(ServiceParsing, CommandValuesAreExactUnsignedIntegers)
{
  EXPECT_EQ(parse_command_value(0.0F), 0U);
  EXPECT_EQ(parse_command_value(2010.0F), 2010U);
  EXPECT_EQ(parse_command_value(16777216.0F), 16777216U);
  for (const float value : {-1.0F, 1.7F, 16777218.0F, 1e10F,
      std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()})
  {
    EXPECT_THROW(parse_command_value(value), std::invalid_argument) << value;
  }
}

TEST(ServiceParsing, SensorIpMustBeUsableUnicast)
{
  EXPECT_NO_THROW(validate_sensor_ipv4(3232238347U));  // 192.168.11.11
  EXPECT_NO_THROW(validate_sensor_ipv4(0x0A000001U));  // 10.0.0.1
  for (const uint32_t address : {0U, 0x00FFFFFFU, 0x7F000001U, 0xE0000001U, 0xF0000000U,
      0xFFFFFFFFU})
  {
    EXPECT_THROW(validate_sensor_ipv4(address), std::invalid_argument) << address;
  }
}

TEST(SensorConfig, RejectsUnknownOrInconsistentConfiguration)
{
  EXPECT_NO_THROW(validate_sensor_config("s", "eth", "umrr96_v1_2_2", "target"));
  EXPECT_NO_THROW(validate_sensor_config("s", "can", "umrra4_can_mse_v3_0_0", "mse"));
  EXPECT_THROW(validate_sensor_config("s", "can", "umrr96_can_v1_2_2", ""), std::invalid_argument);
  EXPECT_THROW(validate_sensor_config("s", "can", "umrr96_v1_2_2", "target"),
    std::invalid_argument);
  EXPECT_THROW(validate_sensor_config("s", "eth", "umrr96_can_v1_2_2", "target"),
    std::invalid_argument);
  EXPECT_THROW(validate_sensor_config("s", "eth", "umrr11", "target"), std::invalid_argument);
  EXPECT_THROW(validate_sensor_config("s", "usb", "umrr96_v1_2_2", "target"),
    std::invalid_argument);
  EXPECT_THROW(validate_sensor_config("s", "eth", "umrra4_mse_v3_0_0", "target"),
    std::invalid_argument);
  EXPECT_THROW(validate_sensor_config("s", "eth", "umrr96_v1_2_2", "mse"), std::invalid_argument);
}
