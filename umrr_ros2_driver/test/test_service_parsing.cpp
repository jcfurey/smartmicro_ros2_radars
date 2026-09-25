// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <umrr_ros2_driver/sensor_models.hpp>
#include <umrr_ros2_driver/service_parsing.hpp>

#include <string>
#include <variant>

using smartmicro::drivers::radar::parse_mode_value;
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
