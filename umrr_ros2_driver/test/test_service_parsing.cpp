// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <umrr_ros2_driver/sensor_models.hpp>
#include <umrr_ros2_driver/service_parsing.hpp>

#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <string>
#include <variant>

using smartmicro::drivers::radar::parse_command_value;
using smartmicro::drivers::radar::parse_mode_value;
using smartmicro::drivers::radar::validate_sensor_ipv4;
using smartmicro::drivers::radar::validate_sensor_config;
using smartmicro::drivers::radar::kCanModels;
using smartmicro::drivers::radar::kEthernetModels;
using smartmicro::drivers::radar::model_user_interface;
using smartmicro::drivers::radar::resolve_user_interface;

namespace
{
// Message of the std::invalid_argument thrown by call, or "" if none is thrown.
template<typename Call>
std::string rejection(Call && call)
{
  try {
    call();
  } catch (const std::invalid_argument & error) {
    return error.what();
  }
  return {};
}
}  // namespace

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

TEST(SensorConfig, UserInterfaceOfEveryModelMatchesCatalogueAndSdk)
{
  // model -> uifname pairs of param/model_uif_catalogue.yaml.
  std::map<std::string, std::string> catalogue;
  std::ifstream file(SMARTMICRO_MODEL_CATALOGUE);
  ASSERT_TRUE(file.is_open());
  const auto value = [](const std::string & line) {
      auto text = line.substr(line.find(':') + 1);
      text.erase(0, text.find_first_not_of(" \""));
      return text.substr(0, text.find_last_not_of(" \"") + 1);
    };
  std::string line, model;
  while (std::getline(file, line)) {
    if (line.find("- model:") != std::string::npos) {
      model = value(line);
    } else if (line.find("uifname:") != std::string::npos) {
      catalogue[model] = value(line);
    }
  }
  ASSERT_EQ(catalogue.size(), kEthernetModels.size() + kCanModels.size());
  const auto check = [&catalogue](std::string_view name) {
      const auto interface = model_user_interface(name);
      EXPECT_EQ(interface.name, catalogue.at(std::string(name))) << name;
      // The interface the driver compiles and links for this model (CMakeLists.txt).
      const auto directory = interface.name + "_v" + std::to_string(interface.major) + "_" +
        std::to_string(interface.minor) + "_" + std::to_string(interface.patch);
      EXPECT_TRUE(std::filesystem::is_directory(
          std::filesystem::path(SMARTMICRO_SDK_INCLUDE_DIR) / directory)) << name;
      EXPECT_NE(std::string(name).find(
          "_v" + std::to_string(interface.major) + "_" + std::to_string(interface.minor) + "_" +
          std::to_string(interface.patch)), std::string::npos) << name;
    };
  for (const auto name : kEthernetModels) {
    check(name);
                                                       }
  for (const auto name : kCanModels) {
    check(name);
                                                  }
}

TEST(SensorConfig, UserInterfaceParametersMustMatchTheModel)
{
  const auto filled = resolve_user_interface("s", "umrr96_v1_2_2", "", 0, 0, 0);
  EXPECT_EQ(filled.name, "umrr96_t153_automotive");
  EXPECT_EQ(filled.major, 1U);
  EXPECT_EQ(filled.minor, 2U);
  EXPECT_EQ(filled.patch, 2U);
  EXPECT_EQ(resolve_user_interface("s", "umrr9f_can_mse_v1_3_0", "umrr9f_t169_mse", 1, 3, 0).name,
    "umrr9f_t169_mse");
  EXPECT_EQ(rejection([] {
      resolve_user_interface("s", "umrr96_v1_2_2", "umrra4_automotive", 1, 6, 0);
    }),
    "s.uifname 'umrra4_automotive' does not match model 'umrr96_v1_2_2' "
    "(expects 'umrr96_t153_automotive')");
  EXPECT_EQ(rejection([] {
      resolve_user_interface("s", "umrr96_v1_2_2", "umrr96_t153_automotive", 1, 2, 1);
    }), "s.uifpatchv 1 does not match model 'umrr96_v1_2_2' (expects 2)");
  // A name without a version is a mismatch, not "unset".
  EXPECT_EQ(rejection([] {
      resolve_user_interface("s", "umrra4_v1_6_0", "umrra4_automotive", 0, 0, 0);
    }), "s.uifmajorv 0 does not match model 'umrra4_v1_6_0' (expects 1)");
  for (const auto * model : {"umrr11", "umrr96_v1_2", "umrr96_v1_2_2_3", "umrr42_v1_0_0",
      "umrr96_v1_x_2", "umrr96_v1_2_2_"})
  {
    EXPECT_THROW(model_user_interface(model), std::invalid_argument) << model;
  }
}
