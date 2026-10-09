// SPDX-License-Identifier: Apache-2.0
// Standalone executable for the data node component. Unlike the generated
// rclcpp_components main, it reports constructor errors (invalid parameters,
// SDK initialisation) with exit status 1 after unwinding, so the private SDK
// configuration directory is removed.

#include <rclcpp/rclcpp.hpp>
#include <umrr_ros2_driver/runtime_config.hpp>
#include <umrr_ros2_driver/smartmicro_radar_node.hpp>

#include <exception>
#include <memory>
#include <utility>

int main(int argc, char ** argv)
{
  using smartmicro::drivers::radar::RuntimeConfig;
  // The SDK takes its configuration path only from the environment. It is set here,
  // before rclcpp::init starts middleware threads, because setenv is not safe
  // against a concurrent getenv.
  std::unique_ptr<RuntimeConfig> config;
  try {
    config = std::make_unique<RuntimeConfig>("smartmicro-data");
    config->activate();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("smartmicro_radar_node"), "%s", error.what());
    return 1;
  }
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(
      std::make_shared<smartmicro::drivers::radar::SmartmicroRadarNode>(
        rclcpp::NodeOptions{}, std::move(config)));
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("smartmicro_radar_node"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
