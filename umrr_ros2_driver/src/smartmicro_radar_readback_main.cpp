// SPDX-License-Identifier: Apache-2.0
// Standalone executable for the readback component. Unlike the generated
// rclcpp_components main, it reports constructor errors (invalid parameters,
// SDK initialisation) with exit status 1 after unwinding, so the private SDK
// configuration directory is removed.

#include <rclcpp/rclcpp.hpp>
#include <umrr_ros2_driver/readback_node.hpp>

#include <exception>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(smartmicro::drivers::radar::make_readback_node(rclcpp::NodeOptions{}));
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("smart_radar_readback"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
