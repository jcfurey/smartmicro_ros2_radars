// SPDX-License-Identifier: Apache-2.0
#ifndef UMRR_ROS2_DRIVER__READBACK_NODE_HPP_
#define UMRR_ROS2_DRIVER__READBACK_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace smartmicro::drivers::radar
{
// Creates the UMRR-96 control/readback node, also registered as the component
// smartmicro::drivers::radar::ReadbackNode. Throws on invalid parameters.
std::shared_ptr<rclcpp::Node> make_readback_node(const rclcpp::NodeOptions & options);
}  // namespace smartmicro::drivers::radar

#endif  // UMRR_ROS2_DRIVER__READBACK_NODE_HPP_
