// SPDX-License-Identifier: Apache-2.0
#ifndef UMRR_ROS2_DRIVER__SENSOR_MODELS_HPP_
#define UMRR_ROS2_DRIVER__SENSOR_MODELS_HPP_

#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>
#include <string_view>

namespace smartmicro::drivers::radar
{
// Model names accepted by the data node. Keep in sync with the registration
// table in smartmicro_radar_node.cpp and param/model_uif_catalogue.yaml.
inline constexpr std::array<std::string_view, 29> kEthernetModels = {
  "umrra4_mse_v3_0_0", "umrra4_mse_v2_1_0", "umrra4_mse_v1_0_0", "umrr9f_mse_v2_0_0",
  "umrr9f_mse_v1_3_0", "umrr9f_mse_v1_1_0", "umrr9f_mse_v1_0_0", "umrr96_v1_2_2",
  "umrr11_v1_1_2", "umrr9f_v1_1_1", "umrr9f_v2_0_0", "umrr9f_v2_1_1", "umrr9f_v2_2_1",
  "umrr9f_v2_4_1", "umrr9f_v3_0_0", "umrr9f_v3_2_0", "umrr9d_v1_0_3", "umrr9d_v1_2_2",
  "umrr9d_v1_4_1", "umrr9d_v1_5_0", "umrr9d_v1_7_0", "umrra4_v1_0_1", "umrra4_v1_2_1",
  "umrra4_v1_4_0", "umrra4_v1_6_0", "umrra1_v1_0_0", "umrra1_v2_0_0", "umrra1_v2_0_1",
  "umrra1_v3_0_0"};

inline constexpr std::array<std::string_view, 23> kCanModels = {
  "umrra4_can_mse_v2_1_0", "umrra4_can_mse_v1_0_0", "umrr9f_can_mse_v1_3_0",
  "umrr9f_can_mse_v1_1_0", "umrr9f_can_mse_v1_0_0", "umrra4_can_mse_v3_0_0",
  "umrr9f_can_mse_v2_0_0", "umrr96_can_v1_2_2", "umrr11_can_v1_1_2", "umrr9f_can_v2_1_1",
  "umrr9f_can_v2_2_1", "umrr9f_can_v2_4_1", "umrr9f_can_v3_0_0", "umrr9f_can_v3_2_0",
  "umrr9d_can_v1_0_3", "umrr9d_can_v1_2_2", "umrr9d_can_v1_4_1", "umrr9d_can_v1_5_0",
  "umrr9d_can_v1_7_0", "umrra4_can_v1_0_1", "umrra4_can_v1_2_1", "umrra4_can_v1_4_0",
  "umrra4_can_v1_6_0"};

inline constexpr std::string_view kEthLinkType = "eth";
inline constexpr std::string_view kCanLinkType = "can";
inline constexpr std::string_view kTargetPubType = "target";
inline constexpr std::string_view kMsePubType = "mse";

template<size_t N>
constexpr bool contains(const std::array<std::string_view, N> & values, std::string_view value)
{
  return std::find(values.begin(), values.end(), value) != values.end();
}

// Rejects configurations the node cannot serve before any publisher or SDK
// callback exists: an unknown combination would otherwise publish nothing, or
// (CAN with an unknown pub_type) dereference a missing publisher on an SDK thread.
inline void validate_sensor_config(
  const std::string & prefix, std::string_view link_type, std::string_view model,
  std::string_view pub_type)
{
  if (link_type != kEthLinkType && link_type != kCanLinkType) {
    throw std::invalid_argument(
            prefix + ".link_type must be 'eth' or 'can', got '" + std::string(link_type) + "'");
  }
  if (pub_type != kTargetPubType && pub_type != kMsePubType) {
    throw std::invalid_argument(
            prefix + ".pub_type must be 'target' or 'mse', got '" + std::string(pub_type) + "'");
  }
  const bool known = link_type == kEthLinkType ?
    contains(kEthernetModels, model) : contains(kCanModels, model);
  if (!known) {
    throw std::invalid_argument(
            prefix + ".model '" + std::string(model) + "' is not a supported " +
            std::string(link_type) + " model; see param/model_uif_catalogue.yaml");
  }
  const bool is_mse = pub_type == kMsePubType;
  const bool has_mse = model.find(kMsePubType) != std::string_view::npos;
  if (is_mse != has_mse) {
    throw std::invalid_argument(
            prefix + ".model '" + std::string(model) + "' " + (is_mse ? "must" : "must not") +
            " contain 'mse' when pub_type is '" + std::string(pub_type) + "'");
  }
}
}  // namespace smartmicro::drivers::radar
#endif  // UMRR_ROS2_DRIVER__SENSOR_MODELS_HPP_
