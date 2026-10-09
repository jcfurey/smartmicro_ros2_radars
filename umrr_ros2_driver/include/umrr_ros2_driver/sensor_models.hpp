// SPDX-License-Identifier: Apache-2.0
#ifndef UMRR_ROS2_DRIVER__SENSOR_MODELS_HPP_
#define UMRR_ROS2_DRIVER__SENSOR_MODELS_HPP_

#include <algorithm>
#include <array>
#include <charconv>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <tuple>
#include <utility>

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

// The Smart Access user interface a model is compiled against: the name of its
// sensor family and the version of its suffix (umrr96_can_v1_2_2 ->
// umrr96_t153_automotive 1.2.2), as listed in param/model_uif_catalogue.yaml.
struct UserInterface
{
  std::string name;
  uint32_t major{}, minor{}, patch{};
};

inline UserInterface model_user_interface(std::string_view model)
{
  static constexpr std::array<std::pair<std::string_view, std::string_view>, 8> kFamilies{{
    {"umrr11", "umrr11_t132_automotive"}, {"umrr96", "umrr96_t153_automotive"},
    {"umrr9d", "umrr9d_t152_automotive"}, {"umrr9f", "umrr9f_t169_automotive"},
    {"umrr9f_mse", "umrr9f_t169_mse"}, {"umrra1", "umrra1_t166_b_automotive"},
    {"umrra4", "umrra4_automotive"}, {"umrra4_mse", "umrra4_mse"}}};
  const auto unknown = [model] {
      return std::invalid_argument("No user interface for model '" + std::string(model) + "'");
    };
  const auto suffix = model.rfind("_v");
  if (suffix == std::string_view::npos) {throw unknown();}
  auto family = std::string(model.substr(0, suffix));
  if (const auto can = family.find("_can"); can != std::string::npos) {
    family.erase(can, 4);
  }
  const auto entry = std::find_if(
    kFamilies.begin(), kFamilies.end(), [&family](const auto & f) {return f.first == family;});
  if (entry == kFamilies.end()) {throw unknown();}
  UserInterface result{std::string(entry->second)};
  const auto * cursor = model.data() + suffix + 2;
  const auto * const end = model.data() + model.size();
  for (auto * part : {&result.major, &result.minor, &result.patch}) {
    const auto parsed = std::from_chars(cursor, end, *part);
    if (parsed.ec != std::errc{} || (parsed.ptr != end && *parsed.ptr != '_') ||
      (part == &result.patch) != (parsed.ptr == end))
    {
      throw unknown();
    }
    cursor = parsed.ptr + (parsed.ptr != end);
  }
  return result;
}

// The SDK decodes a client's data with the user interface named in its routing
// table entry, while the node registers callbacks for the model's interface only:
// a different name or version starts cleanly but never delivers data. An unset
// interface (empty name, zero version) is taken from the model; any set field
// must match it.
inline UserInterface resolve_user_interface(
  const std::string & prefix, std::string_view model, const std::string & name,
  uint32_t major, uint32_t minor, uint32_t patch)
{
  auto expected = model_user_interface(model);
  if (name.empty() && !major && !minor && !patch) {
    return expected;
  }
  const auto mismatch = [&](const std::string & parameter, const std::string & value,
    const std::string & wanted) {
      return std::invalid_argument(
        prefix + "." + parameter + " " + value + " does not match model '" +
        std::string(model) + "' (expects " + wanted + ")");
    };
  if (name != expected.name) {
    throw mismatch("uifname", "'" + name + "'", "'" + expected.name + "'");
  }
  for (const auto & [parameter, value, wanted] : {
      std::tuple{"uifmajorv", major, expected.major},
      std::tuple{"uifminorv", minor, expected.minor},
      std::tuple{"uifpatchv", patch, expected.patch}})
  {
    if (value != wanted) {
      throw mismatch(parameter, std::to_string(value), std::to_string(wanted));
    }
  }
  return expected;
}
}  // namespace smartmicro::drivers::radar
#endif  // UMRR_ROS2_DRIVER__SENSOR_MODELS_HPP_
