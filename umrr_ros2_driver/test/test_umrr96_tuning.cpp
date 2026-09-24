// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <umrr_ros2_driver/umrr96_tuning.hpp>

using smartmicro::drivers::radar::validate_umrr96_tuning;

TEST(Umrr96Tuning, TypedFloatAndPrfRequests)
{
  const auto values = validate_umrr96_tuning(
    {"prf_selector_manual", "prf_manual_value_idx", "prf_set_selector",
      "tv_min_speed_sweep_idx_2", "tv_max_speed_sweep_idx_2"},
    {"1", "2", "0", "-12.5", "25.25"}, {3, 3, 3, 0, 0});
  EXPECT_EQ(std::get<uint8_t>(values[1]), 2);
  EXPECT_FLOAT_EQ(std::get<float>(values[3]), -12.5F);
  EXPECT_FLOAT_EQ(std::get<float>(values[4]), 25.25F);
}

TEST(Umrr96Tuning, RejectsMalformedFloatsAndInconsistentWindows)
{
  const std::vector<std::string> names = {"tv_min_speed_sweep_idx_2", "tv_max_speed_sweep_idx_2"};
  for (const auto * invalid : {"nan", "inf", "-inf", "151", "-151", "", " 1", "1x", "1e90"}) {
    EXPECT_THROW(validate_umrr96_tuning(names, {invalid, "150"}, {0, 0}), std::invalid_argument);
  }
  EXPECT_THROW(validate_umrr96_tuning(names, {"5", "-5"}, {0, 0}), std::invalid_argument);
  EXPECT_THROW(validate_umrr96_tuning({names[0]}, {"-20"}, {0}), std::invalid_argument);
  EXPECT_THROW(validate_umrr96_tuning(names, {"-20", "20"}, {3, 3}), std::invalid_argument);
  EXPECT_NO_THROW(validate_umrr96_tuning(names, {"-150", "150"}, {0, 0}));
}

TEST(Umrr96Tuning, RejectsUnsupportedControlsBeforeTransmission)
{
  for (const auto * name : {"ip_source_address", "time_sync_mode", "output_control_target_list_eth",
      "output_control_object_list_eth", "tv_min_speed_sweep_idx_3"})
  {
    EXPECT_THROW(validate_umrr96_tuning({name}, {"1"}, {3}), std::invalid_argument);
  }
  EXPECT_THROW(validate_umrr96_tuning({"prf_set_selector"}, {"1"}, {3}), std::invalid_argument);
  EXPECT_THROW(validate_umrr96_tuning({"prf_manual_value_idx"}, {"3"}, {3}), std::invalid_argument);
  EXPECT_THROW(validate_umrr96_tuning({"prf_selector_manual"}, {"1"}, {0}), std::invalid_argument);
  EXPECT_THROW(validate_umrr96_tuning({"prf_selector_manual", "prf_selector_manual"},
    {"0", "1"}, {3, 3}), std::invalid_argument);
}
