// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <umrr_ros2_driver/point_cloud_builder.hpp>

#include <array>
#include <cstring>
#include <vector>

using smartmicro::drivers::radar::RadarCloudBuilder;
using smartmicro::drivers::radar::RadarPoint;
using smartmicro::drivers::radar::ObjectCloudBuilder;
using smartmicro::drivers::radar::ObjectPoint;
using Cloud = sensor_msgs::msg::PointCloud2;
using Field = sensor_msgs::msg::PointField;

namespace
{
float from_bits(uint32_t bits)
{
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

std::vector<uint8_t> bytes(const std::string & hex)
{
  std::vector<uint8_t> result;
  for (size_t i = 0; i < hex.size(); i += 2) {
    result.push_back(static_cast<uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
  }
  return result;
}

void expect_fields(
  const Cloud & cloud, const std::vector<std::string> & names,
  const std::vector<uint32_t> & offsets, const std::vector<uint8_t> & types)
{
  ASSERT_EQ(cloud.fields.size(), names.size());
  for (size_t i = 0; i < names.size(); ++i) {
    EXPECT_EQ(cloud.fields[i].name, names[i]);
    EXPECT_EQ(cloud.fields[i].offset, offsets[i]);
    EXPECT_EQ(cloud.fields[i].datatype, types[i]);
    EXPECT_EQ(cloud.fields[i].count, 1U);
  }
}

template<typename Builder, typename Point>
void check_records(const Point & point, const std::vector<uint8_t> & golden)
{
  for (const auto count : {0U, 1U, 30U, 1024U}) {
    Cloud cloud;
    cloud.header.stamp.sec = 123;
    cloud.header.stamp.nanosec = 456;
    Builder builder(cloud, "radar_test");
    builder.reserve(count);
    const auto capacity = cloud.data.capacity();
    for (size_t i = 0; i < count; ++i) {builder.push_back(point);}
    EXPECT_EQ(cloud.data.capacity(), capacity);
    EXPECT_EQ(cloud.header.frame_id, "radar_test");
    EXPECT_EQ(cloud.header.stamp.sec, 123);
    EXPECT_EQ(cloud.header.stamp.nanosec, 456U);
    EXPECT_EQ(cloud.height, 1U);
    EXPECT_EQ(cloud.width, count);
    EXPECT_EQ(cloud.point_step, golden.size());
    EXPECT_EQ(cloud.row_step, count * golden.size());
    EXPECT_EQ(cloud.data.size(), cloud.row_step);
    EXPECT_FALSE(cloud.is_bigendian);
    EXPECT_FALSE(cloud.is_dense);
    for (size_t i = 0; i < count; ++i) {
      EXPECT_EQ(std::vector<uint8_t>(cloud.data.begin() + i * golden.size(),
        cloud.data.begin() + (i + 1) * golden.size()), golden);
    }
  }
}
}  // namespace

TEST(PointCloudBuilder, TargetSchemaKeepsPublishedOffsetsAndTypes)
{
  Cloud cloud;
  RadarCloudBuilder builder(cloud, "radar_test");
  expect_fields(cloud,
    {"x", "y", "z", "radial_speed", "power", "rcs", "noise", "snr", "azimuth_angle",
      "elevation_angle", "range", "variance_range", "variance_speed", "variance_azimuth_angle",
      "variance_elevation_angle", "false_alarm_probability", "flags", "peak_idx"},
    {0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68},
    {7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, Field::UINT32, Field::UINT16});
}

TEST(PointCloudBuilder, ObjectSchemaKeepsIntegerSignednessAndPadding)
{
  Cloud cloud;
  ObjectCloudBuilder builder(cloud, "radar_test");
  expect_fields(cloud,
    {"x", "y", "z", "speed_absolute", "heading", "length", "mileage", "quality",
      "acceleration", "object_id", "idle_cycles", "spline_idx", "object_class", "status"},
    {0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 38, 40, 42, 44},
    {7, 7, 7, 7, 7, 7, 7, 7, 7, Field::INT16, Field::UINT16, Field::UINT16,
      Field::UINT8, Field::UINT16});
}

TEST(PointCloudBuilder, TargetRecordsMatchIndependentLittleEndianFixture)
{
  // Python struct.pack('<16fIH2x', ...) plus the explicit NaN payload at offset 60.
  const RadarPoint point{
    1, -2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, from_bits(0x7fc12345),
    0x89abcdef, 0xfedc};
  check_records<RadarCloudBuilder>(point, bytes(
      "0000803f000000c000004040000080400000a0400000c0400000e04000000041"
      "000010410000204100003041000040410000504100006041000070414523c17f"
      "efcdab89dcfe0000"));
}

TEST(PointCloudBuilder, ObjectRecordsMatchIndependentLittleEndianFixture)
{
  // Python struct.pack('<9fhHHBxH2x', ...); class and status have a one-byte gap.
  const ObjectPoint point{1, -2, 3, 4, 5, 6, 7, 8, 9, -32768, 0xabcd, 0x0123, 0xe5, 0xfedc};
  check_records<ObjectCloudBuilder>(point, bytes(
      "0000803f000000c000004040000080400000a0400000c0400000e04000000041"
      "000010410080cdab2301e500dcfe0000"));
}

TEST(PointCloudBuilder, PreservesSpecialFloatsAndIntegerLimitsAcrossAppends)
{
  Cloud targets, objects;
  RadarCloudBuilder radar(targets, "radar_test");
  ObjectCloudBuilder object(objects, "radar_test");
  const std::array<uint32_t, 10> patterns{
    0, 0x80000000, 1, 0x80000001, 0x7f7fffff, 0xff7fffff,
    0x7f800000, 0xff800000, 0x7fc12345, 0xffc54321};
  for (const auto pattern : patterns) {
    RadarPoint r;
    r.x = from_bits(pattern);
    r.false_alarm_probability = from_bits(pattern);
    r.flags = UINT32_MAX;
    r.peak_idx = UINT16_MAX;
    radar.push_back(r);
    ObjectPoint o;
    o.x = from_bits(pattern);
    o.object_id = -1;
    o.idle_cycles = UINT16_MAX;
    o.spline_idx = UINT16_MAX;
    o.object_class = UINT8_MAX;
    o.status = UINT16_MAX;
    object.push_back(o);
  }
  for (size_t i = 0; i < patterns.size(); ++i) {
    for (size_t byte = 0; byte < 4; ++byte) {
      const auto expected = static_cast<uint8_t>(patterns[i] >> (8 * byte));
      EXPECT_EQ(targets.data[i * 72 + byte], expected);
      EXPECT_EQ(targets.data[i * 72 + 60 + byte], expected);
      EXPECT_EQ(objects.data[i * 48 + byte], expected);
    }
    for (size_t byte = 64; byte < 70; ++byte) {EXPECT_EQ(targets.data[i * 72 + byte], 255);}
    for (size_t byte = 36; byte < 46; ++byte) {
      EXPECT_EQ(objects.data[i * 48 + byte], byte == 43 ? 0 : 255);
    }
    EXPECT_EQ(targets.data[i * 72 + 70], 0);
    EXPECT_EQ(targets.data[i * 72 + 71], 0);
    EXPECT_EQ(objects.data[i * 48 + 46], 0);
    EXPECT_EQ(objects.data[i * 48 + 47], 0);
  }
}

TEST(PointCloudBuilder, RejectsRowOverflowAndAlreadyInitializedMessages)
{
  Cloud cloud;
  RadarCloudBuilder builder(cloud, "radar_test");
  const auto too_many = static_cast<size_t>(UINT32_MAX) / 72 + 1;
  EXPECT_THROW(builder.reserve(too_many), std::length_error);
  EXPECT_TRUE(cloud.data.empty());
  EXPECT_EQ(cloud.width, 0U);
  EXPECT_EQ(cloud.row_step, 0U);
  EXPECT_THROW((RadarCloudBuilder{cloud, "other"}), std::invalid_argument);
  EXPECT_EQ(cloud.header.frame_id, "radar_test");
}
