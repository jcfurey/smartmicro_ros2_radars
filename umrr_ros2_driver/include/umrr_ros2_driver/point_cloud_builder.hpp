// Copyright 2021 Apex.AI, Inc.; adapted by smartmicro.
// SPDX-License-Identifier: Apache-2.0
#ifndef UMRR_ROS2_DRIVER__POINT_CLOUD_BUILDER_HPP_
#define UMRR_ROS2_DRIVER__POINT_CLOUD_BUILDER_HPP_

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace smartmicro::drivers::radar
{
// Values passed by the SDK callbacks. Their C++ layout is not the wire layout.
struct RadarPoint
{
  float x{};
  float y{};
  float z{};
  float radial_speed{};
  float power{};
  float rcs{};
  float noise{};
  float snr{};
  float azimuth_angle{};
  float elevation_angle{};
  float range{};
  float variance_range{};
  float variance_speed{};
  float variance_azimuth_angle{};
  float variance_elevation_angle{};
  float false_alarm_probability{};
  uint32_t flags{};
  uint16_t peak_idx{};
};

struct ObjectPoint
{
  float x{};
  float y{};
  float z{};
  float speed_absolute{};
  float heading{};
  float length{};
  float mileage{};
  float quality{};
  float acceleration{};
  int16_t object_id{};
  uint16_t idle_cycles{};
  uint16_t spline_idx{};
  uint8_t object_class{};
  uint16_t status{};
};

namespace detail
{
// Shared allocation/bookkeeping for the two fixed schemas below.
class CloudBuilder
{
public:
  void reserve(size_t count)
  {
    check_count(count);
    modifier_.reserve(count);
  }

protected:
  using Field = sensor_msgs::msg::PointField;
  struct FieldSpec {const char * name; uint32_t offset; uint8_t datatype;};

  CloudBuilder(
    sensor_msgs::msg::PointCloud2 & cloud, const std::string & frame_id,
    uint32_t stride, std::initializer_list<FieldSpec> fields)
  : cloud_(cloud), modifier_(cloud)
  {
    if (!cloud.fields.empty() || !cloud.data.empty()) {
      throw std::invalid_argument("Cloud builder requires an uninitialized message");
    }
    cloud.header.frame_id = frame_id;
    cloud.height = 1;
    cloud.width = 0;
    cloud.point_step = stride;
    cloud.row_step = 0;
    cloud.is_bigendian = false;
    cloud.is_dense = false;  // Unavailable attributes are represented by NaNs.
    cloud.fields.reserve(fields.size());
    for (const auto & spec : fields) {
      Field field;
      field.name = spec.name;
      field.offset = spec.offset;
      field.datatype = spec.datatype;
      field.count = 1;
      cloud.fields.push_back(field);
    }
  }

  uint8_t * append_point()
  {
    const auto count = static_cast<size_t>(cloud_.width) + 1;
    check_count(count);
    const auto offset = cloud_.data.size();
    modifier_.resize(count);  // New bytes, including padding, are zero-initialized.
    return cloud_.data.data() + offset;
  }

  template<typename T>
  static std::enable_if_t<std::is_integral_v<T>> write(uint8_t * data, size_t offset, T value)
  {
    // Explicit little-endian encoding, independent of host alignment/endianness.
    const auto bits = static_cast<std::make_unsigned_t<T>>(value);
    for (size_t byte = 0; byte < sizeof(T); ++byte) {
      data[offset + byte] = static_cast<uint8_t>(bits >> (8 * byte));
    }
  }

  static void write(uint8_t * data, size_t offset, float value)
  {
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
      "Radar cloud fields require IEEE 754 float32");
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    write(data, offset, bits);
  }

private:
  void check_count(size_t count) const
  {
    if (count > std::numeric_limits<uint32_t>::max() / cloud_.point_step) {
      throw std::length_error("Point cloud row exceeds the uint32 row_step limit");
    }
  }

  sensor_msgs::msg::PointCloud2 & cloud_;
  sensor_msgs::PointCloud2Modifier modifier_;
};
}  // namespace detail

class RadarCloudBuilder : public detail::CloudBuilder
{
public:
  RadarCloudBuilder(sensor_msgs::msg::PointCloud2 & cloud, const std::string & frame_id)
  : CloudBuilder(cloud, frame_id, 72, {
      {"x", 0, Field::FLOAT32}, {"y", 4, Field::FLOAT32}, {"z", 8, Field::FLOAT32},
      {"radial_speed", 12, Field::FLOAT32}, {"power", 16, Field::FLOAT32},
      {"rcs", 20, Field::FLOAT32}, {"noise", 24, Field::FLOAT32}, {"snr", 28, Field::FLOAT32},
      {"azimuth_angle", 32, Field::FLOAT32}, {"elevation_angle", 36, Field::FLOAT32},
      {"range", 40, Field::FLOAT32}, {"variance_range", 44, Field::FLOAT32},
      {"variance_speed", 48, Field::FLOAT32}, {"variance_azimuth_angle", 52, Field::FLOAT32},
      {"variance_elevation_angle", 56, Field::FLOAT32},
      {"false_alarm_probability", 60, Field::FLOAT32}, {"flags", 64, Field::UINT32},
      {"peak_idx", 68, Field::UINT16}})
  {}

  void push_back(const RadarPoint & p)
  {
    auto * data = append_point();
    write(data, 0, p.x);
    write(data, 4, p.y);
    write(data, 8, p.z);
    write(data, 12, p.radial_speed);
    write(data, 16, p.power);
    write(data, 20, p.rcs);
    write(data, 24, p.noise);
    write(data, 28, p.snr);
    write(data, 32, p.azimuth_angle);
    write(data, 36, p.elevation_angle);
    write(data, 40, p.range);
    write(data, 44, p.variance_range);
    write(data, 48, p.variance_speed);
    write(data, 52, p.variance_azimuth_angle);
    write(data, 56, p.variance_elevation_angle);
    write(data, 60, p.false_alarm_probability);
    write(data, 64, p.flags);
    write(data, 68, p.peak_idx);
  }
};

class ObjectCloudBuilder : public detail::CloudBuilder
{
public:
  ObjectCloudBuilder(sensor_msgs::msg::PointCloud2 & cloud, const std::string & frame_id)
  : CloudBuilder(cloud, frame_id, 48, {
      {"x", 0, Field::FLOAT32}, {"y", 4, Field::FLOAT32}, {"z", 8, Field::FLOAT32},
      {"speed_absolute", 12, Field::FLOAT32}, {"heading", 16, Field::FLOAT32},
      {"length", 20, Field::FLOAT32}, {"mileage", 24, Field::FLOAT32},
      {"quality", 28, Field::FLOAT32}, {"acceleration", 32, Field::FLOAT32},
      {"object_id", 36, Field::INT16}, {"idle_cycles", 38, Field::UINT16},
      {"spline_idx", 40, Field::UINT16}, {"object_class", 42, Field::UINT8},
      {"status", 44, Field::UINT16}})
  {}

  void push_back(const ObjectPoint & p)
  {
    auto * data = append_point();
    write(data, 0, p.x);
    write(data, 4, p.y);
    write(data, 8, p.z);
    write(data, 12, p.speed_absolute);
    write(data, 16, p.heading);
    write(data, 20, p.length);
    write(data, 24, p.mileage);
    write(data, 28, p.quality);
    write(data, 32, p.acceleration);
    write(data, 36, p.object_id);
    write(data, 38, p.idle_cycles);
    write(data, 40, p.spline_idx);
    write(data, 42, p.object_class);
    write(data, 44, p.status);
  }
};
}  // namespace smartmicro::drivers::radar
#endif  // UMRR_ROS2_DRIVER__POINT_CLOUD_BUILDER_HPP_
