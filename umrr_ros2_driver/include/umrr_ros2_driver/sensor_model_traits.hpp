// SPDX-License-Identifier: Apache-2.0
#ifndef UMRR_ROS2_DRIVER__SENSOR_MODEL_TRAITS_HPP_
#define UMRR_ROS2_DRIVER__SENSOR_MODEL_TRAITS_HPP_

// One traits type per Smart Access user interface compiled into the driver: its
// data stream service, the streams the driver registers and the target data it
// deliberately does not publish. The registration table in
// smartmicro_radar_node.cpp maps parameter model names to these types.

#include <umrr11_t132_automotive_v1_1_2/DataStreamServiceIface.h>
#include <umrr96_t153_automotive_v1_2_2/DataStreamServiceIface.h>
#include <umrr9d_t152_automotive_v1_0_3/DataStreamServiceIface.h>
#include <umrr9d_t152_automotive_v1_2_2/DataStreamServiceIface.h>
#include <umrr9d_t152_automotive_v1_4_1/DataStreamServiceIface.h>
#include <umrr9d_t152_automotive_v1_5_0/DataStreamServiceIface.h>
#include <umrr9d_t152_automotive_v1_7_0/DataStreamServiceIface.h>
#include <umrr9f_t169_automotive_v1_1_1/DataStreamServiceIface.h>
#include <umrr9f_t169_automotive_v2_0_0/DataStreamServiceIface.h>
#include <umrr9f_t169_automotive_v2_1_1/DataStreamServiceIface.h>
#include <umrr9f_t169_automotive_v2_2_1/DataStreamServiceIface.h>
#include <umrr9f_t169_automotive_v2_4_1/DataStreamServiceIface.h>
#include <umrr9f_t169_automotive_v3_0_0/DataStreamServiceIface.h>
#include <umrr9f_t169_automotive_v3_2_0/DataStreamServiceIface.h>
#include <umrr9f_t169_mse_v1_0_0/DataStreamServiceIface.h>
#include <umrr9f_t169_mse_v1_1_0/DataStreamServiceIface.h>
#include <umrr9f_t169_mse_v1_3_0/DataStreamServiceIface.h>
#include <umrr9f_t169_mse_v2_0_0/DataStreamServiceIface.h>
#include <umrra1_t166_b_automotive_v1_0_0/DataStreamServiceIface.h>
#include <umrra1_t166_b_automotive_v2_0_0/DataStreamServiceIface.h>
#include <umrra1_t166_b_automotive_v2_0_1/DataStreamServiceIface.h>
#include <umrra1_t166_b_automotive_v3_0_0/DataStreamServiceIface.h>
#include <umrra4_automotive_v1_0_1/DataStreamServiceIface.h>
#include <umrra4_automotive_v1_2_1/DataStreamServiceIface.h>
#include <umrra4_automotive_v1_4_0/DataStreamServiceIface.h>
#include <umrra4_automotive_v1_6_0/DataStreamServiceIface.h>
#include <umrra4_mse_v1_0_0/DataStreamServiceIface.h>
#include <umrra4_mse_v2_1_0/DataStreamServiceIface.h>
#include <umrra4_mse_v3_0_0/DataStreamServiceIface.h>

#include <umrr_ros2_driver/stream_codecs.hpp>

namespace smartmicro::drivers::radar::models
{
using codec::TargetOptions;

// Streams: Ethernet target list (always), object list, fault reports, and the
// CAN (base) target and object lists.
template<typename ServiceT, bool PortObjects, bool FaultReports, bool CanTargets, bool CanObjects,
  bool TargetListPort = false>
struct ModelBase
{
  using Service = ServiceT;
  static constexpr bool kPortObjects = PortObjects;
  static constexpr bool kFaultReports = FaultReports;
  static constexpr bool kCanTargets = CanTargets;
  static constexpr bool kCanObjects = CanObjects;
  // umrr9f v1.1.1 / v2.0.0 deliver ComTargetListPort via a different registration call.
  static constexpr bool kTargetListPort = TargetListPort;
  static constexpr TargetOptions kTargetOptions{};
};

struct Umrra4MseV300 : ModelBase<com::master::umrra4_mse_v3_0_0::DataStreamServiceIface, true, true,
    true, true>
{
  // AcquisitionTimestampBase exists but has not been published historically.
  static constexpr TargetOptions kTargetOptions = [] {
      TargetOptions o; o.timestamp_base = false;
      return o;
    }();
};
struct Umrra4MseV210 : ModelBase<com::master::umrra4_mse_v2_1_0::DataStreamServiceIface, true,
    false, true, true> {};
struct Umrra4MseV100 : ModelBase<com::master::umrra4_mse_v1_0_0::DataStreamServiceIface, true,
    false, true, true> {};
struct Umrr9fMseV200 : ModelBase<com::master::umrr9f_t169_mse_v2_0_0::DataStreamServiceIface, true,
    true, true, true>
{
  // AcquisitionTimestampBase exists but has not been published historically.
  static constexpr TargetOptions kTargetOptions = [] {
      TargetOptions o; o.timestamp_base = false;
      return o;
    }();
};
struct Umrr9fMseV130 : ModelBase<com::master::umrr9f_t169_mse_v1_3_0::DataStreamServiceIface, true,
    false, true, true> {};
struct Umrr9fMseV110 : ModelBase<com::master::umrr9f_t169_mse_v1_1_0::DataStreamServiceIface, true,
    false, true, true> {};
struct Umrr9fMseV100 : ModelBase<com::master::umrr9f_t169_mse_v1_0_0::DataStreamServiceIface, true,
    false, true, true> {};
struct Umrr96V122 : ModelBase<com::master::umrr96_t153_automotive_v1_2_2::DataStreamServiceIface,
    false, false, true, false>
{
  // FAP/flags semantics are unverified: published raw in Umrr96RawQuality.
  static constexpr TargetOptions kTargetOptions = [] {
      TargetOptions o; o.raw_quality = true; o.acquisition_setup = true;
      return o;
    }();
};
struct Umrr11V112 : ModelBase<com::master::umrr11_t132_automotive_v1_1_2::DataStreamServiceIface,
    false, false, true, false>
{
  // The interface has quality getters; the driver has never published them.
  static constexpr TargetOptions kTargetOptions = [] {
      TargetOptions o; o.variances = false; o.quality = false; o.peak_idx = false;
      return o;
    }();
};
struct Umrr9fV111 : ModelBase<com::master::umrr9f_t169_automotive_v1_1_1::DataStreamServiceIface,
    false, false, false, false, true>
{
  // The static header has AcquisitionStart/Setup; not published historically.
  static constexpr TargetOptions kTargetOptions = [] {
      TargetOptions o; o.acquisition_start = false;
      return o;
    }();
};
struct Umrr9fV200 : ModelBase<com::master::umrr9f_t169_automotive_v2_0_0::DataStreamServiceIface,
    false, false, false, false, true> {};
struct Umrr9fV211 : ModelBase<com::master::umrr9f_t169_automotive_v2_1_1::DataStreamServiceIface,
    false, false, true, false> {};
struct Umrr9fV221 : ModelBase<com::master::umrr9f_t169_automotive_v2_2_1::DataStreamServiceIface,
    false, false, true, false> {};
struct Umrr9fV241 : ModelBase<com::master::umrr9f_t169_automotive_v2_4_1::DataStreamServiceIface,
    false, false, true, false> {};
struct Umrr9fV300 : ModelBase<com::master::umrr9f_t169_automotive_v3_0_0::DataStreamServiceIface,
    false, false, true, false> {};
struct Umrr9fV320 : ModelBase<com::master::umrr9f_t169_automotive_v3_2_0::DataStreamServiceIface,
    false, true, true, false> {};
struct Umrr9dV103 : ModelBase<com::master::umrr9d_t152_automotive_v1_0_3::DataStreamServiceIface,
    false, false, true, false> {};
struct Umrr9dV122 : ModelBase<com::master::umrr9d_t152_automotive_v1_2_2::DataStreamServiceIface,
    false, false, true, false> {};
struct Umrr9dV141 : ModelBase<com::master::umrr9d_t152_automotive_v1_4_1::DataStreamServiceIface,
    false, false, true, false> {};
struct Umrr9dV150 : ModelBase<com::master::umrr9d_t152_automotive_v1_5_0::DataStreamServiceIface,
    false, false, true, false> {};
struct Umrr9dV170 : ModelBase<com::master::umrr9d_t152_automotive_v1_7_0::DataStreamServiceIface,
    false, true, true, false> {};
struct Umrra4V101 : ModelBase<com::master::umrra4_automotive_v1_0_1::DataStreamServiceIface, false,
    false, true, false> {};
struct Umrra4V121 : ModelBase<com::master::umrra4_automotive_v1_2_1::DataStreamServiceIface, false,
    false, true, false> {};
struct Umrra4V140 : ModelBase<com::master::umrra4_automotive_v1_4_0::DataStreamServiceIface, false,
    false, true, false> {};
struct Umrra4V160 : ModelBase<com::master::umrra4_automotive_v1_6_0::DataStreamServiceIface, false,
    true, true, false> {};
struct Umrra1V100 : ModelBase<com::master::umrra1_t166_b_automotive_v1_0_0::DataStreamServiceIface,
    false, false, false, false> {};
struct Umrra1V200 : ModelBase<com::master::umrra1_t166_b_automotive_v2_0_0::DataStreamServiceIface,
    false, false, false, false> {};
struct Umrra1V201 : ModelBase<com::master::umrra1_t166_b_automotive_v2_0_1::DataStreamServiceIface,
    false, false, false, false> {};
struct Umrra1V300 : ModelBase<com::master::umrra1_t166_b_automotive_v3_0_0::DataStreamServiceIface,
    false, false, false, false> {};
}  // namespace smartmicro::drivers::radar::models

#endif  // UMRR_ROS2_DRIVER__SENSOR_MODEL_TRAITS_HPP_
