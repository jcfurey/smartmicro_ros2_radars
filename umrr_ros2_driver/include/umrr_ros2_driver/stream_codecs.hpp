// SPDX-License-Identifier: Apache-2.0
#ifndef UMRR_ROS2_DRIVER__STREAM_CODECS_HPP_
#define UMRR_ROS2_DRIVER__STREAM_CODECS_HPP_

// Conversion of Smart Access SDK data streams into ROS messages, written once
// per stream kind instead of once per sensor model. The functions are templates
// on the SDK list types and do not include SDK headers: model differences are
// resolved with getter detection (renamed getters, fields a user interface does
// not have) plus the explicit per-model options below for data that an
// interface provides but the driver deliberately does not publish.

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <umrr_ros2_driver/point_cloud_builder.hpp>
#include <umrr_ros2_msgs/msg/can_object_header.hpp>
#include <umrr_ros2_msgs/msg/can_target_header.hpp>
#include <umrr_ros2_msgs/msg/port_fault_report.hpp>
#include <umrr_ros2_msgs/msg/port_fault_reports_msg.hpp>
#include <umrr_ros2_msgs/msg/port_object_header.hpp>
#include <umrr_ros2_msgs/msg/port_target_header.hpp>
#include <umrr_ros2_msgs/msg/umrr96_raw_quality.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <utility>

namespace smartmicro::drivers::radar::codec
{
inline constexpr float kFloatSentinel = std::numeric_limits<float>::quiet_NaN();
inline constexpr uint32_t kFlagsSentinel = std::numeric_limits<uint32_t>::max();
inline constexpr uint16_t kU16Sentinel = std::numeric_limits<uint16_t>::max();
inline constexpr uint8_t kU8Sentinel = std::numeric_limits<uint8_t>::max();
// CAN object lists report heading in degrees (UIF signal HeadingDeg, unit _deg);
// port object lists and the published cloud use radians (REP 103).
inline constexpr float kDegreesToRadians = static_cast<float>(3.14159265358979323846 / 180.0);

// Target data an interface may provide but a model does not publish.
struct TargetOptions
{
  bool variances{true};           // variance_* point fields
  bool quality{true};             // false_alarm_probability and flags point fields
  bool peak_idx{true};            // peak_idx point field
  bool raw_quality{false};        // UMRR-96: FAP/flags go to Umrr96RawQuality instead
  bool acquisition_start{true};   // PortTargetHeader.acquisition_start
  bool timestamp_base{true};      // PortTargetHeader.acquisition_time_stamp_base
  bool acquisition_setup{false};  // PortTargetHeader.acquisition_setup (+ _valid)
};

namespace detail
{
#define UMRR_DETECT_GETTER(getter) \
  template<typename T, typename = void> \
  struct has_ ## getter : std::false_type {}; \
  template<typename T> \
  struct has_ ## getter<T, std::void_t<decltype(std::declval<const T &>().getter())>> \
    : std::true_type {}; \
  template<typename T> \
  inline constexpr bool has_ ## getter ## _v = has_ ## getter<T>::value;

UMRR_DETECT_GETTER(GetPortHeader)
UMRR_DETECT_GETTER(GetPortIdentifier)
UMRR_DETECT_GETTER(GetTargetListHeader)
UMRR_DETECT_GETTER(GetAcquisitionTxAntIdx)
UMRR_DETECT_GETTER(GetAcquisitionTx)
UMRR_DETECT_GETTER(GetPrf)
UMRR_DETECT_GETTER(GetUmambiguousSpeed)
UMRR_DETECT_GETTER(GetAcquisitionStart)
UMRR_DETECT_GETTER(GetAcquisitionTimestampBase)
UMRR_DETECT_GETTER(GetRcs)
UMRR_DETECT_GETTER(GetTgtNoise)
UMRR_DETECT_GETTER(GetVarianceRange)
UMRR_DETECT_GETTER(GetFalseAlarmProbability)
UMRR_DETECT_GETTER(GetPeakIdx)
UMRR_DETECT_GETTER(GetCycleDuration)
UMRR_DETECT_GETTER(GetAcqTimeStampFraction)
#undef UMRR_DETECT_GETTER

template<typename T>
using Pointee = typename std::decay_t<T>::element_type;
}  // namespace detail

// Generic port header of an Ethernet port (target, object and fault ports).
template<typename List>
auto port_header(const List & list)
{
  if constexpr (detail::has_GetPortHeader_v<List>) {
    return list.GetPortHeader();
  } else {
    return list.GetGenericPortHeader();  // umrr9f v1.1.1 / v2.0.0 ComTargetListPort
  }
}

template<typename Header, typename Msg>
void fill_generic_port_header(const Header & port, Msg & msg)
{
  if constexpr (detail::has_GetPortIdentifier_v<Header>) {
    msg.port_identifier = port.GetPortIdentifier();
  } else {
    msg.port_identifier = port.GetPortId();
  }
  msg.port_ver_major = port.GetPortVersionMajor();
  msg.port_ver_minor = port.GetPortVersionMinor();
  msg.port_size = port.GetPortSize();
  msg.body_endianness = port.GetBodyEndianness();
  msg.port_index = port.GetPortIndex();
  msg.header_ver_major = port.GetHeaderVersionMajor();
  msg.header_ver_minor = port.GetHeaderVersionMinor();
}

// ---- Ethernet target lists ----------------------------------------------------

template<typename List>
auto port_target_list_header(const List & list)
{
  if constexpr (detail::has_GetTargetListHeader_v<List>) {
    return list.GetTargetListHeader();
  } else {
    return list.GetStaticPortHeader();  // umrr9f v1.1.1 / v2.0.0 ComTargetListPort
  }
}

template<const TargetOptions & Options, typename ListHeader>
void fill_port_target_header(
  const ListHeader & list, umrr_ros2_msgs::msg::PortTargetHeader & header)
{
  header.cycle_time = list.GetCycleTime();
  header.number_of_targets = list.GetNumberOfTargets();
  if constexpr (detail::has_GetAcquisitionTxAntIdx_v<ListHeader>) {
    header.acquisition_tx_ant_idx = list.GetAcquisitionTxAntIdx();
    header.acquisition_sweep_idx = list.GetAcquisitionSweepIdx();
    header.acquisition_cf_idx = list.GetAcquisitionCfIdx();
  } else if constexpr (detail::has_GetAcquisitionTx_v<ListHeader>) {
    // umrr9f v2.0.0: the getter names are swapped relative to their meaning.
    // com_target_list_port.xml documents offset 6 "AcquisitionTx" as the TX
    // antenna index and offset 8 "AcquisitionTxAnt" as the centre frequency
    // index, the same layout as the other port target lists.
    header.acquisition_tx_ant_idx = list.GetAcquisitionTx();
    header.acquisition_sweep_idx = list.GetAcquisitionSweep();
    header.acquisition_cf_idx = list.GetAcquisitionTxAnt();
  }
  if constexpr (detail::has_GetPrf_v<ListHeader>) {
    header.prf = list.GetPrf();
  }
  if constexpr (detail::has_GetUmambiguousSpeed_v<ListHeader>) {
    header.unambiguous_speed = list.GetUmambiguousSpeed();
    header.umambiguous_speed = header.unambiguous_speed;  // Deprecated spelling.
  }
  if constexpr (Options.acquisition_start && detail::has_GetAcquisitionStart_v<ListHeader>) {
    header.acquisition_start = list.GetAcquisitionStart();
  }
  if constexpr (Options.timestamp_base &&
    detail::has_GetAcquisitionTimestampBase_v<ListHeader>)
  {
    header.acquisition_time_stamp_base = list.GetAcquisitionTimestampBase();
  }
  if constexpr (Options.acquisition_setup) {
    header.acquisition_setup = list.GetAcquisitionSetup();
    header.acquisition_setup_valid = true;
  }
}

// Shared geometry of Ethernet and CAN targets: spherical to Cartesian.
template<typename Target>
RadarPoint target_geometry(const Target & target, float power, float noise, float rcs)
{
  const auto range = target.GetRange();
  const auto elevation_angle = target.GetElevationAngle();
  const auto range_2d = range * std::cos(elevation_angle);
  const auto azimuth_angle = target.GetAzimuthAngle();
  RadarPoint point;
  point.x = range_2d * std::cos(azimuth_angle);
  point.y = range_2d * std::sin(azimuth_angle);
  point.z = range * std::sin(elevation_angle);
  point.radial_speed = target.GetSpeedRadial();
  point.power = power;
  point.rcs = rcs;
  point.noise = noise;
  point.snr = power - noise;
  point.azimuth_angle = azimuth_angle;
  point.elevation_angle = elevation_angle;
  point.range = range;
  point.variance_range = kFloatSentinel;
  point.variance_speed = kFloatSentinel;
  point.variance_azimuth_angle = kFloatSentinel;
  point.variance_elevation_angle = kFloatSentinel;
  point.false_alarm_probability = kFloatSentinel;
  point.flags = kFlagsSentinel;
  point.peak_idx = kU16Sentinel;
  return point;
}

template<const TargetOptions & Options, typename Target>
RadarPoint port_target_point(const Target & target)
{
  float noise{};
  if constexpr (detail::has_GetTgtNoise_v<Target>) {
    noise = target.GetTgtNoise();
  } else {
    noise = target.GetNoise();
  }
  float rcs{};
  if constexpr (detail::has_GetRcs_v<Target>) {
    rcs = target.GetRcs();
  } else {
    rcs = target.GetRCS();
  }
  auto point = target_geometry(target, target.GetPower(), noise, rcs);
  if constexpr (Options.variances && detail::has_GetVarianceRange_v<Target>) {
    point.variance_range = target.GetVarianceRange();
    point.variance_speed = target.GetVarianceSpeed();
    point.variance_azimuth_angle = target.GetVarianceAzimuthAngle();
    point.variance_elevation_angle = target.GetVarianceElevationAngle();
  }
  if constexpr (Options.quality && !Options.raw_quality &&
    detail::has_GetFalseAlarmProbability_v<Target>)
  {
    point.false_alarm_probability = target.GetFalseAlarmProbability();
    point.flags = target.GetFlags();
  }
  if constexpr (Options.peak_idx && detail::has_GetPeakIdx_v<Target>) {
    point.peak_idx = target.GetPeakIdx();
  }
  return point;
}

// Fills the header and cloud of an Ethernet target list. With raw_quality the
// UMRR-96 false-alarm probability and flags are appended to *raw (if not null).
template<const TargetOptions & Options, typename List>
void convert_port_targets(
  const List & list, umrr_ros2_msgs::msg::PortTargetHeader & header,
  RadarCloudBuilder & cloud, umrr_ros2_msgs::msg::Umrr96RawQuality * raw = nullptr)
{
  fill_generic_port_header(*port_header(list), header);
  fill_port_target_header<Options>(*port_target_list_header(list), header);
  const auto & targets = list.GetTargetList();
  cloud.reserve(targets.size());
  if constexpr (Options.raw_quality) {
    if (raw) {
      raw->false_alarm_probability_raw.reserve(targets.size());
      raw->flags_raw.reserve(targets.size());
    }
  }
  for (const auto & target : targets) {
    cloud.push_back(port_target_point<Options>(*target));
    if constexpr (Options.raw_quality) {
      if (raw) {
        raw->false_alarm_probability_raw.push_back(target->GetFalseAlarmProbability());
        raw->flags_raw.push_back(target->GetFlags());
      }
    }
  }
}

// ---- Ethernet object lists ----------------------------------------------------

template<typename List>
void convert_port_objects(
  const List & list, umrr_ros2_msgs::msg::PortObjectHeader & header, ObjectCloudBuilder & cloud)
{
  fill_generic_port_header(*list.GetPortHeader(), header);
  const auto object_header = list.GetObjectListHeader();
  header.cycle_time = object_header->GetCycleTime();
  header.number_of_objects = object_header->GetNumberOfObjects();
  header.ts_measurement = object_header->GetTimestampOfMeasurement();
  const auto & objects = list.GetObjectList();
  cloud.reserve(objects.size());
  for (const auto & object : objects) {
    cloud.push_back(
      {object->GetPosX(), object->GetPosY(), object->GetPosZ(), object->GetSpeedAbs(),
        object->GetHeading(), object->GetLength(), object->GetMileage(), object->GetQuality(),
        object->GetAcceleration(), object->GetObjectId(), object->GetIdleCycles(),
        object->GetSplineIdx(), object->GetObjectClass(), object->GetStatus()});
  }
}

// ---- Ethernet fault reports ---------------------------------------------------

template<typename Reports>
void convert_fault_reports(const Reports & reports, umrr_ros2_msgs::msg::PortFaultReportsMsg & msg)
{
  auto & header = msg.fault_report_header;
  fill_generic_port_header(*reports.GetPortHeader(), header);
  const auto fault_header = reports.GetFaultReportHeader();
  header.num_max_reports = fault_header->GetNumMaxReports();
  header.num_valid_reports = fault_header->GetNumValidReports();
  header.faults_time_line = fault_header->GetFaultsTimeline();
  const auto & faults = reports.GetFaultReportList();
  msg.reports.reserve(faults.size());
  for (const auto & fault : faults) {
    if (!fault) {
      continue;
    }
    umrr_ros2_msgs::msg::PortFaultReport report;
    report.module_id = fault->GetModuleId();
    report.fault_group = fault->GetFaultGroup();
    report.fault_code = fault->GetFaultCode();
    report.fault_errno = fault->GetFaultErrno();
    report.fault_time_stamp = fault->GetFaultTimestamp();
    report.cycle_count = fault->GetCycleCount();
    report.instance_id = fault->GetInstanceId();
    report.criticality = fault->GetCriticality();
    report.occurrence_count = fault->GetOccurrenceCount();
    report.occurence_count = report.occurrence_count;  // Deprecated spelling.
    msg.reports.push_back(report);
  }
}

// ---- CAN target and object lists ----------------------------------------------

template<typename List>
void convert_can_targets(
  const List & list, umrr_ros2_msgs::msg::CanTargetHeader & header, RadarCloudBuilder & cloud)
{
  const auto target_header = list.GetTargetListHeader();
  using ListHeader = detail::Pointee<decltype(target_header)>;
  if constexpr (detail::has_GetCycleDuration_v<ListHeader>) {
    header.cycle_time = target_header->GetCycleDuration();
  }
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.cycle_count = target_header->GetCycleCount();
  header.time_stamp = target_header->GetTimeStamp();
  if constexpr (detail::has_GetAcqTimeStampFraction_v<ListHeader>) {
    header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
  }
  const auto & targets = list.GetTargetList();
  cloud.reserve(targets.size());
  for (const auto & target : targets) {
    // CAN target lists carry no variances, false-alarm probability, flags or peak index.
    cloud.push_back(
      target_geometry(*target, target->GetSignalLevel(), target->GetNoise(), target->GetRCS()));
  }
}

template<typename List>
void convert_can_objects(
  const List & list, umrr_ros2_msgs::msg::CanObjectHeader & header, ObjectCloudBuilder & cloud)
{
  const auto object_header = list.GetComObjectBaseListHeader();
  header.cycle_time = object_header->GetCycleDuration();
  header.cycle_count = object_header->GetCycleCount();
  header.number_of_objects = object_header->GetNoOfObjects();
  header.ego_speed = object_header->GetSpeed();
  header.ego_speed_quality = object_header->GetSpeedQuality();
  header.ego_yaw_rate = object_header->GetYawRate();
  header.ego_yaw_rate_quality = object_header->GetYawRateQuality();
  header.dyn_source = object_header->GetDynamicSource();
  const auto & objects = list.GetObjectList();
  cloud.reserve(objects.size());
  for (const auto & object : objects) {
    // The SDK object id is uint16; the cloud field stays int16 for compatibility.
    cloud.push_back(
      {object->GetXPoint1(), object->GetYPoint1(), object->GetZPoint1(), object->GetSpeedAbs(),
        object->GetHeadingDeg() * kDegreesToRadians, object->GetObjectLen(), kFloatSentinel,
        object->GetQuality(), object->GetAcceleration(),
        static_cast<int16_t>(object->GetObjectId()), kU16Sentinel, kU16Sentinel, kU8Sentinel,
        kU16Sentinel});
  }
}
}  // namespace smartmicro::drivers::radar::codec

#endif  // UMRR_ROS2_DRIVER__STREAM_CODECS_HPP_
