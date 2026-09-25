// Copyright 2021 Apex.AI, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// The initial version of the code was developed by Apex.AI and
// was thereafter adapted and extended by smartmicro.

#include "umrr_ros2_driver/smartmicro_radar_node.hpp"

#include <nlohmann/json.hpp>
#include <umrr_ros2_driver/point_cloud_builder.hpp>
#include <umrr_ros2_driver/sensor_models.hpp>
#include <umrr_ros2_driver/service_parsing.hpp>
#include <umrr_ros2_driver/udp_socket_health.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <umrr11_t132_automotive_v1_1_2/comtargetlist/PortHeader.h>
#include <umrr11_t132_automotive_v1_1_2/comtargetlist/Target.h>
#include <umrr96_t153_automotive_v1_2_2/comtargetlist/PortHeader.h>
#include <umrr96_t153_automotive_v1_2_2/comtargetlist/Target.h>
#include <umrr9d_t152_automotive_v1_0_3/comtargetlist/PortHeader.h>
#include <umrr9d_t152_automotive_v1_0_3/comtargetlist/Target.h>
#include <umrr9d_t152_automotive_v1_2_2/comtargetlist/PortHeader.h>
#include <umrr9d_t152_automotive_v1_2_2/comtargetlist/Target.h>
#include <umrr9d_t152_automotive_v1_4_1/comtargetlist/PortHeader.h>
#include <umrr9d_t152_automotive_v1_4_1/comtargetlist/Target.h>
#include <umrr9d_t152_automotive_v1_5_0/comtargetlist/PortHeader.h>
#include <umrr9d_t152_automotive_v1_5_0/comtargetlist/Target.h>
#include <umrr9f_t169_automotive_v3_2_0/comtargetlist/PortHeader.h>
#include <umrr9f_t169_automotive_v3_2_0/comtargetlist/Target.h>
#include <umrr9f_t169_automotive_v3_2_0/faultreports/FaultReports.h>
#include <umrr9f_t169_automotive_v3_2_0/faultreports/PortHeader.h>

#include <umrr9d_t152_automotive_v1_7_0/faultreports/PortHeader.h>
#include <umrr9d_t152_automotive_v1_7_0/faultreports/FaultReports.h>

#include <umrr9d_t152_automotive_v1_7_0/comtargetlist/PortHeader.h>
#include <umrr9d_t152_automotive_v1_7_0/comtargetlist/Target.h>
#include <umrr9d_t152_automotive_v1_7_0/comtargetbaselist/ComTargetBaseList.h>
#include <umrr9d_t152_automotive_v1_7_0/comtargetbaselist/Target.h>
#include <umrr9f_t169_mse_v2_0_0/comobjectlist/ComObjectList.h>
#include <umrr9f_t169_mse_v2_0_0/comobjectlist/Object.h>
#include <umrr9f_t169_mse_v2_0_0/comobjectbaselist/ComObjectBaseList.h>
#include <umrr9f_t169_mse_v2_0_0/comobjectbaselist/Object.h>
#include <umrr9f_t169_mse_v2_0_0/comtargetbaselist/ComTargetBaseList.h>
#include <umrr9f_t169_mse_v2_0_0/comtargetbaselist/Target.h>

#include <umrr9f_t169_automotive_v1_1_1/comtargetlistport/GenericPortHeader.h>
#include <umrr9f_t169_automotive_v1_1_1/comtargetlistport/Target.h>
#include <umrr9f_t169_automotive_v2_0_0/comtargetlistport/GenericPortHeader.h>
#include <umrra1_t166_b_automotive_v2_0_1/comtargetlist/PortHeader.h>
#include <umrra1_t166_b_automotive_v2_0_1/comtargetlist/Target.h>
#include <umrra1_t166_b_automotive_v3_0_0/comtargetlist/PortHeader.h>
#include <umrra1_t166_b_automotive_v3_0_0/comtargetlist/Target.h>
#include <umrr9f_t169_automotive_v2_0_0/comtargetlistport/Target.h>
#include <umrr9f_t169_automotive_v2_1_1/comtargetlist/PortHeader.h>
#include <umrr9f_t169_automotive_v2_1_1/comtargetlist/Target.h>
#include <umrr9f_t169_automotive_v2_2_1/comtargetlist/PortHeader.h>
#include <umrr9f_t169_automotive_v2_2_1/comtargetlist/Target.h>
#include <umrr9f_t169_automotive_v2_4_1/comtargetlist/PortHeader.h>
#include <umrra4_automotive_v1_6_0/comtargetlist/PortHeader.h>
#include <umrra4_automotive_v1_6_0/comtargetlist/Target.h>
#include <umrra4_automotive_v1_6_0/faultreports/FaultReports.h>
#include <umrra4_automotive_v1_6_0/faultreports/PortHeader.h>
#include <umrr9f_t169_automotive_v2_4_1/comtargetlist/Target.h>
#include <umrr9f_t169_automotive_v3_0_0/comtargetlist/PortHeader.h>
#include <umrr9f_t169_automotive_v3_0_0/comtargetlist/Target.h>
#include <umrr9f_t169_automotive_v3_2_0/comtargetbaselist/ComTargetBaseList.h>
#include <umrr9f_t169_automotive_v3_2_0/comtargetbaselist/Target.h>
#include <umrra4_automotive_v1_6_0/comtargetbaselist/ComTargetBaseList.h>
#include <umrra4_automotive_v1_6_0/comtargetbaselist/Target.h>

#include <umrra4_mse_v3_0_0/comobjectlist/ComObjectList.h>
#include <umrra4_mse_v3_0_0/comobjectlist/Object.h>
#include <umrra4_mse_v3_0_0/comobjectbaselist/ComObjectBaseList.h>
#include <umrra4_mse_v3_0_0/comobjectbaselist/Object.h>
#include <umrra4_mse_v3_0_0/comtargetbaselist/ComTargetBaseList.h>
#include <umrra4_mse_v3_0_0/comtargetbaselist/Target.h>

#include <umrr9f_t169_mse_v1_0_0/comobjectlist/ComObjectList.h>
#include <umrr9f_t169_mse_v1_0_0/comobjectlist/Object.h>
#include <umrr9f_t169_mse_v1_1_0/comobjectlist/ComObjectList.h>
#include <umrr9f_t169_mse_v1_1_0/comobjectlist/Object.h>
#include <umrr9f_t169_mse_v1_3_0/comobjectlist/ComObjectList.h>
#include <umrr9f_t169_mse_v1_3_0/comobjectlist/Object.h>

#include <umrra1_t166_b_automotive_v1_0_0/comtargetlist/PortHeader.h>
#include <umrra1_t166_b_automotive_v1_0_0/comtargetlist/Target.h>
#include <umrra1_t166_b_automotive_v2_0_0/comtargetlist/PortHeader.h>
#include <umrra1_t166_b_automotive_v2_0_0/comtargetlist/Target.h>

#include <umrra4_automotive_v1_0_1/comtargetlist/PortHeader.h>
#include <umrra4_automotive_v1_0_1/comtargetlist/Target.h>
#include <umrra4_automotive_v1_2_1/comtargetlist/PortHeader.h>
#include <umrra4_automotive_v1_2_1/comtargetlist/Target.h>
#include <umrra4_automotive_v1_4_0/comtargetlist/PortHeader.h>
#include <umrra4_automotive_v1_4_0/comtargetlist/Target.h>

#include <umrra4_mse_v1_0_0/comobjectlist/ComObjectList.h>
#include <umrra4_mse_v1_0_0/comobjectlist/Object.h>
#include <umrra4_mse_v2_1_0/comobjectlist/ComObjectList.h>
#include <umrra4_mse_v2_1_0/comobjectlist/Object.h>

#include <fstream>
#include <limits>
#include <memory>
#include <utility>
#include <string>
#include <variant>
#include <vector>

#include "umrr_ros2_driver/config_path.hpp"

using com::common::Instruction;
using com::master::CmdRequest;
using com::master::CommunicationServicesIface;
using com::master::GetParamRequest;
using com::master::GetStatusRequest;
using com::master::InstructionBatch;
using com::master::InstructionServiceIface;
using com::master::Response;
using com::master::ResponseBatch;
using com::master::SetParamRequest;
using smartmicro::drivers::radar::ModeValue;
using smartmicro::drivers::radar::parse_mode_value;

namespace
{
using smartmicro::drivers::radar::kEthLinkType;
using smartmicro::drivers::radar::kMsePubType;
using smartmicro::drivers::radar::kTargetPubType;
using smartmicro::drivers::radar::validate_sensor_config;

constexpr auto kDefaultClientId = 0;
constexpr auto kDefaultPort = 55555;
constexpr auto kDefaultHistorySize = 10;
constexpr auto kDefaultFrameId = "umrr";
constexpr auto kDefaultSensorType = "umrr11";

constexpr auto kDefaultHwDevId = 0;
constexpr auto kDefaultHwDevIface = "slcan";
constexpr auto kDefaultHwLinkType = "can";

constexpr auto kHwDevLinkTag = "type";
constexpr auto kClientLinkTag = "link_type";
constexpr auto kHwDevIdTag = "dev_id";
constexpr auto kHwDevIfaceNameTag = "iface_name";
constexpr auto kHwDevIpAddressTag = "ip_address";

constexpr auto kClientIdTag = "client_id";
constexpr auto kPortTag = "port";
constexpr auto kBaudRateTag = "baudrate";
constexpr auto kIpTag = "ip";

constexpr auto kInstSerialTypeTag = "master_inst_serial_type";
constexpr auto kDataSerialTypeTag = "master_data_serial_type";

constexpr auto kInstSerialTypeJsonTag = "instruction_serialization_type";
constexpr auto kDataSerialTypeJsonTag = "data_serialization_type";

constexpr bool supports_fault_reports(std::string_view model) noexcept
{
  return model == "umrra4_mse_v3_0_0" || model == "umrr9f_mse_v2_0_0" ||
         model == "umrr9f_v3_2_0" || model == "umrr9d_v1_7_0" ||
         model == "umrra4_v1_6_0";
}

constexpr auto kClientsJsonTag = "clients";
constexpr auto kHwItemsJsonTag = "hwItems";
constexpr auto kUINameTag = "user_interface_name";
constexpr auto kUIMajorVTag = "user_interface_major_v";
constexpr auto kUIMinorVTag = "user_interface_minor_v";
constexpr auto kUIPatchVTag = "user_interface_patch_v";

// CAN object lists report heading in degrees (UIF signal HeadingDeg, unit _deg);
// port object lists and the published cloud use radians (REP 103).
constexpr float kDegreesToRadians = static_cast<float>(3.14159265358979323846 / 180.0);
constexpr float kRadarFloatSentinel = std::numeric_limits<float>::quiet_NaN();
constexpr uint32_t kRadarFlagsSentinel = std::numeric_limits<uint32_t>::max();
constexpr uint16_t kRadarPeakIdxSentinel = std::numeric_limits<uint16_t>::max();
constexpr uint16_t kRadarU16Sentinel = std::numeric_limits<uint16_t>::max();
constexpr uint8_t kRadarU8Sentinel = std::numeric_limits<uint8_t>::max();

}  // namespace

namespace smartmicro
{
namespace drivers
{
namespace radar
{
SmartmicroRadarNode::SmartmicroRadarNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node{"smartmicro_radar_node", node_options}
{
  callback_gate_.set_error_handler([logger = get_logger()](const std::string & message) {
      RCLCPP_ERROR(logger, "%s", message.c_str());
    });
  try {
    update_config_files_from_params();
    update_service = std::make_shared<UpdateService>();

    runtime_config_.activate();
    setup_diagnostics();

    initialize_services();
    setup_publishers();

    get_node_base_interface()->get_context()->on_shutdown(callback_gate_.shutdown_callback());
  } catch (...) {
    callback_gate_.close();
    throw;
  }
}

SmartmicroRadarNode::~SmartmicroRadarNode()
{
  // Drains running SDK callbacks and drops late ones before members are destroyed.
  callback_gate_.close();
}

builtin_interfaces::msg::Time SmartmicroRadarNode::receive_stamp(
  uint64_t timestamp_us, uint32_t sensor_idx, uint8_t stream)
{
  auto timing_ptr = std::make_unique<umrr_ros2_msgs::msg::RadarTiming>();
  auto & timing = *timing_ptr;
  timing.header.stamp = now();
  timing.header.frame_id = m_sensors[sensor_idx].frame_id;
  timing.sensor_id = m_sensors[sensor_idx].id;
  timing.device_timestamp_us = timestamp_us;
  timing.stream = stream;
  timing.timestamp_source = umrr_ros2_msgs::msg::RadarTiming::ROS_RECEIVE_TIME;
  if (stream == umrr_ros2_msgs::msg::RadarTiming::TARGETS) {
    target_health_[sensor_idx].receive(timestamp_us);
  }
  const auto stamp = timing.header.stamp;
  timing_publishers_[sensor_idx]->publish(std::move(timing_ptr));
  return stamp;
}

void SmartmicroRadarNode::setup_diagnostics()
{
  auto descriptor = startup_descriptor();
  descriptor.description = "Target stream silence threshold in seconds; restart to change.";
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = 0.1;
  range.to_value = 3600.0;
  descriptor.floating_point_range.push_back(range);
  stale_timeout_seconds_ = declare_parameter("diagnostics.stale_timeout", 2.0, descriptor);
  if (!std::isfinite(stale_timeout_seconds_)) {
    throw std::invalid_argument("diagnostics.stale_timeout must be finite");
  }
  diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
  // hardware_id identifies the physical device: <model>@<ip> for Ethernet,
  // <model>@can<dev_id> for CAN (the client id is reported as a value).
  const auto sensor_hardware_id = [this](size_t i) {
      const auto & sensor = m_sensors[i];
      return sensor.model + "@" +
             (sensor.link_type == kEthLinkType ? sensor.ip : "can" + std::to_string(sensor.dev_id));
    };
  diagnostics_->setHardwareID(
    m_number_of_sensors == 1 ? sensor_hardware_id(0) : std::string{"smartmicro_radar"});
  for (size_t i = 0; i < m_number_of_sensors; ++i) {
    timing_publishers_[i] = create_publisher<umrr_ros2_msgs::msg::RadarTiming>(
      "smart_radar/timing_" + std::to_string(i), m_sensors[i].history_size);
    if (m_sensors[i].model == "umrr96_v1_2_2" && m_sensors[i].data_type == "port_based") {
      raw_quality_publishers_[i] = create_publisher<umrr_ros2_msgs::msg::Umrr96RawQuality>(
        "smart_radar/umrr96_raw_quality_" + std::to_string(i), m_sensors[i].history_size);
    }
    diagnostics_->add("Target stream " + std::to_string(i),
      [this, i, hardware_id = sensor_hardware_id(i)](
        diagnostic_updater::DiagnosticStatusWrapper & status) {
        using Status = diagnostic_msgs::msg::DiagnosticStatus;
        const auto health = target_health_[i].snapshot();
        status.hardware_id = hardware_id;
        status.add("sensor_id", m_sensors[i].id);
        if (!health.frames || health.age_seconds > stale_timeout_seconds_) {
          status.summary(Status::STALE, health.frames ? "No recent targets" : "Waiting for targets");
        } else if (health.timestamp_warning) {
          status.summary(Status::WARN, "Device timestamp anomaly; headers use ROS receive time");
        } else {
          status.summary(Status::OK, "Receiving targets");
        }
        status.add("frames_received", health.frames);
        status.add("last_receive_age_seconds", health.age_seconds);
        status.add("frequency_hz", health.frequency_hz);
        status.add("device_timestamp_us", health.device_timestamp_us);
        status.add("timestamp_repeats", health.repeated);
        status.add("timestamp_backwards", health.backwards);
        status.add("timestamp_zero", health.zero);
        status.add("receive_interval_seconds", health.receive_interval_seconds);
        status.add("device_interval_seconds", health.device_interval_seconds);
        status.add("relative_delay_change_seconds", health.relative_delay_change_seconds);
        status.add("max_positive_delay_change_seconds", health.max_positive_delay_change_seconds);
        status.add("timestamp_source", "ros_receive_time");
        status.add("sensor_clock_synchronized", false);
      });
  }
  for (size_t i = 0; i < m_number_of_adapters; ++i) {
    if (m_adapters[i].hw_type != "eth") {continue;}
    diagnostics_->add("UDP adapter " + std::to_string(i),
      [this, i, previous_inode = std::string{}, previous_drops = uint64_t{}]
      (diagnostic_updater::DiagnosticStatusWrapper & status) mutable {
        using Status = diagnostic_msgs::msg::DiagnosticStatus;
        const auto socket = udp_socket_health(static_cast<uint16_t>(m_adapters[i].port));
        status.hardware_id = "udp@" +
        (m_adapters[i].hw_ip_address.empty() ? m_adapters[i].hw_iface_name :
        m_adapters[i].hw_ip_address) + ":" + std::to_string(m_adapters[i].port);
        status.add("local_port", m_adapters[i].port);
        status.add("kernel_counters_available", socket.available);
        status.add("sdk_queue_depth_available", false);
        if (!socket.available) {
          status.summary(Status::WARN, "IPv4 UDP socket counters unavailable or ambiguous");
          return;
        }
        const auto delta = socket.inode == previous_inode && socket.drops >= previous_drops ?
          socket.drops - previous_drops : socket.drops;
        previous_inode = socket.inode;
        previous_drops = socket.drops;
        status.summary(delta ? Status::WARN : Status::OK,
          delta ? "Kernel dropped UDP datagrams" : "No new kernel UDP drops; not a liveness check");
        status.add("kernel_drops_total", socket.drops);
        status.add("kernel_drops_since_last_check", delta);
        status.add("kernel_receive_memory_bytes", socket.receive_memory_bytes);
        status.add("socket_inode", socket.inode);
      });
  }
}

void SmartmicroRadarNode::initialize_services()
{
  // Getting the communication services
  m_services = CommunicationServicesIface::Get();
  if (!m_services->Init()) {
    throw std::runtime_error("Communication Service initialization failed");
  }

  // Getting the data stream service
  data_umrra4_v1_0_1 = com::master::umrra4_automotive_v1_0_1::DataStreamServiceIface::Get();
  data_umrra4_v1_2_1 = com::master::umrra4_automotive_v1_2_1::DataStreamServiceIface::Get();
  data_umrra4_v1_4_0 = com::master::umrra4_automotive_v1_4_0::DataStreamServiceIface::Get();
  data_umrr11 = com::master::umrr11_t132_automotive_v1_1_2::DataStreamServiceIface::Get();
  data_umrr96 = com::master::umrr96_t153_automotive_v1_2_2::DataStreamServiceIface::Get();
  data_umrr9f_v1_1_1 = com::master::umrr9f_t169_automotive_v1_1_1::DataStreamServiceIface::Get();
  data_umrr9f_v2_0_0 = com::master::umrr9f_t169_automotive_v2_0_0::DataStreamServiceIface::Get();
  data_umrr9f_v2_1_1 = com::master::umrr9f_t169_automotive_v2_1_1::DataStreamServiceIface::Get();
  data_umrr9f_v2_2_1 = com::master::umrr9f_t169_automotive_v2_2_1::DataStreamServiceIface::Get();
  data_umrr9f_v2_4_1 = com::master::umrr9f_t169_automotive_v2_4_1::DataStreamServiceIface::Get();
  data_umrr9f_v3_0_0 = com::master::umrr9f_t169_automotive_v3_0_0::DataStreamServiceIface::Get();
  data_umrr9f_v3_2_0 = com::master::umrr9f_t169_automotive_v3_2_0::DataStreamServiceIface::Get();
  data_umrr9d_v1_0_3 = com::master::umrr9d_t152_automotive_v1_0_3::DataStreamServiceIface::Get();
  data_umrr9d_v1_2_2 = com::master::umrr9d_t152_automotive_v1_2_2::DataStreamServiceIface::Get();
  data_umrr9d_v1_4_1 = com::master::umrr9d_t152_automotive_v1_4_1::DataStreamServiceIface::Get();
  data_umrr9d_v1_5_0 = com::master::umrr9d_t152_automotive_v1_5_0::DataStreamServiceIface::Get();
  data_umrr9d_v1_7_0 = com::master::umrr9d_t152_automotive_v1_7_0::DataStreamServiceIface::Get();
  data_umrr9f_mse_v1_0_0 = com::master::umrr9f_t169_mse_v1_0_0::DataStreamServiceIface::Get();
  data_umrr9f_mse_v1_1_0 = com::master::umrr9f_t169_mse_v1_1_0::DataStreamServiceIface::Get();
  data_umrr9f_mse_v1_3_0 = com::master::umrr9f_t169_mse_v1_3_0::DataStreamServiceIface::Get();
  data_umrr9f_mse_v2_0_0 = com::master::umrr9f_t169_mse_v2_0_0::DataStreamServiceIface::Get();
  data_umrra4_mse_v1_0_0 = com::master::umrra4_mse_v1_0_0::DataStreamServiceIface::Get();
  data_umrra4_mse_v2_1_0 = com::master::umrra4_mse_v2_1_0::DataStreamServiceIface::Get();
  data_umrra4_mse_v3_0_0 = com::master::umrra4_mse_v3_0_0::DataStreamServiceIface::Get();
  data_umrra4_v1_6_0 = com::master::umrra4_automotive_v1_6_0::DataStreamServiceIface::Get();
  data_umrra1_v1_0_0 = com::master::umrra1_t166_b_automotive_v1_0_0::DataStreamServiceIface::Get();
  data_umrra1_v2_0_0 = com::master::umrra1_t166_b_automotive_v2_0_0::DataStreamServiceIface::Get();
  data_umrra1_v2_0_1 = com::master::umrra1_t166_b_automotive_v2_0_1::DataStreamServiceIface::Get();
  data_umrra1_v3_0_0 = com::master::umrra1_t166_b_automotive_v3_0_0::DataStreamServiceIface::Get();

  RCLCPP_INFO(this->get_logger(), "Data stream services have been received!");

  // create a ros2 service to change the radar parameters
  mode_srv_ = create_service<umrr_ros2_msgs::srv::SetMode>(
    "smart_radar/set_radar_mode",
    std::bind(
      &SmartmicroRadarNode::set_radar_mode, this, std::placeholders::_1, std::placeholders::_2));

  // create a ros2 service to change the IP address
  ip_addr_srv_ = create_service<umrr_ros2_msgs::srv::SetIp>(
    "smart_radar/set_ip_address",
    std::bind(
      &SmartmicroRadarNode::ip_address, this, std::placeholders::_1, std::placeholders::_2));

  // create a ros2 service to send command to radar
  command_srv_ = create_service<umrr_ros2_msgs::srv::SendCommand>(
    "smart_radar/send_command",
    std::bind(
      &SmartmicroRadarNode::radar_command, this, std::placeholders::_1, std::placeholders::_2));

  // create a ros2 service to perform firmware download
  download_srv_ = create_service<umrr_ros2_msgs::srv::FirmwareDownload>(
    "smart_radar/firmware_download",
    std::bind(
      &SmartmicroRadarNode::firmware_download, this, std::placeholders::_1, std::placeholders::_2));

  // create a ros2 service to read the radar status
  status_srv_ = create_service<umrr_ros2_msgs::srv::GetStatus>(
    "smart_radar/get_radar_status",
    std::bind(
      &SmartmicroRadarNode::get_radar_status, this, std::placeholders::_1, std::placeholders::_2));

  // create a ros2 service to read the radar modes
  read_mode_srv_ = create_service<umrr_ros2_msgs::srv::GetMode>(
    "smart_radar/get_radar_mode",
    std::bind(
      &SmartmicroRadarNode::get_radar_mode, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Radar services are ready.");
}

void SmartmicroRadarNode::setup_publishers()
{
  for (size_t i = 0; i < m_number_of_sensors; ++i) {
    const auto & sensor = m_sensors[i];
    // Already validated with the parameters; repeated so this path never
    // registers an SDK callback without its publishers.
    validate_sensor_config(
      "sensors.sensor_" + std::to_string(i), sensor.link_type, sensor.model, sensor.pub_type);
    if (sensor.link_type == kEthLinkType) {
      port_publishers(sensor, i);
    } else {
      can_publishers(sensor, i);
    }
  }
}

void SmartmicroRadarNode::port_publishers(const detail::SensorConfig & sensor, size_t sensor_idx)
{
  std::string_view pub_type{m_sensors[sensor_idx].pub_type};

  try {
    if (pub_type == kMsePubType) {
      m_publishers_obj[sensor_idx] = create_publisher<sensor_msgs::msg::PointCloud2>(
        "smart_radar/port_objects_" + std::to_string(sensor_idx), sensor.history_size);
      m_publishers_port_obj_header[sensor_idx] =
        create_publisher<umrr_ros2_msgs::msg::PortObjectHeader>(
        "smart_radar/port_objectheader_" + std::to_string(sensor_idx), sensor.history_size);
      m_publishers[sensor_idx] = create_publisher<sensor_msgs::msg::PointCloud2>(
        "smart_radar/port_targets_" + std::to_string(sensor_idx), sensor.history_size);
      m_publishers_port_target_header[sensor_idx] =
        create_publisher<umrr_ros2_msgs::msg::PortTargetHeader>(
        "smart_radar/port_targetheader_" + std::to_string(sensor_idx), sensor.history_size);
      if (supports_fault_reports(sensor.model)) {
        m_publishers_fault_report_msg[sensor_idx] =
          create_publisher<umrr_ros2_msgs::msg::PortFaultReportsMsg>(
          "smart_radar/port_faultreport_" + std::to_string(sensor_idx), sensor.history_size);
      }

    } else if (pub_type == kTargetPubType) {
      m_publishers[sensor_idx] = create_publisher<sensor_msgs::msg::PointCloud2>(
        "smart_radar/port_targets_" + std::to_string(sensor_idx), sensor.history_size);
      m_publishers_port_target_header[sensor_idx] =
        create_publisher<umrr_ros2_msgs::msg::PortTargetHeader>(
        "smart_radar/port_targetheader_" + std::to_string(sensor_idx), sensor.history_size);
      if (supports_fault_reports(sensor.model)) {
        m_publishers_fault_report_msg[sensor_idx] =
          create_publisher<umrr_ros2_msgs::msg::PortFaultReportsMsg>(
          "smart_radar/port_faultreport_" + std::to_string(sensor_idx), sensor.history_size);
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown publish type: %s", sensor.pub_type.c_str());
      throw std::invalid_argument("Unknown publish type");
    }

    RCLCPP_INFO(get_logger(), "Successfully created PORT publishers for sensor %zu", sensor_idx);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "Failed to create publishers for sensor %zu: %s", sensor_idx, e.what());
    throw;
  }
  if (sensor.model == "umrra4_mse_v3_0_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrra4_mse_v3_0_0->RegisterComObjectListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::objectlist_callback_umrra4_mse_v3_0_0, this, sensor_idx,
          std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register objectlist callback for sensor umrra4_mse_v3_0_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrra4_mse_v3_0_0->RegisterComTargetListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::targetlist_callback_umrra4_mse_v3_0_0, this, sensor_idx,
          std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register targetlist callback for sensor umrra4_mse_v3_0_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrra4_mse_v3_0_0->RegisterFaultReportsReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::faultreport_callback_umrra4_mse_v3_0_0, this, sensor_idx,
          std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register faultreport callback for sensor umrra4_mse_v3_0_0");
    }
  }
  if (sensor.model == "umrra4_mse_v2_1_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrra4_mse_v2_1_0->RegisterComObjectListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::objectlist_callback_umrra4_mse_v2_1_0, this, sensor_idx,
          std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register objectlist callback for sensor umrra4_mse_v2_1_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrra4_mse_v2_1_0->RegisterComTargetListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::targetlist_callback_umrra4_mse_v2_1_0, this, sensor_idx,
          std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register targetlist callback for sensor umrra4_mse_v2_1_0");
    }
  }
  if (sensor.model == "umrra4_mse_v1_0_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrra4_mse_v1_0_0->RegisterComObjectListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::objectlist_callback_umrra4_mse_v1_0_0, this, sensor_idx,
          std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register objectlist callback for sensor umrra4_mse_v1_0_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrra4_mse_v1_0_0->RegisterComTargetListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::targetlist_callback_umrra4_mse_v1_0_0, this, sensor_idx,
          std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register targetlist callback for sensor umrra4_mse_v1_0_0");
    }
  }
  if (sensor.model == "umrr9f_mse_v2_0_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v2_0_0->RegisterComObjectListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::objectlist_callback_umrr9f_mse_v2_0_0, this, sensor_idx,
          std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register objectlist callback for sensor umrr9f_mse_v2_0_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v2_0_0->RegisterComTargetListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::targetlist_callback_umrr9f_mse_v2_0_0, this, sensor_idx,
          std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register targetlist callback for sensor umrr9f_mse_v2_0_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v2_0_0->RegisterFaultReportsReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::faultreport_callback_umrr9f_mse_v2_0_0, this, sensor_idx,
          std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register faultreport callback for sensor umrr9f_mse_v2_0_0");
    }
  }
  if (sensor.model == "umrr9f_mse_v1_3_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_3_0->RegisterComObjectListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::objectlist_callback_umrr9f_mse_v1_3_0, this, sensor_idx,
          std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register objectlist callback for sensor umrr9f_mse_v1_3_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_3_0->RegisterComTargetListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::targetlist_callback_umrr9f_mse_v1_3_0, this, sensor_idx,
          std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register targetlist callback for sensor umrr9f_mse_v1_3_0");
    }
  }
  if (sensor.model == "umrr9f_mse_v1_1_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_1_0->RegisterComObjectListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::objectlist_callback_umrr9f_mse_v1_1_0, this, sensor_idx,
          std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register objectlist callback for sensor umrr9f_mse_v1_1_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_1_0->RegisterComTargetListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::targetlist_callback_umrr9f_mse_v1_1_0, this, sensor_idx,
          std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register targetlist callback for sensor umrr9f_mse_v1_1_0");
    }
  }
  if (sensor.model == "umrr9f_mse_v1_0_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_0_0->RegisterComObjectListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::objectlist_callback_umrr9f_mse_v1_0_0, this, sensor_idx,
          std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register objectlist callback for sensor umrr9f_mse_v1_0_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_0_0->RegisterComTargetListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::targetlist_callback_umrr9f_mse_v1_0_0, this, sensor_idx,
          std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register targetlist callback for sensor umrr9f_mse_v1_0_0");
    }
  }
  if (
    sensor.model == "umrr96_v1_2_2" &&
    com::types::ERROR_CODE_OK !=
    data_umrr96->RegisterComTargetListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::targetlist_callback_umrr96, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrr96_v1_2_2");
  }
  if (
    sensor.model == "umrr11_v1_1_2" &&
    com::types::ERROR_CODE_OK !=
    data_umrr11->RegisterComTargetListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::targetlist_callback_umrr11, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrr11_v1_1_2");
  }
  if (
    sensor.model == "umrr9f_v1_1_1" &&
    com::types::ERROR_CODE_OK !=
    data_umrr9f_v1_1_1->RegisterComTargetListPortReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::targetlist_callback_umrr9f_v1_1_1, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrr9f_v1_1_1");
  }
  if (
    sensor.model == "umrr9f_v2_0_0" &&
    com::types::ERROR_CODE_OK !=
    data_umrr9f_v2_0_0->RegisterComTargetListPortReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::targetlist_callback_umrr9f_v2_0_0, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrr9f_v2_0_0");
  }
  if (
    sensor.model == "umrr9f_v2_1_1" &&
    com::types::ERROR_CODE_OK !=
    data_umrr9f_v2_1_1->RegisterComTargetListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::targetlist_callback_umrr9f_v2_1_1, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrr9f_v2_1_1");
  }
  if (
    sensor.model == "umrr9f_v2_2_1" &&
    com::types::ERROR_CODE_OK !=
    data_umrr9f_v2_2_1->RegisterComTargetListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::targetlist_callback_umrr9f_v2_2_1, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrr9f_v2_2_1");
  }
  if (
    sensor.model == "umrr9f_v2_4_1" &&
    com::types::ERROR_CODE_OK !=
    data_umrr9f_v2_4_1->RegisterComTargetListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::targetlist_callback_umrr9f_v2_4_1, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrr9f_v2_4_1");
  }
  if (
    sensor.model == "umrr9f_v3_0_0" &&
    com::types::ERROR_CODE_OK !=
    data_umrr9f_v3_0_0->RegisterComTargetListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::targetlist_callback_umrr9f_v3_0_0, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrr9f_v3_0_0");
  }

  if (sensor.model == "umrr9f_v3_2_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_v3_2_0->RegisterComTargetListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::targetlist_callback_umrr9f_v3_2_0, this, sensor_idx,
          std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register targetlist callback for sensor umrr9f_v3_2_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_v3_2_0->RegisterFaultReportsReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::faultreport_callback_umrr9f_v3_2_0, this, sensor_idx,
          std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register faultreport callback for sensor umrr9f_v3_2_0");
    }
  }

  if (
    sensor.model == "umrr9d_v1_0_3" &&
    com::types::ERROR_CODE_OK !=
    data_umrr9d_v1_0_3->RegisterComTargetListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::targetlist_callback_umrr9d_v1_0_3, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrr9d_v1_0_3");
  }
  if (
    sensor.model == "umrr9d_v1_2_2" &&
    com::types::ERROR_CODE_OK !=
    data_umrr9d_v1_2_2->RegisterComTargetListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::targetlist_callback_umrr9d_v1_2_2, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrr9d_v1_2_2");
  }
  if (
    sensor.model == "umrr9d_v1_4_1" &&
    com::types::ERROR_CODE_OK !=
    data_umrr9d_v1_4_1->RegisterComTargetListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::targetlist_callback_umrr9d_v1_4_1, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrr9d_v1_4_1");
  }
  if (
    sensor.model == "umrr9d_v1_5_0" &&
    com::types::ERROR_CODE_OK !=
    data_umrr9d_v1_5_0->RegisterComTargetListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::targetlist_callback_umrr9d_v1_5_0, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrr9d_v1_5_0");
  }
  if (sensor.model == "umrr9d_v1_7_0") {
    RCLCPP_INFO(
      this->get_logger(),
      "Registering umrr9d_v1_7_0 callbacks for sensor_idx=%zu client_id=%u",
      sensor_idx, sensor.id);

    const auto target_cb_ret = data_umrr9d_v1_7_0->RegisterComTargetListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::targetlist_callback_umrr9d_v1_7_0, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2)));

    const auto fault_cb_ret = data_umrr9d_v1_7_0->RegisterFaultReportsReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::faultreport_callback_umrr9d_v1_7_0, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2)));

    if (
      com::types::ERROR_CODE_OK !=
      target_cb_ret)
    {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register targetlist callback for sensor umrr9d_v1_7_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      fault_cb_ret)
    {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register faultreport callback for sensor umrr9d_v1_7_0");
    }
  }
  if (
    sensor.model == "umrra4_v1_0_1" &&
    com::types::ERROR_CODE_OK !=
    data_umrra4_v1_0_1->RegisterComTargetListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::targetlist_callback_umrra4_v1_0_1, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrra4_v1_0_1");
  }
  if (
    sensor.model == "umrra4_v1_2_1" &&
    com::types::ERROR_CODE_OK !=
    data_umrra4_v1_2_1->RegisterComTargetListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::targetlist_callback_umrra4_v1_2_1, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrra4_v1_2_1");
  }
  if (
    sensor.model == "umrra4_v1_4_0" &&
    com::types::ERROR_CODE_OK !=
    data_umrra4_v1_4_0->RegisterComTargetListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::targetlist_callback_umrra4_v1_4_0, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrra4_v1_4_0");
  }

  if (sensor.model == "umrra4_v1_6_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrra4_v1_6_0->RegisterComTargetListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::targetlist_callback_umrra4_v1_6_0, this, sensor_idx,
          std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register targetlist callback for sensor umrra4_v1_6_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrra4_v1_6_0->RegisterFaultReportsReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::faultreport_callback_umrra4_v1_6_0, this, sensor_idx,
          std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register faultreport callback for sensor umrra4_v1_6_0");
    }
  }

  if (
    sensor.model == "umrra1_v1_0_0" &&
    com::types::ERROR_CODE_OK !=
    data_umrra1_v1_0_0->RegisterComTargetListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::targetlist_callback_umrra1_v1_0_0, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrra1_v1_0_0");
  }
  if (
    sensor.model == "umrra1_v2_0_0" &&
    com::types::ERROR_CODE_OK !=
    data_umrra1_v2_0_0->RegisterComTargetListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::targetlist_callback_umrra1_v2_0_0, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrra1_v2_0_0");
  }
  if (
    sensor.model == "umrra1_v2_0_1" &&
    com::types::ERROR_CODE_OK !=
    data_umrra1_v2_0_1->RegisterComTargetListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::targetlist_callback_umrra1_v2_0_1, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrra1_v2_0_1");
  }
  if (
    sensor.model == "umrra1_v3_0_0" &&
    com::types::ERROR_CODE_OK !=
    data_umrra1_v3_0_0->RegisterComTargetListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::targetlist_callback_umrra1_v3_0_0, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrra1_v3_0_0");
  }
}

void SmartmicroRadarNode::can_publishers(const detail::SensorConfig & sensor, size_t sensor_idx)
{
  std::string_view pub_type{m_sensors[sensor_idx].pub_type};

  try {
    if (pub_type == kMsePubType) {
      m_publishers_obj[sensor_idx] = create_publisher<sensor_msgs::msg::PointCloud2>(
        "smart_radar/can_objects_" + std::to_string(sensor_idx), sensor.history_size);
      m_publishers_can_obj_header[sensor_idx] =
        create_publisher<umrr_ros2_msgs::msg::CanObjectHeader>(
        "smart_radar/can_objectheader_" + std::to_string(sensor_idx), sensor.history_size);

      m_publishers[sensor_idx] = create_publisher<sensor_msgs::msg::PointCloud2>(
        "smart_radar/can_targets_" + std::to_string(sensor_idx), sensor.history_size);
      m_publishers_can_target_header[sensor_idx] =
        create_publisher<umrr_ros2_msgs::msg::CanTargetHeader>(
        "smart_radar/can_targetheader_" + std::to_string(sensor_idx), sensor.history_size);
    } else if (pub_type == kTargetPubType) {
      m_publishers[sensor_idx] = create_publisher<sensor_msgs::msg::PointCloud2>(
        "smart_radar/can_targets_" + std::to_string(sensor_idx), sensor.history_size);
      m_publishers_can_target_header[sensor_idx] =
        create_publisher<umrr_ros2_msgs::msg::CanTargetHeader>(
        "smart_radar/can_targetheader_" + std::to_string(sensor_idx), sensor.history_size);
    } else {
      throw std::invalid_argument("Unknown publish type: " + sensor.pub_type);
    }

    RCLCPP_INFO(get_logger(), "Successfully created CAN publishers for sensor %zu", sensor_idx);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "Failed to create publishers for sensor %zu: %s", sensor_idx, e.what());
    throw;
  }

  if (sensor.model == "umrra4_can_mse_v2_1_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrra4_mse_v2_1_0->RegisterComObjectBaseListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::CAN_objectlist_callback_umrra4_mse_v2_1_0, this,
          sensor_idx, std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to register objectlist callback for sensor umrra4_can_mse_v2_1_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrra4_mse_v2_1_0->RegisterComTargetBaseListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::CAN_targetlist_callback_umrra4_mse_v2_1_0, this,
          sensor_idx, std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to register targetlist callback for sensor umrra4_can_mse_v2_1_0");
    }
  }
  if (sensor.model == "umrra4_can_mse_v1_0_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrra4_mse_v1_0_0->RegisterComObjectBaseListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::CAN_objectlist_callback_umrra4_mse_v1_0_0, this,
          sensor_idx, std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to register objectlist callback for sensor umrra4_can_mse_v1_0_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrra4_mse_v1_0_0->RegisterComTargetBaseListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::CAN_targetlist_callback_umrra4_mse_v1_0_0, this,
          sensor_idx, std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to register targetlist callback for sensor umrra4_can_mse_v1_0_0");
    }
  }
  if (sensor.model == "umrr9f_can_mse_v1_3_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_3_0->RegisterComObjectBaseListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::CAN_objectlist_callback_umrr9f_mse_v1_3_0, this,
          sensor_idx, std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to register objectlist callback for sensor umrr9f_can_mse_v1_3_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_3_0->RegisterComTargetBaseListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_mse_v1_3_0, this,
          sensor_idx, std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to register targetlist callback for sensor umrr9f_can_mse_v1_3_0");
    }
  }
  if (sensor.model == "umrr9f_can_mse_v1_1_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_1_0->RegisterComObjectBaseListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::CAN_objectlist_callback_umrr9f_mse_v1_1_0, this,
          sensor_idx, std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to register objectlist callback for sensor umrr9f_can_mse_v1_1_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_1_0->RegisterComTargetBaseListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_mse_v1_1_0, this,
          sensor_idx, std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to register targetlist callback for sensor umrr9f_can_mse_v1_1_0");
    }
  }
  if (sensor.model == "umrr9f_can_mse_v1_0_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_0_0->RegisterComObjectBaseListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::CAN_objectlist_callback_umrr9f_mse_v1_0_0, this,
          sensor_idx, std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to register objectlist callback for sensor umrr9f_can_mse_v1_0_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_0_0->RegisterComTargetBaseListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_mse_v1_0_0, this,
          sensor_idx, std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to register targetlist callback for sensor umrr9f_can_mse_v1_0_0");
    }
  }
  if (sensor.model == "umrra4_can_mse_v3_0_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrra4_mse_v3_0_0->RegisterComObjectBaseListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::CAN_objectlist_callback_umrra4_mse_v3_0_0, this,
          sensor_idx, std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to register objectlist callback for sensor umrra4_can_mse_v3_0_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrra4_mse_v3_0_0->RegisterComTargetBaseListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::CAN_targetlist_callback_umrra4_mse_v3_0_0, this,
          sensor_idx, std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to register targetlist callback for sensor umrra4_can_mse_v3_0_0");
    }
  }
  if (sensor.model == "umrr9f_can_mse_v2_0_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v2_0_0->RegisterComObjectBaseListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::CAN_objectlist_callback_umrr9f_mse_v2_0_0, this,
          sensor_idx, std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to register objectlist callback for sensor umrr9f_can_mse_v2_0_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v2_0_0->RegisterComTargetBaseListReceiveCallback(
        sensor.id, callback_gate_.wrap(std::bind(
          &SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_mse_v2_0_0, this,
          sensor_idx, std::placeholders::_1, std::placeholders::_2))))
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to register targetlist callback for sensor umrr9f_can_mse_v2_0_0");
    }
  }
  if (
    sensor.model == "umrr96_can_v1_2_2" &&
    com::types::ERROR_CODE_OK !=
    data_umrr96->RegisterComTargetBaseListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::CAN_targetlist_callback_umrr96, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrr96_can_v1_2_2");
  }
  if (
    sensor.model == "umrr11_can_v1_1_2" &&
    com::types::ERROR_CODE_OK !=
    data_umrr11->RegisterComTargetBaseListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::CAN_targetlist_callback_umrr11, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrr11_can_v1_1_2");
  }
  if (
    sensor.model == "umrr9f_can_v2_1_1" &&
    com::types::ERROR_CODE_OK !=
    data_umrr9f_v2_1_1->RegisterComTargetBaseListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_v2_1_1, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrr9f_can_v2_1_1");
  }
  if (
    sensor.model == "umrr9f_can_v2_2_1" &&
    com::types::ERROR_CODE_OK !=
    data_umrr9f_v2_2_1->RegisterComTargetBaseListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_v2_2_1, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrr9f_can_v2_2_1");
  }
  if (
    sensor.model == "umrr9f_can_v2_4_1" &&
    com::types::ERROR_CODE_OK !=
    data_umrr9f_v2_4_1->RegisterComTargetBaseListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_v2_4_1, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrr9f_can_v2_4_1");
  }
  if (
    sensor.model == "umrr9f_can_v3_0_0" &&
    com::types::ERROR_CODE_OK !=
    data_umrr9f_v3_0_0->RegisterComTargetBaseListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_v3_0_0, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrr9f_can_v3_0_0");
  }
  if (
    sensor.model == "umrr9f_can_v3_2_0" &&
    com::types::ERROR_CODE_OK !=
    data_umrr9f_v3_2_0->RegisterComTargetBaseListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_v3_2_0, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrr9f_can_v3_2_0");
  }
  if (
    sensor.model == "umrr9d_can_v1_0_3" &&
    com::types::ERROR_CODE_OK !=
    data_umrr9d_v1_0_3->RegisterComTargetBaseListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::CAN_targetlist_callback_umrr9d_v1_0_3, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrr9d_can_v1_0_3");
  }
  if (
    sensor.model == "umrr9d_can_v1_2_2" &&
    com::types::ERROR_CODE_OK !=
    data_umrr9d_v1_2_2->RegisterComTargetBaseListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::CAN_targetlist_callback_umrr9d_v1_2_2, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrr9d_can_v1_2_2");
  }
  if (
    sensor.model == "umrr9d_can_v1_4_1" &&
    com::types::ERROR_CODE_OK !=
    data_umrr9d_v1_4_1->RegisterComTargetBaseListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::CAN_targetlist_callback_umrr9d_v1_4_1, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrr9d_can_v1_4_1");
  }
  if (
    sensor.model == "umrr9d_can_v1_5_0" &&
    com::types::ERROR_CODE_OK !=
    data_umrr9d_v1_5_0->RegisterComTargetBaseListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::CAN_targetlist_callback_umrr9d_v1_5_0, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrr9d_can_v1_5_0");
  }
  if (
    sensor.model == "umrr9d_can_v1_7_0" &&
    com::types::ERROR_CODE_OK !=
    data_umrr9d_v1_7_0->RegisterComTargetBaseListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::CAN_targetlist_callback_umrr9d_v1_7_0, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrr9d_can_v1_7_0");
  }
  if (
    sensor.model == "umrra4_can_v1_0_1" &&
    com::types::ERROR_CODE_OK !=
    data_umrra4_v1_0_1->RegisterComTargetBaseListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::CAN_targetlist_callback_umrra4_v1_0_1, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrra4_can_v1_0_1");
  }
  if (
    sensor.model == "umrra4_can_v1_2_1" &&
    com::types::ERROR_CODE_OK !=
    data_umrra4_v1_2_1->RegisterComTargetBaseListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::CAN_targetlist_callback_umrra4_v1_2_1, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrra4_can_v1_2_1");
  }
  if (
    sensor.model == "umrra4_can_v1_4_0" &&
    com::types::ERROR_CODE_OK !=
    data_umrra4_v1_4_0->RegisterComTargetBaseListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::CAN_targetlist_callback_umrra4_v1_4_0, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrra4_can_v1_4_0");
  }
  if (
    sensor.model == "umrra4_can_v1_6_0" &&
    com::types::ERROR_CODE_OK !=
    data_umrra4_v1_6_0->RegisterComTargetBaseListReceiveCallback(
      sensor.id, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::CAN_targetlist_callback_umrra4_v1_6_0, this, sensor_idx,
        std::placeholders::_1, std::placeholders::_2))))
  {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrra4_can_v1_6_0");
  }
}

void SmartmicroRadarNode::firmware_download(
  const std::shared_ptr<umrr_ros2_msgs::srv::FirmwareDownload::Request> request,
  std::shared_ptr<umrr_ros2_msgs::srv::FirmwareDownload::Response> result)
{
  const auto client_id = request->sensor_id;
  std::string update_image = request->file_path;
  if (!is_configured_sensor(client_id)) {
    result->res = "The sensor ID value entered is invalid! ";
    return;
  }

  const auto update_result = update_service->StartSoftwareUpdate(client_id, update_image);
  switch (update_result) {
    case UpdateResult::kSuccess:
      result->res = "Firmware download completed successfully.";
      break;
    case UpdateResult::kBusy:
      result->res = "Firmware download rejected: another update is already in progress.";
      break;
    case UpdateResult::kFileOpenError:
      result->res = "Firmware download failed: could not open update image file.";
      break;
    case UpdateResult::kFileSizeError:
      result->res = "Firmware download failed: could not determine update image size.";
      break;
    case UpdateResult::kServiceUnavailable:
      result->res = "Firmware download failed: update service is unavailable.";
      break;
    case UpdateResult::kStartFailed:
      result->res = "Firmware download failed: could not start software update.";
      break;
    case UpdateResult::kTimeout:
      result->res = "Firmware download failed: timed out and aborted.";
      break;
    case UpdateResult::kStoppedByMaster:
      result->res = "Firmware download stopped by master.";
      break;
    case UpdateResult::kStoppedBySlave:
      result->res = "Firmware download stopped by slave.";
      break;
    case UpdateResult::kBlockRepeatError:
      result->res = "Firmware download failed: block repeat error.";
      break;
    case UpdateResult::kImageInvalid:
      result->res = "Firmware download failed: invalid image.";
      break;
    case UpdateResult::kUnknownError:
    default:
      result->res = "Firmware download failed: unknown error.";
      break;
  }
}

void SmartmicroRadarNode::set_radar_mode(
  const std::shared_ptr<umrr_ros2_msgs::srv::SetMode::Request> request,
  std::shared_ptr<umrr_ros2_msgs::srv::SetMode::Response> result)
{
  const auto client_id = request->sensor_id;
  if (!is_configured_sensor(client_id)) {
    result->res = "Error: Sensor ID is invalid! ";
    return;
  }

  auto section_name = request->section_name;
  if (
    section_name != "auto_interface_0dim" && section_name != "auto_interface_rrm" &&
    section_name != "Parameter")
  {
    result->res =
      "Error: Invalid section name specified! Must be 'auto_interface_0dim', "
      "'auto_interface_rrm', or 'Parameter'.";
    return;
  }

  // Check arrays have same length
  if (
    request->params.size() != request->values.size() ||
    request->params.size() != request->value_types.size())
  {
    result->res = "Error: param, values and value_types arrays must have same length";
    return;
  }

  // Parse every value before allocating an SDK batch, so a rejected request
  // neither sends a partial batch nor leaves one allocated.
  std::vector<ModeValue> parsed_values;
  parsed_values.reserve(request->params.size());
  for (size_t i = 0; i < request->params.size(); i++) {
    try {
      parsed_values.push_back(parse_mode_value(request->values[i], request->value_types[i]));
    } catch (const std::exception & e) {
      result->res = "Error: parameter '" + request->params[i] + "': " + e.what();
      return;
    }
  }

  std::shared_ptr<InstructionServiceIface> inst{m_services->GetInstructionService()};
  if (!inst) {
    result->res = "Error: Failed to get instruction service";
    return;
  }

  std::shared_ptr<InstructionBatch> batch;
  if (!inst->AllocateInstructionBatch(client_id, batch)) {
    result->res = "Error: Failed to allocate instruction! ";
    return;
  }

  for (size_t i = 0; i < request->params.size(); i++) {
    const auto & param = request->params[i];
    const bool request_added = std::visit(
      [&](auto typed_value) {
        return batch->AddRequest(
          std::make_shared<SetParamRequest<decltype(typed_value)>>(section_name, param, typed_value));
      }, parsed_values[i]);

    if (!request_added) {
      result->res = "Error: Failed to add instruction '" + param + "'! ";
      return;
    }
  }

  if (
    com::types::ERROR_CODE_OK !=
    inst->SendInstructionBatch(
      batch, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::mode_response, this, client_id, std::placeholders::_2,
        request->params, section_name))))
  {
    result->res = "Error: Check params are valid for this sensor and values within range!";
    return;
  }
  result->res = "Success: Request sent successfully. Check main terminal for sensor response!";
  RCLCPP_INFO(this->get_logger(), "Service call result: %s", result->res.c_str());
}

void SmartmicroRadarNode::ip_address(
  const std::shared_ptr<umrr_ros2_msgs::srv::SetIp::Request> request,
  std::shared_ptr<umrr_ros2_msgs::srv::SetIp::Response> result)
{
  const auto client_id = request->sensor_id;
  if (!is_configured_sensor(client_id)) {
    result->res_ip = "Sensor ID entered is not listed in the param file! ";
    return;
  }
  std::shared_ptr<InstructionServiceIface> inst{m_services->GetInstructionService()};
  if (!inst) {
    result->res_ip = "Failed to get instruction service";
    return;
  }

  std::shared_ptr<InstructionBatch> batch;
  if (!inst->AllocateInstructionBatch(client_id, batch)) {
    result->res_ip = "Failed to allocate instruction! ";
    return;
  }

  std::shared_ptr<SetParamRequest<uint32_t>> ip_address =
    std::make_shared<SetParamRequest<uint32_t>>(
    "auto_interface_0dim", "ip_source_address", request->value_ip);

  std::shared_ptr<CmdRequest> cmd =
    std::make_shared<CmdRequest>("auto_interface_command", "comp_eeprom_ctrl_save_param_sec", 2010);

  if (!batch->AddRequest(ip_address)) {
    result->res_ip = "Failed to add instruction! ";
    return;
  }
  if (!batch->AddRequest(cmd)) {
    result->res_ip = "Failed to add instruction! ";
    return;
  }
  // send instruction batch to the device
  if (
    com::types::ERROR_CODE_OK !=
    inst->SendInstructionBatch(
      batch, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::sensor_response_ip, this, client_id, std::placeholders::_2))))
  {
    result->res_ip = "Service not conducted";
    return;
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Radar must be restarted and the parameters in the param file "
      "must be updated !!.");
    result->res_ip =
      "Success: IP change executed successfully. Radar must be restarted "
      "and the parameters in the param file must be updated";
  }
}

void SmartmicroRadarNode::radar_command(
  const std::shared_ptr<umrr_ros2_msgs::srv::SendCommand::Request> request,
  std::shared_ptr<umrr_ros2_msgs::srv::SendCommand::Response> result)
{
  const std::string command_name = request->command;
  const auto client_id = request->sensor_id;
  if (!is_configured_sensor(client_id)) {
    result->res = "The sensor ID value entered is invalid! ";
    return;
  }

  auto section_name = request->section_name;
  if (
    section_name != "auto_interface_command" && section_name != "auto_interface_rrm_command" &&
    section_name != "Command")
  {
    result->res =
      "Error: Invalid section name specified! Must be 'auto_interface_command', "
      "'auto_interface_rrm_command', or 'Command'.";
    return;
  }

  std::shared_ptr<InstructionServiceIface> inst{m_services->GetInstructionService()};
  if (!inst) {
    result->res = "Failed to get instruction service";
    return;
  }
  std::shared_ptr<InstructionBatch> batch;

  if (!inst->AllocateInstructionBatch(client_id, batch)) {
    result->res = "Failed to allocate instruction! ";
    return;
  }

  std::shared_ptr<CmdRequest> radar_command =
    std::make_shared<CmdRequest>(section_name, request->command, request->value);

  if (!batch->AddRequest(radar_command)) {
    result->res = "Failed to add instruction! ";
    return;
  }

  if (
    com::types::ERROR_CODE_OK != inst->SendInstructionBatch(
      batch, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::command_response, this, client_id,
        std::placeholders::_2, command_name, section_name))))
  {
    result->res = "Error in sending command to the sensor!";
    return;
  }
  result->res = "Success: Request sent successfully.";
}

void SmartmicroRadarNode::get_radar_status(
  const std::shared_ptr<umrr_ros2_msgs::srv::GetStatus::Request> request,
  std::shared_ptr<umrr_ros2_msgs::srv::GetStatus::Response> result)
{
  const auto client_id = request->sensor_id;
  if (!is_configured_sensor(client_id)) {
    result->res = "Error: Sensor ID is invalid! ";
    return;
  }

  auto section_name = request->section_name;
  if (
    section_name != "auto_interface" && section_name != "auto_interface_rrm" &&
    section_name != "Status")
  {
    result->res =
      "Error: Invalid section name specified! Must be 'auto_interface', 'auto_interface_rrm', or "
      "'Status'.";
    return;
  }

  // Check arrays have same length
  if (request->statuses.size() != request->status_types.size()) {
    result->res = "Error: status and status_types arrays must have same length";
    return;
  }

  std::shared_ptr<InstructionServiceIface> inst{m_services->GetInstructionService()};
  if (!inst) {
    result->res = "Error: Failed to get instruction service";
    return;
  }

  std::shared_ptr<InstructionBatch> batch;
  if (!inst->AllocateInstructionBatch(client_id, batch)) {
    result->res = "Error: Failed to allocate instruction! ";
    return;
  }

  for (size_t i = 0; i < request->statuses.size(); i++) {
    const auto & status = request->statuses[i];
    const auto & status_type = request->status_types[i];
    bool request_added = false;

    switch (status_type) {
      case 0: {
          auto radar_status_u32 =
            std::make_shared<GetStatusRequest<uint32_t>>(section_name, status);
          request_added = batch->AddRequest(radar_status_u32);
          break;
        }
      case 1: {
          auto radar_status_u16 =
            std::make_shared<GetStatusRequest<uint16_t>>(section_name, status);
          request_added = batch->AddRequest(radar_status_u16);
          break;
        }
      case 2: {
          auto radar_status_u8 = std::make_shared<GetStatusRequest<uint8_t>>(section_name, status);
          request_added = batch->AddRequest(radar_status_u8);
          break;
        }
      case 3: {
          auto radar_status_i32 = std::make_shared<GetStatusRequest<int32_t>>(section_name, status);
          request_added = batch->AddRequest(radar_status_i32);
          break;
        }
      default:
        result->res =
          "Error: Invalid value_type specified. Must be 0 (u32), 1 (u16), 2 (u8), or 3 (i32)";
        return;
    }

    if (!request_added) {
      result->res = "Error: Failed to add instruction '" + status + "' ! ";
      return;
    }
  }

  if (
    com::types::ERROR_CODE_OK !=
    inst->SendInstructionBatch(
      batch, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::status_response, this, client_id, std::placeholders::_2,
        request->statuses, section_name))))
  {
    result->res = "Error: Check status are valid for this sensor!";
    return;
  }
  result->res = "Success: Request sent successfully. Check main terminal for sensor response!";
}

void SmartmicroRadarNode::get_radar_mode(
  const std::shared_ptr<umrr_ros2_msgs::srv::GetMode::Request> request,
  std::shared_ptr<umrr_ros2_msgs::srv::GetMode::Response> result)
{
  const auto client_id = request->sensor_id;
  if (!is_configured_sensor(client_id)) {
    result->res = "Error: Sensor ID is invalid! ";
    return;
  }

  auto section_name = request->section_name;
  if (
    section_name != "auto_interface_0dim" && section_name != "auto_interface_rrm" &&
    section_name != "Parameter")
  {
    result->res =
      "Error: Invalid section name specified! Must be 'auto_interface_0dim', 'auto_interface_rrm', "
      "or 'Parameter'.";
    return;
  }

  // Check arrays have same length
  if (request->params.size() != request->param_types.size()) {
    result->res = "Error: param and value_types arrays must have same length";
    return;
  }

  std::shared_ptr<InstructionServiceIface> inst{m_services->GetInstructionService()};
  if (!inst) {
    result->res = "Error: Failed to get instruction service";
    return;
  }

  std::shared_ptr<InstructionBatch> batch;
  if (!inst->AllocateInstructionBatch(client_id, batch)) {
    result->res = "Error: Failed to allocate instruction! ";
    return;
  }

  for (size_t i = 0; i < request->params.size(); i++) {
    const auto & param = request->params[i];
    const auto & param_type = request->param_types[i];
    bool request_added = false;

    switch (param_type) {
      case 0: {
          auto radar_param_float = std::make_shared<GetParamRequest<float>>(section_name, param);
          request_added = batch->AddRequest(radar_param_float);
          break;
        }
      case 1: {
          auto radar_param_u32 = std::make_shared<GetParamRequest<uint32_t>>(section_name, param);
          request_added = batch->AddRequest(radar_param_u32);
          break;
        }
      case 2: {
          auto radar_param_u16 = std::make_shared<GetParamRequest<uint16_t>>(section_name, param);
          request_added = batch->AddRequest(radar_param_u16);
          break;
        }
      case 3: {
          auto radar_param_u8 = std::make_shared<GetParamRequest<uint8_t>>(section_name, param);
          request_added = batch->AddRequest(radar_param_u8);
          break;
        }
      default:
        result->res =
          "Error: Invalid param_type specified. Must be 0 (f32), 1 (u32), 2 (u16) or 3 (u8)";
        return;
    }

    if (!request_added) {
      result->res = "Error: Failed to add instruction '" + param + "' Check param types match! ";
      return;
    }
  }

  if (
    com::types::ERROR_CODE_OK !=
    inst->SendInstructionBatch(
      batch, callback_gate_.wrap(std::bind(
        &SmartmicroRadarNode::param_response, this, client_id, std::placeholders::_2,
        request->params, section_name))))
  {
    result->res = "Error: Check params are valid for this sensor!";
    return;
  }
  result->res = "Success: Request sent successfully. Check main terminal for sensor response!";
}

void SmartmicroRadarNode::mode_response(
  const com::types::ClientId client_id,
  const std::shared_ptr<com::master::ResponseBatch> & response,
  const std::vector<std::string> & instruction_names, const std::string & section_name)
{
  for (const auto & instruction_name : instruction_names) {
    std::vector<std::shared_ptr<Response<uint8_t>>> resp_u8;
    std::vector<std::shared_ptr<Response<uint16_t>>> resp_u16;
    std::vector<std::shared_ptr<Response<uint32_t>>> resp_u32;
    std::vector<std::shared_ptr<Response<float>>> resp_f;
    bool response_found = false;

    if (response->GetResponse<uint8_t>(section_name, instruction_name.c_str(), resp_u8)) {
      response_found = true;
      for (auto & resp : resp_u8) {
        RCLCPP_INFO(
          this->get_logger(),
          "Response details:\n"
          "   Instruction: %s\n"
          "   Response type: %u\n"
          "   Value: %u\n",
          resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
      }
    }

    if (response->GetResponse<uint32_t>(section_name, instruction_name.c_str(), resp_u32)) {
      response_found = true;
      for (auto & resp : resp_u32) {
        RCLCPP_INFO(
          this->get_logger(),
          "Response details:\n"
          "   Instruction: %s\n"
          "   Response type: %u\n"
          "   Value: %u\n",
          resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
      }
    }

    if (response->GetResponse<uint16_t>(section_name, instruction_name.c_str(), resp_u16)) {
      response_found = true;
      for (auto & resp : resp_u16) {
        RCLCPP_INFO(
          this->get_logger(),
          "Response details:\n"
          "   Instruction: %s\n"
          "   Response type: %u\n"
          "   Value: %u\n",
          resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
      }
    }

    if (response->GetResponse<float>(section_name, instruction_name.c_str(), resp_f)) {
      response_found = true;
      for (auto & resp : resp_f) {
        RCLCPP_INFO(
          this->get_logger(),
          "Response details:\n"
          "   Instruction: %s\n"
          "   Response type: %u\n"
          "   Value: %f\n",
          resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
      }
    }

    if (!response_found) {
      RCLCPP_WARN(this->get_logger(), "No response received!");
    }
  }
}

void SmartmicroRadarNode::sensor_response_ip(
  const com::types::ClientId client_id,
  const std::shared_ptr<com::master::ResponseBatch> & response)
{
  std::vector<std::shared_ptr<Response<uint32_t>>> resp_ip;
  if (response->GetResponse<uint32_t>("auto_interface_0dim", "ip_source_address", resp_ip)) {
    for (auto & resp : resp_ip) {
      RCLCPP_INFO(
        this->get_logger(),
        "Response details:\n"
        "   Instruction: %s\n"
        "   Response type: %u\n"
        "   Value: %u\n",
        resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
    }
  }
}

void SmartmicroRadarNode::command_response(
  const com::types::ClientId client_id,
  const std::shared_ptr<com::master::ResponseBatch> & response, const std::string command_name,
  const std::string & section_name)
{
  std::vector<std::shared_ptr<Response<uint32_t>>> command_resp;
  if (response->GetResponse<uint32_t>(section_name, command_name.c_str(), command_resp)) {
    for (auto & resp : command_resp) {
      RCLCPP_INFO(
        this->get_logger(),
        "Response details:\n"
        "   Instruction: %s\n"
        "   Response type: %u\n"
        "   Value: %u\n",
        resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
    }
  }
}

void SmartmicroRadarNode::status_response(
  const com::types::ClientId client_id,
  const std::shared_ptr<com::master::ResponseBatch> & response,
  const std::vector<std::string> & statuses, const std::string & section_name)
{
  for (const auto & instruction_name : statuses) {
    std::vector<std::shared_ptr<Response<uint16_t>>> resp_u16;
    std::vector<std::shared_ptr<Response<uint32_t>>> resp_u32;
    std::vector<std::shared_ptr<Response<uint8_t>>> resp_u8;
    std::vector<std::shared_ptr<Response<int32_t>>> resp_i32;
    bool response_found = false;

    if (response->GetResponse<uint16_t>(section_name, instruction_name.c_str(), resp_u16)) {
      response_found = true;
      for (auto & resp : resp_u16) {
        RCLCPP_INFO(
          this->get_logger(),
          "Response details:\n"
          "   Instruction: %s\n"
          "   Response type: %u\n"
          "   Value: %u\n",
          resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
      }
    }

    if (response->GetResponse<uint32_t>(section_name, instruction_name.c_str(), resp_u32)) {
      response_found = true;
      for (auto & resp : resp_u32) {
        RCLCPP_INFO(
          this->get_logger(),
          "Response details:\n"
          "   Instruction: %s\n"
          "   Response type: %u\n"
          "   Value: %u\n",
          resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
      }
    }

    if (response->GetResponse<uint8_t>(section_name, instruction_name.c_str(), resp_u8)) {
      response_found = true;
      for (auto & resp : resp_u8) {
        RCLCPP_INFO(
          this->get_logger(),
          "Response details:\n"
          "   Instruction: %s\n"
          "   Response type: %u\n"
          "   Value: %u\n",
          resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
      }
    }

    if (response->GetResponse<int32_t>(section_name, instruction_name.c_str(), resp_i32)) {
      response_found = true;
      for (auto & resp : resp_i32) {
        RCLCPP_INFO(
          this->get_logger(),
          "Response details:\n"
          "   Instruction: %s\n"
          "   Response type: %u\n"
          "   Value: %d\n",
          resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
      }
    }

    if (!response_found) {
      RCLCPP_WARN(this->get_logger(), "No response received!");
    }
  }
}

void SmartmicroRadarNode::param_response(
  const com::types::ClientId client_id,
  const std::shared_ptr<com::master::ResponseBatch> & response,
  const std::vector<std::string> & statuses, const std::string & section_name)
{
  for (const auto & instruction_name : statuses) {
    std::vector<std::shared_ptr<Response<uint16_t>>> resp_u16;
    std::vector<std::shared_ptr<Response<uint32_t>>> resp_u32;
    std::vector<std::shared_ptr<Response<uint8_t>>> resp_u8;
    std::vector<std::shared_ptr<Response<float>>> resp_f;
    bool response_found = false;

    if (response->GetResponse<uint16_t>(section_name, instruction_name.c_str(), resp_u16)) {
      response_found = true;
      for (auto & resp : resp_u16) {
        RCLCPP_INFO(
          this->get_logger(),
          "Response details:\n"
          "   Instruction: %s\n"
          "   Response type: %u\n"
          "   Value: %u\n",
          resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
      }
    }

    if (response->GetResponse<uint32_t>(section_name, instruction_name.c_str(), resp_u32)) {
      response_found = true;
      for (auto & resp : resp_u32) {
        RCLCPP_INFO(
          this->get_logger(),
          "Response details:\n"
          "   Instruction: %s\n"
          "   Response type: %u\n"
          "   Value: %u\n",
          resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
      }
    }

    if (response->GetResponse<uint8_t>(section_name, instruction_name.c_str(), resp_u8)) {
      response_found = true;
      for (auto & resp : resp_u8) {
        RCLCPP_INFO(
          this->get_logger(),
          "Response details:\n"
          "   Instruction: %s\n"
          "   Response type: %u\n"
          "   Value: %u\n",
          resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
      }
    }

    if (response->GetResponse<float>(section_name, instruction_name.c_str(), resp_f)) {
      response_found = true;
      for (auto & resp : resp_f) {
        RCLCPP_INFO(
          this->get_logger(),
          "Response details:\n"
          "   Instruction: %s\n"
          "   Response type: %u\n"
          "   Value: %f\n",
          resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
      }
    }

    if (!response_found) {
      RCLCPP_WARN(this->get_logger(), "No response received!");
    }
  }
}

void SmartmicroRadarNode::objectlist_callback_umrra4_mse_v3_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_mse_v3_0_0::comobjectlist::ComObjectList> &
  objectlist_port_umrra4_mse_v3_0_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra4_mse_v3_0_0::comobjectlist::PortHeader> port_header;
  port_header = objectlist_port_umrra4_mse_v3_0_0->GetPortHeader();
  std::shared_ptr<com::master::umrra4_mse_v3_0_0::comobjectlist::ObjectListHeader> object_header;
  object_header = objectlist_port_umrra4_mse_v3_0_0->GetObjectListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortObjectHeader>();
  auto & header = *header_ptr;
  ObjectCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();
  header.cycle_time = object_header->GetCycleTime();
  header.number_of_objects = object_header->GetNumberOfObjects();
  header.ts_measurement = object_header->GetTimestampOfMeasurement();
  const auto & objects = objectlist_port_umrra4_mse_v3_0_0->GetObjectList();
  modifier.reserve(objects.size());
  for (const auto & object : objects) {
    const auto x_pos = object->GetPosX();
    const auto y_pos = object->GetPosY();
    const auto z_pos = object->GetPosZ();
    const auto speed_abs = object->GetSpeedAbs();
    const auto heading = object->GetHeading();
    const auto length = object->GetLength();
    const auto mileage = object->GetMileage();
    const auto quality = object->GetQuality();
    const auto acceleration = object->GetAcceleration();
    const auto object_id = object->GetObjectId();
    const auto idle_cycles = object->GetIdleCycles();
    const auto spline_idx = object->GetSplineIdx();
    const auto object_class = object->GetObjectClass();
    const auto status = object->GetStatus();

    modifier.push_back(
      {x_pos, y_pos, z_pos, speed_abs, heading, length, mileage, quality, acceleration, object_id,
        idle_cycles, spline_idx, object_class, status});
  }

  m_publishers_obj[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_obj_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrra4_mse_v3_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_mse_v3_0_0::comtargetlist::ComTargetList> &
  targetlist_port_umrra4_mse_v3_0_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra4_mse_v3_0_0::comtargetlist::PortHeader> port_header;
  port_header = targetlist_port_umrra4_mse_v3_0_0->GetPortHeader();
  std::shared_ptr<com::master::umrra4_mse_v3_0_0::comtargetlist::TargetListHeader> target_header;
  target_header = targetlist_port_umrra4_mse_v3_0_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();
  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.prf = target_header->GetPrf();
  header.umambiguous_speed = target_header->GetUmambiguousSpeed();
  header.acquisition_start = target_header->GetAcquisitionStart();
  const auto & targets = targetlist_port_umrra4_mse_v3_0_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::faultreport_callback_umrra4_mse_v3_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrra4_mse_v3_0_0::faultreports::FaultReports> &
  fault_report_umrra4_mse_v3_0_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra4_mse_v3_0_0::faultreports::PortHeader> port_header;
  std::shared_ptr<com::master::umrra4_mse_v3_0_0::faultreports::FaultReportHeader>
  fault_header;
  port_header = fault_report_umrra4_mse_v3_0_0->GetPortHeader();
  fault_header = fault_report_umrra4_mse_v3_0_0->GetFaultReportHeader();
  auto msg_ptr = std::make_unique<umrr_ros2_msgs::msg::PortFaultReportsMsg>();
  auto & msg = *msg_ptr;

  fill_ros_header_stamp(
    msg,
    port_header->GetTimestamp(),
    sensor_idx
  );

  msg.fault_report_header.port_identifier = port_header->GetPortIdentifier();
  msg.fault_report_header.port_ver_major = port_header->GetPortVersionMajor();
  msg.fault_report_header.port_ver_minor = port_header->GetPortVersionMinor();
  msg.fault_report_header.port_size = port_header->GetPortSize();
  msg.fault_report_header.body_endianness = port_header->GetBodyEndianness();
  msg.fault_report_header.port_index = port_header->GetPortIndex();
  msg.fault_report_header.header_ver_major = port_header->GetHeaderVersionMajor();
  msg.fault_report_header.header_ver_minor = port_header->GetHeaderVersionMinor();

  msg.fault_report_header.num_max_reports = fault_header->GetNumMaxReports();
  msg.fault_report_header.num_valid_reports = fault_header->GetNumValidReports();
  msg.fault_report_header.faults_time_line = fault_header->GetFaultsTimeline();

  const auto & fault_list = fault_report_umrra4_mse_v3_0_0->GetFaultReportList();

  msg.reports.reserve(fault_list.size());

  for (const auto & fault : fault_list) {
    if (!fault) {
      continue;
    }

    umrr_ros2_msgs::msg::PortFaultReport report_msg;

    report_msg.module_id = fault->GetModuleId();
    report_msg.fault_group = fault->GetFaultGroup();
    report_msg.fault_code = fault->GetFaultCode();
    report_msg.fault_errno = fault->GetFaultErrno();
    report_msg.fault_time_stamp = fault->GetFaultTimestamp();
    report_msg.cycle_count = fault->GetCycleCount();
    report_msg.instance_id = fault->GetInstanceId();
    report_msg.criticality = fault->GetCriticality();
    report_msg.occurence_count = fault->GetOccurrenceCount();

    msg.reports.push_back(report_msg);
  }

  m_publishers_fault_report_msg[sensor_idx]->publish(std::move(msg_ptr));
}

void SmartmicroRadarNode::objectlist_callback_umrra4_mse_v2_1_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_mse_v2_1_0::comobjectlist::ComObjectList> &
  objectlist_port_umrra4_mse_v2_1_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra4_mse_v2_1_0::comobjectlist::PortHeader> port_header;
  port_header = objectlist_port_umrra4_mse_v2_1_0->GetPortHeader();
  std::shared_ptr<com::master::umrra4_mse_v2_1_0::comobjectlist::ObjectListHeader> object_header;
  object_header = objectlist_port_umrra4_mse_v2_1_0->GetObjectListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortObjectHeader>();
  auto & header = *header_ptr;
  ObjectCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();
  header.cycle_time = object_header->GetCycleTime();
  header.number_of_objects = object_header->GetNumberOfObjects();
  header.ts_measurement = object_header->GetTimestampOfMeasurement();
  const auto & objects = objectlist_port_umrra4_mse_v2_1_0->GetObjectList();
  modifier.reserve(objects.size());
  for (const auto & object : objects) {
    const auto x_pos = object->GetPosX();
    const auto y_pos = object->GetPosY();
    const auto z_pos = object->GetPosZ();
    const auto speed_abs = object->GetSpeedAbs();
    const auto heading = object->GetHeading();
    const auto length = object->GetLength();
    const auto mileage = object->GetMileage();
    const auto quality = object->GetQuality();
    const auto acceleration = object->GetAcceleration();
    const auto object_id = object->GetObjectId();
    const auto idle_cycles = object->GetIdleCycles();
    const auto spline_idx = object->GetSplineIdx();
    const auto object_class = object->GetObjectClass();
    const auto status = object->GetStatus();

    modifier.push_back(
      {x_pos, y_pos, z_pos, speed_abs, heading, length, mileage, quality, acceleration, object_id,
        idle_cycles, spline_idx, object_class, status});
  }

  m_publishers_obj[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_obj_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrra4_mse_v2_1_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_mse_v2_1_0::comtargetlist::ComTargetList> &
  targetlist_port_umrra4_mse_v2_1_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra4_mse_v2_1_0::comtargetlist::PortHeader> port_header;
  port_header = targetlist_port_umrra4_mse_v2_1_0->GetPortHeader();
  std::shared_ptr<com::master::umrra4_mse_v2_1_0::comtargetlist::TargetListHeader> target_header;
  target_header = targetlist_port_umrra4_mse_v2_1_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();
  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.prf = target_header->GetPrf();
  header.umambiguous_speed = target_header->GetUmambiguousSpeed();
  header.acquisition_start = target_header->GetAcquisitionStart();
  const auto & targets = targetlist_port_umrra4_mse_v2_1_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::objectlist_callback_umrra4_mse_v1_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_mse_v1_0_0::comobjectlist::ComObjectList> &
  objectlist_port_umrra4_mse_v1_0_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra4_mse_v1_0_0::comobjectlist::PortHeader> port_header;
  port_header = objectlist_port_umrra4_mse_v1_0_0->GetPortHeader();
  std::shared_ptr<com::master::umrra4_mse_v1_0_0::comobjectlist::ObjectListHeader> object_header;
  object_header = objectlist_port_umrra4_mse_v1_0_0->GetObjectListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortObjectHeader>();
  auto & header = *header_ptr;
  ObjectCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();
  header.cycle_time = object_header->GetCycleTime();
  header.number_of_objects = object_header->GetNumberOfObjects();
  header.ts_measurement = object_header->GetTimestampOfMeasurement();
  const auto & objects = objectlist_port_umrra4_mse_v1_0_0->GetObjectList();
  modifier.reserve(objects.size());
  for (const auto & object : objects) {
    const auto x_pos = object->GetPosX();
    const auto y_pos = object->GetPosY();
    const auto z_pos = object->GetPosZ();
    const auto speed_abs = object->GetSpeedAbs();
    const auto heading = object->GetHeading();
    const auto length = object->GetLength();
    const auto mileage = object->GetMileage();
    const auto quality = object->GetQuality();
    const auto acceleration = object->GetAcceleration();
    const auto object_id = object->GetObjectId();
    const auto idle_cycles = object->GetIdleCycles();
    const auto spline_idx = object->GetSplineIdx();
    const auto object_class = object->GetObjectClass();
    const auto status = object->GetStatus();

    modifier.push_back(
      {x_pos, y_pos, z_pos, speed_abs, heading, length, mileage, quality, acceleration, object_id,
        idle_cycles, spline_idx, object_class, status});
  }

  m_publishers_obj[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_obj_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrra4_mse_v1_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_mse_v1_0_0::comtargetlist::ComTargetList> &
  targetlist_port_umrra4_mse_v1_0_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra4_mse_v1_0_0::comtargetlist::PortHeader> port_header;
  port_header = targetlist_port_umrra4_mse_v1_0_0->GetPortHeader();
  std::shared_ptr<com::master::umrra4_mse_v1_0_0::comtargetlist::TargetListHeader> target_header;
  target_header = targetlist_port_umrra4_mse_v1_0_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();
  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.prf = target_header->GetPrf();
  header.umambiguous_speed = target_header->GetUmambiguousSpeed();
  header.acquisition_start = target_header->GetAcquisitionStart();
  const auto & targets = targetlist_port_umrra4_mse_v1_0_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::objectlist_callback_umrr9f_mse_v2_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v2_0_0::comobjectlist::ComObjectList> &
  objectlist_port_umrr9f_mse_v2_0_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_mse_v2_0_0::comobjectlist::PortHeader> port_header;
  port_header = objectlist_port_umrr9f_mse_v2_0_0->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_mse_v2_0_0::comobjectlist::ObjectListHeader>
  object_header;
  object_header = objectlist_port_umrr9f_mse_v2_0_0->GetObjectListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortObjectHeader>();
  auto & header = *header_ptr;
  ObjectCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();
  header.cycle_time = object_header->GetCycleTime();
  header.number_of_objects = object_header->GetNumberOfObjects();
  header.ts_measurement = object_header->GetTimestampOfMeasurement();
  const auto & objects = objectlist_port_umrr9f_mse_v2_0_0->GetObjectList();
  modifier.reserve(objects.size());
  for (const auto & object : objects) {
    const auto x_pos = object->GetPosX();
    const auto y_pos = object->GetPosY();
    const auto z_pos = object->GetPosZ();
    const auto speed_abs = object->GetSpeedAbs();
    const auto heading = object->GetHeading();
    const auto length = object->GetLength();
    const auto mileage = object->GetMileage();
    const auto quality = object->GetQuality();
    const auto acceleration = object->GetAcceleration();
    const auto object_id = object->GetObjectId();
    const auto idle_cycles = object->GetIdleCycles();
    const auto spline_idx = object->GetSplineIdx();
    const auto object_class = object->GetObjectClass();
    const auto status = object->GetStatus();

    modifier.push_back(
      {x_pos, y_pos, z_pos, speed_abs, heading, length, mileage, quality, acceleration, object_id,
        idle_cycles, spline_idx, object_class, status});
  }

  m_publishers_obj[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_obj_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrr9f_mse_v2_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v2_0_0::comtargetlist::ComTargetList> &
  targetlist_port_umrr9f_mse_v2_0_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_mse_v2_0_0::comtargetlist::PortHeader> port_header;
  port_header = targetlist_port_umrr9f_mse_v2_0_0->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_mse_v2_0_0::comtargetlist::TargetListHeader>
  target_header;
  target_header = targetlist_port_umrr9f_mse_v2_0_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();
  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.prf = target_header->GetPrf();
  header.umambiguous_speed = target_header->GetUmambiguousSpeed();
  header.acquisition_start = target_header->GetAcquisitionStart();
  const auto & targets = targetlist_port_umrr9f_mse_v2_0_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::faultreport_callback_umrr9f_mse_v2_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9f_t169_mse_v2_0_0::faultreports::FaultReports> &
  fault_report_umrr9f_mse_v2_0_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_mse_v2_0_0::faultreports::PortHeader> port_header;
  std::shared_ptr<com::master::umrr9f_t169_mse_v2_0_0::faultreports::FaultReportHeader>
  fault_header;
  port_header = fault_report_umrr9f_mse_v2_0_0->GetPortHeader();
  fault_header = fault_report_umrr9f_mse_v2_0_0->GetFaultReportHeader();
  auto msg_ptr = std::make_unique<umrr_ros2_msgs::msg::PortFaultReportsMsg>();
  auto & msg = *msg_ptr;

  fill_ros_header_stamp(
    msg,
    port_header->GetTimestamp(),
    sensor_idx
  );

  msg.fault_report_header.port_identifier = port_header->GetPortIdentifier();
  msg.fault_report_header.port_ver_major = port_header->GetPortVersionMajor();
  msg.fault_report_header.port_ver_minor = port_header->GetPortVersionMinor();
  msg.fault_report_header.port_size = port_header->GetPortSize();
  msg.fault_report_header.body_endianness = port_header->GetBodyEndianness();
  msg.fault_report_header.port_index = port_header->GetPortIndex();
  msg.fault_report_header.header_ver_major = port_header->GetHeaderVersionMajor();
  msg.fault_report_header.header_ver_minor = port_header->GetHeaderVersionMinor();

  msg.fault_report_header.num_max_reports = fault_header->GetNumMaxReports();
  msg.fault_report_header.num_valid_reports = fault_header->GetNumValidReports();
  msg.fault_report_header.faults_time_line = fault_header->GetFaultsTimeline();

  const auto & fault_list = fault_report_umrr9f_mse_v2_0_0->GetFaultReportList();

  msg.reports.reserve(fault_list.size());

  for (const auto & fault : fault_list) {
    if (!fault) {
      continue;
    }

    umrr_ros2_msgs::msg::PortFaultReport report_msg;

    report_msg.module_id = fault->GetModuleId();
    report_msg.fault_group = fault->GetFaultGroup();
    report_msg.fault_code = fault->GetFaultCode();
    report_msg.fault_errno = fault->GetFaultErrno();
    report_msg.fault_time_stamp = fault->GetFaultTimestamp();
    report_msg.cycle_count = fault->GetCycleCount();
    report_msg.instance_id = fault->GetInstanceId();
    report_msg.criticality = fault->GetCriticality();
    report_msg.occurence_count = fault->GetOccurrenceCount();

    msg.reports.push_back(report_msg);
  }

  m_publishers_fault_report_msg[sensor_idx]->publish(std::move(msg_ptr));
}

void SmartmicroRadarNode::objectlist_callback_umrr9f_mse_v1_3_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v1_3_0::comobjectlist::ComObjectList> &
  objectlist_port_umrr9f_mse_v1_3_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_mse_v1_3_0::comobjectlist::PortHeader> port_header;
  port_header = objectlist_port_umrr9f_mse_v1_3_0->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_mse_v1_3_0::comobjectlist::ObjectListHeader>
  object_header;
  object_header = objectlist_port_umrr9f_mse_v1_3_0->GetObjectListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortObjectHeader>();
  auto & header = *header_ptr;
  ObjectCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();
  header.cycle_time = object_header->GetCycleTime();
  header.number_of_objects = object_header->GetNumberOfObjects();
  header.ts_measurement = object_header->GetTimestampOfMeasurement();
  const auto & objects = objectlist_port_umrr9f_mse_v1_3_0->GetObjectList();
  modifier.reserve(objects.size());
  for (const auto & object : objects) {
    const auto x_pos = object->GetPosX();
    const auto y_pos = object->GetPosY();
    const auto z_pos = object->GetPosZ();
    const auto speed_abs = object->GetSpeedAbs();
    const auto heading = object->GetHeading();
    const auto length = object->GetLength();
    const auto mileage = object->GetMileage();
    const auto quality = object->GetQuality();
    const auto acceleration = object->GetAcceleration();
    const auto object_id = object->GetObjectId();
    const auto idle_cycles = object->GetIdleCycles();
    const auto spline_idx = object->GetSplineIdx();
    const auto object_class = object->GetObjectClass();
    const auto status = object->GetStatus();

    modifier.push_back(
      {x_pos, y_pos, z_pos, speed_abs, heading, length, mileage, quality, acceleration, object_id,
        idle_cycles, spline_idx, object_class, status});
  }

  m_publishers_obj[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_obj_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrr9f_mse_v1_3_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v1_3_0::comtargetlist::ComTargetList> &
  targetlist_port_umrr9f_mse_v1_3_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_mse_v1_3_0::comtargetlist::PortHeader> port_header;
  port_header = targetlist_port_umrr9f_mse_v1_3_0->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_mse_v1_3_0::comtargetlist::TargetListHeader>
  target_header;
  target_header = targetlist_port_umrr9f_mse_v1_3_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();
  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.prf = target_header->GetPrf();
  header.umambiguous_speed = target_header->GetUmambiguousSpeed();
  header.acquisition_start = target_header->GetAcquisitionStart();
  const auto & targets = targetlist_port_umrr9f_mse_v1_3_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::objectlist_callback_umrr9f_mse_v1_1_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v1_1_0::comobjectlist::ComObjectList> &
  objectlist_port_umrr9f_mse_v1_1_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_mse_v1_1_0::comobjectlist::PortHeader> port_header;
  port_header = objectlist_port_umrr9f_mse_v1_1_0->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_mse_v1_1_0::comobjectlist::ObjectListHeader>
  object_header;
  object_header = objectlist_port_umrr9f_mse_v1_1_0->GetObjectListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortObjectHeader>();
  auto & header = *header_ptr;
  ObjectCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();
  header.cycle_time = object_header->GetCycleTime();
  header.number_of_objects = object_header->GetNumberOfObjects();
  header.ts_measurement = object_header->GetTimestampOfMeasurement();
  const auto & objects = objectlist_port_umrr9f_mse_v1_1_0->GetObjectList();
  modifier.reserve(objects.size());
  for (const auto & object : objects) {
    const auto x_pos = object->GetPosX();
    const auto y_pos = object->GetPosY();
    const auto z_pos = object->GetPosZ();
    const auto speed_abs = object->GetSpeedAbs();
    const auto heading = object->GetHeading();
    const auto length = object->GetLength();
    const auto mileage = object->GetMileage();
    const auto quality = object->GetQuality();
    const auto acceleration = object->GetAcceleration();
    const auto object_id = object->GetObjectId();
    const auto idle_cycles = object->GetIdleCycles();
    const auto spline_idx = object->GetSplineIdx();
    const auto object_class = object->GetObjectClass();
    const auto status = object->GetStatus();

    modifier.push_back(
      {x_pos, y_pos, z_pos, speed_abs, heading, length, mileage, quality, acceleration, object_id,
        idle_cycles, spline_idx, object_class, status});
  }

  m_publishers_obj[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_obj_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrr9f_mse_v1_1_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v1_1_0::comtargetlist::ComTargetList> &
  targetlist_port_umrr9f_mse_v1_1_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_mse_v1_1_0::comtargetlist::PortHeader> port_header;
  port_header = targetlist_port_umrr9f_mse_v1_1_0->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_mse_v1_1_0::comtargetlist::TargetListHeader>
  target_header;
  target_header = targetlist_port_umrr9f_mse_v1_1_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();
  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.prf = target_header->GetPrf();
  header.umambiguous_speed = target_header->GetUmambiguousSpeed();
  header.acquisition_start = target_header->GetAcquisitionStart();
  const auto & targets = targetlist_port_umrr9f_mse_v1_1_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::objectlist_callback_umrr9f_mse_v1_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v1_0_0::comobjectlist::ComObjectList> &
  objectlist_port_umrr9f_mse_v1_0_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_mse_v1_0_0::comobjectlist::PortHeader> port_header;
  port_header = objectlist_port_umrr9f_mse_v1_0_0->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_mse_v1_0_0::comobjectlist::ObjectListHeader>
  object_header;
  object_header = objectlist_port_umrr9f_mse_v1_0_0->GetObjectListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortObjectHeader>();
  auto & header = *header_ptr;
  ObjectCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();
  header.cycle_time = object_header->GetCycleTime();
  header.number_of_objects = object_header->GetNumberOfObjects();
  header.ts_measurement = object_header->GetTimestampOfMeasurement();
  const auto & objects = objectlist_port_umrr9f_mse_v1_0_0->GetObjectList();
  modifier.reserve(objects.size());
  for (const auto & object : objects) {
    const auto x_pos = object->GetPosX();
    const auto y_pos = object->GetPosY();
    const auto z_pos = object->GetPosZ();
    const auto speed_abs = object->GetSpeedAbs();
    const auto heading = object->GetHeading();
    const auto length = object->GetLength();
    const auto mileage = object->GetMileage();
    const auto quality = object->GetQuality();
    const auto acceleration = object->GetAcceleration();
    const auto object_id = object->GetObjectId();
    const auto idle_cycles = object->GetIdleCycles();
    const auto spline_idx = object->GetSplineIdx();
    const auto object_class = object->GetObjectClass();
    const auto status = object->GetStatus();

    modifier.push_back(
      {x_pos, y_pos, z_pos, speed_abs, heading, length, mileage, quality, acceleration, object_id,
        idle_cycles, spline_idx, object_class, status});
  }

  m_publishers_obj[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_obj_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrr9f_mse_v1_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v1_0_0::comtargetlist::ComTargetList> &
  targetlist_port_umrr9f_mse_v1_0_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_mse_v1_0_0::comtargetlist::PortHeader> port_header;
  port_header = targetlist_port_umrr9f_mse_v1_0_0->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_mse_v1_0_0::comtargetlist::TargetListHeader>
  target_header;
  target_header = targetlist_port_umrr9f_mse_v1_0_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();
  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.prf = target_header->GetPrf();
  header.umambiguous_speed = target_header->GetUmambiguousSpeed();
  header.acquisition_start = target_header->GetAcquisitionStart();
  const auto & targets = targetlist_port_umrr9f_mse_v1_0_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrr96(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr96_t153_automotive_v1_2_2::comtargetlist::ComTargetList> &
  targetlist_port_umrr96,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr96_t153_automotive_v1_2_2::comtargetlist::PortHeader>
  port_header;
  port_header = targetlist_port_umrr96->GetPortHeader();
  std::shared_ptr<com::master::umrr96_t153_automotive_v1_2_2::comtargetlist::TargetListHeader>
  target_header;
  target_header = targetlist_port_umrr96->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();
  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.acquisition_setup_valid = true;
  const auto & targets = targetlist_port_umrr96->GetTargetList();
  modifier.reserve(targets.size());
  auto raw_quality_ptr = std::make_unique<umrr_ros2_msgs::msg::Umrr96RawQuality>();
  auto & raw_quality = *raw_quality_ptr;
  const bool publish_raw = raw_quality_publishers_[sensor_idx] &&
    raw_quality_publishers_[sensor_idx]->get_subscription_count() > 0;
  if (publish_raw) {
    raw_quality.header = msg.header;
    raw_quality.sensor_id = client_id;
    raw_quality.semantics = umrr_ros2_msgs::msg::Umrr96RawQuality::SEMANTICS_UNVERIFIED;
    raw_quality.false_alarm_probability_raw.reserve(targets.size());
    raw_quality.flags_raw.reserve(targets.size());
  }
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        kRadarFloatSentinel, kRadarFlagsSentinel, target->GetPeakIdx()});
    if (publish_raw) {
      raw_quality.false_alarm_probability_raw.push_back(target->GetFalseAlarmProbability());
      raw_quality.flags_raw.push_back(target->GetFlags());
    }
  }

  if (publish_raw) {raw_quality_publishers_[sensor_idx]->publish(std::move(raw_quality_ptr));}
  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrr11(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr11_t132_automotive_v1_1_2::comtargetlist::ComTargetList> &
  targetlist_port_umrr11,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr11_t132_automotive_v1_1_2::comtargetlist::PortHeader>
  port_header;
  port_header = targetlist_port_umrr11->GetPortHeader();
  std::shared_ptr<com::master::umrr11_t132_automotive_v1_1_2::comtargetlist::TargetListHeader>
  target_header;
  target_header = targetlist_port_umrr11->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();

  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  const auto & targets = targetlist_port_umrr11->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrr9f_v1_1_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9f_t169_automotive_v1_1_1::comtargetlistport::ComTargetListPort> &
  targetlist_port_umrr9f_v1_1_1,
  const com::types::ClientId client_id)
{
  std::shared_ptr<
    com::master::umrr9f_t169_automotive_v1_1_1::comtargetlistport::GenericPortHeader>
  port_header;
  port_header = targetlist_port_umrr9f_v1_1_1->GetGenericPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_automotive_v1_1_1::comtargetlistport::StaticPortHeader>
  target_header;
  target_header = targetlist_port_umrr9f_v1_1_1->GetStaticPortHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortId();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();

  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  const auto & targets = targetlist_port_umrr9f_v1_1_1->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetTgtNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRCS(), target->GetTgtNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrr9f_v2_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9f_t169_automotive_v2_0_0::comtargetlistport::ComTargetListPort> &
  targetlist_port_umrr9f_v2_0_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<
    com::master::umrr9f_t169_automotive_v2_0_0::comtargetlistport::GenericPortHeader>
  port_header;
  port_header = targetlist_port_umrr9f_v2_0_0->GetGenericPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_automotive_v2_0_0::comtargetlistport::StaticPortHeader>
  target_header;
  target_header = targetlist_port_umrr9f_v2_0_0->GetStaticPortHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortId();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();

  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  // The v2.0.0 getter names are swapped relative to their meaning: the UIF
  // serialization (com_target_list_port.xml) documents offset 6 "AcquisitionTx"
  // as the TX antenna index and offset 8 "AcquisitionTxAnt" as the centre
  // frequency index, the same layout as other port target lists.
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweep();
  header.acquisition_cf_idx = target_header->GetAcquisitionTxAnt();
  header.acquisition_start = target_header->GetAcquisitionStart();
  const auto & targets = targetlist_port_umrr9f_v2_0_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetTgtNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRCS(), target->GetTgtNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrr9f_v2_1_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_automotive_v2_1_1::comtargetlist::ComTargetList> &
  targetlist_port_umrr9f_v2_1_1,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_automotive_v2_1_1::comtargetlist::PortHeader>
  port_header;
  port_header = targetlist_port_umrr9f_v2_1_1->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_automotive_v2_1_1::comtargetlist::TargetListHeader>
  target_header;
  target_header = targetlist_port_umrr9f_v2_1_1->GetTargetListHeader();
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();

  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.acquisition_start = target_header->GetAcquisitionStart();
  const auto & targets = targetlist_port_umrr9f_v2_1_1->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrr9f_v2_2_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_automotive_v2_2_1::comtargetlist::ComTargetList> &
  targetlist_port_umrr9f_v2_2_1,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_automotive_v2_2_1::comtargetlist::PortHeader>
  port_header;
  port_header = targetlist_port_umrr9f_v2_2_1->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_automotive_v2_2_1::comtargetlist::TargetListHeader>
  target_header;
  target_header = targetlist_port_umrr9f_v2_2_1->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();

  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.prf = target_header->GetPrf();
  header.umambiguous_speed = target_header->GetUmambiguousSpeed();
  header.acquisition_start = target_header->GetAcquisitionStart();
  const auto & targets = targetlist_port_umrr9f_v2_2_1->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrr9f_v2_4_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_automotive_v2_4_1::comtargetlist::ComTargetList> &
  targetlist_port_umrr9f_v2_4_1,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_automotive_v2_4_1::comtargetlist::PortHeader>
  port_header;
  port_header = targetlist_port_umrr9f_v2_4_1->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_automotive_v2_4_1::comtargetlist::TargetListHeader>
  target_header;
  target_header = targetlist_port_umrr9f_v2_4_1->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();

  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.prf = target_header->GetPrf();
  header.umambiguous_speed = target_header->GetUmambiguousSpeed();
  header.acquisition_start = target_header->GetAcquisitionStart();
  const auto & targets = targetlist_port_umrr9f_v2_4_1->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrr9f_v3_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_automotive_v3_0_0::comtargetlist::ComTargetList> &
  targetlist_port_umrr9f_v3_0_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_automotive_v3_0_0::comtargetlist::PortHeader>
  port_header;
  port_header = targetlist_port_umrr9f_v3_0_0->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_automotive_v3_0_0::comtargetlist::TargetListHeader>
  target_header;
  target_header = targetlist_port_umrr9f_v3_0_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();

  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.prf = target_header->GetPrf();
  header.umambiguous_speed = target_header->GetUmambiguousSpeed();
  header.acquisition_start = target_header->GetAcquisitionStart();
  const auto & targets = targetlist_port_umrr9f_v3_0_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrr9f_v3_2_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_automotive_v3_2_0::comtargetlist::ComTargetList> &
  targetlist_port_umrr9f_v3_2_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_automotive_v3_2_0::comtargetlist::PortHeader>
  port_header;
  port_header = targetlist_port_umrr9f_v3_2_0->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_automotive_v3_2_0::comtargetlist::TargetListHeader>
  target_header;
  target_header = targetlist_port_umrr9f_v3_2_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();

  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.prf = target_header->GetPrf();
  header.umambiguous_speed = target_header->GetUmambiguousSpeed();
  header.acquisition_start = target_header->GetAcquisitionStart();
  header.acquisition_time_stamp_base = target_header->GetAcquisitionTimestampBase();

  const auto & targets = targetlist_port_umrr9f_v3_2_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }
  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::faultreport_callback_umrr9f_v3_2_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9f_t169_automotive_v3_2_0::faultreports::FaultReports> &
  fault_report_umrr9f_v3_2_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_automotive_v3_2_0::faultreports::PortHeader>
  port_header;
  std::shared_ptr<com::master::umrr9f_t169_automotive_v3_2_0::faultreports::FaultReportHeader>
  fault_header;
  port_header = fault_report_umrr9f_v3_2_0->GetPortHeader();
  fault_header = fault_report_umrr9f_v3_2_0->GetFaultReportHeader();
  auto msg_ptr = std::make_unique<umrr_ros2_msgs::msg::PortFaultReportsMsg>();
  auto & msg = *msg_ptr;

  fill_ros_header_stamp(
    msg,
    port_header->GetTimestamp(),
    sensor_idx
  );

  msg.fault_report_header.port_identifier = port_header->GetPortIdentifier();
  msg.fault_report_header.port_ver_major = port_header->GetPortVersionMajor();
  msg.fault_report_header.port_ver_minor = port_header->GetPortVersionMinor();
  msg.fault_report_header.port_size = port_header->GetPortSize();
  msg.fault_report_header.body_endianness = port_header->GetBodyEndianness();
  msg.fault_report_header.port_index = port_header->GetPortIndex();
  msg.fault_report_header.header_ver_major = port_header->GetHeaderVersionMajor();
  msg.fault_report_header.header_ver_minor = port_header->GetHeaderVersionMinor();

  msg.fault_report_header.num_max_reports = fault_header->GetNumMaxReports();
  msg.fault_report_header.num_valid_reports = fault_header->GetNumValidReports();
  msg.fault_report_header.faults_time_line = fault_header->GetFaultsTimeline();

  const auto & fault_list = fault_report_umrr9f_v3_2_0->GetFaultReportList();

  msg.reports.reserve(fault_list.size());

  for (const auto & fault : fault_list) {
    if (!fault) {
      continue;
    }

    umrr_ros2_msgs::msg::PortFaultReport report_msg;

    report_msg.module_id = fault->GetModuleId();
    report_msg.fault_group = fault->GetFaultGroup();
    report_msg.fault_code = fault->GetFaultCode();
    report_msg.fault_errno = fault->GetFaultErrno();
    report_msg.fault_time_stamp = fault->GetFaultTimestamp();
    report_msg.cycle_count = fault->GetCycleCount();
    report_msg.instance_id = fault->GetInstanceId();
    report_msg.criticality = fault->GetCriticality();
    report_msg.occurence_count = fault->GetOccurrenceCount();

    msg.reports.push_back(report_msg);
  }

  m_publishers_fault_report_msg[sensor_idx]->publish(std::move(msg_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrr9d_v1_0_3(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9d_t152_automotive_v1_0_3::comtargetlist::ComTargetList> &
  targetlist_port_umrr9d_v1_0_3,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9d_t152_automotive_v1_0_3::comtargetlist::PortHeader>
  port_header;
  port_header = targetlist_port_umrr9d_v1_0_3->GetPortHeader();
  std::shared_ptr<com::master::umrr9d_t152_automotive_v1_0_3::comtargetlist::TargetListHeader>
  target_header;
  target_header = targetlist_port_umrr9d_v1_0_3->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();

  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.acquisition_start = target_header->GetAcquisitionStart();
  const auto & targets = targetlist_port_umrr9d_v1_0_3->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrr9d_v1_2_2(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9d_t152_automotive_v1_2_2::comtargetlist::ComTargetList> &
  targetlist_port_umrr9d_v1_2_2,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9d_t152_automotive_v1_2_2::comtargetlist::PortHeader>
  port_header;
  port_header = targetlist_port_umrr9d_v1_2_2->GetPortHeader();
  std::shared_ptr<com::master::umrr9d_t152_automotive_v1_2_2::comtargetlist::TargetListHeader>
  target_header;
  target_header = targetlist_port_umrr9d_v1_2_2->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();

  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.prf = target_header->GetPrf();
  header.umambiguous_speed = target_header->GetUmambiguousSpeed();
  header.acquisition_start = target_header->GetAcquisitionStart();
  const auto & targets = targetlist_port_umrr9d_v1_2_2->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrr9d_v1_4_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9d_t152_automotive_v1_4_1::comtargetlist::ComTargetList> &
  targetlist_port_umrr9d_v1_4_1,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9d_t152_automotive_v1_4_1::comtargetlist::PortHeader>
  port_header;
  port_header = targetlist_port_umrr9d_v1_4_1->GetPortHeader();
  std::shared_ptr<com::master::umrr9d_t152_automotive_v1_4_1::comtargetlist::TargetListHeader>
  target_header;
  target_header = targetlist_port_umrr9d_v1_4_1->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();

  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.prf = target_header->GetPrf();
  header.umambiguous_speed = target_header->GetUmambiguousSpeed();
  header.acquisition_start = target_header->GetAcquisitionStart();
  const auto & targets = targetlist_port_umrr9d_v1_4_1->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrr9d_v1_5_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9d_t152_automotive_v1_5_0::comtargetlist::ComTargetList> &
  targetlist_port_umrr9d_v1_5_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9d_t152_automotive_v1_5_0::comtargetlist::PortHeader>
  port_header;
  port_header = targetlist_port_umrr9d_v1_5_0->GetPortHeader();
  std::shared_ptr<com::master::umrr9d_t152_automotive_v1_5_0::comtargetlist::TargetListHeader>
  target_header;
  target_header = targetlist_port_umrr9d_v1_5_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();

  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.prf = target_header->GetPrf();
  header.umambiguous_speed = target_header->GetUmambiguousSpeed();
  header.acquisition_start = target_header->GetAcquisitionStart();
  const auto & targets = targetlist_port_umrr9d_v1_5_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}
void SmartmicroRadarNode::targetlist_callback_umrr9d_v1_7_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9d_t152_automotive_v1_7_0::comtargetlist::ComTargetList> &
  targetlist_port_umrr9d_v1_7_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9d_t152_automotive_v1_7_0::comtargetlist::PortHeader>
  port_header;
  port_header = targetlist_port_umrr9d_v1_7_0->GetPortHeader();
  std::shared_ptr<com::master::umrr9d_t152_automotive_v1_7_0::comtargetlist::TargetListHeader>
  target_header;
  target_header = targetlist_port_umrr9d_v1_7_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();

  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.prf = target_header->GetPrf();
  header.umambiguous_speed = target_header->GetUmambiguousSpeed();
  header.acquisition_start = target_header->GetAcquisitionStart();

  header.acquisition_time_stamp_base = target_header->GetAcquisitionTimestampBase();

  const auto & targets = targetlist_port_umrr9d_v1_7_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::faultreport_callback_umrr9d_v1_7_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9d_t152_automotive_v1_7_0::faultreports::FaultReports> &
  fault_report_umrr9d_v1_7_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9d_t152_automotive_v1_7_0::faultreports::PortHeader>
  port_header;
  std::shared_ptr<com::master::umrr9d_t152_automotive_v1_7_0::faultreports::FaultReportHeader>
  fault_header;
  port_header = fault_report_umrr9d_v1_7_0->GetPortHeader();
  fault_header = fault_report_umrr9d_v1_7_0->GetFaultReportHeader();
  auto msg_ptr = std::make_unique<umrr_ros2_msgs::msg::PortFaultReportsMsg>();
  auto & msg = *msg_ptr;

  fill_ros_header_stamp(
    msg,
    port_header->GetTimestamp(),
    sensor_idx
  );

  msg.fault_report_header.port_identifier = port_header->GetPortIdentifier();
  msg.fault_report_header.port_ver_major = port_header->GetPortVersionMajor();
  msg.fault_report_header.port_ver_minor = port_header->GetPortVersionMinor();
  msg.fault_report_header.port_size = port_header->GetPortSize();
  msg.fault_report_header.body_endianness = port_header->GetBodyEndianness();
  msg.fault_report_header.port_index = port_header->GetPortIndex();
  msg.fault_report_header.header_ver_major = port_header->GetHeaderVersionMajor();
  msg.fault_report_header.header_ver_minor = port_header->GetHeaderVersionMinor();
  msg.fault_report_header.num_max_reports = fault_header->GetNumMaxReports();
  msg.fault_report_header.num_valid_reports = fault_header->GetNumValidReports();
  msg.fault_report_header.faults_time_line = fault_header->GetFaultsTimeline();

  const auto & fault_list = fault_report_umrr9d_v1_7_0->GetFaultReportList();

  msg.reports.reserve(fault_list.size());

  for (const auto & fault : fault_list) {
    if (!fault) {
      continue;
    }

    umrr_ros2_msgs::msg::PortFaultReport report_msg;

    report_msg.module_id = fault->GetModuleId();
    report_msg.fault_group = fault->GetFaultGroup();
    report_msg.fault_code = fault->GetFaultCode();
    report_msg.fault_errno = fault->GetFaultErrno();
    report_msg.fault_time_stamp = fault->GetFaultTimestamp();
    report_msg.cycle_count = fault->GetCycleCount();
    report_msg.instance_id = fault->GetInstanceId();
    report_msg.criticality = fault->GetCriticality();
    report_msg.occurence_count = fault->GetOccurrenceCount();

    msg.reports.push_back(report_msg);
  }

  m_publishers_fault_report_msg[sensor_idx]->publish(std::move(msg_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrra4_v1_0_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_automotive_v1_0_1::comtargetlist::ComTargetList> &
  targetlist_port_umrra4_v1_0_1,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra4_automotive_v1_0_1::comtargetlist::PortHeader> port_header;
  port_header = targetlist_port_umrra4_v1_0_1->GetPortHeader();
  std::shared_ptr<com::master::umrra4_automotive_v1_0_1::comtargetlist::TargetListHeader>
  target_header;
  target_header = targetlist_port_umrra4_v1_0_1->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();

  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.prf = target_header->GetPrf();
  header.umambiguous_speed = target_header->GetUmambiguousSpeed();
  header.acquisition_start = target_header->GetAcquisitionStart();
  const auto & targets = targetlist_port_umrra4_v1_0_1->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrra4_v1_2_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_automotive_v1_2_1::comtargetlist::ComTargetList> &
  targetlist_port_umrra4_v1_2_1,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra4_automotive_v1_2_1::comtargetlist::PortHeader> port_header;
  port_header = targetlist_port_umrra4_v1_2_1->GetPortHeader();
  std::shared_ptr<com::master::umrra4_automotive_v1_2_1::comtargetlist::TargetListHeader>
  target_header;
  target_header = targetlist_port_umrra4_v1_2_1->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();

  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.prf = target_header->GetPrf();
  header.umambiguous_speed = target_header->GetUmambiguousSpeed();
  header.acquisition_start = target_header->GetAcquisitionStart();
  const auto & targets = targetlist_port_umrra4_v1_2_1->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrra4_v1_4_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_automotive_v1_4_0::comtargetlist::ComTargetList> &
  targetlist_port_umrra4_v1_4_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra4_automotive_v1_4_0::comtargetlist::PortHeader> port_header;
  port_header = targetlist_port_umrra4_v1_4_0->GetPortHeader();
  std::shared_ptr<com::master::umrra4_automotive_v1_4_0::comtargetlist::TargetListHeader>
  target_header;
  target_header = targetlist_port_umrra4_v1_4_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();

  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.prf = target_header->GetPrf();
  header.umambiguous_speed = target_header->GetUmambiguousSpeed();
  header.acquisition_start = target_header->GetAcquisitionStart();
  const auto & targets = targetlist_port_umrra4_v1_4_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrra4_v1_6_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_automotive_v1_6_0::comtargetlist::ComTargetList> &
  targetlist_port_umrra4_v1_6_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra4_automotive_v1_6_0::comtargetlist::PortHeader>
  port_header;
  port_header = targetlist_port_umrra4_v1_6_0->GetPortHeader();
  std::shared_ptr<com::master::umrra4_automotive_v1_6_0::comtargetlist::TargetListHeader>
  target_header;
  target_header = targetlist_port_umrra4_v1_6_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();

  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.prf = target_header->GetPrf();
  header.umambiguous_speed = target_header->GetUmambiguousSpeed();
  header.acquisition_start = target_header->GetAcquisitionStart();
  header.acquisition_time_stamp_base = target_header->GetAcquisitionTimestampBase();

  const auto & targets = targetlist_port_umrra4_v1_6_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }
  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::faultreport_callback_umrra4_v1_6_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrra4_automotive_v1_6_0::faultreports::FaultReports> &
  fault_report_umrra4_v1_6_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra4_automotive_v1_6_0::faultreports::PortHeader> port_header;
  std::shared_ptr<com::master::umrra4_automotive_v1_6_0::faultreports::FaultReportHeader>
  fault_header;
  port_header = fault_report_umrra4_v1_6_0->GetPortHeader();
  fault_header = fault_report_umrra4_v1_6_0->GetFaultReportHeader();
  auto msg_ptr = std::make_unique<umrr_ros2_msgs::msg::PortFaultReportsMsg>();
  auto & msg = *msg_ptr;

  fill_ros_header_stamp(
    msg,
    port_header->GetTimestamp(),
    sensor_idx
  );

  msg.fault_report_header.port_identifier = port_header->GetPortIdentifier();
  msg.fault_report_header.port_ver_major = port_header->GetPortVersionMajor();
  msg.fault_report_header.port_ver_minor = port_header->GetPortVersionMinor();
  msg.fault_report_header.port_size = port_header->GetPortSize();
  msg.fault_report_header.body_endianness = port_header->GetBodyEndianness();
  msg.fault_report_header.port_index = port_header->GetPortIndex();
  msg.fault_report_header.header_ver_major = port_header->GetHeaderVersionMajor();
  msg.fault_report_header.header_ver_minor = port_header->GetHeaderVersionMinor();

  msg.fault_report_header.num_max_reports = fault_header->GetNumMaxReports();
  msg.fault_report_header.num_valid_reports = fault_header->GetNumValidReports();
  msg.fault_report_header.faults_time_line = fault_header->GetFaultsTimeline();

  const auto & fault_list = fault_report_umrra4_v1_6_0->GetFaultReportList();

  msg.reports.reserve(fault_list.size());

  for (const auto & fault : fault_list) {
    if (!fault) {
      continue;
    }

    umrr_ros2_msgs::msg::PortFaultReport report_msg;

    report_msg.module_id = fault->GetModuleId();
    report_msg.fault_group = fault->GetFaultGroup();
    report_msg.fault_code = fault->GetFaultCode();
    report_msg.fault_errno = fault->GetFaultErrno();
    report_msg.fault_time_stamp = fault->GetFaultTimestamp();
    report_msg.cycle_count = fault->GetCycleCount();
    report_msg.instance_id = fault->GetInstanceId();
    report_msg.criticality = fault->GetCriticality();
    report_msg.occurence_count = fault->GetOccurrenceCount();

    msg.reports.push_back(report_msg);
  }

  m_publishers_fault_report_msg[sensor_idx]->publish(std::move(msg_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrra1_v1_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrra1_t166_b_automotive_v1_0_0::comtargetlist::ComTargetList> &
  targetlist_port_umrra1_v1_0_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra1_t166_b_automotive_v1_0_0::comtargetlist::PortHeader>
  port_header;
  port_header = targetlist_port_umrra1_v1_0_0->GetPortHeader();
  std::shared_ptr<com::master::umrra1_t166_b_automotive_v1_0_0::comtargetlist::TargetListHeader>
  target_header;
  target_header = targetlist_port_umrra1_v1_0_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();

  header.cycle_time = target_header->GetCycleTime();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.prf = target_header->GetPrf();
  header.umambiguous_speed = target_header->GetUmambiguousSpeed();
  header.acquisition_start = target_header->GetAcquisitionStart();
  const auto & targets = targetlist_port_umrra1_v1_0_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrra1_v2_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrra1_t166_b_automotive_v2_0_0::comtargetlist::ComTargetList> &
  targetlist_port_umrra1_v2_0_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra1_t166_b_automotive_v2_0_0::comtargetlist::PortHeader>
  port_header;
  port_header = targetlist_port_umrra1_v2_0_0->GetPortHeader();
  std::shared_ptr<com::master::umrra1_t166_b_automotive_v2_0_0::comtargetlist::TargetListHeader>
  target_header;
  target_header = targetlist_port_umrra1_v2_0_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.cycle_time = target_header->GetCycleTime();
  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();

  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.prf = target_header->GetPrf();
  header.umambiguous_speed = target_header->GetUmambiguousSpeed();
  header.acquisition_start = target_header->GetAcquisitionStart();
  const auto & targets = targetlist_port_umrra1_v2_0_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrra1_v2_0_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrra1_t166_b_automotive_v2_0_1::comtargetlist::ComTargetList> &
  targetlist_port_umrra1_v2_0_1,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra1_t166_b_automotive_v2_0_1::comtargetlist::PortHeader>
  port_header;
  port_header = targetlist_port_umrra1_v2_0_1->GetPortHeader();
  std::shared_ptr<com::master::umrra1_t166_b_automotive_v2_0_1::comtargetlist::TargetListHeader>
  target_header;
  target_header = targetlist_port_umrra1_v2_0_1->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.cycle_time = target_header->GetCycleTime();
  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();

  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.prf = target_header->GetPrf();
  header.umambiguous_speed = target_header->GetUmambiguousSpeed();
  header.acquisition_start = target_header->GetAcquisitionStart();
  const auto & targets = targetlist_port_umrra1_v2_0_1->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::targetlist_callback_umrra1_v3_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrra1_t166_b_automotive_v3_0_0::comtargetlist::ComTargetList> &
  targetlist_port_umrra1_v3_0_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra1_t166_b_automotive_v3_0_0::comtargetlist::PortHeader>
  port_header;
  port_header = targetlist_port_umrra1_v3_0_0->GetPortHeader();
  std::shared_ptr<com::master::umrra1_t166_b_automotive_v3_0_0::comtargetlist::TargetListHeader>
  target_header;
  target_header = targetlist_port_umrra1_v3_0_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.cycle_time = target_header->GetCycleTime();
  header.port_identifier = port_header->GetPortIdentifier();
  header.port_ver_major = port_header->GetPortVersionMajor();
  header.port_ver_minor = port_header->GetPortVersionMinor();
  header.port_size = port_header->GetPortSize();
  header.body_endianness = port_header->GetBodyEndianness();
  header.port_index = port_header->GetPortIndex();
  header.header_ver_major = port_header->GetHeaderVersionMajor();
  header.header_ver_minor = port_header->GetHeaderVersionMinor();

  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
  header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
  header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
  header.prf = target_header->GetPrf();
  header.umambiguous_speed = target_header->GetUmambiguousSpeed();
  header.acquisition_start = target_header->GetAcquisitionStart();
  const auto & targets = targetlist_port_umrra1_v3_0_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetPower() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
        target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        target->GetVarianceRange(), target->GetVarianceSpeed(),
        target->GetVarianceAzimuthAngle(), target->GetVarianceElevationAngle(),
        target->GetFalseAlarmProbability(), target->GetFlags(), target->GetPeakIdx()});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_objectlist_callback_umrra4_mse_v2_1_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_mse_v2_1_0::comobjectbaselist::ComObjectBaseList> &
  objectlist_can_umrra4_mse_v2_1_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra4_mse_v2_1_0::comobjectbaselist::PortHeader> port_header;
  port_header = objectlist_can_umrra4_mse_v2_1_0->GetPortHeader();
  std::shared_ptr<com::master::umrra4_mse_v2_1_0::comobjectbaselist::ComObjectBaseListHeader>
  object_header;
  object_header = objectlist_can_umrra4_mse_v2_1_0->GetComObjectBaseListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanObjectHeader>();
  auto & header = *header_ptr;
  ObjectCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.cycle_time = object_header->GetCycleDuration();
  header.cycle_count = object_header->GetCycleCount();
  header.number_of_objects = object_header->GetNoOfObjects();
  header.ego_speed = object_header->GetSpeed();
  header.ego_speed_quality = object_header->GetSpeedQuality();
  header.ego_yaw_rate = object_header->GetYawRate();
  header.ego_yaw_rate_quality = object_header->GetYawRateQuality();
  header.dyn_source = object_header->GetDynamicSource();
  const auto & objects = objectlist_can_umrra4_mse_v2_1_0->GetObjectList();
  modifier.reserve(objects.size());
  for (const auto & object : objects) {
    const auto x_pos = object->GetXPoint1();
    const auto y_pos = object->GetYPoint1();
    const auto z_pos = object->GetZPoint1();
    const auto speed_abs = object->GetSpeedAbs();
    const auto heading = object->GetHeadingDeg() * kDegreesToRadians;
    const auto length = object->GetObjectLen();
    const auto quality = object->GetQuality();
    const auto acceleration = object->GetAcceleration();
    const auto object_id = static_cast<int16_t>(object->GetObjectId());
    modifier.push_back(
      {x_pos, y_pos, z_pos, speed_abs, heading, length, kRadarFloatSentinel, quality,
        acceleration, object_id, kRadarU16Sentinel, kRadarU16Sentinel, kRadarU8Sentinel,
        kRadarU16Sentinel});
  }

  m_publishers_obj[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_obj_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrra4_mse_v2_1_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_mse_v2_1_0::comtargetbaselist::ComTargetBaseList> &
  targetlist_can_umrra4_mse_v2_1_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra4_mse_v2_1_0::comtargetbaselist::PortHeader> port_header;
  port_header = targetlist_can_umrra4_mse_v2_1_0->GetPortHeader();
  std::shared_ptr<com::master::umrra4_mse_v2_1_0::comtargetbaselist::TargetListHeader>
  target_header;
  target_header = targetlist_can_umrra4_mse_v2_1_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.cycle_time = target_header->GetCycleDuration();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.cycle_count = target_header->GetCycleCount();
  header.time_stamp = target_header->GetTimeStamp();
  header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
  const auto & targets = targetlist_can_umrra4_mse_v2_1_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetSignalLevel() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_objectlist_callback_umrra4_mse_v1_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_mse_v1_0_0::comobjectbaselist::ComObjectBaseList> &
  objectlist_can_umrra4_mse_v1_0_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra4_mse_v1_0_0::comobjectbaselist::PortHeader> port_header;
  port_header = objectlist_can_umrra4_mse_v1_0_0->GetPortHeader();
  std::shared_ptr<com::master::umrra4_mse_v1_0_0::comobjectbaselist::ComObjectBaseListHeader>
  object_header;
  object_header = objectlist_can_umrra4_mse_v1_0_0->GetComObjectBaseListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanObjectHeader>();
  auto & header = *header_ptr;
  ObjectCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.cycle_time = object_header->GetCycleDuration();
  header.cycle_count = object_header->GetCycleCount();
  header.number_of_objects = object_header->GetNoOfObjects();
  header.ego_speed = object_header->GetSpeed();
  header.ego_speed_quality = object_header->GetSpeedQuality();
  header.ego_yaw_rate = object_header->GetYawRate();
  header.ego_yaw_rate_quality = object_header->GetYawRateQuality();
  header.dyn_source = object_header->GetDynamicSource();
  const auto & objects = objectlist_can_umrra4_mse_v1_0_0->GetObjectList();
  modifier.reserve(objects.size());
  for (const auto & object : objects) {
    const auto x_pos = object->GetXPoint1();
    const auto y_pos = object->GetYPoint1();
    const auto z_pos = object->GetZPoint1();
    const auto speed_abs = object->GetSpeedAbs();
    const auto heading = object->GetHeadingDeg() * kDegreesToRadians;
    const auto length = object->GetObjectLen();
    const auto quality = object->GetQuality();
    const auto acceleration = object->GetAcceleration();
    const auto object_id = static_cast<int16_t>(object->GetObjectId());
    modifier.push_back(
      {x_pos, y_pos, z_pos, speed_abs, heading, length, kRadarFloatSentinel, quality,
        acceleration, object_id, kRadarU16Sentinel, kRadarU16Sentinel, kRadarU8Sentinel,
        kRadarU16Sentinel});
  }

  m_publishers_obj[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_obj_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrra4_mse_v1_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_mse_v1_0_0::comtargetbaselist::ComTargetBaseList> &
  targetlist_can_umrra4_mse_v1_0_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra4_mse_v1_0_0::comtargetbaselist::PortHeader> port_header;
  port_header = targetlist_can_umrra4_mse_v1_0_0->GetPortHeader();
  std::shared_ptr<com::master::umrra4_mse_v1_0_0::comtargetbaselist::TargetListHeader>
  target_header;
  target_header = targetlist_can_umrra4_mse_v1_0_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.cycle_time = target_header->GetCycleDuration();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.cycle_count = target_header->GetCycleCount();
  header.time_stamp = target_header->GetTimeStamp();
  header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
  const auto & targets = targetlist_can_umrra4_mse_v1_0_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetSignalLevel() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_objectlist_callback_umrr9f_mse_v1_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v1_0_0::comobjectbaselist::ComObjectBaseList> &
  objectlist_can_umrr9f_mse_v1_0_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_mse_v1_0_0::comobjectbaselist::PortHeader> port_header;
  port_header = objectlist_can_umrr9f_mse_v1_0_0->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_mse_v1_0_0::comobjectbaselist::ComObjectBaseListHeader>
  object_header;
  object_header = objectlist_can_umrr9f_mse_v1_0_0->GetComObjectBaseListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanObjectHeader>();
  auto & header = *header_ptr;
  ObjectCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.cycle_time = object_header->GetCycleDuration();
  header.cycle_count = object_header->GetCycleCount();
  header.number_of_objects = object_header->GetNoOfObjects();
  header.ego_speed = object_header->GetSpeed();
  header.ego_speed_quality = object_header->GetSpeedQuality();
  header.ego_yaw_rate = object_header->GetYawRate();
  header.ego_yaw_rate_quality = object_header->GetYawRateQuality();
  header.dyn_source = object_header->GetDynamicSource();
  const auto & objects = objectlist_can_umrr9f_mse_v1_0_0->GetObjectList();
  modifier.reserve(objects.size());
  for (const auto & object : objects) {
    const auto x_pos = object->GetXPoint1();
    const auto y_pos = object->GetYPoint1();
    const auto z_pos = object->GetZPoint1();
    const auto speed_abs = object->GetSpeedAbs();
    const auto heading = object->GetHeadingDeg() * kDegreesToRadians;
    const auto length = object->GetObjectLen();
    const auto quality = object->GetQuality();
    const auto acceleration = object->GetAcceleration();
    const auto object_id = static_cast<int16_t>(object->GetObjectId());
    modifier.push_back(
      {x_pos, y_pos, z_pos, speed_abs, heading, length, kRadarFloatSentinel, quality,
        acceleration, object_id, kRadarU16Sentinel, kRadarU16Sentinel, kRadarU8Sentinel,
        kRadarU16Sentinel});
  }

  m_publishers_obj[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_obj_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_mse_v1_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v1_0_0::comtargetbaselist::ComTargetBaseList> &
  targetlist_can_umrr9f_mse_v1_0_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_mse_v1_0_0::comtargetbaselist::PortHeader> port_header;
  port_header = targetlist_can_umrr9f_mse_v1_0_0->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_mse_v1_0_0::comtargetbaselist::TargetListHeader>
  target_header;
  target_header = targetlist_can_umrr9f_mse_v1_0_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.cycle_time = target_header->GetCycleDuration();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.cycle_count = target_header->GetCycleCount();
  header.time_stamp = target_header->GetTimeStamp();
  header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
  const auto & targets = targetlist_can_umrr9f_mse_v1_0_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetSignalLevel() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_objectlist_callback_umrr9f_mse_v1_1_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v1_1_0::comobjectbaselist::ComObjectBaseList> &
  objectlist_can_umrr9f_mse_v1_1_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_mse_v1_1_0::comobjectbaselist::PortHeader> port_header;
  port_header = objectlist_can_umrr9f_mse_v1_1_0->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_mse_v1_1_0::comobjectbaselist::ComObjectBaseListHeader>
  object_header;
  object_header = objectlist_can_umrr9f_mse_v1_1_0->GetComObjectBaseListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanObjectHeader>();
  auto & header = *header_ptr;
  ObjectCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};


  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.cycle_time = object_header->GetCycleDuration();
  header.cycle_count = object_header->GetCycleCount();
  header.number_of_objects = object_header->GetNoOfObjects();
  header.ego_speed = object_header->GetSpeed();
  header.ego_speed_quality = object_header->GetSpeedQuality();
  header.ego_yaw_rate = object_header->GetYawRate();
  header.ego_yaw_rate_quality = object_header->GetYawRateQuality();
  header.dyn_source = object_header->GetDynamicSource();
  const auto & objects = objectlist_can_umrr9f_mse_v1_1_0->GetObjectList();
  modifier.reserve(objects.size());
  for (const auto & object : objects) {
    const auto x_pos = object->GetXPoint1();
    const auto y_pos = object->GetYPoint1();
    const auto z_pos = object->GetZPoint1();
    const auto speed_abs = object->GetSpeedAbs();
    const auto heading = object->GetHeadingDeg() * kDegreesToRadians;
    const auto length = object->GetObjectLen();
    const auto quality = object->GetQuality();
    const auto acceleration = object->GetAcceleration();
    const auto object_id = static_cast<int16_t>(object->GetObjectId());
    modifier.push_back(
      {x_pos, y_pos, z_pos, speed_abs, heading, length, kRadarFloatSentinel, quality,
        acceleration, object_id, kRadarU16Sentinel, kRadarU16Sentinel, kRadarU8Sentinel,
        kRadarU16Sentinel});
  }

  m_publishers_obj[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_obj_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_mse_v1_1_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v1_1_0::comtargetbaselist::ComTargetBaseList> &
  targetlist_can_umrr9f_mse_v1_1_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_mse_v1_1_0::comtargetbaselist::PortHeader> port_header;
  port_header = targetlist_can_umrr9f_mse_v1_1_0->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_mse_v1_1_0::comtargetbaselist::TargetListHeader>
  target_header;
  target_header = targetlist_can_umrr9f_mse_v1_1_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.cycle_time = target_header->GetCycleDuration();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.cycle_count = target_header->GetCycleCount();
  header.time_stamp = target_header->GetTimeStamp();
  header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
  const auto & targets = targetlist_can_umrr9f_mse_v1_1_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetSignalLevel() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_objectlist_callback_umrr9f_mse_v1_3_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v1_3_0::comobjectbaselist::ComObjectBaseList> &
  objectlist_can_umrr9f_mse_v1_3_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_mse_v1_3_0::comobjectbaselist::PortHeader> port_header;
  port_header = objectlist_can_umrr9f_mse_v1_3_0->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_mse_v1_3_0::comobjectbaselist::ComObjectBaseListHeader>
  object_header;
  object_header = objectlist_can_umrr9f_mse_v1_3_0->GetComObjectBaseListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanObjectHeader>();
  auto & header = *header_ptr;
  ObjectCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.cycle_time = object_header->GetCycleDuration();
  header.cycle_count = object_header->GetCycleCount();
  header.number_of_objects = object_header->GetNoOfObjects();
  header.ego_speed = object_header->GetSpeed();
  header.ego_speed_quality = object_header->GetSpeedQuality();
  header.ego_yaw_rate = object_header->GetYawRate();
  header.ego_yaw_rate_quality = object_header->GetYawRateQuality();
  header.dyn_source = object_header->GetDynamicSource();
  const auto & objects = objectlist_can_umrr9f_mse_v1_3_0->GetObjectList();
  modifier.reserve(objects.size());
  for (const auto & object : objects) {
    const auto x_pos = object->GetXPoint1();
    const auto y_pos = object->GetYPoint1();
    const auto z_pos = object->GetZPoint1();
    const auto speed_abs = object->GetSpeedAbs();
    const auto heading = object->GetHeadingDeg() * kDegreesToRadians;
    const auto length = object->GetObjectLen();
    const auto quality = object->GetQuality();
    const auto acceleration = object->GetAcceleration();
    const auto object_id = static_cast<int16_t>(object->GetObjectId());
    modifier.push_back(
      {x_pos, y_pos, z_pos, speed_abs, heading, length, kRadarFloatSentinel, quality,
        acceleration, object_id, kRadarU16Sentinel, kRadarU16Sentinel, kRadarU8Sentinel,
        kRadarU16Sentinel});
  }

  m_publishers_obj[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_obj_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_mse_v1_3_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v1_3_0::comtargetbaselist::ComTargetBaseList> &
  targetlist_can_umrr9f_mse_v1_3_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_mse_v1_3_0::comtargetbaselist::PortHeader> port_header;
  port_header = targetlist_can_umrr9f_mse_v1_3_0->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_mse_v1_3_0::comtargetbaselist::TargetListHeader>
  target_header;
  target_header = targetlist_can_umrr9f_mse_v1_3_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.cycle_time = target_header->GetCycleDuration();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.cycle_count = target_header->GetCycleCount();
  header.time_stamp = target_header->GetTimeStamp();
  header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
  const auto & targets = targetlist_can_umrr9f_mse_v1_3_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetSignalLevel() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr96(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr96_t153_automotive_v1_2_2::comtargetbaselist::ComTargetBaseList> &
  targetlist_can_umrr96,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr96_t153_automotive_v1_2_2::comtargetbaselist::PortHeader>
  port_header;
  port_header = targetlist_can_umrr96->GetPortHeader();
  std::shared_ptr<com::master::umrr96_t153_automotive_v1_2_2::comtargetbaselist::TargetListHeader>
  target_header;
  target_header = targetlist_can_umrr96->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
  header.time_stamp = target_header->GetTimeStamp();
  header.cycle_time = target_header->GetCycleDuration();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.cycle_count = target_header->GetCycleCount();
  const auto & targets = targetlist_can_umrr96->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetSignalLevel() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr11(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr11_t132_automotive_v1_1_2::comtargetbaselist::ComTargetBaseList> &
  targetlist_can_umrr11,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr11_t132_automotive_v1_1_2::comtargetbaselist::PortHeader>
  port_header;
  port_header = targetlist_can_umrr11->GetPortHeader();
  std::shared_ptr<com::master::umrr11_t132_automotive_v1_1_2::comtargetbaselist::TargetListHeader>
  target_header;
  target_header = targetlist_can_umrr11->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
  header.time_stamp = target_header->GetTimeStamp();
  header.cycle_time = target_header->GetCycleDuration();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.cycle_count = target_header->GetCycleCount();
  const auto & targets = targetlist_can_umrr11->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetSignalLevel() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9d_v1_0_3(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9d_t152_automotive_v1_0_3::comtargetbaselist::ComTargetBaseList> &
  targetlist_can_umrr9d_v1_0_3,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9d_t152_automotive_v1_0_3::comtargetbaselist::PortHeader>
  port_header;
  port_header = targetlist_can_umrr9d_v1_0_3->GetPortHeader();
  std::shared_ptr<com::master::umrr9d_t152_automotive_v1_0_3::comtargetbaselist::TargetListHeader>
  target_header;
  target_header = targetlist_can_umrr9d_v1_0_3->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
  header.time_stamp = target_header->GetTimeStamp();
  header.cycle_time = target_header->GetCycleDuration();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.cycle_count = target_header->GetCycleCount();
  const auto & targets = targetlist_can_umrr9d_v1_0_3->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetSignalLevel() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9d_v1_2_2(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9d_t152_automotive_v1_2_2::comtargetbaselist::ComTargetBaseList> &
  targetlist_can_umrr9d_v1_2_2,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9d_t152_automotive_v1_2_2::comtargetbaselist::PortHeader>
  port_header;
  port_header = targetlist_can_umrr9d_v1_2_2->GetPortHeader();
  std::shared_ptr<com::master::umrr9d_t152_automotive_v1_2_2::comtargetbaselist::TargetListHeader>
  target_header;
  target_header = targetlist_can_umrr9d_v1_2_2->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
  header.time_stamp = target_header->GetTimeStamp();
  header.cycle_time = target_header->GetCycleDuration();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.cycle_count = target_header->GetCycleCount();
  const auto & targets = targetlist_can_umrr9d_v1_2_2->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetSignalLevel() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9d_v1_4_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9d_t152_automotive_v1_4_1::comtargetbaselist::ComTargetBaseList> &
  targetlist_can_umrr9d_v1_4_1,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9d_t152_automotive_v1_4_1::comtargetbaselist::PortHeader>
  port_header;
  port_header = targetlist_can_umrr9d_v1_4_1->GetPortHeader();
  std::shared_ptr<com::master::umrr9d_t152_automotive_v1_4_1::comtargetbaselist::TargetListHeader>
  target_header;
  target_header = targetlist_can_umrr9d_v1_4_1->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );

  header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
  header.time_stamp = target_header->GetTimeStamp();
  header.cycle_time = target_header->GetCycleDuration();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.cycle_count = target_header->GetCycleCount();
  const auto & targets = targetlist_can_umrr9d_v1_4_1->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetSignalLevel() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9d_v1_5_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9d_t152_automotive_v1_5_0::comtargetbaselist::ComTargetBaseList> &
  targetlist_can_umrr9d_v1_5_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9d_t152_automotive_v1_5_0::comtargetbaselist::PortHeader>
  port_header;
  port_header = targetlist_can_umrr9d_v1_5_0->GetPortHeader();
  std::shared_ptr<com::master::umrr9d_t152_automotive_v1_5_0::comtargetbaselist::TargetListHeader>
  target_header;
  target_header = targetlist_can_umrr9d_v1_5_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );
  header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
  header.time_stamp = target_header->GetTimeStamp();
  header.cycle_time = target_header->GetCycleDuration();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.cycle_count = target_header->GetCycleCount();
  const auto & targets = targetlist_can_umrr9d_v1_5_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetSignalLevel() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_v2_1_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9f_t169_automotive_v2_1_1::comtargetbaselist::ComTargetBaseList> &
  targetlist_can_umrr9f_v2_1_1,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_automotive_v2_1_1::comtargetbaselist::PortHeader>
  port_header;
  port_header = targetlist_can_umrr9f_v2_1_1->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_automotive_v2_1_1::comtargetbaselist::TargetListHeader>
  target_header;
  target_header = targetlist_can_umrr9f_v2_1_1->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );    header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
  header.time_stamp = target_header->GetTimeStamp();
  header.cycle_time = target_header->GetCycleDuration();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.cycle_count = target_header->GetCycleCount();
  const auto & targets = targetlist_can_umrr9f_v2_1_1->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetSignalLevel() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_v2_2_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9f_t169_automotive_v2_2_1::comtargetbaselist::ComTargetBaseList> &
  targetlist_can_umrr9f_v2_2_1,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_automotive_v2_2_1::comtargetbaselist::PortHeader>
  port_header;
  port_header = targetlist_can_umrr9f_v2_2_1->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_automotive_v2_2_1::comtargetbaselist::TargetListHeader>
  target_header;
  target_header = targetlist_can_umrr9f_v2_2_1->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );    header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
  header.time_stamp = target_header->GetTimeStamp();
  header.cycle_time = target_header->GetCycleDuration();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.cycle_count = target_header->GetCycleCount();
  const auto & targets = targetlist_can_umrr9f_v2_2_1->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetSignalLevel() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_v2_4_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9f_t169_automotive_v2_4_1::comtargetbaselist::ComTargetBaseList> &
  targetlist_can_umrr9f_v2_4_1,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_automotive_v2_4_1::comtargetbaselist::PortHeader>
  port_header;
  port_header = targetlist_can_umrr9f_v2_4_1->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_automotive_v2_4_1::comtargetbaselist::TargetListHeader>
  target_header;
  target_header = targetlist_can_umrr9f_v2_4_1->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );    header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
  header.time_stamp = target_header->GetTimeStamp();
  header.cycle_time = target_header->GetCycleDuration();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.cycle_count = target_header->GetCycleCount();
  const auto & targets = targetlist_can_umrr9f_v2_4_1->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetSignalLevel() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_v3_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9f_t169_automotive_v3_0_0::comtargetbaselist::ComTargetBaseList> &
  targetlist_can_umrr9f_v3_0_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_automotive_v3_0_0::comtargetbaselist::PortHeader>
  port_header;
  port_header = targetlist_can_umrr9f_v3_0_0->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_automotive_v3_0_0::comtargetbaselist::TargetListHeader>
  target_header;
  target_header = targetlist_can_umrr9f_v3_0_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );    header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
  header.time_stamp = target_header->GetTimeStamp();
  header.cycle_time = target_header->GetCycleDuration();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.cycle_count = target_header->GetCycleCount();
  const auto & targets = targetlist_can_umrr9f_v3_0_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetSignalLevel() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrra4_v1_0_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrra4_automotive_v1_0_1::comtargetbaselist::ComTargetBaseList> &
  targetlist_can_umrra4_v1_0_1,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra4_automotive_v1_0_1::comtargetbaselist::PortHeader>
  port_header;
  port_header = targetlist_can_umrra4_v1_0_1->GetPortHeader();
  std::shared_ptr<com::master::umrra4_automotive_v1_0_1::comtargetbaselist::TargetListHeader>
  target_header;
  target_header = targetlist_can_umrra4_v1_0_1->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );    header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
  header.time_stamp = target_header->GetTimeStamp();
  header.cycle_time = target_header->GetCycleDuration();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.cycle_count = target_header->GetCycleCount();
  const auto & targets = targetlist_can_umrra4_v1_0_1->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetSignalLevel() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrra4_v1_2_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrra4_automotive_v1_2_1::comtargetbaselist::ComTargetBaseList> &
  targetlist_can_umrra4_v1_2_1,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra4_automotive_v1_2_1::comtargetbaselist::PortHeader>
  port_header;
  port_header = targetlist_can_umrra4_v1_2_1->GetPortHeader();
  std::shared_ptr<com::master::umrra4_automotive_v1_2_1::comtargetbaselist::TargetListHeader>
  target_header;
  target_header = targetlist_can_umrra4_v1_2_1->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );    header.cycle_time = target_header->GetCycleDuration();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.cycle_count = target_header->GetCycleCount();
  header.time_stamp = target_header->GetTimeStamp();
  header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
  const auto & targets = targetlist_can_umrra4_v1_2_1->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetSignalLevel() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrra4_v1_4_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrra4_automotive_v1_4_0::comtargetbaselist::ComTargetBaseList> &
  targetlist_can_umrra4_v1_4_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra4_automotive_v1_4_0::comtargetbaselist::PortHeader>
  port_header;
  port_header = targetlist_can_umrra4_v1_4_0->GetPortHeader();
  std::shared_ptr<com::master::umrra4_automotive_v1_4_0::comtargetbaselist::TargetListHeader>
  target_header;
  target_header = targetlist_can_umrra4_v1_4_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(
    msg,
    header,
    port_header->GetTimestamp(),
    sensor_idx
  );    header.cycle_time = target_header->GetCycleDuration();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.cycle_count = target_header->GetCycleCount();
  header.time_stamp = target_header->GetTimeStamp();
  header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
  const auto & targets = targetlist_can_umrra4_v1_4_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetSignalLevel() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_v3_2_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9f_t169_automotive_v3_2_0::comtargetbaselist::ComTargetBaseList> &
  targetlist_can_umrr9f_v3_2_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_automotive_v3_2_0::comtargetbaselist::PortHeader>
  port_header;
  port_header = targetlist_can_umrr9f_v3_2_0->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_automotive_v3_2_0::comtargetbaselist::TargetListHeader>
  target_header;
  target_header = targetlist_can_umrr9f_v3_2_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(msg, header, port_header->GetTimestamp(), sensor_idx);
  header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
  header.time_stamp = target_header->GetTimeStamp();
  header.cycle_time = target_header->GetCycleDuration();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.cycle_count = target_header->GetCycleCount();
  const auto & targets = targetlist_can_umrr9f_v3_2_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetSignalLevel() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel, kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9d_v1_7_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9d_t152_automotive_v1_7_0::comtargetbaselist::ComTargetBaseList> &
  targetlist_can_umrr9d_v1_7_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9d_t152_automotive_v1_7_0::comtargetbaselist::PortHeader>
  port_header;
  port_header = targetlist_can_umrr9d_v1_7_0->GetPortHeader();
  std::shared_ptr<com::master::umrr9d_t152_automotive_v1_7_0::comtargetbaselist::TargetListHeader>
  target_header;
  target_header = targetlist_can_umrr9d_v1_7_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(msg, header, port_header->GetTimestamp(), sensor_idx);
  header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
  header.time_stamp = target_header->GetTimeStamp();
  header.cycle_time = target_header->GetCycleDuration();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.cycle_count = target_header->GetCycleCount();
  const auto & targets = targetlist_can_umrr9d_v1_7_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetSignalLevel() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel, kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrra4_v1_6_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrra4_automotive_v1_6_0::comtargetbaselist::ComTargetBaseList> &
  targetlist_can_umrra4_v1_6_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra4_automotive_v1_6_0::comtargetbaselist::PortHeader>
  port_header;
  port_header = targetlist_can_umrra4_v1_6_0->GetPortHeader();
  std::shared_ptr<com::master::umrra4_automotive_v1_6_0::comtargetbaselist::TargetListHeader>
  target_header;
  target_header = targetlist_can_umrra4_v1_6_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(msg, header, port_header->GetTimestamp(), sensor_idx);
  header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
  header.time_stamp = target_header->GetTimeStamp();
  header.cycle_time = target_header->GetCycleDuration();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.cycle_count = target_header->GetCycleCount();
  const auto & targets = targetlist_can_umrra4_v1_6_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetSignalLevel() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel, kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_objectlist_callback_umrra4_mse_v3_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_mse_v3_0_0::comobjectbaselist::ComObjectBaseList> &
  objectlist_can_umrra4_mse_v3_0_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra4_mse_v3_0_0::comobjectbaselist::PortHeader> port_header;
  port_header = objectlist_can_umrra4_mse_v3_0_0->GetPortHeader();
  std::shared_ptr<com::master::umrra4_mse_v3_0_0::comobjectbaselist::ComObjectBaseListHeader>
  object_header;
  object_header = objectlist_can_umrra4_mse_v3_0_0->GetComObjectBaseListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanObjectHeader>();
  auto & header = *header_ptr;
  ObjectCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(msg, header, port_header->GetTimestamp(), sensor_idx);

  header.cycle_time = object_header->GetCycleDuration();
  header.cycle_count = object_header->GetCycleCount();
  header.number_of_objects = object_header->GetNoOfObjects();
  header.ego_speed = object_header->GetSpeed();
  header.ego_speed_quality = object_header->GetSpeedQuality();
  header.ego_yaw_rate = object_header->GetYawRate();
  header.ego_yaw_rate_quality = object_header->GetYawRateQuality();
  header.dyn_source = object_header->GetDynamicSource();
  const auto & objects = objectlist_can_umrra4_mse_v3_0_0->GetObjectList();
  modifier.reserve(objects.size());
  for (const auto & object : objects) {
    const auto x_pos = object->GetXPoint1();
    const auto y_pos = object->GetYPoint1();
    const auto z_pos = object->GetZPoint1();
    const auto speed_abs = object->GetSpeedAbs();
    const auto heading = object->GetHeadingDeg() * kDegreesToRadians;
    const auto length = object->GetObjectLen();
    const auto quality = object->GetQuality();
    const auto acceleration = object->GetAcceleration();
    const auto object_id = static_cast<int16_t>(object->GetObjectId());
    modifier.push_back(
      {x_pos, y_pos, z_pos, speed_abs, heading, length, kRadarFloatSentinel, quality,
        acceleration, object_id, kRadarU16Sentinel, kRadarU16Sentinel, kRadarU8Sentinel,
        kRadarU16Sentinel});
  }

  m_publishers_obj[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_obj_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrra4_mse_v3_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_mse_v3_0_0::comtargetbaselist::ComTargetBaseList> &
  targetlist_can_umrra4_mse_v3_0_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrra4_mse_v3_0_0::comtargetbaselist::PortHeader> port_header;
  port_header = targetlist_can_umrra4_mse_v3_0_0->GetPortHeader();
  std::shared_ptr<com::master::umrra4_mse_v3_0_0::comtargetbaselist::TargetListHeader>
  target_header;
  target_header = targetlist_can_umrra4_mse_v3_0_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(msg, header, port_header->GetTimestamp(), sensor_idx);

  header.cycle_time = target_header->GetCycleDuration();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.cycle_count = target_header->GetCycleCount();
  header.time_stamp = target_header->GetTimeStamp();
  header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
  const auto & targets = targetlist_can_umrra4_mse_v3_0_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetSignalLevel() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel, kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_objectlist_callback_umrr9f_mse_v2_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v2_0_0::comobjectbaselist::ComObjectBaseList> &
  objectlist_can_umrr9f_mse_v2_0_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_mse_v2_0_0::comobjectbaselist::PortHeader> port_header;
  port_header = objectlist_can_umrr9f_mse_v2_0_0->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_mse_v2_0_0::comobjectbaselist::ComObjectBaseListHeader>
  object_header;
  object_header = objectlist_can_umrr9f_mse_v2_0_0->GetComObjectBaseListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanObjectHeader>();
  auto & header = *header_ptr;
  ObjectCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(msg, header, port_header->GetTimestamp(), sensor_idx);

  header.cycle_time = object_header->GetCycleDuration();
  header.cycle_count = object_header->GetCycleCount();
  header.number_of_objects = object_header->GetNoOfObjects();
  header.ego_speed = object_header->GetSpeed();
  header.ego_speed_quality = object_header->GetSpeedQuality();
  header.ego_yaw_rate = object_header->GetYawRate();
  header.ego_yaw_rate_quality = object_header->GetYawRateQuality();
  header.dyn_source = object_header->GetDynamicSource();
  const auto & objects = objectlist_can_umrr9f_mse_v2_0_0->GetObjectList();
  modifier.reserve(objects.size());
  for (const auto & object : objects) {
    const auto x_pos = object->GetXPoint1();
    const auto y_pos = object->GetYPoint1();
    const auto z_pos = object->GetZPoint1();
    const auto speed_abs = object->GetSpeedAbs();
    const auto heading = object->GetHeadingDeg() * kDegreesToRadians;
    const auto length = object->GetObjectLen();
    const auto quality = object->GetQuality();
    const auto acceleration = object->GetAcceleration();
    const auto object_id = static_cast<int16_t>(object->GetObjectId());
    modifier.push_back(
      {x_pos, y_pos, z_pos, speed_abs, heading, length, kRadarFloatSentinel, quality,
        acceleration, object_id, kRadarU16Sentinel, kRadarU16Sentinel, kRadarU8Sentinel,
        kRadarU16Sentinel});
  }

  m_publishers_obj[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_obj_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_mse_v2_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v2_0_0::comtargetbaselist::ComTargetBaseList> &
  targetlist_can_umrr9f_mse_v2_0_0,
  const com::types::ClientId client_id)
{
  std::shared_ptr<com::master::umrr9f_t169_mse_v2_0_0::comtargetbaselist::PortHeader> port_header;
  port_header = targetlist_can_umrr9f_mse_v2_0_0->GetPortHeader();
  std::shared_ptr<com::master::umrr9f_t169_mse_v2_0_0::comtargetbaselist::TargetListHeader>
  target_header;
  target_header = targetlist_can_umrr9f_mse_v2_0_0->GetTargetListHeader();
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};

  fill_ros_header_stamp(msg, header, port_header->GetTimestamp(), sensor_idx);

  header.cycle_time = target_header->GetCycleDuration();
  header.number_of_targets = target_header->GetNumberOfTargets();
  header.acquisition_setup = target_header->GetAcquisitionSetup();
  header.cycle_count = target_header->GetCycleCount();
  header.time_stamp = target_header->GetTimeStamp();
  header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
  const auto & targets = targetlist_can_umrr9f_mse_v2_0_0->GetTargetList();
  modifier.reserve(targets.size());
  for (const auto & target : targets) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    const auto snr = target->GetSignalLevel() - target->GetNoise();
    modifier.push_back(
      {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
        range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
        target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range,
        kRadarFloatSentinel, kRadarFloatSentinel, kRadarFloatSentinel, kRadarFloatSentinel,
        kRadarFloatSentinel, kRadarFlagsSentinel, kRadarPeakIdxSentinel});
  }

  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_target_header[sensor_idx]->publish(std::move(header_ptr));
}

void SmartmicroRadarNode::update_config_files_from_params()
{
  const auto master_inst_serial_type = startup_parameter(*this, kInstSerialTypeTag, std::string{});
  const auto master_data_serial_type = startup_parameter(*this, kDataSerialTypeTag, std::string{});

  auto read_adapter_params_if_possible = [&](const std::uint32_t index) {
      auto & current_adapter = m_adapters[index];
      const auto prefix_2 = "adapters.adapter_" + std::to_string(index);
      current_adapter.hw_dev_id = startup_parameter(*this, prefix_2 + ".hw_dev_id", kDefaultHwDevId);
      if (current_adapter.hw_dev_id == kDefaultHwDevId) {
        // The id was not set, so the adapter with this index was not defined.
        // Stop here.
        return false;
      }
      current_adapter.hw_iface_name =
        startup_parameter(*this, prefix_2 + ".hw_iface_name", kDefaultHwDevIface);
      current_adapter.hw_ip_address =
        startup_parameter(*this, prefix_2 + ".hw_ip_address", std::string{});
      current_adapter.hw_type = startup_parameter(*this, prefix_2 + ".hw_type", kDefaultHwLinkType);
      current_adapter.baudrate = startup_parameter(*this, prefix_2 + ".baudrate", 500000);
      current_adapter.port = startup_parameter(*this, prefix_2 + ".port", kDefaultPort, 0, 65535);

      if (current_adapter.port > 65535 ||
        (current_adapter.hw_type == "eth" && current_adapter.port == 0))
      {
        throw std::invalid_argument(prefix_2 + ".port must be a valid UDP port for Ethernet");
      }
      return true;
    };

  auto read_sensor_params_if_possible = [&](const std::uint32_t index) {
      auto & sensor = m_sensors[index];
      const auto prefix_3 = "sensors.sensor_" + std::to_string(index);
      sensor.dev_id = startup_parameter(*this, prefix_3 + ".dev_id", kDefaultHwDevId);
      sensor.uifname = startup_parameter(*this, prefix_3 + ".uifname", "");
      sensor.uifmajorv = startup_parameter(*this, prefix_3 + ".uifmajorv", 0);
      sensor.uifminorv = startup_parameter(*this, prefix_3 + ".uifminorv", 0);
      sensor.uifpatchv = startup_parameter(*this, prefix_3 + ".uifpatchv", 0);
      sensor.model = startup_parameter(*this, prefix_3 + ".model", kDefaultSensorType);
      sensor.id = startup_parameter(*this, prefix_3 + ".id", kDefaultClientId);
      if (sensor.id == kDefaultClientId) {
        // The id was not set, so the sensor with this index was not defined. Stop
        // here.
        return false;
      }
      sensor.ip = startup_parameter(*this, prefix_3 + ".ip", "");
      sensor.port = startup_parameter(*this, prefix_3 + ".port", 0, 0, 65535);
      sensor.frame_id = startup_parameter(*this, prefix_3 + ".frame_id", kDefaultFrameId);
      sensor.history_size =
        startup_parameter(*this, prefix_3 + ".history_size", kDefaultHistorySize, 1, UINT32_MAX);
      sensor.inst_type = startup_parameter(*this, prefix_3 + ".inst_type", "");
      sensor.data_type = startup_parameter(*this, prefix_3 + ".data_type", "");
      sensor.link_type = startup_parameter(*this, prefix_3 + ".link_type", kDefaultHwLinkType);
      sensor.pub_type = startup_parameter(*this, prefix_3 + ".pub_type", "");
      validate_sensor_config(prefix_3, sensor.link_type, sensor.model, sensor.pub_type);
      if (sensor.port > 65535 || (sensor.link_type == "eth" && sensor.port == 0) ||
        sensor.history_size == 0 || sensor.frame_id.empty())
      {
        throw std::invalid_argument(prefix_3 + ": invalid port, empty frame_id or zero history_size");
      }
      bool adapter_found = false;
      for (size_t i = 0; i < m_number_of_adapters; ++i) {
        adapter_found |= m_adapters[i].hw_dev_id == sensor.dev_id;
      }
      if (!adapter_found) {
        throw std::invalid_argument(prefix_3 + ".dev_id does not identify a configured adapter");
      }
      return true;
    };

  for (auto j = 0UL; j < m_adapters.size(); ++j) {
    if (!read_adapter_params_if_possible(j)) {
      break;
    }
    ++m_number_of_adapters;
  }

  if (!m_number_of_adapters) {
    throw std::runtime_error("At least one adapter must be configured.");
  }

  for (auto i = 0UL; i < m_sensors.size(); ++i) {
    if (!read_sensor_params_if_possible(i)) {
      break;
    }
    ++m_number_of_sensors;
  }
  if (!m_number_of_sensors) {
    throw std::runtime_error("At least one sensor must be configured.");
  }

  auto config = nlohmann::json::parse(std::ifstream{kConfigFilePath});
  config[kDataSerialTypeJsonTag] = master_data_serial_type;
  config[kInstSerialTypeJsonTag] = master_inst_serial_type;
  config["config_path"] = runtime_config_.path.string();
  runtime_config_.write("smart_access_config.json", config);

  auto hw_inventory = nlohmann::json::parse(std::ifstream{kHwInventoryFilePath});
  auto & hw_items = hw_inventory[kHwItemsJsonTag];
  if (hw_items.empty()) {
    throw std::runtime_error("There are no 'hwItems' defined in the hw_inventory.json file.");
  }
  auto hw_item = hw_items.front();
  hw_items.clear();
  for (auto j = 0UL; j < m_number_of_adapters; ++j) {
    const auto & adapter = m_adapters[j];
    hw_item[kPortTag] = adapter.port;
    hw_item[kHwDevLinkTag] = adapter.hw_type;
    hw_item[kHwDevIdTag] = adapter.hw_dev_id;
    hw_item[kHwDevIfaceNameTag] = adapter.hw_iface_name;
    // Clear a previous adapter/run's address when interface-based selection is requested.
    hw_item.erase(kHwDevIpAddressTag);
    if (adapter.hw_type == "eth" && !adapter.hw_ip_address.empty()) {
      hw_item[kHwDevIpAddressTag] = adapter.hw_ip_address;
    }
    hw_item[kBaudRateTag] = adapter.baudrate;
    hw_items.push_back(hw_item);
  }
  runtime_config_.write("hw_inventory.json", hw_inventory);

  auto routing_table = nlohmann::json::parse(std::ifstream{kRoutingTableFilePath});
  auto & clients = routing_table[kClientsJsonTag];
  if (clients.empty()) {
    throw std::runtime_error("There are no 'clients' defined in the routing_table.json file.");
  }
  auto client = clients.front();  // Make a copy of the first client.
  clients.clear();
  for (auto i = 0UL; i < m_number_of_sensors; ++i) {
    const auto & sensor = m_sensors[i];
    client[kClientLinkTag] = sensor.link_type;
    client[kClientIdTag] = sensor.id;
    client[kHwDevIdTag] = sensor.dev_id;
    client[kPortTag] = sensor.port;
    client[kIpTag] = sensor.ip;
    client[kInstSerialTypeJsonTag] = sensor.inst_type;
    client[kDataSerialTypeJsonTag] = sensor.data_type;
    client[kUINameTag] = sensor.uifname;
    client[kUIMajorVTag] = sensor.uifmajorv;
    client[kUIMinorVTag] = sensor.uifminorv;
    client[kUIPatchVTag] = sensor.uifpatchv;
    clients.push_back(client);
  }

  runtime_config_.write("routing_table.json", routing_table);
}

}  // namespace radar
}  // namespace drivers
}  // namespace smartmicro

RCLCPP_COMPONENTS_REGISTER_NODE(smartmicro::drivers::radar::SmartmicroRadarNode)
