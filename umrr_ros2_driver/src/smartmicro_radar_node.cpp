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
#ifdef UMRR_HAVE_RADAR_MSGS
#include <radar_msgs/msg/radar_scan.hpp>
#endif
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <umrr_ros2_driver/sensor_models.hpp>
#include <umrr_ros2_driver/service_parsing.hpp>
#include <umrr_ros2_driver/udp_socket_health.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <umrr_ros2_driver/sensor_model_traits.hpp>
#include <umrr_ros2_driver/stream_codecs.hpp>

#include <fstream>
#include <limits>
#include <memory>
#include <utility>
#include <string>
#include <thread>
#include <tuple>
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
using smartmicro::drivers::radar::parse_command_value;
using smartmicro::drivers::radar::parse_mode_value;
using smartmicro::drivers::radar::validate_sensor_ipv4;

namespace
{
using SetModeRequest = umrr_ros2_msgs::srv::SetMode::Request;
using smartmicro::drivers::radar::ModeValueType;
static_assert(
  static_cast<uint8_t>(ModeValueType::kFloat32) == SetModeRequest::TYPE_FLOAT32 &&
  static_cast<uint8_t>(ModeValueType::kUint32) == SetModeRequest::TYPE_UINT32 &&
  static_cast<uint8_t>(ModeValueType::kUint16) == SetModeRequest::TYPE_UINT16 &&
  static_cast<uint8_t>(ModeValueType::kUint8) == SetModeRequest::TYPE_UINT8,
  "SetMode value type constants must match parse_mode_value");

using smartmicro::drivers::radar::kCanLinkType;
using smartmicro::drivers::radar::kCanModels;
using smartmicro::drivers::radar::kEthLinkType;
using smartmicro::drivers::radar::kEthernetModels;
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

constexpr auto kClientsJsonTag = "clients";
constexpr auto kHwItemsJsonTag = "hwItems";
constexpr auto kUINameTag = "user_interface_name";
constexpr auto kUIMajorVTag = "user_interface_major_v";
constexpr auto kUIMinorVTag = "user_interface_minor_v";
constexpr auto kUIPatchVTag = "user_interface_patch_v";

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
  if (update_service) {update_service->Cancel();}
  {
    std::lock_guard<std::mutex> lock(firmware_worker_mutex_);
    if (firmware_worker_.joinable()) {firmware_worker_.join();}
  }
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
  descriptor.description =
    "Target stream silence threshold in seconds (0.1..3600); restart to change.";
  descriptor.additional_constraints = "Number within [0.1, 3600]";
  // Dynamic typing: an integer such as `stale_timeout: 5` is accepted, as in the Python nodes.
  descriptor.dynamic_typing = true;
  const auto stale_timeout =
    declare_parameter("diagnostics.stale_timeout", rclcpp::ParameterValue(2.0), descriptor);
  if (stale_timeout.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    stale_timeout_seconds_ = static_cast<double>(stale_timeout.get<int64_t>());
  } else if (stale_timeout.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    stale_timeout_seconds_ = stale_timeout.get<double>();
  } else {
    throw std::invalid_argument("diagnostics.stale_timeout must be a number");
  }
  if (!std::isfinite(stale_timeout_seconds_) || stale_timeout_seconds_ < 0.1 ||
    stale_timeout_seconds_ > 3600.0)
  {
    throw std::invalid_argument("diagnostics.stale_timeout must be within 0.1..3600 s");
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
          status.summary(Status::STALE,
                health.frames ? "No recent targets" : "Waiting for targets");
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
    [this](
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<umrr_ros2_msgs::srv::FirmwareDownload::Request> request) {
      firmware_download(request_header, request);
    });

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
#ifdef UMRR_HAVE_RADAR_MSGS
    if (publish_radar_scan_) {
      radar_scan_publishers_[i] = create_publisher<radar_msgs::msg::RadarScan>(
        "smart_radar/radar_scan_" + std::to_string(i), sensor.history_size);
    }
#endif
    // Last: SDK callbacks may run as soon as they are registered.
    register_sensor_streams(sensor, i);
  }
}

void SmartmicroRadarNode::publish_radar_scan(
  size_t sensor_idx, const sensor_msgs::msg::PointCloud2 & cloud)
{
#ifdef UMRR_HAVE_RADAR_MSGS
  const auto publisher = std::static_pointer_cast<rclcpp::Publisher<radar_msgs::msg::RadarScan>>(
    radar_scan_publishers_[sensor_idx]);
  if (!publisher || publisher->get_subscription_count() == 0) {
    return;
  }
  // Same detections, header and order as the target cloud. doppler_velocity is
  // the SDK radial speed without sign conversion; amplitude is power [dB].
  auto scan = std::make_unique<radar_msgs::msg::RadarScan>();
  scan->header = cloud.header;
  scan->returns.resize(cloud.width);
  sensor_msgs::PointCloud2ConstIterator<float> range(cloud, "range");
  sensor_msgs::PointCloud2ConstIterator<float> azimuth(cloud, "azimuth_angle");
  sensor_msgs::PointCloud2ConstIterator<float> elevation(cloud, "elevation_angle");
  sensor_msgs::PointCloud2ConstIterator<float> speed(cloud, "radial_speed");
  sensor_msgs::PointCloud2ConstIterator<float> power(cloud, "power");
  for (auto & detection : scan->returns) {
    detection.range = *range;
    detection.azimuth = *azimuth;
    detection.elevation = *elevation;
    detection.doppler_velocity = *speed;
    detection.amplitude = *power;
    ++range, ++azimuth, ++elevation, ++speed, ++power;
  }
  publisher->publish(std::move(scan));
#else
  (void)sensor_idx;
  (void)cloud;
#endif
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

    } else if (pub_type == kTargetPubType) {
      m_publishers[sensor_idx] = create_publisher<sensor_msgs::msg::PointCloud2>(
        "smart_radar/port_targets_" + std::to_string(sensor_idx), sensor.history_size);
      m_publishers_port_target_header[sensor_idx] =
        create_publisher<umrr_ros2_msgs::msg::PortTargetHeader>(
        "smart_radar/port_targetheader_" + std::to_string(sensor_idx), sensor.history_size);
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
}

void SmartmicroRadarNode::firmware_download(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<umrr_ros2_msgs::srv::FirmwareDownload::Request> request)
{
  // The transfer and flash can take minutes. Reply is deferred: the request is
  // validated here, the blocking update runs on a worker thread and the worker
  // sends the response, so the executor keeps serving diagnostics and the other
  // services whatever executor or container the component runs in.
  umrr_ros2_msgs::srv::FirmwareDownload::Response response;
  if (!is_configured_sensor(request->sensor_id)) {
    response.res = "The sensor ID value entered is invalid! ";
    download_srv_->send_response(*request_header, response);
    return;
  }
  std::lock_guard<std::mutex> lock(firmware_worker_mutex_);
  // firmware_active_ is set before the worker starts: Busy() only turns true once the
  // worker is inside StartSoftwareUpdate, and joining a running worker here would block
  // this executor for the whole transfer.
  if (firmware_active_ || update_service->Busy()) {
    response.res = firmware_download_result(UpdateResult::kBusy);
    download_srv_->send_response(*request_header, response);
    return;
  }
  if (firmware_worker_.joinable()) {
    firmware_worker_.join();  // The previous worker has finished; reap its thread.
  }
  firmware_active_ = true;
  try {
    firmware_worker_ = std::thread(
      [this, request_header, client_id = request->sensor_id, image = request->file_path]() {
        struct ClearActive
        {
          std::atomic<bool> & active;
          ~ClearActive() {active = false;}
        } clear_active{firmware_active_};
        umrr_ros2_msgs::srv::FirmwareDownload::Response result;
        result.res =
        firmware_download_result(update_service->StartSoftwareUpdate(client_id, image));
        try {
          download_srv_->send_response(*request_header, result);
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "Could not send firmware download reply: %s", error.what());
        }
      });
  } catch (...) {
    firmware_active_ = false;
    throw;
  }
}

std::string SmartmicroRadarNode::firmware_download_result(UpdateResult update_result)
{
  switch (update_result) {
    case UpdateResult::kSuccess:
      return "Firmware download completed successfully.";
    case UpdateResult::kBusy:
      return "Firmware download rejected: another update is already in progress.";
    case UpdateResult::kFileOpenError:
      return "Firmware download failed: could not open update image file.";
    case UpdateResult::kFileSizeError:
      return "Firmware download failed: could not determine update image size.";
    case UpdateResult::kServiceUnavailable:
      return "Firmware download failed: update service is unavailable.";
    case UpdateResult::kStartFailed:
      return "Firmware download failed: could not start software update.";
    case UpdateResult::kTimeout:
      return "Firmware download failed: timed out and aborted.";
    case UpdateResult::kStoppedByMaster:
      return "Firmware download stopped by master.";
    case UpdateResult::kStoppedBySlave:
      return "Firmware download stopped by slave.";
    case UpdateResult::kBlockRepeatError:
      return "Firmware download failed: block repeat error.";
    case UpdateResult::kImageInvalid:
      return "Firmware download failed: invalid image.";
    case UpdateResult::kUnknownError:
    default:
      return "Firmware download failed: unknown error.";
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
          std::make_shared<SetParamRequest<decltype(typed_value)>>(section_name, param,
              typed_value));
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
  try {
    validate_sensor_ipv4(request->value_ip);
  } catch (const std::invalid_argument & error) {
    result->res_ip = std::string("Error: ") + error.what();
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

  uint32_t command_value{};
  try {
    command_value = parse_command_value(request->value);
  } catch (const std::invalid_argument & error) {
    result->res = std::string("Error: ") + error.what();
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
    std::make_shared<CmdRequest>(section_name, request->command, command_value);

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
      case umrr_ros2_msgs::srv::GetStatus::Request::TYPE_UINT32: {
          auto radar_status_u32 =
            std::make_shared<GetStatusRequest<uint32_t>>(section_name, status);
          request_added = batch->AddRequest(radar_status_u32);
          break;
        }
      case umrr_ros2_msgs::srv::GetStatus::Request::TYPE_UINT16: {
          auto radar_status_u16 =
            std::make_shared<GetStatusRequest<uint16_t>>(section_name, status);
          request_added = batch->AddRequest(radar_status_u16);
          break;
        }
      case umrr_ros2_msgs::srv::GetStatus::Request::TYPE_UINT8: {
          auto radar_status_u8 = std::make_shared<GetStatusRequest<uint8_t>>(section_name, status);
          request_added = batch->AddRequest(radar_status_u8);
          break;
        }
      case umrr_ros2_msgs::srv::GetStatus::Request::TYPE_INT32: {
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
      case umrr_ros2_msgs::srv::GetMode::Request::TYPE_FLOAT32: {
          auto radar_param_float = std::make_shared<GetParamRequest<float>>(section_name, param);
          request_added = batch->AddRequest(radar_param_float);
          break;
        }
      case umrr_ros2_msgs::srv::GetMode::Request::TYPE_UINT32: {
          auto radar_param_u32 = std::make_shared<GetParamRequest<uint32_t>>(section_name, param);
          request_added = batch->AddRequest(radar_param_u32);
          break;
        }
      case umrr_ros2_msgs::srv::GetMode::Request::TYPE_UINT16: {
          auto radar_param_u16 = std::make_shared<GetParamRequest<uint16_t>>(section_name, param);
          request_added = batch->AddRequest(radar_param_u16);
          break;
        }
      case umrr_ros2_msgs::srv::GetMode::Request::TYPE_UINT8: {
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

// ---- Sensor model registration (table-driven) ---------------------------------

void SmartmicroRadarNode::register_sensor_streams(
  const detail::SensorConfig & sensor, size_t sensor_idx)
{
  struct ModelEntry
  {
    std::string_view name;
    bool can_link;
    void (SmartmicroRadarNode::* register_streams)(const detail::SensorConfig &, size_t, bool);
  };
  // Model names accepted in sensors.sensor_N.model; kept in sync with
  // sensor_models.hpp (checked below) and param/model_uif_catalogue.yaml.
  static const std::array<ModelEntry, 52> table = {{
    {"umrra4_mse_v3_0_0", false, &SmartmicroRadarNode::register_model<models::Umrra4MseV300>},
    {"umrra4_mse_v2_1_0", false, &SmartmicroRadarNode::register_model<models::Umrra4MseV210>},
    {"umrra4_mse_v1_0_0", false, &SmartmicroRadarNode::register_model<models::Umrra4MseV100>},
    {"umrr9f_mse_v2_0_0", false, &SmartmicroRadarNode::register_model<models::Umrr9fMseV200>},
    {"umrr9f_mse_v1_3_0", false, &SmartmicroRadarNode::register_model<models::Umrr9fMseV130>},
    {"umrr9f_mse_v1_1_0", false, &SmartmicroRadarNode::register_model<models::Umrr9fMseV110>},
    {"umrr9f_mse_v1_0_0", false, &SmartmicroRadarNode::register_model<models::Umrr9fMseV100>},
    {"umrr96_v1_2_2", false, &SmartmicroRadarNode::register_model<models::Umrr96V122>},
    {"umrr11_v1_1_2", false, &SmartmicroRadarNode::register_model<models::Umrr11V112>},
    {"umrr9f_v1_1_1", false, &SmartmicroRadarNode::register_model<models::Umrr9fV111>},
    {"umrr9f_v2_0_0", false, &SmartmicroRadarNode::register_model<models::Umrr9fV200>},
    {"umrr9f_v2_1_1", false, &SmartmicroRadarNode::register_model<models::Umrr9fV211>},
    {"umrr9f_v2_2_1", false, &SmartmicroRadarNode::register_model<models::Umrr9fV221>},
    {"umrr9f_v2_4_1", false, &SmartmicroRadarNode::register_model<models::Umrr9fV241>},
    {"umrr9f_v3_0_0", false, &SmartmicroRadarNode::register_model<models::Umrr9fV300>},
    {"umrr9f_v3_2_0", false, &SmartmicroRadarNode::register_model<models::Umrr9fV320>},
    {"umrr9d_v1_0_3", false, &SmartmicroRadarNode::register_model<models::Umrr9dV103>},
    {"umrr9d_v1_2_2", false, &SmartmicroRadarNode::register_model<models::Umrr9dV122>},
    {"umrr9d_v1_4_1", false, &SmartmicroRadarNode::register_model<models::Umrr9dV141>},
    {"umrr9d_v1_5_0", false, &SmartmicroRadarNode::register_model<models::Umrr9dV150>},
    {"umrr9d_v1_7_0", false, &SmartmicroRadarNode::register_model<models::Umrr9dV170>},
    {"umrra4_v1_0_1", false, &SmartmicroRadarNode::register_model<models::Umrra4V101>},
    {"umrra4_v1_2_1", false, &SmartmicroRadarNode::register_model<models::Umrra4V121>},
    {"umrra4_v1_4_0", false, &SmartmicroRadarNode::register_model<models::Umrra4V140>},
    {"umrra4_v1_6_0", false, &SmartmicroRadarNode::register_model<models::Umrra4V160>},
    {"umrra1_v1_0_0", false, &SmartmicroRadarNode::register_model<models::Umrra1V100>},
    {"umrra1_v2_0_0", false, &SmartmicroRadarNode::register_model<models::Umrra1V200>},
    {"umrra1_v2_0_1", false, &SmartmicroRadarNode::register_model<models::Umrra1V201>},
    {"umrra1_v3_0_0", false, &SmartmicroRadarNode::register_model<models::Umrra1V300>},
    {"umrra4_can_mse_v2_1_0", true, &SmartmicroRadarNode::register_model<models::Umrra4MseV210>},
    {"umrra4_can_mse_v1_0_0", true, &SmartmicroRadarNode::register_model<models::Umrra4MseV100>},
    {"umrr9f_can_mse_v1_3_0", true, &SmartmicroRadarNode::register_model<models::Umrr9fMseV130>},
    {"umrr9f_can_mse_v1_1_0", true, &SmartmicroRadarNode::register_model<models::Umrr9fMseV110>},
    {"umrr9f_can_mse_v1_0_0", true, &SmartmicroRadarNode::register_model<models::Umrr9fMseV100>},
    {"umrra4_can_mse_v3_0_0", true, &SmartmicroRadarNode::register_model<models::Umrra4MseV300>},
    {"umrr9f_can_mse_v2_0_0", true, &SmartmicroRadarNode::register_model<models::Umrr9fMseV200>},
    {"umrr96_can_v1_2_2", true, &SmartmicroRadarNode::register_model<models::Umrr96V122>},
    {"umrr11_can_v1_1_2", true, &SmartmicroRadarNode::register_model<models::Umrr11V112>},
    {"umrr9f_can_v2_1_1", true, &SmartmicroRadarNode::register_model<models::Umrr9fV211>},
    {"umrr9f_can_v2_2_1", true, &SmartmicroRadarNode::register_model<models::Umrr9fV221>},
    {"umrr9f_can_v2_4_1", true, &SmartmicroRadarNode::register_model<models::Umrr9fV241>},
    {"umrr9f_can_v3_0_0", true, &SmartmicroRadarNode::register_model<models::Umrr9fV300>},
    {"umrr9f_can_v3_2_0", true, &SmartmicroRadarNode::register_model<models::Umrr9fV320>},
    {"umrr9d_can_v1_0_3", true, &SmartmicroRadarNode::register_model<models::Umrr9dV103>},
    {"umrr9d_can_v1_2_2", true, &SmartmicroRadarNode::register_model<models::Umrr9dV122>},
    {"umrr9d_can_v1_4_1", true, &SmartmicroRadarNode::register_model<models::Umrr9dV141>},
    {"umrr9d_can_v1_5_0", true, &SmartmicroRadarNode::register_model<models::Umrr9dV150>},
    {"umrr9d_can_v1_7_0", true, &SmartmicroRadarNode::register_model<models::Umrr9dV170>},
    {"umrra4_can_v1_0_1", true, &SmartmicroRadarNode::register_model<models::Umrra4V101>},
    {"umrra4_can_v1_2_1", true, &SmartmicroRadarNode::register_model<models::Umrra4V121>},
    {"umrra4_can_v1_4_0", true, &SmartmicroRadarNode::register_model<models::Umrra4V140>},
    {"umrra4_can_v1_6_0", true, &SmartmicroRadarNode::register_model<models::Umrra4V160>}
  }};

  static_assert(
    std::tuple_size_v<decltype(table)> == kEthernetModels.size() + kCanModels.size(),
    "Model table and sensor_models.hpp lists differ");
  const bool can_link = sensor.link_type == kCanLinkType;
  for (const auto & entry : table) {
    if (entry.name == sensor.model && entry.can_link == can_link) {
      (this->*entry.register_streams)(sensor, sensor_idx, can_link);
      return;
    }
  }
  throw std::invalid_argument("No registration for model " + sensor.model);
}

template<typename Model>
void SmartmicroRadarNode::register_model(
  const detail::SensorConfig & sensor, size_t sensor_idx, bool can_link)
{
  const auto service = Model::Service::Get();
  if (!service) {
    throw std::runtime_error("Smart Access data service unavailable for model " + sensor.model);
  }
  data_services_.push_back(service);
  const auto idx = static_cast<std::uint32_t>(sensor_idx);
  const auto check = [&](com::types::ErrorCode result, const char * stream) {
      if (result != com::types::ERROR_CODE_OK) {
        throw std::runtime_error(
                std::string("Failed to register the ") + stream + " callback for sensor " +
                std::to_string(sensor_idx) + " (" + sensor.model + ")");
      }
      RCLCPP_DEBUG(get_logger(), "Registered %s callback for sensor %zu", stream, sensor_idx);
    };
  if (!can_link) {
    auto targets = callback_gate_.wrap(
      [this, idx](const auto & list, com::types::ClientId client_id) {
        on_port_targets<Model>(idx, list, client_id);
      });
    if constexpr (Model::kTargetListPort) {
      check(service->RegisterComTargetListPortReceiveCallback(sensor.id, targets), "target list");
    } else {
      check(service->RegisterComTargetListReceiveCallback(sensor.id, targets), "target list");
    }
    if constexpr (Model::kPortObjects) {
      if (m_publishers_obj[sensor_idx]) {  // pub_type mse
        check(
          service->RegisterComObjectListReceiveCallback(
            sensor.id, callback_gate_.wrap(
              [this, idx](const auto & list, com::types::ClientId client_id) {
                on_port_objects<Model>(idx, list, client_id);
              })), "object list");
      }
    }
    if constexpr (Model::kFaultReports) {
      m_publishers_fault_report_msg[sensor_idx] =
        create_publisher<umrr_ros2_msgs::msg::PortFaultReportsMsg>(
        "smart_radar/port_faultreport_" + std::to_string(sensor_idx), sensor.history_size);
      check(
        service->RegisterFaultReportsReceiveCallback(
          sensor.id, callback_gate_.wrap(
            [this, idx](const auto & list, com::types::ClientId client_id) {
              on_fault_reports<Model>(idx, list, client_id);
            })), "fault report");
    }
  } else {
    if constexpr (Model::kCanTargets) {
      check(
        service->RegisterComTargetBaseListReceiveCallback(
          sensor.id, callback_gate_.wrap(
            [this, idx](const auto & list, com::types::ClientId client_id) {
              on_can_targets<Model>(idx, list, client_id);
            })), "CAN target list");
    }
    if constexpr (Model::kCanObjects) {
      if (m_publishers_obj[sensor_idx]) {  // pub_type mse
        check(
          service->RegisterComObjectBaseListReceiveCallback(
            sensor.id, callback_gate_.wrap(
              [this, idx](const auto & list, com::types::ClientId client_id) {
                on_can_objects<Model>(idx, list, client_id);
              })), "CAN object list");
      }
    }
  }
}

// ---- Per-stream SDK callbacks ---------------------------------------------------

template<typename Model, typename List>
void SmartmicroRadarNode::on_port_targets(
  const std::uint32_t sensor_idx, const std::shared_ptr<List> & list,
  const com::types::ClientId client_id)
{
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};
  fill_ros_header_stamp(msg, header, codec::port_header(*list)->GetTimestamp(), sensor_idx);
  if constexpr (Model::kTargetOptions.raw_quality) {
    auto raw_quality_ptr = std::make_unique<umrr_ros2_msgs::msg::Umrr96RawQuality>();
    auto & raw_quality = *raw_quality_ptr;
    const bool publish_raw = raw_quality_publishers_[sensor_idx] &&
      raw_quality_publishers_[sensor_idx]->get_subscription_count() > 0;
    if (publish_raw) {
      raw_quality.header = msg.header;
      raw_quality.sensor_id = client_id;
      raw_quality.semantics = umrr_ros2_msgs::msg::Umrr96RawQuality::SEMANTICS_UNVERIFIED;
    }
    codec::convert_port_targets<Model::kTargetOptions>(
      *list, header, modifier, publish_raw ? &raw_quality : nullptr);
    if (publish_raw) {raw_quality_publishers_[sensor_idx]->publish(std::move(raw_quality_ptr));}
  } else {
    codec::convert_port_targets<Model::kTargetOptions>(*list, header, modifier);
  }
  publish_radar_scan(sensor_idx, msg);
  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_target_header[sensor_idx]->publish(std::move(header_ptr));
}

template<typename Model, typename List>
void SmartmicroRadarNode::on_port_objects(
  const std::uint32_t sensor_idx, const std::shared_ptr<List> & list,
  const com::types::ClientId /*client_id*/)
{
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::PortObjectHeader>();
  auto & header = *header_ptr;
  ObjectCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};
  fill_ros_header_stamp(msg, header, list->GetPortHeader()->GetTimestamp(), sensor_idx);
  codec::convert_port_objects(*list, header, modifier);
  m_publishers_obj[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_port_obj_header[sensor_idx]->publish(std::move(header_ptr));
}

template<typename Model, typename List>
void SmartmicroRadarNode::on_fault_reports(
  const std::uint32_t sensor_idx, const std::shared_ptr<List> & list,
  const com::types::ClientId /*client_id*/)
{
  auto msg_ptr = std::make_unique<umrr_ros2_msgs::msg::PortFaultReportsMsg>();
  auto & msg = *msg_ptr;
  fill_ros_header_stamp(msg, list->GetPortHeader()->GetTimestamp(), sensor_idx);
  codec::convert_fault_reports(*list, msg);
  m_publishers_fault_report_msg[sensor_idx]->publish(std::move(msg_ptr));
}

template<typename Model, typename List>
void SmartmicroRadarNode::on_can_targets(
  const std::uint32_t sensor_idx, const std::shared_ptr<List> & list,
  const com::types::ClientId /*client_id*/)
{
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanTargetHeader>();
  auto & header = *header_ptr;
  RadarCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};
  fill_ros_header_stamp(msg, header, list->GetPortHeader()->GetTimestamp(), sensor_idx);
  codec::convert_can_targets(*list, header, modifier);
  publish_radar_scan(sensor_idx, msg);
  m_publishers[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_target_header[sensor_idx]->publish(std::move(header_ptr));
}

template<typename Model, typename List>
void SmartmicroRadarNode::on_can_objects(
  const std::uint32_t sensor_idx, const std::shared_ptr<List> & list,
  const com::types::ClientId /*client_id*/)
{
  auto msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto & msg = *msg_ptr;
  auto header_ptr = std::make_unique<umrr_ros2_msgs::msg::CanObjectHeader>();
  auto & header = *header_ptr;
  ObjectCloudBuilder modifier{msg, m_sensors[sensor_idx].frame_id};
  fill_ros_header_stamp(msg, header, list->GetPortHeader()->GetTimestamp(), sensor_idx);
  codec::convert_can_objects(*list, header, modifier);
  m_publishers_obj[sensor_idx]->publish(std::move(msg_ptr));
  m_publishers_can_obj_header[sensor_idx]->publish(std::move(header_ptr));
}
void SmartmicroRadarNode::update_config_files_from_params()
{
  auto radar_scan_descriptor = startup_descriptor();
  radar_scan_descriptor.description =
    "Also publish target lists as radar_msgs/RadarScan on smart_radar/radar_scan_N "
    "(requires a build with radar_msgs); restart to change.";
  publish_radar_scan_ = declare_parameter("publish_radar_scan", false, radar_scan_descriptor);
#ifndef UMRR_HAVE_RADAR_MSGS
  if (publish_radar_scan_) {
    throw std::invalid_argument(
            "publish_radar_scan is true, but this driver was built without radar_msgs");
  }
#endif
  const auto master_inst_serial_type = startup_parameter(*this, kInstSerialTypeTag, std::string{});
  const auto master_data_serial_type = startup_parameter(*this, kDataSerialTypeTag, std::string{});

  auto read_adapter_params_if_possible = [&](const std::uint32_t index) {
      auto & current_adapter = m_adapters[index];
      const auto prefix_2 = "adapters.adapter_" + std::to_string(index);
      current_adapter.hw_dev_id = startup_parameter(*this, prefix_2 + ".hw_dev_id",
            kDefaultHwDevId);
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
        throw std::invalid_argument(prefix_3 +
              ": invalid port, empty frame_id or zero history_size");
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
  config["shared_lib_path"] =
    runtime_config_.sdk_library_path(config["shared_lib_path"].get<std::string>());
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
