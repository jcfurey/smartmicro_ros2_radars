// Copyright 2021 Apex.AI, Inc.
//
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

#ifndef UMRR_ROS2_DRIVER__SMARTMICRO_RADAR_NODE_HPP_
#define UMRR_ROS2_DRIVER__SMARTMICRO_RADAR_NODE_HPP_

#include <CommunicationServicesIface.h>
#include <InstructionServiceIface.h>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <umrr_ros2_msgs/msg/radar_timing.hpp>
#include <umrr_ros2_msgs/msg/umrr96_raw_quality.hpp>
#include <umrr_ros2_driver/runtime_config.hpp>
#include <umrr_ros2_driver/startup_parameter.hpp>
#include <umrr_ros2_driver/stream_health.hpp>
#include <umrr_ros2_driver/sdk_callback_gate.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <umrr_ros2_msgs/msg/can_object_header.hpp>
#include <umrr_ros2_msgs/msg/can_target_header.hpp>
#include <umrr_ros2_msgs/msg/port_fault_report.hpp>
#include <umrr_ros2_msgs/msg/port_fault_report_header.hpp>
#include <umrr_ros2_msgs/msg/port_fault_reports_msg.hpp>
#include <umrr_ros2_msgs/msg/port_object_header.hpp>
#include <umrr_ros2_msgs/msg/port_target_header.hpp>
#include <umrr_ros2_msgs/srv/firmware_download.hpp>
#include <umrr_ros2_msgs/srv/get_mode.hpp>
#include <umrr_ros2_msgs/srv/get_status.hpp>
#include <umrr_ros2_msgs/srv/send_command.hpp>
#include <umrr_ros2_msgs/srv/set_ip.hpp>
#include <umrr_ros2_msgs/srv/set_mode.hpp>

#include <umrr_ros2_driver/instruction_reply.hpp>
#include <umrr_ros2_driver/update_service.hpp>
#include <umrr_ros2_driver/visibility_control.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace smartmicro
{
namespace drivers
{
namespace radar
{
namespace detail
{
constexpr auto kMaxSensorCount = 10UL;
constexpr auto kMaxHwCount = 6UL;

struct SensorConfig
{
  std::uint32_t id{};
  std::uint32_t dev_id{};
  std::string frame_id{};
  std::uint32_t history_size{};
  std::string ip{};
  std::string link_type{};
  std::string model{};
  std::uint32_t port{};
  std::string inst_type{};
  std::string data_type{};
  std::string uifname{};
  std::uint32_t uifmajorv{};
  std::uint32_t uifminorv{};
  std::uint32_t uifpatchv{};
  std::string pub_type{};
  // Subtracted from the receive time for every header stamp of this sensor [s].
  double stamp_offset_s{};
  // Target field also published as `intensity` (empty: none).
  std::string intensity_field{};
};

struct HWConfig
{
  std::uint32_t hw_dev_id{};
  std::string hw_iface_name{};
  std::string hw_ip_address{};
  std::string hw_type{};
  std::uint32_t baudrate{};
  std::uint32_t port{};
};
}  // namespace detail

///
/// @brief      The class for the Smartmicro radar node.
///
class UMRR_ROS2_DRIVER_PUBLIC SmartmicroRadarNode : public ::rclcpp::Node
{
public:
  ///
  /// @brief      Constructs the Smartmicro radar node.
  ///
  /// @param[in]  node_options  Node options for this node.
  ///
  explicit SmartmicroRadarNode(const rclcpp::NodeOptions & node_options);

  ///
  /// @brief      Constructs the node with a private SDK configuration directory that
  ///             the caller created, and activated before rclcpp::init (standalone
  ///             executable; see RuntimeConfig::activate).
  ///
  /// @param[in]  node_options    Node options for this node.
  /// @param[in]  runtime_config  The SDK configuration directory; owned by the node.
  ///
  SmartmicroRadarNode(
    const rclcpp::NodeOptions & node_options, std::unique_ptr<RuntimeConfig> runtime_config);

  ~SmartmicroRadarNode() override;

private:
  ///
  /// @brief      Registers the SDK callbacks of the configured model's streams.
  ///
  /// @param[in]  sensor       The sensor configuration (model, link type, id).
  /// @param[in]  sensor_idx   The sensor index used in topic names.
  ///
  void register_sensor_streams(const detail::SensorConfig & sensor, size_t sensor_idx);

  ///
  /// @brief      Per-stream SDK callbacks, instantiated once per sensor model.
  ///
  /// @tparam     Model  Model traits (SDK namespace and publishing options).
  /// @param[in]  sensor_idx  The sensor index for the respective published topics.
  /// @param[in]  list        The received SDK list.
  /// @param[in]  client_id   The client identifier of the sensor.
  ///
  template<typename Model, typename List>
  void on_port_targets(
    std::uint32_t sensor_idx, const std::shared_ptr<List> & list,
    com::types::ClientId client_id);
  template<typename Model, typename List>
  void on_port_objects(
    std::uint32_t sensor_idx, const std::shared_ptr<List> & list,
    com::types::ClientId client_id);
  template<typename Model, typename List>
  void on_fault_reports(
    std::uint32_t sensor_idx, const std::shared_ptr<List> & list,
    com::types::ClientId client_id);
  template<typename Model, typename List>
  void on_can_targets(
    std::uint32_t sensor_idx, const std::shared_ptr<List> & list,
    com::types::ClientId client_id);
  template<typename Model, typename List>
  void on_can_objects(
    std::uint32_t sensor_idx, const std::shared_ptr<List> & list,
    com::types::ClientId client_id);

  template<typename Model>
  void register_model(const detail::SensorConfig & sensor, size_t sensor_idx, bool can_link);

  ///
  /// @brief      Read parameters and update the json config files required by
  /// Smart Access C++ API.
  ///
  void update_config_files_from_params();

  ///
  /// @brief      Creates publishers for sensors using ports.
  ///
  /// @param[in]  sensor       The sensor configuration.
  /// @param[in]  sensor_idx   The sensor index.
  ///
  void port_publishers(const detail::SensorConfig & sensor, size_t sensor_idx);

  ///
  /// @brief      Publishes a target cloud as radar_msgs/RadarScan when enabled.
  ///
  void publish_radar_scan(size_t sensor_idx, const sensor_msgs::msg::PointCloud2 & cloud);

  ///
  /// @brief      Creates publishers for sensors using CAN.
  ///
  /// @param[in]  sensor       The sensor configuration.
  /// @param[in]  sensor_idx   The sensor index.
  ///
  void can_publishers(const detail::SensorConfig & sensor, size_t sensor_idx);

  ///
  /// @brief      Sends instructions to the sensor.
  ///
  /// @param[in]  request_header  Identifies the request for the deferred reply.
  /// @param[in]  request         The request.
  ///
  void set_radar_mode(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<umrr_ros2_msgs::srv::SetMode::Request> request);

  ///
  /// @brief      Configures the sensor IP address.
  ///
  /// @param[in]  request_header  Identifies the request for the deferred reply.
  /// @param[in]  request         The request.
  ///
  void ip_address(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<umrr_ros2_msgs::srv::SetIp::Request> request);

  ///
  /// @brief      Sends command to the sensor.
  ///
  /// @param[in]  request_header  Identifies the request for the deferred reply.
  /// @param[in]  request         The request.
  ///
  void radar_command(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<umrr_ros2_msgs::srv::SendCommand::Request> request);

  ///
  /// @brief      Service for firmware download (deferred response).
  ///
  /// @param[in]  request_header  Identifies the request for the deferred reply.
  /// @param[in]  request         The request.
  ///
  void firmware_download(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<umrr_ros2_msgs::srv::FirmwareDownload::Request> request);

  static std::string firmware_download_result(UpdateResult update_result);

  ///
  /// @brief      Publishes the RadarTiming of a received list, updates the stream's
  ///             health and returns the header stamp: the ROS receive time minus the
  ///             sensor's stamp_offset_s (not earlier than time zero).
  ///
  /// @param[in]  timestamp_us  Device timestamp [us], or none if the list has none.
  ///
  builtin_interfaces::msg::Time receive_stamp(
    std::optional<uint64_t> timestamp_us, uint32_t sensor_idx, uint8_t stream);
  void setup_diagnostics();

  ///
  /// @brief      Adds the liveness and device-timestamp status of one data stream.
  ///
  /// @param[in]  name         Status name, e.g. "Target stream 0".
  /// @param[in]  health       The stream's health, updated by receive_stamp().
  /// @param[in]  sensor_idx   The sensor index.
  /// @param[in]  items        What the stream delivers ("targets", "objects").
  /// @param[in]  hardware_id  The sensor's hardware id.
  /// @param[in]  device_timestamps  Whether the stream's lists carry a device timestamp.
  ///
  void add_stream_status(
    const std::string & name, StreamHealth & health, size_t sensor_idx,
    const std::string & items, const std::string & hardware_id, bool device_timestamps);

  ///
  /// @brief Fills the ROS timestamp for the PointCloud2 message and the custom header message.
  ///
  /// @tparam HeaderMsgT One of the custom header message types:
  ///         PortTargetHeader, PortObjectHeader, CanTargetHeader, CanObjectHeader.
  ///
  /// @param[in,out] msg         The PointCloud2 message.
  /// @param[in,out] header      The custom ROS header message.
  /// @param[in]     timestamp_us Sensor timestamp in microseconds, if the list has one.
  /// @param[in]     sensor_idx   Sensor index used to select the frame_id.
  ///
  template<typename HeaderMsgT>
  void fill_ros_header_stamp(
    sensor_msgs::msg::PointCloud2 & msg,
    HeaderMsgT & header,
    const std::optional<std::uint64_t> timestamp_us,
    const std::uint32_t sensor_idx)
  {
    constexpr bool objects =
      std::is_same_v<HeaderMsgT, umrr_ros2_msgs::msg::PortObjectHeader>||
      std::is_same_v<HeaderMsgT, umrr_ros2_msgs::msg::CanObjectHeader>;
    using Timing = umrr_ros2_msgs::msg::RadarTiming;
    const auto stamp = receive_stamp(
      timestamp_us, sensor_idx, objects ? Timing::OBJECTS : Timing::TARGETS);

    msg.header.stamp = stamp;
    header.header.stamp = stamp;
    header.header.frame_id = m_sensors[sensor_idx].frame_id;
  }

  ///
  /// @brief      Fills the ROS header timestamp and frame ID of a message.
  ///
  /// @tparam MsgT  A ROS message type that contains a standard header member.
  /// @param[in,out] msg  The ROS message to update.
  /// @param[in] timestamp_us  Sensor timestamp in microseconds.
  /// @param[in] sensor_idx  Sensor index used to select the frame ID.
  ///
  template<typename MsgT>
  void fill_ros_header_stamp(
    MsgT & msg,
    const std::uint64_t timestamp_us,
    const std::uint32_t sensor_idx)
  {
    const auto stamp = receive_stamp(
      timestamp_us, sensor_idx, umrr_ros2_msgs::msg::RadarTiming::FAULTS);

    msg.header.stamp = stamp;
    msg.header.frame_id = m_sensors[sensor_idx].frame_id;
  }

  ///
  /// @brief      Whether a service request names one of the configured sensors.
  ///
  bool is_configured_sensor(com::types::ClientId client_id) const
  {
    for (size_t i = 0; i < m_number_of_sensors; ++i) {
      if (m_sensors[i].id == client_id) {return true;}
    }
    return false;
  }

  ///
  /// @brief      Initializes all the smart access and ros2 services.
  ///
  void initialize_services();

  ///
  /// @brief      Check and set up publsihers w.r.t defined radar parameters.
  ///
  void setup_publishers();

  ///
  /// @brief      Service to get sensor status.
  ///
  /// @param[in]  request_header  Identifies the request for the deferred reply.
  /// @param[in]  request         The request.
  ///
  void get_radar_status(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<umrr_ros2_msgs::srv::GetStatus::Request> request);

  ///
  /// @brief      Service to get sensor modes.
  ///
  /// @param[in]  request_header  Identifies the request for the deferred reply.
  /// @param[in]  request         The request.
  ///
  void get_radar_mode(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<umrr_ros2_msgs::srv::GetMode::Request> request);

  using InstructionReply = std::function<void (const std::string & text)>;

  ///
  /// @brief      Sends an allocated instruction batch. The service reply is deferred
  ///             until the sensor answers or instruction_timeout_ms expires; the
  ///             batch is then released on the executor, never in an SDK callback.
  ///
  /// @param[in,out] batch       The allocated batch; taken over once sent.
  /// @param[in]  sensor_id      The sensor the batch is addressed to.
  /// @param[in]  section        The request's interface section (reported in the reply).
  /// @param[in]  items          The instructions whose reply values are reported.
  /// @param[in]  reply          Sends the service response; called exactly once.
  /// @param[in]  success_note   Added to a reply the sensor accepted, when not empty.
  ///
  void send_instructions(
    InstructionBatchLease & batch, com::types::ClientId sensor_id, const std::string & section,
    std::vector<InstructionItem> items, InstructionReply reply,
    const std::string & success_note = {});

  ///
  /// @brief      Replies to timed-out instruction requests and releases answered batches.
  ///
  void expire_instructions();

  ///
  /// @brief      Creates a data publisher whose QoS can be overridden by parameters
  ///             (qos_overrides.<topic>.publisher.{reliability,history,depth}).
  ///
  template<typename MsgT>
  typename rclcpp::Publisher<MsgT>::SharedPtr create_data_publisher(
    const std::string & topic, size_t depth);

  // Declared first so its directory outlives the node's publishers/services.
  std::unique_ptr<RuntimeConfig> runtime_config_;
  SdkCallbackGate callback_gate_;
  // Liveness per configured stream: target lists, and object lists (pub_type mse).
  std::array<StreamHealth, detail::kMaxSensorCount> target_health_;
  std::array<StreamHealth, detail::kMaxSensorCount> object_health_;
  std::array<rclcpp::Publisher<umrr_ros2_msgs::msg::RadarTiming>::SharedPtr,
    detail::kMaxSensorCount> timing_publishers_;
  std::array<rclcpp::Publisher<umrr_ros2_msgs::msg::Umrr96RawQuality>::SharedPtr,
    detail::kMaxSensorCount> raw_quality_publishers_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  double stale_timeout_seconds_{2.0};

  rclcpp::Service<umrr_ros2_msgs::srv::SetMode>::SharedPtr mode_srv_;
  rclcpp::Service<umrr_ros2_msgs::srv::SetIp>::SharedPtr ip_addr_srv_;
  rclcpp::Service<umrr_ros2_msgs::srv::SendCommand>::SharedPtr command_srv_;
  rclcpp::Service<umrr_ros2_msgs::srv::FirmwareDownload>::SharedPtr download_srv_;
  rclcpp::Service<umrr_ros2_msgs::srv::GetStatus>::SharedPtr status_srv_;
  rclcpp::Service<umrr_ros2_msgs::srv::GetMode>::SharedPtr read_mode_srv_;

  // Instruction batches sent and awaiting the sensor's reply, by request number.
  struct PendingInstruction
  {
    std::shared_ptr<com::master::InstructionBatch> batch;
    std::chrono::steady_clock::time_point deadline;
    com::types::ClientId sensor_id;
    std::string section;
    InstructionReply reply;
  };
  std::mutex instructions_mutex_;
  std::map<uint64_t, PendingInstruction> pending_instructions_;
  std::vector<std::shared_ptr<com::master::InstructionBatch>> answered_batches_;
  uint64_t next_instruction_id_{};
  uint64_t instruction_timeouts_{};
  std::chrono::milliseconds instruction_timeout_{3000};
  rclcpp::TimerBase::SharedPtr instruction_timer_;
  uint64_t reported_callback_exceptions_{};

  std::array<detail::SensorConfig, detail::kMaxSensorCount> m_sensors{};
  std::array<detail::HWConfig, detail::kMaxHwCount> m_adapters{};

  std::array<
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr, detail::kMaxSensorCount>
  m_publishers_obj{};

  std::array<
    rclcpp::Publisher<umrr_ros2_msgs::msg::PortObjectHeader>::SharedPtr, detail::kMaxSensorCount>
  m_publishers_port_obj_header{};

  std::array<
    rclcpp::Publisher<umrr_ros2_msgs::msg::CanObjectHeader>::SharedPtr, detail::kMaxSensorCount>
  m_publishers_can_obj_header{};

  std::array<
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr, detail::kMaxSensorCount>
  m_publishers{};

  std::array<
    rclcpp::Publisher<umrr_ros2_msgs::msg::PortTargetHeader>::SharedPtr, detail::kMaxSensorCount>
  m_publishers_port_target_header{};

  std::array<
    rclcpp::Publisher<umrr_ros2_msgs::msg::CanTargetHeader>::SharedPtr, detail::kMaxSensorCount>
  m_publishers_can_target_header{};

  std::array<
    rclcpp::Publisher<umrr_ros2_msgs::msg::PortFaultReportsMsg>::SharedPtr, detail::kMaxSensorCount>
  m_publishers_fault_report_msg{};

  // radar_msgs/RadarScan publishers (type-erased: radar_msgs is optional).
  bool publish_radar_scan_{false};
  std::array<rclcpp::PublisherBase::SharedPtr, detail::kMaxSensorCount> radar_scan_publishers_{};

  std::size_t m_number_of_sensors{};
  std::size_t m_number_of_adapters{};
  std::shared_ptr<UpdateService> update_service;
  std::mutex firmware_worker_mutex_;
  std::thread firmware_worker_;
  std::atomic<bool> firmware_active_{false};  // From request acceptance to worker exit.

  // SDK service handles are owned by the node (not namespace-scope globals in an
  // installed header) and released before the SDK's own static objects.
  std::shared_ptr<com::master::CommunicationServicesIface> m_services{};
  // Data stream services of the configured models (type-erased).
  std::vector<std::shared_ptr<void>> data_services_;
};


}  // namespace radar
}  // namespace drivers
}  // namespace smartmicro

#endif  // UMRR_ROS2_DRIVER__SMARTMICRO_RADAR_NODE_HPP_
