// SPDX-License-Identifier: Apache-2.0

#include <CommunicationServicesIface.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <umrr_ros2_driver/readback_node.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <umrr_ros2_driver/runtime_config.hpp>
#include <umrr_ros2_driver/startup_parameter.hpp>
#include <umrr_ros2_driver/umrr96_tuning.hpp>
#include <umrr_ros2_msgs/srv/get_mode.hpp>
#include <umrr_ros2_msgs/srv/get_status.hpp>
#include <umrr_ros2_msgs/srv/set_mode.hpp>

#include <array>
#include <chrono>
#include <charconv>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <map>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
using Json = nlohmann::json;
using GetMode = umrr_ros2_msgs::srv::GetMode;
using GetStatus = umrr_ros2_msgs::srv::GetStatus;
using SetMode = umrr_ros2_msgs::srv::SetMode;
using com::master::InstructionBatch;
using com::master::ResponseBatch;

using smartmicro::drivers::radar::RuntimeConfig;
using smartmicro::drivers::radar::startup_parameter;

enum class ValueType { F32, U32, U16, U8, I32 };

struct BatchLease
{
  ~BatchLease()
  {
    // Also release rejected requests and expired batches from the SDK's registry.
    if (batch) {
      service->ReleaseInstructionBatch(batch);
    }
  }

  std::shared_ptr<com::master::InstructionServiceIface> service;
  std::shared_ptr<InstructionBatch> batch;
};

template<typename T>
bool add_read(
  const std::shared_ptr<InstructionBatch> & batch, bool status,
  const std::string & section, const std::string & name)
{
  if (status) {
    return batch->AddRequest(std::make_shared<com::master::GetStatusRequest<T>>(section, name));
  }
  return batch->AddRequest(std::make_shared<com::master::GetParamRequest<T>>(section, name));
}

template<typename T>
Json read_value(
  const std::shared_ptr<ResponseBatch> & batch,
  const std::string & section, const std::string & name)
{
  std::vector<std::shared_ptr<com::master::Response<T>>> values;
  if (!batch || !batch->GetResponse<T>(section, name, values) ||
    values.size() != 1 || !values.front())
  {
    return {{"response_type", 0}, {"error", "Missing or unexpected sensor response"}};
  }
  const auto code = values.front()->GetResponseType();
  Json result = {{"response_type", code}};
  if (code == 1) {
    result["value"] = values.front()->GetValue();
  } else {
    result["error"] = "Sensor rejected the instruction";
  }
  return result;
}

// A sensor reply that did not arrive in time; counted separately from failures.
class ReplyTimeout : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

struct PendingResponse
{
  std::mutex mutex;
  std::condition_variable ready;
  bool received{false};
  Json result;
};

}  // namespace

namespace smartmicro::drivers::radar
{
// Composable, but the SDK is a process-wide singleton: load it only into a
// container that does not also hold the data node or another readback node.
class ReadbackNode : public rclcpp::Node
{
public:
  explicit ReadbackNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("smart_radar_readback", options)
  {
    const auto sensor_id = startup_parameter(*this, "sensor_id", 0, 1, UINT32_MAX);
    const auto host_port = startup_parameter(*this, "host_port", 55556, 1, 65535);
    const auto sensor_port = startup_parameter(*this, "sensor_port", 55555, 1, 65535);
    // Bounded below the 5 s deadline of the RViz UMRR-96 panels, so a sensor
    // timeout is reported to the panel instead of the panel giving up first.
    const auto timeout_ms = startup_parameter(*this, "timeout_ms", 2000, 1, kMaxTimeoutMs);
    const auto interface = startup_parameter(*this, "interface_name", "enp68s0f0");
    const auto host_ip = startup_parameter(*this, "host_ip", "192.168.11.17");
    const auto sensor_ip = startup_parameter(*this, "sensor_ip", "192.168.11.11");
    if (sensor_id <= 0 || sensor_id > UINT32_MAX || host_port <= 0 || host_port > 65535 ||
      sensor_port <= 0 || sensor_port > 65535 || timeout_ms <= 0 || timeout_ms > kMaxTimeoutMs)
    {
      throw std::invalid_argument("Invalid sensor_id, UDP port, or timeout_ms");
    }
    sensor_id_ = static_cast<uint32_t>(sensor_id);
    timeout_ = std::chrono::milliseconds(timeout_ms);
    // The sensor restores its stored CAN target-list setting at every power-up; with
    // CAN serialization on, the Ethernet target stream of this UMRR-96 drops from
    // 18.2 to 8.3 Hz. 0 (default) switches it off, 1 on, -1 leaves it unchanged.
    startup_can_output_ = startup_parameter(*this, "startup_can_target_output", 0, -1, 1);
    if (startup_can_output_ < -1 || startup_can_output_ > 1) {
      throw std::invalid_argument("startup_can_target_output must be -1, 0 or 1");
    }

    config_.write("smart_access_config.json", {
        {"name", "UMRR-96 readback"}, {"version", "1.0.0"},
        {"client_id", 0xc0000001u}, {"role", "master"}, {"alive", false},
        {"shared_lib_path", config_.sdk_library_path(SMARTMICRO_SDK_LIBRARY_PATH)},
        {"config_path", config_.path.string()}, {"download_path", ""},
        {"user_interface_name", "base"}, {"user_interface_major_v", 1},
        {"user_interface_minor_v", 0}, {"user_interface_patch_v", 2},
        {"instruction_serialization_type", "can_based"},
        {"data_serialization_type", "can_based"}});
    config_.write("hw_inventory.json", {
        {"name", "Readback socket"}, {"version", "1.1.0"},
        {"hwItems", Json::array({{
            {"type", "eth"}, {"dev_id", 1}, {"iface_name", interface},
            {"ip_address", host_ip}, {"port", host_port}}})}});
    config_.write("routing_table.json", {
        {"name", "Readback route"}, {"version", "1.0.0"},
        {"clients", Json::array({{
            {"client_id", sensor_id_}, {"link_type", "eth"}, {"dev_id", 1},
            {"ip", sensor_ip}, {"port", sensor_port}, {"can_network_id", 0},
            {"instruction_serialization_type", "can_based"},
            {"data_serialization_type", "can_based"},
            {"user_interface_name", "umrr96_t153_automotive"},
            {"user_interface_major_v", 1}, {"user_interface_minor_v", 2},
            {"user_interface_patch_v", 2}}})}});
    config_.activate();
    services_ = com::master::CommunicationServicesIface::Get();
    if (!services_->Init()) {
      throw std::runtime_error("Readback SDK initialization failed");
    }

    mode_service_ = create_service<GetMode>(
      "smart_radar/get_radar_mode",
      [this](const GetMode::Request::SharedPtr request, GetMode::Response::SharedPtr response) {
        response->res = read(
          request->sensor_id, request->section_name, request->params,
          request->param_types, false).dump(2);
      });
    status_service_ = create_service<GetStatus>(
      "smart_radar/get_radar_status",
      [this](const GetStatus::Request::SharedPtr request, GetStatus::Response::SharedPtr response) {
        response->res = read(
          request->sensor_id, request->section_name, request->statuses,
          request->status_types, true).dump(2);
      });
    set_service_ = create_service<SetMode>(
      "smart_radar/set_radar_mode",
      [this](const SetMode::Request::SharedPtr request, SetMode::Response::SharedPtr response) {
        response->res = write(*request).dump(2);
      });
    diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
    diagnostics_->setHardwareID("umrr96@" + sensor_ip);
    diagnostics_->add("Control requests",
      [this](diagnostic_updater::DiagnosticStatusWrapper & stat) {
        using Status = diagnostic_msgs::msg::DiagnosticStatus;
        if (!last_error_.empty()) {
          stat.summary(Status::WARN, last_error_);
        } else if (!exchanges_) {
          stat.summary(Status::STALE, "Idle; sensor reachability has not been checked");
        } else {
          stat.summary(Status::OK, "Last request succeeded; no automatic polling");
        }
        stat.add("sensor_id", sensor_id_);
        stat.add("requests_sent_or_attempted", exchanges_);
        stat.add("invalid_requests", invalid_requests_);
        stat.add("failed_requests", failed_requests_);
        stat.add("timeouts", timeouts_);
        stat.add("sensor_rejected_batches", sensor_rejections_);
        stat.add("responses_received", responses_);
        stat.add("last_response_age_seconds", responses_ ?
          std::chrono::duration<double>(std::chrono::steady_clock::now() - last_response_).count() :
          -1.0);
        stat.add("startup_can_target_output", startup_can_state_);
      });
    if (startup_can_output_ >= 0) {
      startup_can_state_ = "pending";
      startup_timer_ = create_wall_timer(
        std::chrono::seconds(1), [this]() {apply_startup_can_output();});
    }
    RCLCPP_INFO(get_logger(), "Readback ready for sensor %u on %s:%s", sensor_id_,
      host_ip.c_str(), std::to_string(host_port).c_str());
  }

private:
  // Volatile write (no EEPROM save) verified by a separate read. Retried every 5 s
  // until confirmed, because the sensor may still be booting. Startup only: a later
  // change through the panel or the service is never overridden, and a sensor power
  // cycle while this node runs restores the stored value until the node restarts.
  void apply_startup_can_output()
  {
    static const std::string kName = "output_control_target_list_can";
    static const std::string kSection = "auto_interface_0dim";
    ++startup_can_attempts_;
    SetMode::Request request;
    request.sensor_id = sensor_id_;
    request.section_name = kSection;
    request.params = {kName};
    request.values = {std::to_string(startup_can_output_)};
    request.value_types = {SetMode::Request::TYPE_UINT8};
    auto result = write(request);
    if (result.value("success", false)) {
      result = read(sensor_id_, kSection, {kName}, {GetMode::Request::TYPE_UINT8}, false);
    }
    const bool confirmed = result.value("success", false) && result.contains("values") &&
      result["values"].contains(kName) &&
      result["values"][kName].value("value", int64_t{-1}) == startup_can_output_;
    if (confirmed) {
      startup_timer_->cancel();
      startup_can_state_ = startup_can_output_ ? "on (confirmed)" : "off (confirmed)";
      RCLCPP_INFO(get_logger(), "CAN target output %s (read back after %u attempt%s)",
        startup_can_output_ ? "on" : "off", startup_can_attempts_,
        startup_can_attempts_ == 1 ? "" : "s");
      return;
    }
    const auto error = result.value("error", std::string("readback did not match"));
    startup_can_state_ = "retrying: " + error;
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 30000,
      "Could not set CAN target output at startup (%s); retrying every 5 s", error.c_str());
    if (startup_can_attempts_ == 1) {
      startup_timer_->cancel();
      startup_timer_ = create_wall_timer(
        std::chrono::seconds(5), [this]() {apply_startup_can_output();});
    }
  }

  Json write(const SetMode::Request & request)
  {
    try {
      if (request.sensor_id != sensor_id_ || request.section_name != "auto_interface_0dim") {
        throw std::invalid_argument("Invalid tuning sensor ID or section");
      }
      const auto values = smartmicro::drivers::radar::validate_umrr96_tuning(
        request.params, request.values, request.value_types);
      auto instructions = services_->GetInstructionService();
      std::shared_ptr<InstructionBatch> batch;
      if (!instructions || !instructions->AllocateInstructionBatch(sensor_id_, batch)) {
        throw std::runtime_error("Could not allocate SDK tuning request");
      }
      const BatchLease lease{instructions, batch};
      std::vector<ValueType> types;
      for (size_t i = 0; i < request.params.size(); ++i) {
        const bool floating = std::holds_alternative<float>(values[i]);
        types.push_back(floating ? ValueType::F32 : ValueType::U8);
        const bool added = std::visit([&](auto value) {
              return batch->AddRequest(
              std::make_shared<com::master::SetParamRequest<decltype(value)>>(
              request.section_name, request.params[i], value));
          }, values[i]);
        if (!added) {
          throw std::invalid_argument("SDK rejected tuning parameter: " + request.params[i]);
        }
      }
      return exchange(batch, request.params, types, request.section_name);
    } catch (const std::invalid_argument & error) {
      ++invalid_requests_;
      return {{"sensor_id", request.sensor_id}, {"section", request.section_name},
        {"success", false}, {"error", error.what()}};
    } catch (const ReplyTimeout & error) {
      last_error_ = error.what();  // Counted once, in timeouts_.
      return {{"sensor_id", request.sensor_id}, {"section", request.section_name},
        {"success", false}, {"error", error.what()}};
    } catch (const std::exception & error) {
      ++failed_requests_;
      last_error_ = error.what();
      return {{"sensor_id", request.sensor_id}, {"section", request.section_name},
        {"success", false}, {"error", error.what()}};
    }
  }

  Json read(
    uint32_t sensor_id, const std::string & section, const std::vector<std::string> & names,
    const std::vector<uint8_t> & requested_types, bool status)
  {
    try {
      if (sensor_id != sensor_id_) {
        throw std::invalid_argument("Sensor ID does not match this readback node");
      }
      if (section != (status ? "auto_interface" : "auto_interface_0dim")) {
        throw std::invalid_argument("Invalid UMRR-96 readback section");
      }
      if (names.empty() || names.size() > 10 || names.size() != requested_types.size()) {
        throw std::invalid_argument("Supply 1–10 names and an equal number of data types");
      }
      if (std::set<std::string>(names.begin(), names.end()).size() != names.size()) {
        throw std::invalid_argument("Readback names must be unique");
      }
      constexpr std::array<ValueType, 4> param_types = {
        ValueType::F32, ValueType::U32, ValueType::U16, ValueType::U8};
      constexpr std::array<ValueType, 4> status_types = {
        ValueType::U32, ValueType::U16, ValueType::U8, ValueType::I32};
      std::vector<ValueType> types;
      auto instructions = services_->GetInstructionService();
      std::shared_ptr<InstructionBatch> batch;
      if (!instructions || !instructions->AllocateInstructionBatch(sensor_id_, batch)) {
        throw std::runtime_error("Could not allocate SDK read request");
      }
      const BatchLease lease{instructions, batch};
      for (size_t i = 0; i < names.size(); ++i) {
        if (requested_types[i] > 3) {
          throw std::invalid_argument("Invalid readback data type");
        }
        const auto type = (status ? status_types : param_types)[requested_types[i]];
        types.push_back(type);
        bool added = false;
        switch (type) {
          case ValueType::F32: added = add_read<float>(batch, status, section, names[i]); break;
          case ValueType::U32: added = add_read<uint32_t>(batch, status, section, names[i]); break;
          case ValueType::U16: added = add_read<uint16_t>(batch, status, section, names[i]); break;
          case ValueType::U8: added = add_read<uint8_t>(batch, status, section, names[i]); break;
          case ValueType::I32: added = add_read<int32_t>(batch, status, section, names[i]); break;
        }
        if (!added) {
          throw std::invalid_argument("Unknown, duplicate, or incorrectly typed read: " + names[i]);
        }
      }
      return exchange(batch, names, types, section);
    } catch (const std::invalid_argument & error) {
      ++invalid_requests_;
      return {{"sensor_id", sensor_id}, {"section", section},
        {"success", false}, {"error", error.what()}};
    } catch (const ReplyTimeout & error) {
      last_error_ = error.what();  // Counted once, in timeouts_.
      return {{"sensor_id", sensor_id}, {"section", section},
        {"success", false}, {"error", error.what()}};
    } catch (const std::exception & error) {
      ++failed_requests_;
      last_error_ = error.what();
      return {{"sensor_id", sensor_id}, {"section", section},
        {"success", false}, {"error", error.what()}};
    }
  }

  Json exchange(
    const std::shared_ptr<InstructionBatch> & batch, const std::vector<std::string> & names,
    const std::vector<ValueType> & types, const std::string & section)
  {
    ++exchanges_;
    const auto sensor_id = sensor_id_;
    auto instructions = services_->GetInstructionService();
    // Capture owned state only: a late reply must remain safe after a timeout.
    const auto pending = std::make_shared<PendingResponse>();
    const auto sent = instructions->SendInstructionBatch(
      batch, [pending, names, types, section, sensor_id](
        com::types::ClientId, const std::shared_ptr<ResponseBatch> response) {
        Json result = {{"sensor_id", sensor_id}, {"section", section},
          {"success", true}, {"values", Json::object()}};
        for (size_t i = 0; i < names.size(); ++i) {
          Json value;
          switch (types[i]) {
            case ValueType::F32: value = read_value<float>(response, section, names[i]); break;
            case ValueType::U32: value = read_value<uint32_t>(response, section, names[i]); break;
            case ValueType::U16: value = read_value<uint16_t>(response, section, names[i]); break;
            case ValueType::U8: value = read_value<uint8_t>(response, section, names[i]); break;
            case ValueType::I32: value = read_value<int32_t>(response, section, names[i]); break;
          }
          if (value["response_type"] != 1) {
            result["success"] = false;
          }
          result["values"][names[i]] = std::move(value);
        }
        {
          std::lock_guard<std::mutex> lock(pending->mutex);
          pending->result = std::move(result);
          pending->received = true;
        }
        pending->ready.notify_one();
      });
    if (sent != com::types::ERROR_CODE_OK) {
      throw std::runtime_error("SDK could not send instruction request");
    }
    std::unique_lock<std::mutex> lock(pending->mutex);
    if (!pending->ready.wait_for(lock, timeout_, [&pending] {return pending->received;})) {
      ++timeouts_;
      throw ReplyTimeout("Timed out waiting for the sensor reply");
    }
    ++responses_;
    last_response_ = std::chrono::steady_clock::now();
    last_error_.clear();
    if (!pending->result["success"].get<bool>()) {
      ++sensor_rejections_;
      last_error_ = "Sensor rejected one or more instructions";
    }
    return pending->result;
  }

  static constexpr int64_t kMaxTimeoutMs = 4000;
  int64_t startup_can_output_{-1};
  unsigned startup_can_attempts_{};
  std::string startup_can_state_{"unchanged"};
  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};  // Log throttling independent of sim time.
  RuntimeConfig config_{"smartmicro-readback"};
  uint64_t exchanges_{}, invalid_requests_{}, failed_requests_{}, timeouts_{};
  uint64_t responses_{}, sensor_rejections_{};
  std::string last_error_;
  std::chrono::steady_clock::time_point last_response_{};
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  uint32_t sensor_id_{};
  std::chrono::milliseconds timeout_{};
  std::shared_ptr<com::master::CommunicationServicesIface> services_;
  rclcpp::Service<GetMode>::SharedPtr mode_service_;
  rclcpp::Service<GetStatus>::SharedPtr status_service_;
  rclcpp::Service<SetMode>::SharedPtr set_service_;
};

std::shared_ptr<rclcpp::Node> make_readback_node(const rclcpp::NodeOptions & options)
{
  return std::make_shared<ReadbackNode>(options);
}
}  // namespace smartmicro::drivers::radar

RCLCPP_COMPONENTS_REGISTER_NODE(smartmicro::drivers::radar::ReadbackNode)
