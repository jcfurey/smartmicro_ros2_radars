// SPDX-License-Identifier: Apache-2.0

#include <CommunicationServicesIface.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <umrr_ros2_driver/instruction_reply.hpp>
#include <umrr_ros2_driver/readback_node.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <umrr_ros2_driver/runtime_config.hpp>
#include <umrr_ros2_driver/startup_parameter.hpp>
#include <umrr_ros2_driver/umrr96_tuning.hpp>
#include <umrr_ros2_msgs/srv/get_mode.hpp>
#include <umrr_ros2_msgs/srv/get_status.hpp>
#include <umrr_ros2_msgs/srv/set_mode.hpp>

#include <algorithm>
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
#include <optional>
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

using smartmicro::drivers::radar::declare_diagnostic_names;
using smartmicro::drivers::radar::RuntimeConfig;
using smartmicro::drivers::radar::startup_parameter;

using ValueType = smartmicro::drivers::radar::InstructionValueType;
using smartmicro::drivers::radar::InstructionBatchLease;
using smartmicro::drivers::radar::InstructionItem;

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

// A sensor reply that did not arrive in time; counted separately from failures.
class ReplyTimeout : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

constexpr char kCanOutputName[] = "output_control_target_list_can";
constexpr char kCanOutputSection[] = "auto_interface_0dim";

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
  : ReadbackNode(options, std::make_unique<RuntimeConfig>("smartmicro-readback"))
  {}

  // With an SDK configuration directory activated before rclcpp::init.
  ReadbackNode(const rclcpp::NodeOptions & options, std::unique_ptr<RuntimeConfig> config)
  : Node("smart_radar_readback", options), config_(std::move(config))
  {
    if (!config_) {
      throw std::invalid_argument("ReadbackNode requires an SDK configuration directory");
    }
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

    config_->write("smart_access_config.json", {
        {"name", "UMRR-96 readback"}, {"version", "1.0.0"},
        {"client_id", 0xc0000001u}, {"role", "master"}, {"alive", false},
        {"shared_lib_path", config_->sdk_library_path(SMARTMICRO_SDK_LIBRARY_PATH)},
        {"config_path", config_->path.string()}, {"download_path", ""},
        {"user_interface_name", "base"}, {"user_interface_major_v", 1},
        {"user_interface_minor_v", 0}, {"user_interface_patch_v", 2},
        {"instruction_serialization_type", "can_based"},
        {"data_serialization_type", "can_based"}});
    config_->write("hw_inventory.json", {
        {"name", "Readback socket"}, {"version", "1.1.0"},
        {"hwItems", Json::array({{
            {"type", "eth"}, {"dev_id", 1}, {"iface_name", interface},
            {"ip_address", host_ip}, {"port", host_port}}})}});
    config_->write("routing_table.json", {
        {"name", "Readback route"}, {"version", "1.0.0"},
        {"clients", Json::array({{
            {"client_id", sensor_id_}, {"link_type", "eth"}, {"dev_id", 1},
            {"ip", sensor_ip}, {"port", sensor_port}, {"can_network_id", 0},
            {"instruction_serialization_type", "can_based"},
            {"data_serialization_type", "can_based"},
            {"user_interface_name", "umrr96_t153_automotive"},
            {"user_interface_major_v", 1}, {"user_interface_minor_v", 2},
            {"user_interface_patch_v", 2}}})}});
    config_->activate();
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
        const auto result = write(*request);
        end_startup_after_service_write(*request, result);
        response->res = result.dump(2);
      });
    declare_diagnostic_names(*this);
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
        stat.add("startup_can_target_attempts", startup_can_attempts_);
      });
    if (startup_can_output_ >= 0) {
      startup_can_state_ = "pending";
      next_startup_attempt_ = Clock::now() + std::chrono::seconds(1);
      startup_timer_ = create_wall_timer(
        std::chrono::milliseconds(100), [this]() {step_startup_can_output();});
    }
    RCLCPP_INFO(get_logger(), "Readback ready for sensor %u on %s:%s", sensor_id_,
      host_ip.c_str(), std::to_string(host_port).c_str());
  }

private:
  using Clock = std::chrono::steady_clock;

  // An allocated batch with its requests added, ready to send.
  struct Prepared
  {
    std::unique_ptr<InstructionBatchLease> lease;
    std::vector<InstructionItem> items;
    std::string section;
  };

  // A sent batch awaiting the sensor's reply; the batch stays allocated until then.
  struct InFlight
  {
    Prepared request;
    std::shared_ptr<PendingResponse> pending;
    Clock::time_point deadline;
  };

  // Volatile write (no EEPROM save) verified by a separate read, retried every 5 s
  // until confirmed because the sensor may still be booting. Both are sent without
  // waiting on the executor, so control requests are still answered within their
  // own timeout_ms while the sensor is unreachable (previously each attempt blocked
  // the executor for up to two timeouts). Startup only: a successful write of the
  // same parameter through the service ends it, and a sensor power cycle while this
  // node runs restores the stored value until the node restarts.
  void step_startup_can_output()
  {
    const auto now = Clock::now();
    if (startup_exchange_) {
      std::optional<Json> reply;
      {
        std::lock_guard<std::mutex> lock(startup_exchange_->pending->mutex);
        if (startup_exchange_->pending->received) {
          reply = startup_exchange_->pending->result;
        }
      }
      if (!reply && now < startup_exchange_->deadline) {
        return;
      }
      startup_exchange_.reset();  // Releases the batch.
      Json result;
      if (reply) {
        result = accept(std::move(*reply));
      } else {
        ++timeouts_;
        last_error_ = "Timed out waiting for the sensor reply";
        result = {{"success", false}, {"error", last_error_}};
      }
      if (!startup_verifying_ && result.value("success", false)) {
        start_startup_exchange(true);
      } else {
        finish_startup_attempt(result);
      }
      return;
    }
    if (now >= next_startup_attempt_) {
      ++startup_can_attempts_;
      start_startup_exchange(false);
    }
  }

  // Sends the startup write (verify = false) or its readback.
  void start_startup_exchange(bool verify)
  {
    startup_verifying_ = verify;
    startup_can_state_ = std::string(verify ? "verifying" : "writing") + " (attempt " +
      std::to_string(startup_can_attempts_) + ")";
    const auto sent = guarded(sensor_id_, kCanOutputSection, [&]() -> Json {
          auto request = verify ?
          prepare_read(sensor_id_, kCanOutputSection, {kCanOutputName},
          {GetMode::Request::TYPE_UINT8}, false) :
          prepare_write(startup_can_request());
          auto pending = send(request);
          startup_exchange_ = InFlight{std::move(request), std::move(pending),
            Clock::now() + timeout_};
          return {{"success", true}};
        });
    if (!sent.value("success", false)) {
      finish_startup_attempt(sent);
    }
  }

  void finish_startup_attempt(const Json & result)
  {
    const bool confirmed = startup_verifying_ && result.value("success", false) &&
      result.contains("values") && result["values"].contains(kCanOutputName) &&
      result["values"][kCanOutputName].value("value", int64_t{-1}) == startup_can_output_;
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
    next_startup_attempt_ = Clock::now() + kStartupRetryPeriod;
  }

  SetMode::Request startup_can_request() const
  {
    SetMode::Request request;
    request.sensor_id = sensor_id_;
    request.section_name = kCanOutputSection;
    request.params = {kCanOutputName};
    request.values = {std::to_string(startup_can_output_)};
    request.value_types = {SetMode::Request::TYPE_UINT8};
    return request;
  }

  // A service write of the CAN output accepted by the sensor takes precedence.
  void end_startup_after_service_write(const SetMode::Request & request, const Json & result)
  {
    if (!startup_timer_ || startup_timer_->is_canceled() || !result.value("success", false) ||
      std::find(request.params.begin(), request.params.end(), kCanOutputName) ==
      request.params.end())
    {
      return;
    }
    startup_timer_->cancel();
    startup_exchange_.reset();
    startup_can_state_ = "superseded by a set_radar_mode request";
    RCLCPP_INFO(get_logger(), "Startup CAN target output write superseded by a service request");
  }

  // Runs one request's validation and exchange; errors become the reply JSON and
  // are counted once (invalid request, timeout or failure).
  template<typename Exchange>
  Json guarded(uint32_t sensor_id, const std::string & section, Exchange && exchange)
  {
    const auto failure = [&](const char * error) {
        return Json{{"sensor_id", sensor_id}, {"section", section}, {"success", false},
        {"error", error}};
      };
    try {
      return exchange();
    } catch (const std::invalid_argument & error) {
      ++invalid_requests_;
      return failure(error.what());
    } catch (const ReplyTimeout & error) {
      last_error_ = error.what();  // Counted once, in timeouts_.
      return failure(error.what());
    } catch (const std::exception & error) {
      ++failed_requests_;
      last_error_ = error.what();
      return failure(error.what());
    }
  }

  Json write(const SetMode::Request & request)
  {
    return guarded(request.sensor_id, request.section_name, [&] {
               return exchange(prepare_write(request));
             });
  }

  Json read(
    uint32_t sensor_id, const std::string & section, const std::vector<std::string> & names,
    const std::vector<uint8_t> & requested_types, bool status)
  {
    return guarded(sensor_id, section, [&] {
               return exchange(prepare_read(sensor_id, section, names, requested_types, status));
             });
  }

  Prepared prepare_write(const SetMode::Request & request)
  {
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
    Prepared prepared{std::make_unique<InstructionBatchLease>(instructions, batch), {},
      request.section_name};
    for (size_t i = 0; i < request.params.size(); ++i) {
      const bool floating = std::holds_alternative<float>(values[i]);
      prepared.items.push_back(
        {request.section_name, request.params[i], floating ? ValueType::kF32 : ValueType::kU8});
      const bool added = std::visit([&](auto value) {
            return batch->AddRequest(
            std::make_shared<com::master::SetParamRequest<decltype(value)>>(
            request.section_name, request.params[i], value));
        }, values[i]);
      if (!added) {
        throw std::invalid_argument("SDK rejected tuning parameter: " + request.params[i]);
      }
    }
    return prepared;
  }

  Prepared prepare_read(
    uint32_t sensor_id, const std::string & section, const std::vector<std::string> & names,
    const std::vector<uint8_t> & requested_types, bool status)
  {
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
      ValueType::kF32, ValueType::kU32, ValueType::kU16, ValueType::kU8};
    constexpr std::array<ValueType, 4> status_types = {
      ValueType::kU32, ValueType::kU16, ValueType::kU8, ValueType::kI32};
    auto instructions = services_->GetInstructionService();
    std::shared_ptr<InstructionBatch> batch;
    if (!instructions || !instructions->AllocateInstructionBatch(sensor_id_, batch)) {
      throw std::runtime_error("Could not allocate SDK read request");
    }
    Prepared prepared{std::make_unique<InstructionBatchLease>(instructions, batch), {}, section};
    for (size_t i = 0; i < names.size(); ++i) {
      if (requested_types[i] > 3) {
        throw std::invalid_argument("Invalid readback data type");
      }
      const auto type = (status ? status_types : param_types)[requested_types[i]];
      prepared.items.push_back({section, names[i], type});
      bool added = false;
      switch (type) {
        case ValueType::kF32: added = add_read<float>(batch, status, section, names[i]); break;
        case ValueType::kU32: added = add_read<uint32_t>(batch, status, section, names[i]); break;
        case ValueType::kU16: added = add_read<uint16_t>(batch, status, section, names[i]); break;
        case ValueType::kU8: added = add_read<uint8_t>(batch, status, section, names[i]); break;
        case ValueType::kI32: added = add_read<int32_t>(batch, status, section, names[i]); break;
      }
      if (!added) {
        throw std::invalid_argument("Unknown, duplicate, or incorrectly typed read: " + names[i]);
      }
    }
    return prepared;
  }

  // Sends a prepared batch; the reply arrives on an SDK thread in the returned state.
  std::shared_ptr<PendingResponse> send(const Prepared & request)
  {
    ++exchanges_;
    // Capture owned state only: a late reply must remain safe after a timeout.
    const auto pending = std::make_shared<PendingResponse>();
    const auto sent = services_->GetInstructionService()->SendInstructionBatch(
      request.lease->get(), [pending, items = request.items, section = request.section,
      sensor_id = sensor_id_](
        com::types::ClientId, const std::shared_ptr<ResponseBatch> response) {
        auto result = smartmicro::drivers::radar::decode_instruction_reply(
          response, sensor_id, section, items);
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
    return pending;
  }

  // Sends a service request and waits for the reply, at most timeout_ms (C25).
  Json exchange(Prepared request)
  {
    const auto pending = send(request);
    std::unique_lock<std::mutex> lock(pending->mutex);
    if (!pending->ready.wait_for(lock, timeout_, [&pending] {return pending->received;})) {
      ++timeouts_;
      throw ReplyTimeout("Timed out waiting for the sensor reply");
    }
    return accept(pending->result);
  }

  // Counts a received reply; returns it unchanged.
  Json accept(Json result)
  {
    ++responses_;
    last_response_ = Clock::now();
    last_error_.clear();
    if (!result["success"].get<bool>()) {
      ++sensor_rejections_;
      last_error_ = "Sensor rejected one or more instructions";
    }
    return result;
  }

  static constexpr int64_t kMaxTimeoutMs = 4000;
  static constexpr auto kStartupRetryPeriod = std::chrono::seconds(5);
  int64_t startup_can_output_{-1};
  unsigned startup_can_attempts_{};
  bool startup_verifying_{false};
  Clock::time_point next_startup_attempt_{};
  std::string startup_can_state_{"unchanged"};
  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};  // Log throttling independent of sim time.
  std::unique_ptr<RuntimeConfig> config_;
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
  // Declared last: releases its batch before the SDK services above.
  std::optional<InFlight> startup_exchange_;
};

std::shared_ptr<rclcpp::Node> make_readback_node(const rclcpp::NodeOptions & options)
{
  return std::make_shared<ReadbackNode>(options);
}

std::shared_ptr<rclcpp::Node> make_readback_node(
  const rclcpp::NodeOptions & options, std::unique_ptr<RuntimeConfig> config)
{
  return std::make_shared<ReadbackNode>(options, std::move(config));
}
}  // namespace smartmicro::drivers::radar

RCLCPP_COMPONENTS_REGISTER_NODE(smartmicro::drivers::radar::ReadbackNode)
