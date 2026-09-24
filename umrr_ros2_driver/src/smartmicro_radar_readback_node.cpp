// SPDX-License-Identifier: Apache-2.0

#include <CommunicationServicesIface.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
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

// The SDK is a process-wide singleton. Keep legacy CAN-format instruction
// handling in its own process so the data node can decode Ethernet target ports.
struct RuntimeConfig
{
  RuntimeConfig()
  {
    std::string pattern = "/tmp/smartmicro-readback-XXXXXX";
    const auto directory = mkdtemp(pattern.data());
    if (!directory) {
      throw std::runtime_error("Could not create readback SDK configuration directory");
    }
    path = directory;
  }

  ~RuntimeConfig()
  {
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }

  void write(const std::string & filename, const Json & value) const
  {
    std::ofstream stream;
    stream.exceptions(std::ios::failbit | std::ios::badbit);
    stream.open(path / filename);
    stream << value.dump(2) << '\n';
    stream.flush();
  }

  std::filesystem::path path;
};

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

struct PendingResponse
{
  std::mutex mutex;
  std::condition_variable ready;
  bool received{false};
  Json result;
};

class ReadbackNode : public rclcpp::Node
{
public:
  ReadbackNode()
  : Node("smart_radar_readback")
  {
    const auto sensor_id = declare_parameter<int64_t>("sensor_id", 0);
    const auto host_port = declare_parameter<int64_t>("host_port", 55556);
    const auto sensor_port = declare_parameter<int64_t>("sensor_port", 55555);
    const auto timeout_ms = declare_parameter<int64_t>("timeout_ms", 2000);
    const auto interface = declare_parameter<std::string>("interface_name", "enp68s0f0");
    const auto host_ip = declare_parameter<std::string>("host_ip", "192.168.11.17");
    const auto sensor_ip = declare_parameter<std::string>("sensor_ip", "192.168.11.11");
    if (sensor_id <= 0 || sensor_id > UINT32_MAX || host_port <= 0 || host_port > 65535 ||
      sensor_port <= 0 || sensor_port > 65535 || timeout_ms <= 0 || timeout_ms > 30000)
    {
      throw std::invalid_argument("Invalid sensor_id, UDP port, or timeout_ms");
    }
    sensor_id_ = static_cast<uint32_t>(sensor_id);
    timeout_ = std::chrono::milliseconds(timeout_ms);

    config_.write("smart_access_config.json", {
      {"name", "UMRR-96 readback"}, {"version", "1.0.0"},
      {"client_id", 0xc0000001u}, {"role", "master"}, {"alive", false},
      {"shared_lib_path", SMARTMICRO_SDK_LIBRARY_PATH},
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
    const auto config_path = (config_.path / "smart_access_config.json").string();
    if (setenv("SMART_ACCESS_CFG_FILE_PATH", config_path.c_str(), 1) != 0) {
      throw std::runtime_error("Could not set SDK configuration path");
    }
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
    RCLCPP_INFO(get_logger(), "Readback ready for sensor %u on %s:%ld", sensor_id_,
      host_ip.c_str(), static_cast<long>(host_port));
  }

private:
  Json write(const SetMode::Request & request)
  {
    // Bench tuning controls only. Network, persistence, and reset commands are
    // deliberately outside this service's scope. Bounds follow the UMRR-96 UIF.
    static const std::map<std::string, unsigned> limits = {
      {"tx_antenna_idx", 2}, {"frequency_sweep_idx", 2}, {"range_toggle_mode", 3},
      {"output_control_target_list_can", 1}};
    try {
      if (request.sensor_id != sensor_id_ || request.section_name != "auto_interface_0dim") {
        throw std::invalid_argument("Invalid tuning sensor ID or section");
      }
      if (request.params.empty() || request.params.size() > 10 ||
        request.params.size() != request.values.size() ||
        request.params.size() != request.value_types.size())
      {
        throw std::invalid_argument("Supply 1–10 names, values and types with equal lengths");
      }
      if (std::set<std::string>(request.params.begin(), request.params.end()).size() !=
        request.params.size())
      {
        throw std::invalid_argument("Tuning names must be unique");
      }
      auto instructions = services_->GetInstructionService();
      std::shared_ptr<InstructionBatch> batch;
      if (!instructions || !instructions->AllocateInstructionBatch(sensor_id_, batch)) {
        throw std::runtime_error("Could not allocate SDK tuning request");
      }
      const BatchLease lease{instructions, batch};
      for (size_t i = 0; i < request.params.size(); ++i) {
        const auto bound = limits.find(request.params[i]);
        if (bound == limits.end() || request.value_types[i] != 3) {
          throw std::invalid_argument("Unsupported tuning parameter or type: " + request.params[i]);
        }
        unsigned value{};
        const auto & string = request.values[i];
        const auto parsed = std::from_chars(string.data(), string.data() + string.size(), value);
        if (parsed.ec != std::errc{} || parsed.ptr != string.data() + string.size() ||
          value > bound->second)
        {
          throw std::invalid_argument("Invalid tuning value for " + request.params[i]);
        }
        if (!batch->AddRequest(std::make_shared<com::master::SetParamRequest<uint8_t>>(
            request.section_name, request.params[i], static_cast<uint8_t>(value))))
        {
          throw std::invalid_argument("SDK rejected tuning parameter: " + request.params[i]);
        }
      }
      return exchange(batch, request.params,
        std::vector<ValueType>(request.params.size(), ValueType::U8), request.section_name);
    } catch (const std::exception & error) {
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
    } catch (const std::exception & error) {
      return {{"sensor_id", sensor_id}, {"section", section},
        {"success", false}, {"error", error.what()}};
    }
  }

  Json exchange(
    const std::shared_ptr<InstructionBatch> & batch, const std::vector<std::string> & names,
    const std::vector<ValueType> & types, const std::string & section)
  {
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
      throw std::runtime_error("Timed out waiting for the sensor reply");
    }
    return pending->result;
  }

  RuntimeConfig config_;
  uint32_t sensor_id_{};
  std::chrono::milliseconds timeout_{};
  std::shared_ptr<com::master::CommunicationServicesIface> services_;
  rclcpp::Service<GetMode>::SharedPtr mode_service_;
  rclcpp::Service<GetStatus>::SharedPtr status_service_;
  rclcpp::Service<SetMode>::SharedPtr set_service_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<ReadbackNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("smart_radar_readback"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
