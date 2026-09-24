// SPDX-License-Identifier: Apache-2.0
// Standalone SDK observer for loopback replay. Never point this at the live socket.
#include <CommunicationServicesIface.h>
#include <umrr96_t153_automotive_v1_2_2/DataStreamServiceIface.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>

using Json = nlohmann::json;
using Stream = com::master::umrr96_t153_automotive_v1_2_2::DataStreamServiceIface;

struct Results
{
  std::mutex mutex;
  std::condition_variable ready;
  Json frames = Json::array();
};

template<typename F>
Json inspect(F getter)
{
  try {
    const auto value = getter();
    if constexpr (std::is_floating_point_v<decltype(value)>) {
      if (!std::isfinite(value)) {return {{"nonfinite", true}};}
    }
    return value;
  } catch (const std::exception & error) {
    return {{"error", error.what()}};
  } catch (...) {
    return {{"error", "non-standard exception"}};
  }
}

int main(int argc, char ** argv)
{
  if (argc != 2) {return 1;}
  const auto services = com::master::CommunicationServicesIface::Get();
  if (!services->Init()) {return 2;}
  const auto monitor = services->GetDeviceMonitorService();
  const auto stream = Stream::Get();
  const auto results = std::make_shared<Results>();
  Json report;
  report["connected_before_replay"] = monitor->IsClientConnected(230739);
  report["service_api_version"] = {
    services->GetMajorVer(), services->GetMinorVer(), services->GetPatchVer()};
  const auto registered = stream->RegisterComTargetListReceiveCallback(230739,
    [results](const auto list, auto client) {
      Json frame;
      const auto port = list->GetPortHeader();
      const auto header = list->GetTargetListHeader();
      frame["client_id"] = client;
      frame["port_version"] = {port->GetPortVersionMajor(), port->GetPortVersionMinor()};
      frame["timestamp_us"] = port->GetTimestamp();
      frame["cycle_time"] = header->GetCycleTime();
      frame["count"] = header->GetNumberOfTargets();
      frame["acquisition_setup"] = inspect([&] {return header->GetAcquisitionSetup();});
      frame["targets"] = Json::array();
      for (const auto & target : list->GetTargetList()) {
        Json point;
#define FIELD(name, accessor) point[name] = inspect([&] {return target->accessor();})
        FIELD("range", GetRange);
        FIELD("speed", GetSpeedRadial);
        FIELD("azimuth", GetAzimuthAngle);
        FIELD("elevation", GetElevationAngle);
        FIELD("power", GetPower);
        FIELD("noise", GetNoise);
        FIELD("rcs", GetRCS);
        FIELD("variance_range", GetVarianceRange);
        FIELD("variance_speed", GetVarianceSpeed);
        FIELD("variance_azimuth", GetVarianceAzimuthAngle);
        FIELD("variance_elevation", GetVarianceElevationAngle);
        FIELD("false_alarm_probability", GetFalseAlarmProbability);
        FIELD("flags", GetFlags);
        FIELD("peak_idx", GetPeakIdx);
#undef FIELD
        frame["targets"].push_back(std::move(point));
      }
      {
        std::lock_guard<std::mutex> lock(results->mutex);
        if (results->frames.size() < 30) {results->frames.push_back(std::move(frame));}
      }
      results->ready.notify_one();
    });
  if (registered != com::types::ERROR_CODE_OK) {return 3;}
  std::cout << "READY" << std::endl;
  {
    std::unique_lock<std::mutex> lock(results->mutex);
    results->ready.wait_for(lock, std::chrono::seconds(12), [&] {
      return results->frames.size() >= 30;
    });
    report["frames"] = results->frames;
  }
  report["connected_after_replay"] = monitor->IsClientConnected(230739);
  // The observer runs in its own process. No unsupported unregister/unload is attempted.
  std::ofstream(argv[1]) << report.dump(2) << '\n';
  return report["frames"].empty() ? 4 : 0;
}
