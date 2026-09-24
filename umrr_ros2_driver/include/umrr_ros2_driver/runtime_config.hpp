// SPDX-License-Identifier: Apache-2.0
#ifndef UMRR_ROS2_DRIVER__RUNTIME_CONFIG_HPP_
#define UMRR_ROS2_DRIVER__RUNTIME_CONFIG_HPP_

#include <nlohmann/json.hpp>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace smartmicro::drivers::radar
{
// The SDK is a process-wide singleton: run data and control in separate processes.
// Installed JSON files are templates only, including in a symlink installation.
class RuntimeConfig
{
public:
  explicit RuntimeConfig(const std::string & prefix)
  {
    auto pattern = (std::filesystem::temp_directory_path() / (prefix + "-XXXXXX")).string();
    const auto directory = mkdtemp(pattern.data());
    if (!directory) {
      throw std::runtime_error("Could not create private SDK configuration directory");
    }
    path = directory;
  }

  RuntimeConfig(const RuntimeConfig &) = delete;
  RuntimeConfig & operator=(const RuntimeConfig &) = delete;

  ~RuntimeConfig()
  {
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }

  void write(const std::string & filename, const nlohmann::json & value) const
  {
    std::ofstream stream;
    stream.exceptions(std::ios::failbit | std::ios::badbit);
    stream.open(path / filename);
    stream << value.dump(2) << '\n';
    stream.flush();
  }

  void activate() const
  {
    const auto filename = (path / "smart_access_config.json").string();
    if (setenv("SMART_ACCESS_CFG_FILE_PATH", filename.c_str(), 1) != 0) {
      throw std::runtime_error("Could not set SDK configuration path");
    }
  }

  std::filesystem::path path;
};
}  // namespace smartmicro::drivers::radar
#endif
