// SPDX-License-Identifier: Apache-2.0
#ifndef UMRR_ROS2_DRIVER__UDP_SOCKET_HEALTH_HPP_
#define UMRR_ROS2_DRIVER__UDP_SOCKET_HEALTH_HPP_

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace smartmicro::drivers::radar
{
struct UdpSocketSnapshot
{
  bool available{false};
  uint64_t receive_memory_bytes{}, drops{};
  std::string inode;
};

inline UdpSocketSnapshot parse_udp_socket_health(
  std::istream & table, const std::set<std::string> & owned_inodes, uint16_t port)
{
  UdpSocketSnapshot result;
  std::string line;
  while (std::getline(table, line)) {
    std::istringstream row(line);
    const std::vector<std::string> fields{
      std::istream_iterator<std::string>(row), std::istream_iterator<std::string>()};
    if (fields.size() < 13 || !owned_inodes.count(fields[9])) {continue;}
    try {
      const auto colon = fields[1].find(':');
      const auto queue_colon = fields[4].find(':');
      if (colon == std::string::npos || queue_colon == std::string::npos ||
        std::stoul(fields[1].substr(colon + 1), nullptr, 16) != port) {continue;}
      // A second matching socket is ambiguous (e.g. SO_REUSEPORT). Never
      // silently substitute another process's or adapter's counters.
      if (result.available) {return {};}
      result = {true, std::stoull(fields[4].substr(queue_colon + 1), nullptr, 16),
        std::stoull(fields[12]), fields[9]};
    } catch (const std::exception &) {
      return {};
    }
  }
  return result;
}

inline UdpSocketSnapshot udp_socket_health(uint16_t port)
{
  std::set<std::string> owned;
  std::error_code error;
  for (std::filesystem::directory_iterator it("/proc/self/fd", error), end;
    !error && it != end; it.increment(error))
  {
    std::error_code link_error;
    const auto link = std::filesystem::read_symlink(it->path(), link_error).string();
    if (!link_error && link.compare(0, 8, "socket:[") == 0 && link.back() == ']') {
      owned.insert(link.substr(8, link.size() - 9));
    }
  }
  if (error) {return {};}
  std::ifstream table("/proc/self/net/udp");
  return parse_udp_socket_health(table, owned, port);
}
}  // namespace smartmicro::drivers::radar
#endif  // UMRR_ROS2_DRIVER__UDP_SOCKET_HEALTH_HPP_
