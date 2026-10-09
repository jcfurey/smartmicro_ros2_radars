// SPDX-License-Identifier: Apache-2.0
#ifndef UMRR_ROS2_DRIVER__UDP_SOCKET_HEALTH_HPP_
#define UMRR_ROS2_DRIVER__UDP_SOCKET_HEALTH_HPP_

#include <arpa/inet.h>

#include <cstdint>
#include <cstdio>
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

// /proc/net/udp prints a local IPv4 address as the 32-bit address value in host
// byte order (0100007F for 127.0.0.1 on little-endian hosts). Empty if not IPv4.
inline std::string proc_net_udp_address(const std::string & ipv4)
{
  in_addr address{};
  if (inet_pton(AF_INET, ipv4.c_str(), &address) != 1) {return {};}
  char text[9];
  std::snprintf(text, sizeof(text), "%08X", static_cast<unsigned>(address.s_addr));
  return text;
}

// Counters of the process's socket on the given local port. With local_address
// (proc_net_udp_address format) a socket bound to that address is preferred, so
// adapters that share a port on different addresses stay distinguishable; a socket
// bound to the wildcard address still matches when none is bound to it.
inline UdpSocketSnapshot parse_udp_socket_health(
  std::istream & table, const std::set<std::string> & owned_inodes, uint16_t port,
  const std::string & local_address = {})
{
  std::vector<UdpSocketSnapshot> on_port, on_address;
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
      const UdpSocketSnapshot socket{
        true, std::stoull(fields[4].substr(queue_colon + 1), nullptr, 16),
        std::stoull(fields[12]), fields[9]};
      on_port.push_back(socket);
      if (!local_address.empty() && fields[1].compare(0, colon, local_address) == 0) {
        on_address.push_back(socket);
      }
    } catch (const std::exception &) {
      return {};
    }
  }
  // More than one remaining socket is ambiguous (e.g. SO_REUSEPORT). Never
  // silently substitute another process's or adapter's counters.
  const auto & candidates = on_address.empty() ? on_port : on_address;
  return candidates.size() == 1 ? candidates.front() : UdpSocketSnapshot{};
}

// local_ipv4: the adapter's configured address (hw_ip_address), or empty.
inline UdpSocketSnapshot udp_socket_health(uint16_t port, const std::string & local_ipv4 = {})
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
  return parse_udp_socket_health(table, owned, port, proc_net_udp_address(local_ipv4));
}
}  // namespace smartmicro::drivers::radar
#endif  // UMRR_ROS2_DRIVER__UDP_SOCKET_HEALTH_HPP_
