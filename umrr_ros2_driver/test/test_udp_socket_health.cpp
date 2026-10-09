// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <umrr_ros2_driver/udp_socket_health.hpp>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

using smartmicro::drivers::radar::parse_udp_socket_health;
using smartmicro::drivers::radar::proc_net_udp_address;
using smartmicro::drivers::radar::udp_socket_health;

TEST(UdpSocketHealth, MatchesOwnedSocketAndDoesNotClaimUnavailableCountersAreZero)
{
  const std::string row = "1: 110BA8C0:D903 00000000:0000 07 00000000:00000300 "
    "00:00000000 00000000 1000 0 123 2 0000000000000000 7\n";
  std::istringstream table(row);
  const auto sample = parse_udp_socket_health(table, {"123"}, 55555);
  ASSERT_TRUE(sample.available);
  EXPECT_EQ(sample.drops, 7U);
  EXPECT_EQ(sample.receive_memory_bytes, 768U);
  std::istringstream foreign(row), wrong_port(row), ambiguous(row + row), malformed("garbage\n");
  EXPECT_FALSE(parse_udp_socket_health(foreign, {"456"}, 55555).available);
  EXPECT_FALSE(parse_udp_socket_health(wrong_port, {"123"}, 55556).available);
  EXPECT_FALSE(parse_udp_socket_health(ambiguous, {"123"}, 55555).available);
  EXPECT_FALSE(parse_udp_socket_health(malformed, {"123"}, 55555).available);
}

TEST(UdpSocketHealth, LocalAddressSeparatesAdaptersOnTheSamePort)
{
  // Two adapters on port 55555 bound to 127.0.0.1 and 127.0.0.2, plus a wildcard row.
  const std::string first = "1: 0100007F:D903 00000000:0000 07 00000000:00000100 "
    "00:00000000 00000000 1000 0 123 2 0000000000000000 3\n";
  const std::string second = "2: 0200007F:D903 00000000:0000 07 00000000:00000200 "
    "00:00000000 00000000 1000 0 124 2 0000000000000000 4\n";
  const std::string wildcard = "3: 00000000:D903 00000000:0000 07 00000000:00000300 "
    "00:00000000 00000000 1000 0 125 2 0000000000000000 5\n";
  const std::set<std::string> owned{"123", "124", "125"};
  if (htonl(1) == 1) {
    GTEST_SKIP() << "The synthetic rows use the little-endian /proc/net/udp format";
  }
  EXPECT_EQ(proc_net_udp_address("127.0.0.1"), "0100007F");
  EXPECT_EQ(proc_net_udp_address("not an address"), "");
  std::istringstream both(first + second), by_second(first + second), missing(first + second);
  EXPECT_FALSE(parse_udp_socket_health(both, owned, 55555).available);  // Ambiguous.
  const auto selected = parse_udp_socket_health(
    by_second, owned, 55555, proc_net_udp_address("127.0.0.2"));
  ASSERT_TRUE(selected.available);
  EXPECT_EQ(selected.inode, "124");
  EXPECT_EQ(selected.drops, 4U);
  EXPECT_EQ(selected.receive_memory_bytes, 512U);
  // An address no socket is bound to leaves both candidates: still ambiguous.
  EXPECT_FALSE(
    parse_udp_socket_health(missing, owned, 55555, proc_net_udp_address("127.0.0.3")).available);
  // A single socket bound to the wildcard address still matches a configured address.
  std::istringstream only_wildcard(wildcard), duplicated(second + second);
  const auto any = parse_udp_socket_health(only_wildcard, owned, 55555,
    proc_net_udp_address("127.0.0.1"));
  ASSERT_TRUE(any.available);
  EXPECT_EQ(any.inode, "125");
  EXPECT_FALSE(
    parse_udp_socket_health(duplicated, owned, 55555, proc_net_udp_address("127.0.0.2")).available);
}

TEST(UdpSocketHealth, MeasuresActualLoopbackReceiveBufferOverflow)
{
  struct Socket {int fd{socket(AF_INET, SOCK_DGRAM, 0)}; ~Socket() {if (fd >= 0) {close(fd);}}};
  Socket receiver, sender;
  ASSERT_GE(receiver.fd, 0);
  ASSERT_GE(sender.fd, 0);
  int buffer = 1024;
  ASSERT_EQ(setsockopt(receiver.fd, SOL_SOCKET, SO_RCVBUF, &buffer, sizeof(buffer)), 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  ASSERT_EQ(bind(receiver.fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)), 0);
  socklen_t size = sizeof(address);
  ASSERT_EQ(getsockname(receiver.fd, reinterpret_cast<sockaddr *>(&address), &size), 0);
  const auto before = udp_socket_health(ntohs(address.sin_port));
  ASSERT_TRUE(before.available);
  EXPECT_EQ(before.drops, 0U);
  const char data[256]{};
  for (unsigned i = 0; i < 100; ++i) {
    ASSERT_EQ(sendto(sender.fd, data, sizeof(data), 0,
      reinterpret_cast<sockaddr *>(&address), sizeof(address)), sizeof(data));
  }
  const auto after = udp_socket_health(ntohs(address.sin_port));
  ASSERT_TRUE(after.available);
  EXPECT_EQ(after.inode, before.inode);
  EXPECT_GT(after.drops, 0U);
  EXPECT_GT(after.receive_memory_bytes, 0U);
}

TEST(UdpSocketHealth, SelectsTheLiveSocketBoundToTheAdapterAddress)
{
  struct Socket {int fd{socket(AF_INET, SOCK_DGRAM, 0)}; ~Socket() {if (fd >= 0) {close(fd);}}};
  Socket first, second;
  ASSERT_GE(first.fd, 0);
  ASSERT_GE(second.fd, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  ASSERT_EQ(bind(first.fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)), 0);
  socklen_t size = sizeof(address);
  ASSERT_EQ(getsockname(first.fd, reinterpret_cast<sockaddr *>(&address), &size), 0);
  const auto port = ntohs(address.sin_port);
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK + 1);  // 127.0.0.2, same port.
  ASSERT_EQ(bind(second.fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)), 0);
  EXPECT_FALSE(udp_socket_health(port).available);
  const auto one = udp_socket_health(port, "127.0.0.1");
  const auto two = udp_socket_health(port, "127.0.0.2");
  ASSERT_TRUE(one.available);
  ASSERT_TRUE(two.available);
  EXPECT_NE(one.inode, two.inode);
}
