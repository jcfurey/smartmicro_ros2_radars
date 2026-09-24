// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <umrr_ros2_driver/udp_socket_health.hpp>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

using smartmicro::drivers::radar::parse_udp_socket_health;
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
