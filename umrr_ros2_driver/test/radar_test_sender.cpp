// SPDX-License-Identifier: Apache-2.0
// Replay the repository's synthetic UMRR-96 fixture through the real SDK decoder.
#include <DataServicesIface.h>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iterator>
#include <thread>
#include <vector>

namespace
{
volatile std::sig_atomic_t running = 1;
void stop(int) {running = 0;}
}

int main(int argc, char ** argv)
{
  if (argc != 2) {return 1;}
  std::ifstream file(argv[1], std::ios::binary);
  std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)), {});
  if (bytes.empty()) {return 2;}
  auto service = com::common::DataServicesIface::Get();
  if (!service->Init()) {return 3;}
  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);
  com::types::BufferDescriptor buffer(bytes.data(), bytes.size());
  while (running) {
    if (service->StreamDataPort(1, 66, buffer) != com::types::ERROR_CODE_OK) {return 4;}
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}
