// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <umrr_ros2_driver/runtime_config.hpp>
#include <umrr_ros2_driver/stream_health.hpp>
#include <umrr_ros2_driver/sdk_callback_gate.hpp>
#include <fstream>
#include <future>
#include <string>
#include <thread>
#include <vector>

using smartmicro::drivers::radar::RuntimeConfig;
using smartmicro::drivers::radar::StreamHealth;
using smartmicro::drivers::radar::SdkCallbackGate;

TEST(RuntimeConfig, IndependentDirectoriesAndExceptionCleanup)
{
  std::filesystem::path first, second;
  {
    RuntimeConfig outer("smartmicro-test");
    first = outer.path;
    outer.write("routing_table.json", {{"port", 12345}});
    try {
      RuntimeConfig inner("smartmicro-test");
      second = inner.path;
      ASSERT_NE(first, second);
      inner.write("routing_table.json", {{"port", 54321}});
      EXPECT_EQ(nlohmann::json::parse(std::ifstream(first / "routing_table.json"))["port"],
        12345);
      throw std::runtime_error("constructor failure");
    } catch (const std::runtime_error &) {}
    EXPECT_FALSE(std::filesystem::exists(second));
    EXPECT_TRUE(std::filesystem::exists(first));
  }
  EXPECT_FALSE(std::filesystem::exists(first));
}

TEST(RuntimeConfig, ShortSdkLibraryAliasIsRemovedWithoutTarget)
{
  RuntimeConfig target_owner("smartmicro-test-target");
  const auto target = target_owner.path / std::string(200, 'x');
  std::filesystem::create_directory(target);
  std::ofstream(target / "libsmart_access.so") << "stub";
  std::filesystem::path alias;
  {
    RuntimeConfig config("smartmicro-test");
    alias = config.sdk_library_path(target);
    EXPECT_LE(alias.string().size(), RuntimeConfig::kMaxSdkLibraryPathLength);
    EXPECT_TRUE(std::filesystem::exists(alias / "libsmart_access.so"));
  }
  EXPECT_FALSE(std::filesystem::exists(alias));
  EXPECT_TRUE(std::filesystem::exists(target / "libsmart_access.so"));
}

TEST(StreamHealth, SilenceRecoveryAndCounterReset)
{
  StreamHealth health;
  const auto start = StreamHealth::Clock::now();
  const auto initial = health.snapshot(start);
  EXPECT_EQ(initial.frames, 0U);
  EXPECT_LT(initial.age_seconds, 0);
  health.receive(652000000, start);
  health.receive(652100000, start + std::chrono::milliseconds(100));
  const auto active = health.snapshot(start + std::chrono::seconds(1));
  EXPECT_EQ(active.frames, 2U);
  EXPECT_DOUBLE_EQ(active.frequency_hz, 2.0);
  EXPECT_DOUBLE_EQ(active.age_seconds, .9);
  EXPECT_FALSE(active.timestamp_warning);
  const auto silent = health.snapshot(start + std::chrono::seconds(4));
  EXPECT_DOUBLE_EQ(silent.frequency_hz, 0);
  EXPECT_DOUBLE_EQ(silent.age_seconds, 3.9);
  health.receive(0, start + std::chrono::seconds(5));
  health.receive(0, start + std::chrono::seconds(6));
  const auto reset = health.snapshot(start + std::chrono::seconds(6));
  EXPECT_TRUE(reset.timestamp_warning);
  EXPECT_EQ(reset.backwards, 1U);
  EXPECT_EQ(reset.repeated, 1U);
  EXPECT_EQ(reset.zero, 2U);
  health.receive(100000, start + std::chrono::seconds(7));
  const auto recovered = health.snapshot(start + std::chrono::seconds(7));
  EXPECT_FALSE(recovered.timestamp_warning);
  EXPECT_EQ(recovered.backwards, 1U);  // Lifetime counter remains available.
  EXPECT_EQ(recovered.device_timestamp_us, 100000U);
}

TEST(StreamHealth, DelayVariationDoesNotPretendToMeasureAbsoluteLatency)
{
  StreamHealth health;
  const auto start = StreamHealth::Clock::now();
  health.receive(1000000, start);
  health.receive(1100000, start + std::chrono::milliseconds(140));
  const auto delayed = health.snapshot(start + std::chrono::milliseconds(150));
  EXPECT_NEAR(delayed.receive_interval_seconds, .14, 1e-9);
  EXPECT_NEAR(delayed.device_interval_seconds, .1, 1e-9);
  EXPECT_NEAR(delayed.relative_delay_change_seconds, .04, 1e-9);
  EXPECT_NEAR(delayed.max_positive_delay_change_seconds, .04, 1e-9);
  health.receive(1200000, start + std::chrono::milliseconds(200));
  const auto caught_up = health.snapshot(start + std::chrono::milliseconds(210));
  EXPECT_NEAR(caught_up.relative_delay_change_seconds, -.04, 1e-9);
  EXPECT_DOUBLE_EQ(caught_up.max_positive_delay_change_seconds, 0);
  health.receive(0, start + std::chrono::milliseconds(300));
  EXPECT_LT(health.snapshot().device_interval_seconds, 0);
}

TEST(SdkCallbackGate, DrainsAnActiveCallAndDropsLateCalls)
{
  SdkCallbackGate gate;
  std::promise<void> entered, release;
  auto release_future = release.get_future().share();
  unsigned calls = 0;
  auto callback = gate.wrap([&] {
      ++calls;
      entered.set_value();
      release_future.wait();
    });
  std::thread worker(callback);
  entered.get_future().wait();
  auto closing = std::async(std::launch::async, [&] {gate.close();});
  EXPECT_EQ(closing.wait_for(std::chrono::milliseconds(20)), std::future_status::timeout);
  release.set_value();
  worker.join();
  closing.get();
  callback();
  EXPECT_EQ(calls, 1U);
  gate.close();  // Shutdown and destructor may both close it.
}

TEST(SdkCallbackGate, RetainedCallbackAndShutdownHookOutliveOwner)
{
  unsigned calls = 0;
  std::function<void()> callback, shutdown;
  {
    SdkCallbackGate gate;
    callback = gate.wrap([&] {++calls;});
    shutdown = gate.shutdown_callback();
    callback();
  }
  callback();
  shutdown();
  EXPECT_EQ(calls, 1U);
}

TEST(SdkCallbackGate, ExceptionIsContainedReportedAndReleasesLease)
{
  SdkCallbackGate gate;
  std::vector<std::string> reports;
  gate.set_error_handler([&](const std::string & message) {reports.push_back(message);});
  auto callback = gate.wrap([] {throw std::runtime_error("SDK callback failure");});
  auto unknown = gate.wrap([] {throw 42;});
  EXPECT_NO_THROW(callback());
  EXPECT_NO_THROW(callback());  // Throttled: counted, not reported again within 1 s.
  EXPECT_NO_THROW(unknown());
  EXPECT_EQ(gate.exception_count(), 3U);
  ASSERT_EQ(reports.size(), 1U);
  EXPECT_NE(reports.front().find("SDK callback failure"), std::string::npos);
  gate.close();  // The lease was released despite the exceptions.
  EXPECT_NO_THROW(callback());
  EXPECT_EQ(gate.exception_count(), 3U);
}
