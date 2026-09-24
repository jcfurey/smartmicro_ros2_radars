// SPDX-License-Identifier: Apache-2.0
#ifndef UMRR_ROS2_DRIVER__STREAM_HEALTH_HPP_
#define UMRR_ROS2_DRIVER__STREAM_HEALTH_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>

namespace smartmicro::drivers::radar
{
// Use steady time for liveness even when ROS time pauses or jumps.
class StreamHealth
{
public:
  using Clock = std::chrono::steady_clock;
  struct Snapshot
  {
    uint64_t frames{}, device_timestamp_us{}, repeated{}, backwards{}, zero{};
    double age_seconds{-1.0}, frequency_hz{};
    bool timestamp_warning{false};
  };

  void receive(uint64_t timestamp, Clock::time_point receipt = Clock::now())
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.frames) {
      state_.repeated += timestamp == state_.device_timestamp_us;
      state_.backwards += timestamp < state_.device_timestamp_us;
    }
    state_.zero += timestamp == 0;
    state_.device_timestamp_us = timestamp;
    ++state_.frames;
    last_receipt_ = receipt;
  }

  Snapshot snapshot(Clock::time_point now = Clock::now())
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = state_;
    if (state_.frames) {
      result.age_seconds = std::chrono::duration<double>(now - last_receipt_).count();
    }
    const auto elapsed = std::chrono::duration<double>(now - last_check_).count();
    result.frequency_hz = elapsed > 0 ? (state_.frames - checked_frames_) / elapsed : 0;
    const auto anomalies = state_.repeated + state_.backwards + state_.zero;
    result.timestamp_warning = anomalies != checked_anomalies_;
    checked_anomalies_ = anomalies;
    checked_frames_ = state_.frames;
    last_check_ = now;
    return result;
  }

private:
  std::mutex mutex_;
  Snapshot state_;
  Clock::time_point last_receipt_{}, last_check_{Clock::now()};
  uint64_t checked_frames_{}, checked_anomalies_{};
};
}  // namespace smartmicro::drivers::radar
#endif
