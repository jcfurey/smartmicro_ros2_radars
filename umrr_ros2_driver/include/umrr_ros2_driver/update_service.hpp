// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// The initial version of the code was developed by Apex.AI and
// was thereafter adapted and extended by smartmicro.

#ifndef UMRR_ROS2_DRIVER__UPDATE_SERVICE_HPP_
#define UMRR_ROS2_DRIVER__UPDATE_SERVICE_HPP_

#include <CommunicationServicesIface.h>
#include <Instruction.h>
#include <InstructionBatch.h>
#include <InstructionServiceIface.h>
#include <UpdateServiceIface.h>

#include <rclcpp/rclcpp.hpp>
#include <umrr_ros2_driver/sdk_callback_gate.hpp>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>

///
/// @brief  Result of a firmware update request.
///
enum class UpdateResult
{
  kSuccess,             ///< The firmware update completed successfully.
  kBusy,                ///< Another firmware update is in progress.
  kFileOpenError,       ///< The firmware image could not be opened.
  kFileSizeError,       ///< The firmware image size could not be determined.
  kServiceUnavailable,  ///< The Smart Access update service is unavailable.
  kStartFailed,         ///< Starting the firmware update failed.
  kTimeout,             ///< The firmware update timed out.
  kStoppedByMaster,     ///< The master stopped the firmware update.
  kStoppedBySlave,      ///< The sensor stopped the firmware update.
  kBlockRepeatError,    ///< Repeated transfer blocks exceeded the limit.
  kImageInvalid,        ///< The firmware image is invalid.
  kUnknownError         ///< The firmware update failed for an unknown reason.
};

///
/// @brief  Coordinates firmware updates through the Smart Access update service.
///
class UpdateService
{
public:
  ///
  /// @brief  Constructs a firmware update service.
  ///
  UpdateService();
  ~UpdateService() {callback_gate_.close();}
  ///
  /// @brief  Starts a firmware update and blocks until the sensor reports a
  ///         final (non-RUNNING) status, the timeout expires or Cancel() is called.
  ///
  /// Call from a worker thread, never from an executor callback: the wait can
  /// last several minutes.
  ///
  /// @param[in]  client_id     Client identifier of the target sensor.
  /// @param[in]  update_image  Path to the firmware image.
  /// @return     The result of the firmware update request.
  ///
  UpdateResult StartSoftwareUpdate(
    com::types::ClientId client_id,
    const std::string & update_image);

  ///
  /// @brief  Whether an update is currently running.
  ///
  bool Busy() const;

  ///
  /// @brief  Aborts a running update and rejects new ones (used at shutdown).
  ///
  void Cancel();

private:
  ///
  /// @brief  Stores a firmware update progress notification.
  ///
  /// @param[in]  info  The latest update status received from Smart Access.
  ///
  void UpdateCallback(com::types::SWUpdateInfo & info);
  ///
  /// @brief  Maps the latest Smart Access update status to an update result.
  ///
  /// @return  The corresponding firmware update result.
  ///
  UpdateResult HandleResult();
  smartmicro::drivers::radar::SdkCallbackGate callback_gate_;
  com::types::SWUpdateInfo updateInfo_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  bool update_in_progress_{false};
  bool cancelled_{false};
  std::chrono::steady_clock::time_point last_progress_log_{};
};

#endif  // UMRR_ROS2_DRIVER__UPDATE_SERVICE_HPP_
