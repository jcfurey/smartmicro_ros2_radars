// SPDX-License-Identifier: Apache-2.0
#ifndef SMART_RVIZ_PLUGIN__SMART_DOWNLOAD_HPP_
#define SMART_RVIZ_PLUGIN__SMART_DOWNLOAD_HPP_

#include <QFileDialog>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <QHBoxLayout>

#include <chrono>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "umrr_ros2_msgs/srv/firmware_download.hpp"

namespace smart_rviz_plugin
{
///
/// @brief      The class for the firmware download onto the sensor.
///
/// This class provides a graphical user interface (GUI) panel for downloading
/// firmware to a sensor within the RViz environment. It extends the rviz_common::Panel
/// class and includes functionalities for browsing firmware files, initiating the
/// download process, and displaying responses from the download service.
///
/// The request is asynchronous: the reply is delivered by the panel's own executor,
/// spun from a QTimer on the GUI thread, so no worker thread outlives the panel.
///
class SmartDownloadService : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit SmartDownloadService(QWidget * parent = nullptr);
  ~SmartDownloadService() override;

private slots:
  /// @brief      Send the firmware download request to the selected sensor.
  void download_firmware();

  /// @brief      Open a file dialog to select a firmware file.
  void browse_file();

private:
  /// @brief      Initializes the ROS2 node, client and executor.
  void initialize_ros();

  /// @brief      Initializes the panel's widgets and connections.
  void setup_ui();

  /// @brief      Spin the executor and expire an overdue request.
  void tick();

  /// @brief      Forget the pending request (a late reply is dropped) and re-enable the UI.
  void finish_request();

  void report_error(const QString & message);

  using FirmwareDownload = umrr_ros2_msgs::srv::FirmwareDownload;

  rclcpp::Node::SharedPtr download_node_;
  rclcpp::Client<FirmwareDownload>::SharedPtr download_client_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  QTimer * spin_timer_{nullptr};
  bool pending_{false};
  int64_t pending_id_{};
  std::chrono::steady_clock::time_point deadline_{};

  QLineEdit * file_path_input_{nullptr};
  QLineEdit * sensor_id_input_{nullptr};
  QPushButton * start_download_button_{nullptr};
  QPushButton * browse_button_{nullptr};
  QTextEdit * response_text_edit_{nullptr};

  /// Flashing may take minutes (the driver waits up to 5 min for the transfer).
  static constexpr auto REQUEST_TIMEOUT = std::chrono::minutes(6);
};

}  // namespace smart_rviz_plugin

#endif  // SMART_RVIZ_PLUGIN__SMART_DOWNLOAD_HPP_
