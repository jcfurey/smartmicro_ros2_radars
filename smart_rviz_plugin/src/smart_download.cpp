// SPDX-License-Identifier: Apache-2.0
#include "smart_rviz_plugin/smart_download.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <pluginlib/class_list_macros.hpp>

#include "panel_util.hpp"

namespace smart_rviz_plugin
{

SmartDownloadService::SmartDownloadService(QWidget * parent)
: rviz_common::Panel(parent)
{
  initialize_ros();
  setup_ui();
}

SmartDownloadService::~SmartDownloadService()
{
  spin_timer_->stop();
  if (pending_) {
    download_client_->remove_pending_request(pending_id_);
  }
  executor_.remove_node(download_node_);
}

void SmartDownloadService::initialize_ros()
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  download_node_ = std::make_shared<rclcpp::Node>(
    panel_util::unique_node_name("smart_download_gui"),
    rclcpp::NodeOptions().use_global_arguments(false));
  download_client_ = download_node_->create_client<FirmwareDownload>(
    "smart_radar/firmware_download");
  executor_.add_node(download_node_);

  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, this, &SmartDownloadService::tick);
  spin_timer_->start(50);
}

void SmartDownloadService::setup_ui()
{
  file_path_input_ = new QLineEdit(this);
  file_path_input_->setObjectName("firmware_path");
  sensor_id_input_ = new QLineEdit(this);
  sensor_id_input_->setObjectName("sensor_id");
  sensor_id_input_->setPlaceholderText("Sensor ID (decimal or 0x hex)");
  start_download_button_ = new QPushButton("Start Download", this);
  start_download_button_->setObjectName("start_download");
  browse_button_ = new QPushButton("Browse", this);
  response_text_edit_ = new QTextEdit(this);
  response_text_edit_->setObjectName("response");
  response_text_edit_->setReadOnly(true);
  response_text_edit_->setFixedHeight(120);

  // Compact file path layout
  auto * file_layout = new QHBoxLayout;
  file_layout->addWidget(file_path_input_);
  file_layout->addWidget(browse_button_);

  // Main layout
  auto * layout = new QVBoxLayout(this);
  layout->addWidget(new QLabel("Firmware File Path:"));
  layout->addLayout(file_layout);
  layout->addWidget(new QLabel("Sensor ID:"));
  layout->addWidget(sensor_id_input_);
  layout->addWidget(start_download_button_);
  layout->addWidget(response_text_edit_);
  setLayout(layout);

  connect(start_download_button_, &QPushButton::clicked, this, &SmartDownloadService::download_firmware);
  connect(browse_button_, &QPushButton::clicked, this, &SmartDownloadService::browse_file);
}

void SmartDownloadService::report_error(const QString & message)
{
  RCLCPP_ERROR(download_node_->get_logger(), "%s", message.toStdString().c_str());
  response_text_edit_->append("<font color='red'>Error: " + message.toHtmlEscaped() + "</font>");
}

void SmartDownloadService::tick()
{
  if (!rclcpp::ok()) {return;}
  executor_.spin_some(std::chrono::milliseconds(2));
  if (pending_ && std::chrono::steady_clock::now() > deadline_) {
    download_client_->remove_pending_request(pending_id_);
    finish_request();
    report_error("No reply from the firmware download service; the sensor state is unknown.");
  }
}

void SmartDownloadService::finish_request()
{
  pending_ = false;
  start_download_button_->setEnabled(true);
  start_download_button_->setText("Start Download");
}

void SmartDownloadService::download_firmware()
{
  if (pending_) {return;}
  const QString file_path = file_path_input_->text().trimmed();
  if (file_path.isEmpty()) {
    report_error("Please select a firmware file.");
    return;
  }
  const auto sensor_id = panel_util::parse_uint(sensor_id_input_->text());
  if (!sensor_id) {
    report_error("Sensor ID must be an unsigned decimal or 0x-prefixed hexadecimal number.");
    return;
  }
  if (!download_client_->service_is_ready()) {
    report_error("Firmware download service not available. Is the radar node running?");
    return;
  }

  auto request = std::make_shared<FirmwareDownload::Request>();
  request->file_path = file_path.toStdString();
  request->sensor_id = *sensor_id;

  // Delivered by executor_ in tick() on the GUI thread; removed on timeout/destruction.
  pending_id_ = download_client_->async_send_request(request,
    [this](rclcpp::Client<FirmwareDownload>::SharedFuture future) {
      finish_request();
      try {
        const std::string response = future.get()->res;
        response_text_edit_->append("Response: " + QString::fromStdString(response).toHtmlEscaped());
        RCLCPP_INFO(download_node_->get_logger(), "Firmware download response: %s", response.c_str());
      } catch (const std::exception & error) {
        const std::string message = error.what();
        report_error(QString::fromStdString(message));
      }
    }).request_id;
  pending_ = true;
  deadline_ = std::chrono::steady_clock::now() +
    panel_util::request_timeout(this, REQUEST_TIMEOUT);
  start_download_button_->setEnabled(false);
  start_download_button_->setText("Downloading...");
}

void SmartDownloadService::browse_file()
{
  QString file_path = QFileDialog::getOpenFileName(this, tr("Select Firmware File"), "", tr("All Files (*)"));
  if (!file_path.isEmpty()) {
    file_path_input_->setText(file_path);
  }
}

}  // namespace smart_rviz_plugin

PLUGINLIB_EXPORT_CLASS(smart_rviz_plugin::SmartDownloadService, rviz_common::Panel)
