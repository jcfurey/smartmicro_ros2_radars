// SPDX-License-Identifier: Apache-2.0
// Smoke test for the generic Smart panels: unique node names, strict input parsing,
// non-blocking service calls with deadlines, installed instruction tables, lazy topic
// subscriptions with periodic discovery, and the bounded recorder.
#include <QApplication>
#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QTextEdit>
#include <QTimer>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <umrr_ros2_msgs/msg/port_fault_reports_msg.hpp>
#include <umrr_ros2_msgs/msg/port_target_header.hpp>
#include <umrr_ros2_msgs/srv/firmware_download.hpp>
#include <umrr_ros2_msgs/srv/get_mode.hpp>
#include <umrr_ros2_msgs/srv/send_command.hpp>
#include <umrr_ros2_msgs/srv/set_mode.hpp>

using Clock = std::chrono::steady_clock;
using SendCommand = umrr_ros2_msgs::srv::SendCommand;
using SetMode = umrr_ros2_msgs::srv::SetMode;
using FirmwareDownload = umrr_ros2_msgs::srv::FirmwareDownload;

namespace
{
void check(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}

template<typename T>
T * child(const std::shared_ptr<rviz_common::Panel> & panel, const char * name)
{
  auto widget = panel->findChild<T *>(name);
  check(widget != nullptr, std::string("Widget missing: ") + name);
  return widget;
}

sensor_msgs::msg::PointCloud2 target_cloud(size_t points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp.sec = 42;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  using PF = sensor_msgs::msg::PointField;
  modifier.setPointCloud2Fields(20,
    "x", 1, PF::FLOAT32, "y", 1, PF::FLOAT32, "z", 1, PF::FLOAT32,
    "radial_speed", 1, PF::FLOAT32, "power", 1, PF::FLOAT32, "rcs", 1, PF::FLOAT32,
    "noise", 1, PF::FLOAT32, "snr", 1, PF::FLOAT32, "azimuth_angle", 1, PF::FLOAT32,
    "elevation_angle", 1, PF::FLOAT32, "range", 1, PF::FLOAT32,
    "variance_range", 1, PF::FLOAT32, "variance_speed", 1, PF::FLOAT32,
    "variance_azimuth_angle", 1, PF::FLOAT32, "variance_elevation_angle", 1, PF::FLOAT32,
    "false_alarm_probability", 1, PF::FLOAT32, "flags", 1, PF::UINT32,
    "peak_idx", 1, PF::UINT16, "pad0", 1, PF::UINT16, "pad1", 1, PF::UINT32);
  modifier.resize(points);
  sensor_msgs::PointCloud2Iterator<float> range(cloud, "range");
  for (size_t i = 0; i < points; ++i, ++range) {*range = static_cast<float>(i);}
  return cloud;
}
}  // namespace

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<rclcpp::Node>("generic_panels_fixture");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    bool spin_server = true;
    auto wait = [&](std::function<bool()> done, double seconds, const std::string & what) {
        const auto end = Clock::now() + std::chrono::duration<double>(seconds);
        while (!done() && Clock::now() < end) {
          if (spin_server) {executor.spin_some(std::chrono::milliseconds(2));}
          app.processEvents();
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        check(done(), "Timed out waiting for: " + what);
      };

    std::vector<SendCommand::Request> commands;
    std::vector<SetMode::Request> modes;
    std::vector<FirmwareDownload::Request> downloads;
    auto command_service = node->create_service<SendCommand>("/smart_radar/send_command",
      [&](SendCommand::Request::SharedPtr request, SendCommand::Response::SharedPtr response) {
        commands.push_back(*request);
        response->res = "command ok";
      });
    auto mode_service = node->create_service<SetMode>("/smart_radar/set_radar_mode",
      [&](SetMode::Request::SharedPtr request, SetMode::Response::SharedPtr response) {
        modes.push_back(*request);
        response->res = "mode ok";
      });
    auto download_service = node->create_service<FirmwareDownload>(
      "/smart_radar/firmware_download",
      [&](FirmwareDownload::Request::SharedPtr request, FirmwareDownload::Response::SharedPtr response) {
        downloads.push_back(*request);
        response->res = "download ok";
      });

    unsigned heartbeats = 0;
    QTimer heartbeat;
    QObject::connect(&heartbeat, &QTimer::timeout, [&] {++heartbeats;});
    heartbeat.start(10);

    pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common", "rviz_common::Panel");
    const std::vector<std::string> generic = {
      "smart_rviz_plugin/Smart Recorder", "smart_rviz_plugin/Smart Command Configurator",
      "smart_rviz_plugin/Smart Firmware Download", "smart_rviz_plugin/Smart Status",
      "smart_rviz_plugin/Smart Fault Reports"};

    // C17: two instances of every panel must not produce duplicate node names.
    {
      std::vector<std::shared_ptr<rviz_common::Panel>> twins;
      for (const auto & name : generic) {
        twins.push_back(loader.createSharedInstance(name));
        twins.push_back(loader.createSharedInstance(name));
      }
      std::vector<std::string> names;
      wait([&] {
          names.clear();
          for (const auto & n : node->get_node_names()) {
            if (n.find("gui") != std::string::npos) {names.push_back(n);}
          }
          return names.size() >= 10;
        }, 10, "panel nodes in the graph");
      check(std::set<std::string>(names.begin(), names.end()).size() == names.size(),
        "Duplicate panel node names");
    }

    // --- Command Configurator (C11, C12, C13) ---
    auto services = loader.createSharedInstance("smart_rviz_plugin/Smart Command Configurator");
    services->setProperty("request_timeout_ms", 1500);
    auto response = child<QTextEdit>(services, "response");
    auto sensor_type = child<QComboBox>(services, "sensor_type");
    sensor_type->setCurrentIndex(1);
    Q_EMIT sensor_type->activated(1);  // UMRR9F MSE; the fixture provides its tables.
    check(child<QTableWidget>(services, "param_table")->rowCount() == 3 &&
      child<QTableWidget>(services, "command_table")->rowCount() == 2 &&
      child<QTableWidget>(services, "status_table")->rowCount() == 1,
      "Instruction tables not loaded from SMART_USER_INTERFACES_DIR: " +
      response->toPlainText().toStdString());
    response->clear();
    sensor_type->setCurrentIndex(6);
    Q_EMIT sensor_type->activated(6);  // UMRRA1: absent from the fixture.
    check(child<QTableWidget>(services, "param_table")->rowCount() > 0 ||
      response->toPlainText().contains("not found"), "Missing tables not reported in the panel");

    auto command_name = child<QLineEdit>(services, "command_name");
    auto command_value = child<QLineEdit>(services, "command_value");
    auto command_sensor = child<QLineEdit>(services, "command_sensor_id");
    auto send_command = child<QPushButton>(services, "send_command");
    command_name->setText("comp_eeprom_ctrl_save_param_sec");
    for (const auto & [id, value] : std::vector<std::pair<const char *, const char *>>{
        {"-1", "1"}, {"12abc", "1"}, {"99999999999", "1"}, {"0x", "1"}, {"", "1"},
        {"7", "abc"}, {"7", "nan"}, {"7", "1e99"}, {"7", ""}})
    {
      response->clear();
      command_sensor->setText(id);
      command_value->setText(value);
      send_command->click();
      check(response->toPlainText().contains("must"),
        std::string("Invalid input not reported: ") + id + " / " + value);
    }
    child<QLineEdit>(services, "param_name")->setText("frequency_sweep_idx");
    child<QLineEdit>(services, "param_sensor_id")->setText("7");
    child<QComboBox>(services, "param_value_type")->setCurrentIndex(3);  // uint8
    child<QLineEdit>(services, "param_value")->setText("300");
    response->clear();
    child<QPushButton>(services, "send_param")->click();
    check(response->toPlainText().contains("not a valid"), "Out-of-range uint8 accepted");
    wait([&] {return true;}, .2, "");
    check(commands.empty() && modes.empty(), "Invalid input reached a service");

    // A valid request: hex ID and a fractional float value (not truncated).
    wait([&] {
        response->clear();
        command_sensor->setText("0x10");
        command_value->setText("2.5");
        send_command->click();
        const bool unavailable = response->toPlainText().contains("not available");
        if (!unavailable) {return true;}
        for (int i = 0; i < 20; ++i) {executor.spin_some(std::chrono::milliseconds(2));
          app.processEvents(); std::this_thread::sleep_for(std::chrono::milliseconds(5));}
        return false;
      }, 10, "command service discovered by the panel");
    wait([&] {return response->toPlainText().contains("command ok");}, 5, "command reply");
    check(commands.size() == 1 && commands[0].sensor_id == 16 && commands[0].value == 2.5f,
      "Command request fields wrong");
    child<QLineEdit>(services, "param_value")->setText("0x2A");
    child<QComboBox>(services, "param_value_type")->setCurrentIndex(1);  // uint32
    child<QPushButton>(services, "send_param")->click();
    wait([&] {return response->toPlainText().contains("mode ok");}, 5, "set mode reply");
    check(modes.size() == 1 && modes[0].values == std::vector<std::string>{"42"} &&
      modes[0].value_types == std::vector<uint8_t>{1}, "SetMode not sent as canonical decimal");

    // C11: a server that never answers must not block Qt, and must time out.
    spin_server = false;
    heartbeats = 0;
    response->clear();
    send_command->click();
    check(!send_command->isEnabled(), "Send not disabled while a request is pending");
    wait([&] {return response->toPlainText().contains("timed out");}, 5, "request timeout");
    check(heartbeats > 50, "Qt event loop blocked by the service call");
    check(send_command->isEnabled(), "Send not re-enabled after timeout");
    spin_server = true;
    wait([&] {return commands.size() == 2;}, 3, "late request served");
    wait([&] {return true;}, .3, "");
    check(!response->toPlainText().contains("command ok"), "Late reply delivered after timeout");
    // Destroy with a pending request; the late reply must be dropped safely.
    spin_server = false;
    send_command->click();
    services.reset();
    spin_server = true;
    wait([&] {return commands.size() == 3;}, 3, "late reply after destruction");
    wait([&] {return true;}, .2, "");

    // --- Firmware Download (C15) ---
    auto download = loader.createSharedInstance("smart_rviz_plugin/Smart Firmware Download");
    auto download_response = child<QTextEdit>(download, "response");
    auto start = child<QPushButton>(download, "start_download");
    child<QLineEdit>(download, "firmware_path")->setText("/tmp/firmware.bin");
    for (const char * bad : {"abc", "-3", "1.5"}) {
      download_response->clear();
      child<QLineEdit>(download, "sensor_id")->setText(bad);
      start->click();
      check(download_response->toPlainText().contains("must"), "Invalid download ID accepted");
    }
    child<QLineEdit>(download, "sensor_id")->setText("0");  // 0 is a valid ID.
    wait([&] {
        download_response->clear();
        start->click();
        if (!download_response->toPlainText().contains("not available")) {return true;}
        for (int i = 0; i < 20; ++i) {executor.spin_some(std::chrono::milliseconds(2));
          app.processEvents(); std::this_thread::sleep_for(std::chrono::milliseconds(5));}
        return false;
      }, 10, "download service discovered by the panel");
    wait([&] {return download_response->toPlainText().contains("download ok");}, 5, "download reply");
    check(downloads.size() == 1 && downloads[0].sensor_id == 0, "Download request wrong");
    check(start->isEnabled(), "Download button not re-enabled");
    spin_server = false;
    start->click();
    download.reset();  // Outstanding request; no detached worker may touch the panel.
    spin_server = true;
    wait([&] {return downloads.size() == 2;}, 3, "late download reply after destruction");
    wait([&] {return true;}, .2, "");

    // --- Status (C14): late publisher discovered, subscribed only when selected ---
    auto status = loader.createSharedInstance("smart_rviz_plugin/Smart Status");
    auto status_topic = child<QComboBox>(status, "topic");
    const std::string header_topic = "/smart_radar/port_targetheader_7";
    auto header_pub = node->create_publisher<umrr_ros2_msgs::msg::PortTargetHeader>(header_topic, 10);
    wait([&] {return status_topic->findText(QString::fromStdString(header_topic)) > 0;}, 5,
      "status topic discovery");
    check(node->count_subscribers(header_topic) == 0, "Status subscribed before selection");
    status_topic->setCurrentIndex(status_topic->findText(QString::fromStdString(header_topic)));
    wait([&] {return node->count_subscribers(header_topic) == 1;}, 5, "status subscription");
    auto header_table = child<QTableWidget>(status, "header_table");
    wait([&] {
        umrr_ros2_msgs::msg::PortTargetHeader message;
        message.number_of_targets = 17;
        header_pub->publish(message);
        return header_table->item(1, 0) && header_table->item(1, 0)->text() == "17";
      }, 5, "status table update");
    header_pub.reset();
    wait([&] {return status_topic->currentIndex() == 0 && node->count_subscribers(header_topic) == 0;},
      5, "status unsubscribe after topic vanished");

    // --- Fault reports (C16) ---
    auto faults = loader.createSharedInstance("smart_rviz_plugin/Smart Fault Reports");
    auto fault_topic = child<QComboBox>(faults, "topic");
    const std::string fault_name = "/smart_radar/port_faultreport_3";
    auto fault_pub = node->create_publisher<umrr_ros2_msgs::msg::PortFaultReportsMsg>(fault_name, 10);
    wait([&] {return fault_topic->findText(QString::fromStdString(fault_name)) > 0;}, 5,
      "fault topic discovery");
    fault_topic->setCurrentIndex(fault_topic->findText(QString::fromStdString(fault_name)));
    wait([&] {return node->count_subscribers(fault_name) == 1;}, 5, "fault subscription");
    fault_pub.reset();
    wait([&] {return fault_topic->currentIndex() == 0 && node->count_subscribers(fault_name) == 0;},
      5, "fault unsubscribe after topic vanished");

    // --- Recorder (P8, C14) ---
    auto recorder = loader.createSharedInstance("smart_rviz_plugin/Smart Recorder");
    wait([&] {return true;}, .5, "");
    check(node->count_subscribers("/ip_camera_front_right/image_raw/compressed") == 0 &&
      node->count_publishers("/ip_camera_front_right/image_raw") == 0,
      "Recorder still decodes/republishes a camera topic");
    auto recorder_topic = child<QComboBox>(recorder, "topic");
    const std::string cloud_topic = "/smart_radar/port_targets_5";
    auto cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, 10);
    wait([&] {return recorder_topic->findText(QString::fromStdString(cloud_topic)) > 0;}, 5,
      "recorder topic discovery");
    check(node->count_subscribers(cloud_topic) == 0, "Recorder subscribed before selection");
    recorder_topic->setCurrentIndex(recorder_topic->findText(QString::fromStdString(cloud_topic)));
    wait([&] {return node->count_subscribers(cloud_topic) == 1;}, 5, "recorder subscription");
    auto table = child<QTableWidget>(recorder, "data_table");
    wait([&] {
        cloud_pub->publish(target_cloud(150));
        return table->rowCount() == 150 && table->item(149, 10) &&
          table->item(149, 10)->text() == "149.00";
      }, 5, "recorder table update");
    // Malformed cloud (missing fields) must be reported, not crash RViz.
    sensor_msgs::msg::PointCloud2 bad;
    bad.width = 3;
    cloud_pub->publish(bad);
    wait([&] {return child<QLabel>(recorder, "status")->text().contains("Cannot display");}, 5,
      "malformed cloud reported");
    child<QSpinBox>(recorder, "max_rows")->setValue(200);
    child<QPushButton>(recorder, "record")->click();
    wait([&] {
        cloud_pub->publish(target_cloud(150));
        return child<QLabel>(recorder, "status")->text().contains("limit");
      }, 5, "recording cap");
    check(child<QPushButton>(recorder, "save")->isEnabled() &&
      child<QPushButton>(recorder, "record")->text() == "Record",
      "Recording not stopped at the cap");
    check(child<QLabel>(recorder, "status")->text().contains("200 rows"), "Cap not exact");
    recorder.reset();
    status.reset();
    faults.reset();

    rclcpp::shutdown();
    std::cout << "PASS: unique node names, strict parsing, async services with deadlines, "
      "installed tables, lazy subscriptions with discovery, recorder cap" << std::endl;
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}
