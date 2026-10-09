// SPDX-License-Identifier: Apache-2.0
// Smoke test for the generic Smart panels: unique node names, strict input parsing,
// non-blocking service calls with deadlines, installed instruction tables, lazy topic
// subscriptions with periodic discovery, and the bounded recorder.
#include <QApplication>
#include <QComboBox>
#include <QFile>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QTemporaryDir>
#include <QTextEdit>
#include <QTimer>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>
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
  sensor_msgs::PointCloud2Iterator<float> rcs(cloud, "rcs");
  for (size_t i = 0; i < points; ++i, ++range, ++rcs) {
    *range = static_cast<float>(i);
    *rcs = .0032f;  // m^2 (C47): two decimals would show 0.00.
  }
  return cloud;
}

sensor_msgs::msg::PointCloud2 object_cloud(size_t points, float heading_rad)
{
  sensor_msgs::msg::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  using PF = sensor_msgs::msg::PointField;
  modifier.setPointCloud2Fields(14,
    "x", 1, PF::FLOAT32, "y", 1, PF::FLOAT32, "z", 1, PF::FLOAT32,
    "speed_absolute", 1, PF::FLOAT32, "heading", 1, PF::FLOAT32, "length", 1, PF::FLOAT32,
    "mileage", 1, PF::FLOAT32, "quality", 1, PF::FLOAT32, "acceleration", 1, PF::FLOAT32,
    "object_id", 1, PF::INT16, "idle_cycles", 1, PF::UINT16, "spline_idx", 1, PF::UINT16,
    "object_class", 1, PF::UINT8, "status", 1, PF::UINT16);
  modifier.resize(points);
  sensor_msgs::PointCloud2Iterator<float> heading(cloud, "heading");
  for (size_t i = 0; i < points; ++i, ++heading) {*heading = heading_rad;}
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
    // S29: names carry the process ID, so a second RViz with the same config
    // cannot collide either.
    {
      std::vector<std::shared_ptr<rviz_common::Panel>> twins;
      auto all = generic;
      all.push_back("smart_rviz_plugin/UMRR-96 Configuration");
      for (const auto & name : all) {
        twins.push_back(loader.createSharedInstance(name));
        twins.push_back(loader.createSharedInstance(name));
      }
      const auto pid = "_" + std::to_string(::getpid()) + "_";
      std::vector<std::string> names;
      wait([&] {
          names.clear();
          for (const auto & n : node->get_node_names()) {
            if (n.find("gui") != std::string::npos || n.find("umrr96_config") != std::string::npos) {
              names.push_back(n);
            }
          }
          return names.size() >= 12;
        }, 10, "panel nodes in the graph");
      check(std::set<std::string>(names.begin(), names.end()).size() == names.size(),
        "Duplicate panel node names");
      for (const auto & n : names) {
        check(n.find(pid) != std::string::npos, "Panel node name without the process ID: " + n);
      }
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
    // S25: the publisher is best effort, as with the driver's QoS override; the
    // recorder below checks that a reliable (default) publisher still matches.
    auto status = loader.createSharedInstance("smart_rviz_plugin/Smart Status");
    auto status_topic = child<QComboBox>(status, "topic");
    const std::string header_topic = "/smart_radar/port_targetheader_7";
    auto header_pub = node->create_publisher<umrr_ros2_msgs::msg::PortTargetHeader>(
      header_topic, rclcpp::SensorDataQoS());
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
    auto fault_pub = node->create_publisher<umrr_ros2_msgs::msg::PortFaultReportsMsg>(
      fault_name, rclcpp::SensorDataQoS());
    wait([&] {return fault_topic->findText(QString::fromStdString(fault_name)) > 0;}, 5,
      "fault topic discovery");
    fault_topic->setCurrentIndex(fault_topic->findText(QString::fromStdString(fault_name)));
    wait([&] {return node->count_subscribers(fault_name) == 1;}, 5, "fault subscription");
    auto fault_header = child<QTableWidget>(faults, "fault_header_table");
    wait([&] {
        umrr_ros2_msgs::msg::PortFaultReportsMsg message;
        message.header.frame_id = "best_effort_faults";
        fault_pub->publish(message);
        return fault_header->item(2, 0)->text() == "best_effort_faults";
      }, 5, "best-effort fault report received");
    fault_pub.reset();
    wait([&] {return fault_topic->currentIndex() == 0 && node->count_subscribers(fault_name) == 0;},
      5, "fault unsubscribe after topic vanished");

    // --- Recorder (P8, C14, C41) ---
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
    check(table->item(149, 5)->text() == "0.0032", "RCS [m^2] rounded away in the table");
    // Malformed cloud (missing fields) must be reported, not crash RViz.
    sensor_msgs::msg::PointCloud2 bad;
    bad.width = 3;
    cloud_pub->publish(bad);
    wait([&] {return child<QLabel>(recorder, "status")->text().contains("Cannot display");}, 5,
      "malformed cloud reported");
    child<QSpinBox>(recorder, "max_rows")->setValue(200);
    child<QPushButton>(recorder, "record")->click();
    // Queue eight frames without processing Qt events, simulating a GUI stall.
    // Stay below the subscription's depth of ten and wait for delivery before
    // invoking ticks, so this checks queue draining, not DDS timing. The panel
    // subscribes best effort (S25), so acknowledgements cannot confirm delivery;
    // a best-effort probe on the same topic stands in for the panel's reader.
    size_t probed = 0;
    auto probe = node->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic,
      rclcpp::SensorDataQoS(rclcpp::KeepLast(10)),
      [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr) {++probed;});
    const auto probe_deadline = Clock::now() + std::chrono::seconds(5);
    while (node->count_subscribers(cloud_topic) < 2 && Clock::now() < probe_deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    for (int i = 0; i < 8; ++i) {cloud_pub->publish(target_cloud(26));}
    while (probed < 8 && Clock::now() < probe_deadline) {
      executor.spin_some(std::chrono::milliseconds(2));
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    check(probed == 8, "Recorder burst was not delivered");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // Eight frames hold 208 rows. Seven ticks must process all eight: the eighth
    // no longer fits under the 200-row cap and stops the recording with seven
    // whole frames (C59). One-message-per-tick processing handles only seven
    // frames and never reaches the cap. No Qt events run here, so the panel's
    // timer cannot supply extra ticks behind our back.
    for (int i = 0; i < 7; ++i) {
      check(QMetaObject::invokeMethod(recorder.get(), "check_data", Qt::DirectConnection),
        "Recorder tick slot missing");
    }
    check(child<QLabel>(recorder, "status")->text().contains("limit"),
      "Recorder did not catch up after a queued burst");
    check(child<QPushButton>(recorder, "save")->isEnabled() &&
      child<QPushButton>(recorder, "record")->text() == "Record",
      "Recording not stopped at the cap");
    probe.reset();
    check(child<QLabel>(recorder, "status")->text().contains("182 rows held"),
      "Cap did not keep whole frames: " + child<QLabel>(recorder, "status")->text().toStdString());
    // CSV: m^2 header, significant digits, and whole frames only.
    QTemporaryDir csv_dir;
    check(csv_dir.isValid(), "No temporary directory");
    const auto csv_path = csv_dir.filePath("targets.csv");
    recorder->setProperty("save_path", csv_path);
    child<QPushButton>(recorder, "save")->click();
    check(child<QLabel>(recorder, "status")->text() == "Recording saved.", "Recording not saved");
    QFile csv(csv_path);
    check(csv.open(QIODevice::ReadOnly | QIODevice::Text), "CSV not written");
    const auto lines = QString::fromUtf8(csv.readAll()).split('\n', Qt::SkipEmptyParts);
    check(lines.size() == 183 && lines[0].contains("RCS [m^2]") && !lines[0].contains("RCS [dB]"),
      "CSV header or row count wrong");
    check(lines[1].split(", ")[6] == "0.0032" && lines[182].split(", ")[2] == "25.00",
      "CSV RCS rounded or last frame partial");

    // CAN targets carry m^2 since C47; CAN object heading is rad since C2.
    const std::string can_targets = "/smart_radar/can_targets_5";
    const std::string can_objects = "/smart_radar/can_objects_5";
    auto can_target_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(can_targets, 10);
    auto can_object_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(can_objects, 10);
    wait([&] {return recorder_topic->findText(QString::fromStdString(can_objects)) > 0 &&
        recorder_topic->findText(QString::fromStdString(can_targets)) > 0;}, 5,
      "CAN topic discovery");
    recorder_topic->setCurrentIndex(recorder_topic->findText(QString::fromStdString(can_targets)));
    check(table->horizontalHeaderItem(5)->text() == "RCS [m^2]", "CAN RCS labelled dB");
    wait([&] {
        can_target_pub->publish(target_cloud(4));
        return table->rowCount() == 4 && table->item(3, 5) && table->item(3, 5)->text() == "0.0032";
      }, 5, "CAN target table update");
    recorder_topic->setCurrentIndex(recorder_topic->findText(QString::fromStdString(can_objects)));
    check(table->horizontalHeaderItem(4)->text() == "Heading [Deg]", "CAN heading header");
    wait([&] {
        can_object_pub->publish(object_cloud(3, static_cast<float>(M_PI / 2)));
        return table->rowCount() == 3 && table->item(2, 4) && table->item(2, 4)->text() == "90.00";
      }, 5, "CAN object heading in degrees");
    recorder.reset();
    status.reset();
    faults.reset();

    rclcpp::shutdown();
    std::cout << "PASS: unique node names, strict parsing, async services with deadlines, "
      "installed tables, lazy subscriptions with discovery, recorder burst and cap" << std::endl;
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}
