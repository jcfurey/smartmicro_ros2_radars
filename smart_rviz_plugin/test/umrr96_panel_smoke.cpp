// SPDX-License-Identifier: Apache-2.0
#include <QApplication>
#include <QComboBox>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPushButton>
#include <QPixmap>
#include <QTimer>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <map>
#include <stdexcept>
#include <thread>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <std_msgs/msg/string.hpp>
#include <rviz_common/panel.hpp>
#include <umrr_ros2_msgs/srv/get_mode.hpp>
#include <umrr_ros2_msgs/srv/get_status.hpp>
#include <umrr_ros2_msgs/srv/set_mode.hpp>
#include <umrr_ros2_msgs/msg/port_target_header.hpp>

using GetMode = umrr_ros2_msgs::srv::GetMode;
using GetStatus = umrr_ros2_msgs::srv::GetStatus;
using SetMode = umrr_ros2_msgs::srv::SetMode;
using Clock = std::chrono::steady_clock;

void check(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<rclcpp::Node>("umrr96_panel_fixture");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::map<std::string, int> settings = {
      {"frequency_sweep_idx", 2}, {"range_toggle_mode", 0},
      {"tx_antenna_idx", 0}, {"output_control_target_list_can", 1}};
    unsigned writes = 0;
    bool ignore_writes = false;
    bool malformed_read = false;
    auto encode = [](const QJsonObject & values) {
        return QJsonDocument(QJsonObject{{"success", true}, {"sensor_id", 230739},
          {"values", values}}).toJson(QJsonDocument::Compact).toStdString();
      };
    auto getter = node->create_service<GetMode>("/smart_radar/get_radar_mode",
      [&](GetMode::Request::SharedPtr request, GetMode::Response::SharedPtr response) {
        QJsonObject values;
        for (const auto & name : request->params) {
          values[QString::fromStdString(name)] = QJsonObject{
            {"response_type", 1}, {"value", settings.at(name)}};
        }
        response->res = malformed_read ? "Request queued" : encode(values);
      });
    auto setter = node->create_service<SetMode>("/smart_radar/set_radar_mode",
      [&](SetMode::Request::SharedPtr request, SetMode::Response::SharedPtr response) {
        check(request->sensor_id == 230739, "Wrong sensor ID");
        check(request->section_name == "auto_interface_0dim", "Wrong section");
        check(request->params.size() == 1, "Panel wrote unchanged settings");
        check(request->params.front() == "output_control_target_list_can", "Wrong changed field");
        check(request->value_types == std::vector<uint8_t>{3}, "Wrong field type");
        ++writes;
        QJsonObject values;
        for (size_t i = 0; i < request->params.size(); ++i) {
          const auto v = std::stoi(request->values[i]);
          if (!ignore_writes) {settings.at(request->params[i]) = v;}
          values[QString::fromStdString(request->params[i])] = QJsonObject{
            {"response_type", 1}, {"value", v}};
        }
        response->res = encode(values);
      });
    auto status = node->create_service<GetStatus>("/smart_radar/get_radar_status",
      [&](GetStatus::Request::SharedPtr, GetStatus::Response::SharedPtr response) {
        response->res = encode({
          {"sw_version_major", QJsonObject{{"response_type", 1}, {"value", 5}}},
          {"sw_version_minor", QJsonObject{{"response_type", 1}, {"value", 2}}},
          {"sw_version_patch", QJsonObject{{"response_type", 1}, {"value", 2}}}});
      });
    using FilterService = rcl_interfaces::srv::SetParametersAtomically;
    unsigned filter_writes = 0;
    bool reject_filter = false;
    auto filter_publisher = node->create_publisher<std_msgs::msg::String>(
      "/smart_radar/filter_status", rclcpp::QoS(1).transient_local());
    auto publish_filter = [&](const std::string & mode, double snr, double speed) {
        std_msgs::msg::String message;
        message.data = QJsonDocument(QJsonObject{{"mode", QString::fromStdString(mode)},
          {"min_snr_db", snr}, {"min_abs_speed", speed}, {"input", 12}, {"accepted", 9},
          {"rejected_quality", 1}, {"rejected_motion", 0}, {"rejected_temporal", 2}})
          .toJson(QJsonDocument::Compact).toStdString();
        filter_publisher->publish(message);
      };
    auto filter_service = node->create_service<FilterService>("/umrr96_views/set_parameters_atomically",
      [&](FilterService::Request::SharedPtr request, FilterService::Response::SharedPtr response) {
        check(request->parameters.size() == 3, "Filter settings must be atomic");
        ++filter_writes;
        response->result.successful = !reject_filter;
        response->result.reason = reject_filter ? "Fixture rejection" : "";
        if (!reject_filter) {
          publish_filter(request->parameters[0].value.string_value,
            request->parameters[1].value.double_value, request->parameters[2].value.double_value);
        }
      });
    publish_filter("off", 6.0, .25);
    bool spin_server = true;
    auto publisher = node->create_publisher<umrr_ros2_msgs::msg::PortTargetHeader>(
      "/smart_radar/port_targetheader_0", rclcpp::SensorDataQoS());
    QTimer measurements;
    QObject::connect(&measurements, &QTimer::timeout, [&] {
        umrr_ros2_msgs::msg::PortTargetHeader message;
        message.number_of_targets = 31;
        message.cycle_time = .055;
        publisher->publish(message);
      });
    measurements.start(50);
    auto wait = [&](std::function<bool()> done, double seconds = 8) {
        const auto end = Clock::now() + std::chrono::duration<double>(seconds);
        while (!done() && Clock::now() < end) {
          if (spin_server) {executor.spin_some(std::chrono::milliseconds(2));}
          app.processEvents();
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        check(done(), "Panel operation timed out in test");
      };
    pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common", "rviz_common::Panel");
    auto panel = loader.createSharedInstance("smart_rviz_plugin/UMRR-96 Configuration");
    auto button = [&](const char * name) {
        auto widget = panel->findChild<QPushButton *>(name);
        check(widget != nullptr, "Button missing");
        return widget;
      };
    auto feedback = panel->findChild<QLabel *>("feedback");
    auto identity = panel->findChild<QLabel *>("identity");
    auto can = panel->findChild<QComboBox *>("can_output");
    check(feedback && identity && can, "Panel widgets missing");
    panel->resize(520, 520);
    panel->show();
    wait([&] {return button("preset")->isEnabled() && identity->text().contains("5.2.2");});
    wait([&] {return panel->findChild<QLabel *>("metrics")->text().contains("31 targets");});
    check(writes == 0 && can->currentData().toInt() == 1, "Opening panel modified sensor");
    auto filter_mode = panel->findChild<QComboBox *>("filter_mode");
    auto filter_feedback = panel->findChild<QLabel *>("filter_feedback");
    auto filter_actual = panel->findChild<QLabel *>("filter_actual");
    check(filter_mode && filter_feedback && filter_actual, "Filter controls missing");
    wait([&] {return button("filter_apply")->isEnabled();});
    check(filter_writes == 0 && filter_mode->currentData().toString() == "off",
      "Opening panel modified host filters");
    for (const auto & mode : {"mapping", "moving", "off"}) {
      filter_mode->setCurrentIndex(filter_mode->findData(mode));
      const auto count = filter_writes;
      app.processEvents();
      check(count == filter_writes, "Selecting filter mode must only stage values");
      button("filter_apply")->click();
      wait([&] {return filter_writes == count + 1 && filter_feedback->text().contains("Filter applied");});
      wait([&] {return button("filter_apply")->isEnabled();});
    }
    check(writes == 0, "Host filtering sent a sensor write");
    reject_filter = true;
    filter_mode->setCurrentIndex(filter_mode->findData("mapping"));
    button("filter_apply")->click();
    wait([&] {return filter_feedback->text().contains("Fixture rejection");});
    check(filter_actual->text().contains("Off (raw detections)"), "Rejected filter changed actual mode");
    reject_filter = false;
    button("preset")->click();
    check(writes == 0 && can->currentData().toInt() == 0, "Preset did not only stage values");
    button("apply")->click();
    wait([&] {return feedback->text() == "Changes applied and verified.";});
    check(writes == 1 && settings.at("output_control_target_list_can") == 0, "Apply failed");
    button("starting")->click();
    check(writes == 1 && button("apply")->isEnabled(), "Starting values were not staged");
    button("apply")->click();
    wait([&] {return writes == 2 && feedback->text() == "Changes applied and verified.";});
    check(settings.at("output_control_target_list_can") == 1, "Starting values not restored");

    ignore_writes = true;
    button("preset")->click();
    button("apply")->click();
    wait([&] {return feedback->text().contains("differs");});
    check(can->currentData().toInt() == 1, "Mismatched readback not shown");
    malformed_read = true;
    button("refresh")->click();
    wait([&] {return feedback->text().contains("Unexpected service response");});
    check(!button("apply")->isEnabled(), "Apply enabled after malformed response");
    malformed_read = false;
    button("refresh")->click();
    wait([&] {return button("preset")->isEnabled();});

    if (const auto screenshot = std::getenv("UMRR96_PANEL_SCREENSHOT")) {
      check(panel->grab().save(QString::fromLocal8Bit(screenshot)), "Screenshot failed");
    }
    // Keep Qt running but stop servicing requests: the panel must remain
    // responsive, clean up the timed-out request, and allow a subsequent read.
    unsigned heartbeats = 0;
    QTimer heartbeat;
    QObject::connect(&heartbeat, &QTimer::timeout, [&] {++heartbeats;});
    heartbeat.start(10);
    spin_server = false;
    button("filter_apply")->click();
    button("refresh")->click();
    wait([&] {return feedback->text().contains("timed out");}, 7);
    check(filter_feedback->text().contains("timed out"), "Filter request did not time out");
    check(heartbeats > 100, "Qt blocked waiting for the service");
    spin_server = true;
    button("refresh")->click();
    wait([&] {return button("preset")->isEnabled();});
    spin_server = false;
    button("refresh")->click();
    button("filter_apply")->click();
    panel.reset();  // Destroy with an outstanding request; no worker may hang.
    executor.spin_some();  // Deliver the late reply after destruction.
    app.processEvents();
    rclcpp::shutdown();
    std::cout << "PASS: stage/apply/restore, readback mismatch, malformed response, "
      "filter selection/rejection, responsive timeouts, recovery and pending-request shutdown" << std::endl;
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}
