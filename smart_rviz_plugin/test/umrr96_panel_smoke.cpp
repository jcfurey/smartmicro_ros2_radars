// SPDX-License-Identifier: Apache-2.0
#include <QApplication>
#include <QComboBox>
#include <QDialog>
#include <QDoubleSpinBox>
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
    std::map<std::string, double> settings = {
      {"frequency_sweep_idx", 2}, {"range_toggle_mode", 0},
      {"tx_antenna_idx", 0}, {"output_control_target_list_can", 1},
      {"prf_selector_manual", 1}, {"prf_manual_value_idx", 0}, {"prf_set_selector", 0},
      {"tv_min_speed_sweep_idx_0", -20}, {"tv_max_speed_sweep_idx_0", 20},
      {"tv_min_speed_sweep_idx_1", -20}, {"tv_max_speed_sweep_idx_1", 20},
      {"tv_min_speed_sweep_idx_2", static_cast<float>(-20.123456)}, {"tv_max_speed_sweep_idx_2", 20}};
    const auto initial_settings = settings;
    unsigned writes = 0;
    bool ignore_writes = false;
    bool malformed_read = false;
    bool invalid_advanced_read = false;
    bool partial_velocity_write = false;
    std::string reject_name;
    std::vector<SetMode::Request> requests;
    auto encode = [](const QJsonObject & values) {
        return QJsonDocument(QJsonObject{{"success", true}, {"sensor_id", 230739},
          {"values", values}}).toJson(QJsonDocument::Compact).toStdString();
      };
    auto getter = node->create_service<GetMode>("/smart_radar/get_radar_mode",
      [&](GetMode::Request::SharedPtr request, GetMode::Response::SharedPtr response) {
        check(request->params.size() <= 10, "Read exceeds SDK batch limit");
        QJsonObject values;
        for (size_t i = 0; i < request->params.size(); ++i) {
          const auto & name = request->params[i];
          check(request->param_types[i] == (name.find("tv_") == 0 ? 0 : 3), "Wrong read type");
          values[QString::fromStdString(name)] = QJsonObject{
            {"response_type", 1}, {"value", invalid_advanced_read && name == "prf_selector_manual" ?
              1.5 : settings.at(name)}};
        }
        response->res = malformed_read ? "Request queued" : encode(values);
      });
    auto setter = node->create_service<SetMode>("/smart_radar/set_radar_mode",
      [&](SetMode::Request::SharedPtr request, SetMode::Response::SharedPtr response) {
        check(request->sensor_id == 230739, "Wrong sensor ID");
        check(request->section_name == "auto_interface_0dim", "Wrong section");
        check(!request->params.empty() && request->params.size() <= 10, "Invalid write batch size");
        if (request->params.front() == "output_control_target_list_can") {
          check(request->params.size() == 1, "Panel wrote unchanged basic settings");
          check(request->value_types == std::vector<uint8_t>{3}, "Wrong basic field type");
        }
        ++writes;
        requests.push_back(*request);
        if (partial_velocity_write) {
          for (size_t i = 0; i < request->params.size(); ++i) {
            if (request->params[i] == "tv_min_speed_sweep_idx_2") {
              settings.at(request->params[i]) = std::stod(request->values[i]);
            }
          }
          response->res = QJsonDocument(QJsonObject{{"success", false}, {"sensor_id", 230739},
            {"error", "Fixture applied only the minimum bound"}}).toJson().toStdString();
          return;
        }
        if (!reject_name.empty() && request->params.front() == reject_name) {
          response->res = QJsonDocument(QJsonObject{{"success", false}, {"sensor_id", 230739},
            {"error", "Fixture rejected instruction"}}).toJson().toStdString();
          return;
        }
        QJsonObject values;
        for (size_t i = 0; i < request->params.size(); ++i) {
          const bool floating = request->params[i].find("tv_") == 0;
          check(request->value_types[i] == (floating ? 0 : 3), "Wrong write field type");
          const auto v = floating ? static_cast<double>(static_cast<float>(std::stod(request->values[i]))) :
            std::stod(request->values[i]);
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
    bool report_decay = true;
    double actual_decay = 2.001234;
    auto filter_publisher = node->create_publisher<std_msgs::msg::String>(
      "/smart_radar/filter_status", rclcpp::QoS(1).transient_local());
    auto publish_filter = [&](const std::string & mode, double snr, double speed) {
        std_msgs::msg::String message;
        QJsonObject values{{"mode", QString::fromStdString(mode)},
          {"min_snr_db", snr}, {"min_abs_speed", speed}, {"input", 12}, {"accepted", 9},
          {"rejected_quality", 1}, {"rejected_motion", 0}, {"rejected_temporal", 2}};
        if (report_decay) {values["density_decay_seconds"] = actual_decay;}
        message.data = QJsonDocument(values).toJson(QJsonDocument::Compact).toStdString();
        filter_publisher->publish(message);
      };
    auto filter_service = node->create_service<FilterService>("/umrr96_views/set_parameters_atomically",
      [&](FilterService::Request::SharedPtr request, FilterService::Response::SharedPtr response) {
        check(request->parameters.size() == (report_decay ? 4u : 3u),
          "View settings must be atomic and compatible with the view node");
        if (report_decay) {
          check(request->parameters[3].name == "decay_seconds" &&
            request->parameters[3].value.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
            "Wrong decay parameter or type");
        }
        ++filter_writes;
        response->result.successful = !reject_filter;
        response->result.reason = reject_filter ? "Fixture rejection" : "";
        if (!reject_filter) {
          if (report_decay) {actual_decay = request->parameters[3].value.double_value;}
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
    auto decay = panel->findChild<QDoubleSpinBox *>("density_decay");
    check(filter_mode && filter_feedback && filter_actual && decay, "View controls missing");
    wait([&] {return button("filter_apply")->isEnabled();});
    check(filter_writes == 0 && filter_mode->currentData().toString() == "off",
      "Opening panel modified host filters");
    check(decay->isEnabled() && decay->value() == 2.0, "Actual decay not loaded");
    for (const auto & mode : {"quality", "mapping", "moving", "off"}) {
      filter_mode->setCurrentIndex(filter_mode->findData(mode));
      const auto count = filter_writes;
      app.processEvents();
      check(count == filter_writes, "Selecting filter mode must only stage values");
      button("filter_apply")->click();
      wait([&] {return filter_writes == count + 1 && filter_feedback->text().contains("View settings applied");});
      wait([&] {return button("filter_apply")->isEnabled();});
    }
    check(actual_decay == 2.001234, "Filter edits rounded an unchanged decay value");
    const auto before_decay = filter_writes;
    decay->setValue(.5);
    app.processEvents();
    check(filter_writes == before_decay && actual_decay == 2.001234, "Decay edit must only stage values");
    button("filter_apply")->click();
    wait([&] {return filter_writes == before_decay + 1 &&
        filter_feedback->text().contains("View settings applied") &&
        filter_actual->text().contains("Density decay: 0.50 s");});
    wait([&] {return button("filter_apply")->isEnabled();});
    check(writes == 0, "Host filtering sent a sensor write");
    reject_filter = true;
    filter_mode->setCurrentIndex(filter_mode->findData("mapping"));
    decay->setValue(5.0);
    button("filter_apply")->click();
    wait([&] {return filter_feedback->text().contains("Fixture rejection");});
    check(filter_actual->text().contains("Off (raw detections)"), "Rejected filter changed actual mode");
    check(actual_decay == .5 && filter_actual->text().contains("Density decay: 0.50 s"),
      "Rejected update changed actual decay");
    reject_filter = false;
    // An older view node still supports filters; its read-only decay must not be sent.
    report_decay = false;
    publish_filter("off", 6.0, .25);
    wait([&] {return !decay->isEnabled() && button("filter_apply")->isEnabled();});
    button("filter_apply")->click();
    wait([&] {return filter_feedback->text().contains("View settings applied");});
    report_decay = true;
    publish_filter("mapping", 6.0, .25);
    wait([&] {return decay->isEnabled() && decay->value() == .5;});
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

    // Advanced controls are read only on opening, preserve unedited float
    // precision, and keep basic settings outside their writes.
    ignore_writes = false;
    const auto before_advanced = writes;
    button("advanced")->click();
    auto advanced_feedback = panel->findChild<QLabel *>("advanced_feedback");
    auto mode = panel->findChild<QComboBox *>("prf_selector_manual");
    auto index = panel->findChild<QComboBox *>("prf_manual_value_idx");
    auto minimum = panel->findChild<QDoubleSpinBox *>("tv_min_speed_sweep_idx_2");
    auto maximum = panel->findChild<QDoubleSpinBox *>("tv_max_speed_sweep_idx_2");
    check(advanced_feedback && mode && index && minimum && maximum, "Advanced widgets missing");
    wait([&] {return button("advanced_starting")->isEnabled();});
    check(writes == before_advanced && !button("advanced_apply")->isEnabled(),
      "Opening/rounding advanced settings staged or sent a change");
    index->setCurrentIndex(index->findData(2));
    minimum->setValue(-12.5);
    maximum->setValue(25.25);
    check(writes == before_advanced, "Staging advanced values sent a write");
    const auto first = requests.size();
    button("advanced_apply")->click();
    wait([&] {return advanced_feedback->text() == "Advanced changes applied and verified.";});
    check(requests.size() == first + 4, "Wrong number of advanced write batches");
    check(requests[first].params == std::vector<std::string>{"prf_selector_manual"} &&
      requests[first].values == std::vector<std::string>{"0"}, "PRF not disabled before changing index");
    check(requests[first + 1].params == std::vector<std::string>{"prf_manual_value_idx"}, "Wrong PRF index order");
    check(requests[first + 2].values == std::vector<std::string>{"1"}, "Manual PRF not restored after index");
    check(requests[first + 3].value_types == std::vector<uint8_t>{0, 0} &&
      requests[first + 3].params == std::vector<std::string>{"tv_min_speed_sweep_idx_2", "tv_max_speed_sweep_idx_2"},
      "Velocity window was not sent as a typed pair");
    check(settings.at("tv_min_speed_sweep_idx_2") == -12.5 &&
      settings.at("tv_max_speed_sweep_idx_2") == 25.25, "Float write did not reach fixture");
    auto restore_advanced = [&] {
        const auto count = writes;
        button("advanced_starting")->click();
        check(writes == count, "Starting values did not only stage advanced settings");
        button("advanced_apply")->click();
        wait([&] {return advanced_feedback->text() == "Advanced changes applied and verified.";});
        check(settings == initial_settings, "Advanced restore lost original precision or changed basic settings");
      };
    restore_advanced();

    minimum->setValue(30);  // Maximum is 20: reject locally before any preflight/write.
    check(!button("advanced_apply")->isEnabled(), "Inverted velocity window can be applied");
    button("advanced_starting")->click();
    index->setCurrentIndex(index->findData(2));
    settings["prf_manual_value_idx"] = 1;  // Another control client changed it.
    const auto before_conflict = writes;
    button("advanced_apply")->click();
    wait([&] {return advanced_feedback->text().contains("changed since");});
    check(writes == before_conflict && index->currentData().toInt() == 1,
      "Stale advanced state overwrote a concurrent change");
    restore_advanced();

    reject_name = "prf_manual_value_idx";
    index->setCurrentIndex(index->findData(2));
    const auto before_rejection = writes;
    button("advanced_apply")->click();
    wait([&] {return advanced_feedback->text().contains("changes may be partial");});
    check(writes == before_rejection + 2 && mode->currentData().toInt() == 0 &&
      index->currentData().toInt() == 0, "Rejected PRF sequence continued or hid partial state");
    reject_name.clear();
    restore_advanced();

    // A partly applied pair can leave the sensor window inverted. Show it and
    // allow repair rather than locking out the controls needed for restoration.
    partial_velocity_write = true;
    minimum->setValue(25);
    maximum->setValue(35);
    button("advanced_apply")->click();
    wait([&] {return advanced_feedback->text().contains("inverted velocity window");});
    check(minimum->value() == 25 && maximum->value() == 20 &&
      minimum->isEnabled() && !button("advanced_apply")->isEnabled(), "Inverted sensor window cannot be repaired");
    partial_velocity_write = false;
    restore_advanced();

    ignore_writes = true;
    minimum->setValue(-12.5);
    button("advanced_apply")->click();
    wait([&] {return advanced_feedback->text().contains("differs");});
    check(!button("advanced_apply")->isEnabled(), "Mismatched advanced readback was not staged as actual");
    ignore_writes = false;
    invalid_advanced_read = true;
    button("advanced_refresh")->click();
    wait([&] {return advanced_feedback->text().contains("Unsupported value");});
    check(!button("advanced_apply")->isEnabled(), "Invalid advanced response left Apply enabled");
    invalid_advanced_read = false;
    button("advanced_refresh")->click();
    wait([&] {return button("advanced_starting")->isEnabled();});

    // Lose a write reply while Qt remains responsive. On recovery, the late
    // write may apply, but the remainder of that sequence must stay cancelled.
    index->setCurrentIndex(index->findData(2));
    const auto before_timeout = writes;
    button("advanced_apply")->click();
    wait([&] {return advanced_feedback->text().contains("Applying advanced");});
    spin_server = false;
    wait([&] {return advanced_feedback->text().contains("Reading actual settings");}, 7);
    spin_server = true;
    wait([&] {return advanced_feedback->text().contains("changes may be partial");});
    check(writes == before_timeout + 1 && settings.at("prf_selector_manual") == 0 &&
      settings.at("prf_manual_value_idx") == 0, "Timed-out sequence continued after recovery");
    restore_advanced();

    if (const auto screenshot = std::getenv("UMRR96_ADVANCED_SCREENSHOT")) {
      check(panel->findChild<QDialog *>("advanced_dialog")->grab().save(
        QString::fromLocal8Bit(screenshot)), "Advanced screenshot failed");
    }
    panel->findChild<QDialog *>("advanced_dialog")->hide();

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
    button("advanced_refresh")->click();
    button("filter_apply")->click();
    panel.reset();  // Destroy with an outstanding request; no worker may hang.
    executor.spin_some();  // Deliver the late reply after destruction.
    app.processEvents();
    rclcpp::shutdown();
    std::cout << "PASS: stage/apply/restore, readback mismatch, malformed response, "
      "advanced PRF/float controls, partial failure, conflict detection, precision-preserving restoration, "
      "filter selection/rejection, responsive timeouts, recovery and pending-request shutdown" << std::endl;
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}
