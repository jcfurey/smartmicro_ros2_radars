// SPDX-License-Identifier: Apache-2.0
#include "smart_rviz_plugin/smart_status.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <QSignalBlocker>

#include "panel_util.hpp"

namespace smart_rviz_plugin
{
SmartRadarStatus::SmartRadarStatus(QWidget * parent) : rviz_common::Panel(parent) { initialize(); }

namespace
{
const char * const kSelect = "Select a Topic";
const std::string kPortTarget = "umrr_ros2_msgs/msg/PortTargetHeader";
const std::string kCanTarget = "umrr_ros2_msgs/msg/CanTargetHeader";
const std::string kPortObject = "umrr_ros2_msgs/msg/PortObjectHeader";
const std::string kCanObject = "umrr_ros2_msgs/msg/CanObjectHeader";
}  // namespace

void SmartRadarStatus::initialize()
{
  node_ = std::make_shared<rclcpp::Node>(
    panel_util::unique_node_name("smart_radar_status_gui_node"),
    rclcpp::NodeOptions().use_global_arguments(false));
  executor_.add_node(node_);

  gui_layout_ = new QVBoxLayout();
  topic_dropdown_ = new QComboBox();
  topic_dropdown_->setObjectName("topic");
  topic_dropdown_->addItem(kSelect);

  connect(topic_dropdown_, SIGNAL(currentIndexChanged(int)), this, SLOT(update_table()));

  table_data_ = new QTableWidget();
  table_data_->setObjectName("header_table");
  table_data_->setRowCount(17);
  table_data_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  table_data_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

  splitter_ = new QSplitter(Qt::Vertical);
  splitter_->addWidget(topic_dropdown_);
  splitter_->addWidget(table_data_);

  gui_layout_->addWidget(splitter_);

  timer_ = new QTimer(this);
  connect(timer_, SIGNAL(timeout()), this, SLOT(check_data()));
  timer_->start(50);

  // The graph is usually still empty when RViz starts with the driver: refresh periodically.
  topic_refresh_timer_ = new QTimer(this);
  connect(topic_refresh_timer_, SIGNAL(timeout()), this, SLOT(refresh_topic_list()));
  topic_refresh_timer_->start(1000);
  refresh_topic_list();

  setLayout(gui_layout_);
}

void SmartRadarStatus::refresh_topic_list()
{
  if (!rclcpp::ok()) {return;}
  std::map<std::string, std::string> topics;
  for (const auto & [name, types] : node_->get_topic_names_and_types()) {
    // Count only topics with a publisher: our own subscription keeps a topic in the graph.
    if (node_->count_publishers(name) == 0) {continue;}
    for (const auto & type : types) {
      if (type == kPortTarget || type == kCanTarget || type == kPortObject || type == kCanObject) {
        topics[name] = type;
      }
    }
  }
  if (topics == topic_types_) {return;}
  topic_types_ = topics;

  const std::string previous = selected_topic_;
  int index = 0;
  {
    const QSignalBlocker blocker(topic_dropdown_);
    topic_dropdown_->clear();
    topic_dropdown_->addItem(kSelect);
    for (const auto & entry : topic_types_) {
      topic_dropdown_->addItem(QString::fromStdString(entry.first));
    }
    if (!previous.empty()) {
      index = std::max(0, topic_dropdown_->findText(QString::fromStdString(previous)));
      topic_dropdown_->setCurrentIndex(index);
    }
  }
  if (!previous.empty() && index == 0) {
    update_table();  // Selected topic vanished: unsubscribe and clear.
  }
}

void SmartRadarStatus::port_targetheader_callback(
  const umrr_ros2_msgs::msg::PortTargetHeader::SharedPtr msg, const std::string topic_name)
{
  if (!selected_topic_.empty() && topic_name == selected_topic_) {
    table_data_->setColumnCount(0);
    int col_index = 0;
    std::uint64_t ntp_timestamp = msg->acquisition_start;
    // Extract the first 4 bytes (seconds) - most significant 32 bits
    std::uint32_t seconds = static_cast<uint32_t>(ntp_timestamp >> 32);
    // Extract the last 4 bytes (fraction of a second) - least significant 32 bits
    std::uint32_t fraction_sec = static_cast<uint32_t>(ntp_timestamp & 0xFFFFFFFF);
    table_data_->insertColumn(col_index);
    table_data_->setItem(
      0, col_index, new QTableWidgetItem(QString::number(msg->cycle_time, 'f', 3)));
    table_data_->setItem(
      1, col_index, new QTableWidgetItem(QString::number(msg->number_of_targets)));
    table_data_->setItem(
      2, col_index, new QTableWidgetItem(QString::number(msg->acquisition_tx_ant_idx)));
    table_data_->setItem(
      3, col_index, new QTableWidgetItem(QString::number(msg->acquisition_sweep_idx)));
    table_data_->setItem(
      4, col_index, new QTableWidgetItem(QString::number(msg->acquisition_cf_idx)));
    table_data_->setItem(5, col_index, new QTableWidgetItem(QString::number(seconds)));
    table_data_->setItem(6, col_index, new QTableWidgetItem(QString::number(fraction_sec)));
    table_data_->setItem(7, col_index, new QTableWidgetItem(QString::number(msg->prf)));
    table_data_->setItem(
      8, col_index, new QTableWidgetItem(QString::number(msg->umambiguous_speed, 'f', 2)));
    table_data_->setItem(9, col_index, new QTableWidgetItem(QString::number(msg->port_identifier)));
    table_data_->setItem(10, col_index, new QTableWidgetItem(QString::number(msg->port_ver_major)));
    table_data_->setItem(11, col_index, new QTableWidgetItem(QString::number(msg->port_ver_minor)));
    table_data_->setItem(12, col_index, new QTableWidgetItem(QString::number(msg->port_size)));
    table_data_->setItem(
      13, col_index, new QTableWidgetItem(QString::number(msg->body_endianness)));
    table_data_->setItem(14, col_index, new QTableWidgetItem(QString::number(msg->port_index)));
    table_data_->setItem(
      15, col_index, new QTableWidgetItem(QString::number(msg->header_ver_major)));
    table_data_->setItem(
      16, col_index, new QTableWidgetItem(QString::number(msg->header_ver_minor)));
  }
}

void SmartRadarStatus::can_targetheader_callback(
  const umrr_ros2_msgs::msg::CanTargetHeader::SharedPtr msg, const std::string topic_name)
{
  if (!selected_topic_.empty() && topic_name == selected_topic_) {
    table_data_->setColumnCount(0);
    int col_index = 0;
    table_data_->insertColumn(col_index);
    table_data_->setItem(
      0, col_index, new QTableWidgetItem(QString::number(msg->cycle_time, 'f', 3)));
    table_data_->setItem(
      1, col_index, new QTableWidgetItem(QString::number(msg->number_of_targets)));
    table_data_->setItem(2, col_index, new QTableWidgetItem(QString::number(msg->cycle_count)));
    table_data_->setItem(
      3, col_index, new QTableWidgetItem(QString::number(msg->acquisition_setup)));
    table_data_->setItem(4, col_index, new QTableWidgetItem(QString::number(msg->time_stamp)));
    table_data_->setItem(5, col_index, new QTableWidgetItem(QString::number(msg->acq_ts_fraction)));
  }
}

void SmartRadarStatus::port_objectheader_callback(
  const umrr_ros2_msgs::msg::PortObjectHeader::SharedPtr msg, const std::string topic_name)
{
  if (!selected_topic_.empty() && topic_name == selected_topic_) {
    float_t cycle_time = msg->cycle_time;
    std::uint16_t no_objs = msg->number_of_objects;
    std::uint64_t ntp_timestamp = msg->ts_measurement;
    std::uint32_t seconds = static_cast<uint32_t>(ntp_timestamp >> 32);
    std::uint32_t fraction_sec = static_cast<uint32_t>(ntp_timestamp & 0xFFFFFFFF);

    table_data_->setColumnCount(0);
    int col_index = 0;
    table_data_->insertColumn(col_index);
    table_data_->setItem(0, col_index, new QTableWidgetItem(QString::number(cycle_time, 'f', 3)));
    table_data_->setItem(1, col_index, new QTableWidgetItem(QString::number(no_objs)));
    table_data_->setItem(2, col_index, new QTableWidgetItem(QString::number(seconds)));
    table_data_->setItem(3, col_index, new QTableWidgetItem(QString::number(fraction_sec)));
    table_data_->setItem(4, col_index, new QTableWidgetItem(QString::number(msg->port_identifier)));
    table_data_->setItem(5, col_index, new QTableWidgetItem(QString::number(msg->port_ver_major)));
    table_data_->setItem(6, col_index, new QTableWidgetItem(QString::number(msg->port_ver_minor)));
    table_data_->setItem(7, col_index, new QTableWidgetItem(QString::number(msg->port_size)));
    table_data_->setItem(8, col_index, new QTableWidgetItem(QString::number(msg->body_endianness)));
    table_data_->setItem(9, col_index, new QTableWidgetItem(QString::number(msg->port_index)));
    table_data_->setItem(
      10, col_index, new QTableWidgetItem(QString::number(msg->header_ver_major)));
    table_data_->setItem(
      11, col_index, new QTableWidgetItem(QString::number(msg->header_ver_minor)));
  }
}

void SmartRadarStatus::can_objectheader_callback(
  const umrr_ros2_msgs::msg::CanObjectHeader::SharedPtr msg, const std::string topic_name)
{
  if (!selected_topic_.empty() && topic_name == selected_topic_) {
    table_data_->setColumnCount(0);
    int col_index = 0;
    table_data_->insertColumn(col_index);
    table_data_->setItem(
      0, col_index, new QTableWidgetItem(QString::number(msg->cycle_time, 'f', 3)));
    table_data_->setItem(
      1, col_index, new QTableWidgetItem(QString::number(msg->number_of_objects)));
    table_data_->setItem(2, col_index, new QTableWidgetItem(QString::number(msg->cycle_count)));
    table_data_->setItem(3, col_index, new QTableWidgetItem(QString::number(msg->ego_speed)));
    table_data_->setItem(
      4, col_index, new QTableWidgetItem(QString::number(msg->ego_speed_quality)));
    table_data_->setItem(5, col_index, new QTableWidgetItem(QString::number(msg->ego_yaw_rate)));
    table_data_->setItem(
      6, col_index, new QTableWidgetItem(QString::number(msg->ego_yaw_rate_quality)));
    table_data_->setItem(7, col_index, new QTableWidgetItem(QString::number(msg->dyn_source)));
  }
}

void SmartRadarStatus::update_table()
{
  table_data_->setColumnCount(0);
  const auto choice = topic_dropdown_->currentText().toStdString();
  const auto found = topic_types_.find(choice);
  if (found == topic_types_.end()) {
    selected_topic_.clear();
    selected_type_.clear();
    subscription_.reset();
    return;
  }
  if (choice != selected_topic_ || !subscription_) {
    selected_topic_ = choice;
    selected_type_ = found->second;
    subscription_.reset();
    const auto topic = selected_topic_;
    if (selected_type_ == kPortTarget) {
      subscription_ = node_->create_subscription<umrr_ros2_msgs::msg::PortTargetHeader>(
        topic, 10, [this, topic](const umrr_ros2_msgs::msg::PortTargetHeader::SharedPtr msg) {
          port_targetheader_callback(msg, topic);
        });
    } else if (selected_type_ == kCanTarget) {
      subscription_ = node_->create_subscription<umrr_ros2_msgs::msg::CanTargetHeader>(
        topic, 10, [this, topic](const umrr_ros2_msgs::msg::CanTargetHeader::SharedPtr msg) {
          can_targetheader_callback(msg, topic);
        });
    } else if (selected_type_ == kPortObject) {
      subscription_ = node_->create_subscription<umrr_ros2_msgs::msg::PortObjectHeader>(
        topic, 10, [this, topic](const umrr_ros2_msgs::msg::PortObjectHeader::SharedPtr msg) {
          port_objectheader_callback(msg, topic);
        });
    } else {
      subscription_ = node_->create_subscription<umrr_ros2_msgs::msg::CanObjectHeader>(
        topic, 10, [this, topic](const umrr_ros2_msgs::msg::CanObjectHeader::SharedPtr msg) {
          can_objectheader_callback(msg, topic);
        });
    }
  }
  if (selected_type_ == kPortTarget) {
    table_data_->setVerticalHeaderLabels(
      {"CycleDuration [s]", "NumOfTargets", "AcquisitionTxAntIdx", "AcquisitionSweepIdx",
       "AcquisitionCfIdx", "AcqTimeStamp [s]", "AcqTimeStampfrac [NTP]", "PRF", "UmambiguousSpeed",
       "PortIdentifier", "PortVersionMajor", "PortVersionMinor", "PortSize", "BodyEndianness",
       "PortIndex", "HeaderVersionMajor", "HeaderVersionMinor"});
  } else if (selected_type_ == kCanTarget) {
    table_data_->setVerticalHeaderLabels(
      {"CycleDuration [s]", "NumOfTargets", "CycleCount", "AcquisitionSetup", "AcqTimeStamp [s]",
       "AcqTimeStampfrac [NTP]", "", "", "", "", "", "", "", "", "", "", ""});
  } else if (selected_type_ == kCanObject) {
    table_data_->setColumnCount(0);
    table_data_->setVerticalHeaderLabels(
      {"CycleDuration [s]", "NumOfObjects", "CycleCount", "Speed [km/h]", "SpeedQuality",
       "YawRate [rad/s]", "YawRateQuality", "DynamicSource", "", "", "", "", "", "", "", "", ""});
  } else if (selected_type_ == kPortObject) {
    table_data_->setColumnCount(0);
    table_data_->setVerticalHeaderLabels(
      {"CycleDuration [s]", "NumOfObjects", "AcqTimeStamp [s]", "AcqTimeStampfrac [NTP]",
       "PortIdentifier", "PortVersionMajor", "PortVersionMinor", "PortSize", "BodyEndianness",
       "PortIndex", "HeaderVersionMajor", "HeaderVersionMinor", "", "", "", "", ""});
  }
}

void SmartRadarStatus::check_data()
{
  if (rclcpp::ok())  // Check if ROS2 is still running
  {
    executor_.spin_some(std::chrono::milliseconds(2));
  }
}

}  // namespace smart_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(smart_rviz_plugin::SmartRadarStatus, rviz_common::Panel)
