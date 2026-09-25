// SPDX-License-Identifier: Apache-2.0
#ifndef SMART_RVIZ_PLUGIN__SMART_STATUS_HPP_
#define SMART_RVIZ_PLUGIN__SMART_STATUS_HPP_

#include <QComboBox>
#include <QHeaderView>
#include <QPushButton>
#include <QSplitter>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <map>
#include <string>
#include <vector>

#include "umrr_ros2_msgs/msg/can_object_header.hpp"
#include "umrr_ros2_msgs/msg/can_target_header.hpp"
#include "umrr_ros2_msgs/msg/port_object_header.hpp"
#include "umrr_ros2_msgs/msg/port_target_header.hpp"

namespace smart_rviz_plugin
{
///
/// @brief      The class for the target and object headers.
///
/// This class provides a graphical user interface (GUI) panel for viewing
/// the sensor header data within the RViz environment. It extends the rviz_common::Panel
/// class and includes functionalities for selecting, viewing
/// the sensor header data.
///
class SmartRadarStatus : public rviz_common::Panel
{
  Q_OBJECT

public:
  ///
  /// @brief      Constructor for the SmartRadarStatus class.
  ///
  /// @param      parent  The parent widget. Defaults to nullptr.
  ///
  SmartRadarStatus(QWidget * parent = nullptr);

private slots:
  ///
  /// @brief      Slot function to update the table based on the topic selected.
  ///
  void update_table();

  ///
  /// @brief      Slot function to check the data is being published.
  ///
  void check_data();

  ///
  /// @brief      Slot function to refresh the list of available header topics.
  ///
  void refresh_topic_list();

private:
  ///
  /// @brief      Initializes the panel's components and ROS2 client.
  ///
  /// This function sets up the GUI elements, initializes the ROS2 node and
  /// client for the status, and connects the signals and
  /// slots.
  ///
  void initialize();

  ///
  /// @brief      Subscribe to the port target headers publisheb by the smartmicro_radar_node.
  ///
  void port_targetheader_callback(
    const umrr_ros2_msgs::msg::PortTargetHeader::SharedPtr msg, const std::string topic_name);

  ///
  /// @brief      Subscribe to the can target headers publisheb by the smartmicro_radar_node.
  ///
  void can_targetheader_callback(
    const umrr_ros2_msgs::msg::CanTargetHeader::SharedPtr msg, const std::string topic_name);

  ///
  /// @brief      Subscribe to the port object headers publisheb by the smartmicro_radar_node.
  ///
  void port_objectheader_callback(
    const umrr_ros2_msgs::msg::PortObjectHeader::SharedPtr msg, const std::string topic_name);

  ///
  /// @brief      Subscribe to the can object headers publisheb by the smartmicro_radar_node.
  ///
  void can_objectheader_callback(
    const umrr_ros2_msgs::msg::CanObjectHeader::SharedPtr msg, const std::string topic_name);

  QTableWidget * table_data_{nullptr};
  QSplitter * splitter_{nullptr};
  QComboBox * topic_dropdown_{nullptr};
  QVBoxLayout * gui_layout_{nullptr};
  QTimer * timer_{nullptr};
  QTimer * topic_refresh_timer_{nullptr};
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::string selected_topic_;
  std::string selected_type_;
  /// Header topics currently in the graph: name -> message type.
  std::map<std::string, std::string> topic_types_;
  /// Only the selected topic is subscribed.
  rclcpp::SubscriptionBase::SharedPtr subscription_;
};

}  // namespace smart_rviz_plugin

#endif  // SMART_RVIZ_PLUGIN__SMART_STATUS_HPP_
