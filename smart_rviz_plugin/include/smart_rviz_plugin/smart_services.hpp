// SPDX-License-Identifier: Apache-2.0
#ifndef SMART_RVIZ_PLUGIN__SMART_SERVICES_HPP_
#define SMART_RVIZ_PLUGIN__SMART_SERVICES_HPP_

#include <QComboBox>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTabWidget>
#include <QTableWidget>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <QFormLayout>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "umrr_ros2_msgs/srv/send_command.hpp"
#include "umrr_ros2_msgs/srv/set_mode.hpp"
#include "umrr_ros2_msgs/srv/get_status.hpp"
#include "umrr_ros2_msgs/srv/get_mode.hpp"

namespace smart_rviz_plugin
{
///
/// @brief      The class for the sending instructions to the sensors.
///
/// This class provides a graphical user interface (GUI) panel for sending
/// instructions to the sensor within the RViz environment. It extends the
/// rviz_common::Panel class and includes functionalities for viewing, recording
/// sensor data and sending instructions to the sensor.
///
class SmartRadarService : public rviz_common::Panel
{
  Q_OBJECT

public:
  /// @brief      Constructor for the SmartRadarService class.
  /// @param      parent  The parent widget. Defaults to nullptr.
  explicit SmartRadarService(QWidget * parent = nullptr);
  ~SmartRadarService() override;

private slots:
  /// @brief      Slot function to send a parameter instruction to the sensor.
  void on_send_param();

  /// @brief      Slot function to send a command instruction to the sensor.
  void on_send_command();

  /// @brief      Slot function to send a status instruction to the sensor.
  void on_get_status();

  /// @brief      Slot function to display instruction set of selected sensor model.
  /// @param      index The index of the selected file in the combo box.
  void on_file_selected(int index);

  /// @brief      Slot function to handle param selection from the table.
  void on_param_selection();
  
  /// @brief      Slot function to handle command selection from the table.
  void on_command_selection();

  /// @brief      Slot function to handle status selection from the table.
  void on_status_selection();
  
private:
  /// @brief      Initializes the panel's components.
  void initialize();

  /// @brief      Initializes the ROS2 client.
  void setup_ros_clients();

  /// @brief      Create widgets for the plugin.
  void create_widgets();
  
  /// @brief      Create layout.
  void setup_layout();

  /// @brief      Set up the connections based on activity.
  void setup_connections();

  /// @brief      Read parameters from user interface json file.
  void read_param_json_data();

  /// @brief      Read commands from user interface json file.
  void read_command_json_data();

  /// @brief      Read statuses from user interface json file.
  void read_status_json_data();

  /// @brief      Populates the table with the available user interface files.
  void populate_file_menu();

  /// @brief      Spin the panel's executor and expire overdue requests.
  void tick();

  /// @brief      Send a request without blocking the GUI thread.
  template<typename ServiceT>
  void send_request(
    const typename rclcpp::Client<ServiceT>::SharedPtr & client,
    typename ServiceT::Request::SharedPtr request, const QString & service_name);

  /// @brief      Drop the outstanding request, if any, so a late reply is ignored.
  void cancel_pending();

  /// @brief      Enable or disable the send buttons.
  void set_busy(bool busy);

  /// @brief      Show an error in the response area.
  void report_error(const QString & message);

  /// @brief      Find the directory that holds the sensor user-interface tables.
  QString find_user_interfaces_dir(const QString & relative_file, QStringList * tried) const;

  /// @brief      Load a JSON table file; reports failures in the response area.
  bool load_json(const QString & path, const char * array_key, QJsonObject * object);

  // File paths
  QString param_json_file_path;
  QString command_json_file_path;
  QString status_json_file_path;

  // Layout components
  QVBoxLayout * main_layout{nullptr};
  QTabWidget * tab_widget{nullptr};
  QWidget * param_tab{nullptr};
  QWidget * command_tab{nullptr};
  QWidget * status_tab{nullptr};

  // Parameter input fields
  QLineEdit * param_name_line_edit{nullptr};
  QLineEdit * param_value_line_edit{nullptr};
  QLineEdit * param_sensor_id{nullptr};
  QLineEdit * param_section_name{nullptr};
  
  // Command input fields
  QLineEdit * command_name_line_edit{nullptr};
  //QLineEdit * command_comment_line_edit{nullptr};
  QLineEdit * command_sensor_id{nullptr};
  QLineEdit * command_value_line_edit{nullptr};
  QLineEdit * command_section_name{nullptr};

  // Status input fields
  QLineEdit * status_name_line_edit{nullptr};
  QLineEdit * status_sensor_id{nullptr};
  QLineEdit * status_section_name{nullptr};

  // Buttons
  QPushButton * send_param_button{nullptr};
  QPushButton * send_command_button{nullptr};
  QPushButton * send_status_button{nullptr};
  
  // Tables and combo boxes
  QTableWidget * param_table_widget{nullptr};
  QTableWidget * command_table_widget{nullptr};
  QTableWidget * status_table_widget{nullptr};
  QTextEdit * response_text_edit{nullptr};
  QComboBox * file_selector_combo_box{nullptr};
  QComboBox * param_value_type{nullptr};
  QComboBox * status_value_type{nullptr};
  QComboBox * param_action_combo{nullptr};

  // Forms
  QFormLayout * param_form{nullptr};
  QFormLayout * command_form{nullptr};
  QFormLayout * status_form{nullptr};

  // ROS2 components
  rclcpp::Node::SharedPtr client_node;
  rclcpp::executors::SingleThreadedExecutor executor_;
  QTimer * spin_timer_{nullptr};
  std::function<void()> cancel_request_;
  std::chrono::steady_clock::time_point deadline_{};
  rclcpp::Client<umrr_ros2_msgs::srv::SetMode>::SharedPtr mode_client;
  rclcpp::Client<umrr_ros2_msgs::srv::SendCommand>::SharedPtr command_client;
  rclcpp::Client<umrr_ros2_msgs::srv::GetStatus>::SharedPtr status_client;
  rclcpp::Client<umrr_ros2_msgs::srv::GetMode>::SharedPtr get_param_client;

  // Constants
  static constexpr auto REQUEST_TIMEOUT = std::chrono::seconds(10);
};
}  // namespace smart_rviz_plugin

#endif  // SMART_RVIZ_PLUGIN__SMART_SERVICES_HPP_
