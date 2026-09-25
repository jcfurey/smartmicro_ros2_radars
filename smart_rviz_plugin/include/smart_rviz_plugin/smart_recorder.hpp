// SPDX-License-Identifier: Apache-2.0
#ifndef SMART_RVIZ_PLUGIN__SMART_RECORDER_HPP_
#define SMART_RVIZ_PLUGIN__SMART_RECORDER_HPP_

#include <QComboBox>
#include <QFile>
#include <QFileDialog>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QSplitter>
#include <QTableWidget>
#include <QTextStream>
#include <QTimer>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cstddef>
#include <map>
#include <string>
#include <vector>

namespace smart_rviz_plugin
{
struct TargetData
{
  float range;
  float power;
  float azimuth_deg;
  float elevation_deg;
  float rcs;
  float noise;
  float snr;
  float radial_speed;
  float azimuth_angle;
  float elevation_angle;
  float variance_range;
  float variance_speed;
  float variance_azimuth_angle;
  float variance_elevation_angle;
  float false_alarm_probability;
  uint32_t flags;
  uint16_t peak_idx;
  uint32_t timestamp_sec;
  uint32_t timestamp_nanosec;
};

struct ObjectData
{
  float x_pos;
  float y_pos;
  float z_pos;
  float speed_abs;
  float heading;
  float length;
  float mileage;
  float quality;
  float acceleration;
  int16_t object_id;
  uint16_t idle_cycles;
  uint16_t spline_idx;
  uint8_t object_class;
  uint16_t status;
  uint32_t timestamp_sec;
  uint32_t timestamp_nanosec;
};

///
/// @brief      The class for the target and object recorder.
///
/// This class provides a graphical user interface (GUI) panel for viewing
/// the sensor data within the RViz environment. It extends the rviz_common::Panel
/// class and includes functionalities for selecting, recording
/// and saving the sensor data as csv format.
///
class SmartRadarRecorder : public rviz_common::Panel
{
  Q_OBJECT

public:
  ///
  /// @brief      Constructor for the SmartRadarRecorder class.
  ///
  /// @param      parent  The parent widget. Defaults to nullptr.
  ///
  explicit SmartRadarRecorder(QWidget * parent = nullptr);

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

  /// Default cap on recorded rows (targets/objects), about 70 MB of samples.
  static constexpr std::size_t DEFAULT_MAX_RECORDED_ROWS = 1000000;

private slots:
  ///
  /// @brief      Slot function to start recording the sensor data being viewed.
  ///
  void start_recording();

  ///
  /// @brief      Slot function to stop recording the sensor data.
  ///
  void stop_recording();

  ///
  /// @brief      Slot function to save the recorded sensor data as csv fomrat.
  ///
  void save_data();

  ///
  /// @brief      Slot function to check the data is being published.
  ///
  void check_data();

  ///
  /// @brief      Slot function to update the table based on the topic selected.
  ///
  void update_table();

  ///
  /// @brief      Slot function to refresh the list of available cloud topics.
  ///
  void refresh_topic_list();

private:
  ///
  /// @brief      Initializes the panel's components and ROS2 client.
  ///
  /// This function sets up the GUI elements, initializes the ROS2 node and
  /// client for the recorder, and connects the signals and
  /// slots.
  ///
  void initialize();

  ///
  /// @brief      Subscribe to the port targets topics publisheb by the smartmicro_radar_node.
  ///
  void port_target_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string topic_name);

  ///
  /// @brief      Subscribe to the can targets topics publisheb by the smartmicro_radar_node.
  ///
  void can_target_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string topic_name);

  ///
  /// @brief      Subscribe to the port objects topics publisheb by the smartmicro_radar_node.
  ///
  void port_object_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string topic_name);

  ///
  /// @brief      Subscribe to the can objects topics publisheb by the smartmicro_radar_node.
  ///
  void can_object_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string topic_name);

  ///
  /// @brief      Function to handle the data recording for target topics.
  ///
  bool update_target_recorded_data(
    float range, float power, float azimuth_deg,
    float elevation_deg, float rcs, float noise, float snr, float radial_speed,
    float azimuth_angle, float elevation_angle, float variance_range, float variance_speed,
    float variance_azimuth_angle, float variance_elevation_angle, float false_alarm_probability,
    uint32_t flags, uint16_t peak_idx, uint32_t timestamp_sec, uint32_t timestamp_nanosec);

  ///
  /// @brief      Function to handle the data recording for objects topics.
  ///
  bool update_object_recorded_data(
    float x_pos, float y_pos, float z_pos, float speed_abs,
    float heading, float length, float mileage, float quality, float acceleration,
    int16_t object_id, uint16_t idle_cycles, uint16_t spline_idx, uint8_t object_class,
    uint16_t status, uint32_t timestamp_sec, uint32_t timestamp_nanosec);

  void clear_recorded_data();
  void return_to_ready_state();

  /// Stop recording without a modal prompt and offer save/discard.
  void finish_recording(const QString & reason);

  /// True while the recording is below the row cap; stops the recording at the cap.
  bool has_capacity();

  /// (Re-)subscribe to the selected topic only.
  void subscribe_selected();

  QTableWidget * table_data_{nullptr};
  QTableWidget * table_timestamps_{nullptr};
  QSplitter * splitter_{nullptr};
  QSplitter * horiz_splitter_{nullptr};
  QComboBox * topic_dropdown_{nullptr};
  QVBoxLayout * gui_layout_{nullptr};
  QPushButton * start_button_{nullptr};
  QPushButton * stop_button_{nullptr};
  QPushButton * save_button_{nullptr};
  QSpinBox * max_rows_{nullptr};
  QLabel * status_{nullptr};
  QTimer * timer_{nullptr};
  QTimer * topic_refresh_timer_{nullptr};
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  /// Radar cloud topics currently offered in the dropdown.
  std::vector<std::string> topics_;
  /// Only the selected topic is subscribed.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  std::vector<TargetData> target_recorded_data;
  std::vector<ObjectData> object_recorded_data;
  std::string selected_topic_;
  std::string subscribed_topic_;
  std::string recording_topic_;
  bool recording_active_{false};
  bool pending_save_{false};
};

}  // namespace smart_rviz_plugin

#endif  // SMART_RVIZ_PLUGIN__SMART_RECORDER_HPP_
