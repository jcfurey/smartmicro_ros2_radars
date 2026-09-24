// SPDX-License-Identifier: Apache-2.0
#ifndef SMART_RVIZ_PLUGIN__UMRR96_CONFIG_HPP_
#define SMART_RVIZ_PLUGIN__UMRR96_CONFIG_HPP_

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <array>
#include <chrono>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <std_msgs/msg/string.hpp>
#include <rviz_common/panel.hpp>
#include <umrr_ros2_msgs/msg/port_target_header.hpp>
#include <umrr_ros2_msgs/srv/get_mode.hpp>
#include <umrr_ros2_msgs/srv/get_status.hpp>
#include <umrr_ros2_msgs/srv/set_mode.hpp>

namespace smart_rviz_plugin
{
class Umrr96Config : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit Umrr96Config(QWidget * parent = nullptr);
  ~Umrr96Config() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  using GetMode = umrr_ros2_msgs::srv::GetMode;
  using GetStatus = umrr_ros2_msgs::srv::GetStatus;
  using SetMode = umrr_ros2_msgs::srv::SetMode;
  using Clock = std::chrono::steady_clock;
  using Values = std::array<int, 4>;
  enum class Operation { None, Read, Write, Identity };

  void tick();
  void read_settings(bool verify = false);
  void read_identity();
  void apply();
  void stage(const Values & values);
  void controls();
  void fail(const QString & message);
  void cancel_pending();
  void changed_sensor();
  void apply_filter();
  void filter_status(const std::string & text);
  uint32_t sensor_id() const;
  Values selected() const;

  QLineEdit * sensor_{};
  QLabel * identity_{};
  QLabel * metrics_{};
  QLabel * feedback_{};
  std::array<QComboBox *, 4> choices_{};
  std::array<QLabel *, 4> actual_labels_{};
  QPushButton * refresh_{};
  QPushButton * apply_{};
  QPushButton * preset_{};
  QPushButton * starting_{};
  QTimer * timer_{};
  QComboBox * filter_mode_{};
  QDoubleSpinBox * filter_snr_{};
  QDoubleSpinBox * filter_speed_{};
  QPushButton * filter_apply_{};
  QLabel * filter_actual_{};
  QLabel * filter_feedback_{};
  bool filter_ready_{false};
  bool filter_dirty_{false};
  bool filter_pending_{false};
  int64_t filter_pending_id_{};
  Clock::time_point filter_deadline_{};
  Values actual_{};
  Values initial_{};
  Values expected_{};
  bool have_actual_{false};
  bool have_initial_{false};
  bool verify_{false};
  Operation operation_{Operation::None};
  int64_t pending_id_{};
  Clock::time_point deadline_{};
  Clock::time_point next_read_{};
  std::deque<Clock::time_point> arrivals_;
  unsigned targets_{};
  double cycle_ms_{};
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<GetMode>::SharedPtr getter_;
  rclcpp::Client<GetStatus>::SharedPtr status_;
  rclcpp::Client<SetMode>::SharedPtr setter_;
  rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr filter_setter_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr filter_status_sub_;
  rclcpp::Subscription<umrr_ros2_msgs::msg::PortTargetHeader>::SharedPtr header_;
};
}  // namespace smart_rviz_plugin
#endif
