// SPDX-License-Identifier: Apache-2.0
#include "smart_rviz_plugin/umrr96_config.hpp"

#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <atomic>
#include <cmath>
#include <stdexcept>
#include <pluginlib/class_list_macros.hpp>

namespace smart_rviz_plugin
{
namespace
{
const std::array<std::string, 4> names = {
  "frequency_sweep_idx", "range_toggle_mode", "tx_antenna_idx",
  "output_control_target_list_can"};

QJsonObject response_values(const std::string & text, uint32_t sensor)
{
  QJsonParseError error;
  const auto document = QJsonDocument::fromJson(QByteArray::fromStdString(text), &error);
  const auto object = document.object();
  if (error.error != QJsonParseError::NoError || !document.isObject()) {
    throw std::runtime_error("Unexpected service response. Use umrr96_live.launch.py.");
  }
  if (!object.value("success").toBool()) {
    const auto message = object.value("error").toString("Sensor rejected the request.");
    throw std::runtime_error(message.toStdString());
  }
  if (object.value("sensor_id").toDouble(-1) != sensor || !object.value("values").isObject()) {
    throw std::runtime_error("Response does not identify the selected sensor.");
  }
  return object.value("values").toObject();
}

int value(const QJsonObject & values, const std::string & name)
{
  const auto entry = values.value(QString::fromStdString(name)).toObject();
  const auto number = entry.value("value");
  if (entry.value("response_type").toInt() != 1 || !number.isDouble() ||
    !std::isfinite(number.toDouble()) || std::floor(number.toDouble()) != number.toDouble())
  {
    throw std::runtime_error("Missing or invalid value: " + name);
  }
  return number.toInt(-1);
}
}  // namespace

Umrr96Config::Umrr96Config(QWidget * parent) : rviz_common::Panel(parent)
{
  setMinimumWidth(360);
  auto layout = new QVBoxLayout(this);
  auto title = new QLabel("<b>UMRR-96 · Type 153</b>", this);
  layout->addWidget(title);
  auto device = new QHBoxLayout;
  device->addWidget(new QLabel("Sensor ID", this));
  sensor_ = new QLineEdit("230739", this);
  sensor_->setObjectName("sensor_id");
  sensor_->setToolTip("Decimal ID or hexadecimal with a 0x prefix.");
  device->addWidget(sensor_);
  refresh_ = new QPushButton("Read settings", this);
  refresh_->setObjectName("refresh");
  device->addWidget(refresh_);
  layout->addLayout(device);
  identity_ = new QLabel("Firmware: waiting for sensor", this);
  identity_->setObjectName("identity");
  layout->addWidget(identity_);

  auto live = new QGroupBox("Live measurement", this);
  auto live_layout = new QVBoxLayout(live);
  metrics_ = new QLabel("Waiting for live targets…", live);
  metrics_->setObjectName("metrics");
  metrics_->setWordWrap(true);
  live_layout->addWidget(metrics_);
  layout->addWidget(live);

  auto settings = new QGroupBox("Radar settings", this);
  auto grid = new QGridLayout(settings);
  grid->addWidget(new QLabel("Setting", settings), 0, 0);
  grid->addWidget(new QLabel("Last read", settings), 0, 1);
  grid->addWidget(new QLabel("Next value", settings), 0, 2);
  const std::array<QString, 4> labels = {"Range", "Range switching", "Antenna", "CAN targets"};
  const std::array<QString, 4> widget_names = {"range_mode", "range_toggle", "antenna", "can_output"};
  for (size_t i = 0; i < choices_.size(); ++i) {
    choices_[i] = new QComboBox(settings);
    choices_[i]->setObjectName(widget_names[i]);
    actual_labels_[i] = new QLabel("—", settings);
    actual_labels_[i]->setObjectName(widget_names[i] + "_actual");
    grid->addWidget(new QLabel(labels[i], settings), static_cast<int>(i) + 1, 0);
    grid->addWidget(actual_labels_[i], static_cast<int>(i) + 1, 1);
    grid->addWidget(choices_[i], static_cast<int>(i) + 1, 2);
    connect(choices_[i], QOverload<int>::of(&QComboBox::currentIndexChanged),
      this, [this] {controls();});
  }
  choices_[0]->addItem("Short · 1536 MHz", 2);
  choices_[0]->addItem("Medium · 512 MHz", 1);
  choices_[0]->addItem("Long · 226 MHz", 0);
  choices_[1]->addItem("Off", 0);
  choices_[1]->addItem("Short / medium", 1);
  choices_[1]->addItem("Short / long", 2);
  choices_[1]->addItem("Medium / long", 3);
  for (int i = 0; i < 3; ++i) {choices_[2]->addItem(QString::number(i), i);}
  choices_[2]->setToolTip("Antenna indices 0–2. Beam names are not documented in this interface.");
  choices_[3]->addItem("Enabled", 1);
  choices_[3]->addItem("Disabled", 0);
  choices_[3]->setToolTip("Disable CAN target output when using only Ethernet for targets.");
  layout->addWidget(settings);
  auto presets = new QHBoxLayout;
  preset_ = new QPushButton("Short-range Ethernet preset", this);
  preset_->setObjectName("preset");
  starting_ = new QPushButton("Starting values", this);
  starting_->setObjectName("starting");
  starting_->setToolTip("Stage the first values read by this panel. Click Apply to restore them.");
  presets->addWidget(preset_);
  presets->addWidget(starting_);
  layout->addLayout(presets);
  apply_ = new QPushButton("Apply changes", this);
  apply_->setObjectName("apply");
  layout->addWidget(apply_);
  auto note = new QLabel("Changes take effect immediately after Apply. They are not saved to EEPROM.", this);
  note->setWordWrap(true);
  layout->addWidget(note);
  feedback_ = new QLabel("Waiting for radar control services…", this);
  feedback_->setObjectName("feedback");
  feedback_->setWordWrap(true);
  feedback_->setTextFormat(Qt::PlainText);
  layout->addWidget(feedback_);
  advanced_open_ = new QPushButton("Advanced sensor controls…", this);
  advanced_open_->setObjectName("advanced");
  layout->addWidget(advanced_open_);
  create_advanced_dialog();
  connect(advanced_open_, &QPushButton::clicked, this, [this] {
      advanced_dialog_->show();
      advanced_dialog_->raise();
      if (!have_advanced_) {read_advanced();}
    });

  auto filtering = new QGroupBox("Filtering and density history", this);
  auto filter_layout = new QGridLayout(filtering);
  filter_mode_ = new QComboBox(filtering);
  filter_mode_->setObjectName("filter_mode");
  filter_mode_->addItem("Off (raw detections)", "off");
  filter_mode_->addItem("Quality only", "quality");
  filter_mode_->addItem("Stable mapping", "mapping");
  filter_mode_->addItem("Moving returns", "moving");
  filter_mode_->setToolTip("Quality only applies the SNR gate without scan persistence. "
    "Stable mapping assumes a stationary radar. Moving returns uses sensor-relative radial speed; "
    "it does not identify moving objects when the radar moves.");
  filter_snr_ = new QDoubleSpinBox(filtering);
  filter_snr_->setObjectName("filter_snr");
  filter_snr_->setRange(-20, 80);
  filter_snr_->setSuffix(" dB");
  filter_speed_ = new QDoubleSpinBox(filtering);
  filter_speed_->setObjectName("filter_speed");
  filter_speed_->setRange(0, 30);
  filter_speed_->setDecimals(2);
  filter_speed_->setSingleStep(.05);
  filter_speed_->setSuffix(" m/s");
  filter_speed_->setToolTip("Minimum absolute radial speed in Moving returns mode only.");
  filter_layout->addWidget(new QLabel("Mode", filtering), 0, 0);
  filter_layout->addWidget(filter_mode_, 0, 1);
  filter_layout->addWidget(new QLabel("Minimum SNR", filtering), 1, 0);
  filter_layout->addWidget(filter_snr_, 1, 1);
  filter_layout->addWidget(new QLabel("Minimum radial speed", filtering), 2, 0);
  filter_layout->addWidget(filter_speed_, 2, 1);
  decay_ = new QDoubleSpinBox(filtering);
  decay_->setObjectName("density_decay");
  decay_->setRange(.1, 30);
  decay_->setDecimals(2);
  decay_->setSingleStep(.1);
  decay_->setSuffix(" s");
  decay_->setValue(2.0);
  decay_->setToolTip("Time for a cell's stored hit weight to fall to 37%. "
    "Shorter values fade trails faster. Changing decay preserves existing hits.");
  filter_layout->addWidget(new QLabel("Density decay", filtering), 3, 0);
  filter_layout->addWidget(decay_, 3, 1);
  filter_apply_ = new QPushButton("Apply view settings", filtering);
  filter_apply_->setObjectName("filter_apply");
  filter_layout->addWidget(filter_apply_, 4, 0, 1, 2);
  filter_actual_ = new QLabel("Waiting for view node…", filtering);
  filter_actual_->setObjectName("filter_actual");
  filter_actual_->setWordWrap(true);
  filter_actual_->setTextFormat(Qt::PlainText);
  filter_layout->addWidget(filter_actual_, 5, 0, 1, 2);
  filter_feedback_ = new QLabel("Filter changes clear history. Decay changes preserve existing hits.", filtering);
  filter_feedback_->setObjectName("filter_feedback");
  filter_feedback_->setWordWrap(true);
  filter_feedback_->setTextFormat(Qt::PlainText);
  filter_layout->addWidget(filter_feedback_, 6, 0, 1, 2);
  layout->addWidget(filtering);
  connect(filter_mode_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, [this] {filter_dirty_ = true;});
  connect(filter_snr_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this, [this] {filter_dirty_ = true;});
  connect(filter_speed_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this, [this] {filter_dirty_ = true;});
  connect(decay_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this, [this](double value) {staged_decay_ = value; filter_dirty_ = true;});
  connect(filter_apply_, &QPushButton::clicked, this, &Umrr96Config::apply_filter);
  filter_apply_->setEnabled(false);
  layout->addStretch();

  static std::atomic<unsigned> instance{0};
  node_ = std::make_shared<rclcpp::Node>("umrr96_config_" + std::to_string(instance++),
    rclcpp::NodeOptions().use_global_arguments(false));
  getter_ = node_->create_client<GetMode>("/smart_radar/get_radar_mode");
  status_ = node_->create_client<GetStatus>("/smart_radar/get_radar_status");
  setter_ = node_->create_client<SetMode>("/smart_radar/set_radar_mode");
  filter_setter_ = node_->create_client<rcl_interfaces::srv::SetParametersAtomically>(
    "/umrr96_views/set_parameters_atomically");
  filter_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/smart_radar/filter_status", rclcpp::QoS(1).transient_local(),
    [this](std_msgs::msg::String::ConstSharedPtr msg) {filter_status(msg->data);});
  header_ = node_->create_subscription<umrr_ros2_msgs::msg::PortTargetHeader>(
    "/smart_radar/port_targetheader_0", rclcpp::SensorDataQoS(),
    [this](umrr_ros2_msgs::msg::PortTargetHeader::ConstSharedPtr msg) {
      arrivals_.push_back(Clock::now());
      targets_ = msg->number_of_targets;
      cycle_ms_ = msg->cycle_time * 1000.0;
    });
  executor_.add_node(node_);
  connect(refresh_, &QPushButton::clicked, this, [this] {read_settings();});
  connect(apply_, &QPushButton::clicked, this, &Umrr96Config::apply);
  connect(preset_, &QPushButton::clicked, this, [this] {
      stage({2, 0, 0, 0});
      feedback_->setText("Short-range Ethernet preset staged. Click Apply changes to send it.");
    });
  connect(starting_, &QPushButton::clicked, this, [this] {
      stage(initial_);
      feedback_->setText("Starting values staged. Click Apply changes to restore them.");
    });
  connect(sensor_, &QLineEdit::textEdited, this, [this] {changed_sensor();});
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &Umrr96Config::tick);
  timer_->start(50);
  controls();
}

Umrr96Config::~Umrr96Config()
{
  timer_->stop();
  cancel_pending();
  if (filter_pending_) {filter_setter_->remove_pending_request(filter_pending_id_);}
  executor_.remove_node(node_);
}

void Umrr96Config::cancel_pending()
{
  if (operation_ == Operation::Read || operation_ == Operation::AdvancedRead) {
    getter_->remove_pending_request(pending_id_);
  }
  if (operation_ == Operation::Write || operation_ == Operation::AdvancedWrite) {
    setter_->remove_pending_request(pending_id_);
  }
  if (operation_ == Operation::Identity) {status_->remove_pending_request(pending_id_);}
  operation_ = Operation::None;
}

void Umrr96Config::tick()
{
  if (!rclcpp::ok()) {return;}
  executor_.spin_some(std::chrono::milliseconds(2));
  const auto now = Clock::now();
  if (filter_pending_ && now > filter_deadline_) {
    filter_setter_->remove_pending_request(filter_pending_id_);
    filter_pending_ = false;
    filter_feedback_->setText("View settings request timed out. Check current values before retrying.");
  }
  const bool filter_available = filter_ready_ && !filter_pending_ &&
    filter_setter_->service_is_ready();
  filter_apply_->setEnabled(filter_available);
  filter_mode_->setEnabled(filter_available);
  filter_snr_->setEnabled(filter_available);
  filter_speed_->setEnabled(filter_available);
  decay_->setEnabled(filter_available && decay_ready_);
  while (!arrivals_.empty() && now - arrivals_.front() > std::chrono::seconds(2)) {
    arrivals_.pop_front();
  }
  if (arrivals_.size() > 1) {
    const auto elapsed = std::chrono::duration<double>(arrivals_.back() - arrivals_.front()).count();
    metrics_->setText(QString("%1 targets · %2 Hz\n%3 ms sensor cycle")
      .arg(targets_).arg((arrivals_.size() - 1) / elapsed, 0, 'f', 1).arg(cycle_ms_, 0, 'f', 1));
  } else {
    metrics_->setText("Waiting for live targets…");
  }
  if (operation_ != Operation::None && now > deadline_) {
    const auto expired = operation_;
    cancel_pending();
    if (expired == Operation::AdvancedWrite) {
      recover_advanced("Write timed out; some changes may have reached the sensor.");
    } else if (expired == Operation::AdvancedRead) {
      fail_advanced((advanced_error_.isEmpty() ? QString{} : advanced_error_ + " ") +
        "Advanced read timed out. Actual state is unknown; read settings before retrying.");
    } else {
      fail("Request timed out. Read settings before retrying a change.");
    }
  }
  if (!have_actual_ && operation_ == Operation::None && now >= next_read_) {
    next_read_ = now + std::chrono::seconds(3);
    if (getter_->service_is_ready()) {read_settings();}
  }
}

void Umrr96Config::filter_status(const std::string & text)
{
  const auto document = QJsonDocument::fromJson(QByteArray::fromStdString(text));
  if (!document.isObject()) {return;}
  const auto data = document.object();
  const auto mode = data.value("mode").toString();
  const int index = filter_mode_->findData(mode);
  if (index < 0 || !data.value("min_snr_db").isDouble() ||
    !data.value("min_abs_speed").isDouble()) {return;}
  const auto decay = data.value("density_decay_seconds");
  decay_ready_ = decay.isDouble() && std::isfinite(decay.toDouble()) &&
    decay.toDouble() >= .1 && decay.toDouble() <= 30;
  filter_actual_->setText(QString("Current: %1 · kept %2 / %3\nRejected: quality %4 · motion %5 · persistence %6")
    .arg(filter_mode_->itemText(index)).arg(data.value("accepted").toInt())
    .arg(data.value("input").toInt()).arg(data.value("rejected_quality").toInt())
    .arg(data.value("rejected_motion").toInt()).arg(data.value("rejected_temporal").toInt()));
  filter_actual_->setText(filter_actual_->text() + (decay_ready_ ?
    QString("\nDensity decay: %1 s").arg(decay.toDouble(), 0, 'f', 2) :
    "\nLive decay control needs the updated view node."));
  if (!filter_ready_ || (!filter_dirty_ && !filter_pending_)) {
    const QSignalBlocker mode_block(filter_mode_), snr_block(filter_snr_), speed_block(filter_speed_),
      decay_block(decay_);
    filter_mode_->setCurrentIndex(index);
    filter_snr_->setValue(data.value("min_snr_db").toDouble());
    filter_speed_->setValue(data.value("min_abs_speed").toDouble());
    if (decay_ready_) {
      staged_decay_ = decay.toDouble();
      decay_->setValue(staged_decay_);
    }
  }
  filter_ready_ = true;
}

void Umrr96Config::apply_filter()
{
  if (filter_pending_ || !filter_setter_->service_is_ready()) {return;}
  using Service = rcl_interfaces::srv::SetParametersAtomically;
  auto request = std::make_shared<Service::Request>();
  request->parameters = {
    rclcpp::Parameter("filter_mode", filter_mode_->currentData().toString().toStdString()).to_parameter_msg(),
    rclcpp::Parameter("filter_min_snr_db", filter_snr_->value()).to_parameter_msg(),
    rclcpp::Parameter("filter_min_abs_speed", filter_speed_->value()).to_parameter_msg()};
  if (decay_ready_) {
    request->parameters.push_back(rclcpp::Parameter("decay_seconds", staged_decay_).to_parameter_msg());
  }
  filter_pending_ = true;
  filter_deadline_ = Clock::now() + std::chrono::seconds(3);
  filter_feedback_->setText("Applying view settings…");
  filter_pending_id_ = filter_setter_->async_send_request(request,
    [this](rclcpp::Client<Service>::SharedFuture future) {
      filter_pending_ = false;
      try {
        const auto result = future.get()->result;
        if (result.successful) {
          filter_dirty_ = false;
          filter_feedback_->setText("View settings applied. Decay-only edits preserve history.");
        } else {
          filter_feedback_->setText("View settings rejected: " + QString::fromStdString(result.reason));
        }
      } catch (const std::exception & error) {
        filter_feedback_->setText(QString::fromUtf8(error.what()));
      }
    }).request_id;
}

uint32_t Umrr96Config::sensor_id() const
{
  bool valid = false;
  const auto text = sensor_->text().trimmed();
  const auto id = text.toUInt(&valid, text.startsWith("0x", Qt::CaseInsensitive) ? 16 : 10);
  if (!valid || id == 0) {throw std::runtime_error("Enter a valid sensor ID.");}
  return id;
}

void Umrr96Config::fail(const QString & message)
{
  operation_ = Operation::None;
  have_actual_ = false;
  next_read_ = Clock::now() + std::chrono::seconds(3);
  feedback_->setText(message);
  controls();
}

void Umrr96Config::controls()
{
  // Widget population emits index-change signals before all controls exist.
  if (!apply_) {return;}
  const bool idle = operation_ == Operation::None;
  sensor_->setEnabled(idle);
  refresh_->setEnabled(idle);
  for (auto choice : choices_) {choice->setEnabled(idle && have_actual_);}
  preset_->setEnabled(idle && have_actual_);
  starting_->setEnabled(idle && have_actual_ && have_initial_);
  apply_->setEnabled(idle && have_actual_ && selected() != actual_);
  if (advanced_apply_) {
    advanced_open_->setEnabled(idle);
    advanced_refresh_->setEnabled(idle);
    advanced_starting_->setEnabled(idle && have_advanced_ && have_advanced_initial_);
    prf_mode_->setEnabled(idle && have_advanced_);
    prf_index_->setEnabled(idle && have_advanced_);
    bool valid = true;
    for (size_t i = 0; i < velocity_.size(); ++i) {
      velocity_[i]->setEnabled(idle && have_advanced_);
      if (i % 2 == 0) {valid &= advanced_staged_[i + 3] <= advanced_staged_[i + 4];}
    }
    advanced_apply_->setEnabled(idle && have_advanced_ && valid &&
      advanced_staged_ != advanced_actual_);
  }
}

Umrr96Config::Values Umrr96Config::selected() const
{
  Values values{};
  for (size_t i = 0; i < values.size(); ++i) {values[i] = choices_[i]->currentData().toInt();}
  return values;
}

void Umrr96Config::stage(const Values & values)
{
  for (size_t i = 0; i < values.size(); ++i) {
    const QSignalBlocker blocked(choices_[i]);
    choices_[i]->setCurrentIndex(choices_[i]->findData(values[i]));
  }
  controls();
}

void Umrr96Config::read_settings(bool verify)
{
  if (operation_ != Operation::None) {return;}
  try {
    const auto id = sensor_id();
    if (!getter_->service_is_ready()) {
      throw std::runtime_error("Radar control unavailable. Start umrr96_live.launch.py.");
    }
    auto request = std::make_shared<GetMode::Request>();
    request->sensor_id = id;
    request->section_name = "auto_interface_0dim";
    request->params.assign(names.begin(), names.end());
    request->param_types.assign(names.size(), 3);
    verify_ = verify;
    operation_ = Operation::Read;
    deadline_ = Clock::now() + std::chrono::seconds(5);
    feedback_->setText(verify ? "Checking sensor readback…" : "Reading sensor settings…");
    pending_id_ = getter_->async_send_request(request,
      [this, id](rclcpp::Client<GetMode>::SharedFuture future) {
        operation_ = Operation::None;
        try {
          const auto values = response_values(future.get()->res, id);
          Values actual{};
          for (size_t i = 0; i < names.size(); ++i) {
            actual[i] = value(values, names[i]);
            if (choices_[i]->findData(actual[i]) < 0) {
              throw std::runtime_error("Unsupported value returned for " + names[i]);
            }
          }
          actual_ = actual;
          have_actual_ = true;
          RCLCPP_INFO(node_->get_logger(),
            "Settings read: sweep=%d, range switching=%d, antenna=%d, CAN targets=%d",
            actual_[0], actual_[1], actual_[2], actual_[3]);
          if (!have_initial_) {initial_ = actual_; have_initial_ = true;}
          for (size_t i = 0; i < actual_.size(); ++i) {
            actual_labels_[i]->setText(choices_[i]->itemText(choices_[i]->findData(actual_[i])));
          }
          stage(actual_);
          if (verify_ && actual_ != expected_) {
            feedback_->setText("Readback differs from the requested settings. Showing actual values.");
          } else {
            feedback_->setText(verify_ ? "Changes applied and verified." : "Settings read from sensor.");
          }
          if (!verify_) {read_identity();}
          controls();
        } catch (const std::exception & error) {
          fail(QString::fromStdString(error.what()));
        }
      }).request_id;
    controls();
  } catch (const std::exception & error) {fail(QString::fromStdString(error.what()));}
}

void Umrr96Config::read_identity()
{
  if (!status_->service_is_ready()) {return;}
  const auto id = sensor_id();
  auto request = std::make_shared<GetStatus::Request>();
  request->sensor_id = id;
  request->section_name = "auto_interface";
  request->statuses = {"sw_version_major", "sw_version_minor", "sw_version_patch"};
  request->status_types = {1, 1, 1};
  operation_ = Operation::Identity;
  deadline_ = Clock::now() + std::chrono::seconds(5);
  pending_id_ = status_->async_send_request(request,
    [this, id](rclcpp::Client<GetStatus>::SharedFuture future) {
      operation_ = Operation::None;
      try {
        const auto values = response_values(future.get()->res, id);
        identity_->setText(QString("Firmware %1.%2.%3 · ID 0x%4")
          .arg(value(values, "sw_version_major")).arg(value(values, "sw_version_minor"))
          .arg(value(values, "sw_version_patch")).arg(id, 8, 16, QChar('0')));
        RCLCPP_INFO(node_->get_logger(), "%s", identity_->text().toStdString().c_str());
      } catch (const std::exception & error) {
        identity_->setText("Firmware unavailable: " + QString::fromStdString(error.what()));
      }
      controls();
    }).request_id;
}

void Umrr96Config::apply()
{
  if (!have_actual_ || operation_ != Operation::None) {return;}
  try {
    const auto id = sensor_id();
    if (!setter_->service_is_ready()) {throw std::runtime_error("Tuning service unavailable.");}
    expected_ = selected();
    auto request = std::make_shared<SetMode::Request>();
    request->sensor_id = id;
    request->section_name = "auto_interface_0dim";
    for (size_t i = 0; i < names.size(); ++i) {
      if (expected_[i] != actual_[i]) {
        request->params.push_back(names[i]);
        request->values.push_back(std::to_string(expected_[i]));
        request->value_types.push_back(3);
      }
    }
    if (request->params.empty()) {return;}
    operation_ = Operation::Write;
    deadline_ = Clock::now() + std::chrono::seconds(5);
    feedback_->setText("Applying changes…");
    pending_id_ = setter_->async_send_request(request,
      [this, id](rclcpp::Client<SetMode>::SharedFuture future) {
        operation_ = Operation::None;
        try {
          response_values(future.get()->res, id);
          read_settings(true);
        } catch (const std::exception & error) {fail(QString::fromStdString(error.what()));}
      }).request_id;
    controls();
  } catch (const std::exception & error) {fail(QString::fromStdString(error.what()));}
}

void Umrr96Config::changed_sensor()
{
  have_actual_ = have_initial_ = false;
  have_advanced_ = have_advanced_initial_ = false;
  advanced_writes_.clear();
  for (auto label : advanced_actual_labels_) {label->setText("—");}
  advanced_feedback_->setText("Read advanced settings for the selected sensor.");
  identity_->setText("Firmware: waiting for sensor");
  for (auto label : actual_labels_) {label->setText("—");}
  next_read_ = Clock::now() + std::chrono::seconds(1);
  controls();
  Q_EMIT configChanged();
}

void Umrr96Config::load(const rviz_common::Config & config)
{
  const QSignalBlocker blocked(this);
  rviz_common::Panel::load(config);
  QString id;
  if (config.mapGetString("Sensor ID", &id)) {sensor_->setText(id);}
  changed_sensor();
}

void Umrr96Config::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Sensor ID", sensor_->text());
}
}  // namespace smart_rviz_plugin

PLUGINLIB_EXPORT_CLASS(smart_rviz_plugin::Umrr96Config, rviz_common::Panel)
