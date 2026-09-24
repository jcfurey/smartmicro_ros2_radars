// SPDX-License-Identifier: Apache-2.0
#include "smart_rviz_plugin/umrr96_config.hpp"

#include <QDialogButtonBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace smart_rviz_plugin
{
namespace
{
const std::array<std::string, 9> names = {
  "prf_selector_manual", "prf_manual_value_idx", "prf_set_selector",
  "tv_min_speed_sweep_idx_0", "tv_max_speed_sweep_idx_0",
  "tv_min_speed_sweep_idx_1", "tv_max_speed_sweep_idx_1",
  "tv_min_speed_sweep_idx_2", "tv_max_speed_sweep_idx_2"};

QJsonObject response(const std::string & text, uint32_t id)
{
  QJsonParseError error;
  const auto document = QJsonDocument::fromJson(QByteArray::fromStdString(text), &error);
  const auto object = document.object();
  if (error.error != QJsonParseError::NoError || !document.isObject() ||
    object.value("sensor_id").toDouble(-1) != id || !object.value("success").toBool() ||
    !object.value("values").isObject())
  {
    throw std::runtime_error(object.value("error").toString(
      "Missing, rejected or malformed sensor response.").toStdString());
  }
  return object.value("values").toObject();
}

double read_number(const QJsonObject & values, const std::string & name)
{
  const auto entry = values.value(QString::fromStdString(name)).toObject();
  const auto number = entry.value("value");
  if (entry.value("response_type").toInt() != 1 || !number.isDouble() ||
    !std::isfinite(number.toDouble()))
  {
    throw std::runtime_error("Missing or invalid value: " + name);
  }
  return number.toDouble();
}
}  // namespace

void Umrr96Config::create_advanced_dialog()
{
  advanced_dialog_ = new QDialog(this);
  advanced_dialog_->setObjectName("advanced_dialog");
  advanced_dialog_->setWindowTitle("UMRR-96 advanced tuning");
  advanced_dialog_->setMinimumWidth(540);
  auto layout = new QVBoxLayout(advanced_dialog_);
  auto note = new QLabel("PRF selection and sensor velocity-validation windows. "
    "Changes take effect after Apply and are not saved to EEPROM.", advanced_dialog_);
  note->setWordWrap(true);
  layout->addWidget(note);
  auto grid = new QGridLayout;
  grid->addWidget(new QLabel("Setting"), 0, 0);
  grid->addWidget(new QLabel("Last read"), 0, 1);
  grid->addWidget(new QLabel("Next value"), 0, 2);
  const std::array<QString, 9> labels = {
    "PRF selection", "Manual PRF index", "PRF set", "Long-range minimum speed",
    "Long-range maximum speed", "Medium-range minimum speed", "Medium-range maximum speed",
    "Short-range minimum speed", "Short-range maximum speed"};
  for (size_t i = 0; i < names.size(); ++i) {
    grid->addWidget(new QLabel(labels[i]), static_cast<int>(i) + 1, 0);
    advanced_actual_labels_[i] = new QLabel("—", advanced_dialog_);
    advanced_actual_labels_[i]->setObjectName(QString::fromStdString(names[i]) + "_actual");
    grid->addWidget(advanced_actual_labels_[i], static_cast<int>(i) + 1, 1);
  }
  prf_mode_ = new QComboBox(advanced_dialog_);
  prf_mode_->setObjectName("prf_selector_manual");
  prf_mode_->addItem("Automatic", 0);
  prf_mode_->addItem("Manual", 1);
  prf_index_ = new QComboBox(advanced_dialog_);
  prf_index_->setObjectName("prf_manual_value_idx");
  for (int i = 0; i != 3; ++i) {prf_index_->addItem(QString::number(i), i);}
  prf_index_->setToolTip("Used in Manual mode. Physical PRF frequencies are not documented here.");
  grid->addWidget(prf_mode_, 1, 2);
  grid->addWidget(prf_index_, 2, 2);
  grid->addWidget(new QLabel("Set 0 supported"), 3, 2);
  connect(prf_mode_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this] {
      advanced_staged_[0] = prf_mode_->currentData().toInt();
      advanced_feedback_->setText("Values staged. Click Apply to send changes.");
      controls();
    });
  connect(prf_index_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this] {
      advanced_staged_[1] = prf_index_->currentData().toInt();
      advanced_feedback_->setText("Values staged. Click Apply to send changes.");
      controls();
    });
  for (size_t i = 0; i < velocity_.size(); ++i) {
    auto spin = velocity_[i] = new QDoubleSpinBox(advanced_dialog_);
    spin->setObjectName(QString::fromStdString(names[i + 3]));
    spin->setRange(-150, 150);
    spin->setDecimals(6);
    spin->setSingleStep(.5);
    spin->setSuffix(" m/s");
    grid->addWidget(spin, static_cast<int>(i) + 4, 2);
    connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this, i](double v) {
        advanced_staged_[i + 3] = static_cast<float>(v);
        advanced_feedback_->setText("Values staged. Each minimum must be no greater than its maximum.");
        controls();
      });
  }
  layout->addLayout(grid);
  auto buttons = new QHBoxLayout;
  advanced_refresh_ = new QPushButton("Read advanced settings", advanced_dialog_);
  advanced_refresh_->setObjectName("advanced_refresh");
  advanced_starting_ = new QPushButton("Starting values", advanced_dialog_);
  advanced_starting_->setObjectName("advanced_starting");
  advanced_apply_ = new QPushButton("Apply advanced changes", advanced_dialog_);
  advanced_apply_->setObjectName("advanced_apply");
  buttons->addWidget(advanced_refresh_);
  buttons->addWidget(advanced_starting_);
  buttons->addWidget(advanced_apply_);
  layout->addLayout(buttons);
  advanced_feedback_ = new QLabel("Read advanced settings for the selected sensor.", advanced_dialog_);
  advanced_feedback_->setObjectName("advanced_feedback");
  advanced_feedback_->setWordWrap(true);
  advanced_feedback_->setTextFormat(Qt::PlainText);
  layout->addWidget(advanced_feedback_);
  auto close = new QDialogButtonBox(QDialogButtonBox::Close, advanced_dialog_);
  connect(close, &QDialogButtonBox::rejected, advanced_dialog_, &QDialog::hide);
  layout->addWidget(close);
  connect(advanced_refresh_, &QPushButton::clicked, this, [this] {read_advanced();});
  connect(advanced_starting_, &QPushButton::clicked, this, [this] {
      stage_advanced(advanced_initial_);
      advanced_feedback_->setText("Starting values staged. Click Apply to restore them.");
    });
  connect(advanced_apply_, &QPushButton::clicked, this, &Umrr96Config::apply_advanced);
}

void Umrr96Config::stage_advanced(const AdvancedValues & values)
{
  // Keep the original float values even when the editor rounds its display.
  advanced_staged_ = values;
  const QSignalBlocker mode_blocked(prf_mode_), index_blocked(prf_index_);
  prf_mode_->setCurrentIndex(prf_mode_->findData(static_cast<int>(values[0])));
  prf_index_->setCurrentIndex(prf_index_->findData(static_cast<int>(values[1])));
  for (size_t i = 0; i < velocity_.size(); ++i) {
    const QSignalBlocker blocked(velocity_[i]);
    velocity_[i]->setValue(values[i + 3]);
  }
  controls();
}

void Umrr96Config::fail_advanced(const QString & reason)
{
  operation_ = Operation::None;
  advanced_writes_.clear();
  have_advanced_ = false;
  advanced_feedback_->setText(reason);
  controls();
}

void Umrr96Config::read_advanced(AdvancedRead purpose)
{
  if (operation_ != Operation::None) {return;}
  try {
    const auto id = sensor_id();
    advanced_dialog_->setWindowTitle(QString("UMRR-96 advanced tuning · sensor %1").arg(id));
    if (!getter_->service_is_ready()) {throw std::runtime_error("Radar control unavailable.");}
    auto request = std::make_shared<GetMode::Request>();
    request->sensor_id = id;
    request->section_name = "auto_interface_0dim";
    request->params.assign(names.begin(), names.end());
    request->param_types = {3, 3, 3, 0, 0, 0, 0, 0, 0};
    if (purpose != AdvancedRead::Recover) {advanced_error_.clear();}
    operation_ = Operation::AdvancedRead;
    deadline_ = Clock::now() + std::chrono::seconds(5);
    advanced_feedback_->setText(purpose == AdvancedRead::Recover ?
      advanced_error_ + " Reading actual settings…" : "Reading advanced sensor settings…");
    pending_id_ = getter_->async_send_request(request,
      [this, id, purpose](rclcpp::Client<GetMode>::SharedFuture future) {
        operation_ = Operation::None;
        try {
          const auto values = response(future.get()->res, id);
          AdvancedValues actual{};
          for (size_t i = 0; i < names.size(); ++i) {
            const auto v = read_number(values, names[i]);
            if ((i < 3 && (v != std::floor(v) || v < 0 || v > (i == 0 ? 1 : i == 1 ? 2 : 0))) ||
              (i >= 3 && (v < -150 || v > 150)))
            {
              throw std::runtime_error("Unsupported value returned for " + names[i]);
            }
            actual[i] = i < 3 ? v : static_cast<float>(v);
          }
          bool valid_windows = true;
          for (size_t i = 3; i < names.size(); i += 2) {valid_windows &= actual[i] <= actual[i + 1];}
          const bool changed = actual != advanced_actual_;
          advanced_actual_ = actual;
          have_advanced_ = true;
          if (!have_advanced_initial_ && valid_windows) {advanced_initial_ = actual; have_advanced_initial_ = true;}
          for (size_t i = 0; i < names.size(); ++i) {
            advanced_actual_labels_[i]->setText(i == 0 ? (actual[0] ? "Manual" : "Automatic") :
              QString::number(actual[i], 'g', 8) + (i >= 3 ? " m/s" : ""));
          }
          if (purpose == AdvancedRead::BeforeWrite && !changed) {
            write_advanced_next();
            return;
          }
          stage_advanced(actual);
          if (purpose == AdvancedRead::BeforeWrite) {
            advanced_writes_.clear();
            advanced_feedback_->setText("Sensor settings changed since the last read. Showing actual values; no changes sent.");
          } else if (purpose == AdvancedRead::Recover) {
            advanced_feedback_->setText(advanced_error_ + " Actual settings read; changes may be partial. "
              "Starting values can be staged to restore them.");
          } else if (purpose == AdvancedRead::Verify) {
            advanced_feedback_->setText(actual == advanced_expected_ ? "Advanced changes applied and verified." :
              "Readback differs from the requested advanced settings. Showing actual values.");
          } else {
            advanced_feedback_->setText("Advanced settings read from sensor.");
          }
          if (!valid_windows) {
            advanced_feedback_->setText(advanced_feedback_->text() +
              " Sensor has an inverted velocity window; stage valid bounds to repair it.");
          }
          controls();
        } catch (const std::exception & error) {
          fail_advanced((purpose == AdvancedRead::Recover ? advanced_error_ + " " : QString{}) +
            QString::fromUtf8(error.what()));
        }
      }).request_id;
    controls();
  } catch (const std::exception & error) {
    fail_advanced((purpose == AdvancedRead::Recover ? advanced_error_ + " " : QString{}) +
      QString::fromUtf8(error.what()));
  }
}

void Umrr96Config::apply_advanced()
{
  if (!have_advanced_ || operation_ != Operation::None) {return;}
  advanced_expected_ = advanced_staged_;
  for (size_t i = 3; i < names.size(); i += 2) {
    if (advanced_expected_[i] > advanced_expected_[i + 1]) {return;}
  }
  advanced_writes_.clear();
  auto add = [this](std::initializer_list<size_t> indices, const AdvancedValues & values) {
      auto request = std::make_shared<SetMode::Request>();
      request->sensor_id = sensor_id();
      request->section_name = "auto_interface_0dim";
      for (const auto i : indices) {
        request->params.push_back(names[i]);
        request->values.push_back(QString::number(values[i], 'g',
          std::numeric_limits<float>::max_digits10).toStdString());
        request->value_types.push_back(i < 3 ? 3 : 0);
      }
      advanced_writes_.push_back(request);
    };
  try {
    // Apply an index while automatic selection is active, then set the desired
    // selector. Do not combine PRF ordering assumptions into a sensor batch.
    auto current = advanced_actual_;
    if (current[1] != advanced_expected_[1]) {
      if (current[0] != 0) {current[0] = 0; add({0}, current);}
      add({1}, advanced_expected_);
    }
    if (current[0] != advanced_expected_[0]) {add({0}, advanced_expected_);}
    for (size_t i = 3; i < names.size(); i += 2) {
      if (current[i] != advanced_expected_[i] || current[i + 1] != advanced_expected_[i + 1]) {
        // Always send both bounds. The service validates the intended window;
        // the radar still does not promise an atomic multi-parameter write.
        if (advanced_expected_[i] > current[i + 1]) {add({i + 1, i}, advanced_expected_);}
        else {add({i, i + 1}, advanced_expected_);}
      }
    }
    if (advanced_writes_.empty()) {return;}
    read_advanced(AdvancedRead::BeforeWrite);
  } catch (const std::exception & error) {fail_advanced(QString::fromUtf8(error.what()));}
}

void Umrr96Config::write_advanced_next()
{
  if (advanced_writes_.empty()) {read_advanced(AdvancedRead::Verify); return;}
  if (!setter_->service_is_ready()) {recover_advanced("Tuning service unavailable."); return;}
  const auto request = advanced_writes_.front();
  advanced_writes_.pop_front();
  operation_ = Operation::AdvancedWrite;
  deadline_ = Clock::now() + std::chrono::seconds(5);
  advanced_feedback_->setText("Applying advanced changes…");
  pending_id_ = setter_->async_send_request(request,
    [this, request](rclcpp::Client<SetMode>::SharedFuture future) {
      operation_ = Operation::None;
      try {
        const auto values = response(future.get()->res, request->sensor_id);
        for (const auto & name : request->params) {read_number(values, name);}
        write_advanced_next();
      } catch (const std::exception & error) {recover_advanced(QString::fromUtf8(error.what()));}
    }).request_id;
  controls();
}

void Umrr96Config::recover_advanced(const QString & reason)
{
  advanced_writes_.clear();
  advanced_error_ = reason;
  read_advanced(AdvancedRead::Recover);
}
}  // namespace smart_rviz_plugin
