// SPDX-License-Identifier: Apache-2.0
#include "smart_rviz_plugin/smart_services.hpp"

#include <QJsonParseError>
#include <QProcessEnvironment>
#if __has_include(<ament_index_cpp/get_package_share_path.hpp>)
#include <ament_index_cpp/get_package_share_path.hpp>
#define SMART_SHARE_DIR(pkg) ament_index_cpp::get_package_share_path(pkg).string()
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#define SMART_SHARE_DIR(pkg) ament_index_cpp::get_package_share_directory(pkg)
#endif
#include <array>
#include <chrono>
#include <memory>
#include <string>

#include "panel_util.hpp"

namespace smart_rviz_plugin
{
namespace
{
struct SensorInterface
{
  const char * label;
  const char * directory;
  const char * param;
  const char * command;
  const char * status;
};

// Index 0 of the combo box is the "Choose Sensor Type" placeholder.
const std::array<SensorInterface, 6> sensor_interfaces = {{
  {"UMRR9F MSE", "UserInterfaceUmrr9f_t169_mseV1.0.0",
    "params/auto_interface_0dim.param", "command/auto_interface.command",
    "status/auto_interface.status"},
  {"UMRR9F", "UserInterfaceUmrr9f_t169_automotiveV2.4.1",
    "params/auto_interface_0dim.param", "command/auto_interface.command",
    "status/auto_interface.status"},
  {"UMRR9D", "UserInterfaceUmrr9d_t152_automotiveV1.4.1",
    "params/auto_interface_0dim.param", "command/auto_interface.command",
    "status/auto_interface.status"},
  {"UMRRA4", "UserInterfaceUmrra4_automotiveV1.2.1",
    "params/auto_interface_0dim.param", "command/auto_interface.command",
    "status/auto_interface.status"},
  {"UMRRA4 MSE", "UserInterfaceUmrra4_mseV1.0.0",
    "params/auto_interface_0dim.param", "command/auto_interface.command",
    "status/auto_interface.status"},
  {"UMRRA1", "user_interface_umrra1_t166_b_automotive_v2_0_0",
    "params/auto_interface_rrm.param", "command/auto_interface_rrm.command",
    "status/auto_interface_rrm.status"},
}};

QString html_escape(const QString & text) {return text.toHtmlEscaped();}
}  // namespace

SmartRadarService::SmartRadarService(QWidget * parent) : rviz_common::Panel(parent)
{
  initialize();
}

SmartRadarService::~SmartRadarService()
{
  if (spin_timer_) {spin_timer_->stop();}
  cancel_pending();
  executor_.remove_node(client_node);
}

void SmartRadarService::initialize()
{
  // Initialize ROS 2
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  setup_ros_clients();
  create_widgets();
  setup_layout();
  setup_connections();

  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, this, &SmartRadarService::tick);
  spin_timer_->start(50);
}

void SmartRadarService::setup_ros_clients()
{
  client_node = std::make_shared<rclcpp::Node>(
    panel_util::unique_node_name("smart_service_gui"),
    rclcpp::NodeOptions().use_global_arguments(false));
  mode_client = client_node->create_client<umrr_ros2_msgs::srv::SetMode>("smart_radar/set_radar_mode");
  command_client = client_node->create_client<umrr_ros2_msgs::srv::SendCommand>("smart_radar/send_command");
  status_client = client_node->create_client<umrr_ros2_msgs::srv::GetStatus>("/smart_radar/get_radar_status");
  get_param_client = client_node->create_client<umrr_ros2_msgs::srv::GetMode>("smart_radar/get_radar_mode");
  executor_.add_node(client_node);
}

void SmartRadarService::tick()
{
  if (!rclcpp::ok()) {return;}
  executor_.spin_some(std::chrono::milliseconds(2));
  if (cancel_request_ && std::chrono::steady_clock::now() > deadline_) {
    cancel_pending();
    report_error("Service request timed out; the sensor state is unknown. Check before retrying.");
  }
}

void SmartRadarService::cancel_pending()
{
  if (cancel_request_) {
    cancel_request_();
    cancel_request_ = nullptr;
  }
  set_busy(false);
}

void SmartRadarService::set_busy(bool busy)
{
  for (auto * button : {send_param_button, send_command_button, send_status_button}) {
    if (button) {button->setEnabled(!busy);}
  }
}

void SmartRadarService::report_error(const QString & message)
{
  RCLCPP_ERROR(client_node->get_logger(), "%s", message.toStdString().c_str());
  response_text_edit->append("<font color=\"red\">" + html_escape(message) + "</font>");
}

template<typename ServiceT>
void SmartRadarService::send_request(
  const typename rclcpp::Client<ServiceT>::SharedPtr & client,
  typename ServiceT::Request::SharedPtr request, const QString & service_name)
{
  if (cancel_request_) {
    report_error("A request is still pending; wait for its reply.");
    return;
  }
  if (!client->service_is_ready()) {
    report_error(service_name + " service not available. Is the radar node running?");
    return;
  }
  // The reply is delivered by executor_ inside tick(), i.e. on the GUI thread.
  const auto id = client->async_send_request(request,
    [this](typename rclcpp::Client<ServiceT>::SharedFuture future) {
      cancel_request_ = nullptr;
      set_busy(false);
      try {
        response_text_edit->append(html_escape(QString::fromStdString(future.get()->res)));
      } catch (const std::exception & error) {
        report_error(QString("Service call failed: ") + error.what());
      }
    }).request_id;
  cancel_request_ = [client, id] {client->remove_pending_request(id);};
  deadline_ = std::chrono::steady_clock::now() +
    panel_util::request_timeout(this, REQUEST_TIMEOUT);
  set_busy(true);
}

void SmartRadarService::create_widgets()
{
  // Parameter tab widget
  param_name_line_edit = new QLineEdit(this);
  param_name_line_edit->setObjectName("param_name");
  param_name_line_edit->setPlaceholderText("Enter parameter name");

  param_value_line_edit = new QLineEdit(this);
  param_value_line_edit->setObjectName("param_value");
  param_value_line_edit->setPlaceholderText("Enter value");

  param_sensor_id = new QLineEdit(this);
  param_sensor_id->setObjectName("param_sensor_id");
  param_sensor_id->setPlaceholderText("Sensor ID (decimal or 0x hex)");

  param_section_name = new QLineEdit(this);
  param_section_name->setPlaceholderText("Enter param section name");

  param_action_combo = new QComboBox(this);
  param_action_combo->setObjectName("param_action");
  param_action_combo->addItem("Write Parameter");
  param_action_combo->addItem("Read Parameter");
  
  param_value_type = new QComboBox(this);
  param_value_type->setObjectName("param_value_type");
  param_value_type->addItem("float32, (0)");
  param_value_type->addItem("uint32, (1)");
  param_value_type->addItem("uint16, (2)");
  param_value_type->addItem("uint8, (3)");

  send_param_button = new QPushButton("Send Parameter", this);
  send_param_button->setObjectName("send_param");
  param_table_widget = new QTableWidget(this);
  param_table_widget->setObjectName("param_table");
  param_table_widget->setSelectionBehavior(QAbstractItemView::SelectRows);
  param_table_widget->setSelectionMode(QAbstractItemView::SingleSelection);

  // Command tab widgets
  command_name_line_edit = new QLineEdit(this);
  command_name_line_edit->setObjectName("command_name");
  command_name_line_edit->setPlaceholderText("Enter command name");

  command_value_line_edit = new QLineEdit(this);
  command_value_line_edit->setObjectName("command_value");
  command_value_line_edit->setPlaceholderText("Enter command value (number)");
  
  command_sensor_id = new QLineEdit(this);
  command_sensor_id->setObjectName("command_sensor_id");
  command_sensor_id->setPlaceholderText("Sensor ID (decimal or 0x hex)");

  command_section_name = new QLineEdit(this);
  command_section_name->setPlaceholderText("Enter command section name");
  
  send_command_button = new QPushButton("Send Command", this);
  send_command_button->setObjectName("send_command");
  command_table_widget = new QTableWidget(this);
  command_table_widget->setObjectName("command_table");
  command_table_widget->setSelectionBehavior(QAbstractItemView::SelectRows);
  command_table_widget->setSelectionMode(QAbstractItemView::SingleSelection);

  // Status tab widget
  status_name_line_edit = new QLineEdit(this);
  status_name_line_edit->setObjectName("status_name");
  status_name_line_edit->setPlaceholderText("Enter status name");
  
  status_sensor_id = new QLineEdit(this);
  status_sensor_id->setObjectName("status_sensor_id");
  status_sensor_id->setPlaceholderText("Sensor ID (decimal or 0x hex)");

  status_section_name = new QLineEdit(this);
  status_section_name->setPlaceholderText("Enter status section name");

  status_value_type = new QComboBox(this);
  status_value_type->addItem("uint32, (0)");
  status_value_type->addItem("uint16, (1)");
  status_value_type->addItem("uint8, (2)");
  status_value_type->addItem("int32, (3)");

  send_status_button = new QPushButton("Get Status", this);
  send_status_button->setObjectName("send_status");
  status_table_widget = new QTableWidget(this);
  status_table_widget->setObjectName("status_table");
  status_table_widget->setSelectionBehavior(QAbstractItemView::SelectRows);
  status_table_widget->setSelectionMode(QAbstractItemView::SingleSelection);
  
  // Create tabs
  tab_widget = new QTabWidget(this);
  param_tab = new QWidget(tab_widget);
  command_tab = new QWidget(tab_widget);
  status_tab = new QWidget(tab_widget);
  
  // Add dropdown menu for file selection
  file_selector_combo_box = new QComboBox(this);
  file_selector_combo_box->setObjectName("sensor_type");
  populate_file_menu();
  
  response_text_edit = new QTextEdit(this);
  response_text_edit->setObjectName("response");
  response_text_edit->setReadOnly(true);
  response_text_edit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  response_text_edit->setFixedHeight(100);

}

void SmartRadarService::setup_layout()
{
  // Parameter tab layout
  QVBoxLayout * param_layout = new QVBoxLayout(param_tab);
  param_layout->setSpacing(10);
  param_layout->setContentsMargins(10, 10, 10, 10);

  QFormLayout * param_form = new QFormLayout();
  param_form->addRow("Action:", param_action_combo);
  param_form->addRow("Parameter Name:", param_name_line_edit);
  param_form->addRow("Section Name:", param_section_name); 
  param_form->addRow("Value Type:", param_value_type);
  param_form->addRow("Value:", param_value_line_edit);
  param_form->addRow("Sensor ID:", param_sensor_id);

  param_layout->addWidget(param_table_widget);
  param_layout->addLayout(param_form);
  param_layout->addWidget(send_param_button);
  
  // Command tab widget
  QVBoxLayout * command_layout = new QVBoxLayout(command_tab);
  command_layout->setSpacing(10);
  command_layout->setContentsMargins(10, 10, 10, 10);

  QFormLayout * command_form = new QFormLayout();
  command_form->addRow("Command Name:", command_name_line_edit);
  command_form->addRow("Section Name:", command_section_name);
  command_form->addRow("Value:", command_value_line_edit);
  command_form->addRow("Sensor ID:", command_sensor_id);

  command_layout->addWidget(command_table_widget);
  command_layout->addLayout(command_form);
  command_layout->addWidget(send_command_button);

  // Status tab layout
  QVBoxLayout * status_layout = new QVBoxLayout(status_tab);
  status_layout->setSpacing(10);
  status_layout->setContentsMargins(10, 10, 10, 10);

  QFormLayout * status_form = new QFormLayout();
  status_form->addRow("Status Name:", status_name_line_edit);
  status_form->addRow("Section Name:", status_section_name);
  status_form->addRow("Value Type:", status_value_type);
  status_form->addRow("Sensor ID:", status_sensor_id);

  status_layout->addWidget(status_table_widget);
  status_layout->addLayout(status_form);
  status_layout->addWidget(send_status_button);

  // Add tabs to tab widget
  tab_widget->addTab(param_tab, "Parameter");
  tab_widget->addTab(command_tab, "Command");
  tab_widget->addTab(status_tab, "Status");

  // Main layout
  QVBoxLayout * main_layout = new QVBoxLayout(this);
  main_layout->setSpacing(10);
  main_layout->setContentsMargins(10, 10, 10, 10);

  main_layout->addWidget(file_selector_combo_box);
  main_layout->addWidget(tab_widget);
  main_layout->addWidget(response_text_edit);
}

void SmartRadarService::setup_connections()
{
  // Connect dropdown menu signal to slot
  connect(file_selector_combo_box, QOverload<int>::of(&QComboBox::activated), this, &SmartRadarService::on_file_selected);
  connect(send_param_button, &QPushButton::clicked, this, &SmartRadarService::on_send_param);
  connect(send_command_button, &QPushButton::clicked, this, &SmartRadarService::on_send_command);
  connect(send_status_button, &QPushButton::clicked, this, &SmartRadarService::on_get_status);
  connect(param_table_widget, &QTableWidget::itemSelectionChanged, this, &SmartRadarService::on_param_selection);
  connect(command_table_widget, &QTableWidget::itemSelectionChanged, this, &SmartRadarService::on_command_selection);
  connect(status_table_widget, &QTableWidget::itemSelectionChanged, this, &SmartRadarService::on_status_selection);
  connect(param_action_combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
    [this](int index) {
      // Disable value input when "Read Parameter"
      param_value_line_edit->setEnabled(index == 0);
    });
}

void SmartRadarService::populate_file_menu()
{
  // Add file names to the dropdown menu
  file_selector_combo_box->clear();
  file_selector_combo_box->addItem("Choose Sensor Type");
  for (const auto & sensor : sensor_interfaces) {
    file_selector_combo_box->addItem(sensor.label);
  }
}

void SmartRadarService::on_param_selection()
{
  QList<QTableWidgetItem*> selected = param_table_widget->selectedItems();
  if (!selected.isEmpty()) {
    int row = selected[0]->row();
    param_name_line_edit->setText(param_table_widget->item(row, 1)->text());
    param_section_name->setText(param_table_widget->item(row, 0)->text());

    QTableWidgetItem * type_item = param_table_widget->item(row, 3);
    if (type_item) {
      const QString type = type_item->text().trimmed().toLower();
      if (type == "f32" || type == "float" || type == "float32") {
        param_value_type->setCurrentIndex(0);
      } else if (type == "u32" || type == "uint32") {
        param_value_type->setCurrentIndex(1);
      } else if (type == "u16" || type == "uint16") {
        param_value_type->setCurrentIndex(2);
      } else if (type == "u8" || type == "uint8") {
        param_value_type->setCurrentIndex(3);
      }
    }
  }
}

void SmartRadarService::on_command_selection()
{
  QList<QTableWidgetItem*> selected = command_table_widget->selectedItems();
  if (!selected.isEmpty()) {
    int row = selected[0]->row();
    command_name_line_edit->setText(command_table_widget->item(row, 1)->text());
    command_section_name->setText(command_table_widget->item(row, 0)->text());
  }
}

void SmartRadarService::on_status_selection()
{
  QList<QTableWidgetItem*> selected = status_table_widget->selectedItems();
  if (!selected.isEmpty()) {
    int row = selected[0]->row();
    status_name_line_edit->setText(status_table_widget->item(row, 1)->text());
    status_section_name->setText(status_table_widget->item(row, 0)->text());

    QTableWidgetItem * type_item = status_table_widget->item(row, 3);
    if (type_item) {
      const QString type = type_item->text().trimmed().toLower();
      if (type == "u32") {
        status_value_type->setCurrentIndex(0);
      } else if (type == "u16") {
        status_value_type->setCurrentIndex(1);
      } else if (type == "u8") {
        status_value_type->setCurrentIndex(2);
      } else if (type == "i32") {
        status_value_type->setCurrentIndex(3);
      }
    }
  }
}

QString SmartRadarService::find_user_interfaces_dir(
  const QString & relative_file, QStringList * tried) const
{
  QStringList roots;
  const auto env = QProcessEnvironment::systemEnvironment().value("SMART_USER_INTERFACES_DIR");
  if (!env.isEmpty()) {roots << env;}
  try {
    roots << QString::fromStdString(SMART_SHARE_DIR("smart_rviz_plugin")) + "/user_interfaces";
  } catch (const std::exception &) {
    // Not installed through ament; fall back to the legacy source-tree layout below.
  }
  roots << QDir::currentPath() + "/src/smartmicro_ros2_radars/umrr_ros2_driver/smartmicro/user_interfaces";
  for (const auto & root : roots) {
    tried->append(root);
    if (QFile::exists(root + "/" + relative_file)) {return root;}
  }
  return {};
}

void SmartRadarService::on_file_selected(int index)
{
  // Clear all tabs; a failed load must not leave another sensor's table visible.
  for (auto * table : {param_table_widget, command_table_widget, status_table_widget}) {
    table->setRowCount(0);
  }
  if (index <= 0 || index > static_cast<int>(sensor_interfaces.size())) {
    for (auto * edit : {param_name_line_edit, param_value_line_edit, param_sensor_id,
        param_section_name, command_name_line_edit, command_value_line_edit, command_sensor_id,
        command_section_name, status_name_line_edit, status_sensor_id, status_section_name})
    {
      edit->clear();
    }
    return;
  }

  const auto & sensor = sensor_interfaces[static_cast<size_t>(index - 1)];
  const QString directory = QString(sensor.directory) + "/instructions/";
  QStringList tried;
  const QString base_path = find_user_interfaces_dir(directory + sensor.param, &tried);
  if (base_path.isEmpty()) {
    report_error(QString("Instruction tables for %1 not found. Searched: %2. Run smart_extract.sh "
      "and rebuild smart_rviz_plugin, or set SMART_USER_INTERFACES_DIR.")
      .arg(sensor.label, tried.join(", ")));
    return;
  }
  param_json_file_path = base_path + "/" + directory + sensor.param;
  command_json_file_path = base_path + "/" + directory + sensor.command;
  status_json_file_path = base_path + "/" + directory + sensor.status;

  read_param_json_data();
  read_command_json_data();
  read_status_json_data();
}

bool SmartRadarService::load_json(const QString & path, const char * array_key, QJsonObject * object)
{
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    report_error("Cannot open " + path + ": " + file.errorString());
    return false;
  }
  QJsonParseError error;
  const auto document = QJsonDocument::fromJson(file.readAll(), &error);
  if (error.error != QJsonParseError::NoError || !document.isObject() ||
    !document.object().value(array_key).isArray())
  {
    report_error("Cannot parse " + path + ": " +
      (error.error != QJsonParseError::NoError ? error.errorString() :
      QString("missing \"%1\" array").arg(array_key)));
    return false;
  }
  *object = document.object();
  return true;
}

namespace
{
void fill_table(
  QTableWidget * table, const QJsonObject & json, const char * array_key,
  const QStringList & headers, const QStringList & fields)
{
  const QString section = json["name"].toString();
  const QJsonArray rows = json[array_key].toArray();
  table->setUpdatesEnabled(false);
  table->setColumnCount(headers.size());
  table->setHorizontalHeaderLabels(headers);
  table->setRowCount(rows.size());
  for (int i = 0; i < rows.size(); ++i) {
    const QJsonObject row = rows[i].toObject();
    table->setItem(i, 0, new QTableWidgetItem(section));
    for (int column = 0; column < fields.size(); ++column) {
      table->setItem(i, column + 1, new QTableWidgetItem(row[fields[column]].toString()));
    }
  }
  table->setUpdatesEnabled(true);
}
}  // namespace

void SmartRadarService::read_command_json_data()
{
  QJsonObject json;
  if (load_json(command_json_file_path, "commands", &json)) {
    fill_table(command_table_widget, json, "commands",
      {"Section", "Name", "Argument", "Comment"}, {"name", "argument", "comment"});
  }
}

void SmartRadarService::read_param_json_data()
{
  QJsonObject json;
  if (load_json(param_json_file_path, "parameters", &json)) {
    fill_table(param_table_widget, json, "parameters",
      {"Section", "Name", "Comment", "Type"}, {"name", "comment", "type"});
  }
}

void SmartRadarService::read_status_json_data()
{
  QJsonObject json;
  if (load_json(status_json_file_path, "status", &json)) {
    fill_table(status_table_widget, json, "status",
      {"Section", "Name", "Comment", "Type"}, {"name", "comment", "type"});
  }
}

void SmartRadarService::on_send_param()
{
  // Validate common inputs
  if (param_name_line_edit->text().trimmed().isEmpty()) {
    report_error("Parameter name must be filled.");
    return;
  }
  const auto sensor_id = panel_util::parse_uint(param_sensor_id->text());
  if (!sensor_id) {
    report_error("Sensor ID must be an unsigned decimal or 0x-prefixed hexadecimal number.");
    return;
  }
  const int type = param_value_type->currentIndex();  // 0 float32, 1 u32, 2 u16, 3 u8

  if (param_action_combo->currentIndex() == 0) {
    // Writing param: validate against the selected type and send a canonical decimal string.
    QString value;
    if (type == 0) {
      const auto parsed = panel_util::parse_float(param_value_line_edit->text());
      if (parsed) {value = QString::number(*parsed, 'g', 9);}
    } else {
      static constexpr std::array<uint32_t, 4> limits = {0, 0xFFFFFFFFu, 0xFFFFu, 0xFFu};
      const auto parsed = panel_util::parse_uint(param_value_line_edit->text(),
        limits[static_cast<size_t>(type)]);
      if (parsed) {value = QString::number(*parsed);}
    }
    if (value.isEmpty()) {
      report_error("Value \"" + param_value_line_edit->text() + "\" is not a valid " +
        param_value_type->currentText().section(',', 0, 0) + ".");
      return;
    }
    auto request = std::make_shared<umrr_ros2_msgs::srv::SetMode::Request>();
    request->section_name = param_section_name->text().toStdString();
    request->params.push_back(param_name_line_edit->text().trimmed().toStdString());
    request->sensor_id = *sensor_id;
    request->value_types.push_back(static_cast<uint8_t>(type));
    request->values.push_back(value.toStdString());
    send_request<umrr_ros2_msgs::srv::SetMode>(mode_client, request, "SetMode");
  } else {
    auto request = std::make_shared<umrr_ros2_msgs::srv::GetMode::Request>();
    request->section_name = param_section_name->text().toStdString();
    request->params.push_back(param_name_line_edit->text().trimmed().toStdString());
    request->sensor_id = *sensor_id;
    request->param_types.push_back(static_cast<uint8_t>(type));
    send_request<umrr_ros2_msgs::srv::GetMode>(get_param_client, request, "GetMode");
  }
}

void SmartRadarService::on_send_command()
{
  if (command_name_line_edit->text().trimmed().isEmpty()) {
    report_error("Command name must be filled.");
    return;
  }
  const auto sensor_id = panel_util::parse_uint(command_sensor_id->text());
  if (!sensor_id) {
    report_error("Sensor ID must be an unsigned decimal or 0x-prefixed hexadecimal number.");
    return;
  }
  // SendCommand.value is float32; fractional values are sent unchanged.
  const auto value = panel_util::parse_float(command_value_line_edit->text());
  if (!value) {
    report_error("Command value must be a finite number (enter 0 if the command takes none).");
    return;
  }

  auto request = std::make_shared<umrr_ros2_msgs::srv::SendCommand::Request>();
  request->section_name = command_section_name->text().toStdString();
  request->command = command_name_line_edit->text().trimmed().toStdString();
  request->value = *value;
  request->sensor_id = *sensor_id;
  send_request<umrr_ros2_msgs::srv::SendCommand>(command_client, request, "SendCommand");
}

void SmartRadarService::on_get_status()
{
  if (status_name_line_edit->text().trimmed().isEmpty()) {
    report_error("Status name must be filled.");
    return;
  }
  const auto sensor_id = panel_util::parse_uint(status_sensor_id->text());
  if (!sensor_id) {
    report_error("Sensor ID must be an unsigned decimal or 0x-prefixed hexadecimal number.");
    return;
  }

  auto request = std::make_shared<umrr_ros2_msgs::srv::GetStatus::Request>();
  request->section_name = status_section_name->text().toStdString();
  request->statuses.push_back(status_name_line_edit->text().trimmed().toStdString());
  request->sensor_id = *sensor_id;
  request->status_types.push_back(static_cast<uint8_t>(status_value_type->currentIndex()));
  send_request<umrr_ros2_msgs::srv::GetStatus>(status_client, request, "GetStatus");
}

}  // namespace smart_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(smart_rviz_plugin::SmartRadarService, rviz_common::Panel)
