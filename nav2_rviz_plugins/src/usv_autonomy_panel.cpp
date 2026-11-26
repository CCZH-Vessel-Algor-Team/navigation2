// Copyright 2026 USV team. Licensed under Apache-2.0.
#include "nav2_rviz_plugins/usv_autonomy_panel.hpp"
#include <cmath>
#include <string>
#include <utility>
#include <QFormLayout>
#include <QFontDatabase>
#include <QGridLayout>
#include <QDockWidget>
#include <QMainWindow>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QRegularExpression>
#include <QScrollArea>
#include <QStringList>
#include <QUuid>
#include <QVBoxLayout>
#include "pluginlib/class_list_macros.hpp"

namespace nav2_rviz_plugins
{
namespace
{
const std::array<std::string, 4> kTests = {
  "authority", "maritime_wait", "emergency", "localization_failure"};
QString uniqueId(const QString & prefix)
{
  return prefix + "-" + QUuid::createUuid().toString(QUuid::Id128);
}
}  // namespace

QPushButton * UsvAutonomyPanel::button(const QString & text, const QString & name)
{
  auto widget = new QPushButton(text);
  widget->setObjectName(name);
  return widget;
}

QDoubleSpinBox * UsvAutonomyPanel::number(const QString & name, double value, double low, double high)
{
  auto widget = new QDoubleSpinBox;
  widget->setObjectName(name);
  widget->setRange(low, high);
  widget->setDecimals(2);
  widget->setValue(value);
  return widget;
}

UsvAutonomyPanel::UsvAutonomyPanel(QWidget * parent) : Panel(parent)
{
  // Container images without a CJK font render Chinese labels as empty boxes.
  // Prefer an installed CJK family locally; preserve RViz's font size and DPI.
  const auto families = QFontDatabase().families();
  for (const auto & family : QStringList{"Noto Sans CJK SC", "WenQuanYi Micro Hei"}) {
    if (families.contains(family)) {
      auto readable = font();
      readable.setFamily(family);
      setFont(readable);
      break;
    }
  }
  // Construct controls once, then place infrequent controls in separate tabs.
  // No change to the original RViz dock geometry is required.
  auto contents = new QWidget;
  auto columns = new QHBoxLayout(contents);
  columns->setSpacing(6);
  auto left = new QVBoxLayout;
  auto right = new QVBoxLayout;
  left->setSpacing(4);
  right->setSpacing(4);
  columns->addLayout(left);
  columns->addLayout(right);

  auto connection = new QHBoxLayout;
  namespace_edit_ = new QLineEdit(namespace_);
  namespace_edit_->setObjectName("vessel_namespace");
  namespace_edit_->setMaximumWidth(120);
  auto apply = button("连接", "connect_vessel");
  connection->addWidget(namespace_edit_);
  connection->addWidget(apply);
  left->addLayout(connection);
  connect(apply, &QPushButton::clicked, this, &UsvAutonomyPanel::bind);

  state_label_ = new QLabel("等待任务状态");
  state_label_->setObjectName("autonomy_status");
  state_label_->setWordWrap(true);
  state_label_->setTextFormat(Qt::PlainText);
  state_label_->setMinimumWidth(180);
  gate_label_ = new QLabel("输出门控：未收到状态");
  gate_label_->setObjectName("gate_status");
  gate_label_->setWordWrap(true);
  gate_label_->setTextFormat(Qt::PlainText);
  frame_label_ = new QLabel;
  frame_label_->setWordWrap(true);
  left->addWidget(state_label_);
  left->addWidget(gate_label_);
  maritime_label_ = new QLabel("海事：等待规则状态");
  maritime_label_->setObjectName("maritime_status");
  maritime_label_->setWordWrap(true);
  maritime_label_->setTextFormat(Qt::PlainText);
  left->addWidget(maritime_label_);
  left->addWidget(frame_label_);

  auto navigation = new QGroupBox("航点 / 米、°、m/s");
  auto navigation_layout = new QVBoxLayout(navigation);
  navigation_layout->setSpacing(3);
  auto inputs = new QGridLayout;
  inputs->setSpacing(2);
  x_ = number("goal_x", 0., -1.e7, 1.e7);
  y_ = number("goal_y", 0., -1.e7, 1.e7);
  yaw_ = number("goal_yaw", 0., -180., 180.);
  speed_ = number("goal_speed", .8, 0., 10.);
  x_->setPrefix("X "); y_->setPrefix("Y "); yaw_->setPrefix("航向° "); speed_->setPrefix("m/s ");
  inputs->addWidget(x_, 0, 0); inputs->addWidget(y_, 0, 1);
  inputs->addWidget(yaw_, 1, 0); inputs->addWidget(speed_, 1, 1);
  navigation_layout->addLayout(inputs);
  auto add = button("添加输入航点", "add_waypoint");
  navigation_layout->addWidget(add);
  connect(add, &QPushButton::clicked, this, [this]() {
      addWaypoint(x_->value(), y_->value(), yaw_->value(), speed_->value());
    });
  collect_ = new QCheckBox("用 USV Test Goal 取点");
  collect_->setChecked(true);
  collect_->setObjectName("collect_goals");
  navigation_layout->addWidget(collect_);
  goals_ = new QTableWidget(0, 4);
  goals_->setObjectName("waypoints");
  goals_->setHorizontalHeaderLabels({"X", "Y", "航向°", "速度"});
  goals_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  goals_->verticalHeader()->setVisible(false);
  goals_->setMinimumHeight(70);
  goals_->setMaximumHeight(110);
  navigation_layout->addWidget(goals_);
  auto clear = button("清空航点", "clear_waypoints");
  connect(clear, &QPushButton::clicked, this, [this]() {goals_->setRowCount(0);});
  navigation_layout->addWidget(clear);
  task_id_ = new QLineEdit;
  task_id_->setObjectName("new_task_id");
  task_id_->setPlaceholderText("新任务 ID（留空自动生成）");
  navigation_layout->addWidget(task_id_);
  auto options = new QHBoxLayout;
  policy_ = new QComboBox;
  policy_->setObjectName("waypoint_policy");
  policy_->addItems({"REQUIRED", "VIA"});
  loop_ = new QCheckBox("循环");
  loop_->setObjectName("loop_execution");
  options->addWidget(policy_); options->addWidget(loop_);
  navigation_layout->addLayout(options);
  left->addWidget(navigation);
  left->addStretch();

  auto add_management = [this, right](const QString & text, const QString & name,
      const std::string & command, const std::string & mode = "") {
      auto widget = button(text, name);
      management_buttons_.push_back(widget);
      right->addWidget(widget);
      connect(widget, &QPushButton::clicked, this, [this, command, mode]() {manage(command, mode);});
    };
  add_management("新建导航任务", "start_navigation", "start", "NAVIGATION");
  add_management("NAVIGATION", "switch_navigation", "resume_navigation", "NAVIGATION");
  add_management("暂停", "pause_task", "pause");
  add_management("继续 / 交回确认", "resume_task", "resume");
  add_management("结束（终态）", "stop_task", "stop");

  auto hold = new QGroupBox("HOLD 参数");
  hold->setToolTip("内半径、外半径（m）、返航速度（m/s）、持续秒数（0=不限时）");
  auto hold_layout = new QVBoxLayout(hold);
  hold_layout->setSpacing(2);
  hold_anchor_source_ = new QComboBox;
  hold_anchor_source_->setObjectName("hold_anchor_source");
  hold_anchor_source_->addItem("当前位置（每次切入重新取点）");
  hold_anchor_source_->addItem("手动输入锚点（地图坐标）");
  hold_anchor_source_->setToolTip("当前位置在切入 HOLD 时保存；点击继续不会再次取点。手动坐标单位为米。");
  hold_layout->addWidget(hold_anchor_source_);
  auto anchor_row = new QHBoxLayout;
  hold_anchor_x_ = number("hold_anchor_x", 0., -1.e7, 1.e7);
  hold_anchor_y_ = number("hold_anchor_y", 0., -1.e7, 1.e7);
  hold_anchor_x_->setPrefix("锚点 X m "); hold_anchor_y_->setPrefix("锚点 Y m ");
  anchor_row->addWidget(hold_anchor_x_); anchor_row->addWidget(hold_anchor_y_);
  hold_layout->addLayout(anchor_row);
  auto anchor_inputs = [this]() {
      const bool manual = hold_anchor_source_->currentIndex() == 1;
      hold_anchor_x_->setEnabled(manual); hold_anchor_y_->setEnabled(manual);
    };
  connect(hold_anchor_source_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, [anchor_inputs](int) {anchor_inputs();});
  anchor_inputs();
  auto radius_row = new QHBoxLayout;
  inner_ = number("hold_inner", 3., .1, 10000.);
  outer_ = number("hold_outer", 10., .2, 20000.);
  hold_speed_ = number("hold_speed", .5, .01, 10.);
  hold_duration_ = number("hold_duration", 0., 0., 86400.);
  for (auto widget : {inner_, outer_, hold_speed_, hold_duration_}) {widget->setMaximumWidth(80);}
  radius_row->addWidget(inner_); radius_row->addWidget(outer_);
  radius_row->addWidget(hold_speed_); radius_row->addWidget(hold_duration_);
  hold_layout->addLayout(radius_row);
  right->addWidget(hold);
  add_management("HOLD", "switch_hold", "switch", "HOLD");
  add_management("应用锚点并进入 HOLD", "apply_hold_anchor", "switch", "HOLD");
  add_management("测绘", "switch_survey", "switch", "SURVEY");
  suspended_ = new QComboBox;
  suspended_->setObjectName("suspended_task_id");
  suspended_->setEditable(true);
  suspended_->setToolTip("选择挂起任务，或输入重启前保存的任务 ID");
  right->addWidget(suspended_);
  add_management("接续暂存任务", "resume_stored", "resume_task");
  add_management("DOCK", "switch_dock", "switch", "DOCK");
  add_management("UNDOCK", "switch_undock", "switch", "UNDOCK");
  add_management("复位交接故障", "reset_fault", "reset_fault");
  reset_button_ = button("复位安全保护（急停/海事，不等于继续）", "reset_safety");
  reset_button_->setToolTip("海事目标丢失接管后：确认风险已处理且实际进入人工模式，再复位；复位不直接恢复推进。");
  right->addWidget(reset_button_);
  connect(reset_button_, &QPushButton::clicked, this, &UsvAutonomyPanel::resetSafety);

  auto test_group = new QGroupBox("CCS 仿真功能测试");
  auto tests_layout = new QVBoxLayout(test_group);
  tests_layout->setSpacing(2);
  test_source_label_ = new QLabel;
  test_source_label_->setObjectName("test_authority_source");
  test_source_label_->setWordWrap(true);
  tests_layout->addWidget(test_source_label_);
  const std::array<QString, 8> labels = {"仿真自主权", "仿真人工接管", "触发海事等待", "解除测试等待",
    "触发急停输入", "解除急停输入", "仿真定位失效", "恢复仿真定位"};
  for (size_t i = 0; i < kTests.size(); ++i) {
    auto row = new QHBoxLayout;
    row->setSpacing(2);
    for (size_t j = 0; j < 2; ++j) {
      auto widget = button(labels[i * 2 + j], "test_" + QString::fromStdString(kTests[i]) +
          (j == 0 ? "_on" : "_off"));
      test_buttons_[i * 2 + j] = widget;
      connect(widget, &QPushButton::clicked, this, [this, i, j]() {test(i, j == 0);});
      row->addWidget(widget);
    }
    tests_layout->addLayout(row);
  }
  right->addWidget(test_group);
  response_ = new QLabel("202=交接已接收，完成以状态为准。");
  response_->setObjectName("management_response");
  response_->setWordWrap(true);
  response_->setTextFormat(Qt::PlainText);
  right->addWidget(response_);
  right->addStretch();

  tabs_ = new QTabWidget;
  tabs_->setObjectName("task_tabs");
  tabs_->setUsesScrollButtons(true);
  auto common = new QWidget;
  auto common_layout = new QGridLayout(common);
  auto management = [this](const QString & name) {
      for (auto widget : management_buttons_) {if (widget->objectName() == name) {return widget;}}
      return static_cast<QPushButton *>(nullptr);
    };
  auto modes = new QGroupBox("任务模式");
  modes->setObjectName("mode_switches");
  auto mode_buttons = new QGridLayout(modes);
  mode_buttons->setContentsMargins(4, 4, 4, 4);
  mode_buttons->setSpacing(3);
  const std::array<QString, 5> mode_names = {
    "switch_navigation", "switch_hold", "switch_survey", "switch_dock", "switch_undock"};
  for (size_t i = 0; i < mode_names.size(); ++i) {
    auto widget = management(mode_names[i]);
    widget->setCheckable(true);
    mode_buttons->addWidget(widget, 0, static_cast<int>(i));
  }
  management("switch_navigation")->setToolTip("切回暂存导航任务并暂停；点击继续后执行剩余航点，保留进度与预算");
  management("switch_hold")->setToolTip("按模式页选择手动锚点或切入时当前位置；切回已有 HOLD 更新锚点并暂停，点击继续后执行");
  management("switch_survey")->setToolTip("进入测绘循线：驶向固定测线起点后沿参考路径执行，失败/等待超时切入人工接管");
  management("switch_dock")->setToolTip("切换到靠泊模式；后端未实现时返回 501 并保留当前模式");
  management("switch_undock")->setToolTip("切换到离泊模式；后端未实现时返回 501 并保留当前模式");
  common_layout->addWidget(management("pause_task"), 0, 0);
  common_layout->addWidget(management("resume_task"), 0, 1);
  common_layout->addWidget(management("stop_task"), 1, 0, 1, 2);
  common_layout->setContentsMargins(4, 4, 4, 4);
  common_layout->setSpacing(3);
  // Keep the frequent controls visible in the original Navigation 2 dock's
  // small height. An editable stored-task selector below these controls also
  // stole focus and scrolled them away while a request disabled the buttons.
  common_layout->setRowStretch(2, 1);
  auto goal_page = new QWidget;
  auto goal_layout = new QVBoxLayout(goal_page);
  goal_layout->addWidget(navigation);
  goal_layout->addWidget(management("start_navigation"));
  goal_layout->addStretch();
  auto mode_page = new QWidget;
  auto mode_layout = new QVBoxLayout(mode_page);
  mode_layout->addWidget(hold);
  mode_layout->addWidget(management("apply_hold_anchor"));
  for (const auto & name : {"reset_fault"}) {
    mode_layout->addWidget(management(name));
  }
  mode_layout->addWidget(new QLabel("接续任务 ID（可输入）"));
  mode_layout->addWidget(suspended_);
  mode_layout->addWidget(management("resume_stored"));
  mode_layout->addWidget(reset_button_); mode_layout->addStretch();
  diagnostic_label_ = new QLabel;
  diagnostic_label_->setObjectName("task_diagnostics");
  diagnostic_label_->setWordWrap(true);
  diagnostic_label_->setTextFormat(Qt::PlainText);
  diagnostic_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  left->addWidget(diagnostic_label_);
  auto page = [](QWidget * widget) {
      auto scroll = new QScrollArea;
      scroll->setWidgetResizable(true);
      scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
      scroll->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
      scroll->setMinimumSize(0, 0);
      scroll->setWidget(widget);
      return scroll;
    };
  tabs_->addTab(page(common), "常用");
  tabs_->addTab(page(goal_page), "航点");
  tabs_->addTab(page(mode_page), "模式");
  tests_page_ = page(test_group);
  tabs_->addTab(tests_page_, "测试");
  tabs_->addTab(page(contents), "诊断");
  state_label_->setMaximumHeight(50);
  state_label_->setWordWrap(false);
  gate_label_->setMaximumHeight(55);
  response_->setMaximumHeight(70);
  for (auto label : {state_label_, gate_label_, response_}) {
    label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  }
  auto root = new QVBoxLayout(this);
  root->setSizeConstraint(QLayout::SetNoConstraint);
  root->setContentsMargins(4, 4, 4, 4);
  root->addWidget(state_label_); root->addWidget(gate_label_);
  root->addWidget(modes);
  root->addWidget(tabs_, 1); root->addWidget(response_);
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &UsvAutonomyPanel::poll);
  bind();
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "USV BT v2 panel loaded (namespace=%s, frame=%s)",
      namespace_.toStdString().c_str(), frame_.toStdString().c_str());
  }
  timer_->start(100);
}

UsvAutonomyPanel::~UsvAutonomyPanel() {timer_->stop();}

void UsvAutonomyPanel::onInitialize()
{
  // Append a tab next to the original Navigation 2 dock; preserve the original
  // map/camera dock positions and leave the original tab selected initially.
  QTimer::singleShot(250, this, [this]() {
      QWidget * ancestor = parentWidget();
      while (ancestor && !qobject_cast<QDockWidget *>(ancestor)) {ancestor = ancestor->parentWidget();}
      auto dock = qobject_cast<QDockWidget *>(ancestor);
      auto main = dock ? qobject_cast<QMainWindow *>(dock->window()) : nullptr;
      if (!main) {return;}
      for (auto original : main->findChildren<QDockWidget *>()) {
        if (original != dock && (original->objectName() == "Navigation 2" || original->windowTitle() == "Navigation 2")) {
          const auto size = original->size();
          main->tabifyDockWidget(original, dock);
          main->resizeDocks({original}, {size.width()}, Qt::Horizontal);
          main->resizeDocks({original}, {size.height()}, Qt::Vertical);
          if (base_width_ > 0 && base_height_ > 0 && base_width_ < 32768 && base_height_ < 32768) {
            const auto saved = QSize(base_width_, base_height_);
            // Adding a temporary separate dock can grow the main window before
            // tabification. Reapply the saved size after Qt recomputes its dock
            // minimum, rather than leaving that temporary extra height behind.
            QTimer::singleShot(100, main, [main, saved]() {main->resize(saved);});
          }
          original->raise(); break;
        }
      }
    });
}

QString UsvAutonomyPanel::prefix() const {return namespace_.isEmpty() ? "" : "/" + namespace_;}

void UsvAutonomyPanel::bind()
{
  auto value = namespace_edit_->text().trimmed();
  while (value.startsWith('/')) {value.remove(0, 1);}
  while (value.endsWith('/')) {value.chop(1);}
  const QRegularExpression pattern("^[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*$");
  if (!value.isEmpty() && !pattern.match(value).hasMatch()) {
    namespace_edit_->setText(namespace_);
    response_->setText("命名空间无效，仍连接: " + namespace_); return;
  }
  if (namespace_ != value) {goals_->setRowCount(0); suspended_->clear(); task_id_->clear();}
  namespace_ = value;
  namespace_edit_->setText(value);
  have_state_ = false;
  have_gate_ = false;
  have_maritime_ = false;
  pending_ = Pending::NONE;
  auto options = rclcpp::NodeOptions().use_global_arguments(false);
  node_ = std::make_shared<rclcpp::Node>(
    "rviz_usv_panel_" + QUuid::createUuid().toString(QUuid::Id128).left(8).toStdString(), options);
  manager_ = node_->create_client<Manage>((prefix() + "/autonomy/manage").toStdString());
  reset_ = node_->create_client<Reset>((prefix() + "/autonomy/reset_safety").toStdString());
  for (size_t i = 0; i < kTests.size(); ++i) {
    tests_[i] = node_->create_client<Test>((prefix() + "/autonomy/test/").toStdString() + kTests[i]);
  }
  state_sub_ = node_->create_subscription<usv_interfaces::msg::AutonomyState>(
    (prefix() + "/autonomy/state").toStdString(), 1,
    [this](usv_interfaces::msg::AutonomyState::ConstSharedPtr state) {
      if (have_state_ && state_.task_id != state->task_id && !state_.task_id.empty() &&
        state_.user_intent != "END" && suspended_->findText(QString::fromStdString(state_.task_id)) < 0)
      {suspended_->addItem(QString::fromStdString(state_.task_id));}
      state_ = *state;
      have_state_ = true;
      received_ = std::chrono::steady_clock::now();
    });
  gate_sub_ = node_->create_subscription<usv_interfaces::msg::GateState>(
    (prefix() + "/autonomy/gate_state").toStdString(), 1,
    [this](usv_interfaces::msg::GateState::ConstSharedPtr state) {
      have_gate_ = true;
      gate_received_ = std::chrono::steady_clock::now();
      const QStringList intents = {"DISABLED", "NEUTRAL", "VELOCITY_CONTROL"};
      gate_label_->setText("最终输出: " + intents.value(state->intent, "UNKNOWN") + "\n" +
        QString::fromStdString(state->reason));
    });
  maritime_sub_ = node_->create_subscription<usv_interfaces::msg::MaritimeRuleDecision>(
    (prefix() + "/autonomy/maritime").toStdString(), 1,
    [this](usv_interfaces::msg::MaritimeRuleDecision::ConstSharedPtr state) {
      have_maritime_ = true;
      maritime_received_ = std::chrono::steady_clock::now();
      const QString policy = state->handover_required ? "人工接管锁存" :
        (state->intervention_required ? "等待" : (state->stand_on ? "保向观察" : "普通规划"));
      QString target;
      for (const auto byte : state->target_id.uuid) {
        target += QString("%1").arg(static_cast<unsigned int>(byte), 2, 16, QChar('0'));
      }
      maritime_label_->setText("海事：" + policy + (state->valid ? "" : "（观测未知）") +
        "\n目标: " + target + "\n事件: " + QString::fromStdString(state->event_id) +
        "\n" + QString::fromStdString(state->reason));
    });
  goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    (prefix() + "/autonomy/goal_pose").toStdString(), 10,
    [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr pose) {collect(*pose);});
  frame_label_->setText("航点坐标系: " + frame_ + "；工具 Topic: " + prefix() + "/autonomy/goal_pose");
  gate_label_->setText("输出门控：等待当前船舶状态");
  response_->setText("已连接 " + namespace_ + "；此前请求的结果以原船状态为准。");
  render();
}

bool UsvAutonomyPanel::fresh() const
{
  return have_state_ && !state_.boot_id.empty() &&
         std::chrono::steady_clock::now() - received_ < std::chrono::seconds(1);
}

void UsvAutonomyPanel::addWaypoint(double x, double y, double yaw, double speed)
{
  const int row = goals_->rowCount();
  goals_->insertRow(row);
  const std::array<double, 4> values = {x, y, yaw, speed};
  for (int column = 0; column < 4; ++column) {
    goals_->setItem(row, column, new QTableWidgetItem(QString::number(values[column], 'g', 12)));
  }
}

void UsvAutonomyPanel::collect(const geometry_msgs::msg::PoseStamped & pose)
{
  if (!collect_->isChecked()) {return;}
  const auto & p = pose.pose.position;
  const auto & q = pose.pose.orientation;
  if (QString::fromStdString(pose.header.frame_id) != frame_ ||
    !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(q.w) ||
    !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
    std::abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) > .02)
  {response_->setText("取点帧/数值无效，需要 " + frame_); return;}
  const double yaw = std::atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z));
  addWaypoint(p.x, p.y, yaw * 180. / std::acos(-1.), speed_->value());
}

void UsvAutonomyPanel::manage(const std::string & command, const std::string & mode)
{
  if (namespace_edit_->text().trimmed() != namespace_) {
    response_->setText("请先连接所填写的船舶命名空间"); return;
  }
  if (!fresh() || pending_ != Pending::NONE || !manager_->service_is_ready()) {return;}
  auto request = std::make_shared<Manage::Request>();
  request->command = command;
  request->request_id = uniqueId("rviz").toStdString();
  request->expected_boot_id = state_.boot_id;
  request->expected_revision = state_.revision;
  request->task_id = state_.task_id;
  request->mode = mode;
  if (command == "resume_navigation") {request->task_id.clear();}
  else if (!mode.empty()) {
    request->task_id = (task_id_->text().trimmed().isEmpty() ?
      uniqueId(QString::fromStdString(mode).toLower()) : task_id_->text().trimmed()).toStdString();
  }
  if (command == "resume_task") {request->task_id = suspended_->currentText().trimmed().toStdString();}
  if (mode == "NAVIGATION" && command != "resume_navigation") {
    if (goals_->rowCount() == 0) {response_->setText("请先添加航点"); return;}
    for (int row = 0; row < goals_->rowCount(); ++row) {
      std::array<double, 4> values{};
      for (int column = 0; column < 4; ++column) {
        bool ok = false;
        if (goals_->item(row, column)) {values[column] = goals_->item(row, column)->text().toDouble(&ok);}
        if (!ok || !std::isfinite(values[column])) {response_->setText("航点包含无效数值"); return;}
      }
      if (values[3] < 0) {response_->setText("速度必须非负"); return;}
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = frame_.toStdString();
      pose.pose.position.x = values[0]; pose.pose.position.y = values[1];
      const double radians = values[2] * std::acos(-1.) / 180.;
      pose.pose.orientation.z = std::sin(radians / 2.);
      pose.pose.orientation.w = std::cos(radians / 2.);
      request->goals.push_back(pose);
      request->speeds.push_back(values[3]);
    }
    request->waypoint_policy = policy_->currentText().toStdString();
    request->loop_execution = loop_->isChecked();
  }
  if (mode == "HOLD") {
    if (inner_->value() >= outer_->value()) {response_->setText("HOLD 内半径必须小于外半径"); return;}
    request->hold.use_current_pose = hold_anchor_source_->currentIndex() == 0;
    if (!request->hold.use_current_pose) {
      request->hold.anchor.header.frame_id = frame_.toStdString();
      request->hold.anchor.pose.position.x = hold_anchor_x_->value();
      request->hold.anchor.pose.position.y = hold_anchor_y_->value();
      request->hold.anchor.pose.orientation.w = 1.0;
    }
    request->hold.inner_radius = inner_->value(); request->hold.outer_radius = outer_->value();
    request->hold.return_speed = hold_speed_->value(); request->hold.duration_s = hold_duration_->value();
  }
  auto future = manager_->async_send_request(request);
  request_id_ = future.request_id;
  management_future_ = future.share();
  pending_ = Pending::MANAGE; sent_ = std::chrono::steady_clock::now();
  response_->setText("请求已发送: " + QString::fromStdString(request->request_id));
  render();
}

void UsvAutonomyPanel::resetSafety()
{
  if (namespace_edit_->text().trimmed() != namespace_) {return;}
  if (pending_ != Pending::NONE || !reset_->service_is_ready()) {return;}
  auto request = std::make_shared<Reset::Request>(); request->confirmed = true;
  auto future = reset_->async_send_request(request);
  request_id_ = future.request_id; reset_future_ = future.share();
  pending_ = Pending::RESET; sent_ = std::chrono::steady_clock::now(); render();
}

void UsvAutonomyPanel::test(size_t index, bool value)
{
  if (namespace_edit_->text().trimmed() != namespace_) {return;}
  if (!simulation_tests_ || (index == 0 && !simulation_authority_) ||
    pending_ != Pending::NONE || !tests_[index]->service_is_ready()) {return;}
  auto request = std::make_shared<Test::Request>(); request->data = value;
  auto future = tests_[index]->async_send_request(request);
  request_id_ = future.request_id; test_future_ = future.share(); pending_test_ = index;
  pending_ = Pending::TEST; sent_ = std::chrono::steady_clock::now(); render();
}

void UsvAutonomyPanel::timeoutPending()
{
  if (pending_ == Pending::MANAGE) {manager_->remove_pending_request(request_id_);}
  else if (pending_ == Pending::RESET) {reset_->remove_pending_request(request_id_);}
  else if (pending_ == Pending::TEST) {tests_[pending_test_]->remove_pending_request(request_id_);}
  pending_ = Pending::NONE;
  response_->setText("服务响应超时，结果未知；请查看状态，不能视为已取消/已停止。");
}

void UsvAutonomyPanel::poll()
{
  if (!rclcpp::ok()) {
    timer_->stop();
    for (auto widget : management_buttons_) {widget->setEnabled(false);}
    for (auto widget : test_buttons_) {widget->setEnabled(false);}
    reset_button_->setEnabled(false);
    state_label_->setText("ROS 已关闭");
    return;
  }
  rclcpp::spin_some(node_);
  using namespace std::chrono_literals;
  if (pending_ == Pending::MANAGE && management_future_.wait_for(0s) == std::future_status::ready) {
    auto result = management_future_.get();
    response_->setText(QString("管理确认 %1: %2%3").arg(result->code).arg(
        QString::fromStdString(result->reason), result->code == 202 ? "（以状态流确认激活/完成）" : ""));
    pending_ = Pending::NONE;
  } else if (pending_ == Pending::RESET && reset_future_.wait_for(0s) == std::future_status::ready) {
    auto result = reset_future_.get();
    response_->setText((result->accepted ? "复位确认: " : "复位未通过: ") + QString::fromStdString(result->reason));
    pending_ = Pending::NONE;
  } else if (pending_ == Pending::TEST && test_future_.wait_for(0s) == std::future_status::ready) {
    auto result = test_future_.get();
    response_->setText((result->success ? "仿真输入确认: " : "仿真输入失败: ") + QString::fromStdString(result->message));
    pending_ = Pending::NONE;
  } else if (pending_ != Pending::NONE && std::chrono::steady_clock::now() - sent_ > 2s) {
    timeoutPending();
  }
  render();
}

void UsvAutonomyPanel::render()
{
  if (!have_maritime_ || std::chrono::steady_clock::now() - maritime_received_ > std::chrono::seconds(1)) {
    maritime_label_->setText("海事：规则状态未收到或已过期");
  }
  if (!have_gate_ || std::chrono::steady_clock::now() - gate_received_ > std::chrono::seconds(1)) {
    gate_label_->setText("输出门控：状态未就绪/已过期，不能据旧显示确认推进状态。");
  }
  const bool matching = namespace_edit_->text().trimmed() == namespace_;
  const bool ready = matching && fresh() && manager_ && manager_->service_is_ready() && pending_ == Pending::NONE;
  if (fresh()) {
    QStringList blockers;
    for (const auto & blocker : state_.blockers) {blockers.append(QString::fromStdString(blocker));}
    diagnostic_label_->setText("船舶: " + namespace_ + "\n" + QString("任务: %1\n模式: %2  阶段: %3  意图: %4\n航点: %5/%6  圈: %7\n"
        "阻断: %8\n执行: %9\n尝试: %10  epoch: %11\n%12")
      .arg(QString::fromStdString(state_.task_id), QString::fromStdString(state_.mode),
        QString::fromStdString(state_.phase), QString::fromStdString(state_.user_intent))
      .arg(state_.waypoint_index).arg(state_.waypoint_count).arg(state_.lap)
      .arg(blockers.join(", "), QString::fromStdString(state_.execution_id),
        QString::fromStdString(state_.attempt_id)).arg(state_.epoch).arg(QString::fromStdString(state_.reason)));
    state_label_->setText(QString("%1 · %2 / %3\n航点 %4/%5 · 圈 %6 · 阻断: %7")
      .arg(namespace_, QString::fromStdString(state_.mode), QString::fromStdString(state_.phase))
      .arg(state_.waypoint_index).arg(state_.waypoint_count).arg(state_.lap)
      .arg(blockers.isEmpty() ? "无" : blockers.join(", ")));
    state_label_->setToolTip("任务: " + QString::fromStdString(state_.task_id) + "\n" +
      QString::fromStdString(state_.reason));
  } else {state_label_->setText("任务状态未就绪/已过期；等待当前会话快照。");}
  for (auto widget : management_buttons_) {
    bool enabled = ready;
    const auto name = widget->objectName();
    if (name == "start_navigation") {enabled = enabled && goals_->rowCount() > 0;}
    if (name == "pause_task" || name == "resume_task" || name == "stop_task") {
      enabled = enabled && !state_.task_id.empty() && state_.user_intent != "END";
    }
    if (name == "reset_fault") {enabled = enabled && !state_.task_id.empty();}
    if (name == "resume_stored") {enabled = enabled && !suspended_->currentText().trimmed().isEmpty();}
    const std::array<std::pair<QString, std::string>, 5> modes = {{{"switch_navigation", "NAVIGATION"},
      {"switch_hold", "HOLD"}, {"switch_survey", "SURVEY"},
      {"switch_dock", "DOCK"}, {"switch_undock", "UNDOCK"}}};
    for (const auto & mode : modes) {
      if (name != mode.first) {continue;}
      const bool selected = fresh() && state_.mode == mode.second && state_.user_intent != "END";
      widget->setChecked(selected);
      enabled = enabled && !selected;
    }
    widget->setEnabled(enabled);
  }
  reset_button_->setEnabled(matching && reset_ && reset_->service_is_ready() && pending_ == Pending::NONE);
  // Keep the original test controls discoverable even when unavailable. Disabled
  // authority buttons must never synthesize authority while observing a real FCU.
  tabs_->setTabVisible(tabs_->indexOf(tests_page_), true);
  test_source_label_->setText(!simulation_tests_ ? "仿真故障注入未启用；测试按钮不可用。" :
    simulation_authority_ ? "纯仿真：允许模拟控制权与故障注入。" :
    "真实飞控独占控制权；模拟自主权/人工接管禁用，其余仅注入仿真故障。");
  response_->setToolTip(response_->text());
  for (size_t i = 0; i < test_buttons_.size(); ++i) {
    const bool authority_allowed = i >= 2 || simulation_authority_;
    test_buttons_[i]->setEnabled(matching && simulation_tests_ && authority_allowed &&
      tests_[i/2] && tests_[i/2]->service_is_ready() && pending_ == Pending::NONE);
    test_buttons_[i]->setToolTip(!authority_allowed ?
      "真实飞控控制权只能从实时状态观察，不能通过此按钮伪造。请使用遥控器实际接管。" :
      !simulation_tests_ ? "启动时设置 enable_autonomy_test_controls:=true 以启用仿真故障注入。" :
      "仅注入仿真测试输入；完成以任务/安全状态反馈为准，不代表真实飞控已切换模式。");
  }
}

void UsvAutonomyPanel::load(const rviz_common::Config & config)
{
  Panel::load(config);
  auto selected = namespace_;
  config.mapGetString("Namespace", &selected);
  config.mapGetString("Global Frame", &frame_);
  config.mapGetBool("Simulation Tests", &simulation_tests_);
  simulation_authority_ = false;
  config.mapGetBool("Simulation Authority", &simulation_authority_);
  config.mapGetInt("Base Window Width", &base_width_);
  config.mapGetInt("Base Window Height", &base_height_);
  namespace_edit_->setText(selected);
  bind();
}

void UsvAutonomyPanel::save(rviz_common::Config config) const
{
  Panel::save(config);
  config.mapSetValue("Namespace", namespace_);
  config.mapSetValue("Global Frame", frame_);
  config.mapSetValue("Simulation Tests", simulation_tests_);
  config.mapSetValue("Simulation Authority", simulation_authority_);
  if (auto main = qobject_cast<QMainWindow *>(window())) {
    config.mapSetValue("Base Window Width", main->width());
    config.mapSetValue("Base Window Height", main->height());
  }
}
}  // namespace nav2_rviz_plugins
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::UsvAutonomyPanel, rviz_common::Panel)
