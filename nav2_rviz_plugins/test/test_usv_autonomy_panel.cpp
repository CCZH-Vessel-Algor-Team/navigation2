// Copyright 2026 USV team. Licensed under Apache-2.0.
#include <gtest/gtest.h>
#include <QApplication>
#include <QElapsedTimer>
#include <QGroupBox>
#include <QFontMetrics>
#include <QTest>
#include <functional>
#include <cmath>
#include <vector>
#include "nav2_rviz_plugins/usv_autonomy_panel.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_rviz_plugins/goal_common.hpp"

using Panel = rviz_common::Panel;
using Manage = usv_interfaces::srv::ManageMission;
using Reset = usv_interfaces::srv::ResetSafetyLatch;
using SetBool = std_srvs::srv::SetBool;
using State = usv_interfaces::msg::AutonomyState;

namespace
{
bool waitFor(const std::function<bool()> & predicate, int timeout = 3000)
{
  QElapsedTimer elapsed; elapsed.start();
  while (elapsed.elapsed() < timeout) {
    QCoreApplication::processEvents(); QTest::qWait(10);
    if (predicate()) {return true;}
  }
  return false;
}

class PanelContract : public ::testing::Test
{
protected:
  void SetUp() override
  {
    static int number = 0;
    ns = "panel_contract_" + std::to_string(number++);
    peer = std::make_shared<rclcpp::Node>("peer", "/" + ns);
    manager = peer->create_service<Manage>("autonomy/manage", [this](
        const Manage::Request::SharedPtr request, Manage::Response::SharedPtr response) {
        requests.push_back(*request);
        response->accepted = request->mode != "DOCK" && request->mode != "UNDOCK";
        response->code = response->accepted ? (
          request->command == "start" || request->command == "switch" ||
          request->command == "resume_task" ? 202 : 200) : 501;
        response->reason = response->accepted ? "transition acknowledged" : "executor unavailable";
      });
    reset = peer->create_service<Reset>("autonomy/reset_safety", [this](
        const Reset::Request::SharedPtr request, Reset::Response::SharedPtr response) {
        resets.push_back(request->confirmed);
        response->accepted = false; response->reason = "estop still active";
      });
    const std::array<std::string, 4> names = {"authority", "maritime_wait", "emergency", "localization_failure"};
    for (size_t index = 0; index < names.size(); ++index) {
      tests[index] = peer->create_service<SetBool>("autonomy/test/" + names[index], [this, index](
          const SetBool::Request::SharedPtr request, SetBool::Response::SharedPtr response) {
          injections.emplace_back(index, request->data);
          response->success = true; response->message = "test input acknowledged";
        });
    }
    state.boot_id = "gui-boot"; state.revision = 42;
    state.task_id = "active-route"; state.mode = "NAVIGATION";
    state.phase = "ACTIVE"; state.user_intent = "RUN";
    state.waypoint_index = 1; state.waypoint_count = 3;
    state_publisher = peer->create_publisher<State>("autonomy/state", 10);
    goal_publisher = peer->create_publisher<geometry_msgs::msg::PoseStamped>("autonomy/goal_pose", 10);
    QObject::connect(&publish, &QTimer::timeout, [this]() {state_publisher->publish(state);});
    QObject::connect(&spin, &QTimer::timeout, [this]() {rclcpp::spin_some(peer);});
    publish.start(40); spin.start(10);
    loader = std::make_unique<pluginlib::ClassLoader<Panel>>("rviz_common", "rviz_common::Panel");
    panel = loader->createSharedInstance("nav2_rviz_plugins/USV Autonomy");
    rviz_common::Config config;
    config.mapSetValue("Namespace", QString::fromStdString(ns));
    config.mapSetValue("Global Frame", "map"); config.mapSetValue("Simulation Tests", true);
    config.mapSetValue("Simulation Authority", true);
    panel->load(config); panel->resize(550, 1000); panel->show();
    ASSERT_TRUE(waitFor([this]() {return widget<QPushButton>("pause_task")->isEnabled();}));
  }
  void TearDown() override
  {
    publish.stop(); spin.stop(); panel.reset(); loader.reset();
    for (auto & service : tests) {service.reset();}
    manager.reset(); reset.reset(); state_publisher.reset(); goal_publisher.reset(); peer.reset();
  }
  template<typename T> T * widget(const char * name) {return panel->findChild<T *>(name);}
  void click(const char * name)
  {
    auto target = widget<QPushButton>(name);
    for (auto tabs : panel->findChildren<QTabWidget *>()) {
      for (int index = 0; index < tabs->count(); ++index) {
        if (tabs->widget(index)->isAncestorOf(target)) {tabs->setCurrentIndex(index);}
      }
    }
    ASSERT_TRUE(waitFor([&]() {return widget<QPushButton>(name)->isEnabled();}));
    QTest::mouseClick(widget<QPushButton>(name), Qt::LeftButton);
  }
  void management(const char * name, size_t count)
  {
    click(name);
    ASSERT_TRUE(waitFor([&]() {return requests.size() == count && widget<QPushButton>("pause_task")->isEnabled();}));
  }
  std::string ns;
  rclcpp::Node::SharedPtr peer;
  rclcpp::Service<Manage>::SharedPtr manager;
  rclcpp::Service<Reset>::SharedPtr reset;
  std::array<rclcpp::Service<SetBool>::SharedPtr, 4> tests;
  rclcpp::Publisher<State>::SharedPtr state_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher;
  State state;
  QTimer publish, spin;
  std::unique_ptr<pluginlib::ClassLoader<Panel>> loader;
  std::shared_ptr<Panel> panel;
  std::vector<Manage::Request> requests;
  std::vector<bool> resets;
  std::vector<std::pair<size_t, bool>> injections;
};

TEST_F(PanelContract, MapGoalUsesManagedIdentityAndDoesNotDoubleSubmit)
{
  EXPECT_FALSE(widget<QPushButton>("start_navigation")->isEnabled());
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map"; goal.pose.position.x = 12.; goal.pose.position.y = 23.;
  goal.pose.orientation.z = std::sqrt(.5); goal.pose.orientation.w = std::sqrt(.5);
  goal_publisher->publish(goal);
  ASSERT_TRUE(waitFor([&]() {return widget<QTableWidget>("waypoints")->rowCount() == 1;}));
  widget<QLineEdit>("new_task_id")->setText("gui-route");
  widget<QComboBox>("waypoint_policy")->setCurrentText("VIA");
  widget<QCheckBox>("loop_execution")->setChecked(true);
  click("start_navigation");
  EXPECT_FALSE(widget<QPushButton>("start_navigation")->isEnabled());
  QTest::mouseClick(widget<QPushButton>("start_navigation"), Qt::LeftButton);
  ASSERT_TRUE(waitFor([&]() {return requests.size() == 1 && widget<QPushButton>("pause_task")->isEnabled();}));
  const auto & request = requests.front();
  EXPECT_EQ(request.command, "start"); EXPECT_EQ(request.mode, "NAVIGATION");
  EXPECT_EQ(request.expected_boot_id, "gui-boot"); EXPECT_EQ(request.expected_revision, 42u);
  EXPECT_FALSE(request.request_id.empty()); EXPECT_EQ(request.task_id, "gui-route");
  EXPECT_EQ(request.waypoint_policy, "VIA"); EXPECT_TRUE(request.loop_execution);
  ASSERT_EQ(request.goals.size(), 1u); EXPECT_EQ(request.goals[0].header.frame_id, "map");
  EXPECT_DOUBLE_EQ(request.goals[0].pose.position.x, 12.);
  EXPECT_NEAR(request.goals[0].pose.orientation.z, std::sqrt(.5), 1.e-10);
  ASSERT_EQ(request.speeds.size(), 1u); EXPECT_DOUBLE_EQ(request.speeds[0], .8);
  EXPECT_TRUE(widget<QLabel>("management_response")->text().contains("202"));
  EXPECT_LE(panel->minimumSizeHint().height(), 500);
  EXPECT_LE(panel->minimumSizeHint().width(), 400);
}

TEST_F(PanelContract, LifecycleHoldDockAndResetUseTheirOwnContracts)
{
  management("pause_task", 1); management("resume_task", 2); management("stop_task", 3);
  ASSERT_EQ(requests.size(), 3u);
  EXPECT_EQ(requests[0].command, "pause"); EXPECT_EQ(requests[1].command, "resume");
  EXPECT_EQ(requests[2].command, "stop"); EXPECT_EQ(requests[2].task_id, "active-route");
  management("switch_hold", 4);
  EXPECT_EQ(requests.back().mode, "HOLD"); EXPECT_TRUE(requests.back().hold.use_current_pose);
  EXPECT_DOUBLE_EQ(requests.back().hold.inner_radius, 3.);
  EXPECT_NE(requests.back().task_id, "active-route");
  widget<QComboBox>("suspended_task_id")->setEditText("saved-route");
  management("resume_stored", 5);
  EXPECT_EQ(requests.back().command, "resume_task"); EXPECT_EQ(requests.back().task_id, "saved-route");
  management("switch_dock", 6); EXPECT_EQ(requests.back().mode, "DOCK");
  EXPECT_TRUE(widget<QLabel>("management_response")->text().contains("501"));
  management("switch_undock", 7); EXPECT_EQ(requests.back().mode, "UNDOCK");
  management("reset_fault", 8); EXPECT_EQ(requests.back().command, "reset_fault");
  click("reset_safety");
  ASSERT_TRUE(waitFor([&]() {return resets.size() == 1 && widget<QPushButton>("reset_safety")->isEnabled();}));
  EXPECT_TRUE(resets.front()); EXPECT_TRUE(widget<QLabel>("management_response")->text().contains("estop still active"));
  state.user_intent = "END"; state.phase = "FAULT";
  ASSERT_TRUE(waitFor([&]() {return !widget<QPushButton>("pause_task")->isEnabled() && widget<QPushButton>("reset_fault")->isEnabled();}));
  click("reset_fault");
  ASSERT_TRUE(waitFor([&]() {return requests.size() == 9 && widget<QPushButton>("reset_fault")->isEnabled();}));
  EXPECT_EQ(requests.back().command, "reset_fault");
}

TEST_F(PanelContract, SimulationServicesAndStaleBootFencing)
{
  const std::array<const char *, 8> names = {"test_authority_on", "test_authority_off",
    "test_maritime_wait_on", "test_maritime_wait_off", "test_emergency_on", "test_emergency_off",
    "test_localization_failure_on", "test_localization_failure_off"};
  for (size_t index = 0; index < names.size(); ++index) {
    click(names[index]);
    ASSERT_TRUE(waitFor([&]() {return injections.size() == index + 1 && widget<QPushButton>(names[index])->isEnabled();}));
    EXPECT_EQ(injections.back().first, index / 2); EXPECT_EQ(injections.back().second, index % 2 == 0);
  }
  EXPECT_TRUE(requests.empty());
  publish.stop();
  ASSERT_TRUE(waitFor([&]() {return !widget<QPushButton>("pause_task")->isEnabled();}, 2000));
  QTest::mouseClick(widget<QPushButton>("pause_task"), Qt::LeftButton);
  EXPECT_TRUE(requests.empty());
  state.boot_id = "new-boot"; state.revision = 1; publish.start(40);
  ASSERT_TRUE(waitFor([&]() {return widget<QPushButton>("pause_task")->isEnabled();}));
  management("pause_task", 1);
  EXPECT_EQ(requests.back().expected_boot_id, "new-boot"); EXPECT_EQ(requests.back().expected_revision, 1u);
  rviz_common::Config config; panel->save(config);
  QString saved; EXPECT_TRUE(config.mapGetString("Namespace", &saved)); EXPECT_EQ(saved.toStdString(), ns);
}

TEST_F(PanelContract, RealFcuKeepsTestPageAndFaultButtonsButCannotFakeAuthority)
{
  rviz_common::Config config;
  config.mapSetValue("Namespace", QString::fromStdString(ns));
  config.mapSetValue("Simulation Tests", true);
  config.mapSetValue("Simulation Authority", false);
  panel->load(config);
  ASSERT_TRUE(waitFor([&]() {return widget<QPushButton>("test_maritime_wait_on")->isEnabled();}));
  EXPECT_FALSE(widget<QPushButton>("test_authority_on")->isEnabled());
  EXPECT_FALSE(widget<QPushButton>("test_authority_off")->isEnabled());
  EXPECT_TRUE(widget<QPushButton>("test_authority_on")->toolTip().contains("真实飞控"));
  auto tabs = widget<QTabWidget>("task_tabs");
  bool visible = false;
  for (int i=0; i<tabs->count(); ++i) {
    if (tabs->tabText(i) == "测试") {visible = tabs->isTabVisible(i);}
  }
  EXPECT_TRUE(visible);
  // Even if the UI button were externally enabled, the callback must refuse it.
  auto authority = widget<QPushButton>("test_authority_on");
  authority->setEnabled(true); authority->click();
  QTest::qWait(100); QCoreApplication::processEvents();
  EXPECT_TRUE(injections.empty());
  click("test_maritime_wait_on");
  ASSERT_TRUE(waitFor([&]() {return injections.size() == 1;}));
  EXPECT_EQ(injections.front().first,1u);
  panel->save(config);
  bool saved = true; ASSERT_TRUE(config.mapGetBool("Simulation Authority", &saved)); EXPECT_FALSE(saved);
}

TEST_F(PanelContract, DisabledTestControlsRemainVisibleAndChineseButtonGlyphsAreAvailable)
{
  rviz_common::Config config;
  config.mapSetValue("Namespace", QString::fromStdString(ns));
  config.mapSetValue("Simulation Tests", false);
  panel->load(config);
  ASSERT_TRUE(waitFor([&]() {return !widget<QPushButton>("test_emergency_on")->isEnabled();}));
  auto tabs = widget<QTabWidget>("task_tabs");
  for (int i=0; i<tabs->count(); ++i) {
    if (tabs->tabText(i) == "测试") {EXPECT_TRUE(tabs->isTabVisible(i));}
  }
  for (auto button : panel->findChildren<QPushButton *>()) {
    EXPECT_FALSE(button->text().isEmpty());
    EXPECT_TRUE(QFontMetrics(button->font()).inFontUcs4(0x6682))
      << "Chinese glyph missing; install fonts-noto-cjk in the RViz container";
  }
  EXPECT_DOUBLE_EQ(panel->font().pointSizeF(), QApplication::font().pointSizeF());
}

TEST_F(PanelContract, HoldAnchorSourceSelectsManualMapPointOrCurrentPosition)
{
  auto source = widget<QComboBox>("hold_anchor_source");
  ASSERT_NE(source, nullptr);
  EXPECT_EQ(source->currentIndex(), 0);
  EXPECT_FALSE(widget<QDoubleSpinBox>("hold_anchor_x")->isEnabled());
  source->setCurrentIndex(1);
  EXPECT_TRUE(widget<QDoubleSpinBox>("hold_anchor_x")->isEnabled());
  widget<QDoubleSpinBox>("hold_anchor_x")->setValue(12.5);
  widget<QDoubleSpinBox>("hold_anchor_y")->setValue(-23.75);
  management("switch_hold", 1);
  EXPECT_FALSE(requests.back().hold.use_current_pose);
  EXPECT_EQ(requests.back().hold.anchor.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(requests.back().hold.anchor.pose.position.x, 12.5);
  EXPECT_DOUBLE_EQ(requests.back().hold.anchor.pose.position.y, -23.75);
  EXPECT_DOUBLE_EQ(requests.back().hold.anchor.pose.orientation.w, 1.0);
  source->setCurrentIndex(0);
  management("apply_hold_anchor", 2);
  EXPECT_TRUE(requests.back().hold.use_current_pose);
  EXPECT_EQ(requests.back().command, "switch");
  EXPECT_EQ(requests.back().mode, "HOLD");
}

TEST_F(PanelContract, PeerModeButtonsRestoreNavigationAndOnlyReflectConfirmedMode)
{
  click("add_waypoint");  // Existing draft must not replace a suspended route.
  state.mode = "HOLD";
  ASSERT_TRUE(waitFor([&]() {return widget<QPushButton>("switch_hold")->isChecked();}));
  auto modes = widget<QGroupBox>("mode_switches");
  ASSERT_NE(modes, nullptr);
  for (const auto name : {"switch_navigation", "switch_hold", "switch_dock", "switch_undock"}) {
    EXPECT_TRUE(modes->isAncestorOf(widget<QPushButton>(name)));
  }
  management("switch_navigation", 1);
  EXPECT_EQ(requests.back().command, "resume_navigation");
  EXPECT_EQ(requests.back().mode, "NAVIGATION");
  EXPECT_TRUE(requests.back().task_id.empty());
  EXPECT_TRUE(requests.back().goals.empty());
  EXPECT_TRUE(widget<QPushButton>("switch_hold")->isChecked());
  management("switch_dock", 2);
  EXPECT_TRUE(widget<QLabel>("management_response")->text().contains("501"));
  EXPECT_TRUE(widget<QPushButton>("switch_hold")->isChecked());
  EXPECT_FALSE(widget<QPushButton>("switch_dock")->isChecked());
  state.mode = "NAVIGATION";
  ASSERT_TRUE(waitFor([&]() {return widget<QPushButton>("switch_navigation")->isChecked();}));
  EXPECT_FALSE(widget<QPushButton>("switch_hold")->isChecked());
}

TEST_F(PanelContract, InvalidInputDoesNotSendAndNamespaceChangeDropsOldWaypoints)
{
  click("add_waypoint");
  widget<QTableWidget>("waypoints")->item(0, 0)->setText("nan");
  click("start_navigation"); QTest::qWait(150);
  EXPECT_TRUE(requests.empty());
  widget<QDoubleSpinBox>("hold_inner")->setValue(11.);
  click("switch_hold"); QTest::qWait(150); EXPECT_TRUE(requests.empty());
  widget<QLineEdit>("vessel_namespace")->setText("other_vessel");
  QTest::mouseClick(widget<QPushButton>("pause_task"), Qt::LeftButton);
  QTest::qWait(150); EXPECT_TRUE(requests.empty());
  click("connect_vessel");
  EXPECT_EQ(widget<QTableWidget>("waypoints")->rowCount(), 0);
  EXPECT_FALSE(widget<QPushButton>("pause_task")->isEnabled());
}

TEST_F(PanelContract, OriginalNav2GoalAndButtonsUseManagedLifecycleWithoutLayoutReplacement)
{
  auto original = loader->createSharedInstance("nav2_rviz_plugins/Navigation 2");
  rviz_common::Config config;
  config.mapSetValue("Managed Task", true);
  config.mapSetValue("Namespace", QString::fromStdString(ns));
  original->load(config); original->show();
  auto pause = original->findChild<QPushButton *>("nav2_pause_resume");
  auto mode = original->findChild<QPushButton *>("nav2_navigation_mode");
  auto cancel = original->findChild<QPushButton *>("nav2_start_reset");
  ASSERT_TRUE(waitFor([&]() {return pause->isEnabled();}));
  nav2_rviz_plugins::GoalUpdater.setGoal(10., 20., .5, "map");
  ASSERT_TRUE(waitFor([&]() {return requests.size() == 1 && pause->isEnabled();}));
  EXPECT_EQ(requests.back().command, "start"); EXPECT_EQ(requests.back().expected_boot_id, "gui-boot");
  EXPECT_EQ(requests.back().goals.size(), 1u);
  QTest::mouseClick(pause, Qt::LeftButton);
  ASSERT_TRUE(waitFor([&]() {return requests.size() == 2 && pause->isEnabled();}));
  EXPECT_EQ(requests.back().command, "pause");
  state.user_intent = "PAUSE"; state.phase = "PAUSED";
  ASSERT_TRUE(waitFor([&]() {return pause->text() == "Resume";}));
  QTest::mouseClick(pause, Qt::LeftButton);
  ASSERT_TRUE(waitFor([&]() {return requests.size() == 3 && pause->isEnabled();}));
  EXPECT_EQ(requests.back().command, "resume");
  QTest::mouseClick(mode, Qt::LeftButton);
  nav2_rviz_plugins::GoalUpdater.setGoal(30., 0., 0., "map");
  nav2_rviz_plugins::GoalUpdater.setGoal(40., 0., 0., "map");
  ASSERT_TRUE(waitFor([&]() {return pause->text() == "Start Nav Through Poses" && pause->isEnabled();}));
  QTest::mouseClick(pause, Qt::LeftButton);
  ASSERT_TRUE(waitFor([&]() {return requests.size() == 4 && pause->isEnabled();}));
  EXPECT_EQ(requests.back().waypoint_policy, "VIA"); EXPECT_EQ(requests.back().goals.size(), 2u);
  QTest::mouseClick(cancel, Qt::LeftButton);
  ASSERT_TRUE(waitFor([&]() {return requests.size() == 5 && pause->isEnabled();}));
  EXPECT_EQ(requests.back().command, "stop");
}
}  // namespace

int main(int argc, char ** argv)
{
  if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM")) {qputenv("QT_QPA_PLATFORM", "offscreen");}
  QApplication application(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
