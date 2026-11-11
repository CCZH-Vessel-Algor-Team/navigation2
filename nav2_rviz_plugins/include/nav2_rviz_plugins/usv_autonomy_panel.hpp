// Copyright 2026 USV team. Licensed under Apache-2.0.
#ifndef NAV2_RVIZ_PLUGINS__USV_AUTONOMY_PANEL_HPP_
#define NAV2_RVIZ_PLUGINS__USV_AUTONOMY_PANEL_HPP_

#include <array>
#include <chrono>
#include <future>
#include <memory>
#include <vector>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QTimer>
#include <QTabWidget>
#include "rviz_common/panel.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "usv_interfaces/msg/autonomy_state.hpp"
#include "usv_interfaces/msg/gate_state.hpp"
#include "usv_interfaces/msg/maritime_rule_decision.hpp"
#include "usv_interfaces/msg/safety_constraint_state.hpp"
#include "usv_interfaces/srv/manage_mission.hpp"
#include "usv_interfaces/srv/reset_safety_latch.hpp"

namespace nav2_rviz_plugins
{
class UsvAutonomyPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit UsvAutonomyPanel(QWidget * parent = nullptr);
  ~UsvAutonomyPanel() override;
  void onInitialize() override;
  QSize sizeHint() const override {return QSize(320, 380);}
  QSize minimumSizeHint() const override {return QSize(240, 240);}
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  using Manage = usv_interfaces::srv::ManageMission;
  using Reset = usv_interfaces::srv::ResetSafetyLatch;
  using Test = std_srvs::srv::SetBool;
  enum class Pending {NONE, MANAGE, RESET, TEST};
  QPushButton * button(const QString & text, const QString & name);
  QDoubleSpinBox * number(const QString & name, double value, double low, double high);
  void bind();
  void poll();
  void render();
  bool fresh() const;
  void addWaypoint(double x, double y, double yaw, double speed);
  void collect(const geometry_msgs::msg::PoseStamped & pose);
  void manage(const std::string & command, const std::string & mode = "");
  void resetSafety();
  void test(size_t index, bool value);
  void timeoutPending();
  QString prefix() const;

  QLineEdit * namespace_edit_;
  QLineEdit * task_id_;
  QComboBox * suspended_;
  QComboBox * policy_;
  QCheckBox * loop_;
  QCheckBox * collect_;
  QTableWidget * goals_;
  QDoubleSpinBox * x_;
  QDoubleSpinBox * y_;
  QDoubleSpinBox * yaw_;
  QDoubleSpinBox * speed_;
  QDoubleSpinBox * inner_;
  QDoubleSpinBox * outer_;
  QDoubleSpinBox * hold_speed_;
  QDoubleSpinBox * hold_duration_;
  QComboBox * hold_anchor_source_;
  QDoubleSpinBox * hold_anchor_x_;
  QDoubleSpinBox * hold_anchor_y_;
  QLabel * state_label_;
  QLabel * gate_label_;
  QLabel * maritime_label_;
  QLabel * response_;
  QLabel * frame_label_;
  QLabel * diagnostic_label_;
  QTabWidget * tabs_;
  QWidget * tests_page_;
  QTimer * timer_;
  std::vector<QPushButton *> management_buttons_;
  std::array<QPushButton *, 8> test_buttons_{};
  QPushButton * reset_button_;

  QString namespace_{"usv_1"};
  QString frame_{"map"};
  bool simulation_tests_{false};
  int base_width_{0}, base_height_{0};
  bool have_state_{false};
  bool have_gate_{false};
  bool have_maritime_{false};
  Pending pending_{Pending::NONE};
  int64_t request_id_{0};
  size_t pending_test_{0};
  std::chrono::steady_clock::time_point received_, sent_;
  std::chrono::steady_clock::time_point gate_received_;
  std::chrono::steady_clock::time_point maritime_received_;
  usv_interfaces::msg::AutonomyState state_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<Manage>::SharedPtr manager_;
  rclcpp::Client<Reset>::SharedPtr reset_;
  std::array<rclcpp::Client<Test>::SharedPtr, 4> tests_;
  rclcpp::Subscription<usv_interfaces::msg::AutonomyState>::SharedPtr state_sub_;
  rclcpp::Subscription<usv_interfaces::msg::GateState>::SharedPtr gate_sub_;
  rclcpp::Subscription<usv_interfaces::msg::MaritimeRuleDecision>::SharedPtr maritime_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  std::shared_future<Manage::Response::SharedPtr> management_future_;
  std::shared_future<Reset::Response::SharedPtr> reset_future_;
  std::shared_future<Test::Response::SharedPtr> test_future_;
};
}  // namespace nav2_rviz_plugins
#endif
