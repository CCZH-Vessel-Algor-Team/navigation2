// Copyright 2026 USV team. Licensed under Apache-2.0.
#include "nav2_rviz_plugins/managed_navigation.hpp"
#include <QUuid>

namespace nav2_rviz_plugins
{
ManagedNavigation::ManagedNavigation(const std::string & ns, QObject * parent,
  std::function<void()> updated, std::function<void()> goal_accepted)
: QObject(parent), updated_(updated), goal_accepted_(goal_accepted)
{
  auto options = rclcpp::NodeOptions().use_global_arguments(false);
  node_ = std::make_shared<rclcpp::Node>(
    "rviz_managed_navigation_" + QUuid::createUuid().toString(QUuid::Id128).left(8).toStdString(),
    ns.empty() ? "/" : "/" + ns, options);
  client_ = node_->create_client<Service>("autonomy/manage");
  subscription_ = node_->create_subscription<usv_interfaces::msg::AutonomyState>(
    "autonomy/state", 1, [this](usv_interfaces::msg::AutonomyState::ConstSharedPtr state) {
      state_ = *state; valid_ = true; received_ = std::chrono::steady_clock::now();
    });
  connect(&timer_, &QTimer::timeout, this, [this]() {poll();});
  timer_.start(100);
}

bool ManagedNavigation::fresh() const
{
  return valid_ && !state_.boot_id.empty() &&
         std::chrono::steady_clock::now() - received_ < std::chrono::seconds(1);
}

bool ManagedNavigation::ready() const {return fresh() && !pending_ && client_->service_is_ready();}

bool ManagedNavigation::send(const std::string & command,
  const std::vector<geometry_msgs::msg::PoseStamped> & goals, const std::string & policy)
{
  if (!ready()) {message_ = "等待新鲜任务状态/管理服务，未发送请求"; updated_(); return false;}
  auto request = std::make_shared<Service::Request>();
  request->command = command;
  request->expected_boot_id = state_.boot_id;
  request->expected_revision = state_.revision;
  request->request_id = "rviz-nav-" + QUuid::createUuid().toString(QUuid::Id128).toStdString();
  request->task_id = state_.task_id;
  if (command == "start") {
    if (goals.empty()) {return false;}
    request->task_id = "navigation-" + QUuid::createUuid().toString(QUuid::Id128).toStdString();
    request->mode = "NAVIGATION";
    request->goals = goals;
    request->waypoint_policy = policy;
  }
  auto future = client_->async_send_request(request);
  request_id_ = future.request_id;
  future_ = future.share();
  pending_ = true; submitting_goal_ = command == "start";
  sent_ = std::chrono::steady_clock::now();
  message_ = "管理请求已发送，完成以任务状态为准"; updated_();
  return true;
}

void ManagedNavigation::poll()
{
  if (!rclcpp::ok()) {timer_.stop(); return;}
  rclcpp::spin_some(node_);
  if (pending_ && future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    const auto result = future_.get();
    pending_ = false;
    message_ = QString("%1: %2").arg(result->code).arg(QString::fromStdString(result->reason));
    if (submitting_goal_ && result->accepted) {goal_accepted_();}
  } else if (pending_ && std::chrono::steady_clock::now() - sent_ > std::chrono::seconds(2)) {
    client_->remove_pending_request(request_id_); pending_ = false;
    message_ = "管理响应超时，结果未知，请核对任务状态";
  }
  updated_();
}
}  // namespace nav2_rviz_plugins
