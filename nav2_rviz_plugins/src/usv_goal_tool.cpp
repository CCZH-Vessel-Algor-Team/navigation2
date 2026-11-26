// Copyright 2026 USV team. Licensed under Apache-2.0.
#include "nav2_rviz_plugins/usv_goal_tool.hpp"
#include <cmath>
#include <QUuid>
#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_rviz_plugins
{
UsvGoalTool::UsvGoalTool()
{
  shortcut_key_ = 'u';
  auto options = rclcpp::NodeOptions().use_global_arguments(false);
  node_ = std::make_shared<rclcpp::Node>(
    "rviz_usv_goal_" + QUuid::createUuid().toString(QUuid::Id128).left(8).toStdString(), options);
  topic_ = new rviz_common::properties::StringProperty(
    "Topic", "/usv_1/autonomy/goal_pose", "Collect a waypoint in the task panel; do not start Nav2 directly.",
    getPropertyContainer(), SLOT(updateTopic()), this);
}

void UsvGoalTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("USV Test Goal");
  setIcon(rviz_common::loadPixmap("package://rviz_default_plugins/icons/classes/SetGoal.png"));
  updateTopic();
}

void UsvGoalTool::updateTopic()
{
  try {
    publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_->getStdString(), 10);
  } catch (const std::exception & e) {
    publisher_.reset();
    RCLCPP_ERROR(node_->get_logger(), "Invalid USV goal topic: %s", e.what());
  }
}

void UsvGoalTool::onPoseSet(double x, double y, double theta)
{
  if (!publisher_ || !rclcpp::ok()) {return;}
  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = node_->now();
  goal.header.frame_id = context_->getFixedFrame().toStdString();
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.orientation.z = std::sin(theta / 2.);
  goal.pose.orientation.w = std::cos(theta / 2.);
  publisher_->publish(goal);
}
}  // namespace nav2_rviz_plugins
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::UsvGoalTool, rviz_common::Tool)
