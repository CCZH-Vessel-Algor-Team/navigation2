// Copyright 2026 USV team. Licensed under Apache-2.0.
#ifndef NAV2_RVIZ_PLUGINS__USV_GOAL_TOOL_HPP_
#define NAV2_RVIZ_PLUGINS__USV_GOAL_TOOL_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rviz_default_plugins/tools/pose/pose_tool.hpp"
#include "rviz_common/properties/string_property.hpp"

namespace nav2_rviz_plugins
{
class UsvGoalTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT
public:
  UsvGoalTool();
  void onInitialize() override;
protected:
  void onPoseSet(double x, double y, double theta) override;
private Q_SLOTS:
  void updateTopic();
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rviz_common::properties::StringProperty * topic_;
};
}  // namespace nav2_rviz_plugins
#endif
