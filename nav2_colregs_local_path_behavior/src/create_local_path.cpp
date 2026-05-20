// Copyright (c) 2020 Samsung Research America
// This code is licensed under MIT license (see LICENSE.txt for details)

#include <cmath>
#include <chrono>
#include <memory>

#include "nav2_colregs_local_path_behavior/create_local_path.hpp"

namespace nav2_colregs_local_path_behavior
{

CreateLocalPath::CreateLocalPath()
: TimedBehavior<Action>()
{
}

CreateLocalPath::~CreateLocalPath()
{
}

void CreateLocalPath::onConfigure()
{
  auto node = node_.lock();
}

ResultStatus CreateLocalPath::onRun(const std::shared_ptr<const Action::Goal> command)
{
  auto node = node_.lock();
  if (!node) {
    return ResultStatus{Status::FAILED, ActionResult::EMPTY_PATH};
  }

  const size_t path_size = command->path.poses.size();

  if (path_size == 0) {
    RCLCPP_WARN(node->get_logger(), "Path is empty.");
    return ResultStatus{Status::FAILED, ActionResult::EMPTY_PATH};
  }

  const auto & first = command->path.poses.front().pose.position;
  const auto & last = command->path.poses.back().pose.position;

  RCLCPP_INFO(node->get_logger(), "Got global path, pose count: %zu", path_size);
  RCLCPP_INFO(
    node->get_logger(),
    "Path endpoints: first=(%.3f, %.3f, %.3f), last=(%.3f, %.3f, %.3f)",
    first.x, first.y, first.z, last.x, last.y, last.z);

  return ResultStatus{Status::SUCCEEDED};
}

ResultStatus CreateLocalPath::onCycleUpdate()
{
  return ResultStatus{Status::SUCCEEDED};
}

}  // namespace nav2_colregs_local_path_behavior

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_colregs_local_path_behavior::CreateLocalPath, nav2_core::Behavior)
