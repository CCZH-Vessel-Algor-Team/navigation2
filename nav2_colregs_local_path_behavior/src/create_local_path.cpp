#include <cmath>
#include <chrono>
#include <memory>

#include "nav2_colregs_local_path_behavior/create_local_path.hpp"

namespace nav2_colregs_local_path_behavior
{

// ---------------------------------------------------------------------------
// Construction / Destruction
// ---------------------------------------------------------------------------

CreateLocalPath::CreateLocalPath()
: TimedBehavior<Action>()
{
}

CreateLocalPath::~CreateLocalPath()
{
}

// ---------------------------------------------------------------------------
// Lifecycle hooks
// ---------------------------------------------------------------------------

void CreateLocalPath::onConfigure()
{
  // Lock the parent LifecycleNode pointer for safe access.
  // No parameters are declared yet; this is a stub for future configuration.
  auto node = node_.lock();
}

// ---------------------------------------------------------------------------
// Action execution
// ---------------------------------------------------------------------------

ResultStatus CreateLocalPath::onRun(const std::shared_ptr<const Action::Goal> command)
{
  // Retrieve a safe reference to the LifecycleNode hosting this behavior.
  auto node = node_.lock();
  if (!node) {
    // The parent node has been destroyed — cannot proceed.
    return ResultStatus{Status::FAILED, ActionResult::EMPTY_PATH};
  }

  // Inspect the global path received from the planner (or BT).
  const size_t path_size = command->path.poses.size();

  if (path_size == 0) {
    // An empty path is semantically invalid for navigation.
    // Log a warning and fail so upstream BT logic can trigger recovery.
    RCLCPP_WARN(node->get_logger(), "Path is empty.");
    return ResultStatus{Status::FAILED, ActionResult::EMPTY_PATH};
  }

  // Extract the first and last waypoints for quick inspection.
  const auto & first = command->path.poses.front().pose.position;
  const auto & last  = command->path.poses.back().pose.position;

  RCLCPP_INFO(node->get_logger(), "Got global path, pose count: %zu", path_size);
  RCLCPP_INFO(
    node->get_logger(),
    "Path endpoints: first=(%.3f, %.3f, %.3f), last=(%.3f, %.3f, %.3f)",
    first.x, first.y, first.z, last.x, last.y, last.z);

  // Phase 1: inspection-only; no path mutation.
  return ResultStatus{Status::SUCCEEDED};
}

ResultStatus CreateLocalPath::onCycleUpdate()
{
  // This behavior completes in a single tick — no iterative work is needed.
  return ResultStatus{Status::SUCCEEDED};
}

}  // namespace nav2_colregs_local_path_behavior

// ---------------------------------------------------------------------------
// Plugin registration (pluginlib)
// ---------------------------------------------------------------------------
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_colregs_local_path_behavior::CreateLocalPath,
  nav2_core::Behavior)
