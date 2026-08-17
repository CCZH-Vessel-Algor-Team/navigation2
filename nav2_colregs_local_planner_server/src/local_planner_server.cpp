// Copyright (c) 2026 Vector Wang
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_colregs_local_planner_server/local_planner_server.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_colregs_ts_manager/ts_core.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "rcl_action/rcl_action.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace nav2_colregs_local_planner_server
{

ColregsLocalPlannerServer::ColregsLocalPlannerServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("colregs_local_planner_server", "", options)
{
  declare_parameter("action_server_result_timeout", 10.0);
  declare_parameter("costmap_update_timeout", 1.0);
  declare_parameter("max_planning_time", 0.8);
  declare_parameter("step_size", 1.0);
  declare_parameter("max_iterations", 1000);
  declare_parameter("goal_bias", 0.1);
  declare_parameter("goal_threshold", 0.5);
  declare_parameter("safety_dist", 0.3);
  declare_parameter("cost_weight", 1.0);
  declare_parameter("max_optimize_iters", 200);
  declare_parameter("eta", 1.1);
  declare_parameter("random_seed", 42);
  declare_parameter("prune_path", true);
  declare_parameter("avoid_direction", "right");

  // Humble Costmap2DROS has no use_sim_time constructor argument; pass it as
  // a parameter before the costmap configures its clock-dependent pieces.
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "colregs_costmap", std::string{get_namespace()}, "colregs_costmap");
  costmap_ros_->set_parameter(
    rclcpp::Parameter("use_sim_time", get_parameter("use_sim_time").as_bool()));
  ts_state_ros_ = std::make_shared<nav2_colregs_ts_manager::ColregsTsStateROS>(
    "colregs_ts_state", std::string{get_namespace()},
    get_parameter("use_sim_time").as_bool());
}

ColregsLocalPlannerServer::~ColregsLocalPlannerServer()
{
  costmap_thread_.reset();
  ts_thread_.reset();
}

bool ColregsLocalPlannerServer::loadAndValidateParameters()
{
  const auto max_iterations = get_parameter("max_iterations").as_int();
  const auto max_optimize_iters = get_parameter("max_optimize_iters").as_int();
  const auto random_seed = get_parameter("random_seed").as_int();
  action_server_result_timeout_ = get_parameter("action_server_result_timeout").as_double();
  costmap_update_timeout_ = get_parameter("costmap_update_timeout").as_double();
  max_planning_time_ = get_parameter("max_planning_time").as_double();
  planner_parameters_ = {
    get_parameter("step_size").as_double(),
    static_cast<int>(max_iterations),
    get_parameter("goal_bias").as_double(),
    get_parameter("goal_threshold").as_double(),
    get_parameter("safety_dist").as_double(),
    get_parameter("cost_weight").as_double(),
    static_cast<int>(max_optimize_iters),
    get_parameter("eta").as_double(),
    static_cast<uint32_t>(random_seed),
    get_parameter("prune_path").as_bool()};
  avoid_direction_ = get_parameter("avoid_direction").as_string();

  const bool integer_ranges_valid =
    max_iterations > 0 && max_iterations <= std::numeric_limits<int>::max() &&
    max_optimize_iters >= 0 && max_optimize_iters <= std::numeric_limits<int>::max() &&
    random_seed >= 0 &&
    static_cast<uint64_t>(random_seed) <= std::numeric_limits<uint32_t>::max();
  const bool valid =
    std::isfinite(action_server_result_timeout_) && action_server_result_timeout_ > 0.0 &&
    std::isfinite(costmap_update_timeout_) && costmap_update_timeout_ > 0.0 &&
    std::isfinite(max_planning_time_) && max_planning_time_ > 0.0 &&
    std::isfinite(planner_parameters_.step_size) && planner_parameters_.step_size > 0.0 &&
    std::isfinite(planner_parameters_.goal_bias) && planner_parameters_.goal_bias >= 0.0 &&
    planner_parameters_.goal_bias <= 1.0 &&
    std::isfinite(planner_parameters_.goal_threshold) &&
    planner_parameters_.goal_threshold >= 0.0 &&
    std::isfinite(planner_parameters_.safety_dist) && planner_parameters_.safety_dist >= 0.0 &&
    std::isfinite(planner_parameters_.cost_weight) && planner_parameters_.cost_weight >= 0.0 &&
    std::isfinite(planner_parameters_.eta) && planner_parameters_.eta > 0.0 &&
    (avoid_direction_ == "right" || avoid_direction_ == "left") &&
    integer_ranges_valid;
  if (!valid) {
    RCLCPP_ERROR(get_logger(), "Invalid COLREGS RRT* planner parameters");
  }
  return valid;
}

nav2_util::CallbackReturn ColregsLocalPlannerServer::on_configure(
  const rclcpp_lifecycle::State & state)
{
  if (!loadAndValidateParameters()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  try {
    const auto configured_global_frame =
      costmap_ros_->get_parameter("global_frame").as_string();
    if (configured_global_frame != "map") {
      RCLCPP_ERROR(
        get_logger(), "COLREGS planner costmap global_frame must be exactly 'map', got '%s'",
        configured_global_frame.c_str());
      return nav2_util::CallbackReturn::FAILURE;
    }
    if (costmap_ros_->configure().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      on_cleanup(state);
      return nav2_util::CallbackReturn::FAILURE;
    }
    costmap_ = costmap_ros_->getCostmap();
    costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
    if (ts_state_ros_->configure().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      on_cleanup(state);
      return nav2_util::CallbackReturn::FAILURE;
    }
    ts_thread_ = std::make_unique<nav2_util::NodeThread>(ts_state_ros_);
    plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);

    rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
    server_options.result_timeout.nanoseconds = RCL_S_TO_NS(action_server_result_timeout_);
    action_server_ = std::make_unique<ActionServer>(
      shared_from_this(), "compute_path_to_pose",
      std::bind(&ColregsLocalPlannerServer::computePlan, this), nullptr,
      std::chrono::milliseconds(500), true, server_options);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Failed to configure planner server: %s", error.what());
    on_cleanup(state);
    return nav2_util::CallbackReturn::FAILURE;
  }
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ColregsLocalPlannerServer::on_activate(
  const rclcpp_lifecycle::State &)
{
  plan_publisher_->on_activate();
  action_server_->activate();
  if (costmap_ros_->activate().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    action_server_->deactivate();
    plan_publisher_->on_deactivate();
    return nav2_util::CallbackReturn::FAILURE;
  }
  if (ts_state_ros_->activate().id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    costmap_ros_->deactivate();
    action_server_->deactivate();
    plan_publisher_->on_deactivate();
    return nav2_util::CallbackReturn::FAILURE;
  }
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ColregsLocalPlannerServer::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  action_server_->deactivate();
  plan_publisher_->on_deactivate();
  costmap_ros_->deactivate();
  ts_state_ros_->deactivate();
  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ColregsLocalPlannerServer::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  action_server_.reset();
  plan_publisher_.reset();
  if (costmap_ros_->get_current_state().id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
  {
    costmap_ros_->cleanup();
  }
  costmap_thread_.reset();
  costmap_ = nullptr;
  if (ts_state_ros_->get_current_state().id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
  {
    ts_state_ros_->cleanup();
  }
  ts_thread_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ColregsLocalPlannerServer::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  return nav2_util::CallbackReturn::SUCCESS;
}

bool ColregsLocalPlannerServer::isCostmapCurrent() const
{
  return costmap_ros_->isCurrent();
}

bool ColregsLocalPlannerServer::getRobotPose(geometry_msgs::msg::PoseStamped & pose) const
{
  return costmap_ros_->getRobotPose(pose);
}

bool ColregsLocalPlannerServer::transformPoseToGlobalFrame(
  const geometry_msgs::msg::PoseStamped & input,
  geometry_msgs::msg::PoseStamped & output) const
{
  return costmap_ros_->transformPoseToGlobalFrame(input, output);
}

nav2_colregs_ts_manager::ColregsTsStateROS::PlanningInput
ColregsLocalPlannerServer::getTsPlanningInput(double os_x, double os_y)
{
  return ts_state_ros_->getPlanningInput(os_x, os_y);
}

void ColregsLocalPlannerServer::abortGoal(
  const std::shared_ptr<Action::Result> & result,
  const std::string & message)
{
  RCLCPP_WARN(get_logger(), "Aborting ComputePathToPose goal: %s", message.c_str());
  action_server_->terminate_current(result);
}

nav_msgs::msg::Path ColregsLocalPlannerServer::makePath(
  const std::vector<RRTStarNode> & nodes,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = costmap_ros_->getGlobalFrameID();
  path.header.stamp = now();
  if (nodes.empty()) {
    return path;
  }

  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(nodes.size());
  geometry_msgs::msg::Point first;
  first.x = nodes.front().x;
  first.y = nodes.front().y;
  points.push_back(first);

  const double resolution = costmap_->getResolution();
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    throw std::runtime_error("COLREGS costmap resolution must be finite and positive");
  }
  for (size_t index = 1; index < nodes.size(); ++index) {
    const auto & previous = nodes[index - 1];
    const auto & current = nodes[index];
    const double distance = std::hypot(current.x - previous.x, current.y - previous.y);
    if (distance <= 1e-12) {
      continue;
    }
    const long double step_count = std::ceil(
      static_cast<long double>(distance) / static_cast<long double>(resolution));
    if (!std::isfinite(step_count) ||
      step_count > static_cast<long double>(std::numeric_limits<size_t>::max() - 1u))
    {
      throw std::runtime_error("COLREGS path interpolation step count is invalid");
    }
    const auto steps = static_cast<size_t>(step_count);
    for (size_t step = 1; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(steps);
      geometry_msgs::msg::Point point;
      point.x = previous.x + ratio * (current.x - previous.x);
      point.y = previous.y + ratio * (current.y - previous.y);
      points.push_back(point);
    }
  }
  if (points.size() == 1u) {
    points.push_back(points.front());
  }

  path.poses.resize(points.size());
  for (size_t index = 0; index < points.size(); ++index) {
    auto & pose = path.poses[index];
    pose.header = path.header;
    pose.pose.position = points[index];
    pose.pose.orientation.w = 1.0;
    if (index + 1 < points.size()) {
      const double yaw = std::atan2(
        points[index + 1].y - points[index].y,
        points[index + 1].x - points[index].x);
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, yaw);
      pose.pose.orientation = tf2::toMsg(orientation);
    }
  }
  if (!path.poses.empty()) {
    path.poses.front().pose = start.pose;
    path.poses.back().pose = goal.pose;
  }
  return path;
}

void ColregsLocalPlannerServer::computePlan()
{
  if (!action_server_ || !action_server_->is_server_active()) {
    return;
  }
  auto goal = action_server_->get_current_goal();
  if (!goal) {
    return;
  }

  while (goal) {
    auto result = std::make_shared<Action::Result>();
    const auto started = std::chrono::steady_clock::now();
    auto current_canceled = [this]() {
        return action_server_->is_cancel_requested();
      };
    auto interrupted = [this]() {
        if (action_server_->is_cancel_requested() &&
          action_server_->is_preempt_requested())
        {
          action_server_->terminate_pending_goal();
        }
        return action_server_->is_cancel_requested() ||
               action_server_->is_preempt_requested();
      };

    if (current_canceled()) {
      if (action_server_->is_preempt_requested()) {
        action_server_->terminate_pending_goal();
        continue;
      }
      action_server_->terminate_current(result);
      return;
    }
    if (action_server_->is_preempt_requested()) {
      action_server_->terminate_current(result);
      goal = action_server_->accept_pending_goal();
      continue;
    }

    try {
      if (!goal->planner_id.empty() && goal->planner_id != "RRTStar") {
        abortGoal(result, "Planner ID must be empty or RRTStar");
        return;
      }

      const auto costmap_deadline = started +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(costmap_update_timeout_));
      while (!isCostmapCurrent()) {
        if (interrupted()) {
          break;
        }
        if (std::chrono::steady_clock::now() >= costmap_deadline) {
          abortGoal(result, "Costmap timed out waiting for an update");
          return;
        }
        std::this_thread::sleep_for(10ms);
      }
      if (interrupted()) {
        continue;
      }

      geometry_msgs::msg::PoseStamped start;
      if (goal->use_start) {
        start = goal->start;
      } else if (!getRobotPose(start)) {
        abortGoal(result, "Unable to obtain robot pose");
        return;
      }
      geometry_msgs::msg::PoseStamped transformed_start;
      geometry_msgs::msg::PoseStamped transformed_goal;
      if (!transformPoseToGlobalFrame(start, transformed_start) ||
        !transformPoseToGlobalFrame(goal->goal, transformed_goal))
      {
        abortGoal(result, "Unable to transform poses to costmap frame");
        return;
      }
      if (!std::isfinite(transformed_start.pose.position.x) ||
        !std::isfinite(transformed_start.pose.position.y) ||
        !std::isfinite(transformed_goal.pose.position.x) ||
        !std::isfinite(transformed_goal.pose.position.y))
      {
        abortGoal(result, "Transformed start or goal pose contains non-finite x/y coordinates");
        return;
      }
      if (interrupted()) {
        continue;
      }

      nav2_costmap_2d::Costmap2D snapshot;
      {
        std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
        snapshot = *costmap_;
      }
      unsigned int start_mx;
      unsigned int start_my;
      unsigned int goal_mx;
      unsigned int goal_my;
      if (!snapshot.worldToMap(
          transformed_start.pose.position.x, transformed_start.pose.position.y,
          start_mx, start_my))
      {
        abortGoal(result, "Start pose is outside the costmap");
        return;
      }
      if (!snapshot.worldToMap(
          transformed_goal.pose.position.x, transformed_goal.pose.position.y,
          goal_mx, goal_my))
      {
        abortGoal(result, "Goal pose is outside the costmap");
        return;
      }
      if (snapshot.getCost(start_mx, start_my) >=
        nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        abortGoal(result, "Start pose is occupied");
        return;
      }
      if (snapshot.getCost(goal_mx, goal_my) >=
        nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        abortGoal(result, "Goal pose is occupied");
        return;
      }

      const auto planning_deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(max_planning_time_));

      // COLREGS TS decision: consistent snapshot from the TS sub-node, pure
      // evaluation with the sub-node's CPA/cone parameters. An inactive
      // decision (no threat, infeasible safe heading, or invalid OS velocity)
      // falls back to plain RRT* per design decision D5.
      const auto ts_input = getTsPlanningInput(
        transformed_start.pose.position.x, transformed_start.pose.position.y);
      const auto & ts_params = ts_state_ros_->coreParams();
      const auto ts_snapshot = processTs(ts_input.ts, ts_input.os, ts_params);
      const auto decision = evaluateColregs(
        ts_snapshot, ts_input.os,
        transformed_goal.pose.position.x, transformed_goal.pose.position.y,
        avoid_direction_, ts_params);
      if (interrupted()) {
        continue;
      }

      RRTStar planner(planner_parameters_);
      std::vector<RRTStarNode> nodes;
      PlanStatus status = PlanStatus::NO_PATH;
      if (decision.active) {
        // Two-segment plan: the deterministic start→avoidance-point leg is
        // prepended as a raw node and only densified by makePath (no
        // collision check — VO-RRT semantics, design limitation L2); RRT*
        // runs from the avoidance point to the goal under the barriers.
        std::vector<RRTStarNode> rrt_nodes;
        status = planner.planPath(
          decision.avoidance_point.x, decision.avoidance_point.y,
          transformed_goal.pose.position.x, transformed_goal.pose.position.y,
          snapshot, decision.barrier_points, interrupted, planning_deadline,
          rrt_nodes);
        if (status == PlanStatus::SUCCESS && !rrt_nodes.empty()) {
          rrt_nodes.insert(
            rrt_nodes.begin(),
            {transformed_start.pose.position.x,
              transformed_start.pose.position.y, -1, 0.0});
        }
        nodes = std::move(rrt_nodes);
      } else {
        const std::vector<geometry_msgs::msg::Point> no_barriers;
        status = planner.planPath(
          transformed_start.pose.position.x, transformed_start.pose.position.y,
          transformed_goal.pose.position.x, transformed_goal.pose.position.y,
          snapshot, no_barriers, interrupted, planning_deadline, nodes);
      }
      if (status == PlanStatus::CANCELED) {
        continue;
      }
      if (status == PlanStatus::TIMEOUT) {
        abortGoal(result, "RRT* planning timed out");
        return;
      }
      if (status == PlanStatus::INVALID_INPUT) {
        abortGoal(result, "RRT* rejected the planning input");
        return;
      }
      if (status != PlanStatus::SUCCESS || nodes.empty()) {
        abortGoal(result, "RRT* failed to find a valid path");
        return;
      }

      result->path = makePath(nodes, transformed_start, transformed_goal);
      const auto elapsed = std::chrono::steady_clock::now() - started;
      const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed);
      const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
        elapsed - seconds);
      result->planning_time.sec = static_cast<int32_t>(seconds.count());
      result->planning_time.nanosec = static_cast<uint32_t>(nanoseconds.count());
      if (interrupted()) {
        continue;
      }
      plan_publisher_->publish(result->path);
      action_server_->succeeded_current(result);
      return;
    } catch (const std::exception & error) {
      if (interrupted()) {
        continue;
      }
      abortGoal(result, error.what());
      return;
    }
  }
}

}  // namespace nav2_colregs_local_planner_server

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(
  nav2_colregs_local_planner_server::ColregsLocalPlannerServer)
