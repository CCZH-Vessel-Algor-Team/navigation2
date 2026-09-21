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

#include <algorithm>
#include <array>
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
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

namespace nav2_colregs_local_planner_server
{

namespace
{
constexpr char invalid_ts_input_message[] =
  "TS planning input invalidated by clock epoch or lifecycle change";
constexpr std::array<const char *, 14> configured_parameters = {
  "action_server_result_timeout", "costmap_update_timeout", "max_planning_time",
  "step_size", "max_iterations", "goal_bias", "goal_threshold", "safety_dist",
  "cost_weight", "max_optimize_iters", "eta", "random_seed", "prune_path", "avoid_direction"};
}  // namespace

ColregsLocalPlannerServer::ColregsLocalPlannerServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("colregs_local_planner_server", "", options)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description =
    "Set only while UNCONFIGURED; validated and cached together on configure. "
    "Cleanup is required before changing this parameter.";
  declare_parameter("action_server_result_timeout", 10.0, descriptor);
  declare_parameter("costmap_update_timeout", 1.0, descriptor);
  declare_parameter("max_planning_time", 0.8, descriptor);
  declare_parameter("step_size", 1.0, descriptor);
  declare_parameter("max_iterations", 1000, descriptor);
  declare_parameter("goal_bias", 0.1, descriptor);
  declare_parameter("goal_threshold", 0.5, descriptor);
  declare_parameter("safety_dist", 0.3, descriptor);
  declare_parameter("cost_weight", 1.0, descriptor);
  declare_parameter("max_optimize_iters", 200, descriptor);
  declare_parameter("eta", 1.1, descriptor);
  declare_parameter("random_seed", 42, descriptor);
  declare_parameter("prune_path", true, descriptor);
  declare_parameter("avoid_direction", "right", descriptor);
  parameter_callback_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      if (get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
      {
        return result;
      }
      for (const auto & parameter : parameters) {
        if (std::find(
          configured_parameters.begin(), configured_parameters.end(),
          parameter.get_name()) != configured_parameters.end())
        {
          result.successful = false;
          result.reason = parameter.get_name() +
          " may only be set while UNCONFIGURED; cleanup before changing planner parameters";
          break;
        }
      }
      return result;
    });

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
  const auto action_server_result_timeout =
    get_parameter("action_server_result_timeout").as_double();
  const auto costmap_update_timeout = get_parameter("costmap_update_timeout").as_double();
  const auto max_planning_time = get_parameter("max_planning_time").as_double();
  const RRTStarParameters planner_parameters = {
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
  const auto avoid_direction = get_parameter("avoid_direction").as_string();

  const bool integer_ranges_valid =
    max_iterations > 0 && max_iterations <= std::numeric_limits<int>::max() &&
    max_optimize_iters >= 0 && max_optimize_iters <= std::numeric_limits<int>::max() &&
    random_seed >= 0 &&
    static_cast<uint64_t>(random_seed) <= std::numeric_limits<uint32_t>::max();
  const bool valid =
    std::isfinite(action_server_result_timeout) && action_server_result_timeout > 0.0 &&
    std::isfinite(costmap_update_timeout) && costmap_update_timeout > 0.0 &&
    std::isfinite(max_planning_time) && max_planning_time > 0.0 &&
    std::isfinite(planner_parameters.step_size) && planner_parameters.step_size > 0.0 &&
    std::isfinite(planner_parameters.goal_bias) && planner_parameters.goal_bias >= 0.0 &&
    planner_parameters.goal_bias <= 1.0 &&
    std::isfinite(planner_parameters.goal_threshold) &&
    planner_parameters.goal_threshold >= 0.0 &&
    std::isfinite(planner_parameters.safety_dist) && planner_parameters.safety_dist >= 0.0 &&
    std::isfinite(planner_parameters.cost_weight) && planner_parameters.cost_weight >= 0.0 &&
    std::isfinite(planner_parameters.eta) && planner_parameters.eta > 0.0 &&
    (avoid_direction == "right" || avoid_direction == "left") &&
    integer_ranges_valid;
  if (!valid) {
    RCLCPP_ERROR(get_logger(), "Invalid COLREGS RRT* planner parameters");
    return false;
  }
  action_server_result_timeout_ = action_server_result_timeout;
  costmap_update_timeout_ = costmap_update_timeout;
  max_planning_time_ = max_planning_time;
  planner_parameters_ = planner_parameters;
  avoid_direction_ = avoid_direction;
  RCLCPP_INFO(
    get_logger(),
    "Configured RRT*: result_timeout=%.3f costmap_timeout=%.3f planning_time=%.3f "
    "step=%.3f iterations=%d goal_bias=%.3f goal_threshold=%.3f safety_dist=%.3f "
    "cost_weight=%.3f optimize_iters=%d eta=%.3f seed=%u prune=%s avoid_direction=%s",
    action_server_result_timeout_, costmap_update_timeout_, max_planning_time_,
    planner_parameters_.step_size, planner_parameters_.max_iterations,
    planner_parameters_.goal_bias, planner_parameters_.goal_threshold,
    planner_parameters_.safety_dist, planner_parameters_.cost_weight,
    planner_parameters_.max_optimize_iters, planner_parameters_.eta,
    planner_parameters_.random_seed, planner_parameters_.prune_path ? "true" : "false",
    avoid_direction_.c_str());
  return true;
}

nav2_util::CallbackReturn ColregsLocalPlannerServer::on_configure(
  const rclcpp_lifecycle::State & state)
{
  try {
    if (!loadAndValidateParameters()) {
      return nav2_util::CallbackReturn::FAILURE;
    }
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
    decision_markers_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "colregs_decision_markers", 1);

    rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
    server_options.result_timeout.nanoseconds = RCL_S_TO_NS(action_server_result_timeout_);
    action_server_ = std::make_unique<ActionServer>(
      shared_from_this(), "compute_path_to_pose",
      std::bind(&ColregsLocalPlannerServer::computePlan, this), nullptr,
      std::chrono::milliseconds(500), true, server_options);
    action_server_poses_ = std::make_unique<ActionServerThroughPoses>(
      shared_from_this(), "compute_path_through_poses",
      std::bind(&ColregsLocalPlannerServer::computePlanThroughPoses, this), nullptr,
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
  decision_markers_publisher_->on_activate();
  action_server_->activate();
  action_server_poses_->activate();
  if (costmap_ros_->activate().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    action_server_poses_->deactivate();
    action_server_->deactivate();
    plan_publisher_->on_deactivate();
    decision_markers_publisher_->on_deactivate();
    return nav2_util::CallbackReturn::FAILURE;
  }
  if (ts_state_ros_->activate().id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    costmap_ros_->deactivate();
    action_server_poses_->deactivate();
    action_server_->deactivate();
    plan_publisher_->on_deactivate();
    decision_markers_publisher_->on_deactivate();
    return nav2_util::CallbackReturn::FAILURE;
  }
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ColregsLocalPlannerServer::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  action_server_->deactivate();
  action_server_poses_->deactivate();
  plan_publisher_->on_deactivate();
  decision_markers_publisher_->on_deactivate();
  costmap_ros_->deactivate();
  ts_state_ros_->deactivate();
  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ColregsLocalPlannerServer::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  action_server_.reset();
  action_server_poses_.reset();
  plan_publisher_.reset();
  decision_markers_publisher_.reset();
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
ColregsLocalPlannerServer::getTsPlanningInput(
  double os_x, double os_y, std::chrono::steady_clock::time_point deadline)
{
  return ts_state_ros_->getPlanningInput(os_x, os_y, deadline);
}

bool ColregsLocalPlannerServer::isTsPlanningInputCurrent(
  const nav2_colregs_ts_manager::ColregsTsStateROS::PlanningInput & input) const
{
  return ts_state_ros_->isPlanningInputCurrent(input);
}

void ColregsLocalPlannerServer::publishDecisionMarkers(
  const nav2_colregs_ts_manager::ColregsDecision & decision,
  const geometry_msgs::msg::PoseStamped & start)
{
  // Visualization only: published outside any costmap/TS lock, next to the
  // plan publisher. Active decisions show the avoidance arrow and the U
  // barrier of the same evaluation frame; inactive decisions clear both.
  auto markers = std::make_unique<visualization_msgs::msg::MarkerArray>();
  const auto stamp = now();
  const auto finite_point = [](const geometry_msgs::msg::Point & point) {
      return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    };
  const bool show_decision =
    decision.status == nav2_colregs_ts_manager::DecisionStatus::SUCCESS &&
    finite_point(start.pose.position) && finite_point(decision.avoidance_point) &&
    std::all_of(decision.barrier_points.begin(), decision.barrier_points.end(), finite_point);

  visualization_msgs::msg::Marker arrow;
  arrow.header.frame_id = costmap_ros_->getGlobalFrameID();
  arrow.header.stamp = stamp;
  arrow.ns = "avoidance";
  arrow.id = 0;
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.action = show_decision ? visualization_msgs::msg::Marker::ADD :
    visualization_msgs::msg::Marker::DELETE;
  if (show_decision) {
    arrow.points.resize(2);
    arrow.points[0].x = start.pose.position.x;
    arrow.points[0].y = start.pose.position.y;
    arrow.points[0].z = 0.0;
    arrow.points[1] = decision.avoidance_point;
  }
  arrow.scale.x = 0.1;
  arrow.scale.y = 0.2;
  arrow.scale.z = 0.2;
  arrow.color.r = 0.2;
  arrow.color.g = 0.8;
  arrow.color.b = 0.2;
  arrow.color.a = 0.8;
  arrow.lifetime = rclcpp::Duration::from_seconds(7.0);
  markers->markers.push_back(arrow);

  visualization_msgs::msg::Marker lines;
  lines.header.frame_id = costmap_ros_->getGlobalFrameID();
  lines.header.stamp = stamp;
  lines.ns = "barrier";
  lines.id = 0;
  lines.type = visualization_msgs::msg::Marker::LINE_LIST;
  lines.action = show_decision ? visualization_msgs::msg::Marker::ADD :
    visualization_msgs::msg::Marker::DELETE;
  lines.points = show_decision ? decision.barrier_points :
    std::vector<geometry_msgs::msg::Point>{};
  lines.scale.x = 0.05;
  lines.color.r = 0.9;
  lines.color.g = 0.2;
  lines.color.b = 0.2;
  lines.color.a = 0.8;
  lines.lifetime = rclcpp::Duration::from_seconds(7.0);
  markers->markers.push_back(lines);

  decision_markers_publisher_->publish(std::move(markers));
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
  const geometry_msgs::msg::PoseStamped & goal,
  double resolution)
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
    if (!action_server_->is_server_active()) {
      return;
    }
    auto result = std::make_shared<Action::Result>();
    const auto started = std::chrono::steady_clock::now();
    auto interrupted = [this]() {
        action_server_->terminate_pending_goal_if_cancel_requested();
        return !action_server_->is_server_active() ||
               action_server_->is_current_goal_cancel_requested() ||
               action_server_->is_preempt_requested();
      };

    action_server_->terminate_pending_goal_if_cancel_requested();
    if (action_server_->is_current_goal_cancel_requested()) {
      action_server_->terminate_current(result);
      return;
    }
    if (action_server_->is_preempt_requested()) {
      action_server_->terminate_current(result);
      goal = action_server_->accept_pending_goal();
      continue;
    }

    bool input_invalid = false;
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
      if (!transformPoseToGlobalFrame(start, transformed_start)) {
        abortGoal(result, "Unable to transform poses to costmap frame");
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

      const auto planning_deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(max_planning_time_));

      // One TS snapshot per request, anchored at the transformed planning
      // start (map frame).
      const auto ts_input = getTsPlanningInput(
        transformed_start.pose.position.x, transformed_start.pose.position.y, planning_deadline);
      const auto input_valid = [&]() {
          if (!input_invalid && !isTsPlanningInputCurrent(ts_input)) {
            input_invalid = true;
            publishDecisionMarkers(nav2_colregs_ts_manager::ColregsDecision{}, transformed_start);
          }
          return !input_invalid;
        };

      const auto segment = planSegment(
        transformed_start, goal->goal, snapshot, ts_input, interrupted, input_valid,
        planning_deadline, true, true);
      if (segment.status == PlanStatus::CANCELED) {
        continue;
      }
      if (segment.status != PlanStatus::SUCCESS) {
        abortGoal(result, segment.error);
        return;
      }

      result->path = segment.path;
      const auto elapsed = std::chrono::steady_clock::now() - started;
      const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed);
      const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
        elapsed - seconds);
      result->planning_time.sec = static_cast<int32_t>(seconds.count());
      result->planning_time.nanosec = static_cast<uint32_t>(nanoseconds.count());
      if (!input_valid()) {
        result->path.poses.clear();
        abortGoal(result, invalid_ts_input_message);
        return;
      }
      if (interrupted()) {
        continue;
      }
      if (!action_server_->succeed_current_if_not_interrupted(
          result, [&]() {
            // The action helper serializes goal-state updates, not TS epochs.
            // Recheck the token immediately before publishing under that guard.
            if (!input_valid()) {
              throw std::runtime_error(invalid_ts_input_message);
            }
            if (!action_server_->is_server_active()) {
              throw std::runtime_error("Planner action server is inactive");
            }
            plan_publisher_->publish(result->path);
          }))
      {
        continue;
      }
      return;
    } catch (const std::exception & error) {
      if (input_invalid) {
        result->path.poses.clear();
        abortGoal(result, invalid_ts_input_message);
        return;
      }
      if (interrupted()) {
        continue;
      }
      abortGoal(result, error.what());
      return;
    }
  }
}

void ColregsLocalPlannerServer::abortThroughPosesGoal(
  const std::shared_ptr<ActionThroughPoses::Result> & result,
  const std::string & message)
{
  RCLCPP_WARN(
    get_logger(), "Aborting ComputePathThroughPoses goal: %s", message.c_str());
  action_server_poses_->terminate_current(result);
}

ColregsLocalPlannerServer::SegmentPlan ColregsLocalPlannerServer::planSegment(
  const geometry_msgs::msg::PoseStamped & seg_start,
  const geometry_msgs::msg::PoseStamped & seg_goal,
  const nav2_costmap_2d::Costmap2D & snapshot,
  const nav2_colregs_ts_manager::ColregsTsStateROS::PlanningInput & ts_input,
  const std::function<bool()> & interrupted,
  const std::function<bool()> & input_valid,
  const std::chrono::steady_clock::time_point & deadline,
  bool apply_colregs,
  bool publish_markers)
{
  SegmentPlan outcome;
  const auto check_stopped = [&]() {
      if (!input_valid()) {
        outcome.status = PlanStatus::INVALID_INPUT;
        outcome.error = invalid_ts_input_message;
        return true;
      } else if (interrupted()) {
        outcome.status = PlanStatus::CANCELED;
      } else if (std::chrono::steady_clock::now() >= deadline) {
        outcome.status = PlanStatus::TIMEOUT;
        outcome.error = "COLREGS planning timed out";
      } else {
        return false;
      }
      if (publish_markers && apply_colregs) {
        publishDecisionMarkers(nav2_colregs_ts_manager::ColregsDecision{}, seg_start);
      }
      return true;
    };
  if (check_stopped()) {
    return outcome;
  }

  // Contract: seg_start is already in the costmap global frame (the caller
  // transformed it for the request-wide TS snapshot anchor); only the goal is
  // transformed here.
  const geometry_msgs::msg::PoseStamped & transformed_start = seg_start;
  geometry_msgs::msg::PoseStamped transformed_goal;
  if (!transformPoseToGlobalFrame(seg_goal, transformed_goal)) {
    outcome.status = PlanStatus::INVALID_INPUT;
    outcome.error = "Unable to transform poses to costmap frame";
    return outcome;
  }
  if (!std::isfinite(transformed_start.pose.position.x) ||
    !std::isfinite(transformed_start.pose.position.y) ||
    !std::isfinite(transformed_goal.pose.position.x) ||
    !std::isfinite(transformed_goal.pose.position.y))
  {
    outcome.status = PlanStatus::INVALID_INPUT;
    outcome.error = "Transformed start or goal pose contains non-finite x/y coordinates";
    return outcome;
  }
  unsigned int start_mx;
  unsigned int start_my;
  unsigned int goal_mx;
  unsigned int goal_my;
  if (!snapshot.worldToMap(
      transformed_start.pose.position.x, transformed_start.pose.position.y,
      start_mx, start_my))
  {
    outcome.status = PlanStatus::INVALID_INPUT;
    outcome.error = "Start pose is outside the costmap";
    return outcome;
  }
  if (!snapshot.worldToMap(
      transformed_goal.pose.position.x, transformed_goal.pose.position.y,
      goal_mx, goal_my))
  {
    outcome.status = PlanStatus::INVALID_INPUT;
    outcome.error = "Goal pose is outside the costmap";
    return outcome;
  }
  if (snapshot.getCost(start_mx, start_my) >=
    nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    outcome.status = PlanStatus::INVALID_INPUT;
    outcome.error = "Start pose is occupied";
    return outcome;
  }
  if (snapshot.getCost(goal_mx, goal_my) >=
    nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    outcome.status = PlanStatus::INVALID_INPUT;
    outcome.error = "Goal pose is occupied";
    return outcome;
  }

  // COLREGS TS decision only for the OS-anchored segment: the collision
  // cone and safe heading derive from the OS velocity captured in the
  // request-wide TS snapshot, which is physically meaningful only for the
  // first segment. Later segments are previews and always plan plain
  // barrier-free RRT*; re-planning re-anchors the OS as it advances.
  if (check_stopped()) {
    return outcome;
  }
  using nav2_colregs_ts_manager::DecisionStatus;
  nav2_colregs_ts_manager::ColregsDecision decision;
  if (apply_colregs) {
    const auto & ts_params = ts_input.params;
    const auto ts_snapshot = processTs(ts_input.ts, ts_input.os, ts_params);
    decision = evaluateColregs(
      ts_snapshot, ts_input.os,
      transformed_goal.pose.position.x, transformed_goal.pose.position.y,
      avoid_direction_, ts_params);
    RCLCPP_INFO(
      get_logger(),
      "COLREGS status=%s reason='%s' primary='%s' "
      "effective_threat_radius_scale=%.3f effective_avoidance_radius_scale=%.3f "
      "os_radius=%.3f tcpa_horizon=%.3f",
      decisionStatusName(decision.status), decision.reason.c_str(),
      decision.primary.target_id.c_str(), ts_params.threat_radius_scale,
      ts_params.avoidance_radius_scale, ts_params.os_radius, ts_params.threat_tcpa_horizon);
  }
  if (check_stopped()) {
    return outcome;
  }
  if (publish_markers && apply_colregs) {
    publishDecisionMarkers(decision, transformed_start);
  }
  if (apply_colregs) {
    switch (decision.status) {
      case DecisionStatus::SUCCESS:
      case DecisionStatus::NO_THREAT:
        break;
      case DecisionStatus::NO_DATA:
      case DecisionStatus::STALE_STATE:
      case DecisionStatus::INVALID_STATE:
      case DecisionStatus::INVALID_REQUEST:
      case DecisionStatus::INESCAPABLE:
      default:
        outcome.status = decision.status == DecisionStatus::INESCAPABLE ?
          PlanStatus::NO_PATH : PlanStatus::INVALID_INPUT;
        outcome.error = std::string("COLREGS ") + decisionStatusName(decision.status) +
          ": " + decision.reason;
        return outcome;
    }
  }

  RRTStar planner(planner_parameters_);
  const auto stop_planning = [&]() {
      return !input_valid() || interrupted();
    };
  std::vector<RRTStarNode> nodes;
  PlanStatus status = PlanStatus::NO_PATH;
  if (apply_colregs && decision.status == DecisionStatus::SUCCESS) {
    // Two-segment plan: the deterministic start→avoidance-point leg is
    // prepended as a raw node and only densified by makePath (no collision
    // check — VO-RRT semantics, design limitation L2); RRT* runs from the
    // avoidance point to the segment goal under the barriers.
    std::vector<RRTStarNode> rrt_nodes;
    status = planner.planPath(
      decision.avoidance_point.x, decision.avoidance_point.y,
      transformed_goal.pose.position.x, transformed_goal.pose.position.y,
      snapshot, decision.barrier_points, stop_planning, deadline,
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
      snapshot, no_barriers, stop_planning, deadline, nodes);
  }
  // RRT* reports its stop callback as CANCELED. An invalid TS token is instead
  // a terminal input failure, including for preview segments of this request.
  if (check_stopped()) {
    return outcome;
  }
  if (status == PlanStatus::CANCELED) {
    outcome.status = PlanStatus::CANCELED;
    return outcome;
  }
  if (status == PlanStatus::TIMEOUT) {
    outcome.status = PlanStatus::TIMEOUT;
    outcome.error = "RRT* planning timed out";
    return outcome;
  }
  if (status == PlanStatus::INVALID_INPUT) {
    outcome.status = PlanStatus::INVALID_INPUT;
    outcome.error = "RRT* rejected the planning input";
    return outcome;
  }
  if (status != PlanStatus::SUCCESS || nodes.empty()) {
    outcome.status = PlanStatus::NO_PATH;
    outcome.error = "RRT* failed to find a valid path";
    return outcome;
  }

  outcome.path = makePath(nodes, transformed_start, transformed_goal, snapshot.getResolution());
  if (check_stopped()) {
    outcome.path.poses.clear();
    return outcome;
  }
  outcome.status = PlanStatus::SUCCESS;
  return outcome;
}

void ColregsLocalPlannerServer::computePlanThroughPoses()
{
  if (!action_server_poses_ || !action_server_poses_->is_server_active()) {
    return;
  }
  auto goal = action_server_poses_->get_current_goal();
  if (!goal) {
    return;
  }

  while (goal) {
    if (!action_server_poses_->is_server_active()) {
      return;
    }
    auto result = std::make_shared<ActionThroughPoses::Result>();
    const auto started = std::chrono::steady_clock::now();
    auto interrupted = [this]() {
        action_server_poses_->terminate_pending_goal_if_cancel_requested();
        return !action_server_poses_->is_server_active() ||
               action_server_poses_->is_current_goal_cancel_requested() ||
               action_server_poses_->is_preempt_requested();
      };

    action_server_poses_->terminate_pending_goal_if_cancel_requested();
    if (action_server_poses_->is_current_goal_cancel_requested()) {
      action_server_poses_->terminate_current(result);
      return;
    }
    if (action_server_poses_->is_preempt_requested()) {
      action_server_poses_->terminate_current(result);
      goal = action_server_poses_->accept_pending_goal();
      continue;
    }

    bool input_invalid = false;
    try {
      if (!goal->planner_id.empty() && goal->planner_id != "RRTStar") {
        abortThroughPosesGoal(result, "Planner ID must be empty or RRTStar");
        return;
      }
      if (goal->goals.empty()) {
        abortThroughPosesGoal(result, "No viapoints given");
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
          abortThroughPosesGoal(result, "Costmap timed out waiting for an update");
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
        abortThroughPosesGoal(result, "Unable to obtain robot pose");
        return;
      }
      geometry_msgs::msg::PoseStamped transformed_start;
      if (!transformPoseToGlobalFrame(start, transformed_start)) {
        abortThroughPosesGoal(result, "Unable to transform poses to costmap frame");
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

      // One static costmap snapshot, one TS snapshot and one planning-time
      // budget are shared by all segments of this request. Segment k > 0
      // starts at the exact end of segment k-1 (upstream planner_server
      // chaining semantics).
      const auto planning_deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(max_planning_time_));
      const auto ts_input = getTsPlanningInput(
        transformed_start.pose.position.x, transformed_start.pose.position.y, planning_deadline);
      const auto input_valid = [&]() {
          if (!input_invalid && !isTsPlanningInputCurrent(ts_input)) {
            input_invalid = true;
            publishDecisionMarkers(nav2_colregs_ts_manager::ColregsDecision{}, transformed_start);
          }
          return !input_invalid;
        };

      nav_msgs::msg::Path concat_path;
      concat_path.header.frame_id = costmap_ros_->getGlobalFrameID();
      concat_path.header.stamp = now();
      bool canceled = false;
      for (size_t index = 0; index < goal->goals.size(); ++index) {
        if (!input_valid()) {
          abortThroughPosesGoal(result, invalid_ts_input_message);
          return;
        }
        if (interrupted()) {
          canceled = true;
          break;
        }
        geometry_msgs::msg::PoseStamped seg_start;
        if (index == 0) {
          seg_start = transformed_start;
        } else {
          seg_start = concat_path.poses.back();
          seg_start.header = concat_path.header;
        }
        const auto segment = planSegment(
          seg_start, goal->goals[index], snapshot, ts_input, interrupted, input_valid,
          planning_deadline, index == 0, index == 0);
        if (segment.status == PlanStatus::CANCELED) {
          canceled = true;
          break;
        }
        if (segment.status != PlanStatus::SUCCESS) {
          abortThroughPosesGoal(
            result, "segment " + std::to_string(index) + ": " + segment.error);
          return;
        }
        const auto & seg_poses = segment.path.poses;
        if (seg_poses.empty()) {
          abortThroughPosesGoal(
            result, "segment " + std::to_string(index) + ": empty segment path");
          return;
        }
        if (index == 0) {
          concat_path.poses.insert(
            concat_path.poses.end(), seg_poses.begin(), seg_poses.end());
        } else {
          // Skip the junction pose: it duplicates the previous segment end.
          const size_t skip = seg_poses.size() > 1u ? 1u : 0u;
          concat_path.poses.insert(
            concat_path.poses.end(), seg_poses.begin() + skip, seg_poses.end());
        }
      }
      if (canceled) {
        continue;
      }

      result->path = concat_path;
      const auto elapsed = std::chrono::steady_clock::now() - started;
      const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed);
      const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
        elapsed - seconds);
      result->planning_time.sec = static_cast<int32_t>(seconds.count());
      result->planning_time.nanosec = static_cast<uint32_t>(nanoseconds.count());
      if (!input_valid()) {
        result->path.poses.clear();
        abortThroughPosesGoal(result, invalid_ts_input_message);
        return;
      }
      if (interrupted()) {
        continue;
      }
      if (!action_server_poses_->succeed_current_if_not_interrupted(
          result, [&]() {
            if (!input_valid()) {
              throw std::runtime_error(invalid_ts_input_message);
            }
            if (!action_server_poses_->is_server_active()) {
              throw std::runtime_error("Planner action server is inactive");
            }
            plan_publisher_->publish(result->path);
          }))
      {
        continue;
      }
      return;
    } catch (const std::exception & error) {
      if (input_invalid) {
        result->path.poses.clear();
        abortThroughPosesGoal(result, invalid_ts_input_message);
        return;
      }
      if (interrupted()) {
        continue;
      }
      abortThroughPosesGoal(result, error.what());
      return;
    }
  }
}

}  // namespace nav2_colregs_local_planner_server

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(
  nav2_colregs_local_planner_server::ColregsLocalPlannerServer)
