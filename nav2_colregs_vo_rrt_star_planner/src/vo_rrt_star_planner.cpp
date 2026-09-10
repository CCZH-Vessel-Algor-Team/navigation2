#include "nav2_colregs_vo_rrt_star_planner/rrt_star_planner.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <memory>
#include <string>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"

namespace nav2_colregs_vo_rrt_star_planner
{

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void VORRTStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  name_ = name;
  tf_ = tf;
  parent_node_ = parent;
  clock_ = node->get_clock();
  logger_ = node->get_logger();
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".step_size", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_iterations", rclcpp::ParameterValue(1000));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal_bias", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal_threshold", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".safety_dist", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cost_weight", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_optimize_iters", rclcpp::ParameterValue(200));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".eta", rclcpp::ParameterValue(1.1));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".tolerance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".prune_path", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_informed_sampling", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".colregs_anchor_max_dist", rclcpp::ParameterValue(3.0));

  node->get_parameter(name_ + ".step_size", step_size_);
  node->get_parameter(name_ + ".max_iterations", max_iterations_);
  node->get_parameter(name_ + ".goal_bias", goal_bias_);
  node->get_parameter(name_ + ".goal_threshold", goal_threshold_);
  node->get_parameter(name_ + ".safety_dist", safety_dist_);
  node->get_parameter(name_ + ".cost_weight", cost_weight_);
  node->get_parameter(name_ + ".max_optimize_iters", max_optimize_iters_);
  node->get_parameter(name_ + ".eta", eta_);
  node->get_parameter(name_ + ".tolerance", tolerance_);
  node->get_parameter(name_ + ".prune_path", prune_path_);
  node->get_parameter(name_ + ".use_informed_sampling", use_informed_sampling_);
  node->get_parameter(name_ + ".colregs_anchor_max_dist", colregs_anchor_max_dist_);
  if (!std::isfinite(colregs_anchor_max_dist_) || colregs_anchor_max_dist_ < 0.0) {
    throw std::invalid_argument("colregs_anchor_max_dist must be finite and nonnegative");
  }

  avoidance_client_ = node->create_client<nav2_colregs_msgs::srv::GetAvoidancePoint>(
    "/get_avoidance_point");
  barrier_client_ = node->create_client<nav2_colregs_msgs::srv::GetBarrierLines>(
    "/get_barrier_lines");

  rrt_star_ = std::make_unique<RRTStar>(
    step_size_, max_iterations_, goal_bias_, goal_threshold_,
    safety_dist_, cost_weight_, max_optimize_iters_, eta_,
    use_informed_sampling_);

  RCLCPP_INFO(logger_, "VORRTStarPlanner configured: step=%.1f max_iter=%d "
    "goal_bias=%.2f goal_thresh=%.2f safety_dist=%.2f cost_weight=%.1f "
    "optimize_iters=%d eta=%.1f informed=%s",
    step_size_, max_iterations_, goal_bias_, goal_threshold_, safety_dist_,
    cost_weight_, max_optimize_iters_, eta_,
    use_informed_sampling_ ? "true" : "false");
}

void VORRTStarPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up VORRTStarPlanner: %s", name_.c_str());
  rrt_star_.reset();
  costmap_ros_.reset();
}

void VORRTStarPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating VORRTStarPlanner: %s", name_.c_str());

  auto node = parent_node_.lock();
  if (node) {
    dyn_params_handler_ = node->add_on_set_parameters_callback(
      std::bind(
        &VORRTStarPlanner::dynamicParametersCallback, this,
        std::placeholders::_1));
  }
}

void VORRTStarPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating VORRTStarPlanner: %s", name_.c_str());
  dyn_params_handler_.reset();
}

// ---------------------------------------------------------------------------
// Plan
// ---------------------------------------------------------------------------

nav_msgs::msg::Path VORRTStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal
)
{
  // Validate start/goal are within costmap bounds.
  unsigned int start_mx, start_my, goal_mx, goal_my;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y,
                            start_mx, start_my))
  {
    throw nav2_core::PlannerException("Start is outside the map bounds.");
  }
  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y,
                            goal_mx, goal_my))
  {
    throw nav2_core::PlannerException("Goal is outside the map bounds.");
  }

  // Start/goal occupied check.
  if (costmap_->getCost(start_mx, start_my) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
    throw nav2_core::PlannerException("Start is occupied.");
  }
  if (costmap_->getCost(goal_mx, goal_my) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
    throw nav2_core::PlannerException("Goal is occupied.");
  }

  bool has_colregs_route = false;
  geometry_msgs::msg::Point avoidance_point;
  std::vector<geometry_msgs::msg::Point> barrier_points;

  // COLREGS anchor guard: the VO decision (CPA, collision cone, avoidance
  // point) is only physically meaningful for the segment anchored at the
  // live robot pose. Under NavigateThroughPoses the standard planner_server
  // calls createPlan per segment; preview segments start at future goal
  // positions where the ts_manager's pose-anchored data would be
  // inconsistent. Skip the COLREGS service chain for those segments and
  // plan plain informed RRT* instead.
  bool anchored_to_robot = false;
  {
    geometry_msgs::msg::PoseStamped live_pose;
    if (costmap_ros_->getRobotPose(live_pose)) {
      anchored_to_robot = isColregsAnchored(
        start.pose.position.x, start.pose.position.y,
        live_pose.pose.position.x, live_pose.pose.position.y,
        colregs_anchor_max_dist_);
      const double dist_to_live = std::hypot(
        start.pose.position.x - live_pose.pose.position.x,
        start.pose.position.y - live_pose.pose.position.y);
      if (!anchored_to_robot) {
        RCLCPP_DEBUG(logger_,
          "VORRTStarPlanner: start (%.2f, %.2f) is %.2f m from live pose "
          "(> %.2f m): plain RRT* (preview segment)",
          start.pose.position.x, start.pose.position.y, dist_to_live,
          colregs_anchor_max_dist_);
      }
    } else {
      RCLCPP_WARN(logger_,
        "VORRTStarPlanner: cannot get robot pose; treating as non-anchored");
    }
  }

  // COLREGS: call Avoidance Point + Barrier (only for live-pose segments).
  if (anchored_to_robot) {
    auto request = std::make_shared<nav2_colregs_msgs::srv::GetAvoidancePoint::Request>();
    request->os_pose = start.pose;
    request->goal = goal.pose;
    request->avoid_direction = "right";

    auto result = avoidance_client_->async_send_request(request);
    if (result.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
      auto resp = result.get();
      RCLCPP_INFO(logger_,
        "VORRTStarPlanner: /get_avoidance_point has_threat=%d "
        "safe_heading=%.2f point=(%.2f,%.2f)",
        resp->has_feasible_angle, resp->safe_heading, resp->point.x, resp->point.y);

      if (resp->has_feasible_angle) {
        avoidance_point = resp->point;

        auto barrier_req = std::make_shared<nav2_colregs_msgs::srv::GetBarrierLines::Request>();
        barrier_req->os_pose = start.pose;
        barrier_req->target_id = resp->primary_target_id;
        barrier_req->avoid_direction = "right";

        auto barrier_result = barrier_client_->async_send_request(barrier_req);
        if (barrier_result.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
          auto br = barrier_result.get();
          RCLCPP_INFO(logger_,
            "VORRTStarPlanner: /get_barrier_lines points=%zu",
            br->barriers.points.size());
          barrier_points = br->barriers.points;
          has_colregs_route = true;
        } else {
          RCLCPP_WARN(logger_, "VORRTStarPlanner: /get_barrier_lines timed out");
        }
      }
    } else {
      RCLCPP_WARN(logger_, "VORRTStarPlanner: /get_avoidance_point timed out");
    }
  }

  // RRT* plan. In COLREGS mode, keep the first segment deterministic and
  // only run RRT* from the avoidance point to the final goal.
  auto t_start = std::chrono::steady_clock::now();
  std::vector<RRTStarNode> raw_path;
  bool success = false;

  if (has_colregs_route) {
    std::vector<RRTStarNode> segment_path;
    success = rrt_star_->planPath(
      avoidance_point.x, avoidance_point.y,
      goal.pose.position.x, goal.pose.position.y,
      costmap_, barrier_points, segment_path);

    if (success && prune_path_) {
      rrt_star_->prunePath(segment_path, costmap_, barrier_points);
    }

    if (success && !segment_path.empty()) {
      RRTStarNode start_node;
      start_node.x = start.pose.position.x;
      start_node.y = start.pose.position.y;
      start_node.parent_idx = -1;
      start_node.cost_from_root = 0.0;
      raw_path.push_back(start_node);
      raw_path.insert(raw_path.end(), segment_path.begin(), segment_path.end());
    }
  } else {
    const std::vector<geometry_msgs::msg::Point> no_barriers;
    success = rrt_star_->planPath(
      start.pose.position.x, start.pose.position.y,
      goal.pose.position.x, goal.pose.position.y,
      costmap_, no_barriers, raw_path);

    if (success && prune_path_) {
      rrt_star_->prunePath(raw_path, costmap_, no_barriers);
    }
  }

  auto t_end = std::chrono::steady_clock::now();
  double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  if (!success || raw_path.empty()) {
    throw nav2_core::PlannerException(
      "VORRTStarPlanner: no valid path found.");
  }

  // Densify: linear interpolation at costmap resolution.
  nav_msgs::msg::Path plan = linearInterpolation(raw_path, costmap_->getResolution());

  // Set header.
  plan.header.stamp = clock_->now();
  plan.header.frame_id = global_frame_;

  RCLCPP_INFO(logger_, "VORRTStarPlanner: found path with %zu points "
    "(raw=%zu, prune=%d, colregs=%d, barriers=%zu) plan=%.1f ms",
    plan.poses.size(), raw_path.size(), prune_path_, has_colregs_route,
    barrier_points.size(), elapsed_ms);

  return plan;
}

// ---------------------------------------------------------------------------
// Linear interpolation (same pattern as ThetaStar)
// ---------------------------------------------------------------------------

nav_msgs::msg::Path VORRTStarPlanner::linearInterpolation(
  const std::vector<RRTStarNode> & raw_path,
  double resolution)
{
  nav_msgs::msg::Path plan;

  if (raw_path.size() < 2) {
    for (const auto & node : raw_path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = node.x;
      pose.pose.position.y = node.y;
      pose.pose.orientation.w = 1.0;
      plan.poses.push_back(pose);
    }
    return plan;
  }

  for (size_t j = 0; j < raw_path.size() - 1; ++j) {
    const auto & pt1 = raw_path[j];
    geometry_msgs::msg::PoseStamped p1;
    p1.pose.position.x = pt1.x;
    p1.pose.position.y = pt1.y;
    p1.pose.orientation.w = 1.0;
    plan.poses.push_back(p1);

    const auto & pt2 = raw_path[j + 1];
    double distance = std::hypot(pt2.x - pt1.x, pt2.y - pt1.y);
    int loops = static_cast<int>(distance / resolution);
    if (loops > 0) {
      double cos_a = (pt2.x - pt1.x) / distance;
      double sin_a = (pt2.y - pt1.y) / distance;
      for (int k = 1; k < loops; ++k) {
        p1.pose.position.x = pt1.x + k * resolution * cos_a;
        p1.pose.position.y = pt1.y + k * resolution * sin_a;
        plan.poses.push_back(p1);
      }
    }
  }

  // Last point.
  const auto & last = raw_path.back();
  geometry_msgs::msg::PoseStamped plast;
  plast.pose.position.x = last.x;
  plast.pose.position.y = last.y;
  plast.pose.orientation.w = 1.0;
  plan.poses.push_back(plast);

  return plan;
}

}  // namespace nav2_colregs_vo_rrt_star_planner

// ---------------------------------------------------------------------------
// Dynamic parameters callback
// ---------------------------------------------------------------------------

rcl_interfaces::msg::SetParametersResult
nav2_colregs_vo_rrt_star_planner::VORRTStarPlanner::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    const auto & pname = param.get_name();
    if (pname == name_ + ".step_size") {
      step_size_ = param.as_double();
      rrt_star_ = std::make_unique<RRTStar>(
        step_size_, max_iterations_, goal_bias_, goal_threshold_,
        safety_dist_, cost_weight_, max_optimize_iters_, eta_);
    } else if (pname == name_ + ".max_iterations") {
      max_iterations_ = param.as_int();
      rrt_star_ = std::make_unique<RRTStar>(
        step_size_, max_iterations_, goal_bias_, goal_threshold_,
        safety_dist_, cost_weight_, max_optimize_iters_, eta_);
    } else if (pname == name_ + ".goal_bias") {
      goal_bias_ = param.as_double();
      rrt_star_ = std::make_unique<RRTStar>(
        step_size_, max_iterations_, goal_bias_, goal_threshold_,
        safety_dist_, cost_weight_, max_optimize_iters_, eta_);
    } else if (pname == name_ + ".goal_threshold") {
      goal_threshold_ = param.as_double();
      rrt_star_ = std::make_unique<RRTStar>(
        step_size_, max_iterations_, goal_bias_, goal_threshold_,
        safety_dist_, cost_weight_, max_optimize_iters_, eta_);
    } else if (pname == name_ + ".safety_dist") {
      safety_dist_ = param.as_double();
      rrt_star_ = std::make_unique<RRTStar>(
        step_size_, max_iterations_, goal_bias_, goal_threshold_,
        safety_dist_, cost_weight_, max_optimize_iters_, eta_);
    } else if (pname == name_ + ".cost_weight") {
      cost_weight_ = param.as_double();
      rrt_star_ = std::make_unique<RRTStar>(
        step_size_, max_iterations_, goal_bias_, goal_threshold_,
        safety_dist_, cost_weight_, max_optimize_iters_, eta_);
    } else if (pname == name_ + ".max_optimize_iters") {
      max_optimize_iters_ = param.as_int();
      rrt_star_ = std::make_unique<RRTStar>(
        step_size_, max_iterations_, goal_bias_, goal_threshold_,
        safety_dist_, cost_weight_, max_optimize_iters_, eta_);
    } else if (pname == name_ + ".eta") {
      eta_ = param.as_double();
      rrt_star_ = std::make_unique<RRTStar>(
        step_size_, max_iterations_, goal_bias_, goal_threshold_,
        safety_dist_, cost_weight_, max_optimize_iters_, eta_);
    } else if (pname == name_ + ".tolerance") {
      tolerance_ = param.as_double();
    } else if (pname == name_ + ".prune_path") {
      prune_path_ = param.as_bool();
    }
  }

  return result;
}

// ---------------------------------------------------------------------------
// Plugin export
// ---------------------------------------------------------------------------

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_colregs_vo_rrt_star_planner::VORRTStarPlanner,
  nav2_core::GlobalPlanner)
