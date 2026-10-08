#include "nav2_colregs_vo_rrt_star_planner/rrt_star_planner.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <memory>
#include <mutex>
#include <limits>
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
  costmap_ros_ = costmap_ros;
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
  rcl_interfaces::msg::ParameterDescriptor deprecated;
  deprecated.read_only = true;
  deprecated.description = "Deprecated: ignored; this planner returns the exact goal.";
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".tolerance", rclcpp::ParameterValue(0.5), deprecated);
  RCLCPP_WARN(logger_, "%s.tolerance is deprecated and has no effect", name_.c_str());
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".prune_path", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_informed_sampling", rclcpp::ParameterValue(true));
  rcl_interfaces::msg::ParameterDescriptor anchor;
  anchor.read_only = true;
  anchor.description = "Start-to-live-pose gate [m], finite and nonnegative; restart to change.";
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".colregs_anchor_max_dist", rclcpp::ParameterValue(3.0), anchor);

  applied_parameters_.clear();
  applyParameters();
  node->get_parameter(name_ + ".colregs_anchor_max_dist", colregs_anchor_max_dist_);
  if (!std::isfinite(colregs_anchor_max_dist_) || colregs_anchor_max_dist_ < 0.0) {
    throw std::invalid_argument("colregs_anchor_max_dist must be finite and nonnegative");
  }

  avoidance_client_ = node->create_client<nav2_colregs_msgs::srv::GetAvoidancePoint>(
    "/get_avoidance_point");
  barrier_client_ = node->create_client<nav2_colregs_msgs::srv::GetBarrierLines>(
    "/get_barrier_lines");

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&VORRTStarPlanner::dynamicParametersCallback, this, std::placeholders::_1));

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
  dyn_params_handler_.reset();
  applied_parameters_.clear();
  costmap_ros_.reset();
}

void VORRTStarPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating VORRTStarPlanner: %s", name_.c_str());

}

void VORRTStarPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating VORRTStarPlanner: %s", name_.c_str());
}

// ---------------------------------------------------------------------------
// Plan
// ---------------------------------------------------------------------------

nav_msgs::msg::Path VORRTStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal
)
{
  // Apply only committed ROS values at a planning boundary, never in validation callbacks.
  applyParameters();
  nav2_costmap_2d::Costmap2D snapshot;
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
    snapshot = nav2_costmap_2d::Costmap2D(*costmap_);
  }
  const auto * query_map = &snapshot;
  if (!std::isfinite(start.pose.position.x) || !std::isfinite(start.pose.position.y) ||
    !std::isfinite(goal.pose.position.x) || !std::isfinite(goal.pose.position.y))
  {
    throw nav2_core::PlannerException("Start/goal coordinates must be finite");
  }
  // Validate start/goal are within costmap bounds.
  unsigned int start_mx, start_my, goal_mx, goal_my;
  if (!query_map->worldToMap(start.pose.position.x, start.pose.position.y,
                            start_mx, start_my))
  {
    throw nav2_core::PlannerException("Start is outside the map bounds.");
  }
  if (!query_map->worldToMap(goal.pose.position.x, goal.pose.position.y,
                            goal_mx, goal_my))
  {
    throw nav2_core::PlannerException("Goal is outside the map bounds.");
  }

  // Start/goal occupied check.
  if (query_map->getCost(start_mx, start_my) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
    throw nav2_core::PlannerException("Start is occupied.");
  }
  if (query_map->getCost(goal_mx, goal_my) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
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
      throw nav2_core::PlannerException("COLREGS: cannot get live robot pose for anchoring");
    }
  }

  // COLREGS: call Avoidance Point + Barrier (only for live-pose segments).
  if (anchored_to_robot) {
    auto request = std::make_shared<nav2_colregs_msgs::srv::GetAvoidancePoint::Request>();
    request->header.frame_id = global_frame_;
    request->header.stamp = clock_->now();
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

      using Response = nav2_colregs_msgs::srv::GetAvoidancePoint::Response;
      if (resp->status != Response::SUCCESS && resp->status != Response::NO_THREAT) {
        throw nav2_core::PlannerException("COLREGS avoidance failed: " + resp->message);
      }
      if (resp->header.frame_id != global_frame_ ||
        std::all_of(resp->snapshot_id.uuid.begin(), resp->snapshot_id.uuid.end(),
        [](uint8_t v) {return v == 0;}))
      {
        throw nav2_core::PlannerException("COLREGS: avoidance frame/snapshot mismatch");
      }
      if (resp->status == Response::SUCCESS) {
        if (!resp->has_feasible_angle || !std::isfinite(resp->point.x) ||
          !std::isfinite(resp->point.y) || !std::isfinite(resp->point.z))
        {
          throw nav2_core::PlannerException("COLREGS: malformed avoidance response");
        }
        avoidance_point = resp->point;

        auto barrier_req = std::make_shared<nav2_colregs_msgs::srv::GetBarrierLines::Request>();
        barrier_req->header = resp->header;
        barrier_req->snapshot_id = resp->snapshot_id;
        barrier_req->os_pose = start.pose;
        barrier_req->target_id = resp->primary_target_id;
        barrier_req->avoid_direction = "right";

        auto barrier_result = barrier_client_->async_send_request(barrier_req);
        if (barrier_result.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
          auto br = barrier_result.get();
          if (br->status != nav2_colregs_msgs::srv::GetBarrierLines::Response::SUCCESS ||
            br->snapshot_id != resp->snapshot_id || br->header != resp->header ||
            br->barriers.points.size() != 6 ||
            std::any_of(br->barriers.points.begin(), br->barriers.points.end(),
            [](const auto & p) {
              return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z);
            }))
          {
            throw nav2_core::PlannerException("COLREGS barrier failed/mismatched: " + br->message);
          }
          RCLCPP_INFO(logger_,
            "VORRTStarPlanner: /get_barrier_lines points=%zu",
            br->barriers.points.size());
          barrier_points = br->barriers.points;
          has_colregs_route = true;
        } else {
          barrier_client_->remove_pending_request(barrier_result);
          throw nav2_core::PlannerException("COLREGS: /get_barrier_lines timed out");
        }
      }
    } else {
      avoidance_client_->remove_pending_request(result);
      throw nav2_core::PlannerException("COLREGS: /get_avoidance_point timed out");
    }
  }

  // RRT* plan. In COLREGS mode, keep the first segment deterministic and
  // only run RRT* from the avoidance point to the final goal.
  auto t_start = std::chrono::steady_clock::now();
  std::vector<RRTStarNode> raw_path;
  bool success = false;

  if (has_colregs_route) {
    // The open-water VO leg uses predicted relative motion, not current costmap
    // occupancy. Costmap clearance and U barriers apply to the AP->goal search.
    std::vector<RRTStarNode> segment_path;
    success = rrt_star_->planPath(
      avoidance_point.x, avoidance_point.y,
      goal.pose.position.x, goal.pose.position.y,
      query_map, barrier_points, segment_path);

    if (success && prune_path_) {
      rrt_star_->prunePath(segment_path, query_map, barrier_points);
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
      query_map, no_barriers, raw_path);

    if (success && prune_path_) {
      rrt_star_->prunePath(raw_path, query_map, no_barriers);
    }
  }

  auto t_end = std::chrono::steady_clock::now();
  double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  if (!success || raw_path.empty()) {
    throw nav2_core::PlannerException(
      "VORRTStarPlanner: no valid path found.");
  }

  // Densify: linear interpolation at costmap resolution.
  nav_msgs::msg::Path plan = linearInterpolation(raw_path, query_map->getResolution());

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

std::vector<rclcpp::Parameter> VORRTStarPlanner::validatedParameters(
  const std::vector<rclcpp::Parameter> & overrides) const
{
  auto node = parent_node_.lock();
  if (!node) {
    throw std::runtime_error("Planner parent expired");
  }
  std::vector<std::string> names;
  for (const auto * key : {"step_size", "max_iterations", "goal_bias", "goal_threshold",
      "safety_dist", "cost_weight", "max_optimize_iters", "eta", "prune_path",
      "use_informed_sampling"})
  {
    names.push_back(name_ + "." + key);
  }
  auto values = node->get_parameters(names);
  for (auto & value : values) {
    for (const auto & replacement : overrides) {
      if (replacement.get_name() == value.get_name()) {
        value = replacement;
      }
    }
    const auto key = value.get_name().substr(name_.size() + 1);
    if (key == "prune_path" || key == "use_informed_sampling") {
      (void)value.as_bool();
    } else if (key == "max_iterations" || key == "max_optimize_iters") {
      const auto n = value.as_int();
      if (n < (key == "max_iterations" ? 1 : 0) || n > std::numeric_limits<int>::max()) {
        throw std::invalid_argument(key + " is outside the supported integer range");
      }
    } else {
      const auto v = value.as_double();
      if (!std::isfinite(v) || v < 0.0 ||
        ((key == "step_size" || key == "eta") && v == 0.0) ||
        (key == "goal_bias" && v > 1.0))
      {
        throw std::invalid_argument(key + " has an invalid finite/range value");
      }
    }
  }
  if (values[1].as_int() + values[6].as_int() > std::numeric_limits<int>::max()) {
    throw std::invalid_argument("Combined iteration budget exceeds INT_MAX");
  }
  return values;
}

void VORRTStarPlanner::applyParameters()
{
  const auto values = validatedParameters();
  if (rrt_star_ && values == applied_parameters_) {
    return;
  }
  step_size_ = values[0].as_double();
  max_iterations_ = static_cast<int>(values[1].as_int());
  goal_bias_ = values[2].as_double();
  goal_threshold_ = values[3].as_double();
  safety_dist_ = values[4].as_double();
  cost_weight_ = values[5].as_double();
  max_optimize_iters_ = static_cast<int>(values[6].as_int());
  eta_ = values[7].as_double();
  prune_path_ = values[8].as_bool();
  use_informed_sampling_ = values[9].as_bool();
  rrt_star_ = std::make_unique<RRTStar>(
    step_size_, max_iterations_, goal_bias_, goal_threshold_,
    safety_dist_, cost_weight_, max_optimize_iters_, eta_, use_informed_sampling_);
  applied_parameters_ = values;
  RCLCPP_INFO(logger_,
    "Applied %s parameters: step=%.3f iterations=%d+%d bias=%.3f goal=%.3f "
    "safety=%.3f weight=%.3f eta=%.3f prune=%d informed=%s",
    name_.c_str(), step_size_, max_iterations_, max_optimize_iters_, goal_bias_,
    goal_threshold_, safety_dist_, cost_weight_, eta_, prune_path_,
    use_informed_sampling_ ? "true" : "false");
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

  try {
    (void)validatedParameters(parameters);
  } catch (const std::exception & ex) {
    result.successful = false;
    result.reason = ex.what();
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
