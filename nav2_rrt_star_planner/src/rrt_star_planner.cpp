#include "nav2_rrt_star_planner/rrt_star_planner.hpp"

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

namespace nav2_rrt_star_planner
{

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void RRTStarPlanner::configure(
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

  applied_parameters_.clear();
  applyParameters();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&RRTStarPlanner::dynamicParametersCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "RRTStarPlanner configured: step=%.1f max_iter=%d "
    "goal_bias=%.2f goal_thresh=%.2f safety_dist=%.2f cost_weight=%.1f "
    "optimize_iters=%d eta=%.1f informed=%s",
    step_size_, max_iterations_, goal_bias_, goal_threshold_, safety_dist_,
    cost_weight_, max_optimize_iters_, eta_,
    use_informed_sampling_ ? "true" : "false");
}

void RRTStarPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up RRTStarPlanner: %s", name_.c_str());
  rrt_star_.reset();
  dyn_params_handler_.reset();
  applied_parameters_.clear();
}

void RRTStarPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating RRTStarPlanner: %s", name_.c_str());

}

void RRTStarPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating RRTStarPlanner: %s", name_.c_str());
}

// ---------------------------------------------------------------------------
// Plan
// ---------------------------------------------------------------------------

nav_msgs::msg::Path RRTStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal
)
{
  applyParameters();
  nav2_costmap_2d::Costmap2D snapshot;
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
    snapshot = nav2_costmap_2d::Costmap2D(*costmap_);
  }
  const auto * query_map = &snapshot;
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

  // RRT* plan.
  auto t_start = std::chrono::steady_clock::now();
  std::vector<RRTStarNode> raw_path;
  bool success = rrt_star_->planPath(
    start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y,
    query_map, raw_path);
  auto t_end = std::chrono::steady_clock::now();
  double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  if (!success || raw_path.empty()) {
    throw nav2_core::PlannerException(
      "RRTStarPlanner: no valid path found.");
  }

  // Optional prune.
  if (prune_path_) {
    rrt_star_->prunePath(raw_path, query_map);
  }

  // Densify: linear interpolation at costmap resolution.
  nav_msgs::msg::Path plan = linearInterpolation(raw_path, query_map->getResolution());

  // Set header.
  plan.header.stamp = clock_->now();
  plan.header.frame_id = global_frame_;

  RCLCPP_INFO(logger_, "RRTStarPlanner: found path with %ld points "
    "(raw=%ld, prune=%d) plan=%.1f ms",
    plan.poses.size(), raw_path.size(), prune_path_, elapsed_ms);

  return plan;
}

// ---------------------------------------------------------------------------
// Linear interpolation (same pattern as ThetaStar)
// ---------------------------------------------------------------------------

nav_msgs::msg::Path RRTStarPlanner::linearInterpolation(
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

std::vector<rclcpp::Parameter> RRTStarPlanner::validatedParameters(
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

void RRTStarPlanner::applyParameters()
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

}  // namespace nav2_rrt_star_planner

// ---------------------------------------------------------------------------
// Dynamic parameters callback
// ---------------------------------------------------------------------------

rcl_interfaces::msg::SetParametersResult
nav2_rrt_star_planner::RRTStarPlanner::dynamicParametersCallback(
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
  nav2_rrt_star_planner::RRTStarPlanner,
  nav2_core::GlobalPlanner)
