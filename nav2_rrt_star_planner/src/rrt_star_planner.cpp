#include "nav2_rrt_star_planner/rrt_star_planner.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <memory>
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
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".tolerance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".prune_path", rclcpp::ParameterValue(true));

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

  rrt_star_ = std::make_unique<RRTStar>(
    step_size_, max_iterations_, goal_bias_, goal_threshold_,
    safety_dist_, cost_weight_, max_optimize_iters_, eta_);

  RCLCPP_INFO(logger_, "RRTStarPlanner configured: step=%.1f max_iter=%d "
    "goal_bias=%.2f goal_thresh=%.2f safety_dist=%.2f cost_weight=%.1f "
    "optimize_iters=%d eta=%.1f",
    step_size_, max_iterations_, goal_bias_, goal_threshold_, safety_dist_,
    cost_weight_, max_optimize_iters_, eta_);
}

void RRTStarPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up RRTStarPlanner: %s", name_.c_str());
  rrt_star_.reset();
}

void RRTStarPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating RRTStarPlanner: %s", name_.c_str());

  auto node = parent_node_.lock();
  if (node) {
    dyn_params_handler_ = node->add_on_set_parameters_callback(
      std::bind(
        &RRTStarPlanner::dynamicParametersCallback, this,
        std::placeholders::_1));
  }
}

void RRTStarPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating RRTStarPlanner: %s", name_.c_str());
  dyn_params_handler_.reset();
}

// ---------------------------------------------------------------------------
// Plan
// ---------------------------------------------------------------------------

nav_msgs::msg::Path RRTStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  // Validate start/goal are within costmap bounds.
  unsigned int start_mx, start_my, goal_mx, goal_my;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y,
                            start_mx, start_my))
  {
    throw nav2_core::StartOutsideMapBounds("Start is outside the map bounds.");
  }
  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y,
                            goal_mx, goal_my))
  {
    throw nav2_core::GoalOutsideMapBounds("Goal is outside the map bounds.");
  }

  // Start/goal occupied check.
  if (costmap_->getCost(start_mx, start_my) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
    throw nav2_core::StartOccupied("Start is occupied.");
  }
  if (costmap_->getCost(goal_mx, goal_my) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
    throw nav2_core::GoalOccupied("Goal is occupied.");
  }

  // RRT* plan.
  auto t_start = std::chrono::steady_clock::now();
  std::vector<RRTStarNode> raw_path;
  bool success = rrt_star_->planPath(
    start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y,
    costmap_, cancel_checker, raw_path);
  auto t_end = std::chrono::steady_clock::now();
  double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  if (!success || raw_path.empty()) {
    throw nav2_core::NoValidPathCouldBeFound(
      "RRTStarPlanner: no valid path found.");
  }

  // Optional prune.
  if (prune_path_) {
    rrt_star_->prunePath(raw_path, costmap_);
  }

  // Densify: linear interpolation at costmap resolution.
  nav_msgs::msg::Path plan = linearInterpolation(raw_path, costmap_->getResolution());

  // Set header.
  plan.header.stamp = clock_->now();
  plan.header.frame_id = global_frame_;

  RCLCPP_INFO(logger_, "RRTStarPlanner: found path with %ld points "
    "(raw=%ld, pruned=%ld) in %.1f ms",
    plan.poses.size(), raw_path.size(), (!prune_path_ ? raw_path.size() : raw_path.size()),
    elapsed_ms);

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
  nav2_rrt_star_planner::RRTStarPlanner,
  nav2_core::GlobalPlanner)
