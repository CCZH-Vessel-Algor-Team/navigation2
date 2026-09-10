#include "nav2_skeleton_planner/skeleton_rrt_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_skeleton_planner
{

using nav2_colregs_vo_skeleton_planner::Pt;
using nav2_colregs_vo_skeleton_planner::PlanStats;
using nav2_colregs_vo_skeleton_planner::SkeletonConfig;
using nav2_colregs_vo_skeleton_planner::SkeletonPlanner;
using nav2_colregs_vo_skeleton_planner::Space;

SkeletonRRTPlanner::SkeletonRRTPlanner()
{
}

SkeletonRRTPlanner::~SkeletonRRTPlanner()
{
}

void SkeletonRRTPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_node_ = parent;
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  auto node = parent_node_.lock();
  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".step_size", rclcpp::ParameterValue(4.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal_bias", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".eta", rclcpp::ParameterValue(50.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".node_limit", rclcpp::ParameterValue(1024));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_limit", rclcpp::ParameterValue(256));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".near_limit", rclcpp::ParameterValue(16));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".connector_limit", rclcpp::ParameterValue(128));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".recovery_near_limit", rclcpp::ParameterValue(16));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".global_iterations", rclcpp::ParameterValue(2400));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".local_iterations", rclcpp::ParameterValue(600));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".refine_iterations", rclcpp::ParameterValue(200));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_work", rclcpp::ParameterValue(12000000));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".time_limit", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal_tolerance", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_recovery", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_skip", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".reuse_iterations", rclcpp::ParameterValue(64));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".switch_margin", rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".prune_period", rclcpp::ParameterValue(10));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".safety_dist", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cost_weight", rclcpp::ParameterValue(0.3));

  node->get_parameter(name_ + ".step_size", config_.step);
  node->get_parameter(name_ + ".goal_bias", config_.goal_bias);
  node->get_parameter(name_ + ".eta", config_.eta);
  node->get_parameter(name_ + ".node_limit", config_.node_limit);
  node->get_parameter(name_ + ".path_limit", config_.path_limit);
  node->get_parameter(name_ + ".near_limit", config_.near_limit);
  node->get_parameter(name_ + ".connector_limit", config_.connector_limit);
  node->get_parameter(name_ + ".recovery_near_limit", config_.recovery_near_limit);
  node->get_parameter(name_ + ".global_iterations", config_.global_iterations);
  node->get_parameter(name_ + ".local_iterations", config_.local_iterations);
  node->get_parameter(name_ + ".refine_iterations", config_.refine_iterations);
  node->get_parameter(name_ + ".max_work", config_.max_work);
  node->get_parameter(name_ + ".time_limit", config_.time_limit);
  node->get_parameter(name_ + ".goal_tolerance", config_.goal_tolerance);
  node->get_parameter(name_ + ".allow_recovery", config_.allow_recovery);
  node->get_parameter(name_ + ".allow_skip", config_.allow_skip);
  node->get_parameter(name_ + ".reuse_iterations", config_.reuse_iterations);
  node->get_parameter(name_ + ".switch_margin", config_.switch_margin);
  node->get_parameter(name_ + ".prune_period", config_.prune_period);
  node->get_parameter(name_ + ".safety_dist", safety_dist_);
  node->get_parameter(name_ + ".cost_weight", cost_weight_);
  config_.validate();
  if (!std::isfinite(safety_dist_) || safety_dist_ < 0.0) {
    throw std::invalid_argument("safety_dist must be finite and nonnegative");
  }
  if (!std::isfinite(cost_weight_) || cost_weight_ < 0.0) {
    throw std::invalid_argument("cost_weight must be finite and nonnegative");
  }

  space_ = std::make_unique<Space>(costmap_, safety_dist_, cost_weight_);
  planner_.reset();
  has_planner_goal_ = false;

  RCLCPP_INFO(logger_,
    "SkeletonRRTPlanner configured: step=%.1f eta=%.1f nodes=%d paths=%d "
    "global=%d local=%d refine=%d reuse=%d margin=%.2f",
    config_.step, config_.eta, config_.node_limit, config_.path_limit,
    config_.global_iterations, config_.local_iterations,
    config_.refine_iterations, config_.reuse_iterations, config_.switch_margin);
}

void SkeletonRRTPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up SkeletonRRTPlanner: %s", name_.c_str());
  planner_.reset();
  space_.reset();
}

void SkeletonRRTPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating SkeletonRRTPlanner: %s", name_.c_str());
}

void SkeletonRRTPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating SkeletonRRTPlanner: %s", name_.c_str());
}

nav_msgs::msg::Path SkeletonRRTPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  // One stable costmap view per query (see the VO wrapper, R2-03).
  nav2_costmap_2d::Costmap2D snapshot;
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *(costmap_->getMutex()));
    snapshot = nav2_costmap_2d::Costmap2D(*costmap_);
  }
  const nav2_costmap_2d::Costmap2D * query_map = &snapshot;
  space_->updateCostmap(query_map);

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
  if (query_map->getCost(start_mx, start_my) >=
    nav2_costmap_2d::LETHAL_OBSTACLE)
  {
    throw nav2_core::PlannerException("Start is occupied.");
  }
  if (query_map->getCost(goal_mx, goal_my) >=
    nav2_costmap_2d::LETHAL_OBSTACLE)
  {
    throw nav2_core::PlannerException("Goal is occupied.");
  }

  const Pt final_goal(goal.pose.position.x, goal.pose.position.y);
  const Pt query(start.pose.position.x, start.pose.position.y);

  // Root validation: a goal change resets the persistent planner.
  if (!planner_ || !has_planner_goal_ || planner_->goalChanged(final_goal)) {
    planner_ = std::make_unique<SkeletonPlanner>(
      space_.get(), final_goal, config_, /*seed=*/7ULL);
    has_planner_goal_ = true;
    RCLCPP_INFO(logger_,
      "SkeletonRRTPlanner: new goal-rooted planner for (%.2f, %.2f)",
      final_goal.first, final_goal.second);
  }

  world_revision_++;
  planner_->beginQuery(world_revision_);
  space_->setBarriers({});  // no barriers in the standalone planner

  std::vector<Pt> path;
  const PlanStats stats = planner_->replan(query, path);
  if (path.empty()) {
    RCLCPP_WARN(logger_,
      "SkeletonRRTPlanner: no path (status=%s mode=%s work=%ld nodes=%d)",
      stats.status.c_str(), stats.mode.c_str(),
      static_cast<long>(stats.work), stats.nodes);
    throw nav2_core::PlannerException("SkeletonRRTPlanner: no valid path found.");
  }
  RCLCPP_INFO(logger_,
    "SkeletonRRTPlanner: mode=%s status=%s nodes=%d skips=%d gen=%d "
    "pruned=%d t=%.3fs cost=%.1f",
    stats.mode.c_str(), stats.status.c_str(), stats.nodes, stats.skips,
    stats.generation, stats.pruned_tree_nodes, stats.elapsed_s,
    stats.adopted_cost);

  nav_msgs::msg::Path plan = linearInterpolation(
    path, query_map->getResolution());
  plan.header.stamp = parent_node_.lock()->now();
  plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  if (!plan.poses.empty()) {
    plan.poses.back().pose.orientation = goal.pose.orientation;
  }
  return plan;
}

nav_msgs::msg::Path SkeletonRRTPlanner::linearInterpolation(
  const std::vector<Pt> & raw_path, double resolution)
{
  nav_msgs::msg::Path plan;
  if (raw_path.empty()) {
    return plan;
  }
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.orientation.w = 1.0;
  pose.header.frame_id = "map";
  auto push = [&](double x, double y) {
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      plan.poses.push_back(pose);
    };

  push(raw_path.front().first, raw_path.front().second);
  for (size_t i = 1; i < raw_path.size(); ++i) {
    const auto & a = raw_path[i - 1];
    const auto & b = raw_path[i];
    const double dist = std::hypot(b.first - a.first, b.second - a.second);
    const int intervals = std::max(1, static_cast<int>(dist / resolution));
    for (int k = 1; k <= intervals; ++k) {
      const double t = static_cast<double>(k) / intervals;
      push(a.first + t * (b.first - a.first),
        a.second + t * (b.second - a.second));
    }
  }
  return plan;
}

}  // namespace nav2_skeleton_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_skeleton_planner::SkeletonRRTPlanner, nav2_core::GlobalPlanner)
