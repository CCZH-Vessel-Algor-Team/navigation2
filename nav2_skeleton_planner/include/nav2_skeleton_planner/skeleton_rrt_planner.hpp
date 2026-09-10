#ifndef NAV2_SKELETON_PLANNER__SKELETON_RRT_PLANNER_HPP_
#define NAV2_SKELETON_PLANNER__SKELETON_RRT_PLANNER_HPP_

#include <memory>
#include <string>

#include "nav2_colregs_vo_skeleton_planner/skeleton_planner.hpp"
#include "nav2_colregs_vo_skeleton_planner/skeleton_space.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_skeleton_planner
{

/**
 * Standalone skeleton RRT* Nav2 global planner.
 *
 * Wraps the persistent goal-rooted core from
 * nav2_colregs_vo_skeleton_planner without the VO service chain: the plan
 * query is the passed-in start pose and no barriers are applied. The
 * wrapper owns one SkeletonPlanner instance per goal (root validation),
 * snapshots the costmap under its mutex for a stable per-query view, and
 * bumps a world revision every query so the dynamic maintenance passes
 * (skeleton revalidation, incremental tree pruning with cost refresh) run
 * against the live map.
 */
class SkeletonRRTPlanner : public nav2_core::GlobalPlanner
{
public:
  SkeletonRRTPlanner();
  ~SkeletonRRTPlanner() override;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  nav_msgs::msg::Path linearInterpolation(
    const std::vector<nav2_colregs_vo_skeleton_planner::Pt> & raw_path,
    double resolution);

  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("SkeletonRRTPlanner")};
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_ = nullptr;

  nav2_colregs_vo_skeleton_planner::SkeletonConfig config_;
  double safety_dist_ = 1.5;
  double cost_weight_ = 0.3;
  std::unique_ptr<nav2_colregs_vo_skeleton_planner::Space> space_;
  std::unique_ptr<nav2_colregs_vo_skeleton_planner::SkeletonPlanner> planner_;
  nav2_colregs_vo_skeleton_planner::Pt planner_goal_{0.0, 0.0};
  bool has_planner_goal_ = false;
  uint64_t world_revision_ = 0;
};

}  // namespace nav2_skeleton_planner

#endif  // NAV2_SKELETON_PLANNER__SKELETON_RRT_PLANNER_HPP_
