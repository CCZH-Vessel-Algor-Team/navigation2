#ifndef NAV2_COLREGS_VO_SKELETON_PLANNER__VO_SKELETON_PLANNER_HPP_
#define NAV2_COLREGS_VO_SKELETON_PLANNER__VO_SKELETON_PLANNER_HPP_

#include <cmath>
#include <memory>
#include <string>
#include <string>

#include "nav2_core/exceptions.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_colregs_msgs/srv/get_avoidance_point.hpp"
#include "nav2_colregs_msgs/srv/get_barrier_lines.hpp"
#include "nav2_colregs_vo_skeleton_planner/skeleton_planner.hpp"
#include "nav2_colregs_vo_skeleton_planner/skeleton_space.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_colregs_vo_skeleton_planner
{

class VOSkeletonPlanner : public nav2_core::GlobalPlanner
{
public:
  VOSkeletonPlanner();
  ~VOSkeletonPlanner();

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
    const std::vector<Pt> & raw_path, double resolution,
    const std::string & frame);

  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("VOSkeletonPlanner")};
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_ = nullptr;

  SkeletonConfig config_;
  double safety_dist_ = 1.5;
  double cost_weight_ = 0.3;
  std::unique_ptr<Space> space_;
  std::unique_ptr<SkeletonPlanner> planner_;
  Pt planner_goal_{0.0, 0.0};
  bool has_planner_goal_ = false;
  uint64_t world_revision_ = 0;

  rclcpp::Client<nav2_colregs_msgs::srv::GetAvoidancePoint>::SharedPtr
    avoidance_client_;
  rclcpp::Client<nav2_colregs_msgs::srv::GetBarrierLines>::SharedPtr
    barrier_client_;
};

}  // namespace nav2_colregs_vo_skeleton_planner

#endif  // NAV2_COLREGS_VO_SKELETON_PLANNER__VO_SKELETON_PLANNER_HPP_
