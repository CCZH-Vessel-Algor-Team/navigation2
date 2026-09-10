#ifndef NAV2_COLREGS_VO_RRT_STAR_PLANNER__VO_RRT_STAR_PLANNER_HPP_
#define NAV2_COLREGS_VO_RRT_STAR_PLANNER__VO_RRT_STAR_PLANNER_HPP_

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "nav2_core/global_planner.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_colregs_vo_rrt_star_planner/rrt_star.hpp"
#include "nav2_colregs_msgs/srv/get_avoidance_point.hpp"
#include "nav2_colregs_msgs/srv/get_barrier_lines.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_colregs_vo_rrt_star_planner
{

class VORRTStarPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

  /// Testable pure helper: should this start pose use the COLREGS service
  /// chain? Only segments anchored at the live robot pose produce
  /// physically meaningful VO decisions; preview segments (NavigateThroughPoses
  /// calls with future goal starts) fall back to plain RRT*.
  static bool isColregsAnchored(
    double start_x, double start_y,
    double robot_x, double robot_y,
    double max_dist)
  {
    return std::hypot(start_x - robot_x, start_y - robot_y) <= max_dist;
  }

private:
  static nav_msgs::msg::Path linearInterpolation(
    const std::vector<RRTStarNode> & raw_path,
    double resolution);

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("VORRTStarPlanner")};
  std::string global_frame_, name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  std::unique_ptr<RRTStar> rrt_star_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  double step_size_{1.0};
  int max_iterations_{1000};
  double goal_bias_{0.1};
  double goal_threshold_{0.5};
  double safety_dist_{0.3};
  double cost_weight_{1.0};
  int max_optimize_iters_{200};
  double eta_{1.1};
  double tolerance_{0.5};
  bool prune_path_{true};
  bool use_informed_sampling_{true};
  double colregs_anchor_max_dist_{3.0};

  rclcpp::Client<nav2_colregs_msgs::srv::GetAvoidancePoint>::SharedPtr avoidance_client_;
  rclcpp::Client<nav2_colregs_msgs::srv::GetBarrierLines>::SharedPtr barrier_client_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    dyn_params_handler_;
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace nav2_colregs_vo_rrt_star_planner

#endif  // NAV2_COLREGS_VO_RRT_STAR_PLANNER__VO_RRT_STAR_PLANNER_HPP_
