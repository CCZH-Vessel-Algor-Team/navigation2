#ifndef NAV2_COLREGS_LOS_CONTROLLER__LOS_CONTROLLER_HPP_
#define NAV2_COLREGS_LOS_CONTROLLER__LOS_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_regulated_pure_pursuit_controller/path_handler.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"

namespace nav2_colregs_los_controller
{

/**
 * @class LOSController
 * @brief Minimal Line-of-Sight (LOS) guidance controller for marine USV COLREGS.
 *
 * This controller implements a basic LOS algorithm:
 *   1. Transforms the global plan to the robot's base_link frame via PathHandler.
 *   2. Finds a lookahead point along the path at configured distance.
 *   3. Computes target heading as atan2(lookahead_y, lookahead_x).
 *   4. Generates angular velocity with a trapezoidal acceleration profile
 *      (same formula as RotationShimController).
 *   5. Generates linear velocity with a trapezoidal acceleration profile,
 *      switching to deceleration when the lookahead reaches the goal point.
 *   6. Performs a footprint-based collision check against the costmap.
 */
class LOSController : public nav2_core::Controller
{
public:
  LOSController() = default;
  ~LOSController() = default;

  /**
   * @brief Lifecycle configuration: load parameters, create PathHandler,
   *        FootprintCollisionChecker, and debug publishers.
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  /** @brief Delegate the incoming global plan to internal PathHandler. */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Core control loop entry: produce a TwistStamped from current
   *        pose, speed, and goal checker.
   *
   * @param pose   Robot pose in the costmap global frame (odom).
   * @param speed  Current odometry twist.
   * @param goal_checker  Plugin to obtain goal tolerances.
   * @return TwistStamped with computed linear and angular velocities.
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Return the first point on the transformed plan whose Euclidean
   *        distance from the robot (origin in base_link) exceeds lookahead_dist.
   *        Falls back to the last point if none qualifies.
   */
  geometry_msgs::msg::PoseStamped findLookaheadPoint(
    const nav_msgs::msg::Path & transformed_plan,
    double lookahead_dist);

  /**
   * @brief Compute angular velocity with a trapezoidal acceleration profile.
   *
   * Upper bound: clamp to [current ± max_angular_accel·dt].
   * Lower bound: clamp to √(2·max_angular_accel·|angle_error|) to avoid
   * overshooting the target heading.
   */
  double computeAngularVelocity(
    double angle_error,
    const geometry_msgs::msg::Twist & speed);

  /**
   * @brief Compute linear velocity with a trapezoidal acceleration profile.
   *
   * When tracking the goal point (is_goal_point=true), the target velocity
   * is dynamically set to √(2·max_linear_accel·dist_to_goal), naturally
   * forming an acceleration → coast → deceleration curve without an explicit
   * state machine.  When dist_to_goal is large the target exceeds the cruise
   * speed and the clamp allows acceleration; when it shrinks the target
   * drops below the current speed and the clamp enforces deceleration.
   *
   * When not yet at the goal point, the target is desired_linear_vel_ and
   * the same acceleration bounds apply.
   */
  double computeLinearVelocity(
    const geometry_msgs::msg::Twist & speed,
    bool is_goal_point,
    double dist_to_goal);

  mutable std::mutex mutex_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::unique_ptr<nav2_regulated_pure_pursuit_controller::PathHandler> path_handler_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
    collision_checker_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>>
    carrot_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> plan_pub_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("LOSController")};

  double desired_linear_vel_{0.5};
  double max_linear_accel_{1.0};
  double max_angular_vel_{1.8};
  double max_angular_accel_{3.2};
  double lookahead_dist_{0.6};
  double max_robot_pose_search_dist_{10.0};
  double goal_dist_tol_{0.25};
  double control_duration_{0.05};
  double max_angle_for_motion_{0.3};   // 0 = disabled
  bool debug_log_enabled_{false};
};

}  // namespace nav2_colregs_los_controller

#endif  // NAV2_COLREGS_LOS_CONTROLLER__LOS_CONTROLLER_HPP_
