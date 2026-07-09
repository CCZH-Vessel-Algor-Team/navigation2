#ifndef NAV2_COLREGS_ALOS_CONTROLLER__ALOS_CONTROLLER_HPP_
#define NAV2_COLREGS_ALOS_CONTROLLER__ALOS_CONTROLLER_HPP_

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

namespace nav2_colregs_alos_controller
{

/**
 * @class ALOSController
 * @brief Adaptive Line-of-Sight (ALOS) guidance controller with sideslip
 *        compensation for marine USV COLREGS.
 *
 * Unlike basic LOS which computes the target angle as atan2(y,x) toward a
 * single lookahead point, ALOS decomposes the path into a local tangent
 * direction (pi_h), a cross-track error (y_e), and an adaptive sideslip
 * estimate (beta_hat).  The desired heading is:
 *
 *   psi_d = pi_h - beta_hat - atan(y_e / Delta)
 *
 * The sideslip estimate is updated each control cycle using Fossen's
 * adaptive law:
 *
 *   dot_beta = gamma * Delta * y_e / sqrt(Delta^2 + y_e^2)
 *
 * This eliminates the steady-state cross-track error caused by currents,
 * wind, or hull asymmetry — the key advantage of ALOS over basic LOS.
 *
 * A segmentless path model is used: the closest point on the dense
 * transformed_plan and a point advanced by Delta along the path replace
 * the classic waypoint-segment tracking, avoiding the complexity of
 * waypoint state machines.
 */
class ALOSController : public nav2_core::Controller
{
public:
  ALOSController() = default;
  ~ALOSController() = default;

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

  /**
   * @brief Delegate the incoming global plan to internal PathHandler.
   *        Resets the sideslip estimate if configured.
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Return the index of the closest path point (Euclidean distance)
   *        in the transformed plan (base_link frame, robot at origin).
   */
  size_t findClosestPointIndex(const nav_msgs::msg::Path & transformed_plan);

  /**
   * @brief Return the first point on the transformed plan whose accumulated
   *        Euclidean distance from start_idx exceeds forward_dist.
   *        Falls back to the last point if none qualifies.
   */
  geometry_msgs::msg::Point findForwardPoint(
    const nav_msgs::msg::Path & transformed_plan,
    size_t start_idx,
    double forward_dist);

  double computeAngularVelocity(
    double angle_error,
    const geometry_msgs::msg::Twist & speed);

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
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>>
    closest_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> plan_pub_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("ALOSController")};

  double desired_linear_vel_{0.5};
  double max_linear_accel_{1.0};
  double max_angular_vel_{1.8};
  double max_angular_accel_{3.2};
  double forward_dist_{2.0};
  double gamma_{0.0006};
  double beta_hat0_{0.0};
  double beta_hat_{0.0};
  bool reset_beta_on_new_path_{true};
  double max_robot_pose_search_dist_{10.0};
  double goal_dist_tol_{0.25};
  double control_duration_{0.05};
  double max_angle_for_motion_{0.3};
  bool debug_log_enabled_{false};
};

}  // namespace nav2_colregs_alos_controller

#endif  // NAV2_COLREGS_ALOS_CONTROLLER__ALOS_CONTROLLER_HPP_
