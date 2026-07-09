#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <utility>

#include "nav2_colregs_los_controller/los_controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_colregs_los_controller
{

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void LOSController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::ControllerException("Unable to lock node!");
  }

  node_ = parent;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();

  // Declare and read controller parameters (namespaced under plugin_name_).
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_accel", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.8));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(3.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_robot_pose_search_dist", rclcpp::ParameterValue(10.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angle_for_motion", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".debug_log_enabled", rclcpp::ParameterValue(false));

  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_linear_accel", max_linear_accel_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_robot_pose_search_dist",
                      max_robot_pose_search_dist_);
  node->get_parameter(plugin_name_ + ".max_angle_for_motion", max_angle_for_motion_);
  node->get_parameter(plugin_name_ + ".debug_log_enabled", debug_log_enabled_);

  double controller_frequency = 20.0;
  node->get_parameter("controller_frequency", controller_frequency);
  control_duration_ = 1.0 / controller_frequency;

  // PathHandler: owned locally, not a pluginlib-loaded plugin.
  // Reuses RPP's implementation to transform/trim global plan to base_link.
  path_handler_ = std::make_unique<nav2_regulated_pure_pursuit_controller::PathHandler>(
    tf2::durationFromSec(costmap_ros_->getTransformTolerance()),
    tf_, costmap_ros_);

  // FootprintCollisionChecker: checks robot footprint against costmap cells.
  collision_checker_ = std::make_unique<
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(
    costmap_);

  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
    "lookahead_point", 1);
  plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "received_global_plan", 1);
}

void LOSController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up LOSController: %s", plugin_name_.c_str());
  carrot_pub_.reset();
  plan_pub_.reset();
  path_handler_.reset();
  collision_checker_.reset();
}

void LOSController::activate()
{
  RCLCPP_INFO(logger_, "Activating LOSController: %s", plugin_name_.c_str());
  carrot_pub_->on_activate();
  plan_pub_->on_activate();
}

void LOSController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating LOSController: %s", plugin_name_.c_str());
  carrot_pub_->on_deactivate();
  plan_pub_->on_deactivate();
}

void LOSController::setPlan(const nav_msgs::msg::Path & path)
{
  path_handler_->setPlan(path);
}

void LOSController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (speed_limit <= 0.0) {
    return;
  }

  if (percentage) {
    desired_linear_vel_ = desired_linear_vel_ * speed_limit / 100.0;
  } else {
    desired_linear_vel_ = speed_limit;
  }
}

// ---------------------------------------------------------------------------
// Core control loop
// ---------------------------------------------------------------------------

geometry_msgs::msg::TwistStamped LOSController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // 1 — Obtain goal tolerances from goal checker plugin.
  geometry_msgs::msg::Pose pose_tol;
  geometry_msgs::msg::Twist vel_tol;
  if (goal_checker->getTolerances(pose_tol, vel_tol)) {
    goal_dist_tol_ = pose_tol.position.x;
  }

  // 2 — Transform global plan (map frame) → robot local frame (base_link).
  auto transformed_plan = path_handler_->transformGlobalPlan(
    pose, max_robot_pose_search_dist_);
  plan_pub_->publish(transformed_plan);

  // 3 — Find lookahead point: first point on transformed_plan whose distance
  //     from the origin (robot centre in base_link) exceeds lookahead_dist_.
  auto lookahead_pt = findLookaheadPoint(transformed_plan, lookahead_dist_);
  bool is_goal_point = (lookahead_pt.pose.position.x ==
                        transformed_plan.poses.back().pose.position.x &&
                        lookahead_pt.pose.position.y ==
                        transformed_plan.poses.back().pose.position.y);

  auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  carrot_msg->header.frame_id = costmap_ros_->getBaseFrameID();
  carrot_msg->header.stamp = pose.header.stamp;
  carrot_msg->point.x = lookahead_pt.pose.position.x;
  carrot_msg->point.y = lookahead_pt.pose.position.y;
  carrot_pub_->publish(std::move(carrot_msg));

  // 4 — LOS target heading (base_link frame: robot yaw ≡ 0).
  double target_angle = std::atan2(
    lookahead_pt.pose.position.y,
    lookahead_pt.pose.position.x);
  double angle_error = target_angle;

  // 5 — Angular velocity with trapezoidal profile.
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.angular.z = computeAngularVelocity(angle_error, speed);

  // 5b — Turn-in-place gate: if the heading error exceeds the configured
  //      threshold, keep linear velocity at zero until facing the target.
  //      max_angle_for_motion_ = 0 disables this feature entirely.
  if (max_angle_for_motion_ > 0.0 &&
      std::fabs(angle_error) > max_angle_for_motion_)
  {
    return cmd_vel;
  }

  // 6 — Linear velocity with trapezoidal profile.
  double dist_to_goal = std::hypot(
    lookahead_pt.pose.position.x,
    lookahead_pt.pose.position.y);
  cmd_vel.twist.linear.x = computeLinearVelocity(speed, is_goal_point, dist_to_goal);

  // 7 — Footprint collision check at current pose (rotation-only check).
  double footprint_cost = collision_checker_->footprintCostAtPose(
    pose.pose.position.x, pose.pose.position.y,
    tf2::getYaw(pose.pose.orientation),
    costmap_ros_->getRobotFootprint());

  if (footprint_cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE)) {
    throw nav2_core::NoValidControl("LOSController detected collision ahead!");
  }

  // 8 — Optional CSV debug log for offline trajectory analysis.
  if (debug_log_enabled_) {
    static auto t_start = std::chrono::steady_clock::now();
    static bool header_written = false;
    auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - t_start).count();

    std::ofstream f("/tmp/los_debug.csv", std::ios::app);
    if (!header_written) {
      f << "t,robot_yaw,path_size,"
        << "lookahead_x,lookahead_y,lookahead_dist,"
        << "target_angle,angle_error,"
        << "current_omega,omega_cmd,"
        << "current_vel,linear_cmd,"
        << "is_goal,dist_to_goal,"
        << "first_x,first_y,last_x,last_y\n";
      header_written = true;
    }

    double la_x = lookahead_pt.pose.position.x;
    double la_y = lookahead_pt.pose.position.y;
    double first_x = transformed_plan.poses.front().pose.position.x;
    double first_y = transformed_plan.poses.front().pose.position.y;
    double last_x = transformed_plan.poses.back().pose.position.x;
    double last_y = transformed_plan.poses.back().pose.position.y;

    f << elapsed << ","
      << tf2::getYaw(pose.pose.orientation) << ","
      << transformed_plan.poses.size() << ","
      << la_x << "," << la_y << ","
      << std::hypot(la_x, la_y) << ","
      << target_angle << "," << angle_error << ","
      << speed.angular.z << "," << cmd_vel.twist.angular.z << ","
      << speed.linear.x << "," << cmd_vel.twist.linear.x << ","
      << (is_goal_point ? 1 : 0) << "," << dist_to_goal << ","
      << first_x << "," << first_y << ","
      << last_x << "," << last_y << "\n";

    f.flush();
  }

  return cmd_vel;
}

// ---------------------------------------------------------------------------
// Lookahead point — first path point beyond the lookahead distance.
// ---------------------------------------------------------------------------

geometry_msgs::msg::PoseStamped LOSController::findLookaheadPoint(
  const nav_msgs::msg::Path & transformed_plan,
  double lookahead_dist)
{
  for (const auto & ps : transformed_plan.poses) {
    if (std::hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist) {
      return ps;
    }
  }
  return transformed_plan.poses.back();
}

// ---------------------------------------------------------------------------
// Trapezoidal angular velocity profile.
// Reference: RotationShimController::computeRotateToHeadingCommand.
// ---------------------------------------------------------------------------

double LOSController::computeAngularVelocity(
  double angle_error,
  const geometry_msgs::msg::Twist & speed)
{
  const double current_omega = speed.angular.z;
  const double sign = (angle_error > 0.0) ? 1.0 : -1.0;
  const double target_omega = sign * max_angular_vel_;
  const double dt = control_duration_;

  // Upper bound: acceleration-limited from current speed.
  double min_feasible = current_omega - max_angular_accel_ * dt;
  double max_feasible = current_omega + max_angular_accel_ * dt;
  double omega_cmd = std::clamp(target_omega, min_feasible, max_feasible);

  // Lower bound: v_max = sqrt(2·a·d) — can stop before overshooting the target.
  double max_vel_to_stop =
    std::sqrt(2.0 * max_angular_accel_ * std::fabs(angle_error));
  if (std::fabs(omega_cmd) > max_vel_to_stop) {
    omega_cmd = sign * max_vel_to_stop;
  }

  if (std::fabs(angle_error) < 1e-6) {
    omega_cmd = 0.0;
  }

  return omega_cmd;
}

// ---------------------------------------------------------------------------
// Trapezoidal linear velocity profile.
//
//   Cruise: target = desired_linear_vel_  (accelerate toward cruise speed).
//   Goal  : target = √(2·a·d)             (decelerate to stop at goal).
//
// In both phases the command is clamped by the current velocity ± a·dt,
// naturally forming an acceleration → cruise → deceleration curve without
// an explicit state machine.
// ---------------------------------------------------------------------------

double LOSController::computeLinearVelocity(
  const geometry_msgs::msg::Twist & speed,
  bool is_goal_point,
  double dist_to_goal)
{
  const double current_vel = speed.linear.x;
  const double dt = control_duration_;

  if (is_goal_point) {
    double v_target = std::sqrt(2.0 * max_linear_accel_ * dist_to_goal);

    if (dist_to_goal < goal_dist_tol_) {
      v_target = 0.0;
    }

    double sign = (current_vel >= 0.0) ? 1.0 : -1.0;
    double v_cmd = std::clamp(sign * v_target,
      current_vel - max_linear_accel_ * dt,
      current_vel + max_linear_accel_ * dt);
    return v_cmd;
  }

  double v_cmd = std::clamp(desired_linear_vel_,
    current_vel - max_linear_accel_ * dt,
    current_vel + max_linear_accel_ * dt);
  return v_cmd;
}

}  // namespace nav2_colregs_los_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_colregs_los_controller::LOSController,
  nav2_core::Controller)
