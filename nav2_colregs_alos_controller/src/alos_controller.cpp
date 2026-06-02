#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <utility>

#include "nav2_colregs_alos_controller/alos_controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_colregs_alos_controller
{

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void ALOSController::configure(
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

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_accel", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.8));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(3.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".forward_dist", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".gamma", rclcpp::ParameterValue(0.0006));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".beta_hat0", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".reset_beta_on_new_path", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angle_for_motion", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_robot_pose_search_dist", rclcpp::ParameterValue(10.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".debug_log_enabled", rclcpp::ParameterValue(false));

  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_linear_accel", max_linear_accel_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
  node->get_parameter(plugin_name_ + ".forward_dist", forward_dist_);
  node->get_parameter(plugin_name_ + ".gamma", gamma_);
  node->get_parameter(plugin_name_ + ".beta_hat0", beta_hat0_);
  node->get_parameter(plugin_name_ + ".reset_beta_on_new_path", reset_beta_on_new_path_);
  node->get_parameter(plugin_name_ + ".max_angle_for_motion", max_angle_for_motion_);
  node->get_parameter(plugin_name_ + ".max_robot_pose_search_dist",
                      max_robot_pose_search_dist_);
  node->get_parameter(plugin_name_ + ".debug_log_enabled", debug_log_enabled_);

  beta_hat_ = beta_hat0_;

  double controller_frequency = 20.0;
  node->get_parameter("controller_frequency", controller_frequency);
  control_duration_ = 1.0 / controller_frequency;

  path_handler_ = std::make_unique<nav2_regulated_pure_pursuit_controller::PathHandler>(
    tf2::durationFromSec(costmap_ros_->getTransformTolerance()),
    tf_, costmap_ros_);

  collision_checker_ = std::make_unique<
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(
    costmap_);

  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
    "lookahead_point", 1);
  closest_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
    "closest_point", 1);
  plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "received_global_plan", 1);
}

void ALOSController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up ALOSController: %s", plugin_name_.c_str());
  carrot_pub_.reset();
  closest_pub_.reset();
  plan_pub_.reset();
  path_handler_.reset();
  collision_checker_.reset();
}

void ALOSController::activate()
{
  RCLCPP_INFO(logger_, "Activating ALOSController: %s", plugin_name_.c_str());
  carrot_pub_->on_activate();
  closest_pub_->on_activate();
  plan_pub_->on_activate();
}

void ALOSController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating ALOSController: %s", plugin_name_.c_str());
  carrot_pub_->on_deactivate();
  closest_pub_->on_deactivate();
  plan_pub_->on_deactivate();
}

void ALOSController::setPlan(const nav_msgs::msg::Path & path)
{
  path_handler_->setPlan(path);
  if (reset_beta_on_new_path_) {
    beta_hat_ = beta_hat0_;
  }
}

void ALOSController::setSpeedLimit(const double & speed_limit, const bool & percentage)
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

geometry_msgs::msg::TwistStamped ALOSController::computeVelocityCommands(
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

  // 3 — ALOS: find closest point and forward point on the dense path.
  size_t closest_idx = findClosestPointIndex(transformed_plan);
  auto P_c = transformed_plan.poses[closest_idx].pose.position;
  auto P_f = findForwardPoint(transformed_plan, closest_idx, forward_dist_);

  // Publish debug: lookahead = P_f, closest = P_c
  auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  carrot_msg->header.frame_id = costmap_ros_->getBaseFrameID();
  carrot_msg->header.stamp = pose.header.stamp;
  carrot_msg->point.x = P_f.x;
  carrot_msg->point.y = P_f.y;
  carrot_pub_->publish(std::move(carrot_msg));

  auto closest_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  closest_msg->header.frame_id = costmap_ros_->getBaseFrameID();
  closest_msg->header.stamp = pose.header.stamp;
  closest_msg->point.x = P_c.x;
  closest_msg->point.y = P_c.y;
  closest_pub_->publish(std::move(closest_msg));

  // 4 — ALOS heading computation (segmentless, using P_c and P_f).
  //
  //     Based on Fossen 2023:  psi_d = pi_h - beta_hat - atan(y_e / Delta)
  //
  //     where  pi_h   = local path tangent angle (P_c → P_f),
  //            y_e    = signed cross-track error (positive = left in FLU base_link),
  //            beta_hat = adaptive sideslip estimate.
  //
  //     Robot at origin in base_link: dx = -P_c.x, dy = -P_c.y.
  //     y_e = -sin(pi_h)*dx + cos(pi_h)*dy
  //         = -sin(pi_h)*(-P_c.x) + cos(pi_h)*(-P_c.y)
  //         = sin(pi_h)*P_c.x - cos(pi_h)*P_c.y
  double pi_h = std::atan2(P_f.y - P_c.y, P_f.x - P_c.x);
  double y_e = std::sin(pi_h) * P_c.x - std::cos(pi_h) * P_c.y;
  double target_angle = pi_h - beta_hat_ - std::atan(y_e / forward_dist_);
  double angle_error = target_angle;   // robot yaw = 0 in base_link frame

  // Update sideslip estimate for the NEXT control cycle.
  //  dot_beta = gamma * Delta * y_e / sqrt(Delta^2 + y_e^2)  (Fossen 2023, Ch. 10)
  double denom = std::sqrt(forward_dist_ * forward_dist_ + y_e * y_e);
  beta_hat_ += gamma_ * forward_dist_ * y_e / denom * control_duration_;

  // 5 — Angular velocity with trapezoidal profile.
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.angular.z = computeAngularVelocity(angle_error, speed);

  // 5b — Turn-in-place gate.
  if (max_angle_for_motion_ > 0.0 &&
      std::fabs(angle_error) > max_angle_for_motion_)
  {
    return cmd_vel;
  }

  // 6 — Linear velocity with trapezoidal profile.
  //     is_goal_point: true when P_f has reached or passed the final goal
  //     (either closest_idx is already at end of plan, or forward search
  //     fell back to the last point).
  bool is_goal_point = (closest_idx >= transformed_plan.poses.size() - 1 ||
                        P_f == transformed_plan.poses.back().pose.position);
  double dist_to_goal = std::hypot(P_f.x, P_f.y);
  cmd_vel.twist.linear.x = computeLinearVelocity(speed, is_goal_point, dist_to_goal);

  // 7 — Footprint collision check.
  double footprint_cost = collision_checker_->footprintCostAtPose(
    pose.pose.position.x, pose.pose.position.y,
    tf2::getYaw(pose.pose.orientation),
    costmap_ros_->getRobotFootprint());

  if (footprint_cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE)) {
    throw nav2_core::NoValidControl("ALOSController detected collision ahead!");
  }

  // 8 — Optional CSV debug log.
  if (debug_log_enabled_) {
    static auto t_start = std::chrono::steady_clock::now();
    static bool header_written = false;
    auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - t_start).count();

    std::ofstream f("/tmp/alos_debug.csv", std::ios::app);
    if (!header_written) {
      f << "t,robot_yaw,y_e,pi_h,beta_hat,target_angle,angle_error,"
        << "current_omega,omega_cmd,current_vel,linear_cmd,"
        << "is_goal,dist_to_goal,Pc_x,Pc_y,Pf_x,Pf_y\n";
      header_written = true;
    }
    f << elapsed << ","
      << tf2::getYaw(pose.pose.orientation) << ","
      << y_e << "," << pi_h << "," << beta_hat_ << ","
      << target_angle << "," << angle_error << ","
      << speed.angular.z << "," << cmd_vel.twist.angular.z << ","
      << speed.linear.x << "," << cmd_vel.twist.linear.x << ","
      << (is_goal_point ? 1 : 0) << "," << dist_to_goal << ","
      << P_c.x << "," << P_c.y << "," << P_f.x << "," << P_f.y << "\n";
    f.flush();
  }

  return cmd_vel;
}

// ---------------------------------------------------------------------------
// Path helpers
// ---------------------------------------------------------------------------

size_t ALOSController::findClosestPointIndex(
  const nav_msgs::msg::Path & transformed_plan)
{
  const auto & poses = transformed_plan.poses;
  auto nearest = std::min_element(
    poses.begin(), poses.end(),
    [](const auto & a, const auto & b) {
      return std::hypot(a.pose.position.x, a.pose.position.y) <
             std::hypot(b.pose.position.x, b.pose.position.y);
    });
  return std::distance(poses.begin(), nearest);
}

geometry_msgs::msg::Point ALOSController::findForwardPoint(
  const nav_msgs::msg::Path & transformed_plan,
  size_t start_idx,
  double forward_dist)
{
  const auto & poses = transformed_plan.poses;
  double accumulated = 0.0;
  geometry_msgs::msg::Point prev = poses[start_idx].pose.position;

  for (size_t i = start_idx + 1; i < poses.size(); i++) {
    const auto & pt = poses[i].pose.position;
    double seg_len = std::hypot(pt.x - prev.x, pt.y - prev.y);
    accumulated += seg_len;
    if (accumulated >= forward_dist) {
      return pt;
    }
    prev = pt;
  }
  // Fallback: path too short for forward_dist — return the goal point.
  // pi_h will still be computable from (P_c → goal), and is_goal_point will
  // trigger the deceleration phase.
  return poses.back().pose.position;
}

// ---------------------------------------------------------------------------
// Trapezoidal angular velocity profile.
// ---------------------------------------------------------------------------

double ALOSController::computeAngularVelocity(
  double angle_error,
  const geometry_msgs::msg::Twist & speed)
{
  const double current_omega = speed.angular.z;
  const double sign = (angle_error > 0.0) ? 1.0 : -1.0;
  const double target_omega = sign * max_angular_vel_;
  const double dt = control_duration_;

  double min_feasible = current_omega - max_angular_accel_ * dt;
  double max_feasible = current_omega + max_angular_accel_ * dt;
  double omega_cmd = std::clamp(target_omega, min_feasible, max_feasible);

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
// ---------------------------------------------------------------------------

double ALOSController::computeLinearVelocity(
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

}  // namespace nav2_colregs_alos_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_colregs_alos_controller::ALOSController,
  nav2_core::Controller)
