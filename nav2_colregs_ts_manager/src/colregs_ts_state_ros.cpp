// Copyright (c) 2026 Vector Wang
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_colregs_ts_manager/colregs_ts_state_ros.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_colregs_ts_manager
{

namespace
{

std::string uuidToString(const uint8_t * data)
{
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  for (size_t i = 0; i < 16; ++i) {
    if (i == 4 || i == 6 || i == 8 || i == 10) {oss << '-';}
    oss << std::setw(2) << static_cast<int>(data[i]);
  }
  return oss.str();
}

}  // namespace

ColregsTsStateROS::ColregsTsStateROS(
  const std::string & name, const std::string & parent_namespace,
  const bool & use_sim_time)
: nav2_util::LifecycleNode(name, "",
    // NodeOption arguments take precedence over the ones provided on the
    // command line; use this to place the node in the parent namespace.
    rclcpp::NodeOptions().arguments({
    "--ros-args", "-r", std::string("__ns:=") +
    (parent_namespace.empty() || parent_namespace == "/" ? "/" : parent_namespace),
    "--ros-args", "-r", name + ":" + std::string("__node:=") + name,
    "--ros-args", "-p", "use_sim_time:=" + std::string(use_sim_time ? "true" : "false"),
  }))
{
  declare_parameter("frequency", 10.0);
  declare_parameter("ts_timeout", 3.0);
  declare_parameter("tcpa_horizon", 10.0);
  declare_parameter("safety_factor", 1.1);
  declare_parameter("os_radius", 0.3);
  declare_parameter("barrier_ray_length", 999.0);
  declare_parameter("global_frame", std::string("map"));
  declare_parameter("robot_base_frame", std::string("base_link"));
  declare_parameter("odom_topic", std::string("odom"));
  declare_parameter("tracked_ship_topic", std::string("/tracked_ship"));
}

bool ColregsTsStateROS::loadAndValidateParameters()
{
  frequency_ = get_parameter("frequency").as_double();
  core_params_.ts_timeout = get_parameter("ts_timeout").as_double();
  core_params_.tcpa_horizon = get_parameter("tcpa_horizon").as_double();
  core_params_.safety_factor = get_parameter("safety_factor").as_double();
  core_params_.os_radius = get_parameter("os_radius").as_double();
  core_params_.barrier_ray_length = get_parameter("barrier_ray_length").as_double();
  global_frame_ = get_parameter("global_frame").as_string();
  robot_base_frame_ = get_parameter("robot_base_frame").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  tracked_ship_topic_ = get_parameter("tracked_ship_topic").as_string();

  const bool valid =
    std::isfinite(frequency_) && frequency_ > 0.0 &&
    std::isfinite(core_params_.ts_timeout) && core_params_.ts_timeout > 0.0 &&
    std::isfinite(core_params_.tcpa_horizon) && core_params_.tcpa_horizon > 0.0 &&
    std::isfinite(core_params_.safety_factor) && core_params_.safety_factor > 0.0 &&
    std::isfinite(core_params_.os_radius) && core_params_.os_radius >= 0.0 &&
    std::isfinite(core_params_.barrier_ray_length) &&
    core_params_.barrier_ray_length > 0.0 &&
    !global_frame_.empty() && !robot_base_frame_.empty() &&
    !odom_topic_.empty() && !tracked_ship_topic_.empty();
  if (!valid) {
    RCLCPP_ERROR(get_logger(), "Invalid COLREGS TS state parameters");
  }
  return valid;
}

nav2_util::CallbackReturn ColregsTsStateROS::on_configure(
  const rclcpp_lifecycle::State &)
{
  // Validate parameters before any resource is created so a failed configure
  // never has to tear down timer or TF threads (cleanup/join race avoidance).
  if (!loadAndValidateParameters()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, true);

  ts_sub_ = create_subscription<nav2_colregs_msgs::msg::TrackedShipList>(
    tracked_ship_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&ColregsTsStateROS::trackedShipCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&ColregsTsStateROS::odomCallback, this, std::placeholders::_1));

  cpa_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "cpa_markers", 10);

  RCLCPP_INFO(
    get_logger(),
    "ColregsTsStateROS configured (ts_timeout=%.1fs, topic=%s)",
    core_params_.ts_timeout, tracked_ship_topic_.c_str());
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ColregsTsStateROS::on_activate(
  const rclcpp_lifecycle::State &)
{
  cpa_markers_pub_->on_activate();

  using namespace std::chrono_literals;
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / frequency_));
  timer_ = create_wall_timer(period, std::bind(&ColregsTsStateROS::timerCallback, this));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ColregsTsStateROS::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  timer_.reset();
  cpa_markers_pub_->on_deactivate();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ColregsTsStateROS::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  timer_.reset();
  ts_sub_.reset();
  odom_sub_.reset();
  cpa_markers_pub_.reset();
  tf_listener_.reset();
  tf_.reset();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ts_map_.clear();
    last_odom_.reset();
  }
  return nav2_util::CallbackReturn::SUCCESS;
}

void ColregsTsStateROS::trackedShipCallback(
  nav2_colregs_msgs::msg::TrackedShipList::ConstSharedPtr msg)
{
  const auto stamp = now();

  // Transform and rotate outside the state mutex; only the cache update
  // happens under the lock.
  std::vector<RawTsEntry> entries;
  entries.reserve(msg->ships.size());
  for (const auto & ship : msg->ships) {
    const auto key = uuidToString(ship.target_id.uuid.data());

    RawTsEntry entry;
    entry.target_id = key;
    entry.radius = ship.radius;
    entry.last_seen = stamp;

    geometry_msgs::msg::PoseStamped ts_in, ts_out;
    ts_in.header.frame_id = msg->header.frame_id;
    ts_in.header.stamp = rclcpp::Time(0);
    ts_in.pose = ship.pose;
    try {
      // The reported twist is a finite difference of positions in the message
      // frame, so both pose and velocity rotate by the message→global-frame
      // transform (not by the ship yaw).
      const auto transform = tf_->lookupTransform(
        global_frame_, msg->header.frame_id, tf2::TimePointZero,
        tf2::durationFromSec(1.0));
      tf2::doTransform(ts_in, ts_out, transform);
      entry.x = ts_out.pose.position.x;
      entry.y = ts_out.pose.position.y;
      const double yaw = tf2::getYaw(transform.transform.rotation);
      entry.vx = std::cos(yaw) * ship.twist.linear.x -
        std::sin(yaw) * ship.twist.linear.y;
      entry.vy = std::sin(yaw) * ship.twist.linear.x +
        std::cos(yaw) * ship.twist.linear.y;
    } catch (const tf2::TransformException & ex) {
      // D7: a target ship without a usable transform is skipped entirely
      // instead of falling back to untransformed coordinates.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "ColregsTsStateROS: TF from '%s' to '%s' failed for %s: %s",
        msg->header.frame_id.c_str(), global_frame_.c_str(), key.c_str(), ex.what());
      continue;
    }

    entries.push_back(entry);
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  for (auto & entry : entries) {
    ts_map_[entry.target_id] = entry;
  }
}

void ColregsTsStateROS::odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_odom_ = msg;
}

bool ColregsTsStateROS::getOsPoseAndVelocity(
  double & os_x, double & os_y, double & os_vx, double & os_vy)
{
  geometry_msgs::msg::PoseStamped os_pose;
  os_pose.header.frame_id = robot_base_frame_;
  os_pose.header.stamp = rclcpp::Time(0);
  os_pose.pose.orientation.w = 1.0;
  try {
    os_pose = tf_->transform(os_pose, global_frame_, tf2::durationFromSec(0.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "ColregsTsStateROS: TF lookup %s→%s not available: %s",
      robot_base_frame_.c_str(), global_frame_.c_str(), ex.what());
    return false;
  }

  os_x = os_pose.pose.position.x;
  os_y = os_pose.pose.position.y;
  const double os_yaw = tf2::getYaw(os_pose.pose.orientation);

  nav_msgs::msg::Odometry::ConstSharedPtr odom;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    odom = last_odom_;
  }
  if (odom) {
    const double body_vx = odom->twist.twist.linear.x;
    const double body_vy = odom->twist.twist.linear.y;
    os_vx = std::cos(os_yaw) * body_vx - std::sin(os_yaw) * body_vy;
    os_vy = std::sin(os_yaw) * body_vx + std::cos(os_yaw) * body_vy;
  } else {
    os_vx = 0.0;
    os_vy = 0.0;
  }
  return true;
}

ColregsTsStateROS::PlanningInput ColregsTsStateROS::getPlanningInput(
  double os_x, double os_y)
{
  PlanningInput input;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    input.ts.stamp = now();
    input.ts.ships.reserve(ts_map_.size());
    for (const auto & pair : ts_map_) {
      input.ts.ships.push_back(pair.second);
    }
  }

  input.os.x = os_x;
  input.os.y = os_y;
  double tf_os_x = 0.0;
  double tf_os_y = 0.0;
  if (getOsPoseAndVelocity(tf_os_x, tf_os_y, input.os.vx, input.os.vy)) {
    input.os.velocity_valid = true;
  } else {
    input.os.vx = 0.0;
    input.os.vy = 0.0;
    input.os.velocity_valid = false;
  }
  return input;
}

void ColregsTsStateROS::timerCallback()
{
  const auto stamp = now();

  // Timeout eviction and raw copy under the state mutex; all computation and
  // publishing happens outside the lock.
  RawTsSnapshot raw;
  raw.stamp = stamp;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (auto it = ts_map_.begin(); it != ts_map_.end(); ) {
      if ((stamp - it->second.last_seen).seconds() > core_params_.ts_timeout) {
        it = ts_map_.erase(it);
      } else {
        ++it;
      }
    }
    raw.ships.reserve(ts_map_.size());
    for (const auto & pair : ts_map_) {
      raw.ships.push_back(pair.second);
    }
  }

  double os_x = 0.0;
  double os_y = 0.0;
  double os_vx = 0.0;
  double os_vy = 0.0;
  if (!getOsPoseAndVelocity(os_x, os_y, os_vx, os_vy)) {
    return;
  }

  OsState os;
  os.x = os_x;
  os.y = os_y;
  os.vx = os_vx;
  os.vy = os_vy;
  os.velocity_valid = true;

  const TsSnapshot snapshot = processTs(raw, os, core_params_);

  auto markers = std::make_unique<visualization_msgs::msg::MarkerArray>();
  int id = 0;
  for (const auto & ts : snapshot.ships) {
    double tcpa = ts.tcpa < 0.0 ? 0.0 : ts.tcpa;
    const double cpa_x = ts.x + ts.vx * tcpa;
    const double cpa_y = ts.y + ts.vy * tcpa;

    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = global_frame_;
    m.ns = "cpa";
    m.id = id++;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = cpa_x;
    m.pose.position.y = cpa_y;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.3;
    m.color.a = 0.8;
    if (ts.has_threat) {
      m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.2;
    } else {
      m.color.r = 1.0; m.color.g = 0.6; m.color.b = 0.2;
    }
    m.lifetime.sec = 0;
    m.lifetime.nanosec = 500000000;  // 0.5s
    markers->markers.push_back(m);
  }
  cpa_markers_pub_->publish(std::move(markers));
}

}  // namespace nav2_colregs_ts_manager
