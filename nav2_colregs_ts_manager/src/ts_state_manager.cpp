#include "nav2_colregs_ts_manager/ts_state_manager.hpp"
#include "nav2_colregs_ts_manager/parameter_contract.hpp"
#include "nav2_colregs_ts_manager/decision_geometry.hpp"

#include <cmath>
#include <vector>
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_colregs_ts_manager
{
TSStateManager::TSStateManager()
: rclcpp::Node("ts_state_manager")
{
  declare_parameter("frequency", 10.0, parameterDescription("Wall-timer update frequency [Hz].", true));
  declare_parameter("ts_timeout", 1.0,
    parameterDescription("Maximum track-list measurement AND receipt age [s].", true));
  declare_parameter("odom_timeout", 1.0,
    parameterDescription("Maximum own-ship odometry/TF age [s].", true));
  declare_parameter("tcpa_horizon", 3.0, parameterDescription("Threat TCPA horizon [s].", true));
  declare_parameter("safety_factor", 1.1,
    parameterDescription("Radius inflation >= 1 for threat detection and diagnostic cones.", true));
  declare_parameter("os_radius", 0.3,
    parameterDescription("Own-ship radius for cones and threat threshold [m].", true));
  declare_parameter("global_frame", "map", parameterDescription("Output/geometry frame.", true));
  declare_parameter("robot_base_frame", "base_link", parameterDescription("Own-ship TF frame.", true));
  declare_parameter("odom_topic", "odom", parameterDescription("Own-ship odometry input.", true));
  declare_parameter("tracked_ship_topic", tracked_ship_topic_,
    parameterDescription("Complete stamped TrackedShipList input snapshots.", true));

  frequency_ = get_parameter("frequency").as_double();
  ts_timeout_ = get_parameter("ts_timeout").as_double();
  odom_timeout_ = get_parameter("odom_timeout").as_double();
  tcpa_horizon_ = get_parameter("tcpa_horizon").as_double();
  safety_factor_ = get_parameter("safety_factor").as_double();
  os_radius_ = get_parameter("os_radius").as_double();
  global_frame_ = get_parameter("global_frame").as_string();
  robot_base_frame_ = get_parameter("robot_base_frame").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  tracked_ship_topic_ = get_parameter("tracked_ship_topic").as_string();
  validateNumber("frequency", frequency_, true);
  validateNumber("ts_timeout", ts_timeout_, true);
  validateNumber("odom_timeout", odom_timeout_, true);
  validateNumber("tcpa_horizon", tcpa_horizon_);
  validateSafetyFactor(safety_factor_);
  validateNumber("os_radius", os_radius_);
  const double period_ns = 1e9 / frequency_;
  if (period_ns < 1.0 || period_ns >= static_cast<double>(INT64_MAX)) {
    throw std::invalid_argument("frequency must produce a representable positive timer period");
  }
  if (global_frame_.empty() || robot_base_frame_.empty() ||
    odom_topic_.empty() || tracked_ship_topic_.empty())
  {
    throw std::invalid_argument("TS frames and input topics must not be empty");
  }
  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, true);
  ts_sub_ = create_subscription<nav2_colregs_msgs::msg::TrackedShipList>(
    tracked_ship_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&TSStateManager::trackedShipCallback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&TSStateManager::odomCallback, this, std::placeholders::_1));
  processed_ts_pub_ = create_publisher<nav2_colregs_msgs::msg::ProcessedTSList>("processed_ts_list", 10);
  cpa_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("cpa_markers", 10);
  timer_ = create_wall_timer(std::chrono::nanoseconds(static_cast<int64_t>(period_ns)),
    std::bind(&TSStateManager::timerCallback, this));
  RCLCPP_INFO(get_logger(),
    "TS effective parameters: frequency=%.3f timeout=%.3f odom_timeout=%.3f horizon=%.3f "
    "threat_scale=%.3f os_radius=%.3f frame=%s base=%s odom=%s topic=%s (startup-only)",
    frequency_, ts_timeout_, odom_timeout_, tcpa_horizon_, safety_factor_, os_radius_,
    global_frame_.c_str(), robot_base_frame_.c_str(), odom_topic_.c_str(), tracked_ship_topic_.c_str());
}

void TSStateManager::trackedShipCallback(
  nav2_colregs_msgs::msg::TrackedShipList::ConstSharedPtr msg)
{
  const auto current = now();
  // Ignore delayed packets while a newer, still-fresh snapshot exists. Allow a
  // new clock epoch once the old stamp lies in the future.
  if (have_tracks_ && tracks_valid_ && fresh(track_stamp_, current, ts_timeout_) &&
    rclcpp::Time(msg->header.stamp) < rclcpp::Time(track_stamp_))
  {
    return;
  }
  have_tracks_ = true;
  tracks_valid_ = false;
  track_stamp_ = msg->header.stamp;
  track_receipt_ = current;
  ts_map_.clear();
  if (!fresh(track_stamp_, current, ts_timeout_) || msg->header.frame_id.empty()) {
    return;
  }
  std::unordered_map<std::string, TSEntry> next;
  for (const auto & ship : msg->ships) {
    if (!finitePoint(ship.pose.position) || !std::isfinite(ship.radius) || ship.radius < 0.0 ||
      !std::isfinite(ship.twist.linear.x) || !std::isfinite(ship.twist.linear.y))
    {
      return;
    }
    geometry_msgs::msg::PoseStamped position;
    position.header = msg->header;
    position.pose = ship.pose;
    geometry_msgs::msg::Vector3Stamped velocity;
    velocity.header = msg->header;
    velocity.vector = ship.twist.linear;
    try {
      position = tf_->transform(position, global_frame_, tf2::durationFromSec(0.1));
      velocity = tf_->transform(velocity, global_frame_, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid TS frame: %s", ex.what());
      return;  // Never relabel raw coordinates as global coordinates.
    }
    TSEntry entry;
    entry.id = ship.target_id;
    entry.x = position.pose.position.x;
    entry.y = position.pose.position.y;
    entry.vx = velocity.vector.x;
    entry.vy = velocity.vector.y;
    entry.radius = ship.radius;
    next[uuidToString(ship.target_id.uuid.data())] = entry;
  }
  ts_map_ = std::move(next);  // Lists are complete snapshots; an empty list removes all targets.
  tracks_valid_ = true;
}

void TSStateManager::odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (last_odom_ && fresh(last_odom_->header.stamp, now(), odom_timeout_) &&
    rclcpp::Time(msg->header.stamp) < rclcpp::Time(last_odom_->header.stamp))
  {
    return;
  }
  last_odom_ = msg;
}

void TSStateManager::timerCallback()
{
  const auto current = now();
  nav2_colregs_msgs::msg::ProcessedTSList state;
  state.header.stamp = current;
  state.header.frame_id = global_frame_;
  state.os_radius = os_radius_;
  for (size_t i = 0; i < 16; i += 8) {
    const auto bits = snapshot_rng_();
    for (size_t j = 0; j < 8; ++j) {
      state.snapshot_id.uuid[i + j] = static_cast<uint8_t>(bits >> (j * 8));
    }
  }
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  auto invalid = [&](const std::string & reason) {
      state.valid = false;
      state.status_message = reason;
      state.ships.clear();
      processed_ts_pub_->publish(state);
      cpa_markers_pub_->publish(markers);
    };
  if (!have_tracks_ || !tracks_valid_ || !fresh(track_stamp_, current, ts_timeout_) ||
    !fresh(track_receipt_, current, ts_timeout_))
  {
    invalid("Track stream missing, stale, invalid or from a different clock epoch");
    return;
  }
  if (!last_odom_ || !fresh(last_odom_->header.stamp, current, odom_timeout_) ||
    !std::isfinite(last_odom_->twist.twist.linear.x) ||
    !std::isfinite(last_odom_->twist.twist.linear.y))
  {
    invalid("Own-ship odometry missing or stale/invalid");
    return;
  }
  geometry_msgs::msg::PoseStamped os;
  os.header.frame_id = robot_base_frame_;
  os.pose.orientation.w = 1.0;
  geometry_msgs::msg::Vector3Stamped velocity;
  velocity.header.stamp = last_odom_->header.stamp;
  velocity.header.frame_id = last_odom_->child_frame_id.empty() ?
    robot_base_frame_ : last_odom_->child_frame_id;
  velocity.vector = last_odom_->twist.twist.linear;
  try {
    os = tf_->transform(os, global_frame_, tf2::durationFromSec(0.1));
    velocity = tf_->transform(velocity, global_frame_, tf2::durationFromSec(0.1));
    const bool static_tf = os.header.stamp.sec == 0 && os.header.stamp.nanosec == 0;
    if (!static_tf && !fresh(os.header.stamp, current, odom_timeout_)) {
      invalid("Own-ship TF is stale");
      return;
    }
    if (!static_tf) {
      // Put OS and TS positions at the same calculation time under the same
      // constant-velocity assumption; "latest TF" may still lag behind current.
      const double age = (current -
        rclcpp::Time(os.header.stamp, current.get_clock_type())).seconds();
      os.pose.position.x += velocity.vector.x * age;
      os.pose.position.y += velocity.vector.y * age;
    }
  } catch (const tf2::TransformException & ex) {
    invalid(std::string("Own-ship TF unavailable: ") + ex.what());
    return;
  }
  state.os_pose = os.pose;
  state.os_twist.linear = velocity.vector;
  const double speed = std::hypot(velocity.vector.x, velocity.vector.y);
  const double age = (current - rclcpp::Time(track_stamp_, current.get_clock_type())).seconds();
  int marker_id = 0;
  for (const auto & pair : ts_map_) {
    auto ts = pair.second;
    ts.x += ts.vx * age;
    ts.y += ts.vy * age;
    const double rx = ts.x - os.pose.position.x;
    const double ry = ts.y - os.pose.position.y;
    const double ux = ts.vx - velocity.vector.x;
    const double uy = ts.vy - velocity.vector.y;
    const double speed_sq = ux * ux + uy * uy;
    const double distance = std::hypot(rx, ry);
    const bool overlap = distance <= safety_factor_ * (os_radius_ + ts.radius);
    double tcpa = std::numeric_limits<double>::infinity();
    double dcpa = distance;
    if (speed_sq > 1e-12) {
      tcpa = -(rx * ux + ry * uy) / speed_sq;
      dcpa = std::hypot(rx + ux * tcpa, ry + uy * tcpa);
    }
    // A current safety-domain intrusion is a threat, but must not overwrite
    // mathematical TCPA (including infinity for equal velocities).
    nav2_colregs_msgs::msg::ProcessedTS entry;
    entry.header = state.header;
    entry.target_id = ts.id;
    entry.pose.position.x = ts.x;
    entry.pose.position.y = ts.y;
    entry.pose.orientation.w = 1.0;
    entry.twist.linear.x = ts.vx;
    entry.twist.linear.y = ts.vy;
    entry.radius = ts.radius;
    entry.tcpa = tcpa;
    entry.dcpa = dcpa;
    entry.has_threat = overlap || (tcpa >= 0.0 && tcpa <= tcpa_horizon_ &&
      dcpa < (os_radius_ + ts.radius) * safety_factor_);
    computeCollisionCone(ts, os.pose.position.x, os.pose.position.y, speed,
      entry.collision_cone_min, entry.collision_cone_max);
    state.ships.push_back(entry);
    visualization_msgs::msg::Marker marker;
    marker.header = state.header;
    marker.ns = "cpa";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    const double prediction = std::isfinite(tcpa) ? std::max(0.0, tcpa) : 0.0;
    marker.pose.position.x = ts.x + ts.vx * prediction;
    marker.pose.position.y = ts.y + ts.vy * prediction;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.3;
    marker.color.a = 0.8;
    marker.color.r = 1.0;
    marker.color.g = entry.has_threat ? 0.2 : 0.6;
    marker.color.b = 0.2;
    marker.lifetime.nanosec = 500000000;
    if (finitePoint(marker.pose.position)) {
      markers.markers.push_back(marker);
    }
  }
  std::sort(state.ships.begin(), state.ships.end(), [](const auto & a, const auto & b) {
      return a.target_id.uuid < b.target_id.uuid;
    });
  state.valid = true;
  if (!validSnapshot(state)) {
    invalid("Nonfinite or malformed computed snapshot");
    return;
  }
  state.status_message = "Fresh coherent OS/TS snapshot";
  processed_ts_pub_->publish(state);
  cpa_markers_pub_->publish(markers);
}

void TSStateManager::computeCollisionCone(
  const TSEntry & ts, double os_x, double os_y, double os_speed,
  std::vector<double> & mins, std::vector<double> & maxs)
{
  constexpr int samples = 180;
  constexpr double step = 2.0 * M_PI / samples;
  bool in_interval = false;
  for (int i = 0; i < samples; ++i) {
    const double angle = i * step;
    const bool unsafe = collisionCourse(ts.x - os_x, ts.y - os_y,
      ts.vx - os_speed * std::cos(angle), ts.vy - os_speed * std::sin(angle),
      safety_factor_ * (os_radius_ + ts.radius));
    if (unsafe && !in_interval) {
      mins.push_back(angle);
      in_interval = true;
    } else if (!unsafe && in_interval) {
      maxs.push_back(angle);
      in_interval = false;
    }
  }
  if (in_interval) {
    maxs.push_back(2.0 * M_PI);
  }
}
}  // namespace nav2_colregs_ts_manager
