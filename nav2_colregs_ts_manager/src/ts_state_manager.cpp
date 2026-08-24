#include "nav2_colregs_ts_manager/ts_state_manager.hpp"

#include <cmath>
#include <vector>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_colregs_ts_manager
{

TSStateManager::TSStateManager()
: rclcpp::Node("ts_state_manager")
{
  declare_parameter("frequency", 10.0);
  declare_parameter("ts_timeout", 1.0);
  declare_parameter("tcpa_horizon", 3.0);
  declare_parameter("safety_factor", 1.1);
  declare_parameter("os_radius", 0.3);
  declare_parameter("global_frame", "map");
  declare_parameter("robot_base_frame", "base_link");
  declare_parameter("odom_topic", "odom");
  declare_parameter("tracked_ship_topic", tracked_ship_topic_);

  frequency_ = get_parameter("frequency").as_double();
  ts_timeout_ = get_parameter("ts_timeout").as_double();
  tcpa_horizon_ = get_parameter("tcpa_horizon").as_double();
  safety_factor_ = get_parameter("safety_factor").as_double();
  os_radius_ = get_parameter("os_radius").as_double();
  global_frame_ = get_parameter("global_frame").as_string();
  robot_base_frame_ = get_parameter("robot_base_frame").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  tracked_ship_topic_ = get_parameter("tracked_ship_topic").as_string();

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());

  ts_sub_ = create_subscription<nav2_colregs_msgs::msg::TrackedShipList>(
    tracked_ship_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&TSStateManager::trackedShipCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&TSStateManager::odomCallback, this, std::placeholders::_1));

  processed_ts_pub_ = create_publisher<nav2_colregs_msgs::msg::ProcessedTSList>(
    "processed_ts_list", 10);

  cpa_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "cpa_markers", 10);

  using namespace std::chrono_literals;
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / frequency_));
  timer_ = create_wall_timer(period, std::bind(&TSStateManager::timerCallback, this));

  tf_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, true);

  RCLCPP_INFO(get_logger(), "TSStateManager started (ts_timeout=%.1fs, topic=%s)",
    ts_timeout_, tracked_ship_topic_.c_str());
}

// ---------------------------------------------------------------------------
// Subscribers
// ---------------------------------------------------------------------------

void TSStateManager::trackedShipCallback(
  nav2_colregs_msgs::msg::TrackedShipList::ConstSharedPtr msg)
{
  const auto now = get_clock()->now();

  for (const auto & ship : msg->ships) {
    const auto key = uuidToString(ship.target_id.uuid.data());

    TSEntry e;
    e.radius = ship.radius;
    e.vx = ship.twist.linear.x;
    e.vy = ship.twist.linear.y;
    e.last_seen = now;

    geometry_msgs::msg::PoseStamped ts_in, ts_out;
    ts_in.header.frame_id = msg->header.frame_id;
    ts_in.header.stamp = rclcpp::Time(0);
    ts_in.pose = ship.pose;
    try {
      ts_out = tf_->transform(ts_in, global_frame_, tf2::durationFromSec(1.0));
      e.x = ts_out.pose.position.x;
      e.y = ts_out.pose.position.y;
    } catch (const tf2::TransformException & ex) {
      e.x = ship.pose.position.x;
      e.y = ship.pose.position.y;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "TSStateManager: TF from '%s' to '%s' failed for %s: %s",
        msg->header.frame_id.c_str(), global_frame_.c_str(), key.c_str(), ex.what());
    }

    ts_map_[key] = e;
  }
}

void TSStateManager::odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  last_odom_ = msg;
}

// ---------------------------------------------------------------------------
// Timer — CPA + Collision Cone + Publish
// ---------------------------------------------------------------------------

void TSStateManager::timerCallback()
{
  const auto now = get_clock()->now();

  for (auto it = ts_map_.begin(); it != ts_map_.end(); ) {
    if ((now - it->second.last_seen).seconds() > ts_timeout_) {
      it = ts_map_.erase(it);
    } else {
      ++it;
    }
  }

  geometry_msgs::msg::PoseStamped os_pose;
  os_pose.header.frame_id = robot_base_frame_;
  os_pose.header.stamp = rclcpp::Time(0);
  os_pose.pose.orientation.w = 1.0;
  try {
    os_pose = tf_->transform(os_pose, global_frame_, tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException &) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "TSStateManager: TF lookup %s→%s not yet available, skipping publish",
      robot_base_frame_.c_str(), global_frame_.c_str());
    return;
  }

  double ox = os_pose.pose.orientation.x;
  double oy = os_pose.pose.orientation.y;
  double oz = os_pose.pose.orientation.z;
  double ow = os_pose.pose.orientation.w;
  double os_yaw = std::atan2(2.0 * (ow * oz + ox * oy),
                             1.0 - 2.0 * (oy * oy + oz * oz));

  double os_vx = 0.0, os_vy = 0.0;
  if (last_odom_) {
    double body_vx = last_odom_->twist.twist.linear.x;
    double body_vy = last_odom_->twist.twist.linear.y;
    os_vx = std::cos(os_yaw) * body_vx - std::sin(os_yaw) * body_vy;
    os_vy = std::sin(os_yaw) * body_vx + std::cos(os_yaw) * body_vy;
  }
  const double os_speed = std::hypot(os_vx, os_vy);

  auto list_msg = std::make_shared<nav2_colregs_msgs::msg::ProcessedTSList>();
  list_msg->header.stamp = now;
  list_msg->header.frame_id = global_frame_;

  for (const auto & pair : ts_map_) {
    const auto & ts = pair.second;

    const double rel_x = ts.x - os_pose.pose.position.x;
    const double rel_y = ts.y - os_pose.pose.position.y;
    const double rel_vx = ts.vx - os_vx;
    const double rel_vy = ts.vy - os_vy;

    const double rel_speed_sq = rel_vx * rel_vx + rel_vy * rel_vy;
    double tcpa = std::numeric_limits<double>::infinity();
    double dcpa = std::hypot(rel_x, rel_y);

    if (rel_speed_sq > 1e-6) {
      tcpa = -(rel_x * rel_vx + rel_y * rel_vy) / rel_speed_sq;
      dcpa = std::hypot(rel_x + rel_vx * tcpa, rel_y + rel_vy * tcpa);
    }

    const double safe_dist = (os_radius_ + ts.radius) * safety_factor_;
    bool has_threat = (tcpa > 0.0 && tcpa <= tcpa_horizon_ && dcpa < safe_dist);

    std::vector<double> cone_min, cone_max;
    if (os_speed > 1e-6) {
      computeCollisionCone(ts, os_pose.pose.position.x, os_pose.pose.position.y,
                           os_speed, cone_min, cone_max);
    }

    nav2_colregs_msgs::msg::ProcessedTS entry;
    entry.header.stamp = now;
    entry.header.frame_id = global_frame_;
    entry.pose.position.x = ts.x;
    entry.pose.position.y = ts.y;
    entry.pose.orientation.w = 1.0;
    entry.twist.linear.x = ts.vx;
    entry.twist.linear.y = ts.vy;
    entry.radius = ts.radius;
    entry.tcpa = tcpa;
    entry.dcpa = dcpa;
    entry.has_threat = has_threat;
    entry.collision_cone_min = cone_min;
    entry.collision_cone_max = cone_max;
    entry.encounter_type = nav2_colregs_msgs::msg::ProcessedTS::UNKNOWN;
    {
      const std::string & s = pair.first;
      size_t byte_idx = 0;
      for (size_t i = 0; i < s.size() && byte_idx < 16; ) {
        if (s[i] == '-') { ++i; continue; }
        unsigned int val;
        std::stringstream ss;
        ss << std::hex << s.substr(i, 2);
        ss >> val;
        entry.target_id.uuid[byte_idx++] = static_cast<uint8_t>(val);
        i += 2;
      }
    }

    list_msg->ships.push_back(entry);
  }

  processed_ts_pub_->publish(*list_msg);

  // Publish CPA markers for visualization.
  auto markers = std::make_shared<visualization_msgs::msg::MarkerArray>();
  int id = 0;
  for (const auto & pair : ts_map_) {
    const auto & ts = pair.second;

    const double rel_x = ts.x - os_pose.pose.position.x;
    const double rel_y = ts.y - os_pose.pose.position.y;
    const double rel_vx = ts.vx - os_vx;
    const double rel_vy = ts.vy - os_vy;
    const double rel_speed_sq = rel_vx * rel_vx + rel_vy * rel_vy;
    double tcpa = std::numeric_limits<double>::infinity();
    if (rel_speed_sq > 1e-6) {
      tcpa = -(rel_x * rel_vx + rel_y * rel_vy) / rel_speed_sq;
    }
    if (tcpa < 0.0) tcpa = 0.0;

    bool is_threat = (tcpa > 0.0 && tcpa <= tcpa_horizon_ &&
      std::hypot(rel_x + rel_vx * tcpa, rel_y + rel_vy * tcpa) <
      (os_radius_ + ts.radius) * safety_factor_);

    double cpa_x = ts.x + ts.vx * tcpa;
    double cpa_y = ts.y + ts.vy * tcpa;

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "CPA[%d] OS=(%.1f,%.1f) v_os=(%+.1f,%+.1f) yaw=%.1fdeg "
      "TS=(%.1f,%.1f) v_ts=(%+.1f,%+.1f) "
      "TCPA=%.1fs DCPA=%.1fm cp=(%.1f,%.1f) t=%d",
      id,
      os_pose.pose.position.x, os_pose.pose.position.y,
      os_vx, os_vy,
      os_yaw * 180.0 / M_PI,
      ts.x, ts.y, ts.vx, ts.vy,
      tcpa,
      std::hypot(rel_x + rel_vx * tcpa, rel_y + rel_vy * tcpa),
      cpa_x, cpa_y, (int)is_threat);

    visualization_msgs::msg::Marker m;
    m.header.stamp = now;
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
    if (is_threat) {
      m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.2;
    } else {
      m.color.r = 1.0; m.color.g = 0.6; m.color.b = 0.2;
    }
    m.lifetime.sec = 0;
    m.lifetime.nanosec = 500000000;  // 0.5s
    markers->markers.push_back(m);
  }
  cpa_markers_pub_->publish(*markers);
}

// ---------------------------------------------------------------------------
// LVO Collision Cone
// ---------------------------------------------------------------------------

void TSStateManager::computeCollisionCone(
  const TSEntry & ts,
  double os_x, double os_y, double os_speed,
  std::vector<double> & min_intervals,
  std::vector<double> & max_intervals)
{
  min_intervals.clear();
  max_intervals.clear();

  const double rel_x = ts.x - os_x;
  const double rel_y = ts.y - os_y;
  const double dist = std::hypot(rel_x, rel_y);
  const double sum_r = os_radius_ + ts.radius;

  if (dist <= sum_r) {
    min_intervals.push_back(0.0);
    max_intervals.push_back(2.0 * M_PI);
    return;
  }

  const double threshold = std::asin(sum_r / dist);
  constexpr double kResolution = 2.0 * M_PI / 180.0;

  const int N = static_cast<int>(2.0 * M_PI / kResolution);
  std::vector<bool> unsafe(N, false);
  bool any_unsafe = false;

  for (int i = 0; i < N; ++i) {
    double heading = i * kResolution;
    double os_vx_h = os_speed * std::cos(heading);
    double os_vy_h = os_speed * std::sin(heading);
    double rvx = os_vx_h - ts.vx;
    double rvy = os_vy_h - ts.vy;
    double rv_len = std::hypot(rvx, rvy);

    if (rv_len < 1e-6) {
      if (dist < sum_r) {
        unsafe[i] = true;
        any_unsafe = true;
      }
      continue;
    }

    double dot = rel_x * rvx + rel_y * rvy;
    double cos_angle = dot / (dist * rv_len);
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
    double angle = std::acos(cos_angle);

    if (angle <= threshold) {
      unsafe[i] = true;
      any_unsafe = true;
    }
  }

  if (!any_unsafe) {
    return;
  }

  bool in_interval = false;
  double start = 0.0;

  for (int i = 0; i < N; ++i) {
    if (unsafe[i] && !in_interval) {
      start = i * kResolution;
      in_interval = true;
    }
    if (!unsafe[i] && in_interval) {
      min_intervals.push_back(start);
      max_intervals.push_back(i * kResolution);
      in_interval = false;
    }
  }
  if (in_interval) {
    min_intervals.push_back(start);
    max_intervals.push_back(2.0 * M_PI);
  }
}

}  // namespace nav2_colregs_ts_manager
