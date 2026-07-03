#include "nav2_colregs_ts_manager/ts_state_manager.hpp"

#include <cmath>
#include <vector>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_colregs_ts_manager
{

TSStateManager::TSStateManager()
: rclcpp_lifecycle::LifecycleNode("ts_state_manager")
{
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TSStateManager::on_configure(const rclcpp_lifecycle::State &)
{
  declare_parameter("frequency", frequency_);
  declare_parameter("ts_timeout", ts_timeout_);
  declare_parameter("tcpa_horizon", tcpa_horizon_);
  declare_parameter("safety_factor", safety_factor_);
  declare_parameter("os_radius", os_radius_);
  declare_parameter("global_frame", global_frame_);
  declare_parameter("robot_base_frame", robot_base_frame_);
  declare_parameter("odom_topic", odom_topic_);

  frequency_ = get_parameter("frequency").as_double();
  ts_timeout_ = get_parameter("ts_timeout").as_double();
  tcpa_horizon_ = get_parameter("tcpa_horizon").as_double();
  safety_factor_ = get_parameter("safety_factor").as_double();
  os_radius_ = get_parameter("os_radius").as_double();
  global_frame_ = get_parameter("global_frame").as_string();
  robot_base_frame_ = get_parameter("robot_base_frame").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, shared_from_this(), true);

  RCLCPP_INFO(get_logger(), "TSStateManager configured (ts_timeout=%.1fs)", ts_timeout_);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TSStateManager::on_activate(const rclcpp_lifecycle::State &)
{
  ts_sub_ = create_subscription<nav2_colregs_msgs::msg::TrackedShipList>(
    "tracked_ship", rclcpp::SystemDefaultsQoS(),
    std::bind(&TSStateManager::trackedShipCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&TSStateManager::odomCallback, this, std::placeholders::_1));

  processed_ts_pub_ = create_publisher<nav2_colregs_msgs::msg::ProcessedTSList>(
    "processed_ts_list", 10);

  using namespace std::chrono_literals;
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / frequency_));
  timer_ = create_wall_timer(period, std::bind(&TSStateManager::timerCallback, this));

  RCLCPP_INFO(get_logger(), "TSStateManager activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TSStateManager::on_deactivate(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  ts_sub_.reset();
  odom_sub_.reset();
  processed_ts_pub_.reset();
  ts_map_.clear();
  RCLCPP_INFO(get_logger(), "TSStateManager deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TSStateManager::on_cleanup(const rclcpp_lifecycle::State &)
{
  tf_listener_.reset();
  tf_.reset();
  RCLCPP_INFO(get_logger(), "TSStateManager cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
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

  double os_vx = 0.0, os_vy = 0.0;
  if (last_odom_) {
    os_vx = last_odom_->twist.twist.linear.x;
    os_vy = last_odom_->twist.twist.linear.y;
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

    // CPA.
    const double rel_speed_sq = rel_vx * rel_vx + rel_vy * rel_vy;
    double tcpa = std::numeric_limits<double>::infinity();
    double dcpa = std::hypot(rel_x, rel_y);

    if (rel_speed_sq > 1e-6) {
      tcpa = -(rel_x * rel_vx + rel_y * rel_vy) / rel_speed_sq;
      dcpa = std::hypot(rel_x + rel_vx * tcpa, rel_y + rel_vy * tcpa);
    }

    const double safe_dist = (os_radius_ + ts.radius) * safety_factor_;
    bool has_threat = (tcpa > 0.0 && tcpa <= tcpa_horizon_ && dcpa < safe_dist);

    // Collision cone.
    std::vector<double> cone_min, cone_max;
    if (os_speed > 1e-6) {
      computeCollisionCone(ts, os_pose.pose.position.x, os_pose.pose.position.y,
                           os_speed, cone_min, cone_max);
    }

    // Fill ProcessedTS.
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
    for (size_t i = 0; i < 16; ++i) {
      entry.target_id.uuid[i] = pair.first.empty() ? 0 : 0;
    }
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

  // Inescapable: TS covers entire heading space.
  if (dist <= sum_r) {
    min_intervals.push_back(0.0);
    max_intervals.push_back(2.0 * M_PI);
    return;
  }

  const double threshold = std::asin(sum_r / dist);
  constexpr double kResolution = 2.0 * M_PI / 180.0;  // 2° in radians

  // Brute-force scan: check each heading whether rel_vel collides.
  const int N = static_cast<int>(2.0 * M_PI / kResolution);
  std::vector<bool> unsafe(N, false);
  bool any_unsafe = false;

  for (int i = 0; i < N; ++i) {
    double heading = i * kResolution;
    // OS velocity at this heading.
    double os_vx_h = os_speed * std::cos(heading);
    double os_vy_h = os_speed * std::sin(heading);
    // Relative velocity OS→TS.
    double rvx = os_vx_h - ts.vx;
    double rvy = os_vy_h - ts.vy;
    double rv_len = std::hypot(rvx, rvy);

    if (rv_len < 1e-6) {
      // OS and TS moving identically → no relative motion.
      // If already within collision distance, collision inevitable.
      if (dist < sum_r) {
        unsafe[i] = true;
        any_unsafe = true;
      }
      continue;
    }

    // Angle between rel_pos and rel_vel.
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

  // Merge adjacent unsafe bins into intervals, handling wrap-around.
  // Treat the circular array as linear by scanning once, then check wrap.
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
  // Tail: interval runs to 2π.
  if (in_interval) {
    min_intervals.push_back(start);
    max_intervals.push_back(2.0 * M_PI);
  }

  // Cross-0 intervals (e.g. [350°, 10°]) are kept as two separate intervals.
  // Consumer handles cyclic complement.
}

}  // namespace nav2_colregs_ts_manager
