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
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, shared_from_this(), false);

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
  ts_map_.clear();
  has_threat_ = false;
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

    // Transform TS pose from list frame to global_frame (map).
    geometry_msgs::msg::PoseStamped ts_in, ts_out;
    ts_in.header.frame_id = msg->header.frame_id;
    ts_in.header.stamp = rclcpp::Time(0);
    ts_in.pose = ship.pose;
    try {
      ts_out = tf_->transform(ts_in, global_frame_, tf2::durationFromSec(1.0));
      e.x = ts_out.pose.position.x;
      e.y = ts_out.pose.position.y;
    } catch (const tf2::TransformException & ex) {
      // Fall back to raw coordinates; may cause errors if frame != map.
      e.x = ship.pose.position.x;
      e.y = ship.pose.position.y;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "TSStateManager: TF from '%s' to '%s' failed for %s, using raw coords: %s",
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
// Timer
// ---------------------------------------------------------------------------

void TSStateManager::timerCallback()
{
  const auto now = get_clock()->now();

  // Purge stale TS entries.
  for (auto it = ts_map_.begin(); it != ts_map_.end(); ) {
    if ((now - it->second.last_seen).seconds() > ts_timeout_) {
      it = ts_map_.erase(it);
    } else {
      ++it;
    }
  }

  if (ts_map_.empty()) {
    has_threat_ = false;
    return;
  }

  // OS pose via TF.
  geometry_msgs::msg::PoseStamped os_pose;
  os_pose.header.frame_id = robot_base_frame_;
  os_pose.header.stamp = rclcpp::Time(0);
  os_pose.pose.orientation.w = 1.0;
  try {
    os_pose = tf_->transform(os_pose, global_frame_, tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException &) {
    return;
  }

  // OS velocity via odom.
  double os_vx = 0.0, os_vy = 0.0;
  if (last_odom_) {
    os_vx = last_odom_->twist.twist.linear.x;
    os_vy = last_odom_->twist.twist.linear.y;
  }

  // Find primary threat: smallest TCPA among ships with DCPA violation.
  has_threat_ = false;
  double best_tcpa = std::numeric_limits<double>::infinity();
  double best_dcpa = 0.0;
  const TSEntry * best_entry = nullptr;
  std::string best_key;

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
      if (tcpa < 0.0) {
        tcpa = std::numeric_limits<double>::infinity();
      } else {
        dcpa = std::hypot(rel_x + rel_vx * tcpa, rel_y + rel_vy * tcpa);
      }
    }

    const double safe_dist = (os_radius_ + ts.radius) * safety_factor_;
    if (tcpa <= tcpa_horizon_ && dcpa < safe_dist && tcpa < best_tcpa) {
      best_tcpa = tcpa;
      best_dcpa = dcpa;
      best_entry = &ts;
      best_key = pair.first;
      has_threat_ = true;
    }
  }

  if (!has_threat_ || best_entry == nullptr) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "OS(%.2f,%.2f) tracking %lu ships, no threat",
      os_pose.pose.position.x, os_pose.pose.position.y,
      ts_map_.size());
    return;
  }

  // Fill primary threat ProcessedTS.
  threat_.header.stamp = now;
  threat_.header.frame_id = global_frame_;
  threat_.pose.position.x = best_entry->x;
  threat_.pose.position.y = best_entry->y;
  threat_.pose.position.z = 0.0;
  threat_.pose.orientation.w = 1.0;
  threat_.twist.linear.x = best_entry->vx;
  threat_.twist.linear.y = best_entry->vy;
  threat_.radius = best_entry->radius;
  threat_.tcpa = (best_tcpa == std::numeric_limits<double>::infinity()) ? 0.0 : best_tcpa;
  threat_.dcpa = best_dcpa;
  threat_.has_threat = true;

  // Reconstruct UUID bytes from hex key string.
  {
    const std::string & s = best_key;
    size_t byte_idx = 0;
    for (size_t i = 0; i < s.size() && byte_idx < 16; ) {
      if (s[i] == '-') { ++i; continue; }
      unsigned int val;
      std::stringstream ss;
      ss << std::hex << s.substr(i, 2);
      ss >> val;
      threat_.target_id.uuid[byte_idx++] = static_cast<uint8_t>(val);
      i += 2;
    }
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "OS(%.2f,%.2f) tracking=%lu primary=%s dcpa=%.2f tcpa=%.2f",
    os_pose.pose.position.x, os_pose.pose.position.y,
    ts_map_.size(), best_key.c_str(),
    best_dcpa,
    (best_tcpa == std::numeric_limits<double>::infinity()) ? -1.0 : best_tcpa);
}

}  // namespace nav2_colregs_ts_manager
