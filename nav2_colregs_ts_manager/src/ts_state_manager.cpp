#include "nav2_colregs_ts_manager/ts_state_manager.hpp"

#include <cmath>

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

  threat_service_ = create_service<nav2_colregs_msgs::srv::GetPrimaryThreat>(
    "get_primary_threat",
    std::bind(&TSStateManager::handleServiceRequest, this,
              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  RCLCPP_INFO(get_logger(), "TSStateManager configured");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TSStateManager::on_activate(const rclcpp_lifecycle::State &)
{
  ts_sub_ = create_subscription<nav2_colregs_msgs::msg::TrackedShip>(
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
  ts_valid_ = false;
  has_threat_ = false;
  RCLCPP_INFO(get_logger(), "TSStateManager deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TSStateManager::on_cleanup(const rclcpp_lifecycle::State &)
{
  threat_service_.reset();
  tf_listener_.reset();
  tf_.reset();
  RCLCPP_INFO(get_logger(), "TSStateManager cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Subscribers
// ---------------------------------------------------------------------------

void TSStateManager::trackedShipCallback(
  nav2_colregs_msgs::msg::TrackedShip::ConstSharedPtr msg)
{
  last_ts_ = msg;
  last_ts_stamp_ = get_clock()->now();
  ts_valid_ = true;
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
  if (!ts_valid_ || !last_ts_) {
    return;
  }

  const auto now = get_clock()->now();
  const double age = (now - last_ts_stamp_).seconds();
  if (age > ts_timeout_) {
    ts_valid_ = false;
    has_threat_ = false;
    return;
  }

  const double ts_x = last_ts_->pose.position.x;
  const double ts_y = last_ts_->pose.position.y;
  const double ts_r = last_ts_->radius;
  const double ts_vx = last_ts_->twist.linear.x;
  const double ts_vy = last_ts_->twist.linear.y;

  // OS pose via TF
  geometry_msgs::msg::PoseStamped os_pose;
  os_pose.header.frame_id = robot_base_frame_;
  os_pose.header.stamp = rclcpp::Time(0);
  os_pose.pose.orientation.w = 1.0;
  try {
    os_pose = tf_->transform(os_pose, global_frame_, tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException &) {
    return;
  }

  // OS velocity via odom
  double os_vx = 0.0, os_vy = 0.0;
  if (last_odom_) {
    os_vx = last_odom_->twist.twist.linear.x;
    os_vy = last_odom_->twist.twist.linear.y;
  }

  // Relative motion
  const double rel_x = ts_x - os_pose.pose.position.x;
  const double rel_y = ts_y - os_pose.pose.position.y;
  const double rel_vx = ts_vx - os_vx;
  const double rel_vy = ts_vy - os_vy;

  // CPA / TCPA
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

  // Threat
  const double safe_dist = (os_radius_ + ts_r) * safety_factor_;
  const bool is_threat = (tcpa <= tcpa_horizon_ && dcpa < safe_dist);
  has_threat_ = is_threat;

  // Fill ProcessedTS
  threat_.header.stamp = now;
  threat_.header.frame_id = global_frame_;
  threat_.pose = last_ts_->pose;
  threat_.twist = last_ts_->twist;
  threat_.radius = ts_r;
  threat_.tcpa = (tcpa == std::numeric_limits<double>::infinity()) ? 0.0 : tcpa;
  threat_.dcpa = dcpa;
  threat_.has_threat = has_threat_;

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "OS(%.2f,%.2f) vel(%.2f,%.2f) TS(%.2f,%.2f) vel(%.2f,%.2f) "
    "rel(%.2f,%.2f) dcpa=%.2f tcpa=%.2f has_threat=%d",
    os_pose.pose.position.x, os_pose.pose.position.y,
    os_vx, os_vy, ts_x, ts_y, ts_vx, ts_vy,
    rel_x, rel_y, dcpa,
    (tcpa == std::numeric_limits<double>::infinity()) ? -1.0 : tcpa,
    has_threat_);
}

// ---------------------------------------------------------------------------
// Service
// ---------------------------------------------------------------------------

void TSStateManager::handleServiceRequest(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<nav2_colregs_msgs::srv::GetPrimaryThreat::Request>,
  const std::shared_ptr<nav2_colregs_msgs::srv::GetPrimaryThreat::Response> response)
{
  response->has_threat = has_threat_;
  if (has_threat_) {
    response->primary_threat = threat_;
  }
}

}  // namespace nav2_colregs_ts_manager
