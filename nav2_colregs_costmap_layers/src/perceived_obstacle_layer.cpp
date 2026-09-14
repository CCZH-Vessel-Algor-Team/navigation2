#include "nav2_colregs_costmap_layers/perceived_obstacle_layer.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_colregs_costmap_layers
{
namespace
{
bool validCircle(const geometry_msgs::msg::Point & point, double radius)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
         std::isfinite(radius) && radius >= 0.0 &&
         std::isfinite(point.x - radius) && std::isfinite(point.x + radius) &&
         std::isfinite(point.y - radius) && std::isfinite(point.y + radius);
}
}  // namespace

void PerceivedObstacleLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Cannot initialize PerceivedObstacleLayer without a node");
  }
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  descriptor.description = "Perceived obstacle layer startup parameter";
  const auto declare = [&](const std::string & key, const rclcpp::ParameterValue & value) {
      local_params_.insert(key);
      if (!node->has_parameter(getFullName(key))) {
        node->declare_parameter(getFullName(key), value, descriptor);
      }
    };
  declare("enabled", rclcpp::ParameterValue(true));
  declare("tracked_obstacle_topic", rclcpp::ParameterValue("/tracked_obstacles"));
  declare("observation_timeout", rclcpp::ParameterValue(3.0));
  declare("tracking_frame", rclcpp::ParameterValue("map"));
  enabled_ = node->get_parameter(getFullName("enabled")).as_bool();
  const auto topic = node->get_parameter(getFullName("tracked_obstacle_topic")).as_string();
  tracking_frame_ = node->get_parameter(getFullName("tracking_frame")).as_string();
  const double timeout = node->get_parameter(getFullName("observation_timeout")).as_double();
  if (topic.empty() || tracking_frame_.empty() || !std::isfinite(timeout) ||
    timeout <= 0.0 || timeout * 1e9 >= static_cast<double>(std::numeric_limits<int64_t>::max()))
  {
    throw std::invalid_argument("Invalid perceived obstacle topic, frame or timeout");
  }
  timeout_ns_ = static_cast<int64_t>(timeout * 1e9);
  if (timeout_ns_ < 1) {
    throw std::invalid_argument("observation_timeout must be at least one nanosecond");
  }
  global_frame_ = layered_costmap_->getGlobalFrameID();
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  sub_ = node->create_subscription<usv_interfaces::msg::TrackedObstacleList>(
    topic, rclcpp::SensorDataQoS(),
    std::bind(&PerceivedObstacleLayer::obstacleCallback, this, std::placeholders::_1), options);
  current_ = true;
  RCLCPP_INFO(
    logger_, "PerceivedObstacleLayer: topic=%s timeout=%.3fs tracking_frame=%s costmap_frame=%s",
    topic.c_str(), timeout, tracking_frame_.c_str(), global_frame_.c_str());
}

bool PerceivedObstacleLayer::lookupTransform(
  const std::string & target, const std::string & source, const rclcpp::Time & stamp,
  geometry_msgs::msg::TransformStamped & transform)
{
  if (target == source) {
    transform = geometry_msgs::msg::TransformStamped();
    transform.transform.rotation.w = 1.0;
    return true;
  }
  try {
    if (!tf_) {
      throw tf2::TransformException("TF buffer unavailable");
    }
    // Immediate cache lookup: never wait for TF while holding the costmap update lock.
    transform = static_cast<tf2::BufferCore &>(*tf_).lookupTransform(
      target, source, tf2::TimePoint(std::chrono::nanoseconds(stamp.nanoseconds())));
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "PerceivedObstacleLayer: cannot transform %s to %s: %s",
      source.c_str(), target.c_str(), ex.what());
    return false;
  }
}

void PerceivedObstacleLayer::handleClockJump(int64_t now_ns)
{
  if (last_clock_ns_ >= 0 && now_ns < last_clock_ns_) {
    observations_.clear();
    // Keep previous_bounds_ so old marks can be withdrawn on the next update.
  }
  last_clock_ns_ = now_ns;
}

void PerceivedObstacleLayer::obstacleCallback(
  usv_interfaces::msg::TrackedObstacleList::ConstSharedPtr msg)
{
  if (!enabled_ || msg->header.frame_id.empty() || msg->header.stamp.sec < 0) {
    return;
  }
  const rclcpp::Time stamp(msg->header.stamp);
  const int64_t stamp_ns = stamp.nanoseconds();
  const int64_t now_ns = clock_->now().nanoseconds();
  // Zero is ambiguous (TF interprets it as latest). Very future-dated observations
  // are also rejected; modest clock delivery skew does not extend a track forever.
  if (stamp_ns == 0 || now_ns - stamp_ns > timeout_ns_ || stamp_ns - now_ns > timeout_ns_) {
    return;
  }
  geometry_msgs::msg::TransformStamped transform;
  if (!lookupTransform(tracking_frame_, msg->header.frame_id, stamp, transform)) {
    return;
  }
  std::vector<std::pair<std::string, Observation>> incoming;
  incoming.reserve(msg->obstacles.size());
  for (const auto & object : msg->obstacles) {
    if (!validCircle(object.pose.position, object.radius)) {
      continue;
    }
    geometry_msgs::msg::PointStamped input, output;
    input.header = msg->header;
    input.point = object.pose.position;
    tf2::doTransform(input, output, transform);
    if (!validCircle(output.point, object.radius)) {
      continue;
    }
    const std::string key(
      reinterpret_cast<const char *>(object.target_id.uuid.data()), object.target_id.uuid.size());
    incoming.emplace_back(key, Observation{Circle{output.point, object.radius}, stamp_ns});
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const int64_t accepted_now_ns = clock_->now().nanoseconds();
  handleClockJump(accepted_now_ns);
  // A simulation clock jump may have occurred during transformation of the batch.
  if (accepted_now_ns - stamp_ns > timeout_ns_ || stamp_ns - accepted_now_ns > timeout_ns_) {
    return;
  }
  for (const auto & item : incoming) {
    const auto it = observations_.find(item.first);
    if (it == observations_.end() || item.second.stamp_ns > it->second.stamp_ns) {
      observations_[item.first] = item.second;
    }
  }
  // Missing IDs/empty lists do not delete another publisher's objects.
}

void PerceivedObstacleLayer::Bounds::include(const Circle & circle, double padding)
{
  const double extent = circle.radius + padding;
  const double x0 = circle.center.x - extent, x1 = circle.center.x + extent;
  const double y0 = circle.center.y - extent, y1 = circle.center.y + extent;
  if (!valid) {
    min_x = x0; min_y = y0; max_x = x1; max_y = y1;
    valid = true;
  } else {
    min_x = std::min(min_x, x0); min_y = std::min(min_y, y0);
    max_x = std::max(max_x, x1); max_y = std::max(max_y, y1);
  }
}

void PerceivedObstacleLayer::Bounds::expand(
  double * x0, double * y0, double * x1, double * y1) const
{
  if (valid) {
    *x0 = std::min(*x0, min_x); *y0 = std::min(*y0, min_y);
    *x1 = std::max(*x1, max_x); *y1 = std::max(*y1, max_y);
  }
}

void PerceivedObstacleLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const int64_t now_ns = clock_->now().nanoseconds();
  handleClockJump(now_ns);
  for (auto it = observations_.begin(); it != observations_.end(); ) {
    if (now_ns - it->second.stamp_ns > timeout_ns_) {
      it = observations_.erase(it);
    } else {
      ++it;
    }
  }
  previous_bounds_.expand(min_x, min_y, max_x, max_y);
  previous_bounds_ = Bounds();
  render_snapshot_.clear();
  current_ = true;
  if (!enabled_ || observations_.empty()) {
    return;
  }
  geometry_msgs::msg::TransformStamped transform;
  if (!lookupTransform(global_frame_, tracking_frame_, rclcpp::Time(0), transform)) {
    current_ = false;
    return;
  }
  const double padding = layered_costmap_->getCostmap()->getResolution();
  for (const auto & entry : observations_) {
    geometry_msgs::msg::PointStamped input, output;
    input.point = entry.second.circle.center;
    tf2::doTransform(input, output, transform);
    Circle circle{output.point, entry.second.circle.radius};
    if (validCircle(circle.center, circle.radius + padding)) {
      render_snapshot_.push_back(circle);
      previous_bounds_.include(circle, padding);
    }
  }
  previous_bounds_.expand(min_x, min_y, max_x, max_y);
}

void PerceivedObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master, int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    return;
  }
  min_i = std::max(0, min_i); min_j = std::max(0, min_j);
  max_i = std::min(max_i, static_cast<int>(master.getSizeInCellsX()));
  max_j = std::min(max_j, static_cast<int>(master.getSizeInCellsY()));
  if (min_i >= max_i || min_j >= max_j) {
    return;
  }
  const double resolution = master.getResolution();
  const double half_cell = 0.5 * resolution;
  for (const auto & circle : render_snapshot_) {
    int x0, y0, x1, y1;
    const double extent = circle.radius + resolution;
    master.worldToMapEnforceBounds(circle.center.x - extent, circle.center.y - extent, x0, y0);
    master.worldToMapEnforceBounds(circle.center.x + extent, circle.center.y + extent, x1, y1);
    for (int y = std::max(y0, min_j); y < std::min(y1 + 1, max_j); ++y) {
      for (int x = std::max(x0, min_i); x < std::min(x1 + 1, max_i); ++x) {
        double wx, wy;
        master.mapToWorld(x, y, wx, wy);
        // Mark every cell intersecting the physical circle, including sub-cell buoys.
        const double dx = std::max(0.0, std::abs(wx - circle.center.x) - half_cell);
        const double dy = std::max(0.0, std::abs(wy - circle.center.y) - half_cell);
        if (std::hypot(dx, dy) <= circle.radius) {
          master.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }
}

void PerceivedObstacleLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  observations_.clear();
  render_snapshot_.clear();
  current_ = true;
  // Preserve previous_bounds_ until the next update withdraws our old contribution.
}

}  // namespace nav2_colregs_costmap_layers

PLUGINLIB_EXPORT_CLASS(
  nav2_colregs_costmap_layers::PerceivedObstacleLayer, nav2_costmap_2d::Layer)
