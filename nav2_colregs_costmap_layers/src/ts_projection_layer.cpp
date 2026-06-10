#include "nav2_colregs_costmap_layers/ts_projection_layer.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace nav2_colregs_costmap_layers
{

TSProjectionLayer::TSProjectionLayer()
{
}

void TSProjectionLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in TSProjectionLayer::onInitialize");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  sub_ = node->create_subscription<nav2_colregs_msgs::msg::TrackedShip>(
    "/tracked_ship", rclcpp::SystemDefaultsQoS(),
    std::bind(&TSProjectionLayer::trackedShipCallback, this, std::placeholders::_1));

  global_frame_ = layered_costmap_->getGlobalFrameID();
  current_ = true;
  RCLCPP_INFO(logger_, "TSProjectionLayer initialized, subscribed to /tracked_ship");
}

void TSProjectionLayer::trackedShipCallback(
  nav2_colregs_msgs::msg::TrackedShip::ConstSharedPtr msg)
{
  // Expand cumulative bounds to include the OLD position (for clearing residue).
  if (msg_received_ && last_msg_) {
    double ox = last_msg_->pose.position.x;
    double oy = last_msg_->pose.position.y;
    double or_ = last_msg_->radius;
    if (!has_cumulative_bounds_) {
      cum_min_x_ = ox - or_;
      cum_min_y_ = oy - or_;
      cum_max_x_ = ox + or_;
      cum_max_y_ = oy + or_;
      has_cumulative_bounds_ = true;
    } else {
      cum_min_x_ = std::min(cum_min_x_, ox - or_);
      cum_min_y_ = std::min(cum_min_y_, oy - or_);
      cum_max_x_ = std::max(cum_max_x_, ox + or_);
      cum_max_y_ = std::max(cum_max_y_, oy + or_);
    }
  }
  last_msg_ = msg;
  msg_received_ = true;
}

void TSProjectionLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!msg_received_ || !last_msg_) {
    return;
  }

  // Transform cumulative bounds from map frame to costmap frame.
  if (has_cumulative_bounds_) {
    geometry_msgs::msg::PointStamped c1, c2, t1, t2;
    c1.header.frame_id = last_msg_->header.frame_id; c2.header = c1.header;
    c1.header.stamp = c2.header.stamp = rclcpp::Time(0);
    c1.point.x = cum_min_x_; c1.point.y = cum_min_y_;
    c2.point.x = cum_max_x_; c2.point.y = cum_max_y_;
    try {
      tf_->transform(c1, t1, global_frame_, tf2::durationFromSec(1.0));
      tf_->transform(c2, t2, global_frame_, tf2::durationFromSec(1.0));
      *min_x = std::min(*min_x, std::min(t1.point.x, t2.point.x));
      *min_y = std::min(*min_y, std::min(t1.point.y, t2.point.y));
      *max_x = std::max(*max_x, std::max(t1.point.x, t2.point.x));
      *max_y = std::max(*max_y, std::max(t1.point.y, t2.point.y));
    } catch (const tf2::TransformException &) { /* skip if TF not ready */ }
    has_cumulative_bounds_ = false;
  }

  geometry_msgs::msg::PoseStamped ts_transformed;
  transformTS(ts_transformed);

  const double ts_x = ts_transformed.pose.position.x;
  const double ts_y = ts_transformed.pose.position.y;
  const double ts_r = last_msg_->radius;

  *min_x = std::min(*min_x, ts_x - ts_r);
  *min_y = std::min(*min_y, ts_y - ts_r);
  *max_x = std::max(*max_x, ts_x + ts_r);
  *max_y = std::max(*max_y, ts_y + ts_r);
}

void TSProjectionLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  if (!enabled_ || !msg_received_ || !last_msg_) {
    return;
  }

  geometry_msgs::msg::PoseStamped ts_transformed;
  transformTS(ts_transformed);

  const double ts_x = ts_transformed.pose.position.x;
  const double ts_y = ts_transformed.pose.position.y;
  const double ts_r = last_msg_->radius;

  unsigned int mx, my;
  if (!master_grid.worldToMap(ts_x, ts_y, mx, my)) {
    return;
  }

  // Radius in cells (ceiling to ensure full coverage).
  const unsigned int radius_in_cells =
    static_cast<unsigned int>(std::ceil(ts_r / master_grid.getResolution()));

  // Mark a filled circle at the TS position as LETHAL.
  const unsigned int cell_radius_sq = (radius_in_cells + 1) * (radius_in_cells + 1);
  const unsigned int width = master_grid.getSizeInCellsX();
  const unsigned int height = master_grid.getSizeInCellsY();

  for (unsigned int dx = 0; dx <= radius_in_cells; ++dx) {
    for (unsigned int dy = 0; dy <= radius_in_cells; ++dy) {
      if (dx * dx + dy * dy > cell_radius_sq) {
        continue;
      }
      if (mx + dx < width && my + dy < height) {
        master_grid.getCharMap()[master_grid.getIndex(mx + dx, my + dy)] =
          nav2_costmap_2d::LETHAL_OBSTACLE;
      }
      if (mx >= dx && my + dy < height) {
        master_grid.getCharMap()[master_grid.getIndex(mx - dx, my + dy)] =
          nav2_costmap_2d::LETHAL_OBSTACLE;
      }
      if (mx + dx < width && my >= dy) {
        master_grid.getCharMap()[master_grid.getIndex(mx + dx, my - dy)] =
          nav2_costmap_2d::LETHAL_OBSTACLE;
      }
      if (mx >= dx && my >= dy) {
        master_grid.getCharMap()[master_grid.getIndex(mx - dx, my - dy)] =
          nav2_costmap_2d::LETHAL_OBSTACLE;
      }
    }
  }
}

void TSProjectionLayer::reset()
{
  last_msg_.reset();
  msg_received_ = false;
}

void TSProjectionLayer::transformTS(geometry_msgs::msg::PoseStamped & out)
{
  geometry_msgs::msg::PoseStamped ts_pose;
  ts_pose.header.frame_id = last_msg_->header.frame_id;
  ts_pose.header.stamp = rclcpp::Time(0);
  ts_pose.pose = last_msg_->pose;

  try {
    tf_->transform(ts_pose, out, global_frame_, tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException & e) {
    // When TF chain is not yet available (e.g. map→odom before AMCL init),
    // fall back to using the pose in its original frame. This works correctly
    // when the costmap frame matches the TS message frame (e.g. both "map").
    out = ts_pose;
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000,
      "TSProjectionLayer: TF (%s → %s) unavailable, using original frame: %s",
      ts_pose.header.frame_id.c_str(), global_frame_.c_str(), e.what());
  }
}

bool TSProjectionLayer::isClearable()
{
  return false;
}

}  // namespace nav2_colregs_costmap_layers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_colregs_costmap_layers::TSProjectionLayer,
  nav2_costmap_2d::Layer)
