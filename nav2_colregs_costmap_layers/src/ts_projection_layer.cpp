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

  declareParameter("track_timeout", rclcpp::ParameterValue(3.0));
  track_timeout_ = node->get_parameter(name_ + "." + "track_timeout").as_double();

  sub_ = node->create_subscription<nav2_colregs_msgs::msg::TrackedShipList>(
    "/tracked_ship", rclcpp::SystemDefaultsQoS(),
    std::bind(&TSProjectionLayer::trackedShipCallback, this, std::placeholders::_1));

  global_frame_ = layered_costmap_->getGlobalFrameID();
  current_ = true;
  RCLCPP_INFO(logger_, "TSProjectionLayer initialized, subscribed to /tracked_ship "
    "(track_timeout=%.1fs)", track_timeout_);
}

void TSProjectionLayer::trackedShipCallback(
  nav2_colregs_msgs::msg::TrackedShipList::ConstSharedPtr msg)
{
  const auto now = node_.lock()->get_clock()->now();
  tf_frame_ = msg->header.frame_id;

  for (const auto & ship : msg->ships) {
    const auto key = uuidToString(ship.target_id.uuid.data());
    auto it = ships_.find(key);

    if (it != ships_.end()) {
      // Expand cumulative bounds with OLD position (clear trailing residue),
      // then update position/radius while preserving the accumulated bounds.
      ShipEntry & entry = it->second;
      if (!entry.has_cumulative_bounds) {
        entry.cum_min_x = entry.x - entry.radius;
        entry.cum_min_y = entry.y - entry.radius;
        entry.cum_max_x = entry.x + entry.radius;
        entry.cum_max_y = entry.y + entry.radius;
        entry.has_cumulative_bounds = true;
      } else {
        entry.cum_min_x = std::min(entry.cum_min_x, entry.x - entry.radius);
        entry.cum_min_y = std::min(entry.cum_min_y, entry.y - entry.radius);
        entry.cum_max_x = std::max(entry.cum_max_x, entry.x + entry.radius);
        entry.cum_max_y = std::max(entry.cum_max_y, entry.y + entry.radius);
      }
      entry.x = ship.pose.position.x;
      entry.y = ship.pose.position.y;
      entry.radius = ship.radius;
      entry.last_seen = now;
    } else {
      // New ship: initialize cumulative bounds with current position.
      ShipEntry e;
      e.x = ship.pose.position.x;
      e.y = ship.pose.position.y;
      e.radius = ship.radius;
      e.last_seen = now;
      e.cum_min_x = e.x - e.radius;
      e.cum_min_y = e.y - e.radius;
      e.cum_max_x = e.x + e.radius;
      e.cum_max_y = e.y + e.radius;
      e.has_cumulative_bounds = true;
      ships_[key] = e;
    }
  }

  // Purge stale entries.
  for (auto it = ships_.begin(); it != ships_.end(); ) {
    if ((now - it->second.last_seen).seconds() > track_timeout_) {
      it = ships_.erase(it);
    } else {
      ++it;
    }
  }
}

geometry_msgs::msg::TransformStamped TSProjectionLayer::lookupTransform(
  const std::string & from_frame)
{
  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_->lookupTransform(global_frame_, from_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    // Return identity transform on failure.
    t.transform.rotation.w = 1.0;
  }
  return t;
}

void TSProjectionLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (ships_.empty()) {
    return;
  }

  auto t = lookupTransform(tf_frame_);

  for (auto & entry_pair : ships_) {
    auto & entry = entry_pair.second;

    // Transform cumulative bounds (OLD + NEW positions) to costmap frame.
    geometry_msgs::msg::PointStamped c1, c2, t1, t2;
    c1.header.frame_id = tf_frame_;
    t1.header.frame_id = global_frame_;
    c2.header = c1.header;
    t2.header = t1.header;

    c1.point.x = entry.cum_min_x;
    c1.point.y = entry.cum_min_y;
    c2.point.x = entry.cum_max_x;
    c2.point.y = entry.cum_max_y;
    try {
      tf2::doTransform(c1, t1, t);
      tf2::doTransform(c2, t2, t);
      *min_x = std::min(*min_x, std::min(t1.point.x, t2.point.x));
      *min_y = std::min(*min_y, std::min(t1.point.y, t2.point.y));
      *max_x = std::max(*max_x, std::max(t1.point.x, t2.point.x));
      *max_y = std::max(*max_y, std::max(t1.point.y, t2.point.y));
    } catch (const tf2::TransformException &) { /* skip */ }
    entry.has_cumulative_bounds = false;  // consumed

    // Include current position.
    c1.point.x = entry.x - entry.radius;
    c1.point.y = entry.y - entry.radius;
    c2.point.x = entry.x + entry.radius;
    c2.point.y = entry.y + entry.radius;
    try {
      tf2::doTransform(c1, t1, t);
      tf2::doTransform(c2, t2, t);
      *min_x = std::min(*min_x, std::min(t1.point.x, t2.point.x));
      *min_y = std::min(*min_y, std::min(t1.point.y, t2.point.y));
      *max_x = std::max(*max_x, std::max(t1.point.x, t2.point.x));
      *max_y = std::max(*max_y, std::max(t1.point.y, t2.point.y));
    } catch (const tf2::TransformException &) { /* skip */ }
  }
}

void TSProjectionLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  if (!enabled_ || ships_.empty()) {
    return;
  }

  auto t = lookupTransform(tf_frame_);

  for (const auto & entry_pair : ships_) {
    const auto & entry = entry_pair.second;

    geometry_msgs::msg::PointStamped ts_in, ts_cf;
    ts_in.header.frame_id = tf_frame_;
    ts_in.point.x = entry.x;
    ts_in.point.y = entry.y;
    try {
      tf2::doTransform(ts_in, ts_cf, t);
    } catch (const tf2::TransformException &) {
      continue;
    }

    const double ts_x = ts_cf.point.x;
    const double ts_y = ts_cf.point.y;
    const double ts_r = entry.radius;

    unsigned int mx, my;
    if (!master_grid.worldToMap(ts_x, ts_y, mx, my)) {
      continue;
    }

    const unsigned int radius_in_cells =
      static_cast<unsigned int>(std::ceil(ts_r / master_grid.getResolution()));
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
}

void TSProjectionLayer::reset()
{
  ships_.clear();
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
