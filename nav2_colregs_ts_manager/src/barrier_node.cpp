#include "nav2_colregs_ts_manager/barrier_node.hpp"

#include <cmath>
#include <string>

namespace nav2_colregs_ts_manager
{

BarrierNode::BarrierNode()
: rclcpp::Node("barrier_node")
{
  declare_parameter("ray_length", 999.0);
  declare_parameter("os_radius", 0.3);
  ray_length_ = get_parameter("ray_length").as_double();
  os_radius_ = get_parameter("os_radius").as_double();

  ts_list_sub_ = create_subscription<nav2_colregs_msgs::msg::ProcessedTSList>(
    "processed_ts_list", rclcpp::SystemDefaultsQoS(),
    std::bind(&BarrierNode::tsListCallback, this, std::placeholders::_1));

  service_ = create_service<nav2_colregs_msgs::srv::GetBarrierLines>(
    "get_barrier_lines",
    std::bind(&BarrierNode::handleService, this,
              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  barrier_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("barrier_markers", 10);

  RCLCPP_INFO(get_logger(), "BarrierNode started (ray_length=%.1f, os_radius=%.2f)",
    ray_length_, os_radius_);
}

// ---------------------------------------------------------------------------
// Topic callback
// ---------------------------------------------------------------------------

void BarrierNode::tsListCallback(
  nav2_colregs_msgs::msg::ProcessedTSList::ConstSharedPtr msg)
{
  last_ts_list_ = msg;
}

// ---------------------------------------------------------------------------
// Service
// ---------------------------------------------------------------------------

void BarrierNode::handleService(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<nav2_colregs_msgs::srv::GetBarrierLines::Request> request,
  const std::shared_ptr<nav2_colregs_msgs::srv::GetBarrierLines::Response> response)
{
  response->barriers.points.clear();

  if (!last_ts_list_ || last_ts_list_->ships.empty()) {
    return;
  }

  // Find the target ship by UUID.
  const nav2_colregs_msgs::msg::ProcessedTS * target = nullptr;
  for (const auto & ship : last_ts_list_->ships) {
    bool match = true;
    for (size_t i = 0; i < 16; ++i) {
      if (ship.target_id.uuid[i] != request->target_id.uuid[i]) {
        match = false;
        break;
      }
    }
    if (match) {
      target = &ship;
      break;
    }
  }

  if (!target) {
    RCLCPP_ERROR(get_logger(), "BarrierNode: target_id not found in cached TS list");
    return;
  }

  double os_x = request->os_pose.position.x;
  double os_y = request->os_pose.position.y;
  double ts_x = target->pose.position.x;
  double ts_y = target->pose.position.y;
  double ts_r = target->radius;

  generateBarrierLines(os_x, os_y, ts_x, ts_y, ts_r,
                       request->avoid_direction, response->barriers);

  // Visualize barrier lines.
  if (!response->barriers.points.empty()) {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker lines;
    lines.header.frame_id = "map";
    lines.header.stamp = now();
    lines.ns = "barrier";
    lines.id = 0;
    lines.type = visualization_msgs::msg::Marker::LINE_LIST;
    lines.action = visualization_msgs::msg::Marker::ADD;
    lines.scale.x = 0.05;
    lines.color.r = 0.9;  lines.color.g = 0.2;  lines.color.b = 0.2;  lines.color.a = 0.8;
    lines.lifetime.sec = 0;
    lines.points = response->barriers.points;
    markers.markers.push_back(lines);
    barrier_markers_pub_->publish(markers);
  }
}

// ---------------------------------------------------------------------------
// Barrier geometry — U-shaped 3-segment barrier around a single TS.
// ---------------------------------------------------------------------------

void BarrierNode::generateBarrierLines(
  double os_x, double os_y,
  double ts_x, double ts_y, double ts_r,
  const std::string & avoid_direction,
  nav2_colregs_msgs::msg::VOBarrierLines & barriers)
{
  // Bearing from OS to TS.
  double bearing = std::atan2(ts_y - os_y, ts_x - os_x);

  // Perpendicular direction (±90° based on avoid direction).
  double perp = bearing;
  if (avoid_direction == "right") {
    perp += M_PI_2;  // starboard side
  } else {
    perp -= M_PI_2;  // port side
  }

  const double line1_len = os_radius_ + ts_r;
  const double dist_os_ts = std::hypot(ts_x - os_x, ts_y - os_y);
  const double line2_len = std::max(dist_os_ts, 10.0) + 3.0 * ts_r;

  geometry_msgs::msg::Point p0, p1, p2, p3, p4, p5;

  // Segment 1: from TS, perpendicular to OS→TS bearing.
  p0.x = ts_x;  p0.y = ts_y;  p0.z = 0.0;
  p1.x = ts_x + line1_len * std::cos(perp);
  p1.y = ts_y + line1_len * std::sin(perp);
  p1.z = 0.0;

  // Segment 2: anti-parallel to OS→TS bearing (extending away from OS).
  p2 = p1;
  p3.x = p1.x - line2_len * std::cos(bearing);
  p3.y = p1.y - line2_len * std::sin(bearing);
  p3.z = 0.0;

  // Segment 3: outward ray (substituted by long segment).
  p4 = p3;
  p5.x = p3.x + ray_length_ * std::cos(perp);
  p5.y = p3.y + ray_length_ * std::sin(perp);
  p5.z = 0.0;

  barriers.points = {p0, p1, p2, p3, p4, p5};
}

}  // namespace nav2_colregs_ts_manager
