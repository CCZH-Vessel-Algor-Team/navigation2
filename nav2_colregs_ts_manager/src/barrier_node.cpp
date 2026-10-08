#include "nav2_colregs_ts_manager/barrier_node.hpp"
#include "nav2_colregs_ts_manager/parameter_contract.hpp"
#include "nav2_colregs_ts_manager/decision_geometry.hpp"

#include <cmath>
#include <string>

namespace nav2_colregs_ts_manager
{

BarrierNode::BarrierNode()
: rclcpp::Node("barrier_node")
{
  declare_parameter("closing_segment_length", 999.0,
    parameterDescription("Third barrier segment length [m].", true));
  declare_parameter("lateral_margin", 0.3,
    parameterDescription("Additional lateral offset beyond the TS radius [m].", true));
  closing_segment_length_ = get_parameter("closing_segment_length").as_double();
  lateral_margin_ = get_parameter("lateral_margin").as_double();
  validateNumber("closing_segment_length", closing_segment_length_, true);
  validateNumber("lateral_margin", lateral_margin_);
  snapshot_timeout_ = declare_parameter("snapshot_timeout", 1.0,
    parameterDescription("Maximum decision snapshot age [s].", true));
  max_request_position_delta_ = declare_parameter("max_request_position_delta", 3.0,
    parameterDescription("Maximum request/snapshot OS XY position difference [m].", true));
  validateNumber("snapshot_timeout", snapshot_timeout_, true);
  validateNumber("max_request_position_delta", max_request_position_delta_);

  ts_list_sub_ = create_subscription<nav2_colregs_msgs::msg::ProcessedTSList>(
    "processed_ts_list", rclcpp::SystemDefaultsQoS(),
    std::bind(&BarrierNode::tsListCallback, this, std::placeholders::_1));

  service_ = create_service<nav2_colregs_msgs::srv::GetBarrierLines>(
    "get_barrier_lines",
    std::bind(&BarrierNode::handleService, this,
              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  barrier_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("barrier_markers", 10);
  expiry_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {
      if (!last_ts_list_ || !validSnapshot(*last_ts_list_) ||
        !fresh(last_ts_list_->header.stamp, now(), snapshot_timeout_))
      {
        clearMarkers();
      }
    });

  RCLCPP_INFO(get_logger(), "BarrierNode started (closing_segment_length=%.1f, lateral_margin=%.2f)",
    closing_segment_length_, lateral_margin_);
}

// ---------------------------------------------------------------------------
// Topic callback
// ---------------------------------------------------------------------------

void BarrierNode::tsListCallback(
  nav2_colregs_msgs::msg::ProcessedTSList::ConstSharedPtr msg)
{
  if (last_ts_list_ && fresh(last_ts_list_->header.stamp, now(), snapshot_timeout_) &&
    rclcpp::Time(msg->header.stamp) < rclcpp::Time(last_ts_list_->header.stamp))
  {
    return;
  }
  last_ts_list_ = msg;
  snapshots_.push_back(msg);
  while (snapshots_.size() > 64) {
    snapshots_.pop_front();
  }
  if (!validSnapshot(*msg) || std::none_of(msg->ships.begin(), msg->ships.end(),
    [](const auto & ship) {return ship.has_threat;}))
  {
    clearMarkers();
  }
}

void BarrierNode::clearMarkers()
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  barrier_markers_pub_->publish(markers);
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
  using Response = nav2_colregs_msgs::srv::GetBarrierLines::Response;
  auto fail = [&](uint8_t status, const std::string & message) {
      response->status = status;
      response->message = message;
      response->barriers.points.clear();
      clearMarkers();
    };
  if (!last_ts_list_) {
    fail(Response::NO_DATA, "No processed snapshot received");
    return;
  }
  if (!validSnapshot(*last_ts_list_) ||
    !fresh(last_ts_list_->header.stamp, now(), snapshot_timeout_))
  {
    fail(Response::STALE_STATE, "Latest TS state is invalid or expired");
    return;
  }
  nav2_colregs_msgs::msg::ProcessedTSList::ConstSharedPtr snapshot;
  for (const auto & cached : snapshots_) {
    if (cached->snapshot_id == request->snapshot_id) {
      snapshot = cached;
      break;
    }
  }
  if (!snapshot) {
    fail(Response::SNAPSHOT_MISMATCH, "Avoidance snapshot is not in the barrier cache");
    return;
  }
  if (!validSnapshot(*snapshot) || !fresh(snapshot->header.stamp, now(), snapshot_timeout_)) {
    fail(Response::STALE_STATE, "Requested avoidance snapshot has expired");
    return;
  }
  if (request->header.frame_id != snapshot->header.frame_id ||
    !finitePoint(request->os_pose.position) ||
    (request->avoid_direction != "right" && request->avoid_direction != "left") ||
    std::hypot(request->os_pose.position.x - snapshot->os_pose.position.x,
    request->os_pose.position.y - snapshot->os_pose.position.y) > max_request_position_delta_)
  {
    fail(Response::INVALID_REQUEST, "Invalid frame, direction or non-anchored OS pose");
    return;
  }
  if (std::none_of(last_ts_list_->ships.begin(), last_ts_list_->ships.end(),
    [&](const auto & ship) {return ship.target_id == request->target_id;}))
  {
    fail(Response::TARGET_NOT_FOUND, "Target was removed from the latest snapshot");
    return;
  }
  // Find the target ship by UUID.
  const nav2_colregs_msgs::msg::ProcessedTS * target = nullptr;
  for (const auto & ship : snapshot->ships) {
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
    fail(Response::TARGET_NOT_FOUND, "Target does not exist in the requested snapshot");
    return;
  }

  double os_x = request->os_pose.position.x;
  double os_y = request->os_pose.position.y;
  double ts_x = target->pose.position.x;
  double ts_y = target->pose.position.y;
  double ts_r = target->radius;

  generateBarrierLines(os_x, os_y, ts_x, ts_y, ts_r,
                       request->avoid_direction, response->barriers);
  if (std::any_of(response->barriers.points.begin(), response->barriers.points.end(),
    [](const auto & p) {return !finitePoint(p);}))
  {
    fail(Response::INVALID_REQUEST, "Barrier geometry is not finite");
    return;
  }
  response->status = Response::SUCCESS;
  response->message = "Barrier uses the exact avoidance snapshot";
  response->header = snapshot->header;
  response->snapshot_id = snapshot->snapshot_id;

  // Visualize barrier lines.
  if (!response->barriers.points.empty()) {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker lines;
    lines.header.frame_id = snapshot->header.frame_id;
    lines.header.stamp = now();
    lines.ns = "barrier";
    lines.id = 0;
    lines.type = visualization_msgs::msg::Marker::LINE_LIST;
    lines.action = visualization_msgs::msg::Marker::ADD;
    lines.scale.x = 0.05;
    lines.color.r = 0.9;  lines.color.g = 0.2;  lines.color.b = 0.2;  lines.color.a = 0.8;
    lines.lifetime = rclcpp::Duration::from_seconds(snapshot_timeout_);
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
    perp += M_PI_2;  // port side blockade → forces starboard passing
  } else {
    perp -= M_PI_2;  // starboard side blockade → forces port passing
  }

  const double line1_len = lateral_margin_ + ts_r;
  const double dist_os_ts = std::hypot(ts_x - os_x, ts_y - os_y);
  const double line2_len = std::max(dist_os_ts, 10.0) + 3.0 * ts_r;

  geometry_msgs::msg::Point p0, p1, p2, p3, p4, p5;

  // Segment 1: from TS, perpendicular to OS→TS bearing.
  p0.x = ts_x;  p0.y = ts_y;  p0.z = 0.0;
  p1.x = ts_x + line1_len * std::cos(perp);
  p1.y = ts_y + line1_len * std::sin(perp);
  p1.z = 0.0;

  // Segment 2: anti-parallel to OS→TS bearing (toward and possibly past OS).
  p2 = p1;
  p3.x = p1.x - line2_len * std::cos(bearing);
  p3.y = p1.y - line2_len * std::sin(bearing);
  p3.z = 0.0;

  // Segment 3: opposite perpendicular (closes the U-shape).
  p4 = p3;
  p5.x = p3.x - closing_segment_length_ * std::cos(perp);
  p5.y = p3.y - closing_segment_length_ * std::sin(perp);
  p5.z = 0.0;

  barriers.points = {p0, p1, p2, p3, p4, p5};
}

}  // namespace nav2_colregs_ts_manager
