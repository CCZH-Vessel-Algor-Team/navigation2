#include "nav2_colregs_ts_manager/avoidance_point.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nav2_colregs_ts_manager
{

AvoidancePointNode::AvoidancePointNode()
: rclcpp::Node("avoidance_point_node")
{
  ts_list_sub_ = create_subscription<nav2_colregs_msgs::msg::ProcessedTSList>(
    "processed_ts_list", rclcpp::SystemDefaultsQoS(),
    std::bind(&AvoidancePointNode::tsListCallback, this, std::placeholders::_1));

  service_ = create_service<nav2_colregs_msgs::srv::GetAvoidancePoint>(
    "get_avoidance_point",
    std::bind(&AvoidancePointNode::handleService, this,
              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("avoidance_point_marker", 10);

  RCLCPP_INFO(get_logger(), "AvoidancePointNode started");
}

// ---------------------------------------------------------------------------
// Topic callback
// ---------------------------------------------------------------------------

void AvoidancePointNode::tsListCallback(
  nav2_colregs_msgs::msg::ProcessedTSList::ConstSharedPtr msg)
{
  last_ts_list_ = msg;
}

// ---------------------------------------------------------------------------
// Service
// ---------------------------------------------------------------------------

void AvoidancePointNode::handleService(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<nav2_colregs_msgs::srv::GetAvoidancePoint::Request> request,
  const std::shared_ptr<nav2_colregs_msgs::srv::GetAvoidancePoint::Response> response)
{
  response->has_feasible_angle = false;
  response->safe_heading = 0.0;

  if (!last_ts_list_ || last_ts_list_->ships.empty()) {
    return;
  }

  int idx = selectPrimary(*last_ts_list_);
  if (idx < 0) {
    return;
  }
  const auto & ts = last_ts_list_->ships[idx];
  response->primary_target_id = ts.target_id;

  double os_x = request->os_pose.position.x;
  double os_y = request->os_pose.position.y;
  double goal_x = request->goal.position.x;
  double goal_y = request->goal.position.y;

  double safe_heading = 0.0;
  bool found = findSafeHeading(ts, request->avoid_direction,
                               goal_x, goal_y, os_x, os_y, safe_heading);
  if (!found) {
    response->has_feasible_angle = false;
    return;
  }

  double ts_x = ts.pose.position.x;
  double ts_y = ts.pose.position.y;
  double dist = std::hypot(ts_x - os_x, ts_y - os_y);

  response->point.x = os_x + dist * std::cos(safe_heading);
  response->point.y = os_y + dist * std::sin(safe_heading);
  response->point.z = 0.0;
  response->safe_heading = safe_heading;
  response->has_feasible_angle = true;

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker arrow;
  arrow.header.frame_id = "map";
  arrow.header.stamp = now();
  arrow.ns = "avoidance";
  arrow.id = 0;
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.action = visualization_msgs::msg::Marker::ADD;
  arrow.points.resize(2);
  arrow.points[0].x = os_x;  arrow.points[0].y = os_y;
  arrow.points[1].x = response->point.x;  arrow.points[1].y = response->point.y;
  arrow.scale.x = 0.1;  arrow.scale.y = 0.2;  arrow.scale.z = 0.2;
  arrow.color.r = 0.2;  arrow.color.g = 0.8;  arrow.color.b = 0.2;  arrow.color.a = 0.8;
  arrow.lifetime.sec = 0;
  markers.markers.push_back(arrow);
  marker_pub_->publish(markers);
}

// ---------------------------------------------------------------------------
// Primary threat selection
// ---------------------------------------------------------------------------

int AvoidancePointNode::selectPrimary(
  const nav2_colregs_msgs::msg::ProcessedTSList & list)
{
  int best = -1;
  double best_tcpa = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < list.ships.size(); ++i) {
    const auto & ts = list.ships[i];
    if (ts.has_threat && ts.tcpa < best_tcpa) {
      best_tcpa = ts.tcpa;
      best = static_cast<int>(i);
    }
  }
  return best;
}

// ---------------------------------------------------------------------------
// Safe heading from collision_cone complement
// ---------------------------------------------------------------------------

namespace {

double wrapAngle(double a)
{
  a = std::fmod(a, 2.0 * M_PI);
  if (a < 0.0) a += 2.0 * M_PI;
  return a;
}

}  // namespace

bool AvoidancePointNode::findSafeHeading(
  const nav2_colregs_msgs::msg::ProcessedTS & ts,
  const std::string & avoid_direction,
  double goal_x, double goal_y,
  double os_x, double os_y,
  double & safe_heading)
{
  const auto & mins = ts.collision_cone_min;
  const auto & maxs = ts.collision_cone_max;

  // Inescapable: [[0, 2π]].
  if (mins.size() == 1 && maxs.size() == 1 &&
      mins[0] < 1e-6 && std::abs(maxs[0] - 2.0 * M_PI) < 1e-6)
  {
    return false;
  }

  // No cone → all headings safe.
  if (mins.empty()) {
    safe_heading = std::atan2(goal_y - os_y, goal_x - os_x);
    return true;
  }

  // Build safe intervals = complement of all unsafe intervals.
  std::vector<std::pair<double, double>> safe;
  double cursor = 0.0;

  std::vector<std::pair<double, double>> unsafe;
  for (size_t i = 0; i < mins.size(); ++i) {
    unsafe.emplace_back(mins[i], maxs[i]);
  }
  std::sort(unsafe.begin(), unsafe.end());

  for (const auto & u : unsafe) {
    if (u.first > cursor + 1e-6) {
      safe.emplace_back(cursor, u.first);
    }
    cursor = std::max(cursor, u.second);
  }
  if (cursor < 2.0 * M_PI - 1e-6) {
    safe.emplace_back(cursor, 2.0 * M_PI);
  }

  if (safe.empty()) {
    return false;
  }

  double goal_angle = wrapAngle(std::atan2(goal_y - os_y, goal_x - os_x));

  for (const auto & s : safe) {
    if (goal_angle >= s.first && goal_angle <= s.second) {
      safe_heading = goal_angle;
      return true;
    }
  }

  // ENU: CCW is +. "right" = CW = decreasing angle → scan backward from goal.
  if (avoid_direction == "right") {
    for (auto it = safe.rbegin(); it != safe.rend(); ++it) {
      if (it->second < goal_angle) {
        safe_heading = it->second;
        return true;
      }
    }
    safe_heading = safe.back().second;
  } else {
    for (const auto & s : safe) {
      if (s.first > goal_angle) {
        safe_heading = s.first;
        return true;
      }
    }
    safe_heading = safe.front().first;
  }
  return true;
}

}  // namespace nav2_colregs_ts_manager
