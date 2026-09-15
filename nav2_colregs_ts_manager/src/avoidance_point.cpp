#include "nav2_colregs_ts_manager/avoidance_point.hpp"
#include "nav2_colregs_ts_manager/parameter_contract.hpp"
#include "nav2_colregs_ts_manager/decision_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nav2_colregs_ts_manager
{

AvoidancePointNode::AvoidancePointNode()
: rclcpp::Node("avoidance_point_node")
{
  declare_parameter("avoidance_radius_scale", 1.5,
    parameterDescription("Radius inflation >= 1 for candidate heading collision checks.", false));
  declare_parameter("point_extension_distance", 20.0,
    parameterDescription("Point range beyond current OS-TS distance [m], independent of inflation.",
      false));
  validateRadiusScale("avoidance_radius_scale", get_parameter("avoidance_radius_scale").as_double());
  validateNumber("point_extension_distance", get_parameter("point_extension_distance").as_double());
  snapshot_timeout_ = declare_parameter("snapshot_timeout", 1.0,
    parameterDescription("Maximum decision snapshot age [s].", true));
  max_request_position_delta_ = declare_parameter("max_request_position_delta", 3.0,
    parameterDescription("Maximum request/snapshot OS XY position difference [m].", true));
  validateNumber("snapshot_timeout", snapshot_timeout_, true);
  validateNumber("max_request_position_delta", max_request_position_delta_);
  parameter_callback_ = add_on_set_parameters_callback(
    [](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      try {
        for (const auto & parameter : parameters) {
          if (parameter.get_name() == "avoidance_radius_scale") {
            validateRadiusScale(parameter.get_name(), parameter.as_double());
          } else if (parameter.get_name() == "point_extension_distance")
          {
            validateNumber(parameter.get_name(), parameter.as_double());
          }
        }
      } catch (const std::exception & ex) {
        result.successful = false;
        result.reason = ex.what();
      }
      return result;
    });

  ts_list_sub_ = create_subscription<nav2_colregs_msgs::msg::ProcessedTSList>(
    "processed_ts_list", rclcpp::SystemDefaultsQoS(),
    std::bind(&AvoidancePointNode::tsListCallback, this, std::placeholders::_1));

  service_ = create_service<nav2_colregs_msgs::srv::GetAvoidancePoint>(
    "get_avoidance_point",
    std::bind(&AvoidancePointNode::handleService, this,
              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("avoidance_point_marker", 10);
  expiry_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {
      if (!last_ts_list_ || !validSnapshot(*last_ts_list_) ||
        !fresh(last_ts_list_->header.stamp, now(), snapshot_timeout_))
      {
        clearMarkers();
      }
    });

  RCLCPP_INFO(get_logger(), "AvoidancePointNode started");
  RCLCPP_INFO(get_logger(),
    "Avoidance effective parameters: os_radius=ProcessedTSList.os_radius "
    "avoidance_radius_scale=%.3f point_extension_distance=%.3fm",
    get_parameter("avoidance_radius_scale").as_double(),
    get_parameter("point_extension_distance").as_double());
}

// ---------------------------------------------------------------------------
// Topic callback
// ---------------------------------------------------------------------------

void AvoidancePointNode::tsListCallback(
  nav2_colregs_msgs::msg::ProcessedTSList::ConstSharedPtr msg)
{
  if (last_ts_list_ && fresh(last_ts_list_->header.stamp, now(), snapshot_timeout_) &&
    rclcpp::Time(msg->header.stamp) < rclcpp::Time(last_ts_list_->header.stamp))
  {
    return;
  }
  last_ts_list_ = msg;
  if (!validSnapshot(*msg) || std::none_of(msg->ships.begin(), msg->ships.end(),
    [](const auto & ship) {return ship.has_threat;}))
  {
    clearMarkers();
  }
}

void AvoidancePointNode::clearMarkers()
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  marker_pub_->publish(markers);
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
  using Response = nav2_colregs_msgs::srv::GetAvoidancePoint::Response;
  auto fail = [&](uint8_t status, const std::string & message) {
      response->status = status;
      response->message = message;
      clearMarkers();
    };
  if (!last_ts_list_) {
    fail(Response::NO_DATA, "No processed TS snapshot received");
    return;
  }
  const auto & state = *last_ts_list_;
  response->header = state.header;
  response->snapshot_id = state.snapshot_id;
  if (!validSnapshot(state) || !fresh(state.header.stamp, now(), snapshot_timeout_)) {
    fail(Response::STALE_STATE, "Invalid or expired snapshot: " + state.status_message);
    return;
  }
  if (request->header.frame_id != state.header.frame_id ||
    !finitePoint(request->os_pose.position) || !finitePoint(request->goal.position) ||
    (request->avoid_direction != "right" && request->avoid_direction != "left"))
  {
    fail(Response::INVALID_REQUEST, "Invalid frame, coordinates or avoidance direction");
    return;
  }
  const double os_x = request->os_pose.position.x;
  const double os_y = request->os_pose.position.y;
  if (std::hypot(os_x - state.os_pose.position.x,
    os_y - state.os_pose.position.y) > max_request_position_delta_)
  {
    fail(Response::INVALID_REQUEST, "Request is not anchored near the snapshot OS pose");
    return;
  }
  const auto parameters = get_parameters({"avoidance_radius_scale", "point_extension_distance"});
  const double avoidance_radius_scale = parameters[0].as_double();
  const double extension = parameters[1].as_double();
  int idx = selectPrimary(state, os_x, os_y, avoidance_radius_scale);
  if (idx < 0) {
    fail(Response::NO_THREAT, "Fresh snapshot contains no qualifying threat");
    return;
  }
  const auto & ts = state.ships[idx];
  response->primary_target_id = ts.target_id;

  double goal_x = request->goal.position.x;
  double goal_y = request->goal.position.y;

  double safe_heading = 0.0;
  bool found = findSafeHeading(state, request->avoid_direction,
                               goal_x, goal_y, os_x, os_y, avoidance_radius_scale, safe_heading);
  if (!found) {
    fail(Response::INESCAPABLE,
      "No heading satisfies the inflated collision radii at the current own-ship speed");
    return;
  }

  double ts_x = ts.pose.position.x;
  double ts_y = ts.pose.position.y;
  double dist = std::hypot(ts_x - os_x, ts_y - os_y);
  double point_range = dist + extension;

  response->point.x = os_x + point_range * std::cos(safe_heading);
  response->point.y = os_y + point_range * std::sin(safe_heading);
  response->point.z = 0.0;
  if (!finitePoint(response->point)) {
    fail(Response::INVALID_REQUEST, "Avoidance point is not finite");
    return;
  }
  response->safe_heading = safe_heading;
  response->has_feasible_angle = true;
  response->status = Response::SUCCESS;
  response->message = "Heading validated using inflated OS/TS radii (factor=" +
    std::to_string(avoidance_radius_scale) + ")";

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker arrow;
  arrow.header.frame_id = state.header.frame_id;
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
  arrow.lifetime = rclcpp::Duration::from_seconds(snapshot_timeout_);
  markers.markers.push_back(arrow);
  marker_pub_->publish(markers);
}

// ---------------------------------------------------------------------------
// Primary threat selection
// ---------------------------------------------------------------------------

int AvoidancePointNode::selectPrimary(
  const nav2_colregs_msgs::msg::ProcessedTSList & list, double ox, double oy,
  double avoidance_radius_scale)
{
  int best = -1;
  double best_tcpa = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < list.ships.size(); ++i) {
    const auto & ts = list.ships[i];
    const bool overlap = std::hypot(ts.pose.position.x - ox, ts.pose.position.y - oy) <=
      avoidance_radius_scale * (list.os_radius + ts.radius);
    const double score = overlap || !std::isfinite(ts.tcpa) ? 0.0 : std::max(0.0, ts.tcpa);
    if ((ts.has_threat || overlap) && score < best_tcpa) {
      best_tcpa = score;
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
  const nav2_colregs_msgs::msg::ProcessedTSList & state,
  const std::string & avoid_direction,
  double goal_x, double goal_y,
  double os_x, double os_y,
  double avoidance_radius_scale,
  double & safe_heading)
{
  const double goal_angle = std::atan2(goal_y - os_y, goal_x - os_x);
  const double direction = avoid_direction == "right" ? -1.0 : 1.0;
  // Sample candidates, but never infer safety from inclusive interval endpoints.
  // Validate each selected heading analytically against every current target.
  for (int i = 0; i < 180; ++i) {
    const double candidate = wrapAngle(goal_angle + direction * i * M_PI / 90.0);
    if (headingSafe(state, os_x, os_y, candidate, avoidance_radius_scale)) {
      safe_heading = candidate;
      return true;
    }
  }
  return false;
}

}  // namespace nav2_colregs_ts_manager
