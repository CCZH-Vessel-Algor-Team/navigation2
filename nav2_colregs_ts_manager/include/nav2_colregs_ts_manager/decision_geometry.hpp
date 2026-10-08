#ifndef NAV2_COLREGS_TS_MANAGER__DECISION_GEOMETRY_HPP_
#define NAV2_COLREGS_TS_MANAGER__DECISION_GEOMETRY_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_colregs_msgs/msg/processed_ts_list.hpp"

namespace nav2_colregs_ts_manager
{
inline bool fresh(const builtin_interfaces::msg::Time & stamp,
  const rclcpp::Time & now, double timeout)
{
  const double age = (now - rclcpp::Time(stamp, now.get_clock_type())).seconds();
  return age >= 0.0 && age <= timeout;
}

inline bool finitePoint(const geometry_msgs::msg::Point & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

// Constant-velocity disc collision over t >= 0. Tangency and overlap are unsafe.
// Heading changes cannot avoid a collision when own-ship speed is zero.
inline bool collisionCourse(double rx, double ry, double ux, double uy, double radius)
{
  const double radius_sq = radius * radius;
  if (!std::isfinite(radius_sq) || !std::isfinite(rx * rx + ry * ry) ||
    !std::isfinite(ux * ux + uy * uy))
  {
    return true;
  }
  if (rx * rx + ry * ry <= radius_sq + 1e-9) {
    return true;
  }
  const double speed_sq = ux * ux + uy * uy;
  if (speed_sq <= 1e-12) {
    return false;
  }
  const double time = -(rx * ux + ry * uy) / speed_sq;
  if (time <= 0.0) {
    return false;
  }
  const double dx = rx + ux * time;
  const double dy = ry + uy * time;
  return dx * dx + dy * dy <= radius_sq + 1e-9;
}

inline bool validSnapshot(const nav2_colregs_msgs::msg::ProcessedTSList & state)
{
  if (!state.valid || state.header.frame_id.empty() || !finitePoint(state.os_pose.position) ||
    !std::isfinite(state.os_twist.linear.x) || !std::isfinite(state.os_twist.linear.y) ||
    !std::isfinite(state.os_radius) || state.os_radius < 0.0 ||
    std::all_of(state.snapshot_id.uuid.begin(), state.snapshot_id.uuid.end(),
    [](uint8_t v) {return v == 0;}))
  {
    return false;
  }
  for (const auto & ship : state.ships) {
    if (!finitePoint(ship.pose.position) || !std::isfinite(ship.twist.linear.x) ||
      !std::isfinite(ship.twist.linear.y) || !std::isfinite(ship.radius) || ship.radius < 0.0 ||
      !std::isfinite(ship.dcpa) || ship.dcpa < 0.0 || std::isnan(ship.tcpa) ||
      ship.tcpa == -std::numeric_limits<double>::infinity())
    {
      return false;
    }
  }
  return true;
}

inline bool headingSafeAtSpeed(const nav2_colregs_msgs::msg::ProcessedTSList & state,
  double ox, double oy, double heading, double avoidance_radius_scale, double speed)
{
  for (const auto & ship : state.ships) {
    if (collisionCourse(ship.pose.position.x - ox, ship.pose.position.y - oy,
      ship.twist.linear.x - speed * std::cos(heading),
      ship.twist.linear.y - speed * std::sin(heading),
      avoidance_radius_scale * (state.os_radius + ship.radius)))
    {
      return false;
    }
  }
  return true;
}

inline bool headingSafe(const nav2_colregs_msgs::msg::ProcessedTSList & state,
  double ox, double oy, double heading, double avoidance_radius_scale)
{
  return headingSafeAtSpeed(state, ox, oy, heading, avoidance_radius_scale,
    std::hypot(state.os_twist.linear.x, state.os_twist.linear.y));
}

// Uniform interval samples plus the measured speed if it is not on the grid.
// This is a finite-sample check, not a continuous-interval certificate.
inline std::vector<double> sampleSpeeds(double speed, double tolerance, int count)
{
  std::vector<double> speeds{speed};
  if (tolerance == 0.0) {
    return speeds;
  }
  const double lower = std::max(0.0, speed - tolerance);
  const double upper = speed + tolerance;
  for (int i = 0; i < count; ++i) {
    const double value = lower + (upper - lower) * (static_cast<double>(i) / (count - 1));
    if (std::abs(value - speed) > 1e-9) {
      speeds.push_back(value);
    }
  }
  return speeds;
}

inline bool headingSafeForSpeeds(const nav2_colregs_msgs::msg::ProcessedTSList & state,
  double ox, double oy, double heading, double avoidance_radius_scale,
  const std::vector<double> & speeds)
{
  return std::all_of(speeds.begin(), speeds.end(), [&](double speed) {
      return headingSafeAtSpeed(state, ox, oy, heading, avoidance_radius_scale, speed);
    });
}
}  // namespace nav2_colregs_ts_manager
#endif  // NAV2_COLREGS_TS_MANAGER__DECISION_GEOMETRY_HPP_
