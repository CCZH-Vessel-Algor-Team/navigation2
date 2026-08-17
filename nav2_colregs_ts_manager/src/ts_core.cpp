// Copyright (c) 2026 Vector Wang
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_colregs_ts_manager/ts_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace nav2_colregs_ts_manager
{

namespace
{

double wrapAngle(double a)
{
  a = std::fmod(a, 2.0 * M_PI);
  if (a < 0.0) {a += 2.0 * M_PI;}
  return a;
}

// LVO collision cone: 2-degree heading sweep (ported from TSStateManager).
void computeCollisionCone(
  const RawTsEntry & ts,
  double os_x, double os_y, double os_speed, double os_radius,
  std::vector<double> & min_intervals,
  std::vector<double> & max_intervals)
{
  min_intervals.clear();
  max_intervals.clear();

  const double rel_x = ts.x - os_x;
  const double rel_y = ts.y - os_y;
  const double dist = std::hypot(rel_x, rel_y);
  const double sum_r = os_radius + ts.radius;

  if (dist <= sum_r) {
    min_intervals.push_back(0.0);
    max_intervals.push_back(2.0 * M_PI);
    return;
  }

  const double threshold = std::asin(sum_r / dist);
  constexpr double kResolution = 2.0 * M_PI / 180.0;

  const int N = static_cast<int>(2.0 * M_PI / kResolution);
  std::vector<bool> unsafe(N, false);
  bool any_unsafe = false;

  for (int i = 0; i < N; ++i) {
    double heading = i * kResolution;
    double os_vx_h = os_speed * std::cos(heading);
    double os_vy_h = os_speed * std::sin(heading);
    double rvx = os_vx_h - ts.vx;
    double rvy = os_vy_h - ts.vy;
    double rv_len = std::hypot(rvx, rvy);

    if (rv_len < 1e-6) {
      if (dist < sum_r) {
        unsafe[i] = true;
        any_unsafe = true;
      }
      continue;
    }

    double dot = rel_x * rvx + rel_y * rvy;
    double cos_angle = dot / (dist * rv_len);
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
    double angle = std::acos(cos_angle);

    if (angle <= threshold) {
      unsafe[i] = true;
      any_unsafe = true;
    }
  }

  if (!any_unsafe) {
    return;
  }

  bool in_interval = false;
  double start = 0.0;

  for (int i = 0; i < N; ++i) {
    if (unsafe[i] && !in_interval) {
      start = i * kResolution;
      in_interval = true;
    }
    if (!unsafe[i] && in_interval) {
      min_intervals.push_back(start);
      max_intervals.push_back(i * kResolution);
      in_interval = false;
    }
  }
  if (in_interval) {
    min_intervals.push_back(start);
    max_intervals.push_back(2.0 * M_PI);
  }
}

// Primary threat selection: minimum TCPA among threats (ported from
// AvoidancePointNode::selectPrimary).
int selectPrimary(const TsSnapshot & snapshot)
{
  int best = -1;
  double best_tcpa = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < snapshot.ships.size(); ++i) {
    const auto & ts = snapshot.ships[i];
    if (ts.has_threat && ts.tcpa < best_tcpa) {
      best_tcpa = ts.tcpa;
      best = static_cast<int>(i);
    }
  }
  return best;
}

// Safe heading from the collision cone complement, biased toward the goal
// heading (ported from AvoidancePointNode::findSafeHeading).
bool findSafeHeading(
  const TsState & ts,
  const std::string & avoid_direction,
  double goal_x, double goal_y,
  double os_x, double os_y,
  double & safe_heading)
{
  const auto & mins = ts.cone_min;
  const auto & maxs = ts.cone_max;

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

// U-shaped three-segment barrier around one TS (ported from
// BarrierNode::generateBarrierLines).
std::vector<geometry_msgs::msg::Point> generateBarrierLines(
  double os_x, double os_y,
  double ts_x, double ts_y, double ts_r,
  const std::string & avoid_direction,
  const TsCoreParams & params)
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

  const double line1_len = params.os_radius + ts_r;
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

  // Segment 3: opposite perpendicular (closes the U-shape).
  p4 = p3;
  p5.x = p3.x - params.barrier_ray_length * std::cos(perp);
  p5.y = p3.y - params.barrier_ray_length * std::sin(perp);
  p5.z = 0.0;

  return {p0, p1, p2, p3, p4, p5};
}

}  // namespace

TsSnapshot processTs(
  const RawTsSnapshot & raw, const OsState & os, const TsCoreParams & params)
{
  TsSnapshot snapshot;
  snapshot.stamp = raw.stamp;

  const double os_vx = os.velocity_valid ? os.vx : 0.0;
  const double os_vy = os.velocity_valid ? os.vy : 0.0;
  const double os_speed = std::hypot(os_vx, os_vy);

  for (const auto & entry : raw.ships) {
    if ((raw.stamp - entry.last_seen).seconds() > params.ts_timeout) {
      continue;
    }

    const double rel_x = entry.x - os.x;
    const double rel_y = entry.y - os.y;
    const double rel_vx = entry.vx - os_vx;
    const double rel_vy = entry.vy - os_vy;

    const double rel_speed_sq = rel_vx * rel_vx + rel_vy * rel_vy;
    double tcpa = std::numeric_limits<double>::infinity();
    double dcpa = std::hypot(rel_x, rel_y);

    if (rel_speed_sq > 1e-6) {
      tcpa = -(rel_x * rel_vx + rel_y * rel_vy) / rel_speed_sq;
      dcpa = std::hypot(rel_x + rel_vx * tcpa, rel_y + rel_vy * tcpa);
    }

    const double safe_dist = (params.os_radius + entry.radius) * params.safety_factor;
    bool has_threat = (tcpa > 0.0 && tcpa <= params.tcpa_horizon && dcpa < safe_dist);

    TsState state;
    state.target_id = entry.target_id;
    state.x = entry.x;
    state.y = entry.y;
    state.radius = entry.radius;
    state.vx = entry.vx;
    state.vy = entry.vy;
    state.tcpa = tcpa;
    state.dcpa = dcpa;
    state.has_threat = has_threat;

    if (os_speed > 1e-6) {
      computeCollisionCone(
        entry, os.x, os.y, os_speed, params.os_radius,
        state.cone_min, state.cone_max);
    }

    snapshot.ships.push_back(state);
  }

  return snapshot;
}

ColregsDecision evaluateColregs(
  const TsSnapshot & snapshot, const OsState & os,
  double goal_x, double goal_y,
  const std::string & avoid_direction, const TsCoreParams & params)
{
  ColregsDecision decision;

  if (!os.velocity_valid) {
    return decision;
  }

  const int primary_idx = selectPrimary(snapshot);
  if (primary_idx < 0) {
    return decision;
  }
  decision.primary = snapshot.ships[primary_idx];

  double safe_heading = 0.0;
  const bool found = findSafeHeading(
    decision.primary, avoid_direction, goal_x, goal_y, os.x, os.y, safe_heading);
  if (!found) {
    return decision;
  }
  decision.safe_heading = safe_heading;

  const double dist = std::hypot(decision.primary.x - os.x, decision.primary.y - os.y);
  const double safe_dist =
    dist + (decision.primary.radius + params.os_radius) * params.safety_factor;
  decision.avoidance_point.x = os.x + safe_dist * std::cos(safe_heading);
  decision.avoidance_point.y = os.y + safe_dist * std::sin(safe_heading);
  decision.avoidance_point.z = 0.0;

  decision.barrier_points = generateBarrierLines(
    os.x, os.y, decision.primary.x, decision.primary.y,
    decision.primary.radius, avoid_direction, params);

  decision.active = true;
  return decision;
}

}  // namespace nav2_colregs_ts_manager
