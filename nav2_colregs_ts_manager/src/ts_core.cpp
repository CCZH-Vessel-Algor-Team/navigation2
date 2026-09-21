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
#include <set>
#include <utility>

namespace nav2_colregs_ts_manager
{

namespace
{

constexpr int kHeadingSamples = 180;
constexpr double kHeadingStep = 2.0 * M_PI / kHeadingSamples;

double wrapAngle(double angle)
{
  angle = std::fmod(angle, 2.0 * M_PI);
  if (angle < 0.0) {angle += 2.0 * M_PI;}
  return angle;
}

bool validOs(const OsState & os)
{
  return os.velocity_valid && std::isfinite(os.x) && std::isfinite(os.y) &&
         std::isfinite(os.vx) && std::isfinite(os.vy) &&
         std::isfinite(std::hypot(os.vx, os.vy));
}

bool finitePoint(const geometry_msgs::msg::Point & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

// Preserve mathematical TCPA, but report minimum distance over future time only.
// Exact equal velocities have infinite TCPA; a small nonzero speed is not zero.
bool computeCpa(
  double rx, double ry, double vx, double vy, double & tcpa, double & dcpa)
{
  const double speed = std::hypot(vx, vy);
  dcpa = std::hypot(rx, ry);
  if (!std::isfinite(speed) || !std::isfinite(dcpa)) {
    return false;
  }
  tcpa = std::numeric_limits<double>::infinity();
  if (speed == 0.0) {
    return true;
  }
  tcpa = -(rx * (vx / speed) + ry * (vy / speed)) / speed;
  if (!std::isfinite(tcpa)) {
    return false;
  }
  const double future_time = std::max(0.0, tcpa);
  dcpa = std::hypot(rx + vx * future_time, ry + vy * future_time);
  return std::isfinite(dcpa);
}

// Tangency (including roundoff at the boundary) and current overlap are unsafe.
bool collisionCourse(
  double rx, double ry, double vx, double vy, double radius, bool & collision)
{
  double tcpa, dcpa;
  if (!std::isfinite(radius) || !computeCpa(rx, ry, vx, vy, tcpa, dcpa)) {
    return false;
  }
  const double tolerance = 32.0 * std::numeric_limits<double>::epsilon() *
    std::max(1.0, radius);
  collision = dcpa <= radius || dcpa - radius <= tolerance;
  return true;
}

// Diagnostic intervals only. Decisions independently check every candidate
// against every target, rather than treating sampled interval endpoints as safe.
bool computeCollisionCone(
  const TsState & ts, const OsState & os, const TsCoreParams & params,
  std::vector<double> & mins, std::vector<double> & maxs)
{
  const double speed = std::hypot(os.vx, os.vy);
  const double radius = params.threat_radius_scale * (params.os_radius + ts.radius);
  bool in_interval = false;
  for (int i = 0; i < kHeadingSamples; ++i) {
    const double heading = i * kHeadingStep;
    bool collision;
    if (!collisionCourse(
        ts.x - os.x, ts.y - os.y,
        ts.vx - speed * std::cos(heading), ts.vy - speed * std::sin(heading),
        radius, collision))
    {
      return false;
    }
    if (collision && !in_interval) {
      mins.push_back(heading);
      in_interval = true;
    } else if (!collision && in_interval) {
      maxs.push_back(heading);
      in_interval = false;
    }
  }
  if (in_interval) {
    maxs.push_back(2.0 * M_PI);
  }
  return true;
}

int selectPrimary(const TsSnapshot & snapshot, const OsState & os, const TsCoreParams & params)
{
  int best = -1;
  double best_score = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < snapshot.ships.size(); ++i) {
    const auto & ts = snapshot.ships[i];
    if (!ts.has_threat) {
      continue;
    }
    const bool intrusion = std::hypot(ts.x - os.x, ts.y - os.y) <=
      params.threat_radius_scale * (params.os_radius + ts.radius);
    const double score = intrusion || !std::isfinite(ts.tcpa) ? 0.0 :
      std::max(0.0, ts.tcpa);
    if (best < 0 || score < best_score ||
      (score == best_score && ts.target_id < snapshot.ships[best].target_id))
    {
      best = static_cast<int>(i);
      best_score = score;
    }
  }
  return best;
}

DecisionStatus decisionStatus(InputStatus status)
{
  switch (status) {
    case InputStatus::VALID: return DecisionStatus::SUCCESS;
    case InputStatus::NO_DATA: return DecisionStatus::NO_DATA;
    case InputStatus::STALE_STATE: return DecisionStatus::STALE_STATE;
    case InputStatus::INVALID_STATE: return DecisionStatus::INVALID_STATE;
    case InputStatus::INVALID_REQUEST: return DecisionStatus::INVALID_REQUEST;
  }
  return DecisionStatus::INVALID_STATE;
}

// The first barrier segment starts at the TS center and extends perpendicular
// to the OS->TS bearing, opposite the requested passing side.
std::vector<geometry_msgs::msg::Point> generateBarrierLines(
  double os_x, double os_y,
  double ts_x, double ts_y, double ts_r,
  const std::string & avoid_direction, const TsCoreParams & params)
{
  const double bearing = std::atan2(ts_y - os_y, ts_x - os_x);
  const double perp = bearing + (avoid_direction == "right" ? M_PI_2 : -M_PI_2);
  const double line1_len = ts_r + params.lateral_margin;
  const double line2_len = std::max(std::hypot(ts_x - os_x, ts_y - os_y), 10.0) +
    3.0 * ts_r;

  geometry_msgs::msg::Point p0, p1, p2, p3, p4, p5;
  p0.x = ts_x;  p0.y = ts_y;  p0.z = 0.0;
  p1.x = ts_x + line1_len * std::cos(perp);
  p1.y = ts_y + line1_len * std::sin(perp);
  p1.z = 0.0;
  p2 = p1;
  // Anti-parallel to OS->TS: toward and possibly past OS.
  p3.x = p1.x - line2_len * std::cos(bearing);
  p3.y = p1.y - line2_len * std::sin(bearing);
  p3.z = 0.0;
  p4 = p3;
  p5.x = p3.x - params.closing_segment_length * std::cos(perp);
  p5.y = p3.y - params.closing_segment_length * std::sin(perp);
  p5.z = 0.0;
  return {p0, p1, p2, p3, p4, p5};
}

}  // namespace

const char * inputStatusName(InputStatus status)
{
  switch (status) {
    case InputStatus::VALID: return "VALID";
    case InputStatus::NO_DATA: return "NO_DATA";
    case InputStatus::STALE_STATE: return "STALE_STATE";
    case InputStatus::INVALID_STATE: return "INVALID_STATE";
    case InputStatus::INVALID_REQUEST: return "INVALID_REQUEST";
  }
  return "UNKNOWN";
}

const char * decisionStatusName(DecisionStatus status)
{
  switch (status) {
    case DecisionStatus::SUCCESS: return "SUCCESS";
    case DecisionStatus::NO_THREAT: return "NO_THREAT";
    case DecisionStatus::NO_DATA: return "NO_DATA";
    case DecisionStatus::STALE_STATE: return "STALE_STATE";
    case DecisionStatus::INVALID_STATE: return "INVALID_STATE";
    case DecisionStatus::INVALID_REQUEST: return "INVALID_REQUEST";
    case DecisionStatus::INESCAPABLE: return "INESCAPABLE";
  }
  return "UNKNOWN";
}

bool validateCoreParams(const TsCoreParams & params, std::string & reason)
{
  reason.clear();
  const auto check = [&reason](const char * name, double value, double minimum, bool strict) {
      if (!std::isfinite(value) || (strict ? value <= minimum : value < minimum)) {
        reason = std::string(name) + " must be finite and " +
          (strict ? "greater than " : "at least ") + std::to_string(minimum);
        return false;
      }
      return true;
    };
  return check("track_list_timeout", params.track_list_timeout, 0.0, true) &&
         check("threat_tcpa_horizon", params.threat_tcpa_horizon, 0.0, false) &&
         check("threat_radius_scale", params.threat_radius_scale, 1.0, false) &&
         check("avoidance_radius_scale", params.avoidance_radius_scale, 1.0, false) &&
         check("os_radius", params.os_radius, 0.0, false) &&
         check("point_extension_distance", params.point_extension_distance, 0.0, false) &&
         check("lateral_margin", params.lateral_margin, 0.0, false) &&
         check("closing_segment_length", params.closing_segment_length, 0.0, true);
}

TsSnapshot processTs(
  const RawTsSnapshot & raw, const OsState & os, const TsCoreParams & params)
{
  TsSnapshot snapshot;
  snapshot.stamp = raw.stamp;
  snapshot.frame_id = raw.frame_id;
  const auto fail = [&snapshot](InputStatus status, const std::string & reason) {
      snapshot.status = status;
      snapshot.reason = reason;
      snapshot.ships.clear();
      return snapshot;
    };
  std::string reason;
  if (!validateCoreParams(params, reason)) {
    return fail(InputStatus::INVALID_REQUEST, reason);
  }
  if (raw.status != InputStatus::VALID) {
    return fail(raw.status, raw.reason.empty() ? inputStatusName(raw.status) : raw.reason);
  }
  if (!validOs(os) || raw.frame_id.empty() || raw.stamp.nanoseconds() < 0) {
    return fail(InputStatus::INVALID_STATE, "Invalid own-ship state, frame or calculation time");
  }

  std::set<std::string> ids;
  for (const auto & entry : raw.ships) {
    if (entry.target_id.empty() || !ids.insert(entry.target_id).second ||
      !std::isfinite(entry.x) || !std::isfinite(entry.y) ||
      !std::isfinite(entry.vx) || !std::isfinite(entry.vy) ||
      !std::isfinite(entry.radius) || entry.radius < 0.0 ||
      entry.last_seen.get_clock_type() != raw.stamp.get_clock_type() ||
      entry.last_seen.nanoseconds() < 0)
    {
      return fail(InputStatus::INVALID_STATE, "Invalid target observation: " + entry.target_id);
    }
    // Compare only matching clocks, before subtracting (no cross-clock exceptions).
    if (entry.last_seen > raw.stamp) {
      return fail(InputStatus::INVALID_STATE, "Future target measurement: " + entry.target_id);
    }
    if ((raw.stamp - entry.last_seen).seconds() > params.track_list_timeout) {
      return fail(InputStatus::STALE_STATE, "Expired target measurement: " + entry.target_id);
    }
    TsState state;
    state.target_id = entry.target_id;
    // ROS already projected these coordinates to raw.stamp. Do not extrapolate again.
    state.x = entry.x;
    state.y = entry.y;
    state.radius = entry.radius;
    state.vx = entry.vx;
    state.vy = entry.vy;
    const double distance = std::hypot(entry.x - os.x, entry.y - os.y);
    const double threat_radius = params.threat_radius_scale * (params.os_radius + entry.radius);
    const double avoidance_radius = params.avoidance_radius_scale *
      (params.os_radius + entry.radius);
    if (!std::isfinite(threat_radius) || !std::isfinite(avoidance_radius) ||
      !computeCpa(
        entry.x - os.x, entry.y - os.y, entry.vx - os.vx, entry.vy - os.vy,
        state.tcpa, state.dcpa))
    {
      return fail(InputStatus::INVALID_STATE, "Nonfinite target CPA or radius: " + entry.target_id);
    }
    state.has_threat = distance <= threat_radius ||
      (state.tcpa >= 0.0 && state.tcpa <= params.threat_tcpa_horizon &&
      state.dcpa < threat_radius);
    if (!computeCollisionCone(state, os, params, state.cone_min, state.cone_max)) {
      return fail(InputStatus::INVALID_STATE, "Nonfinite target cone: " + entry.target_id);
    }
    snapshot.ships.push_back(std::move(state));
  }
  snapshot.status = InputStatus::VALID;
  snapshot.reason = "Valid complete target snapshot";
  return snapshot;
}

ColregsDecision evaluateColregs(
  const TsSnapshot & snapshot, const OsState & os,
  double goal_x, double goal_y,
  const std::string & avoid_direction, const TsCoreParams & params)
{
  const auto fail = [](DecisionStatus status, const std::string & reason) {
      ColregsDecision result;
      result.status = status;
      result.reason = reason;
      return result;
    };
  std::string reason;
  if (!validateCoreParams(params, reason)) {
    return fail(DecisionStatus::INVALID_REQUEST, reason);
  }
  if (!std::isfinite(goal_x) || !std::isfinite(goal_y) ||
    (avoid_direction != "right" && avoid_direction != "left"))
  {
    return fail(DecisionStatus::INVALID_REQUEST, "Invalid goal or avoidance direction");
  }
  if (snapshot.status != InputStatus::VALID) {
    return fail(
      decisionStatus(snapshot.status),
      snapshot.reason.empty() ? inputStatusName(snapshot.status) : snapshot.reason);
  }
  if (!validOs(os) || snapshot.frame_id.empty() || snapshot.stamp.nanoseconds() < 0) {
    return fail(DecisionStatus::INVALID_STATE, "Invalid own-ship state, frame or calculation time");
  }
  if (!std::isfinite(goal_x - os.x) || !std::isfinite(goal_y - os.y)) {
    return fail(DecisionStatus::INVALID_REQUEST, "Nonfinite goal displacement");
  }
  std::set<std::string> ids;
  for (const auto & ts : snapshot.ships) {
    double checked_tcpa, checked_dcpa;
    if (ts.target_id.empty() || !ids.insert(ts.target_id).second ||
      !std::isfinite(ts.x) || !std::isfinite(ts.y) ||
      !std::isfinite(ts.vx) || !std::isfinite(ts.vy) ||
      !std::isfinite(ts.radius) || ts.radius < 0.0 ||
      !std::isfinite(ts.dcpa) || ts.dcpa < 0.0 || std::isnan(ts.tcpa) ||
      ts.tcpa == -std::numeric_limits<double>::infinity() ||
      !computeCpa(
        ts.x - os.x, ts.y - os.y, ts.vx - os.vx, ts.vy - os.vy,
        checked_tcpa, checked_dcpa) ||
      !std::isfinite(params.threat_radius_scale * (params.os_radius + ts.radius)) ||
      !std::isfinite(params.avoidance_radius_scale * (params.os_radius + ts.radius)) ||
      ts.cone_min.size() != ts.cone_max.size() ||
      !std::all_of(
        ts.cone_min.begin(), ts.cone_min.end(), [](double angle) {
          return std::isfinite(angle);
        }) ||
      !std::all_of(
        ts.cone_max.begin(), ts.cone_max.end(), [](double angle) {
          return std::isfinite(angle);
        }))
    {
      return fail(DecisionStatus::INVALID_STATE, "Invalid processed target: " + ts.target_id);
    }
  }
  const int primary_idx = selectPrimary(snapshot, os, params);
  if (primary_idx < 0) {
    return fail(DecisionStatus::NO_THREAT, "Valid snapshot has no qualified threat");
  }

  const double goal_heading = std::atan2(goal_y - os.y, goal_x - os.x);
  const double turn = avoid_direction == "right" ? -kHeadingStep : kHeadingStep;
  const double speed = std::hypot(os.vx, os.vy);
  bool found = false;
  double safe_heading = 0.0;
  for (int i = 0; i < kHeadingSamples; ++i) {
    const double heading = wrapAngle(goal_heading + i * turn);
    const double vx = speed * std::cos(heading);
    const double vy = speed * std::sin(heading);
    bool safe = true;
    for (const auto & ts : snapshot.ships) {
      bool collision;
      if (!collisionCourse(
          ts.x - os.x, ts.y - os.y, ts.vx - vx, ts.vy - vy,
          params.avoidance_radius_scale * (params.os_radius + ts.radius), collision))
      {
        return fail(DecisionStatus::INVALID_STATE, "Nonfinite candidate CPA: " + ts.target_id);
      }
      // Include secondary targets even when they failed threat qualification.
      safe = safe && !collision;
    }
    if (safe) {
      safe_heading = heading;
      found = true;
      break;
    }
  }
  if (!found) {
    return fail(DecisionStatus::INESCAPABLE, "No feasible heading in the full 360-degree scan");
  }

  ColregsDecision decision;
  decision.primary = snapshot.ships[primary_idx];
  decision.safe_heading = safe_heading;
  const double distance = std::hypot(decision.primary.x - os.x, decision.primary.y - os.y);
  const double range = distance + params.point_extension_distance;
  decision.avoidance_point.x = os.x + range * std::cos(safe_heading);
  decision.avoidance_point.y = os.y + range * std::sin(safe_heading);
  decision.avoidance_point.z = 0.0;
  decision.barrier_points = generateBarrierLines(
    os.x, os.y, decision.primary.x, decision.primary.y,
    decision.primary.radius, avoid_direction, params);
  if (!std::isfinite(range) || !std::isfinite(safe_heading) ||
    !finitePoint(decision.avoidance_point) ||
    !std::all_of(decision.barrier_points.begin(), decision.barrier_points.end(), finitePoint))
  {
    return fail(DecisionStatus::INVALID_STATE, "Nonfinite avoidance or barrier geometry");
  }
  decision.active = true;
  decision.status = DecisionStatus::SUCCESS;
  decision.reason = "Analytically safe heading against all targets at the measured own-ship speed";
  return decision;
}

}  // namespace nav2_colregs_ts_manager
