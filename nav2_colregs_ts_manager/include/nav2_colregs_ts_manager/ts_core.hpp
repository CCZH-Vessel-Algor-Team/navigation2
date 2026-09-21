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

#ifndef NAV2_COLREGS_TS_MANAGER__TS_CORE_HPP_
#define NAV2_COLREGS_TS_MANAGER__TS_CORE_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/time.hpp"

namespace nav2_colregs_ts_manager
{

enum class InputStatus {VALID, NO_DATA, STALE_STATE, INVALID_STATE, INVALID_REQUEST};
enum class DecisionStatus
{
  SUCCESS, NO_THREAT, NO_DATA, STALE_STATE, INVALID_STATE, INVALID_REQUEST, INESCAPABLE
};

const char * inputStatusName(InputStatus status);
const char * decisionStatusName(DecisionStatus status);

/// Target state transformed/projected to the snapshot calculation time (map).
/// last_seen retains the original measurement time for freshness validation.
struct RawTsEntry
{
  std::string target_id;
  double x{0.0};
  double y{0.0};
  double radius{0.0};
  double vx{0.0};
  double vy{0.0};
  rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
};

/// Complete input aligned at stamp, before CPA/threat derivation.
struct RawTsSnapshot
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  std::vector<RawTsEntry> ships;
  InputStatus status{InputStatus::NO_DATA};
  std::string reason;
  std::string frame_id;
};

/// Processed target-ship state (result of processTs).
struct TsState
{
  std::string target_id;
  double x{0.0};
  double y{0.0};
  double radius{0.0};
  double vx{0.0};
  double vy{0.0};
  double tcpa{0.0};
  double dcpa{0.0};
  bool has_threat{false};
  std::vector<double> cone_min;
  std::vector<double> cone_max;
};

/// Processed snapshot: timeout-filtered entries with CPA and collision cone.
struct TsSnapshot
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  std::vector<TsState> ships;
  InputStatus status{InputStatus::NO_DATA};
  std::string reason;
  std::string frame_id;
};

/// Own-ship state anchored at the live planning start, with velocity transformed
/// from the odometry child frame at its measurement time into the global frame.
struct OsState
{
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  bool velocity_valid{false};
};

/// Parameters shared by the pure TS computations.
struct TsCoreParams
{
  double track_list_timeout{3.0};
  double threat_tcpa_horizon{10.0};
  double threat_radius_scale{1.1};
  double avoidance_radius_scale{1.1};
  double os_radius{0.3};
  double point_extension_distance{1.0};
  double lateral_margin{0.3};
  double closing_segment_length{999.0};
};

bool validateCoreParams(const TsCoreParams & params, std::string & reason);

/// COLREGS decision produced by evaluateColregs. When active, barrier_points
/// holds six points forming three consecutive line segments.
struct ColregsDecision
{
  bool active{false};
  DecisionStatus status{DecisionStatus::NO_DATA};
  std::string reason;
  TsState primary;
  double safe_heading{0.0};
  geometry_msgs::msg::Point avoidance_point{};
  std::vector<geometry_msgs::msg::Point> barrier_points;
};

/// Validate the complete snapshot, then compute CPA and threat flags. Invalid or
/// stale observations invalidate the input instead of becoming an empty scene.
TsSnapshot processTs(
  const RawTsSnapshot & raw, const OsState & os, const TsCoreParams & params);

/// Evaluate the COLREGS decision for one planning request: primary threat
/// selection, all-target analytic candidate checks, avoidance point and barrier
/// lines. Only explicit NO_THREAT permits ordinary planning. Pure function.
ColregsDecision evaluateColregs(
  const TsSnapshot & snapshot, const OsState & os,
  double goal_x, double goal_y,
  const std::string & avoid_direction, const TsCoreParams & params);

}  // namespace nav2_colregs_ts_manager

#endif  // NAV2_COLREGS_TS_MANAGER__TS_CORE_HPP_
