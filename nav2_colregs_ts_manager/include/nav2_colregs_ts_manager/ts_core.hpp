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

/// Raw target-ship observation copied under the state mutex (map frame).
struct RawTsEntry
{
  std::string target_id;
  double x{0.0};
  double y{0.0};
  double radius{0.0};
  double vx{0.0};
  double vy{0.0};
  rclcpp::Time last_seen;
};

/// Raw snapshot: observation entries without any derived computation.
struct RawTsSnapshot
{
  rclcpp::Time stamp;
  std::vector<RawTsEntry> ships;
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
  rclcpp::Time stamp;
  std::vector<TsState> ships;
};

/// Own-ship state. Position comes from the planning start; velocity is the
/// odometry body twist rotated into the map frame (velocity_valid=false when
/// the rotation was not possible, in which case vx/vy are zero).
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
  double ts_timeout{3.0};
  double tcpa_horizon{10.0};
  double safety_factor{1.1};
  double os_radius{0.3};
  double barrier_ray_length{999.0};
};

/// COLREGS decision produced by evaluateColregs. When active, barrier_points
/// holds six points forming three consecutive line segments.
struct ColregsDecision
{
  bool active{false};
  TsState primary;
  double safe_heading{0.0};
  geometry_msgs::msg::Point avoidance_point;
  std::vector<geometry_msgs::msg::Point> barrier_points;
};

/// Compute the processed snapshot: filter entries older than ts_timeout
/// relative to the snapshot stamp, then compute TCPA/DCPA, threat flag and
/// collision cone per entry. Pure function, no locks, no ROS calls.
TsSnapshot processTs(
  const RawTsSnapshot & raw, const OsState & os, const TsCoreParams & params);

/// Evaluate the COLREGS decision for one planning request: primary threat
/// selection (minimum TCPA among threats), safe heading from the collision
/// cone complement, avoidance point and barrier lines. Pure function.
ColregsDecision evaluateColregs(
  const TsSnapshot & snapshot, const OsState & os,
  double goal_x, double goal_y,
  const std::string & avoid_direction, const TsCoreParams & params);

}  // namespace nav2_colregs_ts_manager

#endif  // NAV2_COLREGS_TS_MANAGER__TS_CORE_HPP_
