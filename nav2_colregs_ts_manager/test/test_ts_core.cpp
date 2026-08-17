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

#include <cmath>
#include <limits>
#include <string>

#include "gtest/gtest.h"
#include "nav2_colregs_ts_manager/ts_core.hpp"

namespace nav2_colregs_ts_manager
{

namespace
{

TsCoreParams defaultParams()
{
  TsCoreParams params;
  params.ts_timeout = 3.0;
  params.tcpa_horizon = 10.0;
  params.safety_factor = 1.1;
  params.os_radius = 0.3;
  params.barrier_ray_length = 999.0;
  return params;
}

RawTsEntry makeEntry(
  const std::string & id, double x, double y, double vx, double vy,
  double radius, rclcpp::Time last_seen)
{
  RawTsEntry entry;
  entry.target_id = id;
  entry.x = x;
  entry.y = y;
  entry.vx = vx;
  entry.vy = vy;
  entry.radius = radius;
  entry.last_seen = last_seen;
  return entry;
}

OsState makeOs(double x, double y, double vx, double vy)
{
  OsState os;
  os.x = x;
  os.y = y;
  os.vx = vx;
  os.vy = vy;
  os.velocity_valid = true;
  return os;
}

}  // namespace

TEST(TsCoreProcess, filtersEntriesOlderThanTimeout)
{
  const auto params = defaultParams();
  RawTsSnapshot raw;
  raw.stamp = rclcpp::Time(100, 0);
  raw.ships.push_back(
    makeEntry("stale", 5.0, 5.0, 0.0, 0.0, 0.3, rclcpp::Time(96, 500000000)));
  raw.ships.push_back(
    makeEntry("fresh", 6.0, 6.0, 0.0, 0.0, 0.3, rclcpp::Time(99, 500000000)));

  const auto snapshot = processTs(raw, makeOs(0.0, 0.0, 1.0, 0.0), params);

  ASSERT_EQ(snapshot.ships.size(), 1u);
  EXPECT_EQ(snapshot.ships[0].target_id, "fresh");
}

TEST(TsCoreProcess, computesHeadOnCpaAndThreat)
{
  const auto params = defaultParams();
  RawTsSnapshot raw;
  raw.stamp = rclcpp::Time(100, 0);
  raw.ships.push_back(
    makeEntry("head_on", 10.0, 0.0, -1.0, 0.0, 0.3, rclcpp::Time(100, 0)));

  const auto snapshot = processTs(raw, makeOs(0.0, 0.0, 1.0, 0.0), params);

  ASSERT_EQ(snapshot.ships.size(), 1u);
  const auto & ts = snapshot.ships[0];
  EXPECT_NEAR(ts.tcpa, 5.0, 1e-9);
  EXPECT_NEAR(ts.dcpa, 0.0, 1e-9);
  EXPECT_TRUE(ts.has_threat);
  EXPECT_FALSE(ts.cone_min.empty());
  EXPECT_EQ(ts.cone_min.size(), ts.cone_max.size());
}

TEST(TsCoreProcess, noThreatForRecedingTarget)
{
  const auto params = defaultParams();
  RawTsSnapshot raw;
  raw.stamp = rclcpp::Time(100, 0);
  raw.ships.push_back(
    makeEntry("receding", -10.0, 0.0, -1.0, 0.0, 0.3, rclcpp::Time(100, 0)));

  const auto snapshot = processTs(raw, makeOs(0.0, 0.0, 1.0, 0.0), params);

  ASSERT_EQ(snapshot.ships.size(), 1u);
  EXPECT_FALSE(snapshot.ships[0].has_threat);
}

TEST(TsCoreProcess, omitsCollisionConeWhenOsStatic)
{
  const auto params = defaultParams();
  RawTsSnapshot raw;
  raw.stamp = rclcpp::Time(100, 0);
  raw.ships.push_back(
    makeEntry("static", 5.0, 0.0, 0.0, 0.0, 0.3, rclcpp::Time(100, 0)));

  const auto snapshot = processTs(raw, makeOs(0.0, 0.0, 0.0, 0.0), params);

  ASSERT_EQ(snapshot.ships.size(), 1u);
  EXPECT_TRUE(snapshot.ships[0].cone_min.empty());
  EXPECT_TRUE(snapshot.ships[0].cone_max.empty());
}

TEST(TsCoreEvaluate, inactiveWithoutThreat)
{
  const auto params = defaultParams();
  RawTsSnapshot raw;
  raw.stamp = rclcpp::Time(100, 0);
  raw.ships.push_back(
    makeEntry("receding", -10.0, 0.0, -1.0, 0.0, 0.3, rclcpp::Time(100, 0)));
  const auto snapshot = processTs(raw, makeOs(0.0, 0.0, 1.0, 0.0), params);

  const auto decision = evaluateColregs(
    snapshot, makeOs(0.0, 0.0, 1.0, 0.0), 10.0, 0.0, "right", params);

  EXPECT_FALSE(decision.active);
  EXPECT_TRUE(decision.barrier_points.empty());
}

TEST(TsCoreEvaluate, inactiveWhenVelocityInvalid)
{
  const auto params = defaultParams();
  RawTsSnapshot raw;
  raw.stamp = rclcpp::Time(100, 0);
  raw.ships.push_back(
    makeEntry("head_on", 10.0, 0.0, -1.0, 0.0, 0.3, rclcpp::Time(100, 0)));
  const auto snapshot = processTs(raw, makeOs(0.0, 0.0, 1.0, 0.0), params);

  OsState os = makeOs(0.0, 0.0, 0.0, 0.0);
  os.velocity_valid = false;
  const auto decision = evaluateColregs(snapshot, os, 20.0, 0.0, "right", params);

  EXPECT_FALSE(decision.active);
}

TEST(TsCoreEvaluate, inactiveForInescapableCone)
{
  const auto params = defaultParams();
  RawTsSnapshot raw;
  raw.stamp = rclcpp::Time(100, 0);
  // Distance 0.2 <= os_radius + ts_radius = 0.6 → cone [[0, 2π]].
  raw.ships.push_back(
    makeEntry("overlapping", 0.2, 0.0, 0.0, 0.0, 0.3, rclcpp::Time(100, 0)));
  const auto snapshot = processTs(raw, makeOs(0.0, 0.0, 1.0, 0.0), params);

  ASSERT_EQ(snapshot.ships[0].cone_min.size(), 1u);
  EXPECT_NEAR(snapshot.ships[0].cone_min[0], 0.0, 1e-9);
  EXPECT_NEAR(snapshot.ships[0].cone_max[0], 2.0 * M_PI, 1e-9);

  const auto decision = evaluateColregs(
    snapshot, makeOs(0.0, 0.0, 1.0, 0.0), 10.0, 0.0, "right", params);

  EXPECT_FALSE(decision.active);
}

TEST(TsCoreEvaluate, rightAvoidanceTurnsClockwiseFromGoal)
{
  const auto params = defaultParams();
  // OS sails north; a static threat sits slightly starboard of the course
  // line; the goal lies straight through the threat.
  const OsState os = makeOs(0.0, 0.0, 0.0, 1.0);
  RawTsSnapshot raw;
  raw.stamp = rclcpp::Time(100, 0);
  raw.ships.push_back(
    makeEntry("threat", 0.3, 10.0, 0.0, 0.0, 0.3, rclcpp::Time(100, 0)));
  const auto snapshot = processTs(raw, os, params);
  ASSERT_TRUE(snapshot.ships[0].has_threat);

  const double goal_x = 0.3;
  const double goal_y = 20.0;
  const auto decision = evaluateColregs(snapshot, os, goal_x, goal_y, "right", params);

  ASSERT_TRUE(decision.active);
  ASSERT_EQ(decision.barrier_points.size(), 6u);
  EXPECT_NEAR(decision.barrier_points[0].x, 0.3, 1e-9);
  EXPECT_NEAR(decision.barrier_points[0].y, 10.0, 1e-9);

  const double goal_angle = std::atan2(goal_y - os.y, goal_x - os.x);
  EXPECT_LT(decision.safe_heading, goal_angle);

  const double dist = std::hypot(0.3 - os.x, 10.0 - os.y);
  const double expected_range = dist + (0.3 + 0.3) * params.safety_factor;
  const double point_range = std::hypot(
    decision.avoidance_point.x - os.x, decision.avoidance_point.y - os.y);
  EXPECT_NEAR(point_range, expected_range, 1e-9);
}

TEST(TsCoreEvaluate, leftAvoidanceTurnsCounterClockwiseFromGoal)
{
  const auto params = defaultParams();
  const OsState os = makeOs(0.0, 0.0, 0.0, 1.0);
  RawTsSnapshot raw;
  raw.stamp = rclcpp::Time(100, 0);
  raw.ships.push_back(
    makeEntry("threat", 0.3, 10.0, 0.0, 0.0, 0.3, rclcpp::Time(100, 0)));
  const auto snapshot = processTs(raw, os, params);

  const double goal_x = 0.3;
  const double goal_y = 20.0;
  const auto decision = evaluateColregs(snapshot, os, goal_x, goal_y, "left", params);

  ASSERT_TRUE(decision.active);
  const double goal_angle = std::atan2(goal_y - os.y, goal_x - os.x);
  EXPECT_GT(decision.safe_heading, goal_angle);
  ASSERT_EQ(decision.barrier_points.size(), 6u);
}

TEST(TsCoreEvaluate, returnsGoalHeadingWhenConeEmpty)
{
  const auto params = defaultParams();
  const OsState os = makeOs(0.0, 0.0, 0.0, 0.0);
  RawTsSnapshot raw;
  raw.stamp = rclcpp::Time(100, 0);
  // Static OS → empty cone; the closing TS still yields a threat, and the
  // empty cone makes findSafeHeading return the goal heading directly.
  raw.ships.push_back(
    makeEntry("any", 5.0, 5.0, -1.0, -1.0, 0.3, rclcpp::Time(100, 0)));
  const auto snapshot = processTs(raw, os, params);
  ASSERT_TRUE(snapshot.ships[0].cone_min.empty());

  const auto decision = evaluateColregs(snapshot, os, 10.0, 10.0, "right", params);

  ASSERT_TRUE(decision.active);
  EXPECT_NEAR(decision.safe_heading, std::atan2(10.0, 10.0), 1e-9);
}

}  // namespace nav2_colregs_ts_manager
