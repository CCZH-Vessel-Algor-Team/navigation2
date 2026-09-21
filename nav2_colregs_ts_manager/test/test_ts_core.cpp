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

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_colregs_ts_manager/ts_core.hpp"

namespace nav2_colregs_ts_manager
{

namespace
{

TsCoreParams defaultParams()
{
  return TsCoreParams{};
}

RawTsSnapshot makeRaw()
{
  RawTsSnapshot raw;
  raw.stamp = rclcpp::Time(100, 0);
  raw.status = InputStatus::VALID;
  raw.frame_id = "map";
  return raw;
}

RawTsEntry makeEntry(
  const std::string & id, double x, double y, double vx, double vy,
  double radius = 0.3, rclcpp::Time last_seen = rclcpp::Time(100, 0))
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

// Independent squared-distance CPA check for the moderate-sized test scenes.
double futureClearance(const TsState & ts, const OsState & os, double heading)
{
  const double speed = std::hypot(os.vx, os.vy);
  const double rx = ts.x - os.x;
  const double ry = ts.y - os.y;
  const double ux = ts.vx - speed * std::cos(heading);
  const double uy = ts.vy - speed * std::sin(heading);
  const double speed_sq = ux * ux + uy * uy;
  const double time = speed_sq == 0.0 ? 0.0 :
    std::max(0.0, -(rx * ux + ry * uy) / speed_sq);
  return std::hypot(rx + ux * time, ry + uy * time);
}

void expectSafe(
  const ColregsDecision & decision, const TsSnapshot & snapshot,
  const OsState & os, const TsCoreParams & params)
{
  ASSERT_TRUE(decision.active);
  ASSERT_EQ(decision.status, DecisionStatus::SUCCESS);
  for (const auto & ts : snapshot.ships) {
    EXPECT_GT(
      futureClearance(ts, os, decision.safe_heading),
      params.avoidance_radius_scale * (params.os_radius + ts.radius)) << ts.target_id;
  }
}

}  // namespace

TEST(TsCoreStatus, distinguishesMissingInputFromFreshEmptyList)
{
  const auto params = defaultParams();
  const auto os = makeOs(0.0, 0.0, 1.0, 0.0);
  EXPECT_EQ(RawTsSnapshot{}.status, InputStatus::NO_DATA);
  EXPECT_EQ(TsSnapshot{}.status, InputStatus::NO_DATA);
  EXPECT_EQ(ColregsDecision{}.status, DecisionStatus::NO_DATA);
  const auto missing = processTs(RawTsSnapshot{}, os, params);
  EXPECT_EQ(missing.status, InputStatus::NO_DATA);
  const auto no_data = evaluateColregs(missing, os, 10.0, 0.0, "right", params);
  EXPECT_EQ(no_data.status, DecisionStatus::NO_DATA);
  EXPECT_FALSE(no_data.active);

  const auto empty = processTs(makeRaw(), os, params);
  EXPECT_EQ(empty.status, InputStatus::VALID);
  EXPECT_EQ(empty.frame_id, "map");
  EXPECT_EQ(empty.stamp, rclcpp::Time(100, 0));
  const auto clear = evaluateColregs(empty, os, 10.0, 0.0, "right", params);
  EXPECT_EQ(clear.status, DecisionStatus::NO_THREAT);
  EXPECT_FALSE(clear.active);
  EXPECT_TRUE(clear.barrier_points.empty());
}

TEST(TsCoreStatus, propagatesInputFailuresAndReasons)
{
  const std::vector<std::pair<InputStatus, DecisionStatus>> cases = {
    {InputStatus::NO_DATA, DecisionStatus::NO_DATA},
    {InputStatus::STALE_STATE, DecisionStatus::STALE_STATE},
    {InputStatus::INVALID_STATE, DecisionStatus::INVALID_STATE},
    {InputStatus::INVALID_REQUEST, DecisionStatus::INVALID_REQUEST}};
  const auto params = defaultParams();
  const auto os = makeOs(0.0, 0.0, 1.0, 0.0);
  for (const auto & item : cases) {
    auto raw = makeRaw();
    raw.status = item.first;
    raw.reason = "upstream failure";
    raw.ships.push_back(makeEntry("head_on", 10.0, 0.0, -1.0, 0.0));
    const auto snapshot = processTs(raw, os, params);
    EXPECT_EQ(snapshot.status, item.first);
    EXPECT_EQ(snapshot.reason, raw.reason);
    EXPECT_TRUE(snapshot.ships.empty());
    const auto decision = evaluateColregs(snapshot, os, 10.0, 0.0, "right", params);
    EXPECT_EQ(decision.status, item.second);
    EXPECT_EQ(decision.reason, raw.reason);
    EXPECT_FALSE(decision.active);
    EXPECT_TRUE(decision.barrier_points.empty());
    EXPECT_STREQ(inputStatusName(item.first), decisionStatusName(item.second));
  }
  EXPECT_STREQ(inputStatusName(InputStatus::VALID), "VALID");
  EXPECT_STREQ(decisionStatusName(DecisionStatus::SUCCESS), "SUCCESS");
  EXPECT_STREQ(decisionStatusName(DecisionStatus::NO_THREAT), "NO_THREAT");
  EXPECT_STREQ(decisionStatusName(DecisionStatus::INESCAPABLE), "INESCAPABLE");
}

TEST(TsCoreProcess, expiredEntryInvalidatesCompleteList)
{
  const auto params = defaultParams();
  auto raw = makeRaw();
  // Put a valid entry first to catch accidental partial-list output.
  raw.ships.push_back(
    makeEntry(
      "fresh", 6.0, 6.0, 0.0, 0.0, 0.3,
      rclcpp::Time(99, 500000000)));
  raw.ships.push_back(
    makeEntry(
      "stale", 5.0, 5.0, 0.0, 0.0, 0.3,
      rclcpp::Time(96, 500000000)));
  const auto os = makeOs(0.0, 0.0, 1.0, 0.0);
  const auto snapshot = processTs(raw, os, params);
  EXPECT_EQ(snapshot.status, InputStatus::STALE_STATE);
  EXPECT_FALSE(snapshot.reason.empty());
  EXPECT_TRUE(snapshot.ships.empty());
  EXPECT_EQ(
    evaluateColregs(snapshot, os, 10.0, 0.0, "right", params).status,
    DecisionStatus::STALE_STATE);
}

TEST(TsCoreProcess, acceptsTimeoutBoundaryWithoutExtrapolatingTwice)
{
  auto raw = makeRaw();
  raw.ships.push_back(
    makeEntry(
      "projected", 10.0, 0.0, -1.0, 0.0, 0.3,
      rclcpp::Time(97, 0)));
  const auto snapshot = processTs(raw, makeOs(0.0, 0.0, 1.0, 0.0), defaultParams());
  ASSERT_EQ(snapshot.status, InputStatus::VALID);
  ASSERT_EQ(snapshot.ships.size(), 1u);
  EXPECT_DOUBLE_EQ(snapshot.ships[0].x, 10.0);
  EXPECT_NEAR(snapshot.ships[0].tcpa, 5.0, 1e-9);
}

TEST(TsCoreProcess, rejectsFutureMeasurementsAndClockMismatchWithoutThrowing)
{
  auto raw = makeRaw();
  raw.ships.push_back(
    makeEntry(
      "future", 5.0, 0.0, 0.0, 0.0, 0.3,
      rclcpp::Time(100, 1)));
  const auto os = makeOs(0.0, 0.0, 1.0, 0.0);
  EXPECT_EQ(processTs(raw, os, defaultParams()).status, InputStatus::INVALID_STATE);
  raw.ships[0].last_seen = rclcpp::Time(100, 0, RCL_ROS_TIME);
  ASSERT_NE(raw.stamp.get_clock_type(), raw.ships[0].last_seen.get_clock_type());
  EXPECT_NO_THROW(
    {
      EXPECT_EQ(processTs(raw, os, defaultParams()).status, InputStatus::INVALID_STATE);
    });
}

TEST(TsCoreProcess, rejectsMalformedEntryInsteadOfDroppingIt)
{
  const auto os = makeOs(0.0, 0.0, 1.0, 0.0);
  const std::vector<double RawTsEntry::*> fields = {
    &RawTsEntry::x, &RawTsEntry::y, &RawTsEntry::vx, &RawTsEntry::vy, &RawTsEntry::radius};
  for (auto field : fields) {
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity()})
    {
      auto raw = makeRaw();
      raw.ships.push_back(makeEntry("good", 10.0, 0.0, -1.0, 0.0));
      raw.ships.push_back(makeEntry("bad", 20.0, 0.0, 0.0, 0.0));
      raw.ships.back().*field = invalid;
      const auto snapshot = processTs(raw, os, defaultParams());
      EXPECT_EQ(snapshot.status, InputStatus::INVALID_STATE);
      EXPECT_TRUE(snapshot.ships.empty());
    }
  }
  auto raw = makeRaw();
  raw.ships.push_back(makeEntry("negative_radius", 5.0, 0.0, 0.0, 0.0, -0.1));
  EXPECT_EQ(processTs(raw, os, defaultParams()).status, InputStatus::INVALID_STATE);
  raw.ships[0].radius = 0.0;
  EXPECT_EQ(processTs(raw, os, defaultParams()).status, InputStatus::VALID);
  raw.ships.push_back(raw.ships[0]);
  EXPECT_EQ(processTs(raw, os, defaultParams()).status, InputStatus::INVALID_STATE);
  raw.ships.clear();
  raw.frame_id.clear();
  EXPECT_EQ(processTs(raw, os, defaultParams()).status, InputStatus::INVALID_STATE);
}

TEST(TsCoreProcess, computesHeadOnCpaAndThreat)
{
  auto raw = makeRaw();
  raw.ships.push_back(makeEntry("head_on", 10.0, 0.0, -1.0, 0.0));
  const auto snapshot = processTs(raw, makeOs(0.0, 0.0, 1.0, 0.0), defaultParams());
  ASSERT_EQ(snapshot.status, InputStatus::VALID);
  ASSERT_EQ(snapshot.ships.size(), 1u);
  const auto & ts = snapshot.ships[0];
  EXPECT_NEAR(ts.tcpa, 5.0, 1e-9);
  EXPECT_NEAR(ts.dcpa, 0.0, 1e-9);
  EXPECT_TRUE(ts.has_threat);
  EXPECT_FALSE(ts.cone_min.empty());
  EXPECT_EQ(ts.cone_min.size(), ts.cone_max.size());
}

TEST(TsCoreProcess, noThreatForRecedingTargetUsesFutureDcpa)
{
  auto raw = makeRaw();
  raw.ships.push_back(makeEntry("receding", -10.0, 0.0, -1.0, 0.0));
  const auto snapshot = processTs(raw, makeOs(0.0, 0.0, 1.0, 0.0), defaultParams());
  ASSERT_EQ(snapshot.ships.size(), 1u);
  EXPECT_FALSE(snapshot.ships[0].has_threat);
  EXPECT_NEAR(snapshot.ships[0].tcpa, -5.0, 1e-9);
  EXPECT_NEAR(snapshot.ships[0].dcpa, 10.0, 1e-9);
}

TEST(TsCoreProcess, pastTcpaIntrusionStillThreatensWithoutRewritingTcpa)
{
  auto raw = makeRaw();
  raw.ships.push_back(makeEntry("receding_intruder", -0.5, 0.0, -1.0, 0.0));
  const auto os = makeOs(0.0, 0.0, 1.0, 0.0);
  const auto snapshot = processTs(raw, os, defaultParams());
  ASSERT_EQ(snapshot.ships.size(), 1u);
  EXPECT_TRUE(snapshot.ships[0].has_threat);
  EXPECT_NEAR(snapshot.ships[0].tcpa, -0.25, 1e-9);
  EXPECT_NEAR(snapshot.ships[0].dcpa, 0.5, 1e-9);
  EXPECT_EQ(
    evaluateColregs(snapshot, os, 10.0, 0.0, "right", defaultParams()).status,
    DecisionStatus::INESCAPABLE);
}

TEST(TsCoreProcess, equalVelocitiesPreserveInfiniteTcpa)
{
  auto raw = makeRaw();
  raw.ships.push_back(makeEntry("near", 0.5, 0.0, 1.0, 0.0));
  raw.ships.push_back(makeEntry("far", 10.0, 0.0, 1.0, 0.0));
  const auto os = makeOs(0.0, 0.0, 1.0, 0.0);
  const auto snapshot = processTs(raw, os, defaultParams());
  ASSERT_EQ(snapshot.status, InputStatus::VALID);
  ASSERT_EQ(snapshot.ships.size(), 2u);
  for (const auto & ts : snapshot.ships) {
    EXPECT_EQ(ts.tcpa, std::numeric_limits<double>::infinity());
    EXPECT_DOUBLE_EQ(ts.dcpa, ts.x);
  }
  EXPECT_TRUE(snapshot.ships[0].has_threat);
  EXPECT_FALSE(snapshot.ships[1].has_threat);
  EXPECT_EQ(
    evaluateColregs(snapshot, os, 20.0, 0.0, "right", defaultParams()).status,
    DecisionStatus::INESCAPABLE);
}

TEST(TsCoreProcess, smallNonzeroRelativeSpeedIsNotEqualVelocity)
{
  auto raw = makeRaw();
  raw.ships.push_back(makeEntry("slow", 1.0, 0.0, -1e-9, 0.0));
  const auto snapshot = processTs(raw, makeOs(0.0, 0.0, 0.0, 0.0), defaultParams());
  ASSERT_EQ(snapshot.status, InputStatus::VALID);
  EXPECT_NEAR(snapshot.ships[0].tcpa, 1e9, 1e-6);
  EXPECT_NEAR(snapshot.ships[0].dcpa, 0.0, 1e-12);
  EXPECT_FALSE(snapshot.ships[0].has_threat);
}

TEST(TsCoreProcess, inclusiveCurrentRadiusAndHorizonButStrictFutureRadius)
{
  auto params = defaultParams();
  params.threat_radius_scale = 1.0;
  params.os_radius = 0.5;
  const auto os = makeOs(0.0, 0.0, 1.0, 0.0);
  auto raw = makeRaw();
  raw.ships.push_back(makeEntry("boundary", 0.0, 1.0, 1.0, 0.0, 0.5));
  raw.ships.push_back(makeEntry("horizon", 10.0, 0.0, 0.0, 0.0, 0.5));
  raw.ships.push_back(makeEntry("tangent", 5.0, 1.0, 0.0, 0.0, 0.5));
  raw.ships.push_back(makeEntry("late", 10.01, 0.0, 0.0, 0.0, 0.5));
  const auto snapshot = processTs(raw, os, params);
  ASSERT_EQ(snapshot.status, InputStatus::VALID);
  EXPECT_TRUE(snapshot.ships[0].has_threat);
  EXPECT_TRUE(snapshot.ships[1].has_threat);
  EXPECT_FALSE(snapshot.ships[2].has_threat);
  EXPECT_FALSE(snapshot.ships[3].has_threat);
}

TEST(TsCoreProcess, omitsCollisionConeWhenBothShipsStaticAndSeparated)
{
  auto raw = makeRaw();
  raw.ships.push_back(makeEntry("static", 5.0, 0.0, 0.0, 0.0));
  const auto snapshot = processTs(raw, makeOs(0.0, 0.0, 0.0, 0.0), defaultParams());
  ASSERT_EQ(snapshot.ships.size(), 1u);
  EXPECT_TRUE(snapshot.ships[0].cone_min.empty());
  EXPECT_TRUE(snapshot.ships[0].cone_max.empty());
}

TEST(TsCoreEvaluate, inactiveWithoutThreat)
{
  const auto params = defaultParams();
  auto raw = makeRaw();
  raw.ships.push_back(makeEntry("receding", -10.0, 0.0, -1.0, 0.0));
  const auto os = makeOs(0.0, 0.0, 1.0, 0.0);
  const auto snapshot = processTs(raw, os, params);
  const auto decision = evaluateColregs(snapshot, os, 10.0, 0.0, "right", params);
  EXPECT_EQ(decision.status, DecisionStatus::NO_THREAT);
  EXPECT_FALSE(decision.active);
  EXPECT_TRUE(decision.barrier_points.empty());
}

TEST(TsCoreEvaluate, invalidVelocityNeverBecomesNoThreat)
{
  const auto params = defaultParams();
  auto raw = makeRaw();
  raw.ships.push_back(makeEntry("head_on", 10.0, 0.0, -1.0, 0.0));
  auto os = makeOs(0.0, 0.0, 1.0, 0.0);
  const auto snapshot = processTs(raw, os, params);
  os.velocity_valid = false;
  const auto decision = evaluateColregs(snapshot, os, 20.0, 0.0, "right", params);
  EXPECT_FALSE(decision.active);
  EXPECT_EQ(decision.status, DecisionStatus::INVALID_STATE);
  EXPECT_EQ(processTs(raw, os, params).status, InputStatus::INVALID_STATE);
  // Also reject bad OS data in a received empty list.
  raw.ships.clear();
  EXPECT_EQ(processTs(raw, os, params).status, InputStatus::INVALID_STATE);
  os.velocity_valid = true;
  const std::vector<double OsState::*> fields = {
    &OsState::x, &OsState::y, &OsState::vx, &OsState::vy};
  for (auto field : fields) {
    auto bad_os = os;
    bad_os.*field = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(processTs(raw, bad_os, params).status, InputStatus::INVALID_STATE);
    EXPECT_EQ(
      evaluateColregs(snapshot, bad_os, 20.0, 0.0, "right", params).status,
      DecisionStatus::INVALID_STATE);
  }
}

TEST(TsCoreEvaluate, inactiveForCurrentOverlap)
{
  const auto params = defaultParams();
  auto raw = makeRaw();
  raw.ships.push_back(makeEntry("overlapping", 0.2, 0.0, 0.0, 0.0));
  const auto os = makeOs(0.0, 0.0, 1.0, 0.0);
  const auto snapshot = processTs(raw, os, params);
  ASSERT_EQ(snapshot.ships[0].cone_min.size(), 1u);
  EXPECT_NEAR(snapshot.ships[0].cone_min[0], 0.0, 1e-9);
  EXPECT_NEAR(snapshot.ships[0].cone_max[0], 2.0 * M_PI, 1e-9);
  const auto decision = evaluateColregs(snapshot, os, 10.0, 0.0, "right", params);
  EXPECT_FALSE(decision.active);
  EXPECT_EQ(decision.status, DecisionStatus::INESCAPABLE);
  EXPECT_TRUE(decision.barrier_points.empty());
}

TEST(TsCoreEvaluate, rightAvoidanceTurnsClockwiseFromGoal)
{
  const auto params = defaultParams();
  const OsState os = makeOs(0.0, 0.0, 0.0, 1.0);
  auto raw = makeRaw();
  raw.ships.push_back(makeEntry("threat", 0.3, 10.0, 0.0, 0.0));
  const auto snapshot = processTs(raw, os, params);
  ASSERT_TRUE(snapshot.ships[0].has_threat);
  const double goal_x = 0.3;
  const double goal_y = 20.0;
  const auto decision = evaluateColregs(snapshot, os, goal_x, goal_y, "right", params);
  ASSERT_TRUE(decision.active);
  expectSafe(decision, snapshot, os, params);
  ASSERT_EQ(decision.barrier_points.size(), 6u);
  EXPECT_NEAR(decision.barrier_points[0].x, 0.3, 1e-9);
  EXPECT_NEAR(decision.barrier_points[0].y, 10.0, 1e-9);
  const double goal_angle = std::atan2(goal_y - os.y, goal_x - os.x);
  EXPECT_LT(decision.safe_heading, goal_angle);
  const double dist = std::hypot(0.3 - os.x, 10.0 - os.y);
  const double point_range = std::hypot(
    decision.avoidance_point.x - os.x, decision.avoidance_point.y - os.y);
  EXPECT_NEAR(point_range, dist + params.point_extension_distance, 1e-9);
}

TEST(TsCoreEvaluate, leftAvoidanceTurnsCounterClockwiseFromGoal)
{
  const auto params = defaultParams();
  const OsState os = makeOs(0.0, 0.0, 0.0, 1.0);
  auto raw = makeRaw();
  raw.ships.push_back(makeEntry("threat", 0.3, 10.0, 0.0, 0.0));
  const auto snapshot = processTs(raw, os, params);
  const auto decision = evaluateColregs(snapshot, os, 0.3, 20.0, "left", params);
  ASSERT_TRUE(decision.active);
  expectSafe(decision, snapshot, os, params);
  EXPECT_GT(decision.safe_heading, std::atan2(20.0, 0.3));
  ASSERT_EQ(decision.barrier_points.size(), 6u);
}

TEST(TsCoreEvaluate, staticOwnShipCannotTurnAwayFromIncomingTarget)
{
  const auto params = defaultParams();
  const OsState os = makeOs(0.0, 0.0, 0.0, 0.0);
  auto raw = makeRaw();
  raw.ships.push_back(makeEntry("incoming", 5.0, 5.0, -1.0, -1.0));
  auto snapshot = processTs(raw, os, params);
  ASSERT_TRUE(snapshot.ships[0].has_threat);
  ASSERT_EQ(snapshot.ships[0].cone_min.size(), 1u);
  EXPECT_DOUBLE_EQ(snapshot.ships[0].cone_min[0], 0.0);
  EXPECT_DOUBLE_EQ(snapshot.ships[0].cone_max[0], 2.0 * M_PI);
  // Even absent diagnostics must not make the unsafe goal heading feasible.
  snapshot.ships[0].cone_min.clear();
  snapshot.ships[0].cone_max.clear();
  const auto decision = evaluateColregs(snapshot, os, 10.0, 10.0, "right", params);
  EXPECT_FALSE(decision.active);
  EXPECT_EQ(decision.status, DecisionStatus::INESCAPABLE);
  EXPECT_TRUE(decision.barrier_points.empty());
}

TEST(TsCoreEvaluate, rejectsTangentGoalAndIgnoresOldConeBoundary)
{
  const auto params = defaultParams();
  const auto os = makeOs(0.0, 0.0, 1.0, 0.0);
  auto raw = makeRaw();
  raw.ships.push_back(makeEntry("threat", 5.0, 0.0, 0.0, 0.0));
  auto snapshot = processTs(raw, os, params);
  const double tangent = std::asin(params.avoidance_radius_scale * 0.6 / 5.0);
  // The old complement algorithm accepted an unsafe interval endpoint.
  snapshot.ships[0].cone_min = {0.0};
  snapshot.ships[0].cone_max = {tangent};
  const auto decision = evaluateColregs(
    snapshot, os, 10.0 * std::cos(tangent), 10.0 * std::sin(tangent), "left", params);
  expectSafe(decision, snapshot, os, params);
  EXPECT_NEAR(decision.safe_heading, tangent + 2.0 * M_PI / 180.0, 1e-12);
  // An over-conservative diagnostic cone also cannot veto a feasible candidate.
  snapshot.ships[0].cone_max = {2.0 * M_PI};
  const auto goal_safe = evaluateColregs(snapshot, os, 0.0, 10.0, "left", params);
  expectSafe(goal_safe, snapshot, os, params);
  EXPECT_NEAR(goal_safe.safe_heading, M_PI_2, 1e-12);
}

TEST(TsCoreEvaluate, checksSecondaryTargetsThatAreNotThreats)
{
  const auto params = defaultParams();
  const auto os = makeOs(0.0, 0.0, 1.0, 0.0);
  auto raw = makeRaw();
  raw.ships.push_back(makeEntry("primary", 5.0, 0.0, 0.0, 0.0));
  const auto primary_only = processTs(raw, os, params);
  const auto first = evaluateColregs(primary_only, os, 20.0, 0.0, "right", params);
  expectSafe(first, primary_only, os, params);
  EXPECT_NEAR(first.safe_heading, 352.0 * M_PI / 180.0, 1e-12);
  raw.ships.push_back(makeEntry("secondary", 5.0, -1.0, 0.0, 0.0));
  const auto snapshot = processTs(raw, os, params);
  ASSERT_FALSE(snapshot.ships[1].has_threat);
  EXPECT_LT(futureClearance(snapshot.ships[1], os, first.safe_heading), 0.66);
  const auto decision = evaluateColregs(snapshot, os, 20.0, 0.0, "right", params);
  expectSafe(decision, snapshot, os, params);
  EXPECT_EQ(decision.primary.target_id, "primary");
  EXPECT_NEAR(decision.safe_heading, 340.0 * M_PI / 180.0, 1e-12);
}

TEST(TsCoreEvaluate, candidateSafetyIsNotLimitedByThreatHorizon)
{
  const auto params = defaultParams();
  const auto os = makeOs(0.0, 0.0, 1.0, 0.0);
  auto raw = makeRaw();
  raw.ships.push_back(makeEntry("primary", 5.0, 0.0, 0.0, 0.0));
  const double obstructed = -8.0 * M_PI / 180.0;
  raw.ships.push_back(
    makeEntry(
      "distant", 20.0 * std::cos(obstructed), 20.0 * std::sin(obstructed), 0.0, 0.0));
  const auto snapshot = processTs(raw, os, params);
  ASSERT_FALSE(snapshot.ships[1].has_threat);
  EXPECT_GT(snapshot.ships[1].tcpa, params.threat_tcpa_horizon);
  const auto decision = evaluateColregs(snapshot, os, 20.0, 0.0, "right", params);
  expectSafe(decision, snapshot, os, params);
  EXPECT_NEAR(decision.safe_heading, 350.0 * M_PI / 180.0, 1e-12);
}

TEST(TsCoreEvaluate, candidateChecksUseTargetVelocity)
{
  const auto params = defaultParams();
  const auto os = makeOs(0.0, 0.0, 1.0, 0.0);
  auto raw = makeRaw();
  raw.ships.push_back(makeEntry("head_on", 5.0, 0.0, -1.0, 0.0));
  auto snapshot = processTs(raw, os, params);
  auto decision = evaluateColregs(snapshot, os, 20.0, 0.0, "right", params);
  expectSafe(decision, snapshot, os, params);
  // A stationary disc here permits -8 degrees; the incoming ship needs -16.
  EXPECT_NEAR(decision.safe_heading, 344.0 * M_PI / 180.0, 1e-12);

  raw.ships[0].vx = 0.0;
  raw.ships.push_back(makeEntry("moving_secondary", 5.0, -1.0, 0.0, 0.05));
  snapshot = processTs(raw, os, params);
  ASSERT_FALSE(snapshot.ships[1].has_threat);
  decision = evaluateColregs(snapshot, os, 20.0, 0.0, "right", params);
  expectSafe(decision, snapshot, os, params);
  EXPECT_NEAR(decision.safe_heading, 342.0 * M_PI / 180.0, 1e-12);
}

TEST(TsCoreEvaluate, scansBeyondHalfCircleInRequestedDirection)
{
  const auto params = defaultParams();
  const auto os = makeOs(0.0, 0.0, 1.0, 0.0);
  auto raw = makeRaw();
  // Each disc blocks about +/-41.3 degrees. The union blocks clockwise
  // candidates from zero through 210 degrees, but not the whole circle.
  for (int angle : {-30, -100, -170}) {
    const double radians = angle * M_PI / 180.0;
    raw.ships.push_back(
      makeEntry(
        std::to_string(angle), std::cos(radians), std::sin(radians), 0.0, 0.0));
  }
  const auto snapshot = processTs(raw, os, params);
  const auto decision = evaluateColregs(snapshot, os, 20.0, 0.0, "right", params);
  expectSafe(decision, snapshot, os, params);
  EXPECT_NEAR(decision.safe_heading, 148.0 * M_PI / 180.0, 1e-12);
}

TEST(TsCoreEvaluate, primaryTieBreakIsIndependentOfInputOrder)
{
  const auto params = defaultParams();
  const auto os = makeOs(0.0, 0.0, 1.0, 0.0);
  auto raw = makeRaw();
  raw.ships.push_back(makeEntry("z", 5.0, 0.1, 0.0, 0.0));
  raw.ships.push_back(makeEntry("a", 5.0, -0.1, 0.0, 0.0));
  const auto first = evaluateColregs(processTs(raw, os, params), os, 20.0, 0.0, "right", params);
  ASSERT_EQ(first.status, DecisionStatus::SUCCESS);
  EXPECT_EQ(first.primary.target_id, "a");
  std::reverse(raw.ships.begin(), raw.ships.end());
  const auto reversed = evaluateColregs(processTs(raw, os, params), os, 20.0, 0.0, "right", params);
  ASSERT_EQ(reversed.status, DecisionStatus::SUCCESS);
  EXPECT_EQ(reversed.primary.target_id, "a");
  EXPECT_DOUBLE_EQ(reversed.safe_heading, first.safe_heading);
}

TEST(TsCoreEvaluate, intrusionPriorityDoesNotRewriteInfiniteOrPastTcpa)
{
  auto params = defaultParams();
  params.threat_radius_scale = 2.0;
  params.avoidance_radius_scale = 1.0;
  const auto os = makeOs(0.0, 0.0, 1.0, 0.0);
  auto raw = makeRaw();
  raw.ships.push_back(makeEntry("future", 5.0, 0.0, 0.0, 0.0));
  raw.ships.push_back(makeEntry("near", 0.0, 1.0, 1.0, 0.0));
  auto snapshot = processTs(raw, os, params);
  auto decision = evaluateColregs(snapshot, os, 20.0, 0.0, "right", params);
  expectSafe(decision, snapshot, os, params);
  EXPECT_EQ(decision.primary.target_id, "near");
  EXPECT_EQ(decision.primary.tcpa, std::numeric_limits<double>::infinity());
  raw.ships[1].vx = 1.0;
  raw.ships[1].vy = 1.0;
  snapshot = processTs(raw, os, params);
  decision = evaluateColregs(snapshot, os, 20.0, 0.0, "right", params);
  expectSafe(decision, snapshot, os, params);
  EXPECT_EQ(decision.primary.target_id, "near");
  EXPECT_DOUBLE_EQ(decision.primary.tcpa, -1.0);
  EXPECT_DOUBLE_EQ(decision.primary.dcpa, 1.0);
}

TEST(TsCoreParams, validatesEveryParameterAndItsBoundary)
{
  const std::vector<double TsCoreParams::*> fields = {
    &TsCoreParams::track_list_timeout, &TsCoreParams::threat_tcpa_horizon,
    &TsCoreParams::threat_radius_scale, &TsCoreParams::avoidance_radius_scale,
    &TsCoreParams::os_radius, &TsCoreParams::point_extension_distance,
    &TsCoreParams::lateral_margin, &TsCoreParams::closing_segment_length};
  std::string reason;
  auto params = defaultParams();
  ASSERT_TRUE(validateCoreParams(params, reason));
  EXPECT_TRUE(reason.empty());
  for (auto field : fields) {
    for (double invalid : {-1.0, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity()})
    {
      auto bad = params;
      bad.*field = invalid;
      EXPECT_FALSE(validateCoreParams(bad, reason));
      EXPECT_FALSE(reason.empty());
      const auto os = makeOs(0.0, 0.0, 1.0, 0.0);
      EXPECT_EQ(processTs(makeRaw(), os, bad).status, InputStatus::INVALID_REQUEST);
      EXPECT_EQ(
        evaluateColregs(TsSnapshot{}, os, 1.0, 0.0, "right", bad).status,
        DecisionStatus::INVALID_REQUEST);
    }
  }
  for (auto field : {&TsCoreParams::track_list_timeout, &TsCoreParams::closing_segment_length}) {
    auto bad = params;
    bad.*field = 0.0;
    EXPECT_FALSE(validateCoreParams(bad, reason));
  }
  for (auto field : {&TsCoreParams::threat_radius_scale, &TsCoreParams::avoidance_radius_scale}) {
    auto bad = params;
    bad.*field = 0.99;
    EXPECT_FALSE(validateCoreParams(bad, reason));
  }
  params.threat_radius_scale = 1.0;
  params.avoidance_radius_scale = 1.0;
  params.threat_tcpa_horizon = 0.0;
  params.os_radius = 0.0;
  params.point_extension_distance = 0.0;
  params.lateral_margin = 0.0;
  EXPECT_TRUE(validateCoreParams(params, reason));
  EXPECT_TRUE(reason.empty());
}

TEST(TsCoreParams, threatAndAvoidanceRadiusScalesAreIndependent)
{
  auto params = defaultParams();
  const auto os = makeOs(0.0, 0.0, 1.0, 0.0);
  auto raw = makeRaw();
  raw.ships.push_back(makeEntry("offset", 5.0, 0.8, 0.0, 0.0));
  EXPECT_FALSE(processTs(raw, os, params).ships[0].has_threat);
  params.avoidance_radius_scale = 2.0;
  EXPECT_FALSE(processTs(raw, os, params).ships[0].has_threat);
  params.threat_radius_scale = 2.0;
  const auto inflated = processTs(raw, os, params);
  ASSERT_TRUE(inflated.ships[0].has_threat);
  const auto cautious = evaluateColregs(inflated, os, 20.0, 0.0, "right", params);
  expectSafe(cautious, inflated, os, params);
  EXPECT_NE(cautious.safe_heading, 0.0);
  params.avoidance_radius_scale = 1.0;
  const auto decision = evaluateColregs(inflated, os, 20.0, 0.0, "right", params);
  expectSafe(decision, inflated, os, params);
  EXPECT_DOUBLE_EQ(decision.safe_heading, 0.0);
}

TEST(TsCoreParams, pointExtensionChangesOnlyAvoidancePointRange)
{
  auto params = defaultParams();
  const auto os = makeOs(0.0, 0.0, 1.0, 0.0);
  auto raw = makeRaw();
  raw.ships.push_back(makeEntry("threat", 5.0, 0.0, 0.0, 0.0));
  const auto snapshot = processTs(raw, os, params);
  const auto first = evaluateColregs(snapshot, os, 20.0, 0.0, "right", params);
  ASSERT_EQ(first.status, DecisionStatus::SUCCESS);
  params.point_extension_distance = 4.0;
  const auto extended = evaluateColregs(snapshot, os, 20.0, 0.0, "right", params);
  ASSERT_EQ(extended.status, DecisionStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(first.safe_heading, extended.safe_heading);
  EXPECT_EQ(first.barrier_points, extended.barrier_points);
  EXPECT_NEAR(std::hypot(first.avoidance_point.x, first.avoidance_point.y), 6.0, 1e-9);
  EXPECT_NEAR(std::hypot(extended.avoidance_point.x, extended.avoidance_point.y), 9.0, 1e-9);
  params.os_radius = 0.8;
  params.avoidance_radius_scale = 1.5;
  const auto wider = evaluateColregs(processTs(raw, os, params), os, 20.0, 0.0, "right", params);
  ASSERT_EQ(wider.status, DecisionStatus::SUCCESS);
  EXPECT_NE(wider.safe_heading, extended.safe_heading);
  EXPECT_NEAR(std::hypot(wider.avoidance_point.x, wider.avoidance_point.y), 9.0, 1e-9);
  EXPECT_EQ(wider.barrier_points, extended.barrier_points);
}

TEST(TsCoreParams, barrierUsesIndependentLateralAndClosingLengths)
{
  auto params = defaultParams();
  params.lateral_margin = 0.8;
  params.closing_segment_length = 27.0;
  params.threat_tcpa_horizon = 30.0;
  const auto os = makeOs(0.0, 0.0, 1.0, 0.0);
  for (double distance : {5.0, 20.0}) {
    for (const auto & direction : {"right", "left"}) {
      auto raw = makeRaw();
      raw.ships.push_back(makeEntry("threat", distance, 0.0, 0.0, 0.0, 0.4));
      const auto snapshot = processTs(raw, os, params);
      const auto decision = evaluateColregs(snapshot, os, 40.0, 0.0, direction, params);
      ASSERT_EQ(decision.status, DecisionStatus::SUCCESS);
      const auto & points = decision.barrier_points;
      ASSERT_EQ(points.size(), 6u);
      EXPECT_DOUBLE_EQ(points[0].x, distance);
      EXPECT_DOUBLE_EQ(points[0].y, 0.0);
      EXPECT_EQ(points[1], points[2]);
      EXPECT_EQ(points[3], points[4]);
      EXPECT_NEAR(std::hypot(points[1].x - points[0].x, points[1].y - points[0].y), 1.2, 1e-9);
      EXPECT_NEAR(
        std::hypot(points[3].x - points[2].x, points[3].y - points[2].y),
        std::max(distance, 10.0) + 1.2, 1e-9);
      EXPECT_NEAR(std::hypot(points[5].x - points[4].x, points[5].y - points[4].y), 27.0, 1e-9);
      EXPECT_NEAR(points[1].y, std::string(direction) == "right" ? 1.2 : -1.2, 1e-9);
      EXPECT_LT(points[3].x, points[2].x);
      for (const auto & point : points) {
        EXPECT_DOUBLE_EQ(point.z, 0.0);
      }
    }
  }
}

TEST(TsCoreEvaluate, rejectsInvalidRequestEvenForEmptyScene)
{
  const auto params = defaultParams();
  const auto os = makeOs(0.0, 0.0, 1.0, 0.0);
  const auto snapshot = processTs(makeRaw(), os, params);
  EXPECT_EQ(
    evaluateColregs(snapshot, os, 1.0, 0.0, "RIGHT", params).status,
    DecisionStatus::INVALID_REQUEST);
  EXPECT_EQ(
    evaluateColregs(
      snapshot, os, std::numeric_limits<double>::infinity(),
      0.0, "right", params).status, DecisionStatus::INVALID_REQUEST);
  EXPECT_EQ(
    evaluateColregs(
      snapshot, os, 1.0, std::numeric_limits<double>::quiet_NaN(),
      "right", params).status, DecisionStatus::INVALID_REQUEST);
}

TEST(TsCoreEvaluate, rejectsMalformedProcessedSecondaryEvenWithoutThreats)
{
  const auto params = defaultParams();
  const auto os = makeOs(0.0, 0.0, 1.0, 0.0);
  auto raw = makeRaw();
  raw.ships.push_back(makeEntry("receding", -10.0, 0.0, -1.0, 0.0));
  const auto good = processTs(raw, os, params);
  const std::vector<double TsState::*> fields = {
    &TsState::x, &TsState::y, &TsState::vx, &TsState::vy,
    &TsState::radius, &TsState::tcpa, &TsState::dcpa};
  for (auto field : fields) {
    auto snapshot = good;
    snapshot.ships[0].*field = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(
      evaluateColregs(snapshot, os, 10.0, 0.0, "right", params).status,
      DecisionStatus::INVALID_STATE);
  }
  auto snapshot = good;
  snapshot.ships[0].tcpa = -std::numeric_limits<double>::infinity();
  EXPECT_EQ(
    evaluateColregs(snapshot, os, 10.0, 0.0, "right", params).status,
    DecisionStatus::INVALID_STATE);
  snapshot = good;
  snapshot.ships[0].cone_min = {std::numeric_limits<double>::infinity()};
  snapshot.ships[0].cone_max = {2.0 * M_PI};
  EXPECT_EQ(
    evaluateColregs(snapshot, os, 10.0, 0.0, "right", params).status,
    DecisionStatus::INVALID_STATE);
}

TEST(TsCoreEvaluate, finiteInputsWithOverflowingGeometryReturnInvalidAndNoPoints)
{
  auto params = defaultParams();
  const double huge = std::numeric_limits<double>::max();
  const auto os = makeOs(0.0, huge, 1.0, 0.0);
  auto raw = makeRaw();
  raw.ships.push_back(makeEntry("threat", 5.0, huge, 0.0, 0.0));
  params.lateral_margin = huge;
  const auto snapshot = processTs(raw, os, params);
  ASSERT_EQ(snapshot.status, InputStatus::VALID);
  const auto decision = evaluateColregs(snapshot, os, 10.0, huge, "right", params);
  EXPECT_EQ(decision.status, DecisionStatus::INVALID_STATE);
  EXPECT_FALSE(decision.active);
  EXPECT_TRUE(decision.barrier_points.empty());
  EXPECT_DOUBLE_EQ(decision.avoidance_point.x, 0.0);
  EXPECT_DOUBLE_EQ(decision.avoidance_point.y, 0.0);
  raw.ships[0].x = huge;
  EXPECT_EQ(
    processTs(raw, makeOs(-huge, huge, 1.0, 0.0), defaultParams()).status,
    InputStatus::INVALID_STATE);
}

}  // namespace nav2_colregs_ts_manager
