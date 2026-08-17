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

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_colregs_local_planner_server/rrt_star.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace nav2_colregs_local_planner_server
{

class RRTStarTestPeer
{
public:
  static const std::vector<RRTStarNode> & tree(const RRTStar & planner)
  {
    return planner.tree_;
  }

  static int iterations(const RRTStar & planner)
  {
    return planner.iterations_executed_;
  }

  static void setTree(RRTStar & planner, std::vector<RRTStarNode> tree)
  {
    planner.tree_ = std::move(tree);
  }

  static void rewire(
    RRTStar & planner, int new_idx, const std::vector<int> & near,
    const nav2_costmap_2d::Costmap2D & costmap)
  {
    prepareOperation(planner);
    (void)planner.rewire(new_idx, near, costmap);
  }

  static bool collisionFree(
    RRTStar & planner, double x1, double y1, double x2, double y2,
    const nav2_costmap_2d::Costmap2D & costmap)
  {
    prepareOperation(planner);
    return planner.collisionFree(x1, y1, x2, y2, costmap);
  }

  static bool pointCollisionFree(
    RRTStar & planner, double x, double y,
    const nav2_costmap_2d::Costmap2D & costmap)
  {
    prepareOperation(planner);
    return planner.pointCollisionFree(x, y, costmap);
  }

  static bool invalidGeometry(const RRTStar & planner)
  {
    return planner.invalid_geometry_;
  }

private:
  static void prepareOperation(RRTStar & planner)
  {
    planner.cancel_checker_ = nullptr;
    planner.barriers_ = nullptr;
    planner.deadline_ = std::chrono::steady_clock::time_point::max();
    planner.interrupted_ = false;
    planner.invalid_geometry_ = false;
  }
};

namespace
{

using std::chrono::steady_clock;

RRTStarParameters parameters()
{
  return {1.0, 100, 1.0, 0.05, 0.0, 0.0, 3, 4.0, 17U, false};
}

nav2_costmap_2d::Costmap2D freeMap()
{
  return nav2_costmap_2d::Costmap2D(20, 20, 1.0, 0.0, 0.0, 0);
}

PlanStatus plan(
  RRTStar & planner, const nav2_costmap_2d::Costmap2D & map,
  std::vector<RRTStarNode> & path, double goal_x = 4.5, double goal_y = 0.5,
  const std::function<bool()> & cancel = {},
  const std::vector<geometry_msgs::msg::Point> & barriers = {})
{
  return planner.planPath(
    0.5, 0.5, goal_x, goal_y, map, barriers, cancel,
    steady_clock::now() + std::chrono::seconds(10), path);
}

TEST(RRTStar, ClearsOutputBeforeEveryReturn)
{
  auto map = freeMap();
  RRTStar planner(parameters());
  std::vector<RRTStarNode> path(3);
  EXPECT_EQ(
    planner.planPath(
      0.5, 0.5, 4.5, 0.5, map, {}, []() {return true;},
      steady_clock::now() + std::chrono::seconds(1), path),
    PlanStatus::CANCELED);
  EXPECT_TRUE(path.empty());
}

TEST(RRTStar, RejectsInvalidParametersCoordinatesAndMap)
{
  auto map = freeMap();
  auto invalid = parameters();
  invalid.step_size = 0.0;
  std::vector<RRTStarNode> path;
  RRTStar invalid_planner(invalid);
  EXPECT_EQ(plan(invalid_planner, map, path), PlanStatus::INVALID_INPUT);

  RRTStar planner(parameters());
  EXPECT_EQ(
    planner.planPath(
      std::numeric_limits<double>::quiet_NaN(), 0.5, 4.5, 0.5, map, {}, {},
      steady_clock::now() + std::chrono::seconds(1), path),
    PlanStatus::INVALID_INPUT);
  nav2_costmap_2d::Costmap2D empty_map;
  EXPECT_EQ(plan(planner, empty_map, path), PlanStatus::INVALID_INPUT);
}

TEST(RRTStar, RejectsNonFiniteCostmapOriginsBeforeCoordinateConversion)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  nav2_costmap_2d::Costmap2D nan_origin_map(20, 20, 1.0, nan, 0.0, 0);
  nav2_costmap_2d::Costmap2D infinite_origin_map(20, 20, 1.0, 0.0, infinity, 0);
  nav2_costmap_2d::Costmap2D overflow_bounds_map(
    20, 20, std::numeric_limits<double>::max(), 0.0, 0.0, 0);
  RRTStar planner(parameters());
  std::vector<RRTStarNode> path(2);

  EXPECT_EQ(plan(planner, nan_origin_map, path), PlanStatus::INVALID_INPUT);
  EXPECT_TRUE(path.empty());
  path.resize(2);
  EXPECT_EQ(plan(planner, infinite_origin_map, path), PlanStatus::INVALID_INPUT);
  EXPECT_TRUE(path.empty());
  path.resize(2);
  EXPECT_EQ(plan(planner, overflow_bounds_map, path), PlanStatus::INVALID_INPUT);
  EXPECT_TRUE(path.empty());
}

TEST(RRTStar, RejectsExtremeFiniteRequestCoordinatesBeforeCoordinateConversion)
{
  auto map = freeMap();
  RRTStar planner(parameters());
  std::vector<RRTStarNode> path(2);
  const double maximum = std::numeric_limits<double>::max();

  EXPECT_EQ(
    planner.planPath(
      maximum, 0.5, 4.5, 0.5, map, {},
      steady_clock::now() + std::chrono::seconds(1), path),
    PlanStatus::INVALID_INPUT);
  EXPECT_TRUE(path.empty());

  path.resize(2);
  EXPECT_EQ(
    planner.planPath(
      0.5, 0.5, -maximum, 0.5, map, {},
      steady_clock::now() + std::chrono::seconds(1), path),
    PlanStatus::INVALID_INPUT);
  EXPECT_TRUE(path.empty());
}

TEST(RRTStar, RejectsNonFiniteEdgeAndAccumulatedCosts)
{
  nav2_costmap_2d::Costmap2D high_cost_map(20, 20, 1.0, 0.0, 0.0, 252);
  auto params = parameters();
  params.cost_weight = std::numeric_limits<double>::max();
  params.goal_threshold = 5.0;
  params.max_optimize_iters = 0;
  RRTStar planner(params);
  std::vector<RRTStarNode> path(2);

  EXPECT_EQ(plan(planner, high_cost_map, path), PlanStatus::INVALID_INPUT);
  EXPECT_TRUE(path.empty());
}

TEST(RRTStar, ProducesExactGoalWithExactConnectionCost)
{
  auto map = freeMap();
  auto params = parameters();
  params.goal_threshold = 1.1;
  RRTStar planner(params);
  std::vector<RRTStarNode> path;
  ASSERT_EQ(plan(planner, map, path, 4.25, 0.5), PlanStatus::SUCCESS);
  ASSERT_GE(path.size(), 2U);
  EXPECT_DOUBLE_EQ(path.back().x, 4.25);
  EXPECT_DOUBLE_EQ(path.back().y, 0.5);
  EXPECT_NEAR(path.back().cost_from_root, 3.75, 1e-12);
}

TEST(RRTStar, StartEqualsGoalProducesTwoEndpointNodes)
{
  auto map = freeMap();
  RRTStar planner(parameters());
  std::vector<RRTStarNode> path;
  ASSERT_EQ(plan(planner, map, path, 0.5, 0.5), PlanStatus::SUCCESS);
  ASSERT_EQ(path.size(), 2U);
  EXPECT_DOUBLE_EQ(path.front().x, path.back().x);
  EXPECT_DOUBLE_EQ(path.front().y, path.back().y);
  EXPECT_EQ(path.back().parent_idx, 0);
}

TEST(RRTStar, TinyNonzeroGoalConnectionRetainsExactEndpointAndCost)
{
  auto map = freeMap();
  RRTStar planner(parameters());
  std::vector<RRTStarNode> path;
  constexpr double tiny_goal_x = 0.5 + 5e-10;

  ASSERT_EQ(plan(planner, map, path, tiny_goal_x, 0.5), PlanStatus::SUCCESS);
  ASSERT_EQ(path.size(), 2U);
  EXPECT_DOUBLE_EQ(path.back().x, tiny_goal_x);
  EXPECT_DOUBLE_EQ(path.back().y, 0.5);
  EXPECT_GT(path.back().cost_from_root, 0.0);
  EXPECT_DOUBLE_EQ(path.back().cost_from_root, tiny_goal_x - 0.5);
}

TEST(RRTStar, SamplesAndNodesStayWithinLegalCellCenterBounds)
{
  auto map = freeMap();
  auto params = parameters();
  params.goal_bias = 0.0;
  params.max_iterations = 200;
  params.goal_threshold = 0.01;
  RRTStar planner(params);
  std::vector<RRTStarNode> path;
  (void)plan(planner, map, path, 19.5, 19.5);
  for (const auto & node : RRTStarTestPeer::tree(planner)) {
    EXPECT_GE(node.x, 0.5);
    EXPECT_LE(node.x, 19.5);
    EXPECT_GE(node.y, 0.5);
    EXPECT_LE(node.y, 19.5);
  }
}

TEST(RRTStar, NearEdgeNonCenterEndpointsDoNotExpandTreeOutsideCenterBounds)
{
  auto map = freeMap();
  auto params = parameters();
  params.goal_bias = 1.0;
  params.goal_threshold = 0.8;
  params.max_optimize_iters = 20;
  RRTStar planner(params);
  std::vector<RRTStarNode> path;

  ASSERT_EQ(
    planner.planPath(
      0.01, 0.01, 19.99, 19.99, map, {}, {},
      steady_clock::now() + std::chrono::seconds(10), path),
    PlanStatus::SUCCESS);
  ASSERT_FALSE(path.empty());
  EXPECT_DOUBLE_EQ(path.front().x, 0.01);
  EXPECT_DOUBLE_EQ(path.back().x, 19.99);
  const auto & tree = RRTStarTestPeer::tree(planner);
  for (size_t i = 1; i < tree.size(); ++i) {
    EXPECT_GE(tree[i].x, 0.5);
    EXPECT_LE(tree[i].x, 19.5);
    EXPECT_GE(tree[i].y, 0.5);
    EXPECT_LE(tree[i].y, 19.5);
  }
}

TEST(RRTStar, TreeHasNoZeroLengthEdgesOrDuplicateNodes)
{
  auto map = freeMap();
  RRTStar planner(parameters());
  std::vector<RRTStarNode> path;
  ASSERT_EQ(plan(planner, map, path), PlanStatus::SUCCESS);
  const auto & tree = RRTStarTestPeer::tree(planner);
  for (size_t i = 1; i < tree.size(); ++i) {
    const auto & parent = tree.at(tree[i].parent_idx);
    EXPECT_GT(std::hypot(tree[i].x - parent.x, tree[i].y - parent.y), 1e-9);
    for (size_t j = 0; j < i; ++j) {
      EXPECT_GT(
        std::hypot(tree[i].x - tree[j].x, tree[i].y - tree[j].y), 1e-9);
    }
  }
}

TEST(RRTStar, RunsFixedOptimizationBudgetAfterFirstGoal)
{
  auto map = freeMap();
  RRTStar planner(parameters());
  std::vector<RRTStarNode> path;
  ASSERT_EQ(plan(planner, map, path), PlanStatus::SUCCESS);
  EXPECT_EQ(RRTStarTestPeer::iterations(planner), 7);
}

TEST(RRTStar, EveryDescendantCostMatchesItsCurrentParent)
{
  auto map = freeMap();
  auto params = parameters();
  params.goal_bias = 0.15;
  params.max_iterations = 500;
  params.goal_threshold = 0.5;
  params.max_optimize_iters = 100;
  RRTStar planner(params);
  std::vector<RRTStarNode> path;
  (void)plan(planner, map, path, 15.5, 15.5);
  const auto & tree = RRTStarTestPeer::tree(planner);
  for (size_t i = 1; i < tree.size(); ++i) {
    const auto & parent = tree.at(tree[i].parent_idx);
    EXPECT_NEAR(
      tree[i].cost_from_root,
      parent.cost_from_root + std::hypot(tree[i].x - parent.x, tree[i].y - parent.y),
      1e-9);
  }
}

TEST(RRTStar, RewiringPropagatesCostChangesToDescendants)
{
  auto map = freeMap();
  RRTStar planner(parameters());
  RRTStarTestPeer::setTree(
    planner,
    {{0.5, 0.5, -1, 0.0}, {5.5, 4.5, 0, 9.0}, {6.5, 4.5, 1, 10.0},
      {4.5, 3.5, 0, 5.0}});

  RRTStarTestPeer::rewire(planner, 3, {1}, map);

  const auto & tree = RRTStarTestPeer::tree(planner);
  EXPECT_EQ(tree[1].parent_idx, 3);
  EXPECT_NEAR(tree[1].cost_from_root, 5.0 + std::sqrt(2.0), 1e-9);
  EXPECT_NEAR(tree[2].cost_from_root, 6.0 + std::sqrt(2.0), 1e-9);
}

TEST(RRTStar, SameSeedProducesSamePath)
{
  auto map = freeMap();
  auto params = parameters();
  params.goal_bias = 0.2;
  RRTStar first(params);
  RRTStar second(params);
  std::vector<RRTStarNode> first_path;
  std::vector<RRTStarNode> second_path;
  ASSERT_EQ(plan(first, map, first_path), PlanStatus::SUCCESS);
  ASSERT_EQ(plan(second, map, second_path), PlanStatus::SUCCESS);
  ASSERT_EQ(first_path.size(), second_path.size());
  for (size_t i = 0; i < first_path.size(); ++i) {
    EXPECT_DOUBLE_EQ(first_path[i].x, second_path[i].x);
    EXPECT_DOUBLE_EQ(first_path[i].y, second_path[i].y);
    EXPECT_DOUBLE_EQ(first_path[i].cost_from_root, second_path[i].cost_from_root);
  }
}

TEST(RRTStar, HonorsCancellationAndDeadline)
{
  auto map = freeMap();
  RRTStar planner(parameters());
  std::vector<RRTStarNode> path;
  EXPECT_EQ(
    plan(planner, map, path, 4.5, 0.5, []() {return true;}),
    PlanStatus::CANCELED);
  EXPECT_EQ(
    planner.planPath(
      0.5, 0.5, 4.5, 0.5, map, {}, {}, steady_clock::now(), path),
    PlanStatus::TIMEOUT);
}

TEST(RRTStar, HonorsCancellationTriggeredInsidePlanningWork)
{
  auto map = freeMap();
  auto params = parameters();
  params.goal_bias = 0.0;
  params.max_iterations = 1000;
  RRTStar planner(params);
  std::vector<RRTStarNode> path(2);
  int checks = 0;

  EXPECT_EQ(
    plan(
      planner, map, path, 19.5, 19.5,
      [&checks]() {return ++checks >= 20;}),
    PlanStatus::CANCELED);
  EXPECT_GE(checks, 20);
  EXPECT_TRUE(path.empty());
}

TEST(RRTStar, HonorsDeadlineReachedInsidePlanningWork)
{
  auto map = freeMap();
  auto params = parameters();
  params.goal_bias = 0.0;
  params.max_iterations = 1000;
  RRTStar planner(params);
  std::vector<RRTStarNode> path;
  int checks = 0;
  const auto deadline = steady_clock::now() + std::chrono::milliseconds(100);

  EXPECT_EQ(
    planner.planPath(
      0.5, 0.5, 19.5, 19.5, map, {},
      [&checks]() {
        if (++checks == 20) {
          std::this_thread::sleep_for(std::chrono::milliseconds(150));
        }
        return false;
      },
      deadline, path),
    PlanStatus::TIMEOUT);
  EXPECT_GE(checks, 20);
  EXPECT_TRUE(path.empty());
}

TEST(RRTStar, RejectsUnrepresentableSafetyAndSegmentSampleCounts)
{
  auto map = freeMap();
  auto params = parameters();
  params.safety_dist = std::numeric_limits<double>::max();
  RRTStar planner(params);
  std::vector<RRTStarNode> path;
  EXPECT_EQ(plan(planner, map, path), PlanStatus::INVALID_INPUT);

  RRTStar edge_planner(parameters());
  EXPECT_FALSE(
    RRTStarTestPeer::collisionFree(
      edge_planner, 0.5, 0.5, std::numeric_limits<double>::max(), 0.5, map));
  EXPECT_TRUE(RRTStarTestPeer::invalidGeometry(edge_planner));
}

TEST(RRTStar, ReportsNoPathWhenGoalIsBlocked)
{
  auto map = freeMap();
  map.setCost(4, 0, nav2_costmap_2d::LETHAL_OBSTACLE);
  RRTStar planner(parameters());
  std::vector<RRTStarNode> path;
  EXPECT_EQ(plan(planner, map, path), PlanStatus::NO_PATH);
}

TEST(RRTStar, SafetyDistanceUsesQueryToCellRectangleDistance)
{
  auto map = freeMap();
  map.setCost(1, 1, nav2_costmap_2d::LETHAL_OBSTACLE);
  auto params = parameters();
  params.safety_dist = 0.05;
  RRTStar planner(params);

  EXPECT_FALSE(RRTStarTestPeer::pointCollisionFree(planner, 0.99, 0.99, map));
}

TEST(RRTStar, PruningPreservesExactEndpoints)
{
  auto map = freeMap();
  auto params = parameters();
  params.prune_path = true;
  RRTStar planner(params);
  std::vector<RRTStarNode> path;
  ASSERT_EQ(plan(planner, map, path, 4.25, 0.5), PlanStatus::SUCCESS);
  ASSERT_EQ(path.size(), 2U);
  EXPECT_DOUBLE_EQ(path.front().x, 0.5);
  EXPECT_DOUBLE_EQ(path.front().y, 0.5);
  EXPECT_DOUBLE_EQ(path.back().x, 4.25);
  EXPECT_DOUBLE_EQ(path.back().y, 0.5);
}

geometry_msgs::msg::Point barrierPoint(double x, double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = 0.0;
  return point;
}

TEST(RRTStar, RejectsNonFiniteBarrierPoints)
{
  auto map = freeMap();
  RRTStar planner(parameters());
  std::vector<RRTStarNode> path;
  std::vector<geometry_msgs::msg::Point> barriers;
  barriers.push_back(barrierPoint(2.5, -5.0));
  barriers.push_back(
    barrierPoint(std::numeric_limits<double>::quiet_NaN(), 25.0));

  EXPECT_EQ(
    planner.planPath(
      0.5, 0.5, 4.5, 0.5, map, barriers, {},
      steady_clock::now() + std::chrono::seconds(10), path),
    PlanStatus::INVALID_INPUT);
  EXPECT_TRUE(path.empty());
}

TEST(RRTStar, SpanningBarrierBlocksEveryPath)
{
  auto map = freeMap();
  RRTStar planner(parameters());
  std::vector<RRTStarNode> path;
  // Vertical barrier crossing the whole map between start (0.5, 0.5) and
  // goal (4.5, 0.5): any path must intersect it.
  std::vector<geometry_msgs::msg::Point> barriers;
  barriers.push_back(barrierPoint(2.5, -5.0));
  barriers.push_back(barrierPoint(2.5, 25.0));

  EXPECT_EQ(
    plan(planner, map, path, 4.5, 0.5, {}, barriers),
    PlanStatus::NO_PATH);
  EXPECT_TRUE(path.empty());
}

TEST(RRTStar, PartialBarrierForcesDetourAroundItsEnd)
{
  auto map = freeMap();
  auto params = parameters();
  params.max_iterations = 2000;
  RRTStar planner(params);
  std::vector<RRTStarNode> path;
  // Barrier only covers y < 10 at x = 2.5, so the detour must pass above
  // its end before crossing to the goal side.
  std::vector<geometry_msgs::msg::Point> barriers;
  barriers.push_back(barrierPoint(2.5, -5.0));
  barriers.push_back(barrierPoint(2.5, 10.0));

  const auto status = plan(planner, map, path, 4.5, 0.5, {}, barriers);
  ASSERT_EQ(status, PlanStatus::SUCCESS);
  ASSERT_GE(path.size(), 2u);
  EXPECT_DOUBLE_EQ(path.front().x, 0.5);
  EXPECT_DOUBLE_EQ(path.back().x, 4.5);
  for (size_t i = 1; i < path.size(); ++i) {
    const double x1 = path[i - 1].x;
    const double y1 = path[i - 1].y;
    const double x2 = path[i].x;
    const double y2 = path[i].y;
    const bool straddles = (x1 < 2.5 && x2 > 2.5) || (x1 > 2.5 && x2 < 2.5);
    if (straddles) {
      // A detour edge may cross the barrier line only above its end.
      const double ratio = (2.5 - x1) / (x2 - x1);
      const double crossing_y = y1 + ratio * (y2 - y1);
      ASSERT_GT(crossing_y, 10.0);
    }
    if (x1 == 2.5 || x2 == 2.5) {
      // Endpoints exactly on the barrier line must lie above its end.
      const double on_line_y = x1 == 2.5 ? y1 : y2;
      ASSERT_GT(on_line_y, 10.0);
    }
  }
}

}  // namespace
}  // namespace nav2_colregs_local_planner_server
