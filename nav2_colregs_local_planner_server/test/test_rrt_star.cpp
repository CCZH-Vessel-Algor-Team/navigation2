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

  static const std::vector<std::vector<int>> & children(const RRTStar & planner)
  {
    return planner.children_;
  }

  static bool requestReleased(const RRTStar & planner)
  {
    return planner.cancel_checker_ == nullptr && planner.barriers_ == nullptr;
  }

  static const std::vector<double> & edgeCosts(const RRTStar & planner)
  {
    return planner.edge_costs_;
  }

  static void setTree(
    RRTStar & planner, std::vector<RRTStarNode> tree,
    const nav2_costmap_2d::Costmap2D & costmap)
  {
    prepareOperation(planner);
    planner.tree_ = std::move(tree);
    planner.children_.assign(planner.tree_.size(), {});
    planner.edge_costs_.assign(planner.tree_.size(), 0.0);
    for (size_t i = 0; i < planner.tree_.size(); ++i) {
      const int parent = planner.tree_[i].parent_idx;
      if (parent >= 0) {
        planner.children_.at(parent).push_back(static_cast<int>(i));
        const auto & from = planner.tree_.at(parent);
        const auto & to = planner.tree_[i];
        planner.edge_costs_[i] = planner.edgeCost(from.x, from.y, to.x, to.y, costmap);
      }
    }
  }

  static bool rewire(
    RRTStar & planner, int new_idx, const std::vector<int> & near,
    const nav2_costmap_2d::Costmap2D & costmap,
    const std::function<bool()> & cancel = {})
  {
    prepareOperation(planner);
    planner.cancel_checker_ = &cancel;
    const bool result = planner.rewire(new_idx, near, costmap);
    planner.cancel_checker_ = nullptr;
    return result;
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
    planner.interruption_checks_ = 0;
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

void expectConsistentTree(const RRTStar & planner)
{
  const auto & tree = RRTStarTestPeer::tree(planner);
  const auto & children = RRTStarTestPeer::children(planner);
  const auto & edge_costs = RRTStarTestPeer::edgeCosts(planner);
  ASSERT_EQ(children.size(), tree.size());
  ASSERT_EQ(edge_costs.size(), tree.size());
  std::vector<int> incoming(tree.size(), 0);
  for (size_t parent = 0; parent < children.size(); ++parent) {
    for (const int child : children[parent]) {
      ASSERT_GE(child, 0);
      ASSERT_LT(static_cast<size_t>(child), tree.size());
      EXPECT_EQ(tree[child].parent_idx, static_cast<int>(parent));
      ++incoming[child];
    }
  }
  for (size_t i = 0; i < tree.size(); ++i) {
    EXPECT_EQ(incoming[i], i == 0 ? 0 : 1);
    EXPECT_TRUE(std::isfinite(tree[i].cost_from_root));
    EXPECT_TRUE(std::isfinite(edge_costs[i]));
    EXPECT_GE(edge_costs[i], 0.0);
    int ancestor = static_cast<int>(i);
    size_t depth = 0;
    while (ancestor >= 0 && depth <= tree.size()) {
      ASSERT_LT(static_cast<size_t>(ancestor), tree.size());
      ancestor = tree[ancestor].parent_idx;
      ++depth;
    }
    EXPECT_LE(depth, tree.size()) << "Cycle from node " << i;
  }
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
      maximum, 0.5, 4.5, 0.5, map, {}, {},
      steady_clock::now() + std::chrono::seconds(1), path),
    PlanStatus::INVALID_INPUT);
  EXPECT_TRUE(path.empty());

  path.resize(2);
  EXPECT_EQ(
    planner.planPath(
      0.5, 0.5, -maximum, 0.5, map, {}, {},
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
  expectConsistentTree(planner);
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
      {4.5, 3.5, 0, 5.0}}, map);

  ASSERT_TRUE(RRTStarTestPeer::rewire(planner, 3, {1}, map));
  expectConsistentTree(planner);

  const auto & tree = RRTStarTestPeer::tree(planner);
  EXPECT_EQ(tree[1].parent_idx, 3);
  EXPECT_NEAR(tree[1].cost_from_root, 5.0 + std::sqrt(2.0), 1e-9);
  EXPECT_NEAR(tree[2].cost_from_root, 6.0 + std::sqrt(2.0), 1e-9);
}

TEST(RRTStar, RepeatedRewiringUpdatesBranchedWeightedSubtree)
{
  nav2_costmap_2d::Costmap2D map(20, 20, 1.0, 0.0, 0.0, 127);
  auto params = parameters();
  params.cost_weight = 2.0;  // Every edge costs twice its length.
  RRTStar planner(params);
  RRTStarTestPeer::setTree(
    planner,
    {{0.5, 0.5, -1, 0.0}, {0.5, 4.5, 0, 8.0}, {5.5, 4.5, 1, 18.0},
      {6.5, 4.5, 2, 20.0}, {7.5, 4.5, 3, 22.0}, {5.5, 5.5, 2, 20.0},
      {4.5, 3.5, 0, 10.0}, {3.0, 2.5, 0, std::sqrt(41.0)}}, map);

  ASSERT_TRUE(RRTStarTestPeer::rewire(planner, 6, {2}, map));
  expectConsistentTree(planner);
  EXPECT_EQ(RRTStarTestPeer::tree(planner)[2].parent_idx, 6);
  EXPECT_NEAR(
    RRTStarTestPeer::tree(planner)[4].cost_from_root,
    14.0 + 2.0 * std::sqrt(2.0), 1e-12);
  // Reparent the same subtree again; its old adjacency must no longer retain it.
  ASSERT_TRUE(RRTStarTestPeer::rewire(planner, 7, {2}, map));
  expectConsistentTree(planner);
  const auto & tree = RRTStarTestPeer::tree(planner);
  EXPECT_EQ(tree[2].parent_idx, 7);
  EXPECT_TRUE(RRTStarTestPeer::children(planner)[6].empty());
  for (size_t i = 1; i < tree.size(); ++i) {
    const auto & parent = tree[tree[i].parent_idx];
    EXPECT_NEAR(
      tree[i].cost_from_root,
      parent.cost_from_root + 2.0 * std::hypot(tree[i].x - parent.x, tree[i].y - parent.y),
      1e-12);
  }
}

TEST(RRTStar, RewiringRecoversShortEdgesRoundedOutOfLargeCumulativeCosts)
{
  auto map = freeMap();
  map.setCost(0, 1, 127);
  auto params = parameters();
  params.cost_weight = std::ldexp(1.0, 55);
  const double old_cost = std::ldexp(1.0, 54);
  const double rounded_child_cost = old_cost + 1.0;
  ASSERT_EQ(rounded_child_cost, old_cost);
  RRTStar planner(params);
  // Only the root->1 midpoint hits the high-weight cell. The unit edges
  // 1->2->3 are free; the alternate branch 0->4->5->6 has cost 7.
  RRTStarTestPeer::setTree(
    planner,
    {{0.5, 0.5, -1, 0.0}, {0.5, 1.5, 0, old_cost},
      {1.5, 1.5, 1, rounded_child_cost}, {2.5, 1.5, 2, rounded_child_cost + 1.0},
      {3.0, 0.5, 0, 2.5}, {3.0, 2.5, 4, 4.5}, {0.5, 2.5, 5, 7.0}}, map);
  ASSERT_EQ(RRTStarTestPeer::edgeCosts(planner)[1], old_cost);
  ASSERT_EQ(RRTStarTestPeer::edgeCosts(planner)[2], 1.0);
  ASSERT_TRUE(RRTStarTestPeer::rewire(planner, 6, {1}, map));
  expectConsistentTree(planner);
  const auto & tree = RRTStarTestPeer::tree(planner);
  EXPECT_EQ(tree[1].parent_idx, 6);
  EXPECT_EQ(tree[1].cost_from_root, 8.0);
  EXPECT_EQ(tree[2].cost_from_root, 9.0);
  EXPECT_EQ(tree[3].cost_from_root, 10.0);
  EXPECT_EQ(RRTStarTestPeer::edgeCosts(planner)[1], 1.0);
}

TEST(RRTStar, RewireCannotCreateCycleEvenWithInconsistentAncestorCosts)
{
  auto map = freeMap();
  RRTStar planner(parameters());
  RRTStarTestPeer::setTree(
    planner,
    {{0.5, 0.5, -1, 0.0}, {1.5, 0.5, 0, 100.0},
      {2.5, 0.5, 1, 101.0}, {3.5, 0.5, 2, 1.0}}, map);
  ASSERT_TRUE(RRTStarTestPeer::rewire(planner, 3, {0, 1, 2, 3}, map));
  expectConsistentTree(planner);
  EXPECT_EQ(RRTStarTestPeer::tree(planner)[1].parent_idx, 0);
}

TEST(RRTStar, CancellationDuringSubtreeTranslationDiscardsPartialTree)
{
  auto map = freeMap();
  RRTStar planner(parameters());
  std::vector<RRTStarNode> tree{
    {0.5, 0.5, -1, 0.0}, {5.5, 4.5, 0, 9.0}, {4.5, 3.5, 0, 5.0}};
  for (int i = 0; i < 256; ++i) {
    tree.push_back({5.51 + i * 0.01, 4.5, i == 0 ? 1 : i + 2, 9.01 + i * 0.01});
  }
  RRTStarTestPeer::setTree(planner, std::move(tree), map);
  bool canceled_subtree = false;
  EXPECT_FALSE(
    RRTStarTestPeer::rewire(
      planner, 2, {1}, map, [&]() {
        canceled_subtree = RRTStarTestPeer::tree(planner)[1].parent_idx == 2;
        return canceled_subtree;
      }));
  EXPECT_TRUE(canceled_subtree);
  EXPECT_TRUE(RRTStarTestPeer::tree(planner).empty());
  expectConsistentTree(planner);
  std::vector<RRTStarNode> path;
  ASSERT_EQ(plan(planner, map, path), PlanStatus::SUCCESS);
  expectConsistentTree(planner);
}

TEST(RRTStar, NonFiniteDescendantDiscardsPartialTree)
{
  auto map = freeMap();
  RRTStar planner(parameters());
  RRTStarTestPeer::setTree(
    planner,
    {{0.5, 0.5, -1, 0.0}, {5.5, 4.5, 0, 9.0}, {6.5, 4.5, 1, 10.0},
      {7.5, 4.5, 2, std::numeric_limits<double>::infinity()}, {4.5, 3.5, 0, 5.0}}, map);
  EXPECT_FALSE(RRTStarTestPeer::rewire(planner, 4, {1}, map));
  EXPECT_TRUE(RRTStarTestPeer::invalidGeometry(planner));
  EXPECT_TRUE(RRTStarTestPeer::tree(planner).empty());
  expectConsistentTree(planner);
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

TEST(RRTStar, DenseCollisionValidationBatchesExpensiveCallbacks)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 1.0, 0.0, 0.0, 0);
  auto params = parameters();
  params.safety_dist = 10.0;
  RRTStar planner(params);
  std::vector<RRTStarNode> path;
  int callbacks = 0;
  ASSERT_EQ(
    planner.planPath(
      100.5, 100.5, 100.5, 100.5, map, {}, [&]() {
        ++callbacks;
        return false;
      }, steady_clock::now() + std::chrono::seconds(10), path), PlanStatus::SUCCESS);
  // Two 21x21 candidate-cell scans plus boundary checks: hundreds of cheap
  // checkpoints must not become hundreds of Server-lock/clock acquisitions.
  EXPECT_GT(callbacks, 3);
  EXPECT_LE(callbacks, 20);
  ASSERT_EQ(path.size(), 2U);
}

TEST(RRTStar, HonorsCancellationTriggeredInsidePlanningWork)
{
  auto map = freeMap();
  auto params = parameters();
  params.goal_bias = 0.0;
  params.max_iterations = 1000;
  RRTStar planner(params);
  std::vector<RRTStarNode> path(2);
  bool canceled_during_search = false;

  EXPECT_EQ(
    plan(
      planner, map, path, 19.5, 19.5,
      [&]() {
        canceled_during_search = RRTStarTestPeer::iterations(planner) >= 2;
        return canceled_during_search;
      }),
    PlanStatus::CANCELED);
  EXPECT_TRUE(canceled_during_search);
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
  bool expired_during_search = false;
  const auto deadline = steady_clock::now() + std::chrono::seconds(1);

  EXPECT_EQ(
    planner.planPath(
      0.5, 0.5, 19.5, 19.5, map, {},
      [&]() {
        if (RRTStarTestPeer::iterations(planner) >= 2) {
          expired_during_search = true;
          std::this_thread::sleep_until(deadline + std::chrono::milliseconds(1));
        }
        return false;
      },
      deadline, path),
    PlanStatus::TIMEOUT);
  EXPECT_TRUE(expired_during_search);
  EXPECT_TRUE(path.empty());
}

TEST(RRTStar, FinalCallbackCanCancelBothNormalAndCoincidentEndpointPaths)
{
  auto map = freeMap();
  for (const bool prune : {false, true}) {
    for (const double goal_x : {0.5, 4.5}) {
      auto params = parameters();
      params.prune_path = prune;
      RRTStar planner(params);
      std::vector<RRTStarNode> path;
      bool saw_completed_path = false;
      EXPECT_EQ(
        plan(
          planner, map, path, goal_x, 0.5, [&]() {
            saw_completed_path = !path.empty() && (!prune || path.size() == 2);
            return saw_completed_path;
          }), PlanStatus::CANCELED);
      EXPECT_TRUE(saw_completed_path);
      EXPECT_TRUE(path.empty());
      EXPECT_TRUE(RRTStarTestPeer::requestReleased(planner));
    }
  }
}

TEST(RRTStar, DeadlineExpiringInFinalCallbackCannotReturnSuccess)
{
  auto map = freeMap();
  RRTStar planner(parameters());
  std::vector<RRTStarNode> path;
  const auto deadline = steady_clock::now() + std::chrono::seconds(1);
  bool saw_completed_path = false;
  EXPECT_EQ(
    planner.planPath(
      0.5, 0.5, 0.5, 0.5, map, {}, [&]() {
        if (!path.empty()) {
          saw_completed_path = true;
          std::this_thread::sleep_until(deadline + std::chrono::milliseconds(1));
        }
        return false;
      }, deadline, path), PlanStatus::TIMEOUT);
  EXPECT_TRUE(saw_completed_path);
  EXPECT_TRUE(path.empty());
}

TEST(RRTStar, ReusedPlannerResetsRequestStateAndAdjacency)
{
  auto map = freeMap();
  auto params = parameters();
  params.goal_bias = 0.2;
  RRTStar planner(params);
  RRTStar fresh(params);
  std::vector<RRTStarNode> path;
  std::vector<RRTStarNode> expected;
  ASSERT_EQ(plan(fresh, map, expected), PlanStatus::SUCCESS);
  for (int request = 0; request < 2; ++request) {
    EXPECT_EQ(
      plan(planner, map, path, 4.5, 0.5, []() {return true;}),
      PlanStatus::CANCELED);
    EXPECT_TRUE(path.empty());
    EXPECT_TRUE(RRTStarTestPeer::requestReleased(planner));
    EXPECT_EQ(
      planner.planPath(
        0.5, 0.5, 4.5, 0.5, map, {}, {},
        steady_clock::now(), path), PlanStatus::TIMEOUT);
    ASSERT_EQ(plan(planner, map, path), PlanStatus::SUCCESS);
    expectConsistentTree(planner);
    EXPECT_TRUE(RRTStarTestPeer::requestReleased(planner));
    ASSERT_EQ(path.size(), expected.size());
    for (size_t i = 0; i < path.size(); ++i) {
      EXPECT_DOUBLE_EQ(path[i].x, expected[i].x);
      EXPECT_DOUBLE_EQ(path[i].y, expected[i].y);
      EXPECT_DOUBLE_EQ(path[i].cost_from_root, expected[i].cost_from_root);
    }
  }
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
  params.goal_bias = 0.1;
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
    const bool straddles = (x1<2.5 && x2>2.5) || (x1 > 2.5 && x2 < 2.5);
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
