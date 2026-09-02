#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_rrt_star_planner/rrt_star.hpp"

namespace
{

using nav2_rrt_star_planner::RRTStar;
using nav2_rrt_star_planner::RRTStarNode;

nav2_costmap_2d::Costmap2D makeCostmap()
{
  return nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0.0, 0.0, 0);
}

TEST(RRTStar, terminates_threshold_path_at_requested_goal)
{
  auto costmap = makeCostmap();
  RRTStar planner(1.0, 2, 1.0, 1.1, 0.0, 0.0, 0, 5.0);
  std::vector<RRTStarNode> path;
  constexpr double goal_x = 4.0;
  constexpr double goal_y = 1.0;

  ASSERT_TRUE(planner.planPath(1.0, 1.0, goal_x, goal_y, &costmap, path));
  ASSERT_FALSE(path.empty());
  EXPECT_DOUBLE_EQ(path.back().x, goal_x);
  EXPECT_DOUBLE_EQ(path.back().y, goal_y);

  planner.prunePath(path, &costmap);
  ASSERT_FALSE(path.empty());
  EXPECT_DOUBLE_EQ(path.back().x, goal_x);
  EXPECT_DOUBLE_EQ(path.back().y, goal_y);
}

TEST(RRTStar, terminates_approximate_fallback_at_requested_goal)
{
  auto costmap = makeCostmap();
  RRTStar planner(1.0, 1, 1.0, 0.1, 0.0, 0.0, 0, 5.0);
  std::vector<RRTStarNode> path;
  constexpr double goal_x = 4.0;
  constexpr double goal_y = 1.0;

  ASSERT_TRUE(planner.planPath(1.0, 1.0, goal_x, goal_y, &costmap, path));
  ASSERT_FALSE(path.empty());
  EXPECT_DOUBLE_EQ(path.back().x, goal_x);
  EXPECT_DOUBLE_EQ(path.back().y, goal_y);
}

TEST(RRTStar, does_not_duplicate_existing_goal_node)
{
  auto costmap = makeCostmap();
  RRTStar planner(5.0, 1, 1.0, 0.1, 0.0, 0.0, 0, 5.0);
  std::vector<RRTStarNode> path;
  constexpr double goal_x = 4.0;
  constexpr double goal_y = 1.0;

  ASSERT_TRUE(planner.planPath(1.0, 1.0, goal_x, goal_y, &costmap, path));
  const auto goal_count = std::count_if(
    path.begin(), path.end(),
    [goal_x, goal_y](const RRTStarNode & node) {
      return node.x == goal_x && node.y == goal_y;
    });
  EXPECT_EQ(goal_count, 1);
}

double pathLength(const std::vector<RRTStarNode> & path)
{
  double total = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    total += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return total;
}

TEST(RRTStar, optimization_phase_extends_and_never_worsens_path)
{
  // 10 m x 10 m map with a vertical wall at x = 5 leaving a gap at the top:
  // extra optimization iterations must never lengthen the extracted path.
  auto costmap = nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0.0, 0.0, 0);
  for (unsigned int my = 0; my < 60; ++my) {
    costmap.setCost(50, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  constexpr double goal_x = 9.0;
  constexpr double goal_y = 1.0;

  // Same seed => identical exploration prefix; the optimized variant only
  // appends rewire/compete iterations, so its best solution can only improve.
  RRTStar baseline(1.0, 250, 0.1, 0.8, 0.0, 0.0, 0, 5.0);
  baseline.seedForTesting(42u);
  std::vector<RRTStarNode> path_baseline;
  ASSERT_TRUE(baseline.planPath(1.0, 1.0, goal_x, goal_y, &costmap, path_baseline));

  RRTStar optimized(1.0, 250, 0.1, 0.8, 0.0, 0.0, 500, 5.0);
  optimized.seedForTesting(42u);
  std::vector<RRTStarNode> path_optimized;
  ASSERT_TRUE(optimized.planPath(1.0, 1.0, goal_x, goal_y, &costmap, path_optimized));

  EXPECT_LE(pathLength(path_optimized), pathLength(path_baseline) + 1e-9);
}

TEST(RRTStar, extracted_path_costs_are_chain_consistent)
{
  auto costmap = makeCostmap();
  RRTStar planner(0.8, 800, 0.05, 0.8, 0.0, 0.0, 300, 6.0);
  planner.seedForTesting(7u);
  std::vector<RRTStarNode> path;
  ASSERT_TRUE(planner.planPath(1.0, 1.0, 8.5, 5.0, &costmap, path));
  ASSERT_GT(path.size(), 2u);

  // cost_weight = 0 => edge cost is exact euclidean length. Every extracted
  // hop must satisfy cost(i) == cost(i-1) + |edge|; stale subtree costs from
  // rewiring (without propagation) break this invariant.
  for (size_t i = 1; i < path.size(); ++i) {
    const double edge = std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    EXPECT_NEAR(path[i].cost_from_root - path[i - 1].cost_from_root, edge, 1e-6);
  }
}

TEST(RRTStar, informed_sampling_improves_wall_gap_scenario)
{
  // 100 m x 100 m map, vertical wall at x = 50 m with a gap at the top.
  // Same seeds => deterministic; informed ellipsoidal sampling (seeded from
  // the fallback chord) must beat uniform box sampling on total length.
  auto costmap = nav2_costmap_2d::Costmap2D(1000, 1000, 0.1, 0.0, 0.0, 0);
  for (unsigned int my = 0; my < 700; ++my) {
    costmap.setCost(500, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  double uniform_total = 0.0;
  double informed_total = 0.0;
  for (uint32_t seed = 1; seed <= 10u; ++seed) {
    std::vector<RRTStarNode> path_uniform;
    RRTStar uniform(2.0, 600, 0.1, 1.0, 0.0, 0.0, 0, 5.0, false);
    uniform.seedForTesting(seed);
    ASSERT_TRUE(uniform.planPath(2.0, 2.0, 98.0, 2.0, &costmap, path_uniform));

    std::vector<RRTStarNode> path_informed;
    RRTStar informed(2.0, 600, 0.1, 1.0, 0.0, 0.0, 0, 5.0, true);
    informed.seedForTesting(seed);
    ASSERT_TRUE(informed.planPath(2.0, 2.0, 98.0, 2.0, &costmap, path_informed));

    uniform_total += pathLength(path_uniform);
    informed_total += pathLength(path_informed);
  }
  EXPECT_LT(informed_total, uniform_total);
}

TEST(RRTStar, informed_planning_completes_quickly)
{
  // 600 m x 600 m map, wall at x = 300 m with a top gap: informed sampling
  // with a full optimize budget must stay far below interactive latencies.
  // Guards against the historical regressions of per-iteration LOS scans
  // and per-rewire-call adjacency rebuilds.
  auto costmap = nav2_costmap_2d::Costmap2D(600, 600, 1.0, 0.0, 0.0, 0);
  for (unsigned int my = 0; my < 450; ++my) {
    costmap.setCost(300, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  RRTStar informed(2.0, 5000, 0.1, 2.0, 1.5, 0.3, 5000, 50.0, true);
  informed.seedForTesting(42u);
  std::vector<RRTStarNode> path;
  const auto t0 = std::chrono::steady_clock::now();
  ASSERT_TRUE(informed.planPath(10.0, 10.0, 590.0, 10.0, &costmap, path));
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - t0).count();
  EXPECT_GT(path.size(), 2u);
  std::cout << "informed plan elapsed: " << elapsed << " s\n";
  EXPECT_LT(elapsed, 1.5);
}

}  // namespace
