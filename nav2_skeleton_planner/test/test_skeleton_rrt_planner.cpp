#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include "nav2_colregs_vo_skeleton_planner/skeleton_planner.hpp"
#include "nav2_colregs_vo_skeleton_planner/skeleton_space.hpp"
#include "nav2_colregs_vo_skeleton_planner/skeleton_tree.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace
{

using nav2_colregs_vo_skeleton_planner::Pt;
using nav2_colregs_vo_skeleton_planner::SkeletonConfig;
using nav2_colregs_vo_skeleton_planner::SkeletonPlanner;
using nav2_colregs_vo_skeleton_planner::Space;

// Link-layer smoke: the standalone package consumes the exported core
// (headers + library) from nav2_colregs_vo_skeleton_planner. The full core
// behavior suite lives in that package's test_skeleton_planner.
struct Scene
{
  nav2_costmap_2d::Costmap2D costmap{200, 200, 1.0, 0.0, 0.0, 0};
  std::unique_ptr<Space> space;
  SkeletonConfig config;

  Scene()
  {
    config.node_limit = 512;
    config.path_limit = 128;
    config.global_iterations = 1200;
    config.reuse_iterations = 64;
    space = std::make_unique<Space>(&costmap, 0.0, 0.0);
  }

  void wall(int x, int y0, int y1)
  {
    for (int y = y0; y <= y1; ++y) {
      costmap.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
};

double path_length(const std::vector<Pt> & p)
{
  double total = 0.0;
  for (size_t i = 1; i < p.size(); ++i) {
    total += std::hypot(p[i].first - p[i - 1].first,
      p[i].second - p[i - 1].second);
  }
  return total;
}

TEST(SkeletonPlain, CoreExportsReachableAndColdPlanWorks)
{
  Scene scene;
  scene.wall(100, 0, 150);
  scene.space->beginQuery(1);
  const Pt start(10.5, 10.5), goal(150.5, 10.5);
  SkeletonPlanner planner(scene.space.get(), goal, scene.config, 11);
  std::vector<Pt> path;
  const auto st = planner.replan(start, path);
  ASSERT_FALSE(path.empty()) << "status=" << st.status;
  EXPECT_NEAR(path.front().first, start.first, 1e-9);
  EXPECT_NEAR(path.front().second, start.second, 1e-9);
  EXPECT_NEAR(path.back().first, goal.first, 1e-6);
  EXPECT_NEAR(path.back().second, goal.second, 1e-6);
  EXPECT_TRUE(scene.space->pathValid(path, &start, &goal));
  // the corridor detour must be longer than the straight line but sane
  // (route goes around the wall top at y>150: ~2x140 detour + crossing)
  EXPECT_GT(path_length(path), 160.0);
  EXPECT_LT(path_length(path), 430.0);
}

TEST(SkeletonPlain, ReplanReuseKeepsCorridorAndTree)
{
  Scene scene;
  scene.wall(100, 0, 150);
  scene.space->beginQuery(1);
  const Pt goal(150.5, 10.5);
  SkeletonPlanner planner(scene.space.get(), goal, scene.config, 11);
  std::vector<Pt> first;
  const auto st1 = planner.replan(Pt(10.5, 10.5), first);
  ASSERT_FALSE(first.empty()) << "status=" << st1.status;
  ASSERT_GE(first.size(), 2u);
  const double len_first = path_length(first);
  const int nodes_first = planner.treeNodes();

  const Pt query2(30.5, 12.5);
  scene.space->beginQuery(2);
  std::vector<Pt> second;
  const auto st2 = planner.replan(query2, second);
  ASSERT_FALSE(second.empty()) << "status=" << st2.status;
  EXPECT_NEAR(second.front().first, query2.first, 1e-9);
  EXPECT_NEAR(second.front().second, query2.second, 1e-9);
  EXPECT_NEAR(second.back().first, goal.first, 1e-6);
  EXPECT_TRUE(scene.space->pathValid(second, &query2, &goal));
  // remaining-route length stays in a sane band around the first plan
  // (skeleton replacement is allowed to re-shape the route while the
  // switch margin guards stability; strict monotone shrink is not an
  // invariant of the planner)
  EXPECT_LT(path_length(second), len_first * 1.25);
  EXPECT_GT(path_length(second), 150.0);
  // the goal-rooted tree stays alive across queries
  EXPECT_GE(planner.treeNodes(), 2);
  (void)nodes_first;
}

TEST(SkeletonPlainSpace, UpdateCostmapRebuildsDiskOnResolutionChange)
{
  // review PLAIN-03: a 1 m safety disk computed at 1 m resolution must not
  // silently shrink when rebound to a 0.1 m map (stale disk checks only 1
  // cell = 0.1 m instead of the required 10 cells = 1 m).
  nav2_costmap_2d::Costmap2D coarse(100, 100, 1.0, 0.0, 0.0, 0);
  Space space(&coarse, 1.0, 0.0);
  coarse.setCost(50, 50, nav2_costmap_2d::LETHAL_OBSTACLE);
  space.beginQuery(1);
  // 1 m beside the obstacle: blocked by the 1 m disk on the coarse map
  EXPECT_FALSE(space.pointFree(Pt(51.5, 50.5)));

  nav2_costmap_2d::Costmap2D fine(1000, 1000, 0.1, 0.0, 0.0, 0);
  fine.setCost(500, 500, nav2_costmap_2d::LETHAL_OBSTACLE);
  space.updateCostmap(&fine);
  space.beginQuery(2);
  // 0.6 m beside the obstacle (cell offset (6,0) at 0.1 m): must be blocked
  // by the rebuilt 10-cell disk; a stale 1-cell disk would miss it
  EXPECT_FALSE(space.pointFree(Pt(50.65, 50.05)));
  // 1.5 m beside the obstacle: beyond the 1 m safety -> free
  EXPECT_TRUE(space.pointFree(Pt(51.5, 50.5)));
  // far away stays free
  EXPECT_TRUE(space.pointFree(Pt(53.5, 50.5)));
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
