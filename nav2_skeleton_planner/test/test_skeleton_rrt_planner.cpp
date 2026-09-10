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

TEST(SkeletonPlain, CoreExportsReachableAndColdPlanWorks)
{
  Scene scene;
  scene.wall(100, 0, 150);
  scene.space->beginQuery(1);
  SkeletonPlanner planner(scene.space.get(), Pt(150.5, 10.5), scene.config, 11);
  std::vector<Pt> path;
  const auto st = planner.replan(Pt(10.5, 10.5), path);
  ASSERT_FALSE(path.empty()) << st.status;
  EXPECT_NEAR(path.back().first, 150.5, 1e-6);
  EXPECT_NEAR(path.back().second, 10.5, 1e-6);
  EXPECT_TRUE(scene.space->pathValid(path));
}

TEST(SkeletonPlain, ReplanReuseIsCheapAndStable)
{
  Scene scene;
  scene.wall(100, 0, 150);
  scene.space->beginQuery(1);
  SkeletonPlanner planner(scene.space.get(), Pt(150.5, 10.5), scene.config, 11);
  std::vector<Pt> first;
  ASSERT_TRUE(!planner.replan(Pt(10.5, 10.5), first).status.empty());
  const double len_first = std::hypot(
    first.back().first - first.front().first,
    first.back().second - first.front().second);

  scene.space->beginQuery(2);
  std::vector<Pt> second;
  const auto st = planner.replan(Pt(30.5, 12.5), second);
  ASSERT_FALSE(second.empty()) << st.status;
  // same corridor: the goal is reached with a comparable-length route
  EXPECT_NEAR(second.back().first, 150.5, 1e-6);
  const double len_second = std::hypot(
    second.back().first - second.front().first,
    second.back().second - second.front().second);
  EXPECT_LT(len_second, len_first * 1.2);
  EXPECT_GT(planner.treeNodes(), 2);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
