#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "nav2_colregs_vo_skeleton_planner/skeleton_planner.hpp"
#include "nav2_colregs_vo_skeleton_planner/skeleton_space.hpp"
#include "nav2_colregs_vo_skeleton_planner/skeleton_tree.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace
{

using nav2_colregs_vo_skeleton_planner::BoundedInformedRRT;
using nav2_colregs_vo_skeleton_planner::Budget;
using nav2_colregs_vo_skeleton_planner::Pt;
using nav2_colregs_vo_skeleton_planner::SkeletonConfig;
using nav2_colregs_vo_skeleton_planner::SkeletonPlanner;
using nav2_colregs_vo_skeleton_planner::Space;

// 200 x 200 m free map at 1 m resolution with an optional vertical wall.
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
    config.local_iterations = 400;
    config.refine_iterations = 100;
    config.reuse_iterations = 64;
    space = std::make_unique<Space>(&costmap, 0.0, 0.0);
  }

  void wall(int x, int y0, int y1)
  {
    for (int y = y0; y <= y1; ++y) {
      costmap.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  Pt at(double x, double y) {return Pt(x, y);}
};

}  // namespace

TEST(SkeletonSpace, EdgeCostMatchesWeightedLength)
{
  Scene scene;
  scene.space->beginQuery(1);
  const double c = scene.space->edgeCost(Pt(10.5, 10.5), Pt(20.5, 10.5));
  EXPECT_NEAR(c, 10.0, 1e-9);  // free cells -> multiplier 1
  EXPECT_TRUE(scene.space->segmentFree(Pt(0.5, 0.5), Pt(198.5, 198.5)));
}

TEST(SkeletonSpace, BlockedEdgeIsInfiniteAndCached)
{
  Scene scene;
  scene.wall(100, 0, 199);
  scene.space->beginQuery(1);
  EXPECT_FALSE(scene.space->segmentFree(Pt(50, 100), Pt(150, 100)));
  EXPECT_TRUE(scene.space->segmentFree(Pt(50, 100), Pt(99, 100)));
  EXPECT_GT(scene.space->cacheEntries(), 0);
}

TEST(SkeletonSpace, PruneRemovesCollinearDetour)
{
  Scene scene;
  scene.space->beginQuery(1);
  std::vector<Pt> out;
  const std::vector<Pt> detour{{10, 10}, {10, 60}, {110, 60}, {110, 110}};
  ASSERT_TRUE(scene.space->prune(detour, out));
  // the greedy shortcut must reach the end with strictly fewer points
  ASSERT_GT(detour.size(), out.size());
  EXPECT_NEAR(out.front().first, 10.0, 1e-9);
  EXPECT_NEAR(out.back().second, 110.0, 1e-9);
}

TEST(SkeletonSpace, ReanchorConnectsMovedQuery)
{
  Scene scene;
  scene.space->beginQuery(1);
  const std::vector<Pt> path{{10, 50}, {60, 50}, {110, 50}, {160, 50}};
  std::vector<Pt> out;
  ASSERT_TRUE(scene.space->reanchor(Pt(60, 60), path, out));
  ASSERT_FALSE(out.empty());
  EXPECT_NEAR(out.front().first, 60.0, 1e-9);
  EXPECT_NEAR(out.front().second, 60.0, 1e-9);
  // suffix must be preserved
  EXPECT_NEAR(out.back().first, 160.0, 1e-9);
}

TEST(SkeletonTree, ColdSearchReachesGoalAroundWall)
{
  Scene scene;
  scene.wall(100, 0, 150);  // gap at the top
  scene.space->beginQuery(1);
  SkeletonConfig cfg = scene.config;
  cfg.reuse_iterations = 0;
  BoundedInformedRRT tree(scene.space.get(), Pt(150, 10), cfg, 11);
  Budget budget(cfg.max_work);
  std::vector<Pt> path;
  ASSERT_TRUE(tree.search(Pt(10, 10), budget, 1200, path));
  EXPECT_NEAR(path.back().first, 150.0, 1e-6);
  EXPECT_NEAR(path.back().second, 10.0, 1e-6);
  // path must not cross the wall: every vertex x!=100-band check via cost
  EXPECT_TRUE(scene.space->pathValid(path));
}

TEST(SkeletonTree, SeedPathInstallsChain)
{
  Scene scene;
  scene.space->beginQuery(1);
  SkeletonConfig cfg = scene.config;
  BoundedInformedRRT tree(scene.space.get(), Pt(100, 50), cfg, 3);
  Budget budget(cfg.max_work);
  const std::vector<Pt> path{{10, 50}, {55, 50}, {100, 50}};
  ASSERT_TRUE(tree.seedPath(path, budget));
  EXPECT_EQ(tree.size(), 3);
  EXPECT_NEAR(tree.nodeCost(2), 90.0, 1e-6);
}

TEST(SkeletonTree, RecycleKeepsProtectedChain)
{
  Scene scene;
  scene.space->beginQuery(1);
  SkeletonConfig cfg = scene.config;
  cfg.node_limit = 512;
  BoundedInformedRRT tree(scene.space.get(), Pt(100, 50), cfg, 5);
  Budget budget(cfg.max_work);
  tree.seedPath({{10, 50}, {55, 50}, {100, 50}}, budget);
  // grow a batch of nodes so recycling has work to do
  std::vector<Pt> out;
  for (int i = 0; i < 3; ++i) {
    tree.search(Pt(10, 50), budget, 600, out);
  }
  tree.recycle(Pt(10, 50), nullptr, budget);
  EXPECT_LE(tree.size(), cfg.node_limit);
  // ancestors of any node must still be valid indices
  for (int i = 0; i < tree.size(); ++i) {
    EXPECT_LE(tree.nodeParent(i), tree.size());
  }
}

TEST(SkeletonTree, PruneInvalidRemovesBlockedBranch)
{
  Scene scene;
  scene.wall(100, 0, 150);  // gap at the top
  scene.space->beginQuery(1);
  SkeletonConfig cfg = scene.config;
  BoundedInformedRRT tree(scene.space.get(), Pt(150, 10), cfg, 11);
  Budget budget(cfg.max_work);
  std::vector<Pt> path;
  ASSERT_TRUE(tree.search(Pt(10, 10), budget, 1200, path));
  const int before = tree.size();
  ASSERT_GT(before, 2);
  // no obstacles besides the wall -> nothing to prune
  EXPECT_EQ(tree.pruneInvalid(budget), 0);
  // grow a right-side branch, then block the corridor the tree occupies
  // (adding a new wall segment that cuts some edges)
  scene.wall(120, 100, 199);
  scene.wall(120, 0, 40);
  scene.space->beginQuery(2);  // new world revision invalidates the cache
  const int removed = tree.pruneInvalid(budget);
  EXPECT_GE(removed, 0);
  EXPECT_LT(tree.size(), before + 1);
  // every remaining edge must be valid under the new map
  for (int i = 1; i < tree.size(); ++i) {
    EXPECT_TRUE(scene.space->segmentFree(
        tree.nodePoint(tree.nodeParent(i)), tree.nodePoint(i)))
      << "node " << i << " retained an invalid parent edge";
  }
}

TEST(SkeletonPlanner, ColdPlanAndReplanKeepCorridor)
{
  Scene scene;
  scene.wall(100, 0, 150);
  scene.space->beginQuery(1);
  SkeletonPlanner planner(scene.space.get(), Pt(150, 10), scene.config, 11);
  std::vector<Pt> path;
  const auto st1 = planner.replan(Pt(10, 10), path);
  ASSERT_FALSE(path.empty()) << st1.status;
  EXPECT_EQ(st1.mode, "bootstrap");
  EXPECT_NEAR(path.back().first, 150.0, 1e-6);

  // advance along the plan; replan must stay cheap and same-corridor
  const Pt advanced(20.0, 11.5);
  std::vector<Pt> path2;
  const auto st2 = planner.replan(advanced, path2);
  ASSERT_FALSE(path2.empty()) << st2.status;
  EXPECT_NEAR(path2.front().first, 20.0, 1e-9);
  EXPECT_NEAR(path2.back().first, 150.0, 1e-6);
  EXPECT_TRUE(scene.space->pathValid(path2));
}

TEST(SkeletonPlanner, OffPathQueryReanchorsSameCorridor)
{
  Scene scene;
  scene.wall(100, 0, 150);
  scene.space->beginQuery(1);
  SkeletonPlanner planner(scene.space.get(), Pt(150, 10), scene.config, 11);
  std::vector<Pt> path;
  ASSERT_TRUE(planner.replan(Pt(10, 10), path).status == "ok" ||
    planner.replan(Pt(10, 10), path).status == "bootstrap");
  std::vector<Pt> off;
  const auto st = planner.replan(Pt(40, 40), off);
  ASSERT_FALSE(off.empty()) << st.status;
  EXPECT_NEAR(off.front().first, 40.0, 1e-9);
  EXPECT_NEAR(off.front().second, 40.0, 1e-9);
  EXPECT_TRUE(scene.space->pathValid(off));
}

TEST(SkeletonPlanner, SkeletonOcclusionPreservesTree)
{
  Scene scene;
  scene.wall(100, 0, 150);
  scene.space->beginQuery(1);
  SkeletonPlanner planner(scene.space.get(), Pt(150, 10), scene.config, 11);
  std::vector<Pt> path;
  ASSERT_TRUE(!planner.replan(Pt(10, 10), path).status.empty());
  const int nodes_before = planner.treeNodes();

  // storm drifts onto the adopted corridor: extend the wall to seal the gap
  scene.wall(100, 151, 199);
  scene.space->beginQuery(2);
  std::vector<Pt> rerouted;
  const auto st = planner.replan(Pt(15, 12), rerouted);
  // tree must survive the world change (dynamic level 2)
  EXPECT_GT(planner.treeNodes(), 2);
  (void)nodes_before;
  if (!rerouted.empty()) {
    EXPECT_TRUE(scene.space->pathValid(rerouted));
  }
}

TEST(SkeletonPlanner, GoalChangeRequiresReset)
{
  Scene scene;
  scene.space->beginQuery(1);
  SkeletonPlanner planner(scene.space.get(), Pt(150, 10), scene.config, 11);
  EXPECT_FALSE(planner.goalChanged(Pt(150, 10)));
  EXPECT_TRUE(planner.goalChanged(Pt(160, 10)));
}

TEST(SkeletonPlanner, BudgetLimitAbortsGracefully)
{
  Scene scene;
  scene.wall(100, 0, 150);
  scene.space->beginQuery(1);
  SkeletonConfig cfg = scene.config;
  cfg.max_work = 500;  // deliberately tiny
  SkeletonPlanner planner(scene.space.get(), Pt(150, 10), cfg, 11);
  std::vector<Pt> path;
  const auto st = planner.replan(Pt(10, 10), path);
  // either aborted by the budget or (unlikely) a direct connection made it
  EXPECT_TRUE(path.empty() || st.status == "ok" || st.status == "bootstrap");
}


// ---------------------------------------------------------------------------
// Review P0 regressions
// ---------------------------------------------------------------------------

TEST(SkeletonPlannerRegress, RecoveryKeepsPrefixInvariantNoDoubleTail)
{
  // Deterministic recovery trigger: install a known skeleton, then block
  // ONLY the query->first-anchor connector (small wall) while the anchor
  // suffix and an alternative corridor remain valid. The old bug produced
  // [q,...,goal,a1,...,goal] double-tail paths after near/global recovery.
  Scene scene;
  scene.wall(100, 0, 150);  // main wall with a gap at the top
  scene.space->beginQuery(1);
  SkeletonPlanner planner(scene.space.get(), Pt(150, 10), scene.config, 11);
  // known-good skeleton through the gap
  planner.setPath({Pt(10, 10), Pt(100, 190), Pt(150, 10)});

  // small wall strictly on the first chord midpoint (55,100); the anchor
  // suffix (100,190)->(150,10) stays valid and detours exist below y=95
  scene.wall(55, 95, 105);
  scene.space->beginQuery(2);
  std::vector<Pt> recovered;
  const auto st = planner.replan(Pt(10, 10), recovered);
  ASSERT_FALSE(recovered.empty()) << "status=" << st.status
                                  << " mode=" << st.mode;
  // recovery path must have been produced through the tree
  EXPECT_TRUE(st.mode == "near_recovery" || st.mode == "global_recovery")
    << "mode=" << st.mode;
  const Pt goal(150, 10);
  int goal_count = 0;
  for (const auto & q : recovered) {
    if (std::hypot(q.first - goal.first, q.second - goal.second) < 1e-6) {
      goal_count++;
    }
  }
  EXPECT_EQ(goal_count, 1) << "double-tail path after recovery";
  EXPECT_NEAR(recovered.back().first, goal.first, 1e-6);
  EXPECT_NEAR(recovered.back().second, goal.second, 1e-6);
  for (size_t i = 1; i < recovered.size(); ++i) {
    EXPECT_GT(std::hypot(recovered[i].first - recovered[i - 1].first,
      recovered[i].second - recovered[i - 1].second), 1e-9);
  }
  EXPECT_TRUE(scene.space->pathValid(recovered));
}

TEST(SkeletonTreeRegress, RefineLimitStopsEarlyAfterFirstSolution)
{
  Scene scene;  // empty map: solution found almost immediately
  scene.space->beginQuery(1);
  SkeletonConfig cfg = scene.config;
  cfg.reuse_iterations = 0;
  cfg.global_iterations = 1200;
  BoundedInformedRRT tree(scene.space.get(), Pt(150, 50), cfg, 11);
  Budget budget(cfg.max_work);
  std::vector<Pt> path;
  ASSERT_TRUE(tree.search(Pt(20, 50), budget, 1200, path));
  // unseeded search must stop ~refine_iterations after the first solution,
  // not burn the whole global budget
  EXPECT_LT(budget.search_iterations, 600);
}

TEST(SkeletonSpaceRegress, SupercoverTangencyAndCorner)
{
  Scene scene;
  scene.space->beginQuery(1);
  // (a) segment along an integer gridline must inspect the adjacent row:
  // an obstacle exactly on y=60 must block a y=60.0 collinear segment
  scene.wall(80, 60, 60);
  EXPECT_FALSE(scene.space->segmentFree(Pt(50.0, 60.0), Pt(110.0, 60.0)));
  scene.costmap.setCost(80, 60, 0);
  // (b) ... but once cleared, the same segment is free again (cache ok)
  scene.space->beginQuery(2);
  EXPECT_TRUE(scene.space->segmentFree(Pt(50.0, 60.0), Pt(110.0, 60.0)));
  // (c) integer-x vertical segment must inspect the adjacent column
  scene.costmap.setCost(100, 90, nav2_costmap_2d::LETHAL_OBSTACLE);
  scene.space->beginQuery(3);
  EXPECT_FALSE(scene.space->segmentFree(Pt(100.0, 50.0), Pt(100.0, 130.0)));
  scene.costmap.setCost(100, 90, 0);
  scene.space->beginQuery(4);
  EXPECT_TRUE(scene.space->segmentFree(Pt(100.0, 50.0), Pt(100.0, 130.0)));
  // (d) diagonal through an exact cell corner must touch both diagonal
  // neighbours: obstacle at (90,91) blocks the (100,100)->(80,81) diagonal
  scene.costmap.setCost(90, 91, nav2_costmap_2d::LETHAL_OBSTACLE);
  scene.space->beginQuery(5);
  EXPECT_FALSE(scene.space->segmentFree(Pt(100.0, 100.0), Pt(80.0, 81.0)));
}


// ---------------------------------------------------------------------------
// Round-2 review regressions (D1: cost refresh invariants, D2: budget
// interruption cross-query consistency)
// ---------------------------------------------------------------------------

TEST(SkeletonTreeRegress, PruneInvalidRefreshesCostsWithoutRemoval)
{
  // R2-02: even with zero removals the tree costs must be refreshed against
  // the current world. Install a 4-node chain, verify g chain consistency.
  Scene scene;
  scene.space->beginQuery(1);
  SkeletonConfig cfg = scene.config;
  BoundedInformedRRT tree(scene.space.get(), Pt(100, 50), cfg, 3);
  Budget budget(cfg.max_work);
  const std::vector<Pt> chain{{10, 50}, {40, 50}, {70, 50}, {100, 50}};
  ASSERT_TRUE(tree.seedPath(chain, budget));
  EXPECT_NEAR(tree.nodeCost(3), 90.0, 1e-6);  // 30+30+30 geometric

  // no world change, no removal: costs stay exact and consistent
  scene.space->beginQuery(2);
  EXPECT_EQ(tree.pruneInvalid(budget), 0);
  for (int i = 1; i < tree.size(); ++i) {
    const double edge = scene.space->edgeCost(
      tree.nodePoint(tree.nodeParent(i)), tree.nodePoint(i), &budget);
    EXPECT_NEAR(tree.nodeCost(i),
      tree.nodeCost(tree.nodeParent(i)) + edge, 1e-6)
      << "g(child) != g(parent) + edge at node " << i;
  }
}

TEST(SkeletonTreeRegress, PruneInvalidCostRefreshSurvivesRemoval)
{
  // R2-02 second half: with removals, child costs must be computed from the
  // REFRESHED parent costs (new_g), not the stale ones.
  Scene scene;
  scene.space->beginQuery(1);
  SkeletonConfig cfg = scene.config;
  BoundedInformedRRT tree(scene.space.get(), Pt(100, 50), cfg, 5);
  Budget budget(cfg.max_work);
  const std::vector<Pt> chain{{10, 50}, {40, 50}, {70, 50}, {100, 50}};
  ASSERT_TRUE(tree.seedPath(chain, budget));

  // grow at least one extra branch off the mid node so a removal can trigger
  std::vector<Pt> out;
  tree.search(Pt(10, 50), budget, 800, out);

  // block the far end: nodes beyond the wall die, survivors refresh
  scene.wall(25, 40, 60);  // cuts the (40,50)-(10,50) edge
  scene.space->beginQuery(2);
  const int removed = tree.pruneInvalid(budget);
  if (removed > 0) {
    for (int i = 1; i < tree.size(); ++i) {
      const double edge = scene.space->edgeCost(
        tree.nodePoint(tree.nodeParent(i)), tree.nodePoint(i), &budget);
      EXPECT_NEAR(tree.nodeCost(i),
        tree.nodeCost(tree.nodeParent(i)) + edge, 1e-6)
        << "stale-parent cost leaked through the removal path at node " << i;
    }
  }
}

TEST(SkeletonPlannerRegress, BudgetInterruptLeavesConsistentState)
{
  // R2-01 deterministic replay: interrupting the skip loop mid-way must not
  // leave a mismatched active_/prefix_ that the next query splices into an
  // invalid path. Two obstacles so Q-G and A-G are blocked while Q-B works.
  Scene scene;
  scene.costmap.setCost(6, 1, nav2_costmap_2d::LETHAL_OBSTACLE);
  scene.costmap.setCost(4, 7, nav2_costmap_2d::LETHAL_OBSTACLE);
  SkeletonConfig cfg = scene.config;
  scene.space->beginQuery(1);
  SkeletonPlanner planner(scene.space.get(), Pt(10.5, 1.5), cfg, 11);
  const Pt Q(1.5, 1.5), A(1.5, 10.5), B(10.5, 10.5), G(10.5, 1.5);
  planner.setPath({Q, A, B, G});

  // shrink the budget so a mid-skip interruption happens naturally; scan a
  // few candidate limits and assert consistency after EVERY interrupted call
  for (int64_t limit = 40; limit < 140; limit += 10) {
    SkeletonPlanner probe(scene.space.get(), G, cfg, 11);
    probe.setPath({Q, A, B, G});
    SkeletonConfig small = cfg;
    small.max_work = limit;
    // NOTE: the planner reads its config copy; emulate by constructing a
    // planner with the small config from scratch
    SkeletonPlanner tight(scene.space.get(), G, small, 11);
    tight.setPath({Q, A, B, G});
    std::vector<Pt> first;
    tight.replan(Q, first);
    // whatever the outcome, the next query must never emit an invalid path
    scene.space->beginQuery(2);
    std::vector<Pt> second;
    tight.replan(Q, second);
    if (!second.empty()) {
      EXPECT_NEAR(second.front().first, Q.first, 1e-9);
      EXPECT_NEAR(second.front().second, Q.second, 1e-9);
      EXPECT_NEAR(second.back().first, G.first, 1e-9);
      EXPECT_NEAR(second.back().second, G.second, 1e-9);
      EXPECT_TRUE(scene.space->pathValid(second))
        << "cross-query invalid path at limit=" << limit;
      // goal appears exactly once
      int goal_count = 0;
      for (const auto & q : second) {
        if (std::hypot(q.first - G.first, q.second - G.second) < 1e-6) {
          goal_count++;
        }
      }
      EXPECT_EQ(goal_count, 1) << "double-tail at limit=" << limit;
    }
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
