#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
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

}  // namespace
