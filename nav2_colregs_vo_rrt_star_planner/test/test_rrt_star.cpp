#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_colregs_vo_rrt_star_planner/rrt_star.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace
{

using nav2_colregs_vo_rrt_star_planner::RRTStar;
using nav2_colregs_vo_rrt_star_planner::RRTStarNode;

nav2_costmap_2d::Costmap2D makeCostmap()
{
  return nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0.0, 0.0, 0);
}

TEST(VORRTStar, terminates_threshold_path_at_requested_goal)
{
  auto costmap = makeCostmap();
  RRTStar planner(1.0, 2, 1.0, 1.1, 0.0, 0.0, 0, 5.0);
  const std::vector<geometry_msgs::msg::Point> barriers;
  std::vector<RRTStarNode> path;
  constexpr double goal_x = 4.0;
  constexpr double goal_y = 1.0;

  ASSERT_TRUE(planner.planPath(1.0, 1.0, goal_x, goal_y, &costmap, barriers, path));
  ASSERT_FALSE(path.empty());
  EXPECT_DOUBLE_EQ(path.back().x, goal_x);
  EXPECT_DOUBLE_EQ(path.back().y, goal_y);

  planner.prunePath(path, &costmap, barriers);
  ASSERT_FALSE(path.empty());
  EXPECT_DOUBLE_EQ(path.back().x, goal_x);
  EXPECT_DOUBLE_EQ(path.back().y, goal_y);
}

TEST(VORRTStar, terminates_approximate_fallback_at_requested_goal)
{
  auto costmap = makeCostmap();
  RRTStar planner(1.0, 1, 1.0, 0.1, 0.0, 0.0, 0, 5.0);
  const std::vector<geometry_msgs::msg::Point> barriers;
  std::vector<RRTStarNode> path;
  constexpr double goal_x = 4.0;
  constexpr double goal_y = 1.0;

  ASSERT_TRUE(planner.planPath(1.0, 1.0, goal_x, goal_y, &costmap, barriers, path));
  ASSERT_FALSE(path.empty());
  EXPECT_DOUBLE_EQ(path.back().x, goal_x);
  EXPECT_DOUBLE_EQ(path.back().y, goal_y);
}

TEST(VORRTStar, does_not_duplicate_existing_goal_node)
{
  auto costmap = makeCostmap();
  RRTStar planner(5.0, 1, 1.0, 0.1, 0.0, 0.0, 0, 5.0);
  const std::vector<geometry_msgs::msg::Point> barriers;
  std::vector<RRTStarNode> path;
  constexpr double goal_x = 4.0;
  constexpr double goal_y = 1.0;

  ASSERT_TRUE(planner.planPath(1.0, 1.0, goal_x, goal_y, &costmap, barriers, path));
  const auto goal_count = std::count_if(
    path.begin(), path.end(),
    [goal_x, goal_y](const RRTStarNode & node) {
      return node.x == goal_x && node.y == goal_y;
    });
  EXPECT_EQ(goal_count, 1);
}

TEST(VORRTStar, barrier_pruning_retains_connector_and_goal)
{
  auto costmap = makeCostmap();
  RRTStar planner(1.0, 1, 1.0, 0.1, 0.0, 0.0, 0, 5.0);
  geometry_msgs::msg::Point barrier_start;
  barrier_start.x = 2.0;
  barrier_start.y = 0.5;
  geometry_msgs::msg::Point barrier_end;
  barrier_end.x = 2.0;
  barrier_end.y = 2.0;
  const std::vector<geometry_msgs::msg::Point> barriers{barrier_start, barrier_end};
  std::vector<RRTStarNode> path(3);
  path[0].x = 1.0;
  path[0].y = 1.0;
  path[1].x = 1.0;
  path[1].y = 3.0;
  path[2].x = 4.0;
  path[2].y = 3.0;

  planner.prunePath(path, &costmap, barriers);

  ASSERT_EQ(path.size(), 3u);
  EXPECT_DOUBLE_EQ(path[1].x, 1.0);
  EXPECT_DOUBLE_EQ(path[1].y, 3.0);
  EXPECT_DOUBLE_EQ(path.back().x, 4.0);
  EXPECT_DOUBLE_EQ(path.back().y, 3.0);
}

}  // namespace
