#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "nav2_colregs_alos_controller/alos_controller.hpp"

namespace nav2_colregs_alos_controller
{

class TestableALOSController : public ALOSController
{
public:
  bool isNewGoal(const nav_msgs::msg::Path & path)
  {
    return updateGoalAndCheckIfNew(path);
  }

  void setGoalTolerance(double tolerance)
  {
    beta_reset_goal_dist_tolerance_ = tolerance;
  }
};

nav_msgs::msg::Path makePath(
  const std::string & frame, double goal_x, double goal_y, double goal_yaw = 0.0)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  path.poses.resize(2);
  path.poses.back().pose.position.x = goal_x;
  path.poses.back().pose.position.y = goal_y;
  path.poses.back().pose.orientation.z = std::sin(goal_yaw / 2.0);
  path.poses.back().pose.orientation.w = std::cos(goal_yaw / 2.0);
  return path;
}

TEST(ALOSControllerGoalReset, FirstPlanRecordsGoalWithoutReset)
{
  TestableALOSController controller;

  EXPECT_FALSE(controller.isNewGoal(makePath("map", 10.0, 2.0)));
  EXPECT_FALSE(controller.isNewGoal(makePath("map", 10.0, 2.0)));
}

TEST(ALOSControllerGoalReset, IgnoresTimestampAndOrientationChanges)
{
  TestableALOSController controller;
  auto first_path = makePath("map", 10.0, 2.0);
  controller.isNewGoal(first_path);
  auto replan = makePath("map", 10.0, 2.0, 1.5);
  replan.header.stamp.sec = first_path.header.stamp.sec + 1;

  EXPECT_FALSE(controller.isNewGoal(replan));
}

TEST(ALOSControllerGoalReset, UsesPlanarDistanceTolerance)
{
  TestableALOSController controller;
  controller.setGoalTolerance(0.05);
  controller.isNewGoal(makePath("map", 10.0, 2.0));

  EXPECT_FALSE(controller.isNewGoal(makePath("map", 10.03, 2.04)));
  EXPECT_TRUE(controller.isNewGoal(makePath("map", 10.031, 2.04)));
}

TEST(ALOSControllerGoalReset, FrameChangeIsNewGoal)
{
  TestableALOSController controller;
  controller.isNewGoal(makePath("map", 10.0, 2.0));

  EXPECT_TRUE(controller.isNewGoal(makePath("odom", 10.0, 2.0)));
}

TEST(ALOSControllerGoalReset, EmptyPlanDoesNotChangeRecordedGoal)
{
  TestableALOSController controller;
  controller.isNewGoal(makePath("map", 10.0, 2.0));

  EXPECT_FALSE(controller.isNewGoal(nav_msgs::msg::Path{}));
  EXPECT_FALSE(controller.isNewGoal(makePath("map", 10.0, 2.0)));
}

}  // namespace nav2_colregs_alos_controller
