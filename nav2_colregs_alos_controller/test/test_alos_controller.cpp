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

#include <cmath>
#include <initializer_list>
#include <limits>
#include <string>
#include <utility>

#include "gtest/gtest.h"
#include "nav2_colregs_alos_controller/alos_controller.hpp"
#include "nav2_core/controller_exceptions.hpp"

namespace nav2_colregs_alos_controller
{

class TestableALOSController : public ALOSController
{
public:
  using ALOSController::findClosestPointIndex;
  using ALOSController::findForwardPoint;

  bool isNewGoal(const nav_msgs::msg::Path & path)
  {
    return updateGoalAndCheckIfNew(path);
  }

  void setGoalTolerance(double tolerance)
  {
    beta_reset_goal_dist_tolerance_ = tolerance;
  }
};

nav_msgs::msg::Path makePath(std::initializer_list<std::pair<double, double>> points)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "base_link";
  for (const auto & [x, y] : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

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

TEST(ALOSControllerPathHelpers, rejectsEmptyAndInvalidInputs)
{
  TestableALOSController controller;
  nav_msgs::msg::Path empty;
  const auto path = makePath({{0.0, 0.0}, {1.0, 0.0}});

  EXPECT_THROW(controller.findClosestPointIndex(empty), nav2_core::InvalidPath);
  EXPECT_THROW(controller.findForwardPoint(empty, 0, 1.0), nav2_core::InvalidPath);
  EXPECT_THROW(controller.findForwardPoint(path, path.poses.size(), 1.0), nav2_core::InvalidPath);
  EXPECT_THROW(controller.findForwardPoint(path, 0, 0.0), nav2_core::InvalidPath);
  EXPECT_THROW(
    controller.findForwardPoint(path, 0, std::numeric_limits<double>::infinity()),
    nav2_core::InvalidPath);
}

TEST(ALOSControllerPathHelpers, interpolatesSparsePathAtForwardDistance)
{
  TestableALOSController controller;
  const auto path = makePath({{0.0, 0.0}, {10.0, 0.0}});

  const auto point = controller.findForwardPoint(path, 0, 2.0);

  EXPECT_DOUBLE_EQ(point.x, 2.0);
  EXPECT_DOUBLE_EQ(point.y, 0.0);
}

TEST(ALOSControllerPathHelpers, skipsDuplicateSegments)
{
  TestableALOSController controller;
  const auto path = makePath({{0.0, 0.0}, {0.0, 0.0}, {4.0, 0.0}});

  const auto point = controller.findForwardPoint(path, 0, 2.0);

  EXPECT_DOUBLE_EQ(point.x, 2.0);
  EXPECT_DOUBLE_EQ(point.y, 0.0);
}

TEST(ALOSControllerPathHelpers, returnsEndpointForZeroLengthGoalPath)
{
  TestableALOSController controller;
  const auto path = makePath({{0.0, 0.0}, {0.0, 0.0}});

  const auto point = controller.findForwardPoint(path, 0, 2.0);

  EXPECT_DOUBLE_EQ(point.x, 0.0);
  EXPECT_DOUBLE_EQ(point.y, 0.0);
}

TEST(ALOSControllerPathHelpers, rejectsNonFinitePathCoordinates)
{
  TestableALOSController controller;
  const auto path = makePath(
    {
      {0.0, 0.0}, {std::numeric_limits<double>::quiet_NaN(), 1.0}});

  EXPECT_THROW(controller.findClosestPointIndex(path), nav2_core::InvalidPath);
  EXPECT_THROW(controller.findForwardPoint(path, 0, 1.0), nav2_core::InvalidPath);
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
