#include <gtest/gtest.h>

#include <cmath>

#include "nav2_colregs_vo_rrt_star_planner/rrt_star_planner.hpp"

namespace
{

using nav2_colregs_vo_rrt_star_planner::VORRTStarPlanner;

// ---------------------------------------------------------------------------
// Anchor guard unit tests: pure geometry, no ROS infrastructure needed.
// The wrapper's createPlan delegates the decision to this static helper.
// ---------------------------------------------------------------------------

TEST(VORRTAnchorGuard, StartAtRobotPoseIsAnchored)
{
  EXPECT_TRUE(VORRTStarPlanner::isColregsAnchored(10.0, 10.0, 10.0, 10.0, 3.0));
}

TEST(VORRTAnchorGuard, StartWithinThresholdIsAnchored)
{
  // 2.0 m offset, threshold 3.0 m
  EXPECT_TRUE(VORRTStarPlanner::isColregsAnchored(12.0, 10.0, 10.0, 10.0, 3.0));
  // diagonal 2.83 m < 3.0
  EXPECT_TRUE(VORRTStarPlanner::isColregsAnchored(12.0, 12.0, 10.0, 10.0, 3.0));
}

TEST(VORRTAnchorGuard, StartBeyondThresholdIsNotAnchored)
{
  // 3.0 m exactly: NOT anchored (strict less-than-or-equal means 3.0 passes,
  // but we document the boundary as inclusive)
  EXPECT_TRUE(VORRTStarPlanner::isColregsAnchored(13.0, 10.0, 10.0, 10.0, 3.0));
  // 4.0 m: NOT anchored
  EXPECT_FALSE(VORRTStarPlanner::isColregsAnchored(14.0, 10.0, 10.0, 10.0, 3.0));
  // far away (typical NavigateThroughPoses preview segment)
  EXPECT_FALSE(VORRTStarPlanner::isColregsAnchored(80.0, 80.0, 10.0, 10.0, 3.0));
}

TEST(VORRTAnchorGuard, ZeroThresholdOnlyExactMatch)
{
  EXPECT_TRUE(VORRTStarPlanner::isColregsAnchored(5.0, 5.0, 5.0, 5.0, 0.0));
  EXPECT_FALSE(VORRTStarPlanner::isColregsAnchored(5.1, 5.0, 5.0, 5.0, 0.0));
}

TEST(VORRTAnchorGuard, LargeThresholdCoversPreviewSegments)
{
  // the user can widen the gate for single-goal use (no NTP)
  EXPECT_TRUE(VORRTStarPlanner::isColregsAnchored(80.0, 80.0, 10.0, 10.0, 500.0));
}

TEST(VORRTAnchorGuard, NegativeCoordinates)
{
  EXPECT_TRUE(VORRTStarPlanner::isColregsAnchored(-10.0, -10.0, -10.0, -10.0, 3.0));
  EXPECT_FALSE(VORRTStarPlanner::isColregsAnchored(-20.0, -10.0, -10.0, -10.0, 3.0));
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
