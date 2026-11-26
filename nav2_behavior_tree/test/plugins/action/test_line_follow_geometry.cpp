#include <cmath>

#include "gtest/gtest.h"
#include "nav2_behavior_tree/plugins/action/line_follow_geometry.hpp"

using nav2_behavior_tree::LinePoint;
using nav2_behavior_tree::buildBypass;
using nav2_behavior_tree::makePolyline;
using nav2_behavior_tree::selectRejoinAlong;
using nav2_behavior_tree::projectSegment;
using nav2_behavior_tree::cornerNearby;
using nav2_behavior_tree::cornerPassed;
using nav2_behavior_tree::projectWindow;
using nav2_behavior_tree::sliceRelaxed;

TEST(LineFollowGeometry, ProjectsOntoTheCurrentLeg)
{
  const auto line = makePolyline({
    {0.0, 0.0}, {200.0, 0.0}, {200.0, 40.0}, {0.0, 40.0}});
  const auto progress = projectWindow(line, 40.0, 3.0, 0.0, 20.0, 30.0);
  ASSERT_TRUE(progress.valid);
  EXPECT_EQ(progress.segment, 0);
  EXPECT_NEAR(progress.along, 40.0, 1e-6);
  EXPECT_NEAR(progress.cross, 3.0, 1e-6);
  EXPECT_GT(progress.remaining, 300.0);
}

TEST(LineFollowGeometry, LockedSegmentIgnoresTheNextLane)
{
  const auto line = makePolyline({
    {0.0, 0.0}, {200.0, 0.0}, {200.0, 40.0}, {0.0, 40.0}});
  const auto progress = projectSegment(line, 0, 80.0, 25.0);
  EXPECT_EQ(progress.segment, 0);
  EXPECT_NEAR(progress.along, 80.0, 1e-6);
  EXPECT_NEAR(progress.cross, 25.0, 1e-6);
}

TEST(LineFollowGeometry, BypassStaysLeftAndRejoinsTheSameLine)
{
  const auto line = makePolyline({{0.0, 0.0}, {200.0, 0.0}});
  auto progress = projectSegment(line, 0, 70.0, 0.0);
  const auto bypass = buildBypass(line, progress, 150.0, 25.0, 1.0, 10.0);
  ASSERT_GE(bypass.size(), 4u);
  EXPECT_NEAR(bypass[1].y, 25.0, 1e-6);
  bool rejoined = false;
  for (const auto & point : bypass) {
    if (std::hypot(point.x - 150.0, point.y) < 1.0) {
      rejoined = true;
    }
  }
  EXPECT_TRUE(rejoined);
  EXPECT_NEAR(bypass.back().x, 200.0, 1e-6);
  EXPECT_NEAR(bypass.back().y, 0.0, 1e-6);
}

TEST(LineFollowGeometry, CornerVertexIsNotARequiredWaypoint)
{
  const auto line = makePolyline({
    {0.0, 0.0}, {200.0, 0.0}, {200.0, 40.0}, {0.0, 40.0}});
  const auto path = sliceRelaxed(line, 170.0, 5.0, 20.0);
  ASSERT_GE(path.size(), 2u);
  for (const auto & point : path) {
    EXPECT_GT(std::hypot(point.x - 200.0, point.y), 15.0);
  }
  const auto on_next_leg = projectWindow(line, 205.0, 18.0, 190.0, 20.0, 40.0);
  EXPECT_TRUE(cornerPassed(line, 0, on_next_leg, 205.0, 18.0, 35.0));
  const auto still_on_leg = projectWindow(line, 150.0, 2.0, 140.0, 20.0, 40.0);
  EXPECT_FALSE(cornerPassed(line, 0, still_on_leg, 150.0, 2.0, 35.0));
}

TEST(LineFollowGeometry, CurveModeOnlyNearACorner)
{
  const auto line = makePolyline({
    {0.0, 0.0}, {200.0, 0.0}, {200.0, 40.0}, {0.0, 40.0}});
  EXPECT_TRUE(cornerNearby(line, 175.0, 30.0, 12.0));
  EXPECT_FALSE(cornerNearby(line, 80.0, 30.0, 12.0));
  EXPECT_TRUE(cornerNearby(line, 210.0, 30.0, 12.0));
  EXPECT_FALSE(cornerNearby(line, 220.0, 5.0, 5.0));
}

TEST(LineFollowGeometry, RejoinPassesTheFarthestPerceivedBlock)
{
  const std::vector<int> blocked{4, 6, 5};
  const double rejoin = selectRejoinAlong(60.0, 400.0, blocked, 5.0, 25.0, 0.0);
  EXPECT_NEAR(rejoin, 60.0 + 6.0 * 5.0 + 25.0, 1e-6);
}

TEST(LineFollowGeometry, UnreachableRejoinSlidesAndStaysOnTheLine)
{
  const std::vector<int> blocked{2};
  const double slid = selectRejoinAlong(180.0, 200.0, blocked, 5.0, 25.0, 40.0);
  EXPECT_NEAR(slid, 199.0, 1e-6);
  const double plain = selectRejoinAlong(10.0, 200.0, std::vector<int>{}, 5.0, 25.0, 0.0);
  EXPECT_NEAR(plain, 35.0, 1e-6);
}
