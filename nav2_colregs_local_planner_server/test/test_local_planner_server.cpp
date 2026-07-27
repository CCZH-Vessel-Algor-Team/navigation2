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

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_colregs_local_planner_server/local_planner_server.hpp"
#include "nav2_colregs_msgs/action/compute_local_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

namespace nav2_colregs_local_planner_server
{

constexpr char kTestNamespace[] = "/local_planner_server_test";

class LocalPlannerServerTest : public ::testing::Test
{
protected:
  using Action = nav2_colregs_msgs::action::ComputeLocalPath;
  using ClientGoalHandle = rclcpp_action::ClientGoalHandle<Action>;

  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    auto server_options = rclcpp::NodeOptions();
    server_options.arguments({"--ros-args", "-r", "__ns:=" + std::string(kTestNamespace)});
    server_ = std::make_shared<ColregsLocalPlannerServer>(server_options);
    client_node_ = std::make_shared<rclcpp::Node>(
      "local_planner_server_test_client", kTestNamespace);
    client_ = rclcpp_action::create_client<Action>(client_node_, "compute_local_path");
    path_subscription_ = client_node_->create_subscription<nav_msgs::msg::Path>(
      "local_path", 1,
      [this](const nav_msgs::msg::Path::SharedPtr path) {published_path_ = path;});
  }

  void TearDown() override
  {
    if (server_) {
      const auto state = server_->get_current_state().id();
      if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        server_->deactivate();
      }
      if (server_->get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      {
        server_->cleanup();
      }
      server_->shutdown();
    }
    client_.reset();
    path_subscription_.reset();
    client_node_.reset();
    server_.reset();
  }

  void configure()
  {
    ASSERT_EQ(
      server_->configure().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_TRUE(client_->wait_for_action_server(1s));
  }

  void activate()
  {
    configure();
    ASSERT_EQ(
      server_->activate().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  ClientGoalHandle::SharedPtr sendGoal(const Action::Goal & goal)
  {
    auto future = client_->async_send_goal(goal);
    const auto status = rclcpp::spin_until_future_complete(client_node_, future, 2s);
    EXPECT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      return nullptr;
    }
    return future.get();
  }

  ClientGoalHandle::WrappedResult getResult(const ClientGoalHandle::SharedPtr & goal_handle)
  {
    auto future = client_->async_get_result(goal_handle);
    const auto status = rclcpp::spin_until_future_complete(client_node_, future, 2s);
    EXPECT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      return ClientGoalHandle::WrappedResult();
    }
    return future.get();
  }

  bool waitForPublishedPath()
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!published_path_ && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(client_node_);
      std::this_thread::sleep_for(10ms);
    }
    return published_path_ != nullptr;
  }

  static Action::Goal makeGoal()
  {
    Action::Goal goal;
    goal.reference_path.header.frame_id = "map";
    goal.reference_path.header.stamp.sec = 7;
    goal.reference_path.header.stamp.nanosec = 8;

    geometry_msgs::msg::PoseStamped first;
    first.header.frame_id = "map";
    first.pose.position.x = 1.25;
    first.pose.position.y = -2.5;
    first.pose.position.z = 0.1;
    first.pose.orientation.w = 1.0;

    geometry_msgs::msg::PoseStamped second;
    second.header.frame_id = "map";
    second.pose.position.x = -4.5;
    second.pose.position.y = 6.75;
    second.pose.position.z = 0.2;
    second.pose.orientation.z = 0.5;
    second.pose.orientation.w = 0.5;

    goal.reference_path.poses = {first, second};
    return goal;
  }

  std::shared_ptr<ColregsLocalPlannerServer> server_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  nav_msgs::msg::Path::SharedPtr published_path_;
};

TEST_F(LocalPlannerServerTest, actionAbsentBeforeConfigure)
{
  EXPECT_FALSE(client_->wait_for_action_server(200ms));
}

TEST_F(LocalPlannerServerTest, rejectsGoalWhileInactive)
{
  configure();
  EXPECT_EQ(sendGoal(makeGoal()), nullptr);
}

TEST_F(LocalPlannerServerTest, returnsActionResultPathAfterActivation)
{
  activate();
  const auto goal = makeGoal();
  const auto goal_handle = sendGoal(goal);
  ASSERT_NE(goal_handle, nullptr);

  const auto wrapped_result = getResult(goal_handle);
  ASSERT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_EQ(wrapped_result.result->error_code, Action::Result::NONE);
  EXPECT_TRUE(wrapped_result.result->error_msg.empty());
  EXPECT_EQ(wrapped_result.result->local_path.header.frame_id, "map");
  ASSERT_EQ(wrapped_result.result->local_path.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(wrapped_result.result->local_path.poses[0].pose.position.x, 1.25);
  EXPECT_DOUBLE_EQ(wrapped_result.result->local_path.poses[0].pose.position.y, -2.5);
  EXPECT_DOUBLE_EQ(wrapped_result.result->local_path.poses[0].pose.position.z, 0.1);
  EXPECT_DOUBLE_EQ(wrapped_result.result->local_path.poses[1].pose.position.x, -4.5);
  EXPECT_DOUBLE_EQ(wrapped_result.result->local_path.poses[1].pose.position.y, 6.75);
  EXPECT_DOUBLE_EQ(wrapped_result.result->local_path.poses[1].pose.position.z, 0.2);
  EXPECT_LT(wrapped_result.result->planning_time.nanosec, 1000000000u);
  ASSERT_TRUE(waitForPublishedPath());
  EXPECT_EQ(*published_path_, wrapped_result.result->local_path);
}

TEST_F(LocalPlannerServerTest, rejectsEmptyReferencePath)
{
  activate();
  auto goal = makeGoal();
  goal.reference_path.poses.clear();
  const auto goal_handle = sendGoal(goal);
  ASSERT_NE(goal_handle, nullptr);

  const auto wrapped_result = getResult(goal_handle);
  ASSERT_EQ(wrapped_result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_EQ(wrapped_result.result->error_code, Action::Result::EMPTY_PATH);
  EXPECT_EQ(wrapped_result.result->error_msg, "Reference path is empty");
}

TEST_F(LocalPlannerServerTest, preservesFrameAndPoses)
{
  activate();
  const auto goal = makeGoal();
  const auto goal_handle = sendGoal(goal);
  ASSERT_NE(goal_handle, nullptr);

  const auto wrapped_result = getResult(goal_handle);
  ASSERT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(wrapped_result.result, nullptr);
  const auto & path = wrapped_result.result->local_path;
  EXPECT_EQ(path.header.frame_id, goal.reference_path.header.frame_id);
  EXPECT_NE(path.header.stamp, goal.reference_path.header.stamp);
  ASSERT_EQ(path.poses.size(), goal.reference_path.poses.size());
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    EXPECT_EQ(path.poses[i], goal.reference_path.poses[i]);
  }
}

TEST_F(LocalPlannerServerTest, rejectsEmptyReferenceFrame)
{
  activate();
  auto goal = makeGoal();
  goal.reference_path.header.frame_id.clear();
  const auto goal_handle = sendGoal(goal);
  ASSERT_NE(goal_handle, nullptr);

  const auto wrapped_result = getResult(goal_handle);
  ASSERT_EQ(wrapped_result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_EQ(wrapped_result.result->error_code, Action::Result::INVALID_PATH);
  EXPECT_EQ(wrapped_result.result->error_msg, "Reference path frame_id is empty");
}

TEST_F(LocalPlannerServerTest, acceptedCancellationNeverSucceeds)
{
  activate();
  const auto goal_handle = sendGoal(makeGoal());
  ASSERT_NE(goal_handle, nullptr);

  auto cancel_future = client_->async_cancel_goal(goal_handle);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(client_node_, cancel_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  const bool cancellation_accepted = !cancel_future.get()->goals_canceling.empty();
  const auto wrapped_result = getResult(goal_handle);
  ASSERT_NE(wrapped_result.result, nullptr);

  if (cancellation_accepted) {
    EXPECT_NE(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_TRUE(
      wrapped_result.code == rclcpp_action::ResultCode::CANCELED ||
      wrapped_result.code == rclcpp_action::ResultCode::ABORTED);
    if (wrapped_result.code == rclcpp_action::ResultCode::CANCELED) {
      EXPECT_EQ(wrapped_result.result->error_code, Action::Result::CANCELED);
      EXPECT_EQ(wrapped_result.result->error_msg, "Local path computation canceled");
    }
  } else {
    EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_EQ(wrapped_result.result->error_code, Action::Result::NONE);
  }
}

TEST_F(LocalPlannerServerTest, rejectsGoalAfterDeactivation)
{
  activate();
  ASSERT_EQ(
    server_->deactivate().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(sendGoal(makeGoal()), nullptr);
}

}  // namespace nav2_colregs_local_planner_server
