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
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_colregs_local_planner_server/local_planner_server.hpp"
#include "nav2_colregs_ts_manager/ts_core.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

namespace nav2_colregs_local_planner_server
{

class TestPlannerServer : public ColregsLocalPlannerServer
{
public:
  explicit TestPlannerServer(const rclcpp::NodeOptions & options)
  : ColregsLocalPlannerServer(options) {}

  nav2_costmap_2d::Costmap2D * costmap() {return costmap_;}
  std::string costmapName() const {return costmap_ros_->getName();}
  rclcpp_lifecycle::State tsStateNodeState() const
  {
    return ts_state_ros_->get_current_state();
  }
  const nav2_colregs_ts_manager::TsCoreParams & tsCoreParams() const
  {
    return ts_state_ros_->coreParams();
  }
  void setCurrent(bool current) {current_ = current;}
  void setRobotPoseAvailable(bool available) {robot_pose_available_ = available;}
  void setTransformAvailable(bool available) {transform_available_ = available;}
  void setInjectedTsInput(
    const nav2_colregs_ts_manager::ColregsTsStateROS::PlanningInput & input)
  {
    injected_ts_input_ = input;
    use_injected_ts_input_ = true;
  }
  void blockNextTransform()
  {
    std::lock_guard<std::mutex> lock(transform_mutex_);
    block_transform_ = true;
    transform_entered_ = false;
  }
  bool waitForBlockedTransform()
  {
    std::unique_lock<std::mutex> lock(transform_mutex_);
    return transform_condition_.wait_for(lock, 1s, [this]() {return transform_entered_;});
  }
  void releaseTransform()
  {
    std::lock_guard<std::mutex> lock(transform_mutex_);
    block_transform_ = false;
    transform_condition_.notify_all();
  }
  int transformCallCount() const
  {
    std::lock_guard<std::mutex> lock(transform_mutex_);
    return transform_call_count_;
  }

protected:
  bool isCostmapCurrent() const override {return current_;}

  nav2_colregs_ts_manager::ColregsTsStateROS::PlanningInput getTsPlanningInput(
    double os_x, double os_y) override
  {
    if (use_injected_ts_input_) {
      return injected_ts_input_;
    }
    return ColregsLocalPlannerServer::getTsPlanningInput(os_x, os_y);
  }

  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose) const override
  {
    if (!robot_pose_available_) {
      return false;
    }
    pose.header.frame_id = "map";
    pose.pose.position.x = 1.0;
    pose.pose.position.y = 1.0;
    pose.pose.orientation.w = 1.0;
    return true;
  }

  bool transformPoseToGlobalFrame(
    const geometry_msgs::msg::PoseStamped & input,
    geometry_msgs::msg::PoseStamped & output) const override
  {
    if (!transform_available_) {
      return false;
    }
    {
      std::unique_lock<std::mutex> lock(transform_mutex_);
      ++transform_call_count_;
      if (block_transform_) {
        transform_entered_ = true;
        transform_condition_.notify_all();
        transform_condition_.wait(lock, [this]() {return !block_transform_;});
      }
    }
    output = input;
    if (input.header.frame_id == "nan_transform") {
      output.pose.position.x = std::numeric_limits<double>::quiet_NaN();
    } else if (input.header.frame_id == "inf_transform") {
      output.pose.position.y = std::numeric_limits<double>::infinity();
    }
    if (input.header.frame_id == "translated") {
      output.pose.position.x += 1.0;
      output.pose.position.y += 2.0;
    }
    output.header.frame_id = "map";
    return true;
  }

private:
  bool current_{true};
  bool robot_pose_available_{true};
  bool transform_available_{true};
  bool use_injected_ts_input_{false};
  nav2_colregs_ts_manager::ColregsTsStateROS::PlanningInput injected_ts_input_;
  mutable std::mutex transform_mutex_;
  mutable std::condition_variable transform_condition_;
  mutable bool block_transform_{false};
  mutable bool transform_entered_{false};
  mutable int transform_call_count_{0};
};

class LocalPlannerServerTest : public ::testing::Test
{
protected:
  using Action = nav2_msgs::action::ComputePathToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

  static void SetUpTestSuite()
  {
    char executable[] = "test";
    char ros_args[] = "--ros-args";
    char params_flag[] = "--params-file";
    char params_file[] = TEST_PARAMS_FILE;
    char * argv[] = {executable, ros_args, params_flag, params_file};
    rclcpp::init(4, argv);
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=/colregs_planner_test"});
    server_ = std::make_shared<TestPlannerServer>(options);
    client_node_ = std::make_shared<rclcpp::Node>("client", "/colregs_planner_test");
    client_ = rclcpp_action::create_client<Action>(client_node_, "compute_path_to_pose");
    plan_subscription_ = client_node_->create_subscription<nav_msgs::msg::Path>(
      "plan", 1, [this](nav_msgs::msg::Path::SharedPtr message) {published_plan_ = message;});
  }

  void TearDown() override
  {
    if (server_) {
      if (server_->get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
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
    client_node_.reset();
    server_.reset();
  }

  void configure()
  {
    ASSERT_EQ(
      server_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_TRUE(client_->wait_for_action_server(1s));
  }

  void activate()
  {
    configure();
    ASSERT_EQ(server_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  static Action::Goal makeGoal(double start_x = 1.0, double goal_x = 8.0)
  {
    Action::Goal goal;
    goal.use_start = true;
    goal.planner_id = "RRTStar";
    goal.start.header.frame_id = "map";
    goal.start.pose.position.x = start_x;
    goal.start.pose.position.y = 1.0;
    goal.start.pose.orientation.w = 1.0;
    goal.goal.header.frame_id = "map";
    goal.goal.pose.position.x = goal_x;
    goal.goal.pose.position.y = 1.0;
    goal.goal.pose.orientation.z = 0.25;
    goal.goal.pose.orientation.w = 0.9682458365518543;
    return goal;
  }

  GoalHandle::WrappedResult runGoal(const Action::Goal & goal)
  {
    auto goal_future = client_->async_send_goal(goal);
    const auto goal_status = rclcpp::spin_until_future_complete(client_node_, goal_future, 2s);
    EXPECT_EQ(goal_status, rclcpp::FutureReturnCode::SUCCESS);
    if (goal_status != rclcpp::FutureReturnCode::SUCCESS) {
      return {};
    }
    auto handle = goal_future.get();
    EXPECT_NE(handle, nullptr);
    if (!handle) {
      return {};
    }
    auto result_future = client_->async_get_result(handle);
    const auto result_status = rclcpp::spin_until_future_complete(
      client_node_, result_future, 3s);
    EXPECT_EQ(result_status, rclcpp::FutureReturnCode::SUCCESS);
    if (result_status != rclcpp::FutureReturnCode::SUCCESS) {
      return {};
    }
    return result_future.get();
  }

  GoalHandle::WrappedResult runResult(const GoalHandle::SharedPtr & handle)
  {
    auto result_future = client_->async_get_result(handle);
    const auto result_status = rclcpp::spin_until_future_complete(
      client_node_, result_future, 3s);
    EXPECT_EQ(result_status, rclcpp::FutureReturnCode::SUCCESS);
    if (result_status != rclcpp::FutureReturnCode::SUCCESS) {
      return {};
    }
    return result_future.get();
  }

  GoalHandle::SharedPtr sendGoal(const Action::Goal & goal)
  {
    auto goal_future = client_->async_send_goal(goal);
    EXPECT_EQ(
      rclcpp::spin_until_future_complete(client_node_, goal_future, 2s),
      rclcpp::FutureReturnCode::SUCCESS);
    return goal_future.get();
  }

  void expectError(const Action::Goal & goal)
  {
    const auto result = runGoal(goal);
    ASSERT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
    ASSERT_NE(result.result, nullptr);
  }

  void occupy(double x, double y)
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *server_->costmap()->getMutex());
    unsigned int mx;
    unsigned int my;
    ASSERT_TRUE(server_->costmap()->worldToMap(x, y, mx, my));
    server_->costmap()->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  void addSolidWall()
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *server_->costmap()->getMutex());
    for (unsigned int y = 0; y < server_->costmap()->getSizeInCellsY(); ++y) {
      server_->costmap()->setCost(50, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  std::shared_ptr<TestPlannerServer> server_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_subscription_;
  nav_msgs::msg::Path::SharedPtr published_plan_;
};

TEST_F(LocalPlannerServerTest, lifecycleOwnsActionAndCostmap)
{
  EXPECT_FALSE(client_->wait_for_action_server(100ms));
  configure();
  EXPECT_EQ(server_->costmapName(), "colregs_costmap");
  EXPECT_EQ(server_->costmap()->getSizeInCellsX(), 100u);
  ASSERT_EQ(server_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(server_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(server_->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(server_->costmap(), nullptr);
}

TEST_F(LocalPlannerServerTest, lifecycleOrchestratesTsStateSubNode)
{
  EXPECT_EQ(
    server_->tsStateNodeState().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  configure();
  EXPECT_EQ(
    server_->tsStateNodeState().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(server_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(
    server_->tsStateNodeState().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(server_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(
    server_->tsStateNodeState().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(server_->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(
    server_->tsStateNodeState().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LocalPlannerServerTest, acceptsEmptyAndRrtStarPlannerIds)
{
  activate();
  auto empty_id = makeGoal();
  empty_id.planner_id.clear();
  EXPECT_EQ(runGoal(empty_id).code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(runGoal(makeGoal()).code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(LocalPlannerServerTest, rejectsUnknownPlannerId)
{
  activate();
  auto goal = makeGoal();
  goal.planner_id = "GridBased";
  expectError(goal);
}

TEST_F(LocalPlannerServerTest, usesRobotPoseWhenUseStartIsFalse)
{
  activate();
  auto goal = makeGoal(7.0, 2.0);
  goal.use_start = false;
  const auto result = runGoal(goal);
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(result.result, nullptr);
  ASSERT_FALSE(result.result->path.poses.empty());
  EXPECT_DOUBLE_EQ(result.result->path.poses.front().pose.position.x, 1.0);
}

TEST_F(LocalPlannerServerTest, reportsTfErrorForMissingRobotPoseOrTransform)
{
  activate();
  auto robot_goal = makeGoal();
  robot_goal.use_start = false;
  server_->setRobotPoseAvailable(false);
  expectError(robot_goal);
  server_->setRobotPoseAvailable(true);
  server_->setTransformAvailable(false);
  expectError(makeGoal());
}

TEST_F(LocalPlannerServerTest, rejectsNonFiniteTransformedStartAndGoal)
{
  activate();
  auto nan_start = makeGoal();
  nan_start.start.header.frame_id = "nan_transform";
  expectError(nan_start);

  auto infinite_goal = makeGoal();
  infinite_goal.goal.header.frame_id = "inf_transform";
  expectError(infinite_goal);
}

TEST_F(LocalPlannerServerTest, transformsStartAndGoalIntoMap)
{
  activate();
  auto goal = makeGoal(0.0, 7.0);
  goal.start.header.frame_id = "translated";
  goal.goal.header.frame_id = "translated";
  goal.start.pose.position.y = -1.0;
  goal.goal.pose.position.y = -1.0;
  const auto result = runGoal(goal);
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(result.result, nullptr);
  EXPECT_EQ(result.result->path.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(result.result->path.poses.front().pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(result.result->path.poses.back().pose.position.x, 8.0);
}

TEST_F(LocalPlannerServerTest, reportsEndpointsOutsideMap)
{
  activate();
  expectError(makeGoal(-0.1, 2.0));
  expectError(makeGoal(1.0, 10.0));
}

TEST_F(LocalPlannerServerTest, reportsOccupiedEndpoints)
{
  activate();
  occupy(1.0, 1.0);
  expectError(makeGoal());
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *server_->costmap()->getMutex());
    server_->costmap()->resetMap(0, 0, 100, 100);
  }
  occupy(8.0, 1.0);
  expectError(makeGoal());
}

TEST_F(LocalPlannerServerTest, reportsCostmapUpdateTimeout)
{
  server_->set_parameter(rclcpp::Parameter("costmap_update_timeout", 0.02));
  activate();
  server_->setCurrent(false);
  expectError(makeGoal());
}

TEST_F(LocalPlannerServerTest, reportsNoPathThroughSolidWall)
{
  activate();
  addSolidWall();
  expectError(makeGoal(1.0, 8.0));
}

TEST_F(LocalPlannerServerTest, cancellationTerminatesAsCanceled)
{
  server_->set_parameter(rclcpp::Parameter("max_iterations", 1000000));
  server_->set_parameter(rclcpp::Parameter("max_planning_time", 5.0));
  activate();
  addSolidWall();
  auto goal_future = client_->async_send_goal(makeGoal());
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(client_node_, goal_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto handle = goal_future.get();
  ASSERT_NE(handle, nullptr);
  auto cancel_future = client_->async_cancel_goal(handle);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(client_node_, cancel_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_FALSE(cancel_future.get()->goals_canceling.empty());
  const auto result = runResult(handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED);
  ASSERT_NE(result.result, nullptr);
}

TEST_F(LocalPlannerServerTest, preemptsLongPlanAndPublishesOnlyPendingGoalResult)
{
  server_->set_parameter(rclcpp::Parameter("max_iterations", 1000000));
  server_->set_parameter(rclcpp::Parameter("max_planning_time", 5.0));
  activate();
  addSolidWall();

  const auto first_handle = sendGoal(makeGoal(1.0, 8.0));
  ASSERT_NE(first_handle, nullptr);
  std::this_thread::sleep_for(20ms);
  const auto second_goal = makeGoal(1.0, 3.25);
  const auto second_handle = sendGoal(second_goal);
  ASSERT_NE(second_handle, nullptr);

  const auto first_result = runResult(first_handle);
  const auto second_result = runResult(second_handle);
  EXPECT_EQ(first_result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_NE(first_result.result, nullptr);
  ASSERT_EQ(second_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(second_result.result, nullptr);
  ASSERT_FALSE(second_result.result->path.poses.empty());
  EXPECT_DOUBLE_EQ(second_result.result->path.poses.back().pose.position.x, 3.25);

  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (!published_plan_ && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(client_node_);
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_NE(published_plan_, nullptr);
  ASSERT_FALSE(published_plan_->poses.empty());
  EXPECT_DOUBLE_EQ(published_plan_->poses.back().pose.position.x, 3.25);
}

TEST_F(LocalPlannerServerTest, currentCancellationDoesNotDiscardPendingGoal)
{
  activate();
  server_->blockNextTransform();

  const auto first_handle = sendGoal(makeGoal(1.0, 8.0));
  EXPECT_NE(first_handle, nullptr);
  EXPECT_TRUE(server_->waitForBlockedTransform());
  const auto second_handle = sendGoal(makeGoal(1.0, 3.5));
  EXPECT_NE(second_handle, nullptr);
  auto cancel_future = client_->async_cancel_goal(first_handle);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(client_node_, cancel_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_FALSE(cancel_future.get()->goals_canceling.empty());
  server_->releaseTransform();

  EXPECT_EQ(runResult(first_handle).code, rclcpp_action::ResultCode::CANCELED);
  const auto second_result = runResult(second_handle);
  ASSERT_EQ(second_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(second_result.result, nullptr);
  ASSERT_FALSE(second_result.result->path.poses.empty());
  EXPECT_DOUBLE_EQ(second_result.result->path.poses.back().pose.position.x, 3.5);
}

TEST_F(LocalPlannerServerTest, pendingCancellationDoesNotRestartCurrentGoal)
{
  activate();
  server_->blockNextTransform();

  const auto current_handle = sendGoal(makeGoal(1.0, 4.0));
  EXPECT_NE(current_handle, nullptr);
  EXPECT_TRUE(server_->waitForBlockedTransform());
  const auto pending_handle = sendGoal(makeGoal(1.0, 3.0));
  EXPECT_NE(pending_handle, nullptr);
  auto cancel_future = client_->async_cancel_goal(pending_handle);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(client_node_, cancel_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_FALSE(cancel_future.get()->goals_canceling.empty());
  server_->releaseTransform();

  EXPECT_EQ(runResult(pending_handle).code, rclcpp_action::ResultCode::CANCELED);
  const auto current_result = runResult(current_handle);
  ASSERT_EQ(current_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(server_->transformCallCount(), 2);
}

TEST_F(LocalPlannerServerTest, canceledPendingReplacementPreservesNewestGoal)
{
  activate();
  server_->blockNextTransform();

  const auto current_handle = sendGoal(makeGoal(1.0, 4.0));
  EXPECT_NE(current_handle, nullptr);
  EXPECT_TRUE(server_->waitForBlockedTransform());
  const auto canceled_pending_handle = sendGoal(makeGoal(1.0, 3.0));
  EXPECT_NE(canceled_pending_handle, nullptr);
  auto cancel_future = client_->async_cancel_goal(canceled_pending_handle);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(client_node_, cancel_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_FALSE(cancel_future.get()->goals_canceling.empty());
  const auto newest_goal = makeGoal(1.0, 3.75);
  const auto newest_handle = sendGoal(newest_goal);
  EXPECT_NE(newest_handle, nullptr);
  server_->releaseTransform();

  EXPECT_EQ(runResult(canceled_pending_handle).code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(runResult(current_handle).code, rclcpp_action::ResultCode::ABORTED);
  const auto newest_result = runResult(newest_handle);
  ASSERT_EQ(newest_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(newest_result.result, nullptr);
  ASSERT_FALSE(newest_result.result->path.poses.empty());
  EXPECT_DOUBLE_EQ(newest_result.result->path.poses.back().pose.position.x, 3.75);
}

TEST_F(LocalPlannerServerTest, preservesExactEndpointsAndPublishesSuccessfulPlan)
{
  activate();
  const auto goal = makeGoal();
  const auto result = runGoal(goal);
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(result.result, nullptr);
  ASSERT_GT(result.result->path.poses.size(), 2u);
  EXPECT_EQ(result.result->path.poses.front().pose, goal.start.pose);
  EXPECT_EQ(result.result->path.poses.back().pose, goal.goal.pose);
  for (size_t index = 1; index < result.result->path.poses.size(); ++index) {
    const auto & previous = result.result->path.poses[index - 1].pose.position;
    const auto & current = result.result->path.poses[index].pose.position;
    const double spacing = std::hypot(current.x - previous.x, current.y - previous.y);
    EXPECT_GT(spacing, 1e-9);
    EXPECT_LE(spacing, 0.101);
  }
  EXPECT_LT(result.result->planning_time.nanosec, 1000000000u);
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (!published_plan_ && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(client_node_);
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_NE(published_plan_, nullptr);
  EXPECT_EQ(*published_plan_, result.result->path);
}

TEST_F(LocalPlannerServerTest, invalidParametersFailConfigurationAtomically)
{
  server_->set_parameter(rclcpp::Parameter("goal_bias", 1.1));
  EXPECT_EQ(server_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_FALSE(client_->wait_for_action_server(100ms));
  EXPECT_EQ(server_->costmap(), nullptr);
}

TEST_F(LocalPlannerServerTest, nonMapCostmapFrameFailsConfiguration)
{
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__ns:=/colregs_non_map_test"});
  auto server = std::make_shared<TestPlannerServer>(options);

  EXPECT_EQ(server->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(server->costmap(), nullptr);
  server->shutdown();
}

namespace
{

nav2_colregs_ts_manager::ColregsTsStateROS::PlanningInput headOnThreatInput(
  bool velocity_valid = true, double ts_x = 5.0)
{
  nav2_colregs_ts_manager::ColregsTsStateROS::PlanningInput input;
  input.ts.stamp = rclcpp::Time(100, 0);
  nav2_colregs_ts_manager::RawTsEntry entry;
  entry.target_id = "head-on";
  entry.x = ts_x;
  entry.y = 1.0;
  entry.vx = -1.0;
  entry.vy = 0.0;
  entry.radius = 0.3;
  entry.last_seen = rclcpp::Time(100, 0);
  input.ts.ships.push_back(entry);
  input.os.x = 1.0;
  input.os.y = 1.0;
  input.os.vx = velocity_valid ? 1.0 : 0.0;
  input.os.vy = 0.0;
  input.os.velocity_valid = velocity_valid;
  return input;
}

}  // namespace

TEST_F(LocalPlannerServerTest, activeThreatProducesTwoSegmentPathThroughAvoidancePoint)
{
  server_->set_parameter(rclcpp::Parameter("prune_path", false));
  activate();
  server_->setInjectedTsInput(headOnThreatInput());

  const auto goal = makeGoal();
  const auto result = runGoal(goal);
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(result.result, nullptr);
  const auto & path = result.result->path.poses;
  ASSERT_GT(path.size(), 2u);
  EXPECT_EQ(path.front().pose, goal.start.pose);
  EXPECT_EQ(path.back().pose, goal.goal.pose);

  // Reproduce the server-side pure evaluation and verify the path passes
  // exactly through the avoidance point (raw nodes, pruning disabled).
  const auto input = headOnThreatInput();
  const auto & params = server_->tsCoreParams();
  const auto snapshot = processTs(input.ts, input.os, params);
  const auto decision = evaluateColregs(
    snapshot, input.os, goal.goal.pose.position.x, goal.goal.pose.position.y,
    "right", params);
  ASSERT_TRUE(decision.active);
  bool found_avoidance_point = false;
  for (const auto & pose : path) {
    if (std::abs(pose.pose.position.x - decision.avoidance_point.x) < 1e-9 &&
      std::abs(pose.pose.position.y - decision.avoidance_point.y) < 1e-9)
    {
      found_avoidance_point = true;
      break;
    }
  }
  EXPECT_TRUE(found_avoidance_point);
}

TEST_F(LocalPlannerServerTest, inescapableThreatFallsBackToDirectPath)
{
  activate();
  // Distance 0.2 <= os_radius + ts_radius = 0.6 → inescapable cone [[0, 2π]].
  server_->setInjectedTsInput(headOnThreatInput(true, 1.2));

  const auto goal = makeGoal();
  const auto result = runGoal(goal);
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(result.result, nullptr);
  ASSERT_GT(result.result->path.poses.size(), 1u);
  EXPECT_EQ(result.result->path.poses.front().pose, goal.start.pose);
  EXPECT_EQ(result.result->path.poses.back().pose, goal.goal.pose);
}

TEST_F(LocalPlannerServerTest, invalidOsVelocityFallsBackToDirectPath)
{
  activate();
  server_->setInjectedTsInput(headOnThreatInput(false));

  const auto goal = makeGoal();
  const auto result = runGoal(goal);
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(result.result, nullptr);
  ASSERT_GT(result.result->path.poses.size(), 1u);
  EXPECT_EQ(result.result->path.poses.front().pose, goal.start.pose);
  EXPECT_EQ(result.result->path.poses.back().pose, goal.goal.pose);
}

TEST_F(LocalPlannerServerTest, invalidAvoidDirectionFailsConfiguration)
{
  server_->set_parameter(rclcpp::Parameter("avoid_direction", "up"));
  EXPECT_EQ(server_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

}  // namespace nav2_colregs_local_planner_server
