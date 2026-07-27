#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "nav2_colregs_local_path_bt_nodes/compute_local_path_action.hpp"
#include "nav2_colregs_msgs/action/compute_local_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

namespace
{

using Action = nav2_colregs_msgs::action::ComputeLocalPath;
using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

class ComputeLocalPathActionServer : public rclcpp::Node
{
public:
  enum class ResultMode {SUCCEED, ABORT, CANCEL};

  ComputeLocalPathActionServer()
  : Node("compute_local_path_bt_test_server")
  {
    using namespace std::placeholders;  // NOLINT

    action_server_ = rclcpp_action::create_server<Action>(
      get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
      get_node_waitables_interface(), "compute_local_path",
      std::bind(&ComputeLocalPathActionServer::handleGoal, this, _1, _2),
      std::bind(&ComputeLocalPathActionServer::handleCancel, this, _1),
      std::bind(&ComputeLocalPathActionServer::handleAccepted, this, _1));
  }

  void reset(ResultMode mode)
  {
    stop();
    mode_ = mode;
    goal_received_ = false;
    std::lock_guard<std::mutex> lock(goal_mutex_);
    current_goal_.reset();
  }

  bool goalReceived() const
  {
    return goal_received_;
  }

  std::shared_ptr<const Action::Goal> currentGoal() const
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    return current_goal_;
  }

  void stop()
  {
    std::lock_guard<std::mutex> lock(cancel_mutex_);
    if (cancel_timer_) {
      cancel_timer_->cancel();
      cancel_timer_.reset();
    }
    cancel_goal_.reset();
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal)
  {
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      current_goal_ = goal;
    }
    goal_received_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto result = std::make_shared<Action::Result>();
    const auto mode = mode_.load();

    if (mode == ResultMode::SUCCEED) {
      result->local_path = goal_handle->get_goal()->reference_path;
      result->local_path.poses.at(1).pose.position.x = 42.0;
      result->error_code = Action::Result::NONE;
      goal_handle->succeed(result);
      return;
    }

    if (mode == ResultMode::ABORT) {
      result->error_code = Action::Result::INVALID_PATH;
      result->error_msg = "fake planner aborted";
      goal_handle->abort(result);
      return;
    }

    std::lock_guard<std::mutex> lock(cancel_mutex_);
    cancel_goal_ = goal_handle;
    cancel_timer_ = create_wall_timer(1ms, [this]() {completeCancellation();});
  }

  void completeCancellation()
  {
    std::shared_ptr<GoalHandle> goal_handle;
    {
      std::lock_guard<std::mutex> lock(cancel_mutex_);
      if (!cancel_goal_ || !cancel_goal_->is_canceling()) {
        return;
      }
      goal_handle = cancel_goal_;
      cancel_goal_.reset();
      cancel_timer_->cancel();
      cancel_timer_.reset();
    }

    if (!goal_handle) {
      return;
    }
    auto result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::CANCELED;
    result->error_msg = "fake planner canceled";
    goal_handle->canceled(result);
  }

  rclcpp_action::Server<Action>::SharedPtr action_server_;
  std::atomic<ResultMode> mode_{ResultMode::SUCCEED};
  std::atomic_bool goal_received_{false};
  mutable std::mutex goal_mutex_;
  std::mutex cancel_mutex_;
  std::shared_ptr<const Action::Goal> current_goal_;
  std::shared_ptr<GoalHandle> cancel_goal_;
  rclcpp::TimerBase::SharedPtr cancel_timer_;
};

class ComputeLocalPathActionTest : public ::testing::Test
{
protected:
  ~ComputeLocalPathActionTest() override
  {
    stopServer();
  }

  void SetUp() override
  {
    action_server_ = std::make_shared<ComputeLocalPathActionServer>();
    server_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    server_executor_->add_node(action_server_);
    server_thread_ = std::thread([this]() {server_executor_->spin();});
    action_server_->reset(ComputeLocalPathActionServer::ResultMode::SUCCEED);
    node_ = std::make_shared<rclcpp::Node>("compute_local_path_bt_test_client");
    cancel_node_ = std::make_shared<rclcpp::Node>("compute_local_path_bt_cancel_client");
    cancel_client_ = rclcpp_action::create_client<Action>(cancel_node_, "compute_local_path");

    blackboard_ = BT::Blackboard::create();
    blackboard_->set("node", node_);
    blackboard_->set("server_timeout", 20ms);
    blackboard_->set("bt_loop_duration", 10ms);
    blackboard_->set("wait_for_service_timeout", 1000ms);
    factory_.registerFromPlugin(COMPUTE_LOCAL_PATH_BT_PLUGIN_PATH);
  }

  void TearDown() override
  {
    stopServer();
  }

  void stopServer()
  {
    if (!server_executor_) {
      return;
    }
    server_executor_->cancel();
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
    if (action_server_) {
      action_server_->stop();
      server_executor_->remove_node(action_server_);
    }
    action_server_.reset();
    server_executor_.reset();
  }

  std::unique_ptr<BT::Tree> makeTree(bool include_input = true)
  {
    const std::string input = include_input ? R"( reference_path="{reference_path}")" : "";
    const std::string xml =
      R"(<root BTCPP_format="4"><BehaviorTree ID="MainTree"><ComputeLocalPath)" + input +
      R"( local_path="{local_path}" error_code_id="{error_code_id}" error_msg="{error_msg}"/>)"
      R"(</BehaviorTree></root>)";
    return std::make_unique<BT::Tree>(factory_.createTreeFromText(xml, blackboard_));
  }

  BT::NodeStatus tickUntilTerminal(BT::Tree & tree)
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    auto status = BT::NodeStatus::IDLE;
    do {
      status = tree.rootNode()->executeTick();
      if (status != BT::NodeStatus::RUNNING) {
        return status;
      }
      std::this_thread::sleep_for(1ms);
    } while (std::chrono::steady_clock::now() < deadline);
    return status;
  }

  bool tickUntilGoalReceived(BT::Tree & tree)
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      tree.rootNode()->executeTick();
      if (action_server_->goalReceived()) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr cancel_node_;
  rclcpp_action::Client<Action>::SharedPtr cancel_client_;
  std::shared_ptr<ComputeLocalPathActionServer> action_server_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> server_executor_;
  std::thread server_thread_;
  BT::Blackboard::Ptr blackboard_;
  BT::BehaviorTreeFactory factory_;
};

TEST_F(ComputeLocalPathActionTest, propagatesServerResultPath)
{
  auto tree = makeTree();
  nav_msgs::msg::Path input_path;
  input_path.poses.resize(3);
  input_path.poses.at(1).pose.position.x = 1.0;
  blackboard_->set("reference_path", input_path);

  EXPECT_EQ(tickUntilTerminal(*tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree->rootNode()->status(), BT::NodeStatus::SUCCESS);

  nav_msgs::msg::Path output_path;
  ASSERT_TRUE(blackboard_->get("local_path", output_path));
  EXPECT_DOUBLE_EQ(output_path.poses.at(1).pose.position.x, 42.0);
  ASSERT_NE(action_server_->currentGoal(), nullptr);
  EXPECT_DOUBLE_EQ(
    action_server_->currentGoal()->reference_path.poses.at(1).pose.position.x, 1.0);
}

TEST_F(ComputeLocalPathActionTest, rejectsMissingReferencePath)
{
  auto tree = makeTree(false);
  EXPECT_THROW(tree->rootNode()->executeTick(), BT::RuntimeError);
}

TEST_F(ComputeLocalPathActionTest, propagatesAbortedResult)
{
  action_server_->reset(ComputeLocalPathActionServer::ResultMode::ABORT);
  auto tree = makeTree();
  nav_msgs::msg::Path input_path;
  input_path.poses.resize(2);
  blackboard_->set("reference_path", input_path);

  EXPECT_EQ(tickUntilTerminal(*tree), BT::NodeStatus::FAILURE);
  EXPECT_EQ(
    blackboard_->get<Action::Result::_error_code_type>("error_code_id"),
    Action::Result::INVALID_PATH);
  EXPECT_EQ(blackboard_->get<std::string>("error_msg"), "fake planner aborted");
}

TEST_F(ComputeLocalPathActionTest, propagatesCanceledResultAsFailure)
{
  action_server_->reset(ComputeLocalPathActionServer::ResultMode::CANCEL);
  auto tree = makeTree();
  nav_msgs::msg::Path input_path;
  input_path.poses.resize(2);
  blackboard_->set("reference_path", input_path);
  ASSERT_TRUE(tickUntilGoalReceived(*tree));

  auto cancel_future = cancel_client_->async_cancel_all_goals();
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(cancel_node_, cancel_future, 1s),
    rclcpp::FutureReturnCode::SUCCESS);

  EXPECT_EQ(tickUntilTerminal(*tree), BT::NodeStatus::FAILURE);
  EXPECT_EQ(
    blackboard_->get<Action::Result::_error_code_type>("error_code_id"),
    Action::Result::CANCELED);
  EXPECT_EQ(blackboard_->get<std::string>("error_msg"), "fake planner canceled");
}

TEST_F(ComputeLocalPathActionTest, nullSuccessResultReturnsFailure)
{
  auto tree = makeTree();
  auto * action =
    dynamic_cast<nav2_behavior_tree::ComputeLocalPathAction *>(tree->rootNode());
  ASSERT_NE(action, nullptr);

  EXPECT_EQ(action->on_success(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(
    blackboard_->get<Action::Result::_error_code_type>("error_code_id"),
    Action::Result::INVALID_PATH);
  EXPECT_EQ(
    blackboard_->get<std::string>("error_msg"),
    "ComputeLocalPath action returned a null result");
}

TEST_F(ComputeLocalPathActionTest, nullAbortedResultReturnsFailure)
{
  auto tree = makeTree();
  auto * action =
    dynamic_cast<nav2_behavior_tree::ComputeLocalPathAction *>(tree->rootNode());
  ASSERT_NE(action, nullptr);

  EXPECT_EQ(action->on_aborted(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(
    blackboard_->get<Action::Result::_error_code_type>("error_code_id"),
    Action::Result::INVALID_PATH);
  EXPECT_EQ(
    blackboard_->get<std::string>("error_msg"),
    "ComputeLocalPath action aborted without a result");
}

TEST_F(ComputeLocalPathActionTest, nullCanceledResultReturnsFailure)
{
  auto tree = makeTree();
  auto * action =
    dynamic_cast<nav2_behavior_tree::ComputeLocalPathAction *>(tree->rootNode());
  ASSERT_NE(action, nullptr);

  EXPECT_EQ(action->on_cancelled(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(
    blackboard_->get<Action::Result::_error_code_type>("error_code_id"),
    Action::Result::CANCELED);
  EXPECT_EQ(
    blackboard_->get<std::string>("error_msg"),
    "Local path computation canceled");
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
