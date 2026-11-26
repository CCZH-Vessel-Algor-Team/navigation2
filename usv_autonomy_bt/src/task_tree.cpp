// Copyright 2026 USV team. Licensed under Apache-2.0.
#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "usv_interfaces/msg/autonomy_state.hpp"
#include "usv_interfaces/srv/autonomy_step.hpp"

using State = usv_interfaces::msg::AutonomyState;
using Step = usv_interfaces::srv::AutonomyStep;

struct Runtime
{
  rclcpp::Node::SharedPtr node;
  State state;
  std::chrono::steady_clock::time_point received;
  bool valid{false};
};

class Condition : public BT::ConditionNode
{
public:
  Condition(const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<Runtime> runtime) : BT::ConditionNode(name, config), runtime_(runtime) {}
  static BT::PortsList providedPorts() {return {BT::InputPort<std::string>("kind")};}
  BT::NodeStatus tick() override
  {
    if (!runtime_->valid) {return BT::NodeStatus::FAILURE;}
    std::string kind; getInput("kind", kind);
    const auto & s = runtime_->state;
    auto blocked = [&s](const std::string & key) {
        return std::find(s.blockers.begin(), s.blockers.end(), key) != s.blockers.end();
      };
    bool value = false;
    if (kind == "emergency") {value = blocked("emergency");}
    else if (kind == "safety") {
      value = blocked("safety") || blocked("safety_stale") || blocked("position") ||
        blocked("regular_safety") || blocked("regular_safety_stale") ||
        blocked("rules_unknown") || blocked("fault") || blocked("retry_exhausted") ||
        blocked("execution_failed") || blocked("nonretryable_failure");
    } else if (kind == "handover") {value = blocked("maritime_handover");}
    else if (kind == "maritime") {value = blocked("maritime") || blocked("maritime_test");}
    else if (kind == "not_runnable") {
      value = s.task_id.empty() || s.user_intent != "RUN" || !s.blockers.empty() ||
        s.phase == "SUSPENDED" || s.phase == "FAULT" || s.phase == "TRANSITIONING";
    } else if (kind == "navigation") {value = s.mode == "NAVIGATION";}
    else if (kind == "hold") {value = s.mode == "HOLD";}
    else if (kind == "survey") {value = s.mode == "SURVEY";}
    else if (kind == "hold_done") {value = s.hold_duration_done;}
    else if (kind == "dock") {value = s.mode == "DOCK";}
    else if (kind == "undock") {value = s.mode == "UNDOCK";}
    else if (kind == "navigation_done") {value = s.waypoint_index >= s.waypoint_count;}
    return value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
private:
  std::shared_ptr<Runtime> runtime_;
};

class Primitive : public BT::StatefulActionNode
{
public:
  Primitive(const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<Runtime> runtime) : BT::StatefulActionNode(name, config), runtime_(runtime)
  {client_ = runtime->node->create_client<Step>("autonomy/step");}
  static BT::PortsList providedPorts() {return {BT::InputPort<std::string>("operation")};}
  BT::NodeStatus onStart() override {return send();}
  BT::NodeStatus onRunning() override
  {
    if (!pending_) {return send();}
    if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      if (std::chrono::steady_clock::now() - sent_ > std::chrono::seconds(2)) {
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }
    pending_ = false;
    auto result = future_.get();
    if (revision_ != runtime_->state.revision) {return BT::NodeStatus::FAILURE;}
    if (result->status == Step::Response::SUCCESS) {return BT::NodeStatus::SUCCESS;}
    if (result->status == Step::Response::FAILURE) {return BT::NodeStatus::FAILURE;}
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override {pending_ = false;}
private:
  BT::NodeStatus send()
  {
    if (!client_->service_is_ready() || !runtime_->valid) {return BT::NodeStatus::RUNNING;}
    auto request = std::make_shared<Step::Request>();
    getInput("operation", request->operation);
    request->expected_boot_id = runtime_->state.boot_id;
    revision_ = request->expected_revision = runtime_->state.revision;
    future_ = client_->async_send_request(request).share();
    pending_ = true;
    sent_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
  }
  std::shared_ptr<Runtime> runtime_;
  rclcpp::Client<Step>::SharedPtr client_;
  std::shared_future<Step::Response::SharedPtr> future_;
  std::chrono::steady_clock::time_point sent_;
  uint64_t revision_{0};
  bool pending_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto runtime = std::make_shared<Runtime>();
  runtime->node = std::make_shared<rclcpp::Node>("usv_task_tree_v2");
  const auto path = runtime->node->declare_parameter("tree", ament_index_cpp::get_package_share_directory(
      "usv_autonomy_bt") + "/trees/task.xml");
  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<Condition>("TaskCondition", [runtime](const std::string & name, const BT::NodeConfiguration & c) {
      return std::make_unique<Condition>(name, c, runtime);
    });
  factory.registerBuilder<Primitive>("Primitive", [runtime](const std::string & name, const BT::NodeConfiguration & c) {
      return std::make_unique<Primitive>(name, c, runtime);
    });
  auto tree = factory.createTreeFromFile(path);
  auto subscription = runtime->node->create_subscription<State>("autonomy/state", 1,
    [runtime, &tree](State::ConstSharedPtr state) {
      if (!runtime->valid || runtime->state.boot_id != state->boot_id ||
        runtime->state.revision != state->revision)
      {tree.haltTree();}
      runtime->state = *state;
      runtime->valid = true;
      runtime->received = std::chrono::steady_clock::now();
    });
  auto heartbeat = runtime->node->create_publisher<std_msgs::msg::Bool>("autonomy/bt_heartbeat", 1);
  rclcpp::WallRate rate(20.0);
  while (rclcpp::ok()) {
    rclcpp::spin_some(runtime->node);
    if (runtime->valid && std::chrono::steady_clock::now() - runtime->received < std::chrono::seconds(1)) {
      try {tree.tickRoot();}
      catch (const std::exception & e) {
        RCLCPP_ERROR(runtime->node->get_logger(), "Task BT failed: %s", e.what());
        tree.haltTree();
        break;  // Stop heartbeat; independent gate expires the execution.
      }
      std_msgs::msg::Bool alive; alive.data = true; heartbeat->publish(alive);
    } else {tree.haltTree();}
    rate.sleep();
  }
  tree.haltTree();
  rclcpp::shutdown();
  return 0;
}
