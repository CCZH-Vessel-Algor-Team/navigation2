// Copyright 2026 USV team. Licensed under Apache-2.0.
#include <chrono>
#include <memory>
#include <string>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "usv_interfaces/srv/navigation_control.hpp"

using Control = usv_interfaces::srv::NavigationControl;

// The service coordinates a bounded primitive only; all recovery order is in XML.
class NavigationContract : public BT::StatefulActionNode
{
public:
  NavigationContract(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::StatefulActionNode(name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_.add_callback_group(group_, node_->get_node_base_interface());
    client_ = node_->create_client<Control>(
      "autonomy/navigation_control", rmw_qos_profile_services_default, group_);
  }
  static BT::PortsList providedPorts() {return {BT::InputPort<std::string>("operation")};}
  BT::NodeStatus onStart() override {return send();}
  BT::NodeStatus onRunning() override
  {
    executor_.spin_some();
    if (!pending_) {return send();}
    if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      if (std::chrono::steady_clock::now() - sent_ > std::chrono::seconds(2)) {
        // A possibly registered request is not undone on timeout.
        pending_ = false;
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }
    pending_ = false;
    const auto response = future_.get();
    if (response->status == Control::Response::RUNNING) {return BT::NodeStatus::RUNNING;}
    if (response->status != Control::Response::SUCCESS) {return BT::NodeStatus::FAILURE;}
    if (operation_ == "begin") {
      config().blackboard->set<std::string>("usv_boot_id", response->boot_id);
      config().blackboard->set<std::string>("usv_attempt_id", response->attempt_id);
      config().blackboard->set<uint64_t>("usv_epoch", response->epoch);
      config().blackboard->set<double>("usv_via_radius", response->via_radius);
    }
    return BT::NodeStatus::SUCCESS;
  }
  void onHalted() override {pending_ = false;}
private:
  BT::NodeStatus send()
  {
    getInput("operation", operation_);
    if (!client_->service_is_ready()) {return BT::NodeStatus::RUNNING;}
    auto request = std::make_shared<Control::Request>();
    request->operation = operation_;
    config().blackboard->get<std::string>("usv_execution_id", request->execution_id);
    config().blackboard->get<std::string>("usv_attempt_id", request->attempt_id);
    config().blackboard->get<uint64_t>("usv_epoch", request->epoch);
    if (request->execution_id.empty()) {return BT::NodeStatus::FAILURE;}
    future_ = client_->async_send_request(request).share();
    pending_ = true;
    sent_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<Control>::SharedPtr client_;
  std::shared_future<Control::Response::SharedPtr> future_;
  std::chrono::steady_clock::time_point sent_;
  bool pending_{false};
  std::string operation_;
};

BT_REGISTER_NODES(factory) {factory.registerNodeType<NavigationContract>("NavigationContract");}
