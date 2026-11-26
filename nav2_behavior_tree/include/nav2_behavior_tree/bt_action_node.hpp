// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_

#include <memory>
#include <string>
#include <chrono>
#include <random>
#include <iomanip>
#include <sstream>
#include <type_traits>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_util/node_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "usv_interfaces/srv/navigation_control.hpp"

namespace nav2_behavior_tree
{

using namespace std::chrono_literals;  // NOLINT

template<typename T, typename = void>
struct UsvRetryable {static bool get(const T &) {return false;}};
template<typename T>
struct UsvRetryable<T, std::void_t<decltype(std::declval<T>().usv_retryable)>>
{static bool get(const T & value) {return value.usv_retryable;}};

/**
 * @brief Abstract class representing an action based BT node
 * @tparam ActionT Type of action
 */
template<class ActionT>
class BtActionNode : public BT::ActionNodeBase
{
public:
  /**
   * @brief A nav2_behavior_tree::BtActionNode constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  BtActionNode(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name), should_send_goal_(true)
  {
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    // Get the required items from the blackboard
    auto bt_loop_duration =
      config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");
    server_timeout_ =
      config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
    wait_for_service_timeout_ =
      config().blackboard->template get<std::chrono::milliseconds>("wait_for_service_timeout");

    // timeout should be less than bt_loop_duration to be able to finish the current tick
    max_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(bt_loop_duration * 0.5);

    // Initialize the input and output messages
    goal_ = typename ActionT::Goal();
    result_ = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult();

    std::string remapped_action_name;
    if (getInput("server_name", remapped_action_name)) {
      action_name_ = remapped_action_name;
    }
    createActionClient(action_name_);
    usv_client_ = node_->create_client<usv_interfaces::srv::NavigationControl>(
      "autonomy/navigation_control", rmw_qos_profile_services_default, callback_group_);

    // Give the derive class a chance to do any initialization
    RCLCPP_DEBUG(node_->get_logger(), "\"%s\" BtActionNode initialized", xml_tag_name.c_str());
  }

  BtActionNode() = delete;

  virtual ~BtActionNode()
  {
  }

  /**
   * @brief Create instance of an action client
   * @param action_name Action name to create client for
   */
  void createActionClient(const std::string & action_name)
  {
    // Now that we have the ROS node to use, create the action client for this BT action
    action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name, callback_group_);

    // Make sure the server is actually there before continuing
    RCLCPP_DEBUG(node_->get_logger(), "Waiting for \"%s\" action server", action_name.c_str());
    if (!action_client_->wait_for_action_server(wait_for_service_timeout_)) {
      RCLCPP_ERROR(
        node_->get_logger(), "\"%s\" action server not available after waiting for %.2fs",
        action_name.c_str(),
        wait_for_service_timeout_.count() / 1000.0);
      throw std::runtime_error(
              std::string("Action server ") + action_name +
              std::string(" not available"));
    }
  }

  /**
   * @brief Any subclass of BtActionNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("server_name", "Action server name"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout")
    };
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  // Derived classes can override any of the following methods to hook into the
  // processing for the action: on_tick, on_wait_for_result, and on_success

  /**
   * @brief Function to perform some user-defined operation on tick
   * Could do dynamic checks, such as getting updates to values on the blackboard
   */
  virtual void on_tick()
  {
  }

  /**
   * @brief Function to perform some user-defined operation after a timeout
   * waiting for a result that hasn't been received yet. Also provides access to
   * the latest feedback message from the action server. Feedback will be nullptr
   * in subsequent calls to this function if no new feedback is received while waiting for a result.
   * @param feedback shared_ptr to latest feedback message, nullptr if no new feedback was received
   */
  virtual void on_wait_for_result(std::shared_ptr<const typename ActionT::Feedback>/*feedback*/)
  {
  }

  /**
   * @brief Function to perform some user-defined operation upon successful
   * completion of the action. Could put a value on the blackboard.
   * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
   */
  virtual BT::NodeStatus on_success()
  {
    return BT::NodeStatus::SUCCESS;
  }

  /**
   * @brief Function to perform some user-defined operation when the action is aborted.
   * @return BT::NodeStatus Returns FAILURE by default, user may override return another value
   */
  virtual BT::NodeStatus on_aborted()
  {
    return BT::NodeStatus::FAILURE;
  }

  /**
   * @brief Function to perform some user-defined operation when the action is cancelled.
   * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
   */
  virtual BT::NodeStatus on_cancelled()
  {
    return BT::NodeStatus::SUCCESS;
  }

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override
  {
    try {
      // first step to be done only at the beginning of the Action
      if (status() == BT::NodeStatus::IDLE) {
        // setting the status to RUNNING to notify the BT Loggers (if any)
        setStatus(BT::NodeStatus::RUNNING);

        // reset the flag to send the goal or not, allowing the user the option to set it in on_tick
        should_send_goal_ = true;

        // user defined callback, may modify "should_send_goal_".
        on_tick();

        if (!should_send_goal_) {
          return BT::NodeStatus::FAILURE;
        }
        send_new_goal();
      }

      // if new goal was sent and action server has not yet responded
      // check the future goal handle
      if (future_goal_handle_) {
        auto elapsed =
          (node_->now() - time_goal_sent_).template to_chrono<std::chrono::milliseconds>();
        if (!is_future_goal_handle_complete(elapsed)) {
          // return RUNNING if there is still some time before timeout happens
          if (elapsed < server_timeout_) {
            return BT::NodeStatus::RUNNING;
          }
          // if server has taken more time than the specified timeout value return FAILURE
          RCLCPP_WARN(
            node_->get_logger(),
            "Timed out while waiting for action server to acknowledge goal request for %s",
            action_name_.c_str());
          future_goal_handle_.reset();
          return BT::NodeStatus::FAILURE;
        }
      }

      // The following code corresponds to the "RUNNING" loop
      if (rclcpp::ok() && !goal_result_available_) {
        // user defined callback. May modify the value of "goal_updated_"
        on_wait_for_result(feedback_);

        // reset feedback to avoid stale information
        feedback_.reset();

        auto goal_status = goal_handle_->get_status();
        if (goal_updated_ && (goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
          goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED))
        {
          goal_updated_ = false;
          send_new_goal();
          auto elapsed =
            (node_->now() - time_goal_sent_).template to_chrono<std::chrono::milliseconds>();
          if (!is_future_goal_handle_complete(elapsed)) {
            if (elapsed < server_timeout_) {
              return BT::NodeStatus::RUNNING;
            }
            RCLCPP_WARN(
              node_->get_logger(),
              "Timed out while waiting for action server to acknowledge goal request for %s",
              action_name_.c_str());
            future_goal_handle_.reset();
            return BT::NodeStatus::FAILURE;
          }
        }

        callback_group_executor_.spin_some();

        // check if, after invoking spin_some(), we finally received the result
        if (!goal_result_available_) {
          // Yield this Action, returning RUNNING
          return BT::NodeStatus::RUNNING;
        }
      }
    } catch (const std::runtime_error & e) {
      if (e.what() == std::string("managed navigation registration timed out") ||
        e.what() == std::string("managed navigation registration unavailable or refused"))
      {
        // No Goal was sent for this registration. Freeze the whole attempt,
        // including any FollowPath still running during a path replacement.
        // Return FAILURE to RecoveryNode instead of aborting the outer tree.
        // Its quiet/retry sequence still requires all actual exit evidence and
        // charges the persisted budget. A refused registration is not retryable.
        const bool retryable =
          e.what() == std::string("managed navigation registration timed out");
        stop_managed_attempt(retryable);
        RCLCPP_WARN(node_->get_logger(), "%s for %s", e.what(), action_name_.c_str());
        return BT::NodeStatus::FAILURE;
      }
      if (e.what() == std::string("send_goal failed") ||
        e.what() == std::string("Goal was rejected by the action server"))
      {
        // Action related failure that should not fail the tree, but the node
        return BT::NodeStatus::FAILURE;
      } else {
        // Internal exception to propagate to the tree
        throw e;
      }
    }

    BT::NodeStatus status;
    switch (result_.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        status = on_success();
        break;

      case rclcpp_action::ResultCode::ABORTED:
        stop_managed_attempt(UsvRetryable<typename ActionT::Result>::get(result_.result ? *result_.result : typename ActionT::Result()));
        status = on_aborted();
        break;

      case rclcpp_action::ResultCode::CANCELED:
        stop_managed_attempt(false);
        status = on_cancelled();
        break;

      default:
        throw std::logic_error("BtActionNode::Tick: invalid status value");
    }

    goal_handle_.reset();
    return status;
  }

  /**
   * @brief The other (optional) override required by a BT action. In this case, we
   * make sure to cancel the ROS2 action if it is still running.
   */
  void halt() override
  {
    // Exit reporting remains live, but no callback from this retired send may
    // update the node after its blackboard is reused for another execution.
    ++callback_generation_;
    if (should_cancel_goal()) {
      auto future_result = action_client_->async_get_result(goal_handle_);
      auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
      if (callback_group_executor_.spin_until_future_complete(future_cancel, server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Failed to cancel action server for %s", action_name_.c_str());
      }

      if (callback_group_executor_.spin_until_future_complete(future_result, server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Failed to get result for %s in node halt!", action_name_.c_str());
      }

      on_cancelled();
    }
    goal_handle_.reset();
    future_goal_handle_.reset();
    feedback_.reset();
    goal_result_available_ = false;
    goal_updated_ = false;
    setStatus(BT::NodeStatus::IDLE);
  }

protected:
  void stop_managed_attempt(bool retryable)
  {
    if (action_name_ != "follow_path" && action_name_ != "compute_path_to_pose" &&
      action_name_ != "compute_path_through_poses") {return;}
    using Control = usv_interfaces::srv::NavigationControl;
    auto request = std::make_shared<Control::Request>();
    config().blackboard->get<std::string>("usv_execution_id", request->execution_id);
    if (request->execution_id.empty()) {return;}
    request->operation = "failure";
    config().blackboard->get<std::string>("usv_attempt_id", request->attempt_id);
    config().blackboard->get<uint64_t>("usv_epoch", request->epoch);
    request->retryable = retryable;
    auto response = usv_client_->async_send_request(request);
    callback_group_executor_.spin_until_future_complete(response, server_timeout_);
  }
  /**
   * @brief Function to check if current goal should be cancelled
   * @return bool True if current goal should be cancelled, false otherwise
   */
  bool should_cancel_goal()
  {
    // Shut the node down if it is currently running
    if (status() != BT::NodeStatus::RUNNING) {
      return false;
    }

    // No need to cancel the goal if goal handle is invalid
    if (!goal_handle_) {
      return false;
    }

    callback_group_executor_.spin_some();
    auto status = goal_handle_->get_status();

    // Check if the goal is still executing
    return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
           status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
  }

  /**
   * @brief Function to send new goal to action server
   */
  void send_new_goal()
  {
    const auto generation = ++callback_generation_;
    goal_result_available_ = false;
    feedback_.reset();
    using Control = usv_interfaces::srv::NavigationControl;
    auto tracking = std::make_shared<Control::Request>();
    config().blackboard->get<std::string>("usv_execution_id", tracking->execution_id);
    const bool managed = !tracking->execution_id.empty() &&
      (action_name_ == "follow_path" || action_name_ == "compute_path_to_pose" ||
      action_name_ == "compute_path_through_poses");
    if (managed) {
      config().blackboard->get<std::string>("usv_attempt_id", tracking->attempt_id);
      config().blackboard->get<uint64_t>("usv_epoch", tracking->epoch);
      tracking->operation = "register";
      tracking->key = action_name_ + ":" + std::to_string(std::random_device{}()) + ":" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
      auto registered = usv_client_->async_send_request(tracking);
      const auto registration_status =
        callback_group_executor_.spin_until_future_complete(registered, server_timeout_);
      if (registration_status != rclcpp::FutureReturnCode::SUCCESS ||
        registered.get()->status != Control::Response::SUCCESS)
      {
        // This branch has NOT called async_send_goal. The registration can
        // nevertheless have reached the coordinator before its reply timed
        // out. Retire that exact unsent registration, even after BT halt.
        // The coordinator remembers a tombstone if this arrives first.
        auto unsent = std::make_shared<Control::Request>(*tracking);
        unsent->operation = "not_sent";
        usv_client_->async_send_request(unsent);
        throw std::runtime_error(registration_status == rclcpp::FutureReturnCode::TIMEOUT ?
          "managed navigation registration timed out" :
          "managed navigation registration unavailable or refused");
      }
    }
    auto report_terminal = [client = usv_client_, tracking, managed](uint8_t outcome) {
        if (managed) {
          auto request = std::make_shared<Control::Request>(*tracking);
          request->operation = "terminal";
          request->outcome = outcome;
          client->async_send_request(request);
        }
      };
    auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [report_terminal, client = usv_client_, tracking, managed](typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr handle) {
        if (!handle) {report_terminal(3);}
        else if (managed) {
          auto request = std::make_shared<Control::Request>(*tracking);
          request->operation = "accepted";
          // Humble's to_string does not pad individual bytes. GetResult needs
          // an unambiguous UUID, including bytes below 0x10 and leading zeros.
          std::ostringstream uuid;
          uuid << std::hex << std::setfill('0');
          for (const auto byte : handle->get_goal_id()) {
            uuid << std::setw(2) << static_cast<unsigned>(byte);
          }
          request->goal_id = uuid.str();
          client->async_send_request(request);
        }
      };
    send_goal_options.result_callback =
      [this, report_terminal, generation](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result) {
        report_terminal(result.code == rclcpp_action::ResultCode::SUCCEEDED ? 0 :
          (result.code == rclcpp_action::ResultCode::CANCELED ? 2 : 1));
        if (generation != callback_generation_) {return;}
        // The send identity is fixed in this closure. This also handles a fast
        // result arriving before tick() consumes the goal-acceptance future.
        goal_result_available_ = true;
        result_ = result;
      };
    send_goal_options.feedback_callback =
      [this, generation](typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr handle,
        const std::shared_ptr<const typename ActionT::Feedback> feedback) {
        if (generation == callback_generation_ && goal_handle_ && handle->get_goal_id() == goal_handle_->get_goal_id()) {
          feedback_ = feedback;
        }
      };

    future_goal_handle_ = std::make_shared<
      std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>>(
      action_client_->async_send_goal(goal_, send_goal_options));
    time_goal_sent_ = node_->now();
  }

  /**
   * @brief Function to check if the action server acknowledged a new goal
   * @param elapsed Duration since the last goal was sent and future goal handle has not completed.
   * After waiting for the future to complete, this value is incremented with the timeout value.
   * @return boolean True if future_goal_handle_ returns SUCCESS, False otherwise
   */
  bool is_future_goal_handle_complete(std::chrono::milliseconds & elapsed)
  {
    auto remaining = server_timeout_ - elapsed;

    // server has already timed out, no need to sleep
    if (remaining <= std::chrono::milliseconds(0)) {
      future_goal_handle_.reset();
      return false;
    }

    auto timeout = remaining > max_timeout_ ? max_timeout_ : remaining;
    auto result =
      callback_group_executor_.spin_until_future_complete(*future_goal_handle_, timeout);
    elapsed += timeout;

    if (result == rclcpp::FutureReturnCode::INTERRUPTED) {
      future_goal_handle_.reset();
      throw std::runtime_error("send_goal failed");
    }

    if (result == rclcpp::FutureReturnCode::SUCCESS) {
      goal_handle_ = future_goal_handle_->get();
      future_goal_handle_.reset();
      if (!goal_handle_) {
        throw std::runtime_error("Goal was rejected by the action server");
      }
      return true;
    }

    return false;
  }

  /**
   * @brief Function to increment recovery count on blackboard if this node wraps a recovery
   */
  void increment_recovery_count()
  {
    int recovery_count = 0;
    config().blackboard->template get<int>("number_recoveries", recovery_count);  // NOLINT
    recovery_count += 1;
    config().blackboard->template set<int>("number_recoveries", recovery_count);  // NOLINT
  }

  std::string action_name_;
  typename std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;
  rclcpp::Client<usv_interfaces::srv::NavigationControl>::SharedPtr usv_client_;

  // All ROS2 actions have a goal and a result
  typename ActionT::Goal goal_;
  bool goal_updated_{false};
  bool goal_result_available_{false};
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult result_;

  // To handle feedback from action server
  std::shared_ptr<const typename ActionT::Feedback> feedback_;

  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  // The timeout value while waiting for response from a server when a
  // new action goal is sent or canceled
  std::chrono::milliseconds server_timeout_;

  // The timeout value for BT loop execution
  std::chrono::milliseconds max_timeout_;

  // The timeout value for waiting for a service to response
  std::chrono::milliseconds wait_for_service_timeout_;

  // To track the action server acknowledgement when a new goal is sent
  std::shared_ptr<std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>>
  future_goal_handle_;
  rclcpp::Time time_goal_sent_;
  uint64_t callback_generation_{0};

  // Can be set in on_tick or on_wait_for_result to indicate if a goal should be sent.
  bool should_send_goal_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_
