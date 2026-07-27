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

#include "nav2_colregs_local_planner_server/local_planner_server.hpp"

#include <chrono>
#include <functional>
#include <memory>

#include "rcl_action/rcl_action.h"

namespace nav2_colregs_local_planner_server
{

ColregsLocalPlannerServer::ColregsLocalPlannerServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("colregs_local_planner_server", "", options)
{
  declare_parameter("action_server_result_timeout", 10.0);
}

nav2_util::CallbackReturn ColregsLocalPlannerServer::on_configure(
  const rclcpp_lifecycle::State &)
{
  path_publisher_ = create_publisher<nav_msgs::msg::Path>("local_path", 1);

  rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
  server_options.result_timeout.nanoseconds =
    RCL_S_TO_NS(get_parameter("action_server_result_timeout").as_double());
  action_server_ = std::make_unique<ActionServer>(
    shared_from_this(), "compute_local_path",
    std::bind(&ColregsLocalPlannerServer::execute, this),
    nullptr, std::chrono::milliseconds(500), true, server_options);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ColregsLocalPlannerServer::on_activate(
  const rclcpp_lifecycle::State &)
{
  path_publisher_->on_activate();
  action_server_->activate();
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ColregsLocalPlannerServer::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  action_server_->deactivate();
  path_publisher_->on_deactivate();
  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ColregsLocalPlannerServer::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  action_server_.reset();
  path_publisher_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ColregsLocalPlannerServer::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  return nav2_util::CallbackReturn::SUCCESS;
}

bool ColregsLocalPlannerServer::finalize_cancellation()
{
  if (!action_server_->is_cancel_requested()) {
    return false;
  }

  auto result = std::make_shared<Action::Result>();
  result->error_code = Action::Result::CANCELED;
  result->error_msg = "Local path computation canceled";
  action_server_->terminate_all(result);
  return true;
}

void ColregsLocalPlannerServer::execute()
{
  if (!action_server_ || !action_server_->is_server_active()) {
    return;
  }

  const auto goal = action_server_->get_current_goal();
  if (!goal) {
    return;
  }

  const auto start = std::chrono::steady_clock::now();
  if (finalize_cancellation()) {
    return;
  }

  auto result = std::make_shared<Action::Result>();

  if (goal->reference_path.poses.empty()) {
    result->error_code = Action::Result::EMPTY_PATH;
    result->error_msg = "Reference path is empty";
    action_server_->terminate_current(result);
    return;
  }

  if (goal->reference_path.header.frame_id.empty()) {
    result->error_code = Action::Result::INVALID_PATH;
    result->error_msg = "Reference path frame_id is empty";
    action_server_->terminate_current(result);
    return;
  }

  result->local_path = goal->reference_path;
  result->local_path.header.stamp = now();

  if (finalize_cancellation()) {
    return;
  }

  const auto elapsed = std::chrono::steady_clock::now() - start;
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed);
  const auto nanoseconds =
    std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed - seconds);
  result->planning_time.sec = static_cast<int32_t>(seconds.count());
  result->planning_time.nanosec = static_cast<uint32_t>(nanoseconds.count());
  result->error_code = Action::Result::NONE;

  RCLCPP_INFO(
    get_logger(), "Computed local path: %zu reference poses, %zu result poses, frame '%s'",
    goal->reference_path.poses.size(), result->local_path.poses.size(),
    result->local_path.header.frame_id.c_str());

  if (finalize_cancellation()) {
    return;
  }

  path_publisher_->publish(result->local_path);
  action_server_->succeeded_current(result);
}

}  // namespace nav2_colregs_local_planner_server
