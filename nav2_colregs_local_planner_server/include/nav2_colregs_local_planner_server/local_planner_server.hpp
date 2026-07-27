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

#ifndef NAV2_COLREGS_LOCAL_PLANNER_SERVER__LOCAL_PLANNER_SERVER_HPP_
#define NAV2_COLREGS_LOCAL_PLANNER_SERVER__LOCAL_PLANNER_SERVER_HPP_

#include <memory>

#include "nav2_colregs_msgs/action/compute_local_path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace nav2_colregs_local_planner_server
{

/// Serialized pass-through server; cancellation terminates all goals and preemption is unsupported.
class ColregsLocalPlannerServer : public nav2_util::LifecycleNode
{
public:
  explicit ColregsLocalPlannerServer(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ColregsLocalPlannerServer() override = default;

protected:
  nav2_util::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & state) override;

private:
  using Action = nav2_colregs_msgs::action::ComputeLocalPath;
  using ActionServer = nav2_util::SimpleActionServer<Action>;

  bool finalize_cancellation();
  void execute();

  std::unique_ptr<ActionServer> action_server_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
};

}  // namespace nav2_colregs_local_planner_server

#endif  // NAV2_COLREGS_LOCAL_PLANNER_SERVER__LOCAL_PLANNER_SERVER_HPP_
