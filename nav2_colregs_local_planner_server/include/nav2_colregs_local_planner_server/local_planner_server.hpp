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

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_colregs_local_planner_server/rrt_star.hpp"
#include "nav2_colregs_ts_manager/colregs_ts_state_ros.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace nav2_colregs_local_planner_server
{

class ColregsLocalPlannerServer : public nav2_util::LifecycleNode
{
public:
  explicit ColregsLocalPlannerServer(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ColregsLocalPlannerServer() override;

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

  virtual bool isCostmapCurrent() const;
  virtual bool getRobotPose(geometry_msgs::msg::PoseStamped & pose) const;
  virtual bool transformPoseToGlobalFrame(
    const geometry_msgs::msg::PoseStamped & input,
    geometry_msgs::msg::PoseStamped & output) const;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::shared_ptr<nav2_colregs_ts_manager::ColregsTsStateROS> ts_state_ros_;

private:
  using Action = nav2_msgs::action::ComputePathToPose;
  using ActionServer = nav2_util::SimpleActionServer<Action>;

  bool loadAndValidateParameters();
  void computePlan();
  void abortGoal(
    const std::shared_ptr<Action::Result> & result, uint16_t error_code,
    const std::string & message);
  nav_msgs::msg::Path makePath(
    const std::vector<RRTStarNode> & nodes,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
  std::unique_ptr<nav2_util::NodeThread> ts_thread_;
  std::unique_ptr<ActionServer> action_server_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
  RRTStarParameters planner_parameters_{};
  double action_server_result_timeout_{10.0};
  double costmap_update_timeout_{1.0};
  double max_planning_time_{0.8};
};

}  // namespace nav2_colregs_local_planner_server

#endif  // NAV2_COLREGS_LOCAL_PLANNER_SERVER__LOCAL_PLANNER_SERVER_HPP_
