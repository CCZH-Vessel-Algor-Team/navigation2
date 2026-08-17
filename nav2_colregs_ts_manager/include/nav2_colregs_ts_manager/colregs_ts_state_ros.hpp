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

#ifndef NAV2_COLREGS_TS_MANAGER__COLREGS_TS_STATE_ROS_HPP_
#define NAV2_COLREGS_TS_MANAGER__COLREGS_TS_STATE_ROS_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "nav2_colregs_msgs/msg/tracked_ship_list.hpp"
#include "nav2_colregs_ts_manager/ts_core.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace nav2_colregs_ts_manager
{

/// Lifecycle sub-node owned by the COLREGS local planner server. It maintains
/// the raw target-ship state (tracked-ship subscription + timeout eviction),
/// publishes CPA markers and serves consistent planning snapshots through
/// getPlanningInput. Orchestration mirrors the server-owned costmap child.
class ColregsTsStateROS : public nav2_util::LifecycleNode
{
public:
  struct PlanningInput
  {
    RawTsSnapshot ts;
    OsState os;
  };

  ColregsTsStateROS(
    const std::string & name, const std::string & parent_namespace,
    const bool & use_sim_time);

  /// Single-lock consistent snapshot for one planning request. OS position
  /// comes from the planning start; velocity is the cached odometry body
  /// twist rotated into the global frame (velocity_valid=false on failure).
  PlanningInput getPlanningInput(double os_x, double os_y) const;

  /// CPA/cone/barrier parameters owned by this sub-node; the hosting server
  /// uses them for the pure processTs/evaluateColregs evaluation.
  const TsCoreParams & coreParams() const
  {
    return core_params_;
  }

protected:
  nav2_util::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state) override;

private:
  bool loadAndValidateParameters();
  void trackedShipCallback(
    nav2_colregs_msgs::msg::TrackedShipList::ConstSharedPtr msg);
  void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void timerCallback();

  bool getOsPoseAndVelocity(
    double & os_x, double & os_y, double & os_vx, double & os_vy) const;

  mutable std::mutex state_mutex_;
  std::unordered_map<std::string, RawTsEntry> ts_map_;
  nav_msgs::msg::Odometry::ConstSharedPtr last_odom_;

  rclcpp::Subscription<nav2_colregs_msgs::msg::TrackedShipList>::SharedPtr ts_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    cpa_markers_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  double frequency_{10.0};
  std::string global_frame_{"map"};
  std::string robot_base_frame_{"base_link"};
  std::string odom_topic_{"odom"};
  std::string tracked_ship_topic_{"/tracked_ship"};
  TsCoreParams core_params_{};
};

}  // namespace nav2_colregs_ts_manager

#endif  // NAV2_COLREGS_TS_MANAGER__COLREGS_TS_STATE_ROS_HPP_
