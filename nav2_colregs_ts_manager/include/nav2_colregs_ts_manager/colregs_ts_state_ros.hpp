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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

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
/// complete stamped target-ship and odometry observations,
/// publishes CPA markers and serves consistent planning snapshots through
/// getPlanningInput. Orchestration mirrors the server-owned costmap child.
class ColregsTsStateROS : public nav2_util::LifecycleNode
{
public:
  struct PlanningInput
  {
    RawTsSnapshot ts;
    OsState os;
    std::uint64_t lifecycle_generation{0};
    std::uint64_t clock_epoch{0};
    TsCoreParams params{};
  };

  ColregsTsStateROS(
    const std::string & name, const std::string & parent_namespace,
    const bool & use_sim_time);

  /// Single-lock observation copy, followed by lock-free TF and extrapolation.
  /// The request start anchors OS only when close to the current live pose.
  /// Callers must inspect ts.status before using either part of the input.
  /// TF waiting shares one steady-wall budget, capped by the caller deadline.
  PlanningInput getPlanningInput(
    double os_x, double os_y,
    std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::time_point::max());

  /// Check lifecycle/clock provenance, independently of ts.status/freshness.
  /// The server must recheck during planning and before committing a result.
  bool isPlanningInputCurrent(const PlanningInput & input) const;

  /// CPA/cone/barrier parameters owned by this sub-node; the hosting server
  /// can obtain a thread-safe copy. Planning uses the copy in PlanningInput.
  TsCoreParams coreParams() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
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
    nav2_colregs_msgs::msg::TrackedShipList::ConstSharedPtr msg, uint64_t generation);
  void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg, uint64_t generation);
  void timerCallback(uint64_t generation);

  struct TfResources
  {
    // Keep listener and buffer alive together for lock-free snapshot users.
    std::shared_ptr<tf2_ros::Buffer> buffer;
    std::shared_ptr<tf2_ros::TransformListener> listener;
  };

  struct ObservationTimes
  {
    rclcpp::Time receipt{0, 0, RCL_ROS_TIME};
    int64_t newest_stamp{-1};
    // Receipt admission only: VALID includes bounded-ahead pending data.
    // A query must still check measurement AND receipt age at calculation time.
    InputStatus status{InputStatus::NO_DATA};
    std::string reason;
  };

  struct Configuration
  {
    TsCoreParams core;
    std::chrono::nanoseconds timer_period{100000000};
    std::chrono::steady_clock::duration transform_timeout{std::chrono::milliseconds(200)};
    double own_ship_state_timeout{1.0};
    double max_request_position_delta{3.0};
    std::string global_frame{"map"};
    std::string robot_base_frame{"base_link"};
    std::string odom_topic{"odom"};
    std::string tracked_ship_topic{"/tracked_ship"};
  };

  PlanningInput collectInput(
    bool request_anchor, double os_x, double os_y,
    std::chrono::steady_clock::time_point deadline);
  void clearObservationsLocked();
  void recordReceiptLocked(
    const builtin_interfaces::msg::Time & stamp, const rclcpp::Time & receipt,
    double timeout, ObservationTimes & times);

  mutable std::mutex state_mutex_;
  nav2_colregs_msgs::msg::TrackedShipList::ConstSharedPtr last_tracks_;
  nav_msgs::msg::Odometry::ConstSharedPtr last_odom_;
  ObservationTimes track_times_;
  ObservationTimes odom_times_;
  Configuration config_;
  bool configured_{false};
  bool accepting_inputs_{false};
  bool active_{false};
  uint64_t generation_{0};
  std::atomic<uint64_t> clock_epoch_{0};

  rclcpp::Subscription<nav2_colregs_msgs::msg::TrackedShipList>::SharedPtr ts_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    cpa_markers_pub_;

  std::shared_ptr<TfResources> tf_resources_;

  TsCoreParams core_params_{};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::JumpHandler::SharedPtr clock_jump_handler_;
};

}  // namespace nav2_colregs_ts_manager

#endif  // NAV2_COLREGS_TS_MANAGER__COLREGS_TS_STATE_ROS_HPP_
