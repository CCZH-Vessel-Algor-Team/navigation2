#ifndef NAV2_COLREGS_TS_MANAGER__TS_STATE_MANAGER_HPP_
#define NAV2_COLREGS_TS_MANAGER__TS_STATE_MANAGER_HPP_

#include <memory>
#include <string>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_colregs_msgs/msg/tracked_ship.hpp"
#include "nav2_colregs_msgs/msg/processed_ts.hpp"
#include "nav2_colregs_msgs/srv/get_primary_threat.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_colregs_ts_manager
{

class TSStateManager : public rclcpp_lifecycle::LifecycleNode
{
public:
  TSStateManager();

protected:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

private:
  void trackedShipCallback(nav2_colregs_msgs::msg::TrackedShip::ConstSharedPtr msg);
  void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void timerCallback();
  void handleServiceRequest(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_colregs_msgs::srv::GetPrimaryThreat::Request> request,
    const std::shared_ptr<nav2_colregs_msgs::srv::GetPrimaryThreat::Response> response);

  rclcpp::Subscription<nav2_colregs_msgs::msg::TrackedShip>::SharedPtr ts_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<nav2_colregs_msgs::srv::GetPrimaryThreat>::SharedPtr threat_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  nav2_colregs_msgs::msg::TrackedShip::ConstSharedPtr last_ts_;
  rclcpp::Time last_ts_stamp_;
  nav_msgs::msg::Odometry::ConstSharedPtr last_odom_;
  bool ts_valid_{false};

  nav2_colregs_msgs::msg::ProcessedTS threat_;
  bool has_threat_{false};

  double frequency_{10.0};
  double ts_timeout_{1.0};
  double tcpa_horizon_{3.0};
  double safety_factor_{1.1};
  double os_radius_{0.3};
  std::string global_frame_{"map"};
  std::string robot_base_frame_{"base_link"};
  std::string odom_topic_{"odom"};
};

}  // namespace nav2_colregs_ts_manager

#endif  // NAV2_COLREGS_TS_MANAGER__TS_STATE_MANAGER_HPP_
