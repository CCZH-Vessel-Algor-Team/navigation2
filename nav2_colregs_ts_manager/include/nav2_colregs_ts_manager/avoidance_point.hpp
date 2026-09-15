#ifndef NAV2_COLREGS_TS_MANAGER__AVOIDANCE_POINT_HPP_
#define NAV2_COLREGS_TS_MANAGER__AVOIDANCE_POINT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_colregs_msgs/msg/processed_ts_list.hpp"
#include "nav2_colregs_msgs/srv/get_avoidance_point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_colregs_ts_manager
{

class AvoidancePointNode : public rclcpp::Node
{
public:
  AvoidancePointNode();

private:
  void tsListCallback(
    nav2_colregs_msgs::msg::ProcessedTSList::ConstSharedPtr msg);

  void handleService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_colregs_msgs::srv::GetAvoidancePoint::Request> request,
    const std::shared_ptr<nav2_colregs_msgs::srv::GetAvoidancePoint::Response> response);

  int selectPrimary(const nav2_colregs_msgs::msg::ProcessedTSList & list,
    double ox, double oy, double os_radius, double safety_factor);
  void clearMarkers();

  bool findSafeHeading(
    const nav2_colregs_msgs::msg::ProcessedTSList & state,
    const std::string & avoid_direction,
    double goal_x, double goal_y,
    double os_x, double os_y,
    double os_radius, double safety_factor,
    double & safe_heading);

  rclcpp::Subscription<nav2_colregs_msgs::msg::ProcessedTSList>::SharedPtr
    ts_list_sub_;
  rclcpp::Service<nav2_colregs_msgs::srv::GetAvoidancePoint>::SharedPtr
    service_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  nav2_colregs_msgs::msg::ProcessedTSList::ConstSharedPtr last_ts_list_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::TimerBase::SharedPtr expiry_timer_;
  double state_timeout_{1.0};
  double max_pose_delta_{3.0};
};

}  // namespace nav2_colregs_ts_manager

#endif  // NAV2_COLREGS_TS_MANAGER__AVOIDANCE_POINT_HPP_
