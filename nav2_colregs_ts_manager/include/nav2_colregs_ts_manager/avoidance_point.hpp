#ifndef NAV2_COLREGS_TS_MANAGER__AVOIDANCE_POINT_HPP_
#define NAV2_COLREGS_TS_MANAGER__AVOIDANCE_POINT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_colregs_msgs/msg/processed_ts_list.hpp"
#include "nav2_colregs_msgs/srv/get_avoidance_point.hpp"
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

  int selectPrimary(const nav2_colregs_msgs::msg::ProcessedTSList & list);

  bool findSafeHeading(
    const nav2_colregs_msgs::msg::ProcessedTS & ts,
    const std::string & avoid_direction,
    double goal_x, double goal_y,
    double os_x, double os_y,
    double & safe_heading);

  rclcpp::Subscription<nav2_colregs_msgs::msg::ProcessedTSList>::SharedPtr
    ts_list_sub_;
  rclcpp::Service<nav2_colregs_msgs::srv::GetAvoidancePoint>::SharedPtr
    service_;

  nav2_colregs_msgs::msg::ProcessedTSList::ConstSharedPtr last_ts_list_;
};

}  // namespace nav2_colregs_ts_manager

#endif  // NAV2_COLREGS_TS_MANAGER__AVOIDANCE_POINT_HPP_
