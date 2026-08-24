#ifndef NAV2_COLREGS_TS_MANAGER__BARRIER_NODE_HPP_
#define NAV2_COLREGS_TS_MANAGER__BARRIER_NODE_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "nav2_colregs_msgs/msg/processed_ts_list.hpp"
#include "nav2_colregs_msgs/srv/get_barrier_lines.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_colregs_ts_manager
{

class BarrierNode : public rclcpp::Node
{
public:
  BarrierNode();

private:
  void tsListCallback(
    nav2_colregs_msgs::msg::ProcessedTSList::ConstSharedPtr msg);

  void handleService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_colregs_msgs::srv::GetBarrierLines::Request> request,
    const std::shared_ptr<nav2_colregs_msgs::srv::GetBarrierLines::Response> response);

  void generateBarrierLines(
    double os_x, double os_y,
    double ts_x, double ts_y, double ts_r,
    const std::string & avoid_direction,
    nav2_colregs_msgs::msg::VOBarrierLines & barriers);

  rclcpp::Subscription<nav2_colregs_msgs::msg::ProcessedTSList>::SharedPtr
    ts_list_sub_;
  rclcpp::Service<nav2_colregs_msgs::srv::GetBarrierLines>::SharedPtr
    service_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr barrier_markers_pub_;

  nav2_colregs_msgs::msg::ProcessedTSList::ConstSharedPtr last_ts_list_;

  double ray_length_{999.0};
  double os_radius_{0.3};
};

}  // namespace nav2_colregs_ts_manager

#endif  // NAV2_COLREGS_TS_MANAGER__BARRIER_NODE_HPP_
