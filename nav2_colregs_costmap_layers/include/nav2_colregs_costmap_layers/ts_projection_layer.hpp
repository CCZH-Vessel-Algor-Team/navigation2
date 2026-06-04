#ifndef NAV2_COLREGS_COSTMAP_LAYERS__TS_PROJECTION_LAYER_HPP_
#define NAV2_COLREGS_COSTMAP_LAYERS__TS_PROJECTION_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_colregs_msgs/msg/tracked_ship.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_colregs_costmap_layers
{

class TSProjectionLayer : public nav2_costmap_2d::Layer
{
public:
  TSProjectionLayer();

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override;

private:
  void trackedShipCallback(
    nav2_colregs_msgs::msg::TrackedShip::ConstSharedPtr msg);

  void transformTS(geometry_msgs::msg::PoseStamped & out);

  nav2_colregs_msgs::msg::TrackedShip::ConstSharedPtr last_msg_;
  rclcpp::Subscription<nav2_colregs_msgs::msg::TrackedShip>::SharedPtr sub_;
  std::string global_frame_;

  // Cumulative bounds of all TS positions between updateBounds cycles.
  // Prevents trailing residue when the TS moves faster than costmap updates.
  bool has_cumulative_bounds_{false};
  double cum_min_x_, cum_min_y_, cum_max_x_, cum_max_y_;

  bool msg_received_{false};
};

}  // namespace nav2_colregs_costmap_layers

#endif  // NAV2_COLREGS_COSTMAP_LAYERS__TS_PROJECTION_LAYER_HPP_
