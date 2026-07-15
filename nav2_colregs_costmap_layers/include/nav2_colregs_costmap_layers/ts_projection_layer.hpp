#ifndef NAV2_COLREGS_COSTMAP_LAYERS__TS_PROJECTION_LAYER_HPP_
#define NAV2_COLREGS_COSTMAP_LAYERS__TS_PROJECTION_LAYER_HPP_

#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_colregs_msgs/msg/tracked_ship_list.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_colregs_costmap_layers
{

inline std::string uuidToString(const uint8_t * data)
{
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  for (size_t i = 0; i < 16; ++i) {
    if (i == 4 || i == 6 || i == 8 || i == 10) oss << '-';
    oss << std::setw(2) << static_cast<int>(data[i]);
  }
  return oss.str();
}

struct ShipEntry
{
  double x, y, radius;
  rclcpp::Time last_seen;
  bool has_cumulative_bounds{false};
  double cum_min_x, cum_min_y, cum_max_x, cum_max_y;
};

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
    nav2_colregs_msgs::msg::TrackedShipList::ConstSharedPtr msg);

  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & from_frame);

  rclcpp::Subscription<nav2_colregs_msgs::msg::TrackedShipList>::SharedPtr sub_;
  std::string global_frame_;
  std::string tf_frame_;  // frame_id of last received TrackedShipList

  std::unordered_map<std::string, ShipEntry> ships_;
  double track_timeout_{3.0};
  std::string tracked_ship_topic_{"/dynamic_ship/tracked_ships"};
};

}  // namespace nav2_colregs_costmap_layers

#endif  // NAV2_COLREGS_COSTMAP_LAYERS__TS_PROJECTION_LAYER_HPP_
