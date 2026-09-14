#ifndef NAV2_COLREGS_COSTMAP_LAYERS__PERCEIVED_OBSTACLE_LAYER_HPP_
#define NAV2_COLREGS_COSTMAP_LAYERS__PERCEIVED_OBSTACLE_LAYER_HPP_

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "usv_interfaces/msg/tracked_obstacle_list.hpp"

namespace nav2_colregs_costmap_layers
{

// Object-level occupancy only: type/twist do not participate in COLREGS or prediction.
class PerceivedObstacleLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return false;}

protected:
  void obstacleCallback(usv_interfaces::msg::TrackedObstacleList::ConstSharedPtr msg);

private:
  struct Circle
  {
    geometry_msgs::msg::Point center;
    double radius;
  };
  struct Observation
  {
    Circle circle;
    int64_t stamp_ns;
  };
  struct Bounds
  {
    bool valid{false};
    double min_x{0.0}, min_y{0.0}, max_x{0.0}, max_y{0.0};
    void include(const Circle & circle, double padding);
    void expand(double * x0, double * y0, double * x1, double * y1) const;
  };

  bool lookupTransform(
    const std::string & target, const std::string & source, const rclcpp::Time & stamp,
    geometry_msgs::msg::TransformStamped & transform);
  void handleClockJump(int64_t now_ns);

  std::mutex mutex_;
  std::unordered_map<std::string, Observation> observations_;
  // Only updateBounds prepares this snapshot; subscription callbacks never alter it.
  std::vector<Circle> render_snapshot_;
  Bounds previous_bounds_;
  int64_t last_clock_ns_{-1};
  int64_t timeout_ns_{0};
  std::string tracking_frame_;
  std::string global_frame_;
  rclcpp::Subscription<usv_interfaces::msg::TrackedObstacleList>::SharedPtr sub_;
};

}  // namespace nav2_colregs_costmap_layers

#endif  // NAV2_COLREGS_COSTMAP_LAYERS__PERCEIVED_OBSTACLE_LAYER_HPP_
