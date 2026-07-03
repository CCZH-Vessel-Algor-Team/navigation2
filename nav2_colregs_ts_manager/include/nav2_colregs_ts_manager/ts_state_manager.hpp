#ifndef NAV2_COLREGS_TS_MANAGER__TS_STATE_MANAGER_HPP_
#define NAV2_COLREGS_TS_MANAGER__TS_STATE_MANAGER_HPP_

#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_colregs_msgs/msg/tracked_ship_list.hpp"
#include "nav2_colregs_msgs/msg/processed_ts.hpp"
#include "nav2_colregs_msgs/msg/processed_ts_list.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_colregs_ts_manager
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

struct TSEntry
{
  double x, y, radius;
  double vx, vy;
  rclcpp::Time last_seen;
};

class TSStateManager : public rclcpp::Node
{
public:
  TSStateManager();

private:
  void trackedShipCallback(
    nav2_colregs_msgs::msg::TrackedShipList::ConstSharedPtr msg);
  void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void timerCallback();

  void computeCollisionCone(
    const TSEntry & ts,
    double os_x, double os_y, double os_speed,
    std::vector<double> & min_intervals,
    std::vector<double> & max_intervals);

  rclcpp::Subscription<nav2_colregs_msgs::msg::TrackedShipList>::SharedPtr ts_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav2_colregs_msgs::msg::ProcessedTSList>::SharedPtr processed_ts_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unordered_map<std::string, TSEntry> ts_map_;
  nav_msgs::msg::Odometry::ConstSharedPtr last_odom_;

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
