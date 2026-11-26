// Copyright 2026 USV team. Licensed under Apache-2.0.
#ifndef NAV2_RVIZ_PLUGINS__MANAGED_NAVIGATION_HPP_
#define NAV2_RVIZ_PLUGINS__MANAGED_NAVIGATION_HPP_
#include <chrono>
#include <functional>
#include <future>
#include <string>
#include <vector>
#include <QObject>
#include <QTimer>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "usv_interfaces/msg/autonomy_state.hpp"
#include "usv_interfaces/srv/manage_mission.hpp"

namespace nav2_rviz_plugins
{
// Optional transport adapter: the original Navigation 2 widget stays in place.
class ManagedNavigation : public QObject
{
public:
  ManagedNavigation(const std::string & ns, QObject * parent,
    std::function<void()> updated, std::function<void()> goal_accepted);
  bool ready() const;
  bool fresh() const;
  bool send(const std::string & command,
    const std::vector<geometry_msgs::msg::PoseStamped> & goals = {},
    const std::string & policy = "REQUIRED");
  const usv_interfaces::msg::AutonomyState & state() const {return state_;}
  const QString & message() const {return message_;}
private:
  void poll();
  using Service = usv_interfaces::srv::ManageMission;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<Service>::SharedPtr client_;
  rclcpp::Subscription<usv_interfaces::msg::AutonomyState>::SharedPtr subscription_;
  std::shared_future<Service::Response::SharedPtr> future_;
  usv_interfaces::msg::AutonomyState state_;
  QTimer timer_;
  std::function<void()> updated_, goal_accepted_;
  std::chrono::steady_clock::time_point received_, sent_;
  QString message_;
  bool valid_{false}, pending_{false}, submitting_goal_{false};
  int64_t request_id_{0};
};
}  // namespace nav2_rviz_plugins
#endif
