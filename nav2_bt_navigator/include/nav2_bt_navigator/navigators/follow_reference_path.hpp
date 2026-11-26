#ifndef NAV2_BT_NAVIGATOR__NAVIGATORS__FOLLOW_REFERENCE_PATH_HPP_
#define NAV2_BT_NAVIGATOR__NAVIGATORS__FOLLOW_REFERENCE_PATH_HPP_

#include <memory>
#include <string>

#include "nav2_bt_navigator/navigator.hpp"
#include "nav2_msgs/action/follow_reference_path.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"

namespace nav2_bt_navigator
{

class FollowReferencePathNavigator
  : public nav2_bt_navigator::Navigator<nav2_msgs::action::FollowReferencePath>
{
public:
  using ActionT = nav2_msgs::action::FollowReferencePath;

  FollowReferencePathNavigator()
  : Navigator() {}

  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;
  bool activate() override;
  bool deactivate() override;
  bool cleanup() override;

  std::string getName() override {return std::string("follow_reference_path");}
  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

protected:
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;
  void onLoop() override;
  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;
  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) override;

  void initializeGoal(ActionT::Goal::ConstSharedPtr goal);
  void rememberProgress();
  std::string pathSignature(const nav_msgs::msg::Path & path) const;

  rclcpp::Time start_time_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr reference_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr execution_pub_;
  int epoch_{0};
  std::string saved_signature_;
  double saved_along_{0.0};
  int saved_segment_{0};
  std::string active_signature_;
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__NAVIGATORS__FOLLOW_REFERENCE_PATH_HPP_
