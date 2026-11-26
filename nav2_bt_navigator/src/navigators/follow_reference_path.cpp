#include <sstream>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_bt_navigator/navigators/follow_reference_path.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_bt_navigator
{

bool FollowReferencePathNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
{
  (void)odom_smoother;
  start_time_ = rclcpp::Time(0);
  auto node = parent_node.lock();
  reference_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "line_follow/reference_path", rclcpp::QoS(1).transient_local());
  execution_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "line_follow/execution_path", rclcpp::QoS(1).transient_local());
  return true;
}

bool FollowReferencePathNavigator::activate()
{
  reference_pub_->on_activate();
  execution_pub_->on_activate();
  return true;
}

bool FollowReferencePathNavigator::deactivate()
{
  reference_pub_->on_deactivate();
  execution_pub_->on_deactivate();
  return true;
}

bool FollowReferencePathNavigator::cleanup()
{
  reference_pub_.reset();
  execution_pub_.reset();
  return true;
}

std::string FollowReferencePathNavigator::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();
  if (!node->has_parameter("default_follow_reference_path_bt_xml")) {
    const std::string share = ament_index_cpp::get_package_share_directory("nav2_bt_navigator");
    node->declare_parameter<std::string>(
      "default_follow_reference_path_bt_xml",
      share + "/behavior_trees/follow_reference_path.xml");
  }
  node->get_parameter("default_follow_reference_path_bt_xml", default_bt_xml_filename);
  return default_bt_xml_filename;
}

bool FollowReferencePathNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  if (goal->reference_path.poses.size() < 2) {
    RCLCPP_ERROR(logger_, "Reference path needs at least two poses.");
    return false;
  }
  if (!bt_action_server_->loadBehaviorTree(goal->behavior_tree)) {
    RCLCPP_ERROR(logger_, "Error loading line-follow behavior tree.");
    return false;
  }
  initializeGoal(goal);
  return true;
}

void FollowReferencePathNavigator::goalCompleted(
  typename ActionT::Result::SharedPtr result,
  const nav2_behavior_tree::BtStatus final_bt_status)
{
  rememberProgress();
  bool manual = false;
  try {
    bt_action_server_->getBlackboard()->get<bool>("manual_requested", manual);
  } catch (...) {
  }
  if (manual) {
    result->error_code = ActionT::Result::MANUAL_HANDOVER;
    result->message = "Stopped and requested manual mode.";
    return;
  }
  if (final_bt_status == nav2_behavior_tree::BtStatus::SUCCEEDED) {
    result->error_code = ActionT::Result::SUCCESS;
    result->message = "Reference path completed.";
  } else if (final_bt_status == nav2_behavior_tree::BtStatus::CANCELED) {
    result->error_code = ActionT::Result::CANCELED;
    result->message = "Line follow canceled. Progress was kept for the same path.";
  } else {
    result->error_code = ActionT::Result::FAILED;
    result->message = "Line follow failed.";
  }
}

void FollowReferencePathNavigator::onLoop()
{
  auto feedback = std::make_shared<ActionT::Feedback>();
  geometry_msgs::msg::PoseStamped pose;
  if (nav2_util::getCurrentPose(
      pose, *feedback_utils_.tf, feedback_utils_.global_frame,
      feedback_utils_.robot_frame, feedback_utils_.transform_tolerance))
  {
    feedback->current_pose = pose;
  }
  auto blackboard = bt_action_server_->getBlackboard();
  try {
    int state = 0;
    int segment = 0;
    double along = 0.0;
    double cross = 0.0;
    double remaining = 0.0;
    std::string event;
    nav_msgs::msg::Path execution;
    blackboard->get<int>("line_state", state);
    blackboard->get<int>("segment_index", segment);
    blackboard->get<double>("along_track", along);
    blackboard->get<double>("cross_track", cross);
    blackboard->get<double>("distance_remaining", remaining);
    blackboard->get<std::string>("line_event", event);
    blackboard->get<nav_msgs::msg::Path>("path", execution);
    feedback->state = static_cast<uint8_t>(state);
    feedback->segment_index = segment;
    feedback->along_track = static_cast<float>(along);
    feedback->cross_track_error = static_cast<float>(cross);
    feedback->distance_remaining = static_cast<float>(remaining);
    feedback->event = event;
    feedback->execution_path = execution;
    if (execution_pub_->is_activated()) {
      execution_pub_->publish(execution);
    }
  } catch (...) {
  }
  feedback->navigation_time = clock_->now() - start_time_;
  bt_action_server_->publishFeedback(feedback);
}

void FollowReferencePathNavigator::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
    (goal->behavior_tree.empty() &&
    bt_action_server_->getCurrentBTFilename() == bt_action_server_->getDefaultBTFilename()))
  {
    initializeGoal(goal);
    return;
  }
  RCLCPP_WARN(
    logger_,
    "Rejecting preemption that changes the line-follow behavior tree.");
}

void FollowReferencePathNavigator::initializeGoal(ActionT::Goal::ConstSharedPtr goal)
{
  start_time_ = clock_->now();
  const auto signature = pathSignature(goal->reference_path);
  double along = 0.0;
  int segment = 0;
  if (signature == saved_signature_) {
    along = saved_along_;
    segment = saved_segment_;
  }
  active_signature_ = signature;
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<int>("line_follow_epoch", ++epoch_);
  blackboard->set<nav_msgs::msg::Path>("reference_path", goal->reference_path);
  blackboard->set<nav_msgs::msg::Path>("path", goal->reference_path);
  blackboard->set<int>("blockage_policy", static_cast<int>(goal->blockage_policy));
  blackboard->set<double>(
    "wait_timeout_sec", goal->wait_timeout_sec > 0.0 ? goal->wait_timeout_sec : 60.0);
  blackboard->set<double>(
    "cross_track_tolerance",
    goal->cross_track_tolerance > 0.0 ? goal->cross_track_tolerance : 8.0);
  blackboard->set<int>("segment_index", segment);
  blackboard->set<double>("along_track", along);
  blackboard->set<double>("cross_track", 0.0);
  blackboard->set<double>("distance_remaining", 0.0);
  blackboard->set<int>("line_state", 0);
  blackboard->set<bool>("mission_complete", false);
  blackboard->set<bool>("manual_requested", false);
  blackboard->set<bool>("waiting", false);
  blackboard->set<std::string>("line_event", "");
  if (reference_pub_->is_activated()) {
    reference_pub_->publish(goal->reference_path);
  }
}

void FollowReferencePathNavigator::rememberProgress()
{
  try {
    auto blackboard = bt_action_server_->getBlackboard();
    blackboard->get<int>("segment_index", saved_segment_);
    blackboard->get<double>("along_track", saved_along_);
    saved_signature_ = active_signature_;
  } catch (...) {
  }
}

std::string FollowReferencePathNavigator::pathSignature(const nav_msgs::msg::Path & path) const
{
  std::ostringstream stream;
  stream << path.poses.size();
  if (!path.poses.empty()) {
    const auto & front = path.poses.front().pose.position;
    const auto & back = path.poses.back().pose.position;
    stream << ":" << front.x << "," << front.y << ":" << back.x << "," << back.y;
  }
  return stream.str();
}

}  // namespace nav2_bt_navigator
