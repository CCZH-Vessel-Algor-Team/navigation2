#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_behavior_tree/plugins/action/line_follow_geometry.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{
namespace
{
constexpr int kTracking = 0;
constexpr int kAvoiding = 1;
constexpr int kWaiting = 2;
constexpr int kManual = 3;
constexpr int kDetourIdle = 0;
constexpr int kDetourMeasure = 1;
constexpr int kDetourPlanGoal = 2;
constexpr int kDetourPlanResult = 3;
constexpr int kDetourSmoothGoal = 4;
constexpr int kDetourSmoothResult = 5;

using PlanAction = nav2_msgs::action::ComputePathToPose;
using SmoothAction = nav2_msgs::action::SmoothPath;

geometry_msgs::msg::Quaternion yawQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

nav_msgs::msg::Path toPath(
  const std::vector<LinePoint> & points, const std::string & frame,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  path.header.stamp = stamp;
  path.poses.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double dx = i + 1 < points.size() ? points[i + 1].x - points[i].x : 1.0;
    const double dy = i + 1 < points.size() ? points[i + 1].y - points[i].y : 0.0;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = points[i].x;
    pose.pose.position.y = points[i].y;
    pose.pose.orientation = yawQuaternion(std::atan2(dy, dx));
    path.poses.push_back(pose);
  }
  return path;
}

LinePolyline toPolyline(const nav_msgs::msg::Path & path)
{
  std::vector<LinePoint> points;
  points.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    points.push_back(LinePoint{pose.pose.position.x, pose.pose.position.y});
  }
  return makePolyline(points);
}
}  // namespace

class LineFollowUpdate : public BT::SyncActionNode
{
public:
  LineFollowUpdate(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("reference_path"),
      BT::InputPort<int>("blockage_policy", 0, "0 rejoin, 1 wait"),
      BT::InputPort<double>("wait_timeout_sec", 60.0, "Wait timeout"),
      BT::InputPort<double>("cross_track_tolerance", 8.0, "Return band"),
      BT::InputPort<double>("lookahead_m", 200.0, "Blockage lookahead"),
      BT::InputPort<double>("bypass_offset_m", 25.0, "Lateral bypass"),
      BT::InputPort<double>("standoff_m", 20.0, "Distance kept before a block"),
      BT::InputPort<double>("pass_margin_m", 25.0, "Distance past a block"),
      BT::InputPort<int>("line_follow_epoch", 0, "Changes when a new goal starts"),
      BT::OutputPort<nav_msgs::msg::Path>("path"),
      BT::OutputPort<int>("segment_index"),
      BT::OutputPort<double>("along_track"),
      BT::OutputPort<double>("cross_track"),
      BT::OutputPort<double>("distance_remaining"),
      BT::OutputPort<int>("line_state"),
      BT::OutputPort<bool>("mission_complete"),
      BT::OutputPort<bool>("manual_requested"),
      BT::OutputPort<bool>("waiting"),
      BT::OutputPort<bool>("detour_pending"),
      BT::OutputPort<std::string>("line_event")
    };
  }

  BT::NodeStatus tick() override
  {
    if (!initialized_) {
      initialize();
    }
    if (!initialized_) {
      return BT::NodeStatus::SUCCESS;
    }

    nav_msgs::msg::Path reference;
    if (!getInput("reference_path", reference) || reference.poses.size() < 2) {
      return BT::NodeStatus::SUCCESS;
    }

    int epoch = 0;
    getInput("line_follow_epoch", epoch);
    if (epoch != epoch_) {
      resetRuntime();
      epoch_ = epoch;
      double seeded = 0.0;
      if (config().blackboard->get("along_track", seeded)) {
        along_hint_ = seeded;
      }
    }

    geometry_msgs::msg::PoseStamped pose;
    if (!nav2_util::getCurrentPose(
        pose, *tf_, global_frame_, robot_frame_, transform_tolerance_))
    {
      return BT::NodeStatus::SUCCESS;
    }

    if (line_.points.size() != reference.poses.size()) {
      line_ = toPolyline(reference);
    }

    const auto projection = projectWindow(
      line_, pose.pose.position.x, pose.pose.position.y, along_hint_, 20.0, 30.0);
    LineProjection progress = projection;
    bool passed_corner = false;
    if (state_ == kAvoiding) {
      const auto locked = projectSegment(
        line_, static_cast<std::size_t>(locked_segment_),
        pose.pose.position.x, pose.pose.position.y);
      passed_corner = cornerPassed(
        line_, locked_segment_, projection,
        pose.pose.position.x, pose.pose.position.y, corner_release_m_);
      if (passed_corner) {
        progress = projection;
        locked_segment_ = projection.segment;
      } else if (locked.valid) {
        progress = locked;
      }
    }
    if (!progress.valid) {
      return BT::NodeStatus::SUCCESS;
    }
    along_hint_ = progress.along;

    int policy = 0;
    double wait_timeout = 60.0;
    double tolerance = 8.0;
    double lookahead = 200.0;
    double offset = 25.0;
    double standoff = 20.0;
    double pass_margin = 25.0;
    getInput("blockage_policy", policy);
    getInput("wait_timeout_sec", wait_timeout);
    getInput("cross_track_tolerance", tolerance);
    getInput("lookahead_m", lookahead);
    getInput("bypass_offset_m", offset);
    getInput("standoff_m", standoff);
    getInput("pass_margin_m", pass_margin);

    if (state_ != kAvoiding) {
      const int sample = sampleBlockage(progress, lookahead);
      if (sample > 0) {
        ++blocked_count_;
        clear_count_ = 0;
      } else if (sample == 0) {
        ++clear_count_;
        blocked_count_ = 0;
      }
    }

    if (state_ == kTracking && blocked_count_ >= 2) {
      beginBlockage(progress, policy, standoff, pass_margin);
    } else if (state_ == kWaiting) {
      updateWait(progress, wait_timeout, pose);
      if (state_ == kWaiting && clear_count_ >= 3) {
        enterTracking("wait_resume");
      }
    } else if (state_ == kAvoiding) {
      if (passed_corner) {
        enterTracking("corner_passed");
      } else {
        updateAvoid(progress, tolerance);
        if (state_ == kAvoiding) {
          advanceDetour(progress, lookahead, pass_margin);
        }
      }
    }
    if (state_ != kAvoiding) {
      detour_pending_ = false;
    }

    if (state_ == kTracking && progress.remaining < 4.0 &&
      std::abs(progress.cross) <= tolerance)
    {
      mission_complete_ = true;
      emit("mission_complete");
    }

    updateStuckRecovery(pose, progress);
    updateLookahead(pose, progress);
    publishPath(progress);
    publishRejoinMarker();
    appendAndPublishHistory(pose);
    (void)offset;
    setOutput("segment_index", progress.segment);
    setOutput("along_track", progress.along);
    setOutput("cross_track", progress.cross);
    setOutput("distance_remaining", progress.remaining);
    setOutput("line_state", state_);
    setOutput("mission_complete", mission_complete_);
    setOutput("manual_requested", manual_requested_);
    setOutput("waiting", state_ == kWaiting);
    setOutput("detour_pending", detour_pending_);
    setOutput("line_event", last_event_);
    if (manual_requested_) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  void initialize()
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    node_->get_parameter_or("global_frame", global_frame_, std::string("map"));
    node_->get_parameter_or("robot_base_frame", robot_frame_, std::string("base_link"));
    node_->get_parameter_or("transform_tolerance", transform_tolerance_, 0.5);
    if (!node_->has_parameter("line_follow_manual_mode_topic")) {
      // Optional String telemetry topic; v2 takeover goes through SetBool below.
      node_->declare_parameter<std::string>(
        "line_follow_manual_mode_topic", "line_follow/manual_request");
    }
    node_->get_parameter("line_follow_manual_mode_topic", manual_topic_);
    declareTunables();
    refreshTunables();
    client_ = node_->create_client<nav2_msgs::srv::IsPathValid>("is_path_valid");
    planner_client_ = rclcpp_action::create_client<PlanAction>(node_, "compute_path_to_pose");
    smoother_client_ = rclcpp_action::create_client<SmoothAction>(node_, "smooth_path");
    manual_client_ = node_->create_client<std_srvs::srv::SetBool>("control/request_manual");
    event_pub_ = node_->create_publisher<std_msgs::msg::String>("line_follow/event", 20);
    rejoin_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "line_follow/rejoin_marker", rclcpp::QoS(1).transient_local());
    // Latched so RViz can catch the last GridBased / smoother outputs even when
    // the boat is already tracking the stitched execution path.
    const auto latched = rclcpp::QoS(1).transient_local();
    planned_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
      "line_follow/planned_path", latched);
    smoothed_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
      "line_follow/smoothed_path", latched);
    // Actual travelled trail for RViz "实际航行轨迹".
    history_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
      "line_follow/history_path", latched);
    history_path_.header.frame_id = global_frame_;
    initialized_ = true;
  }

  void resetRuntime()
  {
    state_ = kTracking;
    along_hint_ = 0.0;
    blocked_count_ = 0;
    clear_count_ = 0;
    stable_rejoin_ = 0;
    blocked_along_latch_ = -1.0;
    mission_complete_ = false;
    manual_requested_ = false;
    rejoin_along_ = 0.0;
    side_ = 1.0;
    locked_segment_ = 0;
    last_path_along_ = -1000.0;
    last_event_.clear();
    line_.points.clear();
    stuck_along_ = -1.0;
    stuck_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    recovery_until_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    recovery_start_along_ = -1.0;
    applied_max_angle_ = -1.0;
    history_path_.poses.clear();
    history_path_.header.frame_id = global_frame_;
    cancelDetour();
  }

  void appendAndPublishHistory(const geometry_msgs::msg::PoseStamped & pose)
  {
    if (!history_path_pub_) {
      return;
    }
    const double x = pose.pose.position.x;
    const double y = pose.pose.position.y;
    if (!history_path_.poses.empty()) {
      const auto & last = history_path_.poses.back().pose.position;
      if (std::hypot(x - last.x, y - last.y) < 0.5) {
        return;
      }
    }
    geometry_msgs::msg::PoseStamped sample = pose;
    sample.header.frame_id = global_frame_;
    sample.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    history_path_.header.frame_id = global_frame_;
    history_path_.header.stamp = sample.header.stamp;
    history_path_.poses.push_back(sample);
    // Cap length so long missions do not grow without bound.
    constexpr std::size_t kMaxHistoryPoses = 8000;
    if (history_path_.poses.size() > kMaxHistoryPoses) {
      history_path_.poses.erase(
        history_path_.poses.begin(),
        history_path_.poses.begin() +
        static_cast<std::ptrdiff_t>(history_path_.poses.size() - kMaxHistoryPoses));
    }
    history_path_pub_->publish(history_path_);
  }

  void emit(const std::string & event)
  {
    if (event.empty() || event == last_event_) {
      return;
    }
    last_event_ = event;
    std_msgs::msg::String message;
    message.data = event;
    event_pub_->publish(message);
    RCLCPP_INFO(node_->get_logger(), "Line follow event: %s", event.c_str());
  }

  void enterTracking(const std::string & event)
  {
    state_ = kTracking;
    blocked_count_ = 0;
    clear_count_ = 0;
    stable_rejoin_ = 0;
    blocked_along_latch_ = -1.0;
    last_path_along_ = -1000.0;
    cancelDetour();
    emit(event);
  }

  void beginBlockage(
    const LineProjection & progress, int policy, double standoff, double pass_margin)
  {
    locked_segment_ = progress.segment;
    const double block_along = progress.along + std::max(0.0, block_ahead_);
    block_point_ = pointAt(line_, block_along);
    if (policy == 1) {
      state_ = kWaiting;
      wait_start_ = node_->now();
      block_point_ = pointAt(line_, block_along);
      const double dx = block_point_.x - progress.x;
      const double dy = block_point_.y - progress.y;
      wait_anchor_dist_ = std::hypot(dx, dy);
      last_path_along_ = -1000.0;
      emit("wait_start");
      return;
    }
    // 25 m past the first lethal cell still sits inside the inflation, so the
    // planner then aims at the obstacle. Clear it by the same 80 m used below.
    rejoin_along_ = std::min(
      polylineLength(line_) - 1.0, block_along + std::max(pass_margin, 80.0));
    const double depart = std::max(progress.along, block_along - standoff);
    if (rejoin_along_ <= depart + 5.0) {
      state_ = kWaiting;
      wait_start_ = node_->now();
      const double dx = block_point_.x - progress.x;
      const double dy = block_point_.y - progress.y;
      wait_anchor_dist_ = std::hypot(dx, dy);
      emit("wait_start");
      return;
    }
    side_ = 1.0;
    state_ = kAvoiding;
    stable_rejoin_ = 0;
    last_path_along_ = -1000.0;
    cancelDetour();
    emit("deviation_start");
  }

  void updateWait(
    const LineProjection & progress, double timeout_sec,
    const geometry_msgs::msg::PoseStamped & pose)
  {
    const double elapsed = (node_->now() - wait_start_).seconds();
    const double dx = block_point_.x - pose.pose.position.x;
    const double dy = block_point_.y - pose.pose.position.y;
    const double distance = std::hypot(dx, dy);
    const bool drifted = wait_anchor_dist_ - distance > 8.0 && distance < 30.0;
    if (elapsed > timeout_sec || drifted) {
      requestManual(elapsed > timeout_sec ? "wait_timeout" : "wait_drift");
      return;
    }
    (void)progress;
  }

  void updateAvoid(const LineProjection & progress, double tolerance)
  {
    if (progress.along + 5.0 >= rejoin_along_ && std::abs(progress.cross) <= tolerance) {
      ++stable_rejoin_;
    } else {
      stable_rejoin_ = 0;
    }
    if (stable_rejoin_ >= 3) {
      enterTracking("deviation_end");
    }
  }

  void requestManual(const std::string & event)
  {
    state_ = kManual;
    manual_requested_ = true;
    // Unified v2 manual channel used by MaritimeManualTakeover / mission_node.
    if (manual_client_ && manual_client_->service_is_ready()) {
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = true;
      manual_client_->async_send_request(request);
    }
    std_msgs::msg::String mode;
    mode.data = "MANUAL";
    if (!mode_pub_) {
      mode_pub_ = node_->create_publisher<std_msgs::msg::String>(manual_topic_, 10);
    }
    mode_pub_->publish(mode);
    emit(event);
  }

  int sampleBlockage(const LineProjection & progress, double lookahead)
  {
    const auto now = node_->now();
    if ((now - last_check_).seconds() < 0.5 && have_check_) {
      return -1;
    }
    last_check_ = now;
    have_check_ = true;
    if (!client_->service_is_ready()) {
      block_ahead_ = lookahead;
      return 0;
    }
    auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();
    request->path = toPath(
      sliceDense(line_, progress.along, 5.0), global_frame_, now);
    if (request->path.poses.size() > 2) {
      const double keep = progress.along + lookahead;
      nav_msgs::msg::Path trimmed;
      trimmed.header = request->path.header;
      for (const auto & pose : request->path.poses) {
        const auto projected = projectWindow(
          line_, pose.pose.position.x, pose.pose.position.y, progress.along, 1.0, lookahead + 5.0);
        if (projected.valid && projected.along <= keep + 1.0) {
          trimmed.poses.push_back(pose);
        }
      }
      if (trimmed.poses.size() >= 2) {
        request->path = trimmed;
      }
    }
    auto future = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::milliseconds(200)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return -1;
    }
    const auto response = future.get();
    block_ahead_ = lookahead;
    if (!response->is_valid && !response->invalid_pose_indices.empty()) {
      block_ahead_ = std::max(0, response->invalid_pose_indices.front()) * 5.0;
    }
    return response->is_valid ? 0 : 1;
  }

  void declareTunables()
  {
    // Hot-tunable without rebuilding: ros2 param set <this_node> line_follow.* 
    auto declare_double = [this](const char * name, double value) {
      if (!node_->has_parameter(name)) {
        node_->declare_parameter<double>(name, value);
      }
    };
    auto declare_bool = [this](const char * name, bool value) {
      if (!node_->has_parameter(name)) {
        node_->declare_parameter<bool>(name, value);
      }
    };
    declare_double("line_follow.straight_forward_dist", 12.0);
    declare_double("line_follow.curve_forward_dist", 12.0);
    declare_double("line_follow.extended_forward_dist", 40.0);
    // 1.0 rad (≈57°) freezes at sharp 90° lawnmower corners with short ALOS
    // forward_dist; 1.4 keeps some gate while letting approach→leg turns move.
    declare_double("line_follow.nominal_max_angle", 1.4);
    declare_double("line_follow.recovery_max_angle", 2.0);
    // When false, this node stops writing FollowPath.forward_dist so you can
    // tune ALOS directly with: ros2 param set .../controller_server FollowPath.forward_dist
    declare_bool("line_follow.manage_forward_dist", true);
  }

  void refreshTunables()
  {
    node_->get_parameter("line_follow.straight_forward_dist", nominal_forward_dist_);
    node_->get_parameter("line_follow.curve_forward_dist", curve_forward_dist_);
    node_->get_parameter("line_follow.extended_forward_dist", extended_forward_dist_);
    node_->get_parameter("line_follow.nominal_max_angle", nominal_max_angle_);
    node_->get_parameter("line_follow.recovery_max_angle", recovery_max_angle_);
    node_->get_parameter("line_follow.manage_forward_dist", manage_forward_dist_);
  }

  void ensureParamClient()
  {
    if (!param_client_) {
      param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "controller_server");
    }
  }

  void setControllerParams(double forward_dist, double max_angle)
  {
    ensureParamClient();
    if (!param_client_->service_is_ready()) {
      return;
    }
    std::vector<rclcpp::Parameter> params;
    if (manage_forward_dist_ && std::abs(forward_dist - applied_forward_dist_) >= 0.5) {
      params.emplace_back("FollowPath.forward_dist", forward_dist);
      applied_forward_dist_ = forward_dist;
    }
    if (std::abs(max_angle - applied_max_angle_) >= 0.05) {
      params.emplace_back("FollowPath.max_angle_for_motion", max_angle);
      applied_max_angle_ = max_angle;
    }
    if (!params.empty()) {
      param_client_->set_parameters(params);
    }
  }

  void updateStuckRecovery(
    const geometry_msgs::msg::PoseStamped & pose, const LineProjection & progress)
  {
    const auto now = node_->now();
    if (mission_complete_ || state_ == kManual || state_ == kWaiting) {
      stuck_along_ = progress.along;
      stuck_since_ = now;
      return;
    }
    // Progress is measured on the reference along-track. A boat circling a
    // kink keeps the same along while cmd_vel stays near zero under the ALOS
    // heading gate.
    if (stuck_along_ < 0.0 || std::abs(progress.along - stuck_along_) > 1.5) {
      stuck_along_ = progress.along;
      stuck_since_ = now;
    }
    const bool in_recovery = recovery_until_.nanoseconds() > 0 && now < recovery_until_;
    if (in_recovery) {
      return;
    }
    if (recovery_until_.nanoseconds() > 0 && now >= recovery_until_) {
      // Only restore the 57° gate after the boat has actually advanced past the
      // kink. Snapping back to nominal_max_angle while still frozen just loops
      // stuck_recovery forever (seen at approach→leg 90° corners).
      const double advanced = progress.along - recovery_start_along_;
      if (advanced >= recovery_progress_m_) {
        setControllerParams(applied_forward_dist_, nominal_max_angle_);
        recovery_until_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        recovery_start_along_ = -1.0;
        RCLCPP_INFO(
          node_->get_logger(),
          "Line-follow stuck recovery closed after +%.1f m along-track", advanced);
      } else {
        RCLCPP_WARN(
          node_->get_logger(),
          "Line-follow still stuck at along=%.1f m (+%.1f m). Extending recovery "
          "(keep max_angle=%.2f)",
          progress.along, advanced, recovery_max_angle_);
        // Fall through and rebuild a softened escape path again.
        stuck_since_ = now - rclcpp::Duration::from_seconds(stuck_timeout_sec_ + 1.0);
      }
    }
    if ((now - stuck_since_).seconds() < stuck_timeout_sec_) {
      return;
    }

    const double yaw = tf2::getYaw(pose.pose.orientation);
    std::vector<LinePoint> source;
    if (state_ == kAvoiding && execution_path_.poses.size() >= 2) {
      source.reserve(execution_path_.poses.size());
      for (const auto & stamped : execution_path_.poses) {
        source.push_back(
          LinePoint{stamped.pose.position.x, stamped.pose.position.y});
      }
    } else {
      source = sliceRelaxed(line_, progress.along, 2.0, corner_cut_m_);
    }
    auto freed = skipSharpApproach(
      pose.pose.position.x, pose.pose.position.y, yaw, source,
      recovery_bearing_rad_, 20.0);
    freed = softenTurning(freed, path_max_turn_rad_, path_turn_radius_m_, 2.0);
    if (freed.size() < 2) {
      stuck_since_ = now;
      return;
    }
    execution_path_ = toPath(freed, global_frame_, rclcpp::Time(0, 0, RCL_ROS_TIME));
    path_dirty_ = true;
    have_detour_ = state_ == kAvoiding;
    setControllerParams(extended_forward_dist_, recovery_max_angle_);
    recovery_until_ = now + rclcpp::Duration::from_seconds(recovery_hold_sec_);
    if (recovery_start_along_ < 0.0) {
      recovery_start_along_ = progress.along;
    }
    stuck_along_ = progress.along;
    stuck_since_ = now;
    last_path_along_ = -1000.0;
    emit("stuck_recovery");
    RCLCPP_WARN(
      node_->get_logger(),
      "Line-follow stuck at along=%.1f m. Softened path and raised max_angle to %.2f rad for %.0f s",
      progress.along, recovery_max_angle_, recovery_hold_sec_);
  }

  void updateLookahead(
    const geometry_msgs::msg::PoseStamped & pose, const LineProjection & progress)
  {
    refreshTunables();
    // Straight / curve distances come from line_follow.* params so they can be
    // changed live with ros2 param set, without restarting Gazebo.
    const double ahead_m = curve_mode_ ? 38.0 : 30.0;
    const double behind_m = curve_mode_ ? 20.0 : 12.0;
    curve_mode_ = state_ == kAvoiding ||
      cornerNearby(line_, progress.along, ahead_m, behind_m);
    const char * mode = curve_mode_ ? "curve" : "straight";
    double desired = curve_mode_ ? curve_forward_dist_ : nominal_forward_dist_;
    if (state_ == kAvoiding && rejoin_along_ > 0.0 && !line_.points.empty()) {
      const auto point = pointAt(line_, rejoin_along_);
      const double dx = point.x - pose.pose.position.x;
      const double dy = point.y - pose.pose.position.y;
      const double distance = std::hypot(dx, dy);
      const double yaw = tf2::getYaw(pose.pose.orientation);
      const double ahead = dx * std::cos(yaw) + dy * std::sin(yaw);
      // The rejoin point is beside or behind the boat. A short lookahead
      // tracks the curl around that point; look farther along the path.
      if (distance < 30.0 && ahead < 10.0) {
        desired = extended_forward_dist_;
        mode = "extend";
      }
    }
    if (recovery_until_.nanoseconds() > 0 && node_->now() < recovery_until_) {
      desired = std::max(desired, extended_forward_dist_);
      mode = "recover";
    }
    if (std::abs(desired - applied_forward_dist_) < 0.5) {
      return;
    }
    const double angle = (recovery_until_.nanoseconds() > 0 && node_->now() < recovery_until_) ?
      recovery_max_angle_ : nominal_max_angle_;
    setControllerParams(desired, angle);
    RCLCPP_INFO(
      node_->get_logger(), "Line-follow mode %s, forward distance %.1f m", mode, desired);
  }

  void publishDebugPath(
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & pub,
    const nav_msgs::msg::Path & path) const
  {
    if (!pub || path.poses.size() < 2) {
      return;
    }
    nav_msgs::msg::Path stamped = path;
    stamped.header.frame_id = global_frame_;
    stamped.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    for (auto & pose : stamped.poses) {
      pose.header = stamped.header;
    }
    pub->publish(stamped);
  }

  void publishRejoinMarker()
  {
    if (!rejoin_pub_) {
      return;
    }
    const bool show = state_ == kAvoiding && rejoin_along_ > 1.0 && !line_.points.empty();
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    marker.ns = "rejoin";
    marker.id = 0;
    if (!show) {
      if (!rejoin_marker_visible_) {
        return;
      }
      marker.action = visualization_msgs::msg::Marker::DELETEALL;
      array.markers.push_back(marker);
      rejoin_pub_->publish(array);
      rejoin_marker_visible_ = false;
      return;
    }
    const auto point = pointAt(line_, rejoin_along_);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = 2.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 8.0;
    marker.scale.y = 8.0;
    marker.scale.z = 8.0;
    marker.color.r = 1.0;
    marker.color.g = 0.15;
    marker.color.b = 0.85;
    marker.color.a = 0.95;
    visualization_msgs::msg::Marker label = marker;
    label.id = 1;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.pose.position.z = 10.0;
    label.scale.z = 6.0;
    label.text = "回归点";
    label.color.r = 1.0;
    label.color.g = 1.0;
    label.color.b = 1.0;
    label.color.a = 1.0;
    array.markers.push_back(marker);
    array.markers.push_back(label);
    rejoin_pub_->publish(array);
    rejoin_marker_visible_ = true;
  }

  void publishPath(const LineProjection & progress)
  {
    const bool recovering =
      recovery_until_.nanoseconds() > 0 && node_->now() < recovery_until_;
    if ((state_ == kAvoiding || recovering) && path_dirty_ &&
      execution_path_.poses.size() >= 2)
    {
      setOutput("path", execution_path_);
      path_dirty_ = false;
      last_path_along_ = progress.along;
      return;
    }
    if (state_ == kAvoiding) {
      return;
    }
    if (recovering) {
      return;
    }
    const bool refresh =
      std::abs(progress.along - last_path_along_) > 5.0 || last_path_along_ < -100.0;
    if (!refresh && !mission_complete_) {
      return;
    }
    auto points = sliceRelaxed(line_, progress.along, 2.0, corner_cut_m_);
    points = softenTurning(points, path_max_turn_rad_, path_turn_radius_m_, 2.0);
    if (points.size() < 2) {
      return;
    }
    // Stamp 0 asks TF for the latest map->odom. A sim-time stamp a few
    // hundred milliseconds ahead of odom is treated as a failed transform,
    // and the controller then mistakes the origin for the goal.
    setOutput("path", toPath(points, global_frame_, rclcpp::Time(0, 0, RCL_ROS_TIME)));
    last_path_along_ = progress.along;
  }

  void cancelDetour()
  {
    if (planner_client_ && plan_handle_) {
      planner_client_->async_cancel_goal(plan_handle_);
    }
    if (smoother_client_ && smooth_handle_) {
      smoother_client_->async_cancel_goal(smooth_handle_);
    }
    plan_handle_.reset();
    smooth_handle_.reset();
    detour_phase_ = kDetourIdle;
    push_m_ = 0.0;
    have_detour_ = false;
    path_dirty_ = false;
    execution_path_.poses.clear();
    detour_pending_ = true;
  }

  template<typename FutureT>
  bool ready(FutureT & future)
  {
    if (!future.valid()) {
      return false;
    }
    // The behavior-tree client node is not in the navigator executor. Responses
    // stay queued until this tick drains them.
    rclcpp::spin_some(node_);
    return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  }

  void advanceDetour(
    const LineProjection & progress, double lookahead, double pass_margin)
  {
    (void)pass_margin;
    const auto now = node_->now();
    if (detour_phase_ != kDetourIdle &&
      (now - phase_start_).seconds() > 8.0)
    {
      RCLCPP_WARN(
        node_->get_logger(), "Line-follow detour phase %d timed out; will try again",
        detour_phase_);
      if (planner_client_ && plan_handle_) {
        planner_client_->async_cancel_goal(plan_handle_);
      }
      if (smoother_client_ && smooth_handle_) {
        smoother_client_->async_cancel_goal(smooth_handle_);
      }
      plan_handle_.reset();
      smooth_handle_.reset();
      detour_phase_ = kDetourIdle;
      last_attempt_ = now;
    }

    if (detour_phase_ == kDetourIdle) {
      detour_pending_ = !have_detour_;
      const double period = same_fail_ >= 3 ? 2.0 : (have_detour_ ? 1.0 : 0.4);
      if ((now - last_attempt_).seconds() < period) {
        return;
      }
      if (!client_->service_is_ready() || !planner_client_->action_server_is_ready()) {
        return;
      }
      sendMeasure(progress, lookahead);
      return;
    }
    if (detour_phase_ == kDetourMeasure) {
      if (!ready(measure_future_)) {
        return;
      }
      const auto response = measure_future_.get();
      const double length = polylineLength(line_);
      // A clear scan, or a failure that does not name a cell, must not move
      // the goal. Treating the far end of the window as blocked sent the
      // boat to a later leg; a goal kept 25 m ahead never finished the detour.
      if (!response || response->is_valid || response->invalid_pose_indices.empty()) {
        detour_phase_ = kDetourIdle;
        last_attempt_ = now;
        detour_pending_ = !have_detour_;
        return;
      }
      const double cluster_end = firstClusterEnd(
        measure_from_, response->invalid_pose_indices, 5.0, 12.0, 20.0);
      if (cluster_end < 0.0) {
        detour_phase_ = kDetourIdle;
        last_attempt_ = now;
        detour_pending_ = !have_detour_;
        return;
      }
      // The latch remembers this obstacle only. A value left over from a
      // previous block, or one that jumped onto a later cluster, is dropped.
      if (blocked_along_latch_ < measure_from_ ||
        blocked_along_latch_ > cluster_end + 30.0)
      {
        blocked_along_latch_ = cluster_end;
      } else {
        blocked_along_latch_ = std::max(blocked_along_latch_, cluster_end);
      }
      // 80 m past the near blockage clears the 40 m inflation. push_m_ only
      // covers a goal the planner just refused, and it is capped.
      const double extra = std::min(push_m_, 40.0);
      double candidate = std::min(length - 1.0, blocked_along_latch_ + 80.0 + extra);
      candidate = std::max(candidate, measure_from_ + 20.0);
      if (std::abs(candidate - failed_rejoin_) < 1.0 && failed_rejoin_ > 0.0) {
        ++same_fail_;
      } else {
        same_fail_ = 0;
      }
      rejoin_along_ = candidate;
      if (same_fail_ >= 3) {
        detour_phase_ = kDetourIdle;
        last_attempt_ = now;
        detour_pending_ = !have_detour_;
        if (same_fail_ == 3) {
          RCLCPP_WARN(
            node_->get_logger(),
            "Rejoin at %.1f m is still unreachable. Holding the last feasible detour.",
            rejoin_along_);
        }
        return;
      }
      sendPlan(now);
      return;
    }
    if (detour_phase_ == kDetourPlanGoal) {
      if (!ready(plan_goal_future_)) {
        return;
      }
      plan_handle_ = plan_goal_future_.get();
      if (!plan_handle_) {
        notePlanFailure(now, "planner rejected the rejoin goal");
        return;
      }
      plan_result_future_ = planner_client_->async_get_result(plan_handle_);
      detour_phase_ = kDetourPlanResult;
      return;
    }
    if (detour_phase_ == kDetourPlanResult) {
      if (!ready(plan_result_future_)) {
        return;
      }
      const auto wrapped = plan_result_future_.get();
      plan_handle_.reset();
      if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
        wrapped.result->path.poses.size() < 2)
      {
        notePlanFailure(now, "GridBased found no path to the rejoin point");
        return;
      }
      sendSmooth(wrapped.result->path, now);
      return;
    }
    if (detour_phase_ == kDetourSmoothGoal) {
      if (!ready(smooth_goal_future_)) {
        return;
      }
      smooth_handle_ = smooth_goal_future_.get();
      if (!smooth_handle_) {
        acceptDetour(unsmoothed_path_, now, "smoother rejected the path, using GridBased path");
        return;
      }
      smooth_result_future_ = smoother_client_->async_get_result(smooth_handle_);
      detour_phase_ = kDetourSmoothResult;
      return;
    }
    if (detour_phase_ == kDetourSmoothResult) {
      if (!ready(smooth_result_future_)) {
        return;
      }
      const auto wrapped = smooth_result_future_.get();
      smooth_handle_.reset();
      if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
        wrapped.result->path.poses.size() < 2)
      {
        acceptDetour(unsmoothed_path_, now, "smoothing failed, using GridBased path");
        return;
      }
      acceptDetour(wrapped.result->path, now, "smoothed detour");
    }
  }

  void sendMeasure(const LineProjection & progress, double lookahead)
  {
    const double horizon = std::min(
      polylineLength(line_), progress.along + std::max(lookahead, 200.0));
    std::vector<LinePoint> samples;
    measure_from_ = progress.along;
    for (double along = progress.along; along <= horizon + 1e-3; along += 5.0) {
      samples.push_back(pointAt(line_, along));
    }
    if (samples.size() < 2) {
      samples.push_back(pointAt(line_, horizon));
    }
    auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();
    request->path = toPath(samples, global_frame_, node_->now());
    measure_future_ = std::move(client_->async_send_request(request).future);
    detour_phase_ = kDetourMeasure;
    phase_start_ = node_->now();
  }

  void sendPlan(const rclcpp::Time & now)
  {
    if (!planner_client_->action_server_is_ready()) {
      detour_phase_ = kDetourIdle;
      last_attempt_ = now;
      detour_pending_ = !have_detour_;
      return;
    }
    const auto point = pointAt(line_, rejoin_along_);
    const auto ahead = pointAt(
      line_, std::min(polylineLength(line_), rejoin_along_ + 5.0));
    PlanAction::Goal goal;
    goal.goal.header.frame_id = global_frame_;
    goal.goal.header.stamp = now;
    goal.goal.pose.position.x = point.x;
    goal.goal.pose.position.y = point.y;
    goal.goal.pose.orientation = yawQuaternion(
      std::atan2(ahead.y - point.y, ahead.x - point.x));
    goal.planner_id = "GridBased";
    goal.use_start = false;
    plan_goal_future_ = planner_client_->async_send_goal(goal);
    detour_phase_ = kDetourPlanGoal;
    phase_start_ = now;
    RCLCPP_INFO(
      node_->get_logger(), "Planning GridBased detour to rejoin %.1f m at (%.1f, %.1f)",
      rejoin_along_, point.x, point.y);
  }

  void sendSmooth(const nav_msgs::msg::Path & path, const rclcpp::Time & now)
  {
    unsmoothed_path_ = path;
    publishDebugPath(planned_path_pub_, path);
    if (!smoother_client_->action_server_is_ready()) {
      acceptDetour(path, now, "smoother unavailable, using GridBased path");
      return;
    }
    SmoothAction::Goal goal;
    goal.path = path;
    goal.smoother_id = "constrained_smoother";
    goal.max_smoothing_duration = rclcpp::Duration::from_seconds(0.5);
    goal.check_for_collisions = true;
    smooth_goal_future_ = smoother_client_->async_send_goal(goal);
    detour_phase_ = kDetourSmoothGoal;
    phase_start_ = now;
  }

  void notePlanFailure(const rclcpp::Time & now, const char * reason)
  {
    failed_rejoin_ = rejoin_along_;
    push_m_ = std::min(40.0, push_m_ + 20.0);
    detour_phase_ = kDetourIdle;
    last_attempt_ = now;
    detour_pending_ = !have_detour_;
    RCLCPP_WARN(
      node_->get_logger(), "%s. Next rejoin slides %.0f m farther along the reference.",
      reason, push_m_);
  }

  void acceptDetour(
    const nav_msgs::msg::Path & detour, const rclcpp::Time & now, const char * source)
  {
    publishDebugPath(smoothed_path_pub_, detour);
    auto stitched = stitchDetour(detour, rclcpp::Time(0, 0, RCL_ROS_TIME));
    detour_phase_ = kDetourIdle;
    last_attempt_ = now;
    if (stitched.poses.size() < 2) {
      detour_pending_ = !have_detour_;
      RCLCPP_WARN(node_->get_logger(), "Detour from %s was too short to follow", source);
      return;
    }
    execution_path_ = stitched;
    have_detour_ = true;
    path_dirty_ = true;
    detour_pending_ = false;
    push_m_ = 0.0;
    failed_rejoin_ = -1.0;
    same_fail_ = 0;
    RCLCPP_INFO(
      node_->get_logger(), "Line-follow %s, %zu poses, rejoin %.1f m",
      source, execution_path_.poses.size(), rejoin_along_);
  }

  nav_msgs::msg::Path stitchDetour(
    const nav_msgs::msg::Path & detour, const rclcpp::Time & stamp) const
  {
    std::vector<LinePoint> points;
    points.reserve(detour.poses.size() + 16);
    for (const auto & pose : detour.poses) {
      appendPoint(
        points, LinePoint{pose.pose.position.x, pose.pose.position.y});
    }
    for (const auto & point : sliceRelaxed(line_, rejoin_along_, 2.0, corner_cut_m_)) {
      if (!points.empty()) {
        const auto & last = points.back();
        if (std::hypot(point.x - last.x, point.y - last.y) < 2.0) {
          continue;
        }
      }
      appendPoint(points, point);
    }
    // Soften both the smoother output and the stitch onto the reference so
    // ALOS never sees a kink steeper than its forward-motion gate.
    points = softenTurning(points, path_max_turn_rad_, path_turn_radius_m_, 2.0);
    return toPath(points, global_frame_, stamp);
  }

  bool initialized_{false};
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Client<nav2_msgs::srv::IsPathValid>::SharedPtr client_;
  rclcpp_action::Client<PlanAction>::SharedPtr planner_client_;
  rclcpp_action::Client<SmoothAction>::SharedPtr smoother_client_;
  std::future<rclcpp::Client<nav2_msgs::srv::IsPathValid>::SharedResponse> measure_future_;
  std::shared_future<rclcpp_action::ClientGoalHandle<PlanAction>::SharedPtr> plan_goal_future_;
  std::shared_future<rclcpp_action::ClientGoalHandle<PlanAction>::WrappedResult> plan_result_future_;
  std::shared_future<rclcpp_action::ClientGoalHandle<SmoothAction>::SharedPtr> smooth_goal_future_;
  std::shared_future<rclcpp_action::ClientGoalHandle<SmoothAction>::WrappedResult>
  smooth_result_future_;
  rclcpp_action::ClientGoalHandle<PlanAction>::SharedPtr plan_handle_;
  rclcpp_action::ClientGoalHandle<SmoothAction>::SharedPtr smooth_handle_;
  nav_msgs::msg::Path execution_path_;
  nav_msgs::msg::Path unsmoothed_path_;
  int detour_phase_{kDetourIdle};
  int same_fail_{0};
  bool have_detour_{false};
  bool path_dirty_{false};
  bool detour_pending_{false};
  double push_m_{0.0};
  double measure_from_{0.0};
  double failed_rejoin_{-1.0};
  rclcpp::Time phase_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_attempt_{0, 0, RCL_ROS_TIME};
  rclcpp::AsyncParametersClient::SharedPtr param_client_;
  double nominal_forward_dist_{12.0};
  double curve_forward_dist_{12.0};
  bool curve_mode_{false};
  bool manage_forward_dist_{true};
  double corner_cut_m_{20.0};
  double corner_release_m_{35.0};
  double extended_forward_dist_{40.0};
  double applied_forward_dist_{-1.0};
  // Keep planned corners under ALOS max_angle_for_motion (1.0 rad ≈ 57 deg).
  double path_max_turn_rad_{0.85};
  double path_turn_radius_m_{20.0};
  double nominal_max_angle_{1.4};
  double recovery_max_angle_{2.0};
  double recovery_bearing_rad_{0.75};
  double stuck_timeout_sec_{12.0};
  double recovery_hold_sec_{25.0};
  // Require this much along-track progress before restoring the 57° gate.
  double recovery_progress_m_{8.0};
  double applied_max_angle_{-1.0};
  double stuck_along_{-1.0};
  double recovery_start_along_{-1.0};
  rclcpp::Time stuck_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time recovery_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rejoin_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smoothed_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr history_path_pub_;
  nav_msgs::msg::Path history_path_;
  bool rejoin_marker_visible_{false};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr manual_client_;
  std::string global_frame_{"map"};
  std::string robot_frame_{"base_link"};
  std::string manual_topic_{"line_follow/manual_request"};
  double transform_tolerance_{0.5};
  LinePolyline line_;
  int epoch_{-1};
  int state_{kTracking};
  double along_hint_{0.0};
  int blocked_count_{0};
  int clear_count_{0};
  int stable_rejoin_{0};
  bool mission_complete_{false};
  bool manual_requested_{false};
  double rejoin_along_{0.0};
  double blocked_along_latch_{-1.0};
  double side_{1.0};
  int locked_segment_{0};
  LinePoint block_point_;
  double wait_anchor_dist_{0.0};
  rclcpp::Time wait_start_{0, 0, RCL_ROS_TIME};
  double last_path_along_{-1000.0};
  std::string last_event_;
  rclcpp::Time last_check_{0, 0, RCL_ROS_TIME};
  bool have_check_{false};
  double block_ahead_{0.0};
};

class LineFollowFlag : public BT::ConditionNode
{
public:
  LineFollowFlag(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("value")};
  }

  BT::NodeStatus tick() override
  {
    bool value = false;
    if (!getInput("value", value) || !value) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};

class LineFollowHold : public BT::StatefulActionNode
{
public:
  LineFollowHold(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("cmd_vel_topic", "cmd_vel", "Zero-velocity topic")};
  }

  BT::NodeStatus onStart() override
  {
    publishZero();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    publishZero();
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  void publishZero()
  {
    if (!pub_) {
      auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
      std::string topic = "cmd_vel";
      getInput("cmd_vel_topic", topic);
      pub_ = node->create_publisher<geometry_msgs::msg::Twist>(topic, 10);
    }
    pub_->publish(geometry_msgs::msg::Twist());
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::LineFollowUpdate>("LineFollowUpdate");
  factory.registerNodeType<nav2_behavior_tree::LineFollowFlag>("LineFollowFlag");
  factory.registerNodeType<nav2_behavior_tree::LineFollowHold>("LineFollowHold");
}
