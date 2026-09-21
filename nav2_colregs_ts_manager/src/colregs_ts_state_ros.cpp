// Copyright (c) 2026 Vector Wang
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_colregs_ts_manager/colregs_ts_state_ros.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <exception>
#include <iomanip>
#include <limits>
#include <sstream>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_colregs_ts_manager
{
namespace
{

const std::vector<std::string> configuration_parameters = {
  "update_frequency", "track_list_timeout", "own_ship_state_timeout", "transform_timeout",
  "threat_tcpa_horizon", "threat_radius_scale", "avoidance_radius_scale", "os_radius",
  "point_extension_distance", "lateral_margin", "closing_segment_length",
  "max_request_position_delta", "global_frame", "robot_base_frame", "odom_topic",
  "tracked_ship_topic"};

const std::vector<std::pair<std::string, std::string>> obsolete_parameters = {
  {"frequency", "update_frequency"}, {"ts_timeout", "track_list_timeout"},
  {"tcpa_horizon", "threat_tcpa_horizon"},
  {"safety_factor", "threat_radius_scale / avoidance_radius_scale"},
  {"barrier_ray_length", "closing_segment_length"}};

std::string uuidToString(const uint8_t * data)
{
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  for (size_t i = 0; i < 16; ++i) {
    if (i == 4 || i == 6 || i == 8 || i == 10) {oss << '-';}
    oss << std::setw(2) << static_cast<int>(data[i]);
  }
  return oss.str();
}

bool validFrame(const std::string & frame)
{
  return !frame.empty() && frame.front() != '/' &&
         std::none_of(
    frame.begin(), frame.end(), [](unsigned char c) {
      return std::isspace(c) || std::iscntrl(c);
    });
}

template<typename VectorT>
bool finiteVector(const VectorT & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool finiteVector(const tf2::Vector3 & value)
{
  return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}

bool validQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(norm) && std::abs(norm - 1.0) <= 1e-3;
}

bool validPose(const geometry_msgs::msg::Pose & pose)
{
  // Message heading is not used to rotate velocity (or to obtain the live OS
  // pose). Check finiteness without requiring a supplied/normalized heading.
  const auto & q = pose.orientation;
  return finiteVector(pose.position) && std::isfinite(q.x) && std::isfinite(q.y) &&
         std::isfinite(q.z) && std::isfinite(q.w);
}

bool validTwist(const geometry_msgs::msg::Twist & twist)
{
  return finiteVector(twist.linear) && finiteVector(twist.angular);
}

InputStatus freshness(const rclcpp::Time & stamp, const rclcpp::Time & now, double timeout)
{
  const double age = (now - stamp).seconds();
  if (age < 0.0) {return InputStatus::INVALID_STATE;}
  return age <= timeout ? InputStatus::VALID : InputStatus::STALE_STATE;
}

// lookupTransform at a measurement timestamp must never silently turn stamp=0
// into "latest dynamic TF". Static transforms (returned stamp=0) remain valid.
tf2::Transform measurementTransform(
  tf2_ros::Buffer & buffer, const std::string & target, const std::string & source,
  const rclcpp::Time & stamp)
{
  if (target == source) {return tf2::Transform::getIdentity();}
  const auto message = buffer.lookupTransform(target, source, stamp, rclcpp::Duration(0, 0));
  if (!finiteVector(message.transform.translation) ||
    !validQuaternion(message.transform.rotation) ||
    (stamp.nanoseconds() == 0 && rclcpp::Time(message.header.stamp).nanoseconds() != 0))
  {
    throw tf2::TransformException("Invalid transform or ambiguous zero measurement timestamp");
  }
  tf2::Transform transform;
  tf2::fromMsg(message.transform, transform);
  return transform;
}

}  // namespace

ColregsTsStateROS::ColregsTsStateROS(
  const std::string & name, const std::string & parent_namespace,
  const bool & use_sim_time)
: nav2_util::LifecycleNode(name, "",
    rclcpp::NodeOptions().arguments({
    "--ros-args", "-r", std::string("__ns:=") +
    (parent_namespace.empty() || parent_namespace == "/" ? "/" : parent_namespace),
    "--ros-args", "-r", name + ":" + std::string("__node:=") + name,
    "--ros-args", "-p", "use_sim_time:=" + std::string(use_sim_time ? "true" : "false"),
  }))
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description =
    "Set while UNCONFIGURED. The complete group is validated and cached at configure; "
    "changes are rejected while configured (inactive or active). Cleanup before setting "
    "and configuring again, or restart. use_sim_time is managed separately by ROS.";
  descriptor.read_only = false;
  declare_parameter("update_frequency", 10.0, descriptor);
  declare_parameter("track_list_timeout", 3.0, descriptor);
  declare_parameter("own_ship_state_timeout", 1.0, descriptor);
  auto transform_descriptor = descriptor;
  transform_descriptor.description =
    "Shared planning-only TF wait budget [steady wall seconds], >= 0; diagnostics never wait. " +
    descriptor.description;
  declare_parameter("transform_timeout", 0.2, transform_descriptor);
  declare_parameter("threat_tcpa_horizon", 10.0, descriptor);
  declare_parameter("threat_radius_scale", 1.1, descriptor);
  declare_parameter("avoidance_radius_scale", 1.1, descriptor);
  declare_parameter("os_radius", 0.3, descriptor);
  declare_parameter("point_extension_distance", 1.0, descriptor);
  declare_parameter("lateral_margin", 0.3, descriptor);
  declare_parameter("closing_segment_length", 999.0, descriptor);
  declare_parameter("max_request_position_delta", 3.0, descriptor);
  declare_parameter("global_frame", std::string("map"), descriptor);
  declare_parameter("robot_base_frame", std::string("base_link"), descriptor);
  declare_parameter("odom_topic", std::string("odom"), descriptor);
  declare_parameter("tracked_ship_topic", std::string("/tracked_ship"), descriptor);

  parameter_callback_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      std::lock_guard<std::mutex> lock(state_mutex_);
      for (const auto & parameter : parameters) {
        if (configured_ && std::find(
          configuration_parameters.begin(), configuration_parameters.end(),
          parameter.get_name()) != configuration_parameters.end())
        {
          result.successful = false;
          result.reason = "COLREGS parameters are configure-time: cleanup before changing " +
          parameter.get_name();
          break;
        }
      }
      return result;
    });

  rcl_jump_threshold_t threshold{};
  threshold.on_clock_change = true;
  threshold.min_backward.nanoseconds = -1;
  // Odd epochs denote a clock transition. No clock reads occur under state_mutex_,
  // avoiding inversion with Clock's jump-callback mutex.
  clock_jump_handler_ = get_clock()->create_jump_callback(
    [this]() {clock_epoch_.fetch_add(1);},
    [this](const rcl_time_jump_t &) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      clearObservationsLocked();
      clock_epoch_.fetch_add(1);
    }, threshold);
}

bool ColregsTsStateROS::loadAndValidateParameters()
{
  const auto & overrides = get_node_parameters_interface()->get_parameter_overrides();
  for (const auto & obsolete : obsolete_parameters) {
    if (overrides.count(obsolete.first) || has_parameter(obsolete.first)) {
      RCLCPP_ERROR(
        get_logger(), "Removed parameter '%s'; use '%s' instead",
        obsolete.first.c_str(), obsolete.second.c_str());
      return false;
    }
  }
  Configuration next;
  try {
    const double frequency = get_parameter("update_frequency").as_double();
    const double period_ns = 1e9 / frequency;
    const double transform_timeout = get_parameter("transform_timeout").as_double();
    using SteadyDuration = std::chrono::steady_clock::duration;
    const long double transform_ticks =
      std::chrono::duration<long double, SteadyDuration::period>(
      std::chrono::duration<long double>(transform_timeout)).count();
    if (!std::isfinite(transform_timeout) || transform_timeout < 0.0 ||
      !std::isfinite(transform_ticks) ||
      transform_ticks >= static_cast<long double>(std::numeric_limits<SteadyDuration::rep>::max()))
    {
      RCLCPP_ERROR(
        get_logger(), "transform_timeout must be finite, nonnegative and representable "
        "as a steady-clock duration");
      return false;
    }
    next.transform_timeout = std::chrono::duration_cast<SteadyDuration>(
      std::chrono::duration<long double>(transform_timeout));
    next.core.track_list_timeout = get_parameter("track_list_timeout").as_double();
    next.core.threat_tcpa_horizon = get_parameter("threat_tcpa_horizon").as_double();
    next.core.threat_radius_scale = get_parameter("threat_radius_scale").as_double();
    next.core.avoidance_radius_scale = get_parameter("avoidance_radius_scale").as_double();
    next.core.os_radius = get_parameter("os_radius").as_double();
    next.core.point_extension_distance = get_parameter("point_extension_distance").as_double();
    next.core.lateral_margin = get_parameter("lateral_margin").as_double();
    next.core.closing_segment_length = get_parameter("closing_segment_length").as_double();
    next.own_ship_state_timeout = get_parameter("own_ship_state_timeout").as_double();
    next.max_request_position_delta = get_parameter("max_request_position_delta").as_double();
    next.global_frame = get_parameter("global_frame").as_string();
    next.robot_base_frame = get_parameter("robot_base_frame").as_string();
    next.odom_topic = get_parameter("odom_topic").as_string();
    next.tracked_ship_topic = get_parameter("tracked_ship_topic").as_string();
    std::string reason;
    if (!validateCoreParams(next.core, reason)) {
      RCLCPP_ERROR(get_logger(), "Invalid COLREGS core parameters: %s", reason.c_str());
      return false;
    }
    if (!std::isfinite(frequency) || frequency <= 0.0 || !std::isfinite(period_ns) ||
      period_ns < 1.0 || period_ns >= static_cast<double>(std::numeric_limits<int64_t>::max()) ||
      !std::isfinite(next.own_ship_state_timeout) || next.own_ship_state_timeout <= 0.0 ||
      !std::isfinite(next.max_request_position_delta) || next.max_request_position_delta < 0.0 ||
      next.global_frame != "map" || !validFrame(next.robot_base_frame) ||
      next.odom_topic.empty() || next.tracked_ship_topic.empty())
    {
      RCLCPP_ERROR(
        get_logger(), "Invalid COLREGS ROS parameters: require representable positive timer "
        "period, positive state timeout, nonnegative request delta, global_frame=map, "
        "valid base frame and nonempty topics");
      return false;
    }
    // ROS name validation must precede TF threads and subscription creation too.
    (void)rclcpp::expand_topic_or_service_name(
      next.odom_topic, get_name(), get_namespace(), false);
    (void)rclcpp::expand_topic_or_service_name(
      next.tracked_ship_topic, get_name(), get_namespace(), false);
    next.timer_period = std::chrono::nanoseconds(static_cast<int64_t>(period_ns));
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Invalid COLREGS parameter value/type: %s", ex.what());
    return false;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  config_ = next;
  core_params_ = next.core;
  return true;
}

nav2_util::CallbackReturn ColregsTsStateROS::on_configure(const rclcpp_lifecycle::State &)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    configured_ = true;  // Freeze parameters before reading the whole group.
  }
  if (!loadAndValidateParameters()) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    configured_ = false;
    return nav2_util::CallbackReturn::FAILURE;
  }
  try {
    auto resources = std::make_shared<TfResources>();
    resources->buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    resources->buffer->setUsingDedicatedThread(true);
    resources->listener = std::make_shared<tf2_ros::TransformListener>(*resources->buffer, true);
    uint64_t generation;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      generation = ++generation_;
    }
    auto ts_sub = create_subscription<nav2_colregs_msgs::msg::TrackedShipList>(
      config_.tracked_ship_topic, rclcpp::SystemDefaultsQoS(),
      [this, generation](nav2_colregs_msgs::msg::TrackedShipList::ConstSharedPtr msg) {
        trackedShipCallback(msg, generation);
      });
    auto odom_sub = create_subscription<nav_msgs::msg::Odometry>(
      config_.odom_topic, rclcpp::SystemDefaultsQoS(),
      [this, generation](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        odomCallback(msg, generation);
      });
    auto publisher = create_publisher<visualization_msgs::msg::MarkerArray>("cpa_markers", 10);
    std::lock_guard<std::mutex> lock(state_mutex_);
    clearObservationsLocked();
    tf_resources_ = std::move(resources);
    ts_sub_ = std::move(ts_sub);
    odom_sub_ = std::move(odom_sub);
    cpa_markers_pub_ = std::move(publisher);
    accepting_inputs_ = true;
  } catch (const std::exception & ex) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    configured_ = false;
    RCLCPP_ERROR(get_logger(), "COLREGS configure failed: %s", ex.what());
    return nav2_util::CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(
    get_logger(),
    "COLREGS TS configured: frame=%s base=%s tracks=%s odom=%s "
    "track_list_timeout=%.3f own_ship_state_timeout=%.3f transform_timeout=%.3f "
    "threat_tcpa_horizon=%.3f threat_radius_scale=%.3f avoidance_radius_scale=%.3f "
    "os_radius=%.3f point_extension_distance=%.3f lateral_margin=%.3f "
    "closing_segment_length=%.3f max_request_position_delta=%.3f",
    config_.global_frame.c_str(), config_.robot_base_frame.c_str(),
    config_.tracked_ship_topic.c_str(), config_.odom_topic.c_str(),
    config_.core.track_list_timeout, config_.own_ship_state_timeout,
    std::chrono::duration<double>(config_.transform_timeout).count(),
    config_.core.threat_tcpa_horizon, config_.core.threat_radius_scale,
    config_.core.avoidance_radius_scale, config_.core.os_radius,
    config_.core.point_extension_distance, config_.core.lateral_margin,
    config_.core.closing_segment_length, config_.max_request_position_delta);
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ColregsTsStateROS::on_activate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  cpa_markers_pub_->on_activate();
  const auto generation = generation_;
  timer_ =
    create_wall_timer(config_.timer_period, [this, generation]() {timerCallback(generation);});
  active_ = true;
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ColregsTsStateROS::on_deactivate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  active_ = false;
  timer_.reset();
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  cpa_markers_pub_->publish(markers);
  cpa_markers_pub_->on_deactivate();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ColregsTsStateROS::on_cleanup(const rclcpp_lifecycle::State &)
{
  std::shared_ptr<TfResources> resources;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    active_ = false;
    accepting_inputs_ = false;
    ++generation_;
    timer_.reset();
    ts_sub_.reset();
    odom_sub_.reset();
    cpa_markers_pub_.reset();
    resources = std::move(tf_resources_);
    clearObservationsLocked();
    configured_ = false;
  }
  // Listener teardown may join a thread; never do so holding the cache mutex.
  resources.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

void ColregsTsStateROS::clearObservationsLocked()
{
  last_tracks_.reset();
  last_odom_.reset();
  track_times_ = ObservationTimes{};
  odom_times_ = ObservationTimes{};
}

void ColregsTsStateROS::recordReceiptLocked(
  const builtin_interfaces::msg::Time & stamp, const rclcpp::Time & receipt,
  double timeout, ObservationTimes & times)
{
  times.receipt = receipt;
  times.status = InputStatus::INVALID_STATE;
  times.reason = "Malformed measurement timestamp";
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000u) {return;}
  const rclcpp::Time measurement(stamp, receipt.get_clock_type());
  if ((measurement - receipt).seconds() > timeout) {
    times.reason = "Measurement timestamp exceeds the allowed pending window at receipt";
    return;
  }
  if (measurement.nanoseconds() < times.newest_stamp) {
    times.reason = "Out-of-order measurement timestamp (older than stream high-water mark)";
    return;
  }
  times.newest_stamp = measurement.nanoseconds();
  if (measurement > receipt) {
    // /clock and measurements travel on independent DDS channels. Admit a
    // bounded lead without treating it as current: queries remain invalid until
    // the clock catches up, and the original receipt must still be fresh then.
    times.status = InputStatus::VALID;
    times.reason = "Pending measurement: waiting for calculation clock to catch up";
    return;
  }
  times.status = freshness(measurement, receipt, timeout);
  times.reason = times.status == InputStatus::VALID ? "" : "Measurement stale at receipt";
}

void ColregsTsStateROS::trackedShipCallback(
  nav2_colregs_msgs::msg::TrackedShipList::ConstSharedPtr msg, uint64_t generation)
{
  const auto epoch = clock_epoch_.load();
  const auto receipt = now();
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!accepting_inputs_ || generation != generation_ || epoch % 2 != 0 ||
    epoch != clock_epoch_.load()) {return;}
  // Replace even invalid, empty and older lists; never leave an old target alive
  // after a bad packet. The high-water stamp prevents rollback within an epoch.
  last_tracks_ = std::move(msg);
  recordReceiptLocked(
    last_tracks_->header.stamp, receipt, config_.core.track_list_timeout, track_times_);
}

void ColregsTsStateROS::odomCallback(
  nav_msgs::msg::Odometry::ConstSharedPtr msg, uint64_t generation)
{
  const auto epoch = clock_epoch_.load();
  const auto receipt = now();
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!accepting_inputs_ || generation != generation_ || epoch % 2 != 0 ||
    epoch != clock_epoch_.load()) {return;}
  last_odom_ = std::move(msg);
  recordReceiptLocked(
    last_odom_->header.stamp, receipt, config_.own_ship_state_timeout, odom_times_);
}

ColregsTsStateROS::PlanningInput ColregsTsStateROS::getPlanningInput(
  double os_x, double os_y, std::chrono::steady_clock::time_point deadline)
{
  return collectInput(true, os_x, os_y, deadline);
}

bool ColregsTsStateROS::isPlanningInputCurrent(const PlanningInput & input) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto epoch = clock_epoch_.load();
  return accepting_inputs_ && input.lifecycle_generation == generation_ &&
         input.clock_epoch == epoch && epoch % 2 == 0;
}

ColregsTsStateROS::PlanningInput ColregsTsStateROS::collectInput(
  bool request_anchor, double os_x, double os_y,
  std::chrono::steady_clock::time_point deadline)
{
  PlanningInput input;
  input.ts.frame_id = "map";
  bool calculation_time_selected = false;
  auto fail = [this, &input, &calculation_time_selected](
    InputStatus status, const std::string & reason) {
      // Early failures also carry a ROS-clock stamp; this lambda is never
      // called while state_mutex_ is held.
      if (!calculation_time_selected) {input.ts.stamp = now();}
      input.ts.status = status;
      input.ts.reason = reason;
      input.ts.ships.clear();
      input.os.velocity_valid = false;
      return input;
    };
  nav2_colregs_msgs::msg::TrackedShipList::ConstSharedPtr tracks;
  nav_msgs::msg::Odometry::ConstSharedPtr odom;
  ObservationTimes track_times, odom_times;
  Configuration config;
  std::shared_ptr<TfResources> resources;
  bool accepting_inputs;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // Capture provenance even when observations are missing or invalid. A
    // default-constructed (generation=0) input is not a configured-node token.
    input.lifecycle_generation = generation_;
    input.clock_epoch = clock_epoch_.load();
    tracks = last_tracks_;
    odom = last_odom_;
    track_times = track_times_;
    odom_times = odom_times_;
    config = config_;
    input.params = config.core;
    resources = tf_resources_;
    accepting_inputs = accepting_inputs_;
  }
  if (!accepting_inputs) {return fail(InputStatus::NO_DATA, "TS state is not configured");}
  if (input.clock_epoch % 2 != 0) {
    return fail(InputStatus::INVALID_STATE, "Clock epoch changed during observation copy");
  }
  if (request_anchor && (!std::isfinite(os_x) || !std::isfinite(os_y))) {
    return fail(InputStatus::INVALID_REQUEST, "Nonfinite request start");
  }
  if (!tracks || !odom) {
    return fail(InputStatus::NO_DATA, !tracks ? "No complete tracked-ship list" : "No odometry");
  }
  if (track_times.status != InputStatus::VALID) {
    return fail(track_times.status, "Tracked ships: " + track_times.reason);
  }
  if (odom_times.status != InputStatus::VALID) {
    return fail(odom_times.status, "Odometry: " + odom_times.reason);
  }
  const rclcpp::Time track_stamp(tracks->header.stamp, track_times.receipt.get_clock_type());
  const rclcpp::Time odom_stamp(odom->header.stamp, odom_times.receipt.get_clock_type());
  if (!validFrame(tracks->header.frame_id)) {
    return fail(InputStatus::INVALID_STATE, "Malformed tracked-ship list frame");
  }
  if (!validFrame(odom->header.frame_id) || !validFrame(odom->child_frame_id)) {
    return fail(InputStatus::INVALID_STATE, "Malformed odometry header/child frame");
  }
  if (!validPose(odom->pose.pose) || !validTwist(odom->twist.twist) ||
    !std::all_of(
      odom->pose.covariance.begin(), odom->pose.covariance.end(),
      [](double value) {return std::isfinite(value);}) ||
    !std::all_of(
      odom->twist.covariance.begin(), odom->twist.covariance.end(),
      [](double value) {return std::isfinite(value);}))
  {
    return fail(InputStatus::INVALID_STATE, "Malformed odometry pose, twist or covariance");
  }
  std::unordered_set<std::string> ids;
  for (const auto & ship : tracks->ships) {
    const auto id = uuidToString(ship.target_id.uuid.data());
    if (!ids.insert(id).second) {
      return fail(InputStatus::INVALID_STATE, "Duplicate target UUID: " + id);
    }
    if (!validPose(ship.pose) || !validTwist(ship.twist) ||
      !std::isfinite(ship.radius) || ship.radius < 0.0)
    {
      return fail(InputStatus::INVALID_STATE, "Malformed target pose, twist or radius: " + id);
    }
  }
  if (!resources) {return fail(InputStatus::INVALID_STATE, "TF buffer unavailable");}
  using SteadyClock = std::chrono::steady_clock;
  const auto wait_start = SteadyClock::now();
  const auto timeout = request_anchor ? config.transform_timeout : SteadyClock::duration::zero();
  // Saturate addition for large but representable durations before applying the
  // request-wide deadline. One deadline is shared by OS, TS and live-base TF.
  const auto timeout_deadline = wait_start > SteadyClock::time_point::max() - timeout ?
    SteadyClock::time_point::max() : wait_start + timeout;
  const auto tf_deadline = std::min(deadline, timeout_deadline);
  const auto wait_for_tf = [this, &input, tf_deadline](const auto & lookup) {
      for (;; ) {
        if (!isPlanningInputCurrent(input)) {
          throw tf2::TransformException("Lifecycle/clock epoch changed while waiting for TF");
        }
        try {
          return lookup();  // Always a zero-timeout query; never use TF2's ROS-clock wait.
        } catch (const tf2::TransformException &) {
          const auto current = SteadyClock::now();
          if (current >= tf_deadline) {throw;}
          // The dedicated TF listener can populate the buffer while this thread
          // sleeps. No state mutex is held, including when /clock is paused.
          std::this_thread::sleep_until(
            std::min(tf_deadline, current + std::chrono::milliseconds(2)));
          if (SteadyClock::now() >= tf_deadline) {throw;}
        }
      }
    };
  try {
    const auto velocity_tf = wait_for_tf(
      [&]() {
        return measurementTransform(
          *resources->buffer, config.global_frame, odom->child_frame_id, odom_stamp);
      });
    // Check the list frame even for an empty scene. Retain all required TFs
    // before sampling the calculation clock, so newly arrived receipts/TFs
    // cannot be compared against an earlier time sampled before the cache copy.
    const auto track_tf = wait_for_tf(
      [&]() {
        return measurementTransform(
          *resources->buffer, config.global_frame, tracks->header.frame_id, track_stamp);
      });
    const auto live_tf = wait_for_tf(
      [&]() {
        return resources->buffer->lookupTransform(
          config.global_frame, config.robot_base_frame, tf2::TimePointZero);
      });
    const auto stamp = now();
    input.ts.stamp = stamp;
    calculation_time_selected = true;
    const std::vector<std::pair<rclcpp::Time, double>> times = {
      {track_stamp, config.core.track_list_timeout},
      {track_times.receipt, config.core.track_list_timeout},
      {odom_stamp, config.own_ship_state_timeout},
      {odom_times.receipt, config.own_ship_state_timeout}};
    for (const auto & time : times) {
      const auto status = freshness(time.first, stamp, time.second);
      if (status != InputStatus::VALID) {
        return fail(
          status, "Measurement or receipt is stale/future at calculation time "
          "(bounded pending measurements require the clock to catch up)");
      }
    }
    const auto & linear = odom->twist.twist.linear;
    const auto velocity = velocity_tf.getBasis() * tf2::Vector3(linear.x, linear.y, linear.z);
    input.os.vx = velocity.x();
    input.os.vy = velocity.y();
    if (!finiteVector(live_tf.transform.translation) ||
      !validQuaternion(live_tf.transform.rotation))
    {
      return fail(InputStatus::INVALID_STATE, "Malformed live base TF");
    }
    const rclcpp::Time live_stamp(live_tf.header.stamp, stamp.get_clock_type());
    double pose_age = 0.0;
    if (live_stamp.nanoseconds() != 0) {
      const auto status = freshness(live_stamp, stamp, config.own_ship_state_timeout);
      if (status != InputStatus::VALID) {
        return fail(status, "Live base TF is stale/future");
      }
      pose_age = (stamp - live_stamp).seconds();
    }
    const double live_x = live_tf.transform.translation.x + input.os.vx * pose_age;
    const double live_y = live_tf.transform.translation.y + input.os.vy * pose_age;
    if (!std::isfinite(live_x) || !std::isfinite(live_y) || !finiteVector(velocity)) {
      return fail(InputStatus::INVALID_STATE, "Nonfinite extrapolated OS state");
    }
    if (request_anchor &&
      std::hypot(os_x - live_x, os_y - live_y) > config.max_request_position_delta)
    {
      return fail(InputStatus::INVALID_REQUEST, "Request start exceeds max_request_position_delta");
    }
    input.os.x = request_anchor ? os_x : live_x;
    input.os.y = request_anchor ? os_y : live_y;
    input.os.velocity_valid = true;

    // Velocity rotates by the retained observation-frame TF, never ship yaw.
    const double age = (stamp - track_stamp).seconds();
    input.ts.ships.reserve(tracks->ships.size());
    for (const auto & ship : tracks->ships) {
      const auto & p = ship.pose.position;
      const auto & v = ship.twist.linear;
      const auto position = track_tf * tf2::Vector3(p.x, p.y, p.z);
      const auto velocity = track_tf.getBasis() * tf2::Vector3(v.x, v.y, v.z);
      RawTsEntry entry;
      entry.target_id = uuidToString(ship.target_id.uuid.data());
      entry.x = position.x() + velocity.x() * age;
      entry.y = position.y() + velocity.y() * age;
      entry.vx = velocity.x();
      entry.vy = velocity.y();
      entry.radius = ship.radius;
      entry.last_seen = track_stamp;  // Preserve measurement age; core must not extrapolate again.
      if (!finiteVector(position) || !finiteVector(velocity) ||
        !std::isfinite(entry.x) || !std::isfinite(entry.y))
      {
        return fail(InputStatus::INVALID_STATE, "Nonfinite transformed/extrapolated target state");
      }
      input.ts.ships.push_back(entry);
    }
  } catch (const tf2::TransformException & ex) {
    return fail(InputStatus::INVALID_STATE, std::string("Required TF unavailable: ") + ex.what());
  } catch (const std::exception & ex) {
    return fail(InputStatus::INVALID_STATE, std::string("Malformed stamped input: ") + ex.what());
  }
  std::sort(
    input.ts.ships.begin(), input.ts.ships.end(), [](const auto & a, const auto & b) {
      return a.target_id < b.target_id;
    });
  if (!isPlanningInputCurrent(input)) {
    return fail(InputStatus::INVALID_STATE, "Lifecycle/clock epoch changed during TF evaluation");
  }
  input.ts.status = InputStatus::VALID;
  input.ts.reason = "Fresh complete observations at common calculation time";
  return input;
}

void ColregsTsStateROS::timerCallback(uint64_t generation)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!active_ || generation != generation_) {return;}
  }
  // Diagnostics use the live pose, never an artificial planning-start anchor.
  const auto input = collectInput(false, 0.0, 0.0, std::chrono::steady_clock::time_point::min());
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  if (input.ts.status == InputStatus::VALID) {
    const auto snapshot = processTs(input.ts, input.os, input.params);
    if (snapshot.status == InputStatus::VALID) {
      for (const auto & ts : snapshot.ships) {
        const double prediction = std::isfinite(ts.tcpa) ?
          std::max(0.0, ts.tcpa) : 0.0;
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = input.ts.stamp;
        marker.header.frame_id = input.ts.frame_id;
        // UUID namespaces keep (namespace,id) stable across reorder/removal.
        marker.ns = "cpa/" + ts.target_id;
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = ts.x + ts.vx * prediction;
        marker.pose.position.y = ts.y + ts.vy * prediction;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3;
        marker.color.a = 0.8;
        marker.color.r = 1.0;
        marker.color.g = ts.has_threat ? 0.2 : 0.6;
        marker.color.b = 0.2;
        marker.lifetime.nanosec = 500000000;
        if (finiteVector(marker.pose.position)) {markers.markers.push_back(marker);}
      }
    }
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (active_ && generation == generation_ && cpa_markers_pub_) {
    const auto epoch = clock_epoch_.load();
    if (input.lifecycle_generation != generation_ || input.clock_epoch != epoch || epoch % 2 != 0) {
      markers.markers.resize(1);
    }
    cpa_markers_pub_->publish(markers);
  }
}

}  // namespace nav2_colregs_ts_manager
