#ifndef NAV2_COLREGS_VO_SKELETON_PLANNER__PARAMETER_CONTRACT_HPP_
#define NAV2_COLREGS_VO_SKELETON_PLANNER__PARAMETER_CONTRACT_HPP_

#include <limits>
#include <string>
#include <tuple>
#include <vector>
#include "nav2_util/node_utils.hpp"

namespace nav2_colregs_vo_skeleton_planner
{
template<typename NodeT>
void declareSkeletonParameters(const NodeT & node, const std::string & name)
{
  using Value = rclcpp::ParameterValue;
  const std::vector<std::tuple<std::string, Value, std::string>> defaults = {
    {"step_size", Value(4.0), "Maximum tree extension [m]."},
    {"goal_bias", Value(0.1), "Query/start sampling probability [0,1] in the goal-rooted tree."},
    {"eta", Value(50.0), "Rewire-radius coefficient [m], capped at 15m."},
    {"node_limit", Value(1024), "Tree node capacity."},
    {"path_limit", Value(256), "Raw skeleton path point limit before interpolation."},
    {"near_limit", Value(16), "Parent/rewire candidate limit."},
    {"connector_limit", Value(128), "Connector/shortcut candidate limit."},
    {"recovery_near_limit", Value(16), "Near-recovery candidate limit."},
    {"global_iterations", Value(2400), "Bootstrap/global search attempts."},
    {"local_iterations", Value(600), "Recovery attempts when reuse_iterations is zero."},
    {"refine_iterations", Value(200), "Refinement allowance within unseeded search budget."},
    {"max_work", Value(12000000), "Positive int64 query work credits (not milliseconds)."},
    {"time_limit", Value(0.0), "Core replan deadline [s]; zero disables it."},
    {"goal_tolerance", Value(2.0), "Checked direct-to-exact-goal fast-path distance [m]."},
    {"allow_recovery", Value(true), "Allow recovery from inaccessible anchors."},
    {"allow_skip", Value(true), "Allow skeleton anchor skipping."},
    {"reuse_iterations", Value(64), "Cross-query search attempts; zero changes reuse mode."},
    {"switch_margin", Value(0.03), "Relative replacement improvement [0,1)."},
    {"prune_period", Value(10), "Deprecated and ignored: maintenance runs per query."},
    {"safety_dist", Value(1.5), "Costmap safety-disk radius [m]."},
    {"cost_weight", Value(0.3), "Nonnegative normalized costmap cost multiplier."}
  };
  for (const auto & entry : defaults) {
    const auto & key = std::get<0>(entry);
    const auto full_name = name + "." + key;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = std::get<2>(entry);
    descriptor.additional_constraints = "Startup-only; restart planner_server to change.";
    nav2_util::declare_parameter_if_not_declared(
      node, full_name, std::get<1>(entry), descriptor);
    const auto value = node->get_parameter(full_name);
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER && key != "max_work") {
      if (value.as_int() < std::numeric_limits<int>::min() ||
        value.as_int() > std::numeric_limits<int>::max())
      {
        throw std::invalid_argument(full_name + " exceeds the supported integer range");
      }
    }
  }
  RCLCPP_WARN(node->get_logger(), "%s.prune_period is deprecated and has no effect", name.c_str());
}
}  // namespace nav2_colregs_vo_skeleton_planner
#endif  // NAV2_COLREGS_VO_SKELETON_PLANNER__PARAMETER_CONTRACT_HPP_
