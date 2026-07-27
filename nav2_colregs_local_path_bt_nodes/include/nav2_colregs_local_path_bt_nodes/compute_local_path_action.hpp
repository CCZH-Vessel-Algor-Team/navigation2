#ifndef NAV2_COLREGS_LOCAL_PATH_BT_NODES__COMPUTE_LOCAL_PATH_ACTION_HPP_
#define NAV2_COLREGS_LOCAL_PATH_BT_NODES__COMPUTE_LOCAL_PATH_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_colregs_msgs/action/compute_local_path.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_behavior_tree
{

class ComputeLocalPathAction : public BtActionNode<nav2_colregs_msgs::action::ComputeLocalPath>
{
  using Action = nav2_colregs_msgs::action::ComputeLocalPath;
  using ActionResult = Action::Result;

public:
  ComputeLocalPathAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<nav_msgs::msg::Path>(
          "reference_path", "Reference path for local-path computation"),
        BT::OutputPort<nav_msgs::msg::Path>(
          "local_path", "Local path returned by the local planner"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "The compute_local_path error code"),
        BT::OutputPort<std::string>("error_msg", "The compute_local_path error message"),
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_COLREGS_LOCAL_PATH_BT_NODES__COMPUTE_LOCAL_PATH_ACTION_HPP_
