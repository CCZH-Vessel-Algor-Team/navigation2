#include <memory>
#include <string>

#include "nav2_colregs_local_path_bt_nodes/compute_local_path_action.hpp"

namespace nav2_behavior_tree
{

ComputeLocalPathAction::ComputeLocalPathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_colregs_msgs::action::ComputeLocalPath>(xml_tag_name, action_name, conf)
{
}

void ComputeLocalPathAction::on_tick()
{
  if (!getInput("reference_path", goal_.reference_path)) {
    throw BT::RuntimeError("missing required input [reference_path]");
  }
}

BT::NodeStatus ComputeLocalPathAction::on_success()
{
  if (!result_.result) {
    setOutput("error_code_id", ActionResult::INVALID_PATH);
    setOutput("error_msg", "ComputeLocalPath action returned a null result");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("local_path", result_.result->local_path);
  setOutput("error_code_id", result_.result->error_code);
  setOutput("error_msg", result_.result->error_msg);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputeLocalPathAction::on_aborted()
{
  if (!result_.result) {
    setOutput("error_code_id", ActionResult::INVALID_PATH);
    setOutput("error_msg", "ComputeLocalPath action aborted without a result");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("error_code_id", result_.result->error_code);
  setOutput("error_msg", result_.result->error_msg);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputeLocalPathAction::on_cancelled()
{
  if (!result_.result) {
    setOutput("error_code_id", ActionResult::CANCELED);
    setOutput("error_msg", "Local path computation canceled");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("error_code_id", result_.result->error_code);
  setOutput("error_msg", result_.result->error_msg);
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ComputeLocalPathAction>(
        name, "compute_local_path", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ComputeLocalPathAction>(
    "ComputeLocalPath", builder);
}
