#include <string>
#include <memory>

#include "nav2_colregs_local_path_behavior/plugins/create_local_path_action.hpp"

namespace nav2_behavior_tree
{

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

CreateLocalPathAction::CreateLocalPathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_colregs_local_path_behavior::action::CreateLocalPath>(xml_tag_name, action_name, conf)
{
  // The base class (BtActionNode) handles creation of the ROS 2 action client
  // and subscription to the blackboard.  No additional per-instance setup is
  // required.
}

// ---------------------------------------------------------------------------
// Input-port initialisation
// ---------------------------------------------------------------------------

void CreateLocalPathAction::initialize()
{
  // Read the current global path from the BT blackboard.
  // This path is produced upstream by ComputePathToPose and stored under
  // the blackboard key "path".
  nav_msgs::msg::Path path;
  getInput("path", path);

  // Transfer the path to the action goal for delivery to the CreateLocalPath
  // action server inside behavior_server.
  goal_.path = path;
}

// ---------------------------------------------------------------------------
// BT lifecycle callbacks
// ---------------------------------------------------------------------------

void CreateLocalPathAction::on_tick()
{
  // The BT may call tick() several times before the action goal is sent.
  // We only need to populate the goal once, when the node becomes active.
  if (!BT::isStatusActive(status())) {
    initialize();
  }
}

BT::NodeStatus CreateLocalPathAction::on_success()
{
  // The action server completed normally.  Clear any residual error
  // outputs and signal BT success so control can continue downstream
  // (e.g. FollowPath).
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CreateLocalPathAction::on_aborted()
{
  // The action server rejected or failed to process the goal.
  // Forward the server's error information to the BT blackboard so
  // condition nodes (e.g. WouldAXRecoveryHelp) can decide on recovery.
  setOutput("error_code_id", result_.result->error_code);
  setOutput("error_msg", result_.result->error_msg);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus CreateLocalPathAction::on_cancelled()
{
  // Cancellation is handled gracefully — reset outputs and succeed.
  // This prevents a preemption from cascading into a tree failure.
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

void CreateLocalPathAction::on_timeout()
{
  // The action client could not reach the server within the configured
  // server_timeout.  Provide a human-readable message for diagnostics.
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "Behavior Tree action client timed out waiting.");
}

}  // namespace nav2_behavior_tree

// ---------------------------------------------------------------------------
// BT factory registration
// ---------------------------------------------------------------------------
#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      // The second argument ("create_local_path") must match the behavior
      // plugin ID declared in behavior_server's behavior_plugins list.
      return std::make_unique<nav2_behavior_tree::CreateLocalPathAction>(
        name, "create_local_path", config);
    };

  // "CreateLocalPath" is the XML tag that will be used in behavior tree files.
  factory.registerBuilder<nav2_behavior_tree::CreateLocalPathAction>(
    "CreateLocalPath", builder);
}
