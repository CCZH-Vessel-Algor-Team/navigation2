#include <string>
#include <memory>

#include "nav2_colregs_local_path_bt_nodes/create_local_path_action.hpp"

namespace nav2_behavior_tree
{

CreateLocalPathAction::CreateLocalPathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_colregs_local_path_behavior::action::CreateLocalPath>(xml_tag_name, action_name, conf)
{
}

void CreateLocalPathAction::initialize()
{
  nav_msgs::msg::Path path;
  getInput("path", path);
  goal_.path = path;
  path_ = path;
}

void CreateLocalPathAction::on_tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }
}

BT::NodeStatus CreateLocalPathAction::on_success()
{
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  setOutput("local_path", path_);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CreateLocalPathAction::on_aborted()
{
  setOutput("error_code_id", result_.result->error_code);
  setOutput("error_msg", result_.result->error_msg);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus CreateLocalPathAction::on_cancelled()
{
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

void CreateLocalPathAction::on_timeout()
{
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "Behavior Tree action client timed out waiting.");
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::CreateLocalPathAction>(
        name, "create_local_path", config);
    };

  factory.registerBuilder<nav2_behavior_tree::CreateLocalPathAction>(
    "CreateLocalPath", builder);
}
