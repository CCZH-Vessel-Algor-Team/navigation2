#ifndef NAV2_COLREGS_LOCAL_PATH_BT_NODES__CREATE_LOCAL_PATH_ACTION_HPP_
#define NAV2_COLREGS_LOCAL_PATH_BT_NODES__CREATE_LOCAL_PATH_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_colregs_local_path_behavior/action/create_local_path.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_behavior_tree
{

class CreateLocalPathAction : public BtActionNode<nav2_colregs_local_path_behavior::action::CreateLocalPath>
{
  using Action = nav2_colregs_local_path_behavior::action::CreateLocalPath;
  using ActionResult = Action::Result;

public:
  CreateLocalPathAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;
  void on_timeout();

  void initialize();

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<nav_msgs::msg::Path>("path", "Global path from ComputePathToPose"),
        BT::OutputPort<nav_msgs::msg::Path>("local_path", "Local path passed through to FollowPath"),
        BT::OutputPort<uint16_t>("error_code_id", "The create_local_path error code"),
        BT::OutputPort<std::string>("error_msg", "The create_local_path error msg"),
      });
  }

private:
  nav_msgs::msg::Path path_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_COLREGS_LOCAL_PATH_BT_NODES__CREATE_LOCAL_PATH_ACTION_HPP_
