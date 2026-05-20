#ifndef NAV2_COLREGS_LOCAL_PATH__PLUGINS__CREATE_LOCAL_PATH_ACTION_HPP_
#define NAV2_COLREGS_LOCAL_PATH__PLUGINS__CREATE_LOCAL_PATH_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_colregs_local_path_behavior/action/create_local_path.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_behavior_tree
{

/**
 * @class CreateLocalPathAction
 * @brief Behavior Tree action node that bridges the BT blackboard to the
 *        CreateLocalPath ROS 2 action server.
 *
 * This node reads the global path produced by ComputePathToPose from the
 * BT blackboard (port "path"), packages it into an action goal, and sends
 * it to the CreateLocalPath action server hosted inside behavior_server.
 * The node maps action results (SUCCEEDED / ABORTED / CANCELED / TIMEOUT)
 * back to BT::NodeStatus values.
 *
 * Usage in BT XML:
 * @code
 * <CreateLocalPath path="{path}" error_code_id="{...}" error_msg="{...}"/>
 * @endcode
 */
class CreateLocalPathAction : public BtActionNode<nav2_colregs_local_path_behavior::action::CreateLocalPath>
{
  using Action = nav2_colregs_local_path_behavior::action::CreateLocalPath;
  using ActionResult = Action::Result;

public:
  /**
   * @brief Construct the BT action node.
   *
   * @param xml_tag_name  XML tag used in the behavior tree.
   * @param action_name   ROS 2 action name (must match behavior_plugins ID).
   * @param conf          BT::NodeConfiguration from the BT factory.
   */
  CreateLocalPathAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Called once per BT tick before the goal is sent.
   *
   * Lazily initializes the goal on first active tick by reading the "path"
   * input port from the blackboard.
   */
  void on_tick() override;

  /**
   * @brief Called when the action server returns a successful result.
   *
   * Sets the "error_code_id" and "error_msg" output ports to NONE / empty,
   * and returns BT::NodeStatus::SUCCESS.
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief Called when the action server aborts the goal.
   *
   * Forwards the server-reported error_code and error_msg to the output
   * ports and returns BT::NodeStatus::FAILURE so the BT can trigger recovery.
   */
  BT::NodeStatus on_aborted() override;

  /**
   * @brief Called when the action goal is cancelled (e.g. preempted).
   *
   * Resets the error outputs and returns BT::NodeStatus::SUCCESS so the
   * cancellation does not propagate as a tree failure.
   */
  BT::NodeStatus on_cancelled() override;

  /**
   * @brief Called when the action client times out waiting for a server
   *        response.
   *
   * Sets a descriptive error message on the "error_msg" output port.
   */
  void on_timeout();

  /**
   * @brief Read the "path" input port and populate the action goal.
   *
   * Called by on_tick() when the node transitions from inactive to active.
   */
  void initialize();

  /**
   * @brief Declare the BT ports (inputs and outputs) for this node.
   *
   * Inherits "server_name" and "server_timeout" from BtActionNode;
   * adds the domain-specific "path" input and error-code outputs.
   *
   * @return BT::PortsList containing both basic and node-specific ports.
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<nav_msgs::msg::Path>("path", "Global path from ComputePathToPose"),
        BT::OutputPort<uint16_t>("error_code_id", "The create_local_path error code"),
        BT::OutputPort<std::string>("error_msg", "The create_local_path error msg"),
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_COLREGS_LOCAL_PATH__PLUGINS__CREATE_LOCAL_PATH_ACTION_HPP_
