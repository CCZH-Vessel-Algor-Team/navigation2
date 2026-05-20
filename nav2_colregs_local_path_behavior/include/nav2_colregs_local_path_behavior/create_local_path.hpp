#ifndef NAV2_COLREGS_LOCAL_PATH__CREATE_LOCAL_PATH_HPP_
#define NAV2_COLREGS_LOCAL_PATH__CREATE_LOCAL_PATH_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_colregs_local_path_behavior/action/create_local_path.hpp"

namespace nav2_colregs_local_path_behavior
{

using namespace nav2_behaviors;  // NOLINT
using Action = nav2_colregs_local_path_behavior::action::CreateLocalPath;
using ActionResult = Action::Result;

/**
 * @class CreateLocalPath
 * @brief Phase-1 behavior plugin that receives a global Nav2 path, inspects it,
 *        and returns immediately.
 *
 * This class extends TimedBehavior<CreateLocalPath> and is loaded by the
 * BehaviorServer via pluginlib. In Phase 1 it performs a non-mutating read
 * of the incoming path: it logs the pose count and the coordinates of the
 * first and last waypoints, then finishes.
 *
 * Resource requirements: none (getResourceInfo returns NONE).
 */
class CreateLocalPath : public TimedBehavior<Action>
{
public:
  /**
   * @brief Default constructor. Delegates to TimedBehavior<Action>.
   */
  CreateLocalPath();

  /**
   * @brief Default destructor.
   */
  ~CreateLocalPath();

  /**
   * @brief One-shot entry point called when the action server receives a goal.
   *
   * Reads the global path attached to the goal, validates that it is non-empty,
   * logs the number of poses and the first/last pose coordinates, then returns
   * SUCCEEDED.  An empty path is treated as a failure.
   *
   * @param command Shared pointer to the incoming action goal (contains the path).
   * @return ResultStatus with status SUCCEEDED or FAILED.
   */
  ResultStatus onRun(const std::shared_ptr<const Action::Goal> command) override;

  /**
   * @brief Periodic update callback (unused in this one-shot behavior).
   *
   * Because this is a fire-and-forget behavior the cycle update returns
   * SUCCEEDED immediately.  Downstream behaviours that require iterative
   * work should override this method to publish cmd_vel and return RUNNING
   * until completion.
   *
   * @return ResultStatus always SUCCEEDED.
   */
  ResultStatus onCycleUpdate() override;

  /**
   * @brief Lifecycle configuration hook (currently a no-op).
   *
   * Called once during BehaviorServer::on_configure().  Future parameters
   * (e.g. logging verbosity, prediction horizon for projection) can be
   * declared and read here.
   */
  void onConfigure() override;

  /**
   * @brief Declare the costmap resources this behavior requires.
   *
   * @return CostmapInfoType::NONE — this behavior does not interact with
   *         any costmap layers.
   */
  nav2_core::CostmapInfoType getResourceInfo() override
  {
    return nav2_core::CostmapInfoType::NONE;
  }

protected:
  // Future per-instance state (e.g. cached projection) can be stored here.
};

}  // namespace nav2_colregs_local_path_behavior

#endif  // NAV2_COLREGS_LOCAL_PATH__CREATE_LOCAL_PATH_HPP_
