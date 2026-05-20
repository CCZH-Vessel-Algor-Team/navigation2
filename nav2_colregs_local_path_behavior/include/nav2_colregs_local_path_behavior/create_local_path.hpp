// Copyright (c) 2020 Samsung Research America
// This code is licensed under MIT license (see LICENSE.txt for details)

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

class CreateLocalPath : public TimedBehavior<Action>
{
public:
  CreateLocalPath();
  ~CreateLocalPath();

  ResultStatus onRun(const std::shared_ptr<const Action::Goal> command) override;

  ResultStatus onCycleUpdate() override;

  void onConfigure() override;

  /**
   * @brief Method to determine the required costmap info
   * @return costmap resources needed
   */
  nav2_core::CostmapInfoType getResourceInfo() override {return nav2_core::CostmapInfoType::NONE;}

protected:
};

}  // namespace nav2_colregs_local_path_behavior

#endif  // NAV2_COLREGS_LOCAL_PATH_HPP_
