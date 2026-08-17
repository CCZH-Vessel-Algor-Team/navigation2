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

#ifndef NAV2_COLREGS_LOCAL_PLANNER_SERVER__RRT_STAR_HPP_
#define NAV2_COLREGS_LOCAL_PLANNER_SERVER__RRT_STAR_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <random>
#include <vector>

#include "geometry_msgs/msg/point.hpp"

namespace nav2_costmap_2d
{
class Costmap2D;
}  // namespace nav2_costmap_2d

namespace nav2_colregs_local_planner_server
{

enum class PlanStatus {SUCCESS, NO_PATH, CANCELED, TIMEOUT, INVALID_INPUT};

struct RRTStarParameters
{
  double step_size;
  int max_iterations;
  double goal_bias;
  double goal_threshold;
  double safety_dist;
  double cost_weight;
  int max_optimize_iters;
  double eta;
  uint32_t random_seed;
  bool prune_path;
};

struct RRTStarNode
{
  double x{0.0};
  double y{0.0};
  int parent_idx{-1};
  double cost_from_root{0.0};
};

class RRTStarTestPeer;

class RRTStar
{
public:
  explicit RRTStar(const RRTStarParameters & parameters);

  /// Plan from (start_x, start_y) to (goal_x, goal_y). Barriers are segment
  /// constraints carried as consecutive point pairs (6 points = 3 segments,
  /// matching the VO-RRT convention): any planned edge crossing a barrier
  /// segment is rejected just like a costmap collision.
  PlanStatus planPath(
    double start_x, double start_y, double goal_x, double goal_y,
    const nav2_costmap_2d::Costmap2D & costmap,
    const std::vector<geometry_msgs::msg::Point> & barriers,
    const std::function<bool()> & cancel_checker,
    const std::chrono::steady_clock::time_point & deadline,
    std::vector<RRTStarNode> & path);

private:
  friend class RRTStarTestPeer;

  bool parametersValid() const;
  bool checkInterrupted() const;
  bool boundedCeil(double value, int & result) const;
  bool barrierFree(double x1, double y1, double x2, double y2) const;
  bool pointCollisionFree(
    double x, double y, const nav2_costmap_2d::Costmap2D & costmap) const;
  bool collisionFree(
    double x1, double y1, double x2, double y2,
    const nav2_costmap_2d::Costmap2D & costmap) const;
  double edgeCost(
    double x1, double y1, double x2, double y2,
    const nav2_costmap_2d::Costmap2D & costmap) const;
  int nearestNode(double x, double y) const;
  std::vector<int> findNear(double x, double y) const;
  bool propagateDescendantCosts(
    int parent_idx, const nav2_costmap_2d::Costmap2D & costmap);
  bool rewire(
    int new_idx, const std::vector<int> & near,
    const nav2_costmap_2d::Costmap2D & costmap);
  bool prunePath(
    std::vector<RRTStarNode> & path,
    const nav2_costmap_2d::Costmap2D & costmap) const;

  RRTStarParameters parameters_;
  std::vector<RRTStarNode> tree_;
  std::mt19937 rng_;
  int iterations_executed_{0};
  const std::function<bool()> * cancel_checker_{nullptr};
  const std::vector<geometry_msgs::msg::Point> * barriers_{nullptr};
  std::chrono::steady_clock::time_point deadline_;
  mutable bool interrupted_{false};
  mutable PlanStatus interruption_status_{PlanStatus::CANCELED};
  mutable bool invalid_geometry_{false};
};

}  // namespace nav2_colregs_local_planner_server

#endif  // NAV2_COLREGS_LOCAL_PLANNER_SERVER__RRT_STAR_HPP_
