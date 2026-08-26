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

#include "nav2_colregs_local_planner_server/rrt_star.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace nav2_colregs_local_planner_server
{

namespace
{
constexpr double kEpsilon = 1e-9;

// Segment intersection test ported verbatim from the VO-RRT planner
// (nav2_colregs_vo_rrt_star_planner/src/rrt_star.cpp segmentsIntersect).
bool segmentsIntersect(
  double ax, double ay, double bx, double by,
  double cx, double cy, double dx, double dy)
{
  constexpr double eps = 1e-9;

  auto cross = [](double ux, double uy, double vx, double vy) {
    return ux * vy - uy * vx;
  };

  auto orientation = [&](double px, double py, double qx, double qy, double rx, double ry) {
    return cross(qx - px, qy - py, rx - px, ry - py);
  };

  auto onSegment = [&](double px, double py, double qx, double qy, double rx, double ry) {
    return qx <= std::max(px, rx) + eps && qx + eps >= std::min(px, rx) &&
           qy <= std::max(py, ry) + eps && qy + eps >= std::min(py, ry) &&
           std::abs(orientation(px, py, qx, qy, rx, ry)) <= eps;
  };

  const double o1 = orientation(ax, ay, bx, by, cx, cy);
  const double o2 = orientation(ax, ay, bx, by, dx, dy);
  const double o3 = orientation(cx, cy, dx, dy, ax, ay);
  const double o4 = orientation(cx, cy, dx, dy, bx, by);

  if (((o1 > eps && o2 < -eps) || (o1 < -eps && o2 > eps)) &&
    ((o3 > eps && o4 < -eps) || (o3 < -eps && o4 > eps)))
  {
    return true;
  }

  return onSegment(ax, ay, cx, cy, bx, by) ||
         onSegment(ax, ay, dx, dy, bx, by) ||
         onSegment(cx, cy, ax, ay, dx, dy) ||
         onSegment(cx, cy, bx, by, dx, dy);
}
}  // namespace

RRTStar::RRTStar(const RRTStarParameters & parameters)
: parameters_(parameters), rng_(parameters.random_seed)
{
}

PlanStatus RRTStar::planPath(
  double start_x, double start_y, double goal_x, double goal_y,
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & barriers,
  const std::function<bool()> & cancel_checker,
  const std::chrono::steady_clock::time_point & deadline,
  std::vector<RRTStarNode> & path)
{
  path.clear();
  tree_.clear();
  iterations_executed_ = 0;
  rng_.seed(parameters_.random_seed);
  cancel_checker_ = &cancel_checker;
  barriers_ = &barriers;
  deadline_ = deadline;
  interrupted_ = false;
  invalid_geometry_ = false;

  auto failureStatus = [&](PlanStatus fallback) {
    path.clear();
    if (interrupted_) {
      return interruption_status_;
    }
    return invalid_geometry_ ? PlanStatus::INVALID_INPUT : fallback;
  };

  bool barriers_finite = true;
  for (const auto & barrier_point : barriers) {
    if (!std::isfinite(barrier_point.x) || !std::isfinite(barrier_point.y)) {
      barriers_finite = false;
      break;
    }
  }

  if (!parametersValid() || !std::isfinite(start_x) || !std::isfinite(start_y) ||
    !std::isfinite(goal_x) || !std::isfinite(goal_y) ||
    costmap.getSizeInCellsX() == 0 || costmap.getSizeInCellsY() == 0 ||
    !std::isfinite(costmap.getResolution()) || costmap.getResolution() <= 0.0 ||
    !std::isfinite(costmap.getOriginX()) || !std::isfinite(costmap.getOriginY()) ||
    !barriers_finite)
  {
    return PlanStatus::INVALID_INPUT;
  }

  const double resolution = costmap.getResolution();
  const double outer_min_x = costmap.getOriginX();
  const double outer_min_y = costmap.getOriginY();
  const double min_x = outer_min_x + 0.5 * resolution;
  const double min_y = outer_min_y + 0.5 * resolution;
  const double max_x = outer_min_x +
    (static_cast<double>(costmap.getSizeInCellsX()) - 0.5) * resolution;
  const double max_y = outer_min_y +
    (static_cast<double>(costmap.getSizeInCellsY()) - 0.5) * resolution;
  const double outer_max_x = outer_min_x +
    static_cast<double>(costmap.getSizeInCellsX()) * resolution;
  const double outer_max_y = outer_min_y +
    static_cast<double>(costmap.getSizeInCellsY()) * resolution;
  if (!std::isfinite(min_x) || !std::isfinite(min_y) ||
    !std::isfinite(max_x) || !std::isfinite(max_y) ||
    !std::isfinite(outer_max_x) || !std::isfinite(outer_max_y))
  {
    return PlanStatus::INVALID_INPUT;
  }
  if (start_x < outer_min_x || start_x >= outer_max_x ||
    start_y < outer_min_y || start_y >= outer_max_y ||
    goal_x < outer_min_x || goal_x >= outer_max_x ||
    goal_y < outer_min_y || goal_y >= outer_max_y)
  {
    return PlanStatus::INVALID_INPUT;
  }

  int safety_radius;
  if (!boundedCeil(parameters_.safety_dist / resolution, safety_radius)) {
    return failureStatus(PlanStatus::INVALID_INPUT);
  }
  (void)safety_radius;
  if (!pointCollisionFree(start_x, start_y, costmap)) {
    return failureStatus(PlanStatus::INVALID_INPUT);
  }
  if (!pointCollisionFree(goal_x, goal_y, costmap)) {
    return failureStatus(PlanStatus::NO_PATH);
  }
  if (checkInterrupted()) {
    return failureStatus(PlanStatus::NO_PATH);
  }

  tree_.push_back({start_x, start_y, -1, 0.0});
  if (goal_x == start_x && goal_y == start_y) {
    path = {{start_x, start_y, -1, 0.0}, {goal_x, goal_y, 0, 0.0}};
    return PlanStatus::SUCCESS;
  }

  bool goal_reached = false;
  auto iterate = [&]() {
    ++iterations_executed_;
    if (checkInterrupted()) {
      return false;
    }
    std::uniform_real_distribution<double> unit_distribution(0.0, 1.0);
    double sample_x = std::clamp(goal_x, min_x, max_x);
    double sample_y = std::clamp(goal_y, min_y, max_y);
    if (unit_distribution(rng_) >= parameters_.goal_bias) {
      sample_x = std::uniform_real_distribution<double>(min_x, max_x)(rng_);
      sample_y = std::uniform_real_distribution<double>(min_y, max_y)(rng_);
    }

    const int nearest_idx = nearestNode(sample_x, sample_y);
    if (nearest_idx < 0) {
      return false;
    }
    const auto nearest = tree_[nearest_idx];
    const double dx = sample_x - nearest.x;
    const double dy = sample_y - nearest.y;
    const double distance = std::hypot(dx, dy);
    if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(distance)) {
      invalid_geometry_ = true;
      return false;
    }
    if (distance <= kEpsilon) {
      return true;
    }

    const double scale = std::min(1.0, parameters_.step_size / distance);
    const double new_x = std::clamp(nearest.x + dx * scale, min_x, max_x);
    const double new_y = std::clamp(nearest.y + dy * scale, min_y, max_y);
    if (!std::isfinite(new_x) || !std::isfinite(new_y)) {
      invalid_geometry_ = true;
      return false;
    }
    for (const auto & node : tree_) {
      if (checkInterrupted()) {
        return false;
      }
      if (std::hypot(node.x - new_x, node.y - new_y) <= kEpsilon) {
        return true;
      }
    }
    if (!collisionFree(nearest.x, nearest.y, new_x, new_y, costmap)) {
      return !interrupted_ && !invalid_geometry_;
    }

    const auto near = findNear(new_x, new_y);
    if (interrupted_) {
      return false;
    }
    int best_parent = nearest_idx;
    double best_cost = nearest.cost_from_root +
      edgeCost(nearest.x, nearest.y, new_x, new_y, costmap);
    if (!std::isfinite(best_cost)) {
      invalid_geometry_ = true;
    }
    if (interrupted_ || invalid_geometry_) {
      return false;
    }
    for (const int idx : near) {
      if (checkInterrupted()) {
        return false;
      }
      const auto & candidate = tree_[idx];
      if (!collisionFree(candidate.x, candidate.y, new_x, new_y, costmap)) {
        if (interrupted_ || invalid_geometry_) {
          return false;
        }
        continue;
      }
      const double candidate_cost = candidate.cost_from_root +
        edgeCost(candidate.x, candidate.y, new_x, new_y, costmap);
      if (!std::isfinite(candidate_cost)) {
        invalid_geometry_ = true;
      }
      if (interrupted_ || invalid_geometry_) {
        return false;
      }
      if (candidate_cost < best_cost - kEpsilon) {
        best_parent = idx;
        best_cost = candidate_cost;
      }
    }

    const int new_idx = static_cast<int>(tree_.size());
    tree_.push_back({new_x, new_y, best_parent, best_cost});
    if (!rewire(new_idx, near, costmap)) {
      return false;
    }
    if (std::hypot(new_x - goal_x, new_y - goal_y) <= parameters_.goal_threshold &&
      collisionFree(new_x, new_y, goal_x, goal_y, costmap))
    {
      goal_reached = true;
    }
    return !interrupted_ && !invalid_geometry_;
  };

  if (std::hypot(goal_x - start_x, goal_y - start_y) <= parameters_.goal_threshold &&
    collisionFree(start_x, start_y, goal_x, goal_y, costmap))
  {
    goal_reached = true;
  }
  if (interrupted_ || invalid_geometry_) {
    return failureStatus(PlanStatus::NO_PATH);
  }

  for (
    int iteration = 0;
    iteration < parameters_.max_iterations && !goal_reached;
    ++iteration)
  {
    if (checkInterrupted() || !iterate()) {
      return failureStatus(PlanStatus::NO_PATH);
    }
  }

  if (!goal_reached) {
    return failureStatus(PlanStatus::NO_PATH);
  }

  for (int iteration = 0; iteration < parameters_.max_optimize_iters; ++iteration) {
    if (checkInterrupted() || !iterate()) {
      return failureStatus(PlanStatus::NO_PATH);
    }
  }

  int best_goal_parent = -1;
  double best_goal_cost = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < tree_.size(); ++i) {
    if (checkInterrupted()) {
      return failureStatus(PlanStatus::NO_PATH);
    }
    const auto & node = tree_[i];
    const double distance = std::hypot(node.x - goal_x, node.y - goal_y);
    if (!std::isfinite(distance)) {
      invalid_geometry_ = true;
      return failureStatus(PlanStatus::NO_PATH);
    }
    if (distance > parameters_.goal_threshold ||
      !collisionFree(node.x, node.y, goal_x, goal_y, costmap))
    {
      if (interrupted_ || invalid_geometry_) {
        return failureStatus(PlanStatus::NO_PATH);
      }
      continue;
    }
    const double cost = node.cost_from_root +
      edgeCost(node.x, node.y, goal_x, goal_y, costmap);
    if (!std::isfinite(cost)) {
      invalid_geometry_ = true;
    }
    if (interrupted_ || invalid_geometry_) {
      return failureStatus(PlanStatus::NO_PATH);
    }
    if (cost < best_goal_cost - kEpsilon) {
      best_goal_parent = static_cast<int>(i);
      best_goal_cost = cost;
    }
  }
  if (best_goal_parent < 0) {
    return PlanStatus::NO_PATH;
  }

  for (int idx = best_goal_parent; idx >= 0; idx = tree_[idx].parent_idx) {
    if (checkInterrupted()) {
      return failureStatus(PlanStatus::NO_PATH);
    }
    path.push_back(tree_[idx]);
  }
  std::reverse(path.begin(), path.end());
  if (path.back().x != goal_x || path.back().y != goal_y) {
    path.push_back({goal_x, goal_y, static_cast<int>(path.size()) - 1, best_goal_cost});
  }
  for (size_t i = 0; i < path.size(); ++i) {
    if (checkInterrupted()) {
      return failureStatus(PlanStatus::NO_PATH);
    }
    path[i].parent_idx = i == 0 ? -1 : static_cast<int>(i) - 1;
  }
  if (parameters_.prune_path) {
    if (!prunePath(path, costmap)) {
      return failureStatus(PlanStatus::NO_PATH);
    }
  }
  return PlanStatus::SUCCESS;
}

bool RRTStar::parametersValid() const
{
  return std::isfinite(parameters_.step_size) && parameters_.step_size > 0.0 &&
         parameters_.max_iterations > 0 && std::isfinite(parameters_.goal_bias) &&
         parameters_.goal_bias >= 0.0 && parameters_.goal_bias <= 1.0 &&
         std::isfinite(parameters_.goal_threshold) && parameters_.goal_threshold >= 0.0 &&
         std::isfinite(parameters_.safety_dist) && parameters_.safety_dist >= 0.0 &&
         std::isfinite(parameters_.cost_weight) && parameters_.cost_weight >= 0.0 &&
         parameters_.max_optimize_iters >= 0 && std::isfinite(parameters_.eta) &&
         parameters_.eta > 0.0;
}

bool RRTStar::checkInterrupted() const
{
  if (interrupted_) {
    return true;
  }
  if (cancel_checker_ && *cancel_checker_ && (*cancel_checker_)()) {
    interrupted_ = true;
    interruption_status_ = PlanStatus::CANCELED;
  } else if (std::chrono::steady_clock::now() >= deadline_) {
    interrupted_ = true;
    interruption_status_ = PlanStatus::TIMEOUT;
  }
  return interrupted_;
}

bool RRTStar::boundedCeil(double value, int & result) const
{
  constexpr double max_count = static_cast<double>(std::numeric_limits<int>::max() - 1);
  if (!std::isfinite(value) || value < 0.0 || value > max_count) {
    invalid_geometry_ = true;
    return false;
  }
  result = static_cast<int>(std::ceil(value));
  return true;
}

bool RRTStar::pointCollisionFree(
  double x, double y, const nav2_costmap_2d::Costmap2D & costmap) const
{
  unsigned int map_x;
  unsigned int map_y;
  if (checkInterrupted()) {
    return false;
  }
  if (!costmap.worldToMap(x, y, map_x, map_y)) {
    return false;
  }

  const double resolution = costmap.getResolution();
  int radius;
  if (!boundedCeil(parameters_.safety_dist / resolution, radius)) {
    return false;
  }
  const int64_t min_cell_x = std::max<int64_t>(
    -1, static_cast<int64_t>(map_x) - radius);
  const int64_t max_cell_x = std::min<int64_t>(
    costmap.getSizeInCellsX(), static_cast<int64_t>(map_x) + radius);
  const int64_t min_cell_y = std::max<int64_t>(
    -1, static_cast<int64_t>(map_y) - radius);
  const int64_t max_cell_y = std::min<int64_t>(
    costmap.getSizeInCellsY(), static_cast<int64_t>(map_y) + radius);
  const double squared_safety_distance = parameters_.safety_dist * parameters_.safety_dist;
  for (int64_t cell_x = min_cell_x; cell_x <= max_cell_x; ++cell_x) {
    for (int64_t cell_y = min_cell_y; cell_y <= max_cell_y; ++cell_y) {
      if (checkInterrupted()) {
        return false;
      }
      const double cell_min_x = costmap.getOriginX() + cell_x * resolution;
      const double cell_max_x = cell_min_x + resolution;
      const double cell_min_y = costmap.getOriginY() + cell_y * resolution;
      const double cell_max_y = cell_min_y + resolution;
      const double distance_x = std::max({cell_min_x - x, 0.0, x - cell_max_x});
      const double distance_y = std::max({cell_min_y - y, 0.0, y - cell_max_y});
      const double squared_distance = distance_x * distance_x + distance_y * distance_y;
      if (squared_distance > squared_safety_distance) {
        continue;
      }
      if (cell_x < 0 || cell_y < 0 ||
        cell_x >= static_cast<int64_t>(costmap.getSizeInCellsX()) ||
        cell_y >= static_cast<int64_t>(costmap.getSizeInCellsY()) ||
        costmap.getCost(
          static_cast<unsigned int>(cell_x), static_cast<unsigned int>(cell_y)) >=
        nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        return false;
      }
    }
  }
  return true;
}

bool RRTStar::barrierFree(double x1, double y1, double x2, double y2) const
{
  if (barriers_ == nullptr) {
    return true;
  }
  for (size_t i = 0; i + 1 < barriers_->size(); i += 2) {
    const auto & a = (*barriers_)[i];
    const auto & b = (*barriers_)[i + 1];
    if (segmentsIntersect(x1, y1, x2, y2, a.x, a.y, b.x, b.y)) {
      return false;
    }
  }
  return true;
}

bool RRTStar::collisionFree(
  double x1, double y1, double x2, double y2,
  const nav2_costmap_2d::Costmap2D & costmap) const
{
  const double length = std::hypot(x2 - x1, y2 - y1);
  if (!std::isfinite(length)) {
    invalid_geometry_ = true;
    return false;
  }
  if (!barrierFree(x1, y1, x2, y2)) {
    return false;
  }
  if (length == 0.0) {
    return pointCollisionFree(x1, y1, costmap);
  }
  int samples;
  if (!boundedCeil(length / (0.5 * costmap.getResolution()), samples)) {
    return false;
  }
  samples = std::max(1, samples);
  for (int i = 0; i <= samples; ++i) {
    if (checkInterrupted()) {
      return false;
    }
    const double ratio = static_cast<double>(i) / samples;
    if (!pointCollisionFree(
        x1 + ratio * (x2 - x1), y1 + ratio * (y2 - y1), costmap))
    {
      return false;
    }
  }
  return true;
}

double RRTStar::edgeCost(
  double x1, double y1, double x2, double y2,
  const nav2_costmap_2d::Costmap2D & costmap) const
{
  const double length = std::hypot(x2 - x1, y2 - y1);
  if (!std::isfinite(length)) {
    invalid_geometry_ = true;
    return std::numeric_limits<double>::infinity();
  }
  if (length == 0.0) {
    return 0.0;
  }
  int samples;
  if (!boundedCeil(length / costmap.getResolution(), samples)) {
    return std::numeric_limits<double>::infinity();
  }
  samples = std::max(1, samples);
  double normalized_cost_sum = 0.0;
  for (int i = 0; i < samples; ++i) {
    if (checkInterrupted()) {
      return std::numeric_limits<double>::infinity();
    }
    const double ratio = (static_cast<double>(i) + 0.5) / samples;
    unsigned int map_x;
    unsigned int map_y;
    if (!costmap.worldToMap(
        x1 + ratio * (x2 - x1), y1 + ratio * (y2 - y1), map_x, map_y))
    {
      return std::numeric_limits<double>::infinity();
    }
    normalized_cost_sum += static_cast<double>(costmap.getCost(map_x, map_y)) / 254.0;
    if (!std::isfinite(normalized_cost_sum)) {
      invalid_geometry_ = true;
      return std::numeric_limits<double>::infinity();
    }
  }
  const double average_cost = normalized_cost_sum / samples;
  const double weighted_cost = parameters_.cost_weight * average_cost;
  const double multiplier = 1.0 + weighted_cost;
  const double cost = length * multiplier;
  if (!std::isfinite(average_cost) || !std::isfinite(weighted_cost) ||
    !std::isfinite(multiplier) || !std::isfinite(cost))
  {
    invalid_geometry_ = true;
    return std::numeric_limits<double>::infinity();
  }
  return cost;
}

int RRTStar::nearestNode(double x, double y) const
{
  int nearest_idx = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < tree_.size(); ++i) {
    if (checkInterrupted()) {
      return -1;
    }
    const double distance = std::hypot(tree_[i].x - x, tree_[i].y - y);
    if (!std::isfinite(distance)) {
      invalid_geometry_ = true;
      return -1;
    }
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_idx = static_cast<int>(i);
    }
  }
  return nearest_idx;
}

std::vector<int> RRTStar::findNear(double x, double y) const
{
  const double count = static_cast<double>(tree_.size() + 1U);
  const double radius = std::max(
    parameters_.step_size * 1.1,
    parameters_.eta * std::sqrt(std::log(count) / count));
  std::vector<int> near;
  for (size_t i = 0; i < tree_.size(); ++i) {
    if (checkInterrupted()) {
      return {};
    }
    const double distance = std::hypot(tree_[i].x - x, tree_[i].y - y);
    if (!std::isfinite(distance) || !std::isfinite(radius)) {
      invalid_geometry_ = true;
      return {};
    }
    if (distance <= radius) {
      near.push_back(static_cast<int>(i));
    }
  }
  return near;
}

bool RRTStar::propagateDescendantCosts(
  int parent_idx, const nav2_costmap_2d::Costmap2D & costmap)
{
  for (size_t i = 0; i < tree_.size(); ++i) {
    if (checkInterrupted()) {
      return false;
    }
    if (tree_[i].parent_idx != parent_idx) {
      continue;
    }
    const auto & parent = tree_[parent_idx];
    const double descendant_cost = parent.cost_from_root +
      edgeCost(parent.x, parent.y, tree_[i].x, tree_[i].y, costmap);
    if (!std::isfinite(descendant_cost)) {
      invalid_geometry_ = true;
      return false;
    }
    tree_[i].cost_from_root = descendant_cost;
    if (interrupted_ || invalid_geometry_ ||
      !propagateDescendantCosts(static_cast<int>(i), costmap))
    {
      return false;
    }
  }
  return true;
}

bool RRTStar::rewire(
  int new_idx, const std::vector<int> & near,
  const nav2_costmap_2d::Costmap2D & costmap)
{
  const auto new_node = tree_[new_idx];
  for (const int idx : near) {
    if (checkInterrupted()) {
      return false;
    }
    if (idx == new_node.parent_idx) {
      continue;
    }
    const double new_cost = new_node.cost_from_root +
      edgeCost(new_node.x, new_node.y, tree_[idx].x, tree_[idx].y, costmap);
    if (!std::isfinite(new_cost)) {
      invalid_geometry_ = true;
      return false;
    }
    if (interrupted_ || invalid_geometry_) {
      return false;
    }
    if (new_cost >= tree_[idx].cost_from_root - kEpsilon) {
      continue;
    }
    if (!collisionFree(
        new_node.x, new_node.y, tree_[idx].x, tree_[idx].y, costmap))
    {
      if (interrupted_ || invalid_geometry_) {
        return false;
      }
      continue;
    }
    tree_[idx].parent_idx = new_idx;
    tree_[idx].cost_from_root = new_cost;
    if (!propagateDescendantCosts(idx, costmap)) {
      return false;
    }
  }
  return true;
}

bool RRTStar::prunePath(
  std::vector<RRTStarNode> & path,
  const nav2_costmap_2d::Costmap2D & costmap) const
{
  if (path.size() <= 2) {
    return !checkInterrupted();
  }
  std::vector<RRTStarNode> pruned{path.front()};
  size_t current = 0;
  while (current + 1 < path.size()) {
    if (checkInterrupted()) {
      return false;
    }
    size_t next = path.size() - 1;
    while (next > current + 1 &&
      !collisionFree(
        path[current].x, path[current].y, path[next].x, path[next].y, costmap))
    {
      if (interrupted_ || invalid_geometry_) {
        return false;
      }
      --next;
    }
    pruned.push_back(path[next]);
    current = next;
  }
  pruned.front().parent_idx = -1;
  pruned.front().cost_from_root = 0.0;
  for (size_t i = 1; i < pruned.size(); ++i) {
    if (checkInterrupted()) {
      return false;
    }
    pruned[i].parent_idx = static_cast<int>(i) - 1;
    const double pruned_cost = pruned[i - 1].cost_from_root +
      edgeCost(pruned[i - 1].x, pruned[i - 1].y, pruned[i].x, pruned[i].y, costmap);
    if (!std::isfinite(pruned_cost)) {
      invalid_geometry_ = true;
      return false;
    }
    pruned[i].cost_from_root = pruned_cost;
    if (interrupted_ || invalid_geometry_) {
      return false;
    }
  }
  path = std::move(pruned);
  return true;
}

}  // namespace nav2_colregs_local_planner_server
