#ifndef NAV2_COLREGS_VO_SKELETON_PLANNER__SKELETON_TREE_HPP_
#define NAV2_COLREGS_VO_SKELETON_PLANNER__SKELETON_TREE_HPP_

#include <cstdint>
#include <memory>
#include <random>
#include <vector>

#include "nav2_colregs_vo_skeleton_planner/skeleton_space.hpp"

namespace nav2_colregs_vo_skeleton_planner
{

/** Search and capacity settings shared by the planner and its tree. */
struct SkeletonConfig
{
  double step = 4.0;
  double goal_bias = 0.1;
  double eta = 50.0;
  int node_limit = 1024;
  int path_limit = 256;
  int near_limit = 16;
  int connector_limit = 128;
  int recovery_near_limit = 16;
  int global_iterations = 2400;
  int local_iterations = 600;
  int refine_iterations = 200;
  int64_t max_work = 12000000;
  double time_limit = 0.0;
  double goal_tolerance = 2.0;
  bool allow_recovery = true;
  bool allow_skip = true;
  int reuse_iterations = 64;
  double switch_margin = 0.03;
  // dynamic-world behaviour (C++ extension over the prototype)
  int prune_period = 10;

  /** Throws std::invalid_argument on invalid combinations. */
  void validate() const;
};

/**
 * One fixed-target bounded RRT* tree; query motion never moves the root.
 * Direct port of the prototype BoundedInformedRRT with capacity recycling,
 * random ancestor shortcuts and OS connector nomination.
 */
class BoundedInformedRRT
{
public:
  BoundedInformedRRT(
    Space * space, const Pt & target, const SkeletonConfig & config,
    uint64_t seed);

  int size() const {return n_;}
  const Pt & target() const {return target_;}
  Pt nodePoint(int i) const
  {
    return Pt{x_[i], y_[i]};
  }
  double nodeCost(int i) const {return cost_[i];}
  int nodeParent(int i) const {return parent_[i];}
  int solutionTerminal() const {return solution_terminal_;}
  const char * lastReason() const {return last_reason_;}
  double lastCost() const {return last_cost_;}

  /** Seed from a validated query->target polyline (target must be its end). */
  bool seedPath(const std::vector<Pt> & path, Budget & budget);

  /** Evict low-priority leaves down to 75% capacity; returns removed count. */
  int recycle(const Pt & query, const std::vector<Pt> * incumbent,
    Budget & budget);

  /**
   * Search toward the given query; returns a valid compacted path through
   * `out` (false when none). Incumbent seeds the informed bound.
   */
  bool search(
    const Pt & query, Budget & budget, int iterations,
    std::vector<Pt> & out, const std::vector<Pt> * incumbent = nullptr);

  /** Recovery: connect the query through the nearest existing nodes. */
  bool connectNear(const Pt & query, Budget & budget, std::vector<Pt> & out);

  /**
   * Dynamic-world maintenance (C++ extension): remove every subtree whose
   * root edge is invalid under the current costmap, compacting in place.
   * Returns the number of removed nodes.
   */
  int pruneInvalid(Budget & budget);

private:
  bool extend(const Pt & sample, Budget & budget);
  void reparent(int child, int parent, double cost, Budget & budget);
  void shortcutNodes(Budget & budget);
  std::vector<int> connectionCandidates(
    const Pt & query, double best_cost, Budget & budget);
  void checkpoint(
    const Pt & query, Budget & budget, int iteration,
    std::vector<Pt> & best, double & best_cost, int & first);

  Space * space_;
  Pt target_;
  SkeletonConfig config_;
  std::mt19937_64 rng_;
  std::mt19937_64 connector_rng_;
  std::mt19937_64 shortcut_rng_;

  std::vector<double> x_, y_, cost_;
  std::vector<int> parent_;
  std::vector<std::vector<int>> children_;
  int n_ = 1;
  int solution_terminal_ = -1;
  bool costs_fresh_ = false;  // g values certified for the current revision
  const char * last_reason_ = nullptr;
  double last_cost_ = HUGE_VAL;
};

}  // namespace nav2_colregs_vo_skeleton_planner

#endif  // NAV2_COLREGS_VO_SKELETON_PLANNER__SKELETON_TREE_HPP_
