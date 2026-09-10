#ifndef NAV2_COLREGS_VO_SKELETON_PLANNER__SKELETON_PLANNER_HPP_
#define NAV2_COLREGS_VO_SKELETON_PLANNER__SKELETON_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_colregs_vo_skeleton_planner/skeleton_space.hpp"
#include "nav2_colregs_vo_skeleton_planner/skeleton_tree.hpp"

namespace nav2_colregs_vo_skeleton_planner
{

/** Per-replan diagnostic counters (subset of the prototype stats). */
struct PlanStats
{
  std::string mode = "reuse";
  std::string status = "no_path";
  int anchor = 0;
  int skips = 0;
  int generation = 0;
  int rebuilds = 0;
  int nodes = 0;
  int peak_nodes = 0;
  int64_t work = 0;
  int64_t search_iterations = 0;
  int pruned_tree_nodes = 0;
  bool tree_reused = false;
  double elapsed_s = 0.0;
  double adopted_cost = 0.0;
  bool near_recovery_attempted = false;
  bool near_recovery_hit = false;
  bool global_recovery = false;
};

/**
 * Persistent goal-rooted skeleton planner (port of SkeletonPlanner).
 *
 * Dynamic-world upgrade over the prototype (user-specified mechanism):
 *  1. Root validation  - a goal change resets the whole planner (wrapper).
 *  2. Skeleton validation - the adopted anchors/prefix are revalidated
 *     against the live costmap every replan; when occluded the tree is
 *     PRESERVED and the skeleton is rebuilt through it (near-recovery or
 *     global recovery on the same tree), not dropped.
 *  3. Tree prune - branches whose edges became invalid are physically
 *     removed (DRRT-style) whenever the skeleton failed this replan or
 *     every `prune_period` replans.
 */
class SkeletonPlanner
{
public:
  SkeletonPlanner(
    Space * space, const Pt & goal, const SkeletonConfig & config,
    uint64_t seed = 1);

  /** Install an externally validated seed path ending exactly at goal. */
  void setPath(const std::vector<Pt> & path);

  /** True when the stored goal differs from the requested one. */
  bool goalChanged(const Pt & goal) const;

  /**
   * One replan from the actual query position. Always returns a path
   * connected to the current query or a failure status.
   */
  PlanStats replan(const Pt & query, std::vector<Pt> & out);

  /** Advance the world revision (wrapper calls this every query). */
  void beginQuery(uint64_t world_revision);

  int treeNodes() const {return tree_ ? tree_->size() : 0;}

private:
  void store(const std::vector<Pt> & path);
  bool search(
    const Pt & query, const Pt & target, int iterations, bool persistent,
    const std::vector<Pt> * incumbent, Budget & budget,
    std::vector<Pt> & out);

  Space * space_;
  Pt goal_;
  SkeletonConfig config_;
  uint64_t seed_;
  std::unique_ptr<BoundedInformedRRT> tree_;
  std::vector<Pt> anchors_;
  std::vector<Pt> prefix_;      // query -> anchors_[active]
  std::vector<Pt> candidate_;   // best query-to-goal geometry seen
  int active_ = 0;
  int generation_ = 0;
  int searches_ = 0;
  int replans_ = 0;
  uint64_t revision_ = 0;
};

}  // namespace nav2_colregs_vo_skeleton_planner

#endif  // NAV2_COLREGS_VO_SKELETON_PLANNER__SKELETON_PLANNER_HPP_
