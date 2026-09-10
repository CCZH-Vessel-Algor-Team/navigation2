#ifndef NAV2_COLREGS_VO_SKELETON_PLANNER__SKELETON_SPACE_HPP_
#define NAV2_COLREGS_VO_SKELETON_PLANNER__SKELETON_SPACE_HPP_

#include <cstdint>
#include <list>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace nav2_colregs_vo_skeleton_planner
{

using Pt = std::pair<double, double>;

static constexpr double kEps = 1e-9;

/** Query-wide work/time meter charged before every unit of work. */
struct Budget
{
  explicit Budget(int64_t limit, double time_limit_s = 0.0);
  void take(int64_t amount = 1);
  bool exhausted() const {return reason != nullptr;}

  int64_t limit;
  double deadline;               // seconds on the clock; infinite when unset
  int64_t work = 0;
  const char * reason = nullptr; // "work_limit" / "time_limit"
  // auditable counters
  int collision_checks = 0;
  int cost_checks = 0;
  int search_iterations = 0;
  int connector_candidates = 0;
  int connector_checks = 0;
  int shortcut_candidates = 0;
  int shortcut_checks = 0;
  int shortcut_reparents = 0;
  int near_recovery_candidates = 0;
  int near_recovery_checks = 0;
  int peak_nodes = 0;
  int pruned_nodes = 0;
};

class BudgetExceeded : public std::runtime_error
{
public:
  explicit BudgetExceeded(const char * reason)
  : std::runtime_error(reason) {}
};

/**
 * Live-costmap planning geometry with supercover traversal and additive
 * piecewise edge cost, mirroring the prototype Space semantics:
 *   c(a,b) = |ab| * (1 + cost_weight * integral of cell cost along [a,b])
 * Collision additionally rejects cells within safety_dist of an obstacle
 * (disk test at every visited cell, equivalent to raster dilation) and any
 * COLREGS barrier segment crossing the edge.
 *
 * The edge cache lives for one query: the wrapper calls beginQuery() with a
 * fresh world revision each createPlan, because the underlying costmap is
 * updated in place with no version counter of its own.
 */
class Space
{
public:
  Space(
    const nav2_costmap_2d::Costmap2D * costmap,
    double safety_dist,
    double cost_weight);

  /** New query context: bump revision, reset the edge cache and barriers. */
  void beginQuery(uint64_t world_revision);

  /** Point the space at a per-query stable costmap snapshot (R2-03). */
  void updateCostmap(const nav2_costmap_2d::Costmap2D * costmap)
  {
    costmap_ = costmap;
  }
  void setBarriers(const std::vector<geometry_msgs::msg::Point> & barriers);

  uint64_t revision() const {return revision_;}
  double costWeight() const {return cost_weight_;}
  /** 1 + cost_weight * min free-cell cost; cached per revision. */
  double minMultiplier(Budget * budget = nullptr);

  bool pointFree(const Pt & p, Budget * budget = nullptr);
  bool segmentFree(const Pt & a, const Pt & b, Budget * budget = nullptr);
  /** Infinity when blocked by obstacles, inflation or barriers. */
  double edgeCost(const Pt & a, const Pt & b, Budget * budget = nullptr);

  double pathCost(const std::vector<Pt> & path, Budget * budget = nullptr);
  bool pathValid(
    const std::vector<Pt> & path,
    const Pt * start = nullptr, const Pt * goal = nullptr,
    Budget * budget = nullptr);

  /** Greedy non-increasing-cost shortcut; nullopt-equivalent empty on failure. */
  bool prune(const std::vector<Pt> & path, std::vector<Pt> & out,
    Budget * budget = nullptr);

  /** Splice a moved query onto nearby segments of a valid old path. */
  bool reanchor(const Pt & query, const std::vector<Pt> & path,
    std::vector<Pt> & out, Budget * budget = nullptr, int limit = 12);

  // test accessors
  int cacheEntries() const {return static_cast<int>(cache_.size());}
  const nav2_costmap_2d::Costmap2D * costmap() const {return costmap_;}

private:
  struct EdgeResult
  {
    bool free;
    double cost;
  };
  struct EdgeKey
  {
    Pt a, b;
    bool operator==(const EdgeKey & o) const
    {
      return a == o.a && b == o.b;
    }
  };
  struct EdgeKeyHash
  {
    size_t operator()(const EdgeKey & k) const
    {
      size_t h = std::hash<double>()(k.a.first);
      h = h * 31 + std::hash<double>()(k.a.second);
      h = h * 31 + std::hash<double>()(k.b.first);
      h = h * 31 + std::hash<double>()(k.b.second);
      return h;
    }
  };

  bool blockedDisk(int cx, int cy) const;
  double cellCost(int cx, int cy) const;
  bool inside(int cx, int cy) const
  {
    return cx >= 0 && cx < static_cast<int>(costmap_->getSizeInCellsX()) &&
           cy >= 0 && cy < static_cast<int>(costmap_->getSizeInCellsY());
  }
  bool barriersFree(const Pt & a, const Pt & b) const;
  EdgeResult computeEdge(const Pt & a, const Pt & b, Budget * budget);

  const nav2_costmap_2d::Costmap2D * costmap_;
  double safety_dist_;
  double cost_weight_;
  uint64_t revision_ = 0;
  double min_multiplier_ = 1.0;
  bool min_multiplier_valid_ = false;
  std::vector<geometry_msgs::msg::Point> barriers_;
  // safety disk offsets in cells (dy, dx)
  std::vector<std::pair<int, int>> disk_;
  std::unordered_map<EdgeKey, EdgeResult, EdgeKeyHash> cache_;
  static constexpr int kCacheLimit = 2048;
};

}  // namespace nav2_colregs_vo_skeleton_planner

#endif  // NAV2_COLREGS_VO_SKELETON_PLANNER__SKELETON_SPACE_HPP_
