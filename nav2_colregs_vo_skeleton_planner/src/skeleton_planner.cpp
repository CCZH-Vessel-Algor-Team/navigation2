#include "nav2_colregs_vo_skeleton_planner/skeleton_planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nav2_colregs_vo_skeleton_planner
{

namespace
{

double clockNow()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

SkeletonPlanner::SkeletonPlanner(
  Space * space, const Pt & goal, const SkeletonConfig & config,
  uint64_t seed)
: space_(space),
  goal_(goal),
  config_(config),
  seed_(seed),
  revision_(space->revision())
{
  config_.validate();
}

bool SkeletonPlanner::goalChanged(const Pt & goal) const
{
  return std::hypot(goal.first - goal_.first, goal.second - goal_.second) > kEps;
}

void SkeletonPlanner::beginQuery(uint64_t world_revision)
{
  space_->beginQuery(world_revision);
}

void SkeletonPlanner::store(const std::vector<Pt> & path)
{
  if (static_cast<int>(path.size()) > config_.path_limit) {
    throw std::runtime_error("path_limit");
  }
  anchors_.assign(path.begin() + 1, path.end());
  if (anchors_.empty()) {
    anchors_.push_back(goal_);
  }
  active_ = 0;
  prefix_ = {path.front(), anchors_[0]};
  generation_++;
  revision_ = space_->revision();
  if (config_.reuse_iterations) {
    candidate_ = path;
  } else {
    candidate_.clear();
    tree_.reset();  // no reuse: drop the tree whenever a new path is stored
  }
}

void SkeletonPlanner::setPath(const std::vector<Pt> & path)
{
  if (!space_->pathValid(path, nullptr, &goal_)) {
    throw std::invalid_argument("invalid seed path");
  }
  store(path);
}

bool SkeletonPlanner::search(
  const Pt & query, const Pt & target, int iterations, bool persistent,
  const std::vector<Pt> * incumbent, Budget & budget, std::vector<Pt> & out)
{
  const bool keep_tree = persistent || config_.reuse_iterations > 0;
  if (!keep_tree || !tree_ || tree_->target() != target ||
    (config_.reuse_iterations == 0 && tree_->size() >= config_.node_limit))
  {
    tree_.reset();
    searches_++;
    tree_ = std::make_unique<BoundedInformedRRT>(
      space_, target, config_, seed_ + 7919ULL * searches_);
    if (config_.reuse_iterations > 0 && !candidate_.empty()) {
      tree_->seedPath(candidate_, budget);
    } else if (persistent && !prefix_.empty() && prefix_.back() == target) {
      tree_->seedPath(prefix_, budget);
    }
  }
  return tree_->search(query, budget, iterations, out, incumbent);
}

PlanStats SkeletonPlanner::replan(const Pt & query, std::vector<Pt> & out)
{
  PlanStats stats;
  const double t0 = clockNow();
  out.clear();
  replans_++;

  Budget budget(config_.max_work, config_.time_limit);
  budget.peak_nodes = tree_ ? tree_->size() : 0;
  const bool had_tree = tree_ != nullptr;
  BoundedInformedRRT * tree_before = tree_.get();
  std::vector<Pt> accepted;
  std::vector<Pt> prefix;
  bool have_prefix = false;
  int skips = 0;

  try {
    // ---- dynamic level 2+3: validate skeleton, keep the tree -----------
    const bool world_changed = revision_ != space_->revision();
    if (world_changed) {
      // R2-06: prefix validation consumes real work -> charge the budget
      if (!prefix_.empty() &&
        !space_->pathValid(prefix_, nullptr, nullptr, &budget))
      {
        prefix_.clear();
      }
      if (!anchors_.empty()) {
        std::vector<Pt> suffix(anchors_.begin() + active_, anchors_.end());
        if (!space_->pathValid(suffix, nullptr, &goal_, &budget)) {
          anchors_.clear();
        }
      }
      // R2-04: refresh tree costs EVERY changed query (cheap: one DDA per
      // short tree edge) so g-based shortcuts stay valid; the old
      // every-N-replans scheme left nine queries with stale g compared
      // against fresh edge costs.
      if (tree_ && config_.reuse_iterations > 0) {
        tree_->pruneInvalid(budget);
      }
      revision_ = space_->revision();
      if (config_.reuse_iterations == 0) {
        tree_.reset();  // prototype semantics for the frozen variant
      }
    }
    if (!space_->pointFree(query, &budget)) {
      stats.status = "invalid_start";
    } else if (!space_->pointFree(goal_, &budget)) {
      stats.status = "invalid_goal";
    } else if (std::hypot(query.first - goal_.first,
      query.second - goal_.second) <= config_.goal_tolerance &&
      space_->segmentFree(query, goal_, &budget))
    {
      accepted = {query, goal_};
      stats.mode = "reached";
      stats.status = "reached";
    } else {
      if (anchors_.empty()) {
        std::vector<Pt> path;
        if (search(query, goal_, config_.global_iterations, false, nullptr,
          budget, path))
        {
          store(path);
          accepted = path;
          stats.mode = "bootstrap";
        }
      }
      if (!anchors_.empty()) {
        // Atomic adoption (review R2-01): each stage that obtains a valid
        // prefix commits members (active_/prefix_/accepted) immediately, so
        // a budget interruption can only land BETWEEN consistent states.
        // The helper assembles accepted = prefix + anchors[active+1:] and
        // refuses oversized results before mutating anything.
        bool committed = false;
        auto adopt = [&](const std::vector<Pt> & p)
          {
            std::vector<Pt> full(p);
            full.insert(full.end(),
              anchors_.begin() + active_ + 1, anchors_.end());
            if (static_cast<int>(full.size()) > config_.path_limit) {
              stats.status = "storage_limit";
              return false;
            }
            prefix_ = p;
            accepted = std::move(full);
            committed = true;
            return true;
          };

        const Pt & target = anchors_[active_];
        if (!prefix_.empty() &&
          space_->reanchor(query, prefix_, prefix, &budget))
        {
          std::vector<Pt> pruned;
          if (space_->prune(prefix, pruned, &budget)) {
            prefix = std::move(pruned);
          }
          have_prefix = adopt(prefix);
          if (have_prefix && stats.mode != "bootstrap") {
            stats.mode = "reanchor";
          }
        }
        const double direct = space_->edgeCost(query, target, &budget);
        if (std::isfinite(direct) &&
          (!have_prefix || direct < space_->pathCost(prefix_, &budget) - kEps))
        {
          if (adopt({query, target})) {
            prefix = prefix_;
            have_prefix = true;
            if (stats.mode != "bootstrap") {
              stats.mode = "direct";
            }
          }
        }
        const bool next_exists = active_ + 1 < static_cast<int>(anchors_.size());
        if (!have_prefix && config_.allow_recovery && config_.allow_skip &&
          next_exists)
        {
          const Pt & nxt = anchors_[active_ + 1];
          if (space_->segmentFree(query, nxt, &budget)) {
            active_++;
            if (adopt({query, nxt})) {
              prefix = prefix_;
              have_prefix = true;
              stats.mode = "recovery_direct";
            } else {
              active_--;  // failed adoption must not advance the anchor
            }
          }
        }
        if (!have_prefix && config_.allow_recovery) {
          if (config_.reuse_iterations > 0) {
            std::vector<Pt> path;
            bool near_hit = false;
            const Pt & next_anchor =
              next_exists ? anchors_[active_ + 1] : goal_;
            const bool near_gate = !next_exists ||
              !space_->segmentFree(query, next_anchor, &budget);
            if (tree_ && tree_->target() == goal_ && near_gate) {
              stats.near_recovery_attempted = true;
              near_hit = tree_->connectNear(query, budget, path);
              stats.near_recovery_hit = near_hit;
            }
            if (!near_hit) {
              stats.global_recovery = true;
              search(query, goal_, config_.global_iterations, false, nullptr,
                budget, path);
            }
            if (!path.empty()) {
              store(path);  // resets anchors_/active_/prefix_
              accepted = path;
              prefix = prefix_;
              have_prefix = committed = true;
              stats.mode = near_hit ? "near_recovery" : "global_recovery";
            }
          } else {
            if (search(query, target, config_.local_iterations, true, nullptr,
              budget, prefix))
            {
              have_prefix = adopt(prefix);
              if (have_prefix) {
                stats.mode = "local_recovery";
              }
            } else if (!budget.exhausted()) {
              std::vector<Pt> path;
              if (search(query, goal_, config_.global_iterations, false,
                nullptr, budget, path))
              {
                store(path);
                accepted = path;
                prefix = prefix_;
                have_prefix = committed = true;
                stats.mode = "global_recovery";
              }
            }
          }
        }
        if (have_prefix) {
          // cost-aware skip adoption: each accepted skip commits immediately
          while (config_.allow_skip &&
            active_ + 1 < static_cast<int>(anchors_.size()))
          {
            const Pt & current = anchors_[active_];
            const Pt & nxt = anchors_[active_ + 1];
            const double skip_cost = space_->edgeCost(query, nxt, &budget);
            if (!std::isfinite(skip_cost)) {
              break;
            }
            const double keep_cost = space_->pathCost(prefix_, &budget) +
              space_->edgeCost(current, nxt, &budget);
            if (skip_cost > keep_cost + kEps) {
              break;
            }
            const int previous = active_;
            active_++;
            skips++;
            if (!adopt({query, nxt})) {
              active_ = previous;  // storage_limit: state stays consistent
              break;
            }
            if (stats.mode == "reuse" || stats.mode == "direct" ||
              stats.mode == "reanchor")
            {
              stats.mode = "skip";
            }
          }
        }
        (void)committed;
      }
      // ---- cross-query optimization with the 3% switch margin -----------
      if (config_.reuse_iterations > 0 && !accepted.empty() &&
        stats.mode != "bootstrap" && stats.mode != "near_recovery" &&
        stats.mode != "global_recovery" && stats.mode != "reached")
      {
        const double keep_cost = space_->pathCost(accepted, &budget);
        std::vector<Pt> incumbent = accepted;
        double incumbent_cost = keep_cost;
        if (!candidate_.empty()) {
          std::vector<Pt> reanchored;
          if (space_->reanchor(query, candidate_, reanchored, &budget)) {
            std::vector<Pt> pruned;
            if (space_->prune(reanchored, pruned, &budget)) {
              if (static_cast<int>(pruned.size()) <= config_.path_limit) {
                const double value = space_->pathCost(pruned, &budget);
                if (value < incumbent_cost - kEps) {
                  incumbent = std::move(pruned);
                  incumbent_cost = value;
                }
              }
            }
          }
        }
        const int64_t before = budget.search_iterations;
        std::vector<Pt> candidate;
        if (search(query, goal_, config_.reuse_iterations, true,
          incumbent.empty() ? nullptr : &incumbent, budget, candidate))
        {
          const int64_t optimization_iterations =
            budget.search_iterations - before;
          const double candidate_cost = tree_->lastCost();
          candidate_ = candidate;
          if (candidate_cost <
            keep_cost * (1.0 - config_.switch_margin) - kEps)
          {
            store(candidate);
            accepted = candidate;
            stats.mode = "improved";
            stats.adopted_cost = candidate_cost;
          } else if (optimization_iterations > 0) {
            stats.mode = "reuse_optimized";
            stats.adopted_cost = keep_cost;
          } else {
            stats.adopted_cost = keep_cost;
          }
          stats.search_iterations = optimization_iterations;
        } else {
          stats.adopted_cost = keep_cost;
          stats.search_iterations = budget.search_iterations - before;
        }
      } else if (!accepted.empty()) {
        stats.adopted_cost = space_->pathCost(accepted, &budget);
      }
    }
  } catch (const BudgetExceeded & e) {
    stats.status = e.what();
  } catch (const std::runtime_error & e) {
    if (std::string(e.what()) != "path_limit") {
      throw;
    }
    stats.status = "storage_limit";
  }

  if (!accepted.empty()) {
    if (stats.status != "reached") {
      stats.status = "ok";
    }
    out = std::move(accepted);
  } else if (stats.status == "no_path" && budget.reason) {
    stats.status = budget.reason;
  }

  stats.mode = stats.mode.empty() ? "reuse" : stats.mode;
  stats.anchor = active_;
  stats.skips = skips;
  stats.generation = generation_;
  stats.nodes = tree_ ? tree_->size() : 0;
  stats.peak_nodes = budget.peak_nodes;
  stats.work = budget.work;
  stats.tree_reused = had_tree && tree_.get() == tree_before;
  stats.elapsed_s = clockNow() - t0;
  return stats;
}

}  // namespace nav2_colregs_vo_skeleton_planner
