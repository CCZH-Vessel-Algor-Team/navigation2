#include "nav2_colregs_vo_skeleton_planner/skeleton_tree.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <stdexcept>

namespace nav2_colregs_vo_skeleton_planner
{

void SkeletonConfig::validate() const
{
  auto require_finite_positive = [&](const char * name, double value) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(std::string(name) + " must be finite and positive");
      }
    };
  require_finite_positive("step", step);
  require_finite_positive("eta", eta);
  if (!std::isfinite(goal_bias) || goal_bias < 0.0 || goal_bias > 1.0) {
    throw std::invalid_argument("goal_bias must be in [0, 1]");
  }
  auto require_positive_int = [&](const char * name, int value) {
      if (value < 1) {
        throw std::invalid_argument(std::string(name) + " must be a positive integer");
      }
    };
  require_positive_int("node_limit", node_limit);
  require_positive_int("path_limit", path_limit);
  require_positive_int("near_limit", near_limit);
  require_positive_int("connector_limit", connector_limit);
  require_positive_int("recovery_near_limit", recovery_near_limit);
  require_positive_int("global_iterations", global_iterations);
  require_positive_int("local_iterations", local_iterations);
  require_positive_int("max_work", static_cast<int>(max_work));
  if (node_limit < 2 || path_limit < 2) {
    throw std::invalid_argument("node_limit and path_limit must be at least two");
  }
  if (refine_iterations < 0) {
    throw std::invalid_argument("refine_iterations must be nonnegative");
  }
  if (reuse_iterations < 0) {
    throw std::invalid_argument("reuse_iterations must be nonnegative");
  }
  if (prune_period < 1) {
    throw std::invalid_argument("prune_period must be a positive integer");
  }
  if (!std::isfinite(switch_margin) || switch_margin < 0.0 || switch_margin >= 1.0) {
    throw std::invalid_argument("switch_margin must be in [0, 1)");
  }
  if (!std::isfinite(time_limit) || time_limit < 0.0) {
    throw std::invalid_argument("time_limit must be finite and nonnegative");
  }
  if (!std::isfinite(goal_tolerance) || goal_tolerance < 0.0) {
    throw std::invalid_argument("goal_tolerance must be finite and nonnegative");
  }
}

namespace
{

double uniform01(std::mt19937_64 & rng)
{
  return std::generate_canonical<double, 53>(rng);
}

/** Random subset of [0, n) of size k without replacement (random order). */
std::vector<int> sampleSubset(std::mt19937_64 & rng, int n, int k)
{
  std::vector<int> pool(n);
  for (int i = 0; i < n; ++i) {
    pool[i] = i;
  }
  const int count = std::min(k, n);
  for (int i = 0; i < count; ++i) {
    std::uniform_int_distribution<int> dist(i, n - 1);
    std::swap(pool[i], pool[dist(rng)]);
  }
  pool.resize(count);
  return pool;
}

}  // namespace

BoundedInformedRRT::BoundedInformedRRT(
  Space * space, const Pt & target, const SkeletonConfig & config,
  uint64_t seed)
: space_(space),
  target_(target),
  config_(config),
  rng_(seed),
  connector_rng_(seed ^ 0x9E3779B97F4A7C15ULL),
  shortcut_rng_(seed ^ 0xC2B2AE3D27D4EB4FULL)
{
  x_.assign(config_.node_limit, 0.0);
  y_.assign(config_.node_limit, 0.0);
  cost_.assign(config_.node_limit, 0.0);
  parent_.assign(config_.node_limit, -1);
  children_.assign(config_.node_limit, {});
  x_[0] = target.first;
  y_[0] = target.second;
  cost_[0] = 0.0;
}

bool BoundedInformedRRT::seedPath(const std::vector<Pt> & path, Budget & budget)
{
  if (path.empty() || static_cast<int>(path.size()) > config_.node_limit ||
    !space_->pathValid(path, nullptr, &target_, &budget))
  {
    return false;
  }
  std::vector<Pt> reversed(path.rbegin(), path.rend());
  std::vector<double> edge_costs;
  edge_costs.reserve(reversed.size());
  for (size_t i = 0; i + 1 < reversed.size(); ++i) {
    edge_costs.push_back(space_->edgeCost(reversed[i], reversed[i + 1], &budget));
  }
  budget.take(static_cast<int64_t>(path.size()));
  for (size_t i = 0; i < reversed.size(); ++i) {
    x_[i] = reversed[i].first;
    y_[i] = reversed[i].second;
    parent_[i] = static_cast<int>(i) - 1;
    cost_[i] = i == 0 ? 0.0 : cost_[i - 1] + edge_costs[i - 1];
    children_[i].clear();
    if (i + 1 < reversed.size()) {
      children_[i].push_back(static_cast<int>(i + 1));
    }
  }
  n_ = static_cast<int>(path.size());
  solution_terminal_ = -1;
  budget.peak_nodes = std::max(budget.peak_nodes, n_);
  return true;
}

int BoundedInformedRRT::recycle(
  const Pt & query, const std::vector<Pt> * incumbent, Budget & budget)
{
  const int n = n_;
  budget.take(4 * n);
  std::vector<bool> protected_node(n, false);
  protected_node[0] = true;
  for (int node = solution_terminal_; node > 0; node = parent_[node]) {
    budget.take();
    protected_node[node] = true;
  }
  const int target_count = std::max(
    static_cast<int>(std::count(protected_node.begin(), protected_node.end(), true)),
    std::max(1, static_cast<int>(0.75 * config_.node_limit)));
  if (target_count >= n) {
    if (incumbent && static_cast<int>(incumbent->size()) < n) {
      std::vector<Pt> keep(*incumbent);
      const int before = n_;
      if (seedPath(keep, budget)) {
        budget.pruned_nodes += before - n_;
        return before - n_;
      }
    }
    return 0;
  }

  std::vector<double> distance(n), to_target(n);
  for (int i = 0; i < n; ++i) {
    distance[i] = std::hypot(x_[i] - query.first, y_[i] - query.second);
    to_target[i] = std::hypot(x_[i] - target_.first, y_[i] - target_.second);
  }
  const double multiplier = space_->minMultiplier(&budget);
  const double bound = incumbent ? space_->pathCost(*incumbent, &budget) : HUGE_VAL;
  std::vector<int> child_count(n, 0);
  for (int i = 1; i < n; ++i) {
    child_count[parent_[i]]++;
  }

  using Item = std::tuple<int, double, int>;  // (-outside, -priority, index)
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> leaves;
  for (int i = 1; i < n; ++i) {
    if (child_count[i] == 0 && !protected_node[i]) {
      const bool outside =
        (distance[i] + to_target[i]) * multiplier > bound + kEps;
      leaves.push({outside ? -1 : 0, -(cost_[i] + distance[i]), i});
    }
  }
  std::vector<bool> keep(n, true);
  int remaining = n;
  while (!leaves.empty() && remaining > target_count) {
    budget.take();
    const auto [neg_outside, neg_priority, i] = leaves.top();
    (void)neg_outside;
    (void)neg_priority;
    leaves.pop();
    keep[i] = false;
    remaining--;
    const int parent = parent_[i];
    child_count[parent]--;
    if (child_count[parent] == 0 && !protected_node[parent] && parent != 0) {
      const bool outside =
        (distance[parent] + to_target[parent]) * multiplier > bound + kEps;
      leaves.push(
        {outside ? -1 : 0, -(cost_[parent] + distance[parent]), parent});
    }
  }

  budget.take(3LL * n + config_.node_limit);
  std::vector<int> remap(n, -1);
  int out_idx = 0;
  for (int i = 0; i < n; ++i) {
    if (keep[i]) {
      remap[i] = out_idx++;
    }
  }
  for (int i = 0; i < n; ++i) {
    const int j = remap[i];
    if (j < 0) {
      continue;
    }
    x_[j] = x_[i];
    y_[j] = y_[i];
    cost_[j] = cost_[i];
    parent_[j] = parent_[i] >= 0 ? remap[parent_[i]] : -1;
  }
  for (int i = 0; i < config_.node_limit; ++i) {
    children_[i].clear();
  }
  for (int i = 1; i < remaining; ++i) {
    children_[parent_[i]].push_back(i);
  }
  if (solution_terminal_ >= 0) {
    solution_terminal_ = remap[solution_terminal_];
  }
  n_ = remaining;
  budget.pruned_nodes += n - remaining;
  return n - remaining;
}

bool BoundedInformedRRT::extend(const Pt & sample, Budget & budget)
{
  if (n_ >= config_.node_limit) {
    last_reason_ = "node_limit";
    return false;
  }
  budget.take(n_);
  int nearest = 0;
  double best_distance = HUGE_VAL;
  for (int i = 0; i < n_; ++i) {
    const double d = std::hypot(x_[i] - sample.first, y_[i] - sample.second);
    if (d < best_distance) {
      best_distance = d;
      nearest = i;
    }
  }
  if (best_distance < kEps) {
    return false;
  }
  const double scale = std::min(1.0, config_.step / best_distance);
  const Pt point(
    x_[nearest] + (sample.first - x_[nearest]) * scale,
    y_[nearest] + (sample.second - y_[nearest]) * scale);

  const double edge = space_->edgeCost(
    Pt{x_[nearest], y_[nearest]}, point, &budget);
  if (!std::isfinite(edge)) {
    return false;
  }
  budget.take(n_);
  std::vector<double> distances(n_);
  double min_distance = HUGE_VAL;
  for (int i = 0; i < n_; ++i) {
    distances[i] = std::hypot(x_[i] - point.first, y_[i] - point.second);
    min_distance = std::min(min_distance, distances[i]);
  }
  if (min_distance < kEps) {
    return false;
  }
  const double radius = std::min(15.0,
    std::max(1.1 * config_.step,
      config_.eta * std::sqrt(
        std::log(static_cast<double>(std::max(2, n_))) / n_)));
  std::vector<int> near;
  for (int i = 0; i < n_; ++i) {
    if (distances[i] <= radius) {
      near.push_back(i);
    }
  }
  if (static_cast<int>(near.size()) > config_.near_limit) {
    std::partial_sort(near.begin(), near.begin() + config_.near_limit, near.end(),
      [&](int l, int r) {return distances[l] < distances[r];});
    near.resize(config_.near_limit);
  }

  int parent = nearest;
  double best_cost = cost_[nearest] + edge;
  for (const int i : near) {
    const double lower = cost_[i] + distances[i];
    if (lower >= best_cost - kEps) {
      continue;
    }
    const double candidate =
      cost_[i] + space_->edgeCost(Pt{x_[i], y_[i]}, point, &budget);
    if (candidate < best_cost - kEps) {
      parent = i;
      best_cost = candidate;
    }
  }
  budget.take();
  const int index = n_;
  x_[index] = point.first;
  y_[index] = point.second;
  cost_[index] = best_cost;
  parent_[index] = parent;
  children_[index].clear();
  children_[parent].push_back(index);
  n_++;
  budget.peak_nodes = std::max(budget.peak_nodes, n_);

  for (const int i : near) {
    if (i == parent || best_cost + distances[i] >= cost_[i] - kEps) {
      continue;
    }
    const double candidate = best_cost + space_->edgeCost(
      point, Pt{x_[i], y_[i]}, &budget);
    if (candidate >= cost_[i] - kEps) {
      continue;
    }
    reparent(i, index, candidate, budget);
  }
  return true;
}

void BoundedInformedRRT::reparent(
  int child, int parent, double cost, Budget & budget)
{
  if (cost > cost_[child] + kEps) {
    return;
  }
  std::vector<int> descendants;
  std::vector<int> pending{child};
  while (!pending.empty()) {
    budget.take();
    const int node = pending.back();
    pending.pop_back();
    descendants.push_back(node);
    pending.insert(pending.end(), children_[node].begin(), children_[node].end());
  }
  budget.take(static_cast<int64_t>(descendants.size()));
  const double delta = cost - cost_[child];
  auto & old = children_[parent_[child]];
  old.erase(std::remove(old.begin(), old.end(), child), old.end());
  parent_[child] = parent;
  children_[parent].push_back(child);
  for (const int node : descendants) {
    cost_[node] += delta;
  }
}

void BoundedInformedRRT::shortcutNodes(Budget & budget)
{
  // R2-04: the non-increasing reparent criterion compares tree g values
  // against fresh edge costs; with stale g (refresh failed / not yet run)
  // an "equal-cost" shortcut could silently increase the true cost, so
  // shortcuts are disabled until pruneInvalid certifies the costs.
  if (!costs_fresh_) {
    return;
  }
  const int count = std::min(n_, config_.connector_limit);
  budget.take(count);
  const auto nodes = sampleSubset(shortcut_rng_, n_, count);
  budget.shortcut_candidates += count;
  for (const int node : nodes) {
    budget.take();
    const int parent = parent_[node];
    if (parent <= 0) {
      continue;
    }
    std::vector<int> ancestors;
    for (int a = parent_[parent]; a >= 0; a = parent_[a]) {
      budget.take();
      ancestors.push_back(a);
    }
    for (auto it = ancestors.rbegin(); it != ancestors.rend(); ++it) {
      budget.take();
      budget.shortcut_checks++;
      const double cost = cost_[*it] + space_->edgeCost(
        Pt{x_[node], y_[node]}, Pt{x_[*it], y_[*it]}, &budget);
      if (cost <= cost_[node] + kEps) {
        reparent(node, *it, cost, budget);
        budget.shortcut_reparents++;
        break;
      }
    }
  }
}

std::vector<int> BoundedInformedRRT::connectionCandidates(
  const Pt & query, double best_cost, Budget & budget)
{
  const int count = std::min(n_, config_.connector_limit);
  budget.take(2 * count);
  budget.connector_candidates += count;
  const auto candidates = sampleSubset(connector_rng_, n_, count);
  std::vector<int> eligible;
  eligible.reserve(candidates.size());
  for (const int i : candidates) {
    const double lower = cost_[i] +
      std::hypot(x_[i] - query.first, y_[i] - query.second);
    if (lower < best_cost - kEps) {
      eligible.push_back(i);
    }
  }
  return eligible;
}

void BoundedInformedRRT::checkpoint(
  const Pt & query, Budget & budget, int iteration,
  std::vector<Pt> & best, double & best_cost, int & first)
{
  if (config_.reuse_iterations) {
    shortcutNodes(budget);
  }
  for (const int i : connectionCandidates(query, best_cost, budget)) {
    budget.take();
    if (cost_[i] + std::hypot(x_[i] - query.first, y_[i] - query.second) >=
      best_cost - kEps)
    {
      continue;
    }
    budget.connector_checks++;
    const double total = cost_[i] + space_->edgeCost(
      query, Pt{x_[i], y_[i]}, &budget);
    if (total >= best_cost - kEps) {
      continue;
    }
    const int terminal = i;
    std::vector<Pt> candidate{query};
    for (int node = i; node >= 0; node = parent_[node]) {
      budget.take();
      candidate.push_back(Pt{x_[node], y_[node]});
    }
    std::vector<Pt> pruned;
    if (!space_->prune(candidate, pruned, &budget)) {
      continue;
    }
    if (static_cast<int>(pruned.size()) > config_.path_limit) {
      continue;
    }
    const double value = space_->pathCost(pruned, &budget);
    if (value < best_cost - kEps) {
      best = std::move(pruned);
      best_cost = value;
      last_cost_ = value;
      solution_terminal_ = terminal;
      if (first < 0) {
        first = iteration;
      }
      if (config_.reuse_iterations) {
        return;
      }
    }
  }
}

bool BoundedInformedRRT::connectNear(
  const Pt & query, Budget & budget, std::vector<Pt> & out)
{
  last_reason_ = nullptr;
  const int count = std::min(n_, config_.recovery_near_limit);
  budget.take(2LL * n_ +
    2LL * count * std::max(1,
    static_cast<int>(std::ceil(std::log2(count + 1)))));
  std::vector<std::pair<double, int>> order(n_);
  for (int i = 0; i < n_; ++i) {
    order[i] = {std::hypot(x_[i] - query.first, y_[i] - query.second), i};
  }
  const int k = std::min(count, n_);
  std::partial_sort(order.begin(), order.begin() + k, order.end());
  order.resize(k);
  budget.near_recovery_candidates += k;
  std::vector<std::pair<double, int>> connections;
  for (const auto & [d, node] : order) {
    (void)d;
    budget.take();
    budget.near_recovery_checks++;
    const double total = cost_[node] + space_->edgeCost(
      query, Pt{x_[node], y_[node]}, &budget);
    if (std::isfinite(total)) {
      connections.push_back({total, node});
    }
  }
  std::sort(connections.begin(), connections.end());
  for (const auto & [total, terminal] : connections) {
    (void)total;
    std::vector<Pt> path{query};
    for (int node = terminal; node >= 0; node = parent_[node]) {
      budget.take();
      path.push_back(Pt{x_[node], y_[node]});
    }
    std::vector<Pt> pruned;
    if (!space_->prune(path, pruned, &budget)) {
      continue;
    }
    if (static_cast<int>(pruned.size()) > config_.path_limit) {
      continue;
    }
    const double cost = space_->pathCost(pruned, &budget);
    if (std::isfinite(cost)) {
      last_cost_ = cost;
      solution_terminal_ = terminal;
      out = std::move(pruned);
      return true;
    }
  }
  return false;
}

bool BoundedInformedRRT::search(
  const Pt & query, Budget & budget, int iterations,
  std::vector<Pt> & out, const std::vector<Pt> * incumbent)
{
  // costs captured under an older world revision are not certified anymore
  costs_fresh_ = false;
  last_reason_ = nullptr;
  last_cost_ = HUGE_VAL;
  solution_terminal_ = -1;
  std::vector<Pt> best;
  double best_cost = HUGE_VAL;
  int first = -1;
  const double c_min = std::hypot(
    query.first - target_.first, query.second - target_.second);
  const Pt center((query.first + target_.first) * 0.5,
    (query.second + target_.second) * 0.5);
  const double angle = std::atan2(
    query.second - target_.second, query.first - target_.first);
  const double ca = std::cos(angle), sa = std::sin(angle);

  try {
    if (!space_->pointFree(query, &budget) || !space_->pointFree(target_, &budget)) {
      last_reason_ = "invalid_input";
      return false;
    }
    budget.peak_nodes = std::max(budget.peak_nodes, n_);
    bool seeded = false;
    if (incumbent &&
      static_cast<int>(incumbent->size()) <= config_.path_limit &&
      space_->pathValid(*incumbent, &query, &target_, &budget))
    {
      best_cost = space_->pathCost(*incumbent, &budget);
      best = *incumbent;
      first = 0;
      last_cost_ = best_cost;
      seeded = true;
    }
    checkpoint(query, budget, 0, best, best_cost, first);
    const double lower_bound =
      c_min * (config_.reuse_iterations ? space_->minMultiplier(&budget) : 1.0);
    if (best_cost <= lower_bound + kEps) {
      out = best;
      return true;
    }
    for (int iteration = 0; iteration < iterations; ++iteration) {
      const int refine_limit = seeded ? iterations : config_.refine_iterations;
      if (first >= 0 && iteration - first >= refine_limit) {
        break;
      }
      if (n_ >= config_.node_limit) {
        const int removed = config_.reuse_iterations ?
          recycle(query, best.empty() ? nullptr : &best, budget) : 0;
        if (!removed) {
          last_reason_ = "node_limit";
          break;
        }
      }
      budget.take();
      budget.search_iterations++;
      Pt sample;
      if (uniform01(rng_) < config_.goal_bias) {
        sample = query;
      } else if (std::isfinite(best_cost) && best_cost > c_min + kEps) {
        const double radius = std::sqrt(uniform01(rng_));
        const double theta = 2.0 * M_PI * uniform01(rng_);
        const double ex = best_cost * 0.5 * radius * std::cos(theta);
        const double ey = std::sqrt(std::max(
          0.0, best_cost * best_cost - c_min * c_min)) * 0.5 * radius * std::sin(theta);
        sample = Pt(center.first + ex * ca - ey * sa,
          center.second + ex * sa + ey * ca);
      } else {
        const double res = space_->costmap()->getResolution();
        const double ox = space_->costmap()->getOriginX();
        const double oy = space_->costmap()->getOriginY();
        const double w = space_->costmap()->getSizeInCellsX() * res;
        const double h = space_->costmap()->getSizeInCellsY() * res;
        sample = Pt(
          ox + (0.5 * res) + uniform01(rng_) * (w - res),
          oy + (0.5 * res) + uniform01(rng_) * (h - res));
      }
      extend(sample, budget);
      if (iteration % 25 == 24) {
        checkpoint(query, budget, iteration + 1, best, best_cost, first);
      }
    }
    checkpoint(query, budget, iterations, best, best_cost, first);
  } catch (const BudgetExceeded & e) {
    last_reason_ = budget.reason ? budget.reason : e.what();
  }
  if (best.empty()) {
    return false;
  }
  out = std::move(best);
  return true;
}

int BoundedInformedRRT::pruneInvalid(Budget & budget)
{
  // Atomic dynamic maintenance pass (review R2-02/R2-04):
  //  - DFS from the root keeping children reachable through valid edges;
  //  - fresh root-path costs computed in a LOCAL array with
  //    new_g[child] = new_g[parent] + fresh_edge (never reads stale cost_);
  //  - everything is committed only after the full traversal succeeds, so a
  //    budget interruption leaves the tree completely untouched;
  //  - the cost commit happens even when nothing was removed (the main
  //    "cost field changed" use case).
  const int n = n_;
  budget.take(2LL * n);
  std::vector<int> keep_order;
  keep_order.reserve(n);
  std::vector<double> new_g(n, 0.0);
  std::vector<int> stack{0};
  std::vector<bool> queued(n, false);
  queued[0] = true;
  try {
    while (!stack.empty()) {
      budget.take();
      const int node = stack.back();
      stack.pop_back();
      keep_order.push_back(node);
      for (const int child : children_[node]) {
        if (queued[child]) {
          continue;
        }
        // edgeCost == validity + additive cost in one DDA pass
        const double edge = space_->edgeCost(
          Pt{x_[node], y_[node]}, Pt{x_[child], y_[child]}, &budget);
        if (!std::isfinite(edge)) {
          continue;  // branch invalidated by the current world
        }
        queued[child] = true;
        new_g[child] = new_g[node] + edge;
        stack.push_back(child);
      }
    }
    budget.take(2LL * n + config_.node_limit);
  } catch (const BudgetExceeded &) {
    // Interruption must not leave a half-maintained tree: the traversal and
    // the cost refresh are discarded together and costs stay stale, which
    // disables cost-based shortcuts until a successful refresh (R2-04).
    costs_fresh_ = false;
    throw;
  }

  const int remaining = static_cast<int>(keep_order.size());
  const int removed = n - remaining;
  if (removed == 0) {
    // commit the refreshed costs in place (no compaction needed)
    for (int i = 0; i < remaining; ++i) {
      cost_[keep_order[i]] = new_g[keep_order[i]];
    }
    costs_fresh_ = true;
    return 0;
  }

  std::vector<int> remap(n, -1);
  for (int i = 0; i < remaining; ++i) {
    remap[keep_order[i]] = i;
  }
  std::vector<double> nx(config_.node_limit), ny(config_.node_limit),
    ncost(config_.node_limit);
  std::vector<int> nparent(config_.node_limit, -1);
  for (int i = 0; i < remaining; ++i) {
    const int src = keep_order[i];
    nx[i] = x_[src];
    ny[i] = y_[src];
    ncost[i] = new_g[src];
    nparent[i] = parent_[src] >= 0 ? remap[parent_[src]] : -1;
  }
  x_ = std::move(nx);
  y_ = std::move(ny);
  cost_ = std::move(ncost);
  parent_ = std::move(nparent);
  for (int i = 0; i < config_.node_limit; ++i) {
    children_[i].clear();
  }
  for (int i = 1; i < remaining; ++i) {
    children_[parent_[i]].push_back(i);
  }
  if (solution_terminal_ >= 0) {
    solution_terminal_ = remap[solution_terminal_];
  }
  n_ = remaining;
  budget.pruned_nodes += removed;
  costs_fresh_ = true;
  return removed;
}

}  // namespace nav2_colregs_vo_skeleton_planner
