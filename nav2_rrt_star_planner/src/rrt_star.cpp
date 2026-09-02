#include "nav2_rrt_star_planner/rrt_star.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace nav2_rrt_star_planner
{

RRTStar::RRTStar(
  double step_size,
  int max_iterations,
  double goal_bias,
  double goal_threshold,
  double safety_dist,
  double cost_weight,
  int max_optimize_iters,
  double eta,
  bool use_informed_sampling)
: step_size_(step_size),
  max_iterations_(max_iterations),
  goal_bias_(goal_bias),
  goal_threshold_(goal_threshold),
  safety_dist_(safety_dist),
  cost_weight_(cost_weight),
  max_optimize_iters_(max_optimize_iters),
  eta_(eta),
  use_informed_sampling_(use_informed_sampling),
  c_best_(std::numeric_limits<double>::infinity()),
  c_min_(0.0),
  ellipse_center_x_(0.0),
  ellipse_center_y_(0.0),
  ellipse_cos_(1.0),
  ellipse_sin_(0.0),
  rng_(std::random_device{}()),
  goal_reached_(false),
  best_goal_node_idx_(-1)
{
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool RRTStar::planPath(
  double start_x, double start_y,
  double goal_x, double goal_y,
  const nav2_costmap_2d::Costmap2D * costmap,
  std::vector<RRTStarNode> & path_nodes)
{
  tree_.clear();
  goal_reached_ = false;
  best_goal_node_idx_ = -1;
  c_best_ = std::numeric_limits<double>::infinity();

  // Informed-ellipse geometry for this segment (foci at start and goal).
  c_min_ = std::hypot(goal_x - start_x, goal_y - start_y);
  ellipse_center_x_ = 0.5 * (start_x + goal_x);
  ellipse_center_y_ = 0.5 * (start_y + goal_y);
  const double axis = std::atan2(goal_y - start_y, goal_x - start_x);
  ellipse_cos_ = std::cos(axis);
  ellipse_sin_ = std::sin(axis);

  // Root at start.
  RRTStarNode root;
  root.x = start_x;
  root.y = start_y;
  root.parent_idx = -1;
  root.cost_from_root = 0.0;
  tree_.push_back(root);

  // Sampling bounds from costmap size.
  double min_x, max_x, min_y, max_y;
  {
    const unsigned int w = costmap->getSizeInCellsX();
    const unsigned int h = costmap->getSizeInCellsY();
    double mx, my;
    costmap->mapToWorld(0, 0, mx, my);
    min_x = mx - 10.0;
    min_y = my - 10.0;
    costmap->mapToWorld(w, h, mx, my);
    max_x = mx + 10.0;
    max_y = my + 10.0;

    // Extend bounds to include start and goal.
    min_x = std::min(min_x, start_x - 5.0);
    max_x = std::max(max_x, start_x + 5.0);
    min_y = std::min(min_y, start_y - 5.0);
    max_y = std::max(max_y, start_y + 5.0);
    min_x = std::min(min_x, goal_x - 5.0);
    max_x = std::max(max_x, goal_x + 5.0);
    min_y = std::min(min_y, goal_y - 5.0);
    max_y = std::max(max_y, goal_y + 5.0);
  }

  int total_iters = max_iterations_;
  for (int iter = 0; iter < total_iters; ++iter) {
    // Once a solution exists, extend the budget to keep optimizing (rewiring,
    // best-parent selection, best-goal competition) for max_optimize_iters_
    // more iterations. Idempotent assignment; the old guard
    // `iter >= max_iterations_` was unreachable because total_iters capped
    // the loop at max_iterations_, so the optimization phase never ran.
    if (goal_reached_) {
      total_iters = max_iterations_ + max_optimize_iters_;
    }
    // Keep the informed ellipse fed: seed c_best_ from the fallback chord
    // (tree chain to a line-of-sight node + chord to goal) so sampling is
    // focused even when the tree never reaches the goal region.
    if (use_informed_sampling_ &&
      (iter % 50 == 0 || !std::isfinite(c_best_)))
    {
      refreshBestCost(goal_x, goal_y, costmap);
    }

    // 1) Random sample.
    double sx, sy;
    randomSample(sx, sy, goal_x, goal_y, min_x, max_x, min_y, max_y);

    // 2) Nearest node.
    int nearest = nearestNode(sx, sy);

    // 3) Steer.
    double new_x, new_y;
    steer(nearest, sx, sy, new_x, new_y);

    // 4) Collision check.
    if (!collisionFree(tree_[nearest].x, tree_[nearest].y, new_x, new_y, costmap)) {
      continue;
    }

    // 5) Find near nodes.
    auto near = findNear(new_x, new_y);

    // 6) Choose best parent among near nodes.
    int best_parent = nearest;
    double best_cost = tree_[nearest].cost_from_root +
      edgeCost(tree_[nearest].x, tree_[nearest].y, new_x, new_y, costmap);

    for (int idx : near) {
      if (!collisionFree(tree_[idx].x, tree_[idx].y, new_x, new_y, costmap)) {
        continue;
      }
      double cost = tree_[idx].cost_from_root +
        edgeCost(tree_[idx].x, tree_[idx].y, new_x, new_y, costmap);
      if (cost < best_cost) {
        best_parent = idx;
        best_cost = cost;
      }
    }

    // 7) Add node.
    RRTStarNode node;
    node.x = new_x;
    node.y = new_y;
    node.parent_idx = best_parent;
    node.cost_from_root = best_cost;
    int new_idx = static_cast<int>(tree_.size());
    tree_.push_back(node);

    // 8) Rewire near nodes.
    rewire(new_idx, near, costmap);

    // 9) Check goal. Compete on the TOTAL projected path cost including the
    // final candidate->goal connection edge; comparing bare root costs lets
    // a node nearer the root but farther from the goal win and lengthen the
    // extracted path.
    double dist_to_goal = std::hypot(new_x - goal_x, new_y - goal_y);
    if ((dist_to_goal <= goal_threshold_) &&
        collisionFree(new_x, new_y, goal_x, goal_y, costmap))
    {
      const double total_cost =
        best_cost + edgeCost(new_x, new_y, goal_x, goal_y, costmap);
      if (!goal_reached_) {
        best_goal_node_idx_ = new_idx;
        goal_reached_ = true;
        c_best_ = total_cost;
      } else {
        const double best_total =
          tree_[best_goal_node_idx_].cost_from_root +
          edgeCost(
          tree_[best_goal_node_idx_].x, tree_[best_goal_node_idx_].y,
          goal_x, goal_y, costmap);
        if (total_cost < best_total) {
          best_goal_node_idx_ = new_idx;
          c_best_ = total_cost;
        }
      }
    }
  }

  if (!goal_reached_) {
    // Approximate: find tree node closest to goal that has a collision-free
    // connection to goal.
    double best_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < static_cast<int>(tree_.size()); ++i) {
      double d = std::hypot(tree_[i].x - goal_x, tree_[i].y - goal_y);
      if (d < best_dist && collisionFree(tree_[i].x, tree_[i].y, goal_x, goal_y, costmap)) {
        best_dist = d;
        best_goal_node_idx_ = i;
      }
    }
    if (best_goal_node_idx_ < 0) {
      return false;
    }
  }

  const auto & selected_node = tree_[best_goal_node_idx_];
  if (selected_node.x != goal_x || selected_node.y != goal_y) {
    RRTStarNode goal_node;
    goal_node.x = goal_x;
    goal_node.y = goal_y;
    goal_node.parent_idx = best_goal_node_idx_;
    goal_node.cost_from_root = selected_node.cost_from_root +
      edgeCost(selected_node.x, selected_node.y, goal_x, goal_y, costmap);
    best_goal_node_idx_ = static_cast<int>(tree_.size());
    tree_.push_back(goal_node);
  }

  // Extract path by walking parent pointers.
  path_nodes.clear();
  int idx = best_goal_node_idx_;
  while (idx >= 0) {
    path_nodes.push_back(tree_[idx]);
    idx = tree_[idx].parent_idx;
  }
  std::reverse(path_nodes.begin(), path_nodes.end());

  return true;
}

void RRTStar::seedForTesting(uint32_t seed)
{
  rng_.seed(seed);
}

void RRTStar::prunePath(
  std::vector<RRTStarNode> & path,
  const nav2_costmap_2d::Costmap2D * costmap)
{
  if (path.size() <= 2) {
    return;
  }

  std::vector<RRTStarNode> pruned;
  pruned.push_back(path.front());
  size_t i = 0;

  while (i < path.size() - 1) {
    // Greedy: try to connect to the farthest collision-free node.
    for (size_t j = path.size() - 1; j > i; --j) {
      if (collisionFree(path[i].x, path[i].y, path[j].x, path[j].y, costmap)) {
        pruned.push_back(path[j]);
        i = j;
        break;
      }
    }
    if (i >= path.size() - 1) {
      break;
    }
    // Fallback: accept next.
    ++i;
    if (i < path.size()) {
      pruned.push_back(path[i]);
    }
  }

  path = pruned;
}

// ---------------------------------------------------------------------------
// Sampling
// ---------------------------------------------------------------------------

void RRTStar::refreshBestCost(
  double goal_x, double goal_y,
  const nav2_costmap_2d::Costmap2D * costmap)
{
  if (goal_reached_) {
    const auto & node = tree_[best_goal_node_idx_];
    const double cand =
      node.cost_from_root + edgeCost(node.x, node.y, goal_x, goal_y, costmap);
    c_best_ = std::min(c_best_, cand);
    return;
  }

  // Distance-ordered scan for the nearest line-of-sight node: with a cape
  // occluding the direct corridor the nearest LOS node measured on the 5 km
  // CCS scenario ranked ~246th, so a small fixed candidate set would never
  // activate the ellipse. Bounded to keep the refresh cheap early on.
  std::vector<int> order(tree_.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(
    order.begin(), order.end(),
    [this, goal_x, goal_y](int i, int j) {
      const double di = std::hypot(tree_[i].x - goal_x, tree_[i].y - goal_y);
      const double dj = std::hypot(tree_[j].x - goal_x, tree_[j].y - goal_y);
      return di < dj;
    });
  const size_t limit = std::min(order.size(), static_cast<size_t>(400));
  for (size_t k = 0; k < limit; ++k) {
    const int i = order[k];
    if (!collisionFree(tree_[i].x, tree_[i].y, goal_x, goal_y, costmap)) {
      continue;
    }
    const double cand = tree_[i].cost_from_root +
      edgeCost(tree_[i].x, tree_[i].y, goal_x, goal_y, costmap);
    c_best_ = std::min(c_best_, cand);
    break;
  }
}

void RRTStar::randomSample(
  double & x, double & y,
  double goal_x, double goal_y,
  double min_x, double max_x, double min_y, double max_y)
{
  std::uniform_real_distribution<double> dist(0.0, 1.0);
  if (dist(rng_) < goal_bias_) {
    x = goal_x;
    y = goal_y;
  } else if (use_informed_sampling_ && c_min_ < c_best_ &&
    std::isfinite(c_best_))
  {
    // Uniform unit-disk sample scaled to the informed ellipse, rotated to
    // the start-goal axis and translated to its center.
    constexpr double kTwoPi = 6.28318530717958647692;
    const double rr = std::sqrt(dist(rng_));
    const double th = kTwoPi * dist(rng_);
    const double a = 0.5 * c_best_;
    const double c = 0.5 * c_min_;
    const double b = std::max(std::sqrt(std::max(a * a - c * c, 0.0)), 1e-6);
    const double ex = a * rr * std::cos(th);
    const double ey = b * rr * std::sin(th);
    x = ellipse_center_x_ + ex * ellipse_cos_ - ey * ellipse_sin_;
    y = ellipse_center_y_ + ex * ellipse_sin_ + ey * ellipse_cos_;
  } else {
    x = std::uniform_real_distribution<double>(min_x, max_x)(rng_);
    y = std::uniform_real_distribution<double>(min_y, max_y)(rng_);
  }
}

// ---------------------------------------------------------------------------
// Tree search
// ---------------------------------------------------------------------------

int RRTStar::nearestNode(double x, double y)
{
  int best = 0;
  double best_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < tree_.size(); ++i) {
    double d = std::hypot(tree_[i].x - x, tree_[i].y - y);
    if (d < best_dist) {
      best_dist = d;
      best = static_cast<int>(i);
    }
  }
  return best;
}

std::vector<int> RRTStar::findNear(double x, double y)
{
  std::vector<int> result;

  // Classic RRT* radius: eta * (log(n)/n)^(1/d), clamped.
  const size_t n = tree_.size();
  double radius = eta_ * std::pow(std::log(static_cast<double>(n)) / n, 1.0 / 2.0);
  radius = std::max(radius, step_size_ * 1.1);
  radius = std::min(radius, 15.0);

  double r2 = radius * radius;

  // Nearest node first (always included).
  int nn = nearestNode(x, y);
  if (nn >= 0) {
    result.push_back(nn);
  }

  // Linear scan for neighbors within radius.
  for (size_t i = 0; i < tree_.size(); ++i) {
    if (static_cast<int>(i) == nn) continue;
    double dx = tree_[i].x - x;
    double dy = tree_[i].y - y;
    if (dx * dx + dy * dy < r2) {
      result.push_back(static_cast<int>(i));
    }
  }
  return result;
}

void RRTStar::steer(
  int from_idx, double toward_x, double toward_y,
  double & new_x, double & new_y)
{
  double dx = toward_x - tree_[from_idx].x;
  double dy = toward_y - tree_[from_idx].y;
  double dist = std::hypot(dx, dy);
  if (dist <= step_size_) {
    new_x = toward_x;
    new_y = toward_y;
  } else {
    new_x = tree_[from_idx].x + dx / dist * step_size_;
    new_y = tree_[from_idx].y + dy / dist * step_size_;
  }
}

// ---------------------------------------------------------------------------
// Collision checking
// ---------------------------------------------------------------------------

bool RRTStar::collisionFree(
  double x1, double y1, double x2, double y2,
  const nav2_costmap_2d::Costmap2D * costmap)
{
  const double res = costmap->getResolution();
  const double seg_len = std::hypot(x2 - x1, y2 - y1);
  const double step = res * 0.5;  // oversample 2x for safety

  if (seg_len < 1e-9) {
    unsigned int mx, my;
    costmap->worldToMap(x1, y1, mx, my);
    return costmap->getCost(mx, my) < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }

  int n_samples = std::max(1, static_cast<int>(seg_len / step));
  for (int i = 0; i <= n_samples; ++i) {
    double t = static_cast<double>(i) / n_samples;
    double sx = x1 + t * (x2 - x1);
    double sy = y1 + t * (y2 - y1);

    // Check footprint: safety_dist_ radius around the point.
    const int scan_radius = static_cast<int>(std::ceil(safety_dist_ / res));
    unsigned int mx0, my0;
    if (!costmap->worldToMap(sx, sy, mx0, my0)) {
      return false;
    }
    for (int drx = -scan_radius; drx <= scan_radius; ++drx) {
      for (int dry = -scan_radius; dry <= scan_radius; ++dry) {
        if (drx * drx + dry * dry > scan_radius * scan_radius) continue;
        int cx = static_cast<int>(mx0) + drx;
        int cy = static_cast<int>(my0) + dry;
        if (static_cast<unsigned int>(cx) >= costmap->getSizeInCellsX() ||
            static_cast<unsigned int>(cy) >= costmap->getSizeInCellsY())
        {
          return false;
        }
        if (costmap->getCost(cx, cy) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          return false;
        }
      }
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Cost function
// ---------------------------------------------------------------------------

double RRTStar::edgeCost(
  double x1, double y1, double x2, double y2,
  const nav2_costmap_2d::Costmap2D * costmap)
{
  const double length = std::hypot(x2 - x1, y2 - y1);
  if (length < 1e-9) {
    return 0.0;
  }

  const double res = costmap->getResolution();
  int n_samples = std::max(1, static_cast<int>(length / res));
  double sum_cost = 0.0;

  for (int i = 0; i < n_samples; ++i) {
    double t = (i + 0.5) / n_samples;
    double sx = x1 + t * (x2 - x1);
    double sy = y1 + t * (y2 - y1);
    unsigned int mx, my;
    if (costmap->worldToMap(sx, sy, mx, my)) {
      sum_cost += costmap->getCost(mx, my);
    } else {
      sum_cost += 254.0;
    }
  }

  double avg_cost = sum_cost / n_samples / 254.0;
  return length * (1.0 + cost_weight_ * avg_cost);
}

// ---------------------------------------------------------------------------
// Rewire
// ---------------------------------------------------------------------------

void RRTStar::rewire(
  int new_idx, const std::vector<int> & near,
  const nav2_costmap_2d::Costmap2D * costmap)
{
  auto & node = tree_[new_idx];

  // Child adjacency for this call, kept consistent across re-parentings so
  // cost deltas can be propagated to whole subtrees.
  std::vector<std::vector<int>> children(tree_.size());
  for (size_t j = 0; j < tree_.size(); ++j) {
    const int p = tree_[j].parent_idx;
    if (p >= 0) {
      children[p].push_back(static_cast<int>(j));
    }
  }

  for (int idx : near) {
    if (idx == node.parent_idx) {
      continue;
    }

    double new_cost = node.cost_from_root +
      edgeCost(node.x, node.y, tree_[idx].x, tree_[idx].y, costmap);

    if (new_cost >= tree_[idx].cost_from_root) {
      continue;
    }

    if (!collisionFree(node.x, node.y, tree_[idx].x, tree_[idx].y, costmap)) {
      continue;
    }

    // Rewire: idx now reaches through new_idx at lower cost. Propagate the
    // exact cost delta to idx's subtree so descendant costs, later
    // best-parent selections and the best-goal comparison stay consistent
    // (and the cost-dominance cycle guard stays sound).
    const int old_parent = tree_[idx].parent_idx;
    const double delta = new_cost - tree_[idx].cost_from_root;
    tree_[idx].parent_idx = new_idx;
    tree_[idx].cost_from_root = new_cost;
    children[new_idx].push_back(idx);
    if (old_parent >= 0) {
      auto & siblings = children[old_parent];
      siblings.erase(std::remove(siblings.begin(), siblings.end(), idx), siblings.end());
    }

    std::vector<int> stack{idx};
    while (!stack.empty()) {
      const int cur = stack.back();
      stack.pop_back();
      for (const int child : children[cur]) {
        tree_[child].cost_from_root += delta;
        stack.push_back(child);
      }
    }
  }
}

}  // namespace nav2_rrt_star_planner
