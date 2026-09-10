#include "nav2_colregs_vo_skeleton_planner/skeleton_space.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"

namespace nav2_colregs_vo_skeleton_planner
{

namespace
{

double clockNow()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool segmentsIntersect(
  double ax, double ay, double bx, double by,
  double cx, double cy, double dx, double dy)
{
  const double eps = kEps;
  auto cross = [](double ux, double uy, double vx, double vy) {
      return ux * vy - uy * vx;
    };
  auto orientation = [&](double px, double py, double qx, double qy,
      double rx, double ry) {
      return cross(qx - px, qy - py, rx - px, ry - py);
    };
  auto onSegment = [&](double px, double py, double qx, double qy,
      double rx, double ry) {
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

using CellList = std::vector<std::pair<int, int>>;

}  // namespace

Budget::Budget(int64_t limit, double time_limit_s)
: limit(limit),
  deadline(time_limit_s > 0.0 ? clockNow() + time_limit_s : HUGE_VAL)
{
}

void Budget::take(int64_t amount)
{
  if (reason != nullptr) {
    throw BudgetExceeded(reason);
  }
  if (clockNow() >= deadline) {
    reason = "time_limit";
  } else if (work + amount > limit) {
    reason = "work_limit";
  }
  if (reason != nullptr) {
    throw BudgetExceeded(reason);
  }
  work += amount;
}

Space::Space(
  const nav2_costmap_2d::Costmap2D * costmap,
  double safety_dist, double cost_weight)
: costmap_(costmap),
  safety_dist_(safety_dist),
  cost_weight_(cost_weight)
{
  const double res = costmap_->getResolution();
  const int r = static_cast<int>(std::ceil(safety_dist_ / res));
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      if (dx * dx + dy * dy <= r * r) {
        disk_.emplace_back(dy, dx);
      }
    }
  }
}

void Space::beginQuery(uint64_t world_revision)
{
  revision_ = world_revision;
  cache_.clear();
  min_multiplier_valid_ = false;
}

void Space::setBarriers(const std::vector<geometry_msgs::msg::Point> & barriers)
{
  barriers_ = barriers;
}

bool Space::blockedDisk(int cx, int cy) const
{
  for (const auto & off : disk_) {
    const int x = cx + off.second;
    const int y = cy + off.first;
    if (!inside(x, y)) {
      return true;
    }
    const auto cost = costmap_->getCost(x, y);
    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return true;
    }
  }
  return false;
}

double Space::cellCost(int cx, int cy) const
{
  const auto cost = costmap_->getCost(cx, cy);
  if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
    cost == nav2_costmap_2d::NO_INFORMATION)
  {
    return 1.0;
  }
  return static_cast<double>(cost) / 254.0;
}

bool Space::barriersFree(const Pt & a, const Pt & b) const
{
  for (size_t i = 0; i + 1 < barriers_.size(); i += 2) {
    const auto & s = barriers_[i];
    const auto & e = barriers_[i + 1];
    if (segmentsIntersect(a.first, a.second, b.first, b.second,
      s.x, s.y, e.x, e.y))
    {
      return false;
    }
  }
  return true;
}

double Space::minMultiplier(Budget * budget)
{
  if (!min_multiplier_valid_) {
    double best = 1.0;
    const unsigned int w = costmap_->getSizeInCellsX();
    const unsigned int h = costmap_->getSizeInCellsY();
    // R2-06: the full-grid scan is budget-interruptible; on interruption the
    // conservative multiplier 1+w (all cells max cost) is committed so the
    // result never understates the true bound.
    for (unsigned int my = 0; my < h; ++my) {
      for (unsigned int mx = 0; mx < w; ++mx) {
        if (budget && (my * w + mx) % 4096 == 0) {
          budget->take();
        }
        const auto cost = costmap_->getCost(mx, my);
        if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
          cost == nav2_costmap_2d::NO_INFORMATION)
        {
          continue;
        }
        best = std::min(best, static_cast<double>(cost) / 254.0);
      }
    }
    min_multiplier_ = 1.0 + cost_weight_ * best;
    min_multiplier_valid_ = true;
  }
  return min_multiplier_;
}

Space::EdgeResult Space::computeEdge(const Pt & a, const Pt & b, Budget * budget)
{
  const double res = costmap_->getResolution();
  const double ox = costmap_->getOriginX();
  const double oy = costmap_->getOriginY();

  auto endpointCells = [&](double px, double py, CellList & cells) {
      const int cx = static_cast<int>(std::floor(px));
      const int cy = static_cast<int>(std::floor(py));
      const bool dual_x = std::abs(px - cx) < 1e-10;
      const bool dual_y = std::abs(py - cy) < 1e-10;
      for (int i = 0; i <= (dual_x ? 1 : 0); ++i) {
        for (int j = 0; j <= (dual_y ? 1 : 0); ++j) {
          cells.emplace_back(cx - i, cy - j);
        }
      }
    };

  auto visit = [&](const CellList & cells, double dt, double & integral)
    -> bool {
      if (budget) {
        budget->take(static_cast<int64_t>(cells.size()));
      }
      double highest = 0.0;
      for (const auto & c : cells) {
        if (!inside(c.first, c.second) || blockedDisk(c.first, c.second)) {
          return false;
        }
        highest = std::max(highest, cellCost(c.first, c.second));
      }
      integral += dt * highest;
      return true;
    };

  double integral = 0.0;
  // endpoint contact sets (dt = 0)
  CellList ea, eb;
  endpointCells((a.first - ox) / res, (a.second - oy) / res, ea);
  endpointCells((b.first - ox) / res, (b.second - oy) / res, eb);
  if (!visit(ea, 0.0, integral) || !visit(eb, 0.0, integral)) {
    return {false, HUGE_VAL};
  }

  // interior DDA
  double x = (a.first - ox) / res;
  double y = (a.second - oy) / res;
  const double ex = (b.first - ox) / res;
  const double ey = (b.second - oy) / res;
  const double dx = ex - x;
  const double dy = ey - y;
  int ix = static_cast<int>(std::floor(x));
  int iy = static_cast<int>(std::floor(y));
  const int step_x = dx > 0 ? 1 : (dx < 0 ? -1 : 0);
  const int step_y = dy > 0 ? 1 : (dy < 0 ? -1 : 0);
  const bool vertical = std::abs(dx) < 1e-12 &&
    std::abs(x - std::round(x)) < 1e-10;
  const bool horizontal = std::abs(dy) < 1e-12 &&
    std::abs(y - std::round(y)) < 1e-10;
  double tx = step_x ? ((ix + (step_x > 0)) - x) / dx : HUGE_VAL;
  double ty = step_y ? ((iy + (step_y > 0)) - y) / dy : HUGE_VAL;
  const double dtx = step_x ? std::abs(1.0 / dx) : HUGE_VAL;
  const double dty = step_y ? std::abs(1.0 / dy) : HUGE_VAL;
  double t = 0.0;

  auto interiorCells = [&]() {
      CellList cells;
      const int xs[2] = {ix, ix - 1};   // vertical: dual columns
      const int ys[2] = {iy, iy - 1};   // horizontal: dual rows
      if (vertical && horizontal) {
        for (int i = 0; i < 2; ++i) {
          for (int j = 0; j < 2; ++j) {
            cells.emplace_back(xs[i], ys[j]);
          }
        }
      } else if (vertical) {
        cells.emplace_back(ix, iy);
        cells.emplace_back(ix - 1, iy);
      } else if (horizontal) {
        cells.emplace_back(ix, iy);
        cells.emplace_back(ix, iy - 1);
      } else {
        cells.emplace_back(ix, iy);
      }
      return cells;
    };

  while (t < 1.0) {
    const double nxt = std::min(std::min(tx, ty), 1.0);
    if (!visit(interiorCells(), std::max(0.0, nxt - t), integral)) {
      return {false, HUGE_VAL};
    }
    if (nxt >= 1.0) {
      break;
    }
    if (std::abs(tx - ty) < 1e-12) {
      // exact corner: both diagonal neighbours (dual contact, dt = 0)
      CellList corner;
      corner.emplace_back(ix + step_x, iy);
      corner.emplace_back(ix, iy + step_y);
      if (!visit(corner, 0.0, integral)) {
        return {false, HUGE_VAL};
      }
      ix += step_x;
      iy += step_y;
      tx += dtx;
      ty += dty;
    } else if (tx < ty) {
      ix += step_x;
      tx += dtx;
    } else {
      iy += step_y;
      ty += dty;
    }
    t = nxt;
  }

  const double length = std::hypot(b.first - a.first, b.second - a.second);
  return {true, length * (1.0 + cost_weight_ * integral)};
}

bool Space::pointFree(const Pt & p, Budget * budget)
{
  return segmentFree(p, p, budget);
}

bool Space::segmentFree(const Pt & a, const Pt & b, Budget * budget)
{
  if (budget) {
    budget->collision_checks++;
  }
  return edgeCost(a, b, budget) < HUGE_VAL;
}

double Space::edgeCost(const Pt & a, const Pt & b, Budget * budget)
{
  if (budget) {
    budget->cost_checks++;
    budget->take();  // charged on cache hits too, matching the prototype
  }
  const bool finite = std::isfinite(a.first) && std::isfinite(a.second) &&
    std::isfinite(b.first) && std::isfinite(b.second);
  if (!finite) {
    return HUGE_VAL;
  }
  EdgeKey key{a, b};
  if (b < a) {
    key = EdgeKey{b, a};
  }
  auto it = cache_.find(key);
  if (it != cache_.end()) {
    return it->second.free && barriersFree(a, b) ? it->second.cost : HUGE_VAL;
  }
  const EdgeResult result = computeEdge(a, b, budget);
  if (cache_.size() >= static_cast<size_t>(kCacheLimit)) {
    cache_.clear();
  }
  cache_[key] = result;
  return result.free && barriersFree(a, b) ? result.cost : HUGE_VAL;
}

double Space::pathCost(const std::vector<Pt> & path, Budget * budget)
{
  if (path.empty()) {
    return HUGE_VAL;
  }
  if (path.size() == 1) {
    return pointFree(path[0], budget) ? 0.0 : HUGE_VAL;
  }
  double total = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    const double c = edgeCost(path[i - 1], path[i], budget);
    if (c >= HUGE_VAL) {
      return HUGE_VAL;
    }
    total += c;
  }
  return total;
}

bool Space::pathValid(
  const std::vector<Pt> & path, const Pt * start, const Pt * goal,
  Budget * budget)
{
  if (path.empty()) {
    return false;
  }
  for (const auto & p : path) {
    if (!std::isfinite(p.first) || !std::isfinite(p.second)) {
      return false;
    }
  }
  if (start &&
    std::hypot(path.front().first - start->first,
      path.front().second - start->second) > kEps)
  {
    return false;
  }
  if (goal &&
    std::hypot(path.back().first - goal->first,
      path.back().second - goal->second) > kEps)
  {
    return false;
  }
  if (path.size() == 1) {
    return pointFree(path[0], budget);
  }
  for (size_t i = 1; i < path.size(); ++i) {
    if (!segmentFree(path[i - 1], path[i], budget)) {
      return false;
    }
  }
  return true;
}

bool Space::prune(
  const std::vector<Pt> & path, std::vector<Pt> & out, Budget * budget)
{
  out.clear();
  if (!pathValid(path, nullptr, nullptr, budget)) {
    return false;
  }
  std::vector<Pt> points;
  points.reserve(path.size());
  points.push_back(path.front());
  for (size_t i = 1; i < path.size(); ++i) {
    if (std::hypot(points.back().first - path[i].first,
      points.back().second - path[i].second) > kEps)
    {
      points.push_back(path[i]);
    }
  }
  std::vector<double> costs{0.0};
  costs.reserve(points.size());
  for (size_t i = 1; i < points.size(); ++i) {
    costs.push_back(costs.back() + edgeCost(points[i - 1], points[i], budget));
  }
  out.push_back(points.front());
  size_t i = 0;
  while (i < points.size() - 1) {
    bool advanced = false;
    for (size_t j = points.size() - 1; j > i; --j) {
      const double cost = edgeCost(points[i], points[j], budget);
      if (cost <= costs[j] - costs[i] + kEps) {
        out.push_back(points[j]);
        i = j;
        advanced = true;
        break;
      }
    }
    if (!advanced) {
      out.clear();
      return false;
    }
  }
  return true;
}

bool Space::reanchor(
  const Pt & query, const std::vector<Pt> & path, std::vector<Pt> & out,
  Budget * budget, int limit)
{
  out.clear();
  if (path.empty() || !pathValid(path, nullptr, nullptr, budget)) {
    return false;
  }
  if (std::hypot(query.first - path.front().first,
    query.second - path.front().second) <= kEps)
  {
    out.push_back(query);
    out.insert(out.end(), path.begin() + 1, path.end());
    return true;
  }
  if (path.size() == 1) {
    if (segmentFree(query, path.front(), budget)) {
      out = {query, path.front()};
      return true;
    }
    return false;
  }

  struct Candidate
  {
    double distance;
    size_t end;
    Pt point;
  };
  std::vector<Candidate> candidates;
  candidates.reserve(path.size() + 1);
  candidates.push_back(
    {std::hypot(query.first - path.front().first,
      query.second - path.front().second), 0, path.front()});
  std::vector<double> cumulative{0.0};
  cumulative.reserve(path.size());
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    if (budget) {
      budget->take();
    }
    const auto & a = path[i];
    const auto & b = path[i + 1];
    const double dx = b.first - a.first;
    const double dy = b.second - a.second;
    const double len_sq = dx * dx + dy * dy;
    const double t = len_sq < kEps * kEps ? 0.0 :
      std::max(0.0, std::min(1.0,
      ((query.first - a.first) * dx + (query.second - a.second) * dy) / len_sq));
    const Pt point(a.first + t * dx, a.second + t * dy);
    candidates.push_back(
      {std::hypot(query.first - point.first, query.second - point.second),
        i + 1, point});
    cumulative.push_back(cumulative.back() + edgeCost(a, b, budget));
  }
  std::stable_sort(candidates.begin(), candidates.end(),
    [](const Candidate & l, const Candidate & r) {
      if (l.distance != r.distance) {
        return l.distance < r.distance;
      }
      return l.end < r.end;
    });

  std::vector<Pt> best;
  double best_cost = HUGE_VAL;
  const int checked = std::min(limit, static_cast<int>(candidates.size()));
  for (int k = 0; k < checked; ++k) {
    const auto & cand = candidates[k];
    const double connector = edgeCost(query, cand.point, budget);
    if (!std::isfinite(connector)) {
      continue;
    }
    double remaining;
    std::vector<Pt> suffix;
    if (cand.end == 0) {
      suffix = path;
      remaining = cumulative.back();
    } else {
      suffix.push_back(cand.point);
      suffix.insert(suffix.end(), path.begin() + cand.end, path.end());
      remaining = edgeCost(cand.point, path[cand.end], budget) +
        cumulative.back() - cumulative[cand.end];
    }
    const double total = connector + remaining;
    if (total < best_cost - kEps) {
      best.clear();
      best.push_back(query);
      best.insert(best.end(), suffix.begin(), suffix.end());
      best_cost = total;
    }
  }
  if (best.empty()) {
    return false;
  }
  out = std::move(best);
  return true;
}

}  // namespace nav2_colregs_vo_skeleton_planner
