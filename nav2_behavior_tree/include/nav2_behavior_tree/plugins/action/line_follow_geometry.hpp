#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__LINE_FOLLOW_GEOMETRY_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__LINE_FOLLOW_GEOMETRY_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace nav2_behavior_tree
{

struct LinePoint
{
  double x{0.0};
  double y{0.0};
};

struct LinePolyline
{
  std::vector<LinePoint> points;
  std::vector<double> cumulative;
};

struct LineProjection
{
  int segment{0};
  double along{0.0};
  double cross{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double remaining{0.0};
  bool valid{false};
};

inline LinePolyline makePolyline(const std::vector<LinePoint> & points)
{
  LinePolyline line;
  line.points = points;
  line.cumulative.assign(points.size(), 0.0);
  for (std::size_t i = 1; i < points.size(); ++i) {
    const double dx = points[i].x - points[i - 1].x;
    const double dy = points[i].y - points[i - 1].y;
    line.cumulative[i] = line.cumulative[i - 1] + std::hypot(dx, dy);
  }
  return line;
}

inline double polylineLength(const LinePolyline & line)
{
  if (line.cumulative.empty()) {
    return 0.0;
  }
  return line.cumulative.back();
}

inline LineProjection projectSegment(
  const LinePolyline & line, std::size_t index, double x, double y)
{
  LineProjection projection;
  if (index + 1 >= line.points.size()) {
    return projection;
  }
  const auto & a = line.points[index];
  const auto & b = line.points[index + 1];
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length = std::hypot(dx, dy);
  double ratio = 0.0;
  if (length > 1e-6) {
    ratio = ((x - a.x) * dx + (y - a.y) * dy) / (length * length);
    ratio = std::clamp(ratio, 0.0, 1.0);
  }
  projection.valid = true;
  projection.segment = static_cast<int>(index);
  projection.x = a.x + ratio * dx;
  projection.y = a.y + ratio * dy;
  projection.along = line.cumulative[index] + ratio * length;
  projection.yaw = std::atan2(dy, dx);
  projection.remaining = polylineLength(line) - projection.along;
  if (length > 1e-6) {
    projection.cross = (dx * (y - projection.y) - dy * (x - projection.x)) / length;
  }
  return projection;
}

inline LineProjection projectWindow(
  const LinePolyline & line, double x, double y, double hint_along,
  double back_m, double forward_m)
{
  LineProjection best;
  double best_distance = 1e300;
  if (line.points.size() < 2) {
    return best;
  }
  for (std::size_t i = 0; i + 1 < line.points.size(); ++i) {
    if (line.cumulative[i + 1] < hint_along - back_m ||
      line.cumulative[i] > hint_along + forward_m)
    {
      continue;
    }
    const auto candidate = projectSegment(line, i, x, y);
    const double dx = x - candidate.x;
    const double dy = y - candidate.y;
    const double distance = dx * dx + dy * dy;
    if (distance < best_distance) {
      best_distance = distance;
      best = candidate;
    }
  }
  return best;
}

inline LinePoint pointAt(const LinePolyline & line, double along)
{
  LinePoint point;
  if (line.points.empty()) {
    return point;
  }
  if (line.points.size() == 1 || along <= 0.0) {
    return line.points.front();
  }
  const double length = polylineLength(line);
  if (along >= length) {
    return line.points.back();
  }
  for (std::size_t i = 0; i + 1 < line.points.size(); ++i) {
    if (along > line.cumulative[i + 1]) {
      continue;
    }
    const double span = line.cumulative[i + 1] - line.cumulative[i];
    const double ratio = span > 1e-6 ? (along - line.cumulative[i]) / span : 0.0;
    point.x = line.points[i].x + ratio * (line.points[i + 1].x - line.points[i].x);
    point.y = line.points[i].y + ratio * (line.points[i + 1].y - line.points[i].y);
    return point;
  }
  return line.points.back();
}

inline void appendPoint(std::vector<LinePoint> & points, const LinePoint & point)
{
  if (!points.empty()) {
    const auto & last = points.back();
    if (std::hypot(last.x - point.x, last.y - point.y) < 0.5) {
      return;
    }
  }
  points.push_back(point);
}

inline double wrapPi(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline bool isSharpCorner(
  const LinePolyline & line, std::size_t vertex, double min_turn_rad)
{
  if (vertex == 0 || vertex + 1 >= line.points.size()) {
    return false;
  }
  const double dx0 = line.points[vertex].x - line.points[vertex - 1].x;
  const double dy0 = line.points[vertex].y - line.points[vertex - 1].y;
  const double dx1 = line.points[vertex + 1].x - line.points[vertex].x;
  const double dy1 = line.points[vertex + 1].y - line.points[vertex].y;
  if (std::hypot(dx0, dy0) < 1e-3 || std::hypot(dx1, dy1) < 1e-3) {
    return false;
  }
  const double turn = std::abs(
    wrapPi(std::atan2(dy1, dx1) - std::atan2(dy0, dx0)));
  return turn >= min_turn_rad;
}

// A corner vertex is not a required waypoint. Drop samples inside the cut
// and keep only the points one cut-length before and after the vertex, so
// the controller follows the chord instead of the point itself.
inline std::vector<LinePoint> sliceRelaxed(
  const LinePolyline & line, double from_along, double step_m, double corner_cut_m)
{
  std::vector<LinePoint> points;
  const double length = polylineLength(line);
  if (line.points.size() < 2 || step_m <= 0.0) {
    return points;
  }
  const double start = std::clamp(from_along, 0.0, length);
  const double cut = std::max(0.0, corner_cut_m);
  constexpr double kMinTurn = 0.70;
  auto inside_corner = [&](double along) {
    for (std::size_t vertex = 1; vertex + 1 < line.points.size(); ++vertex) {
      if (!isSharpCorner(line, vertex, kMinTurn)) {
        continue;
      }
      if (std::abs(along - line.cumulative[vertex]) < cut - 0.5) {
        return true;
      }
    }
    return false;
  };
  std::vector<std::pair<double, LinePoint>> samples;
  auto add = [&](double along) {
    const double clipped = std::clamp(along, 0.0, length);
    if (clipped + 1e-3 < start || inside_corner(clipped)) {
      return;
    }
    samples.emplace_back(clipped, pointAt(line, clipped));
  };
  add(start);
  for (double along = start + step_m; along < length; along += step_m) {
    add(along);
  }
  add(length);
  for (std::size_t vertex = 1; vertex + 1 < line.points.size(); ++vertex) {
    if (!isSharpCorner(line, vertex, kMinTurn)) {
      if (line.cumulative[vertex] >= start) {
        samples.emplace_back(line.cumulative[vertex], line.points[vertex]);
      }
      continue;
    }
    add(line.cumulative[vertex] - cut);
    add(line.cumulative[vertex] + cut);
  }
  std::sort(
    samples.begin(), samples.end(),
    [](const auto & lhs, const auto & rhs) {return lhs.first < rhs.first;});
  for (const auto & sample : samples) {
    appendPoint(points, sample.second);
  }
  return points;
}

// True when a sharp corner lies just behind or just ahead of along. Used to
// hold curve-following parameters through the turn, then release them on the
// straight.
inline bool cornerNearby(
  const LinePolyline & line, double along, double ahead_m, double behind_m)
{
  constexpr double kMinTurn = 0.70;
  for (std::size_t vertex = 1; vertex + 1 < line.points.size(); ++vertex) {
    if (!isSharpCorner(line, vertex, kMinTurn)) {
      continue;
    }
    const double delta = line.cumulative[vertex] - along;
    if (delta >= -behind_m && delta <= ahead_m) {
      return true;
    }
  }
  return false;
}

// The locked leg ends at a corner the boat does not have to touch. Once the
// nearest reference point is already on the next leg and the boat is still
// in the corner neighborhood, the vertex counts as passed.
inline bool cornerPassed(
  const LinePolyline & line, int locked_segment, const LineProjection & window,
  double boat_x, double boat_y, double release_m)
{
  if (!window.valid || locked_segment < 0 || release_m <= 0.0) {
    return false;
  }
  const std::size_t vertex = static_cast<std::size_t>(locked_segment) + 1;
  if (vertex >= line.points.size() || window.segment != locked_segment + 1) {
    return false;
  }
  if (std::abs(window.cross) > release_m) {
    return false;
  }
  const double past = window.along - line.cumulative[vertex];
  if (past < -5.0 || past > release_m) {
    return false;
  }
  const auto & corner = line.points[vertex];
  return std::hypot(boat_x - corner.x, boat_y - corner.y) <= release_m;
}

// Soften polyline corners so successive heading changes stay under the ALOS
// motion gate. Sharp vertices are replaced by a circular fillet of radius R.
inline std::vector<LinePoint> softenTurning(
  const std::vector<LinePoint> & input, double max_turn_rad, double radius_m,
  double step_m)
{
  if (input.size() < 3 || max_turn_rad <= 0.0 || radius_m <= 0.0) {
    return input;
  }
  const double step = std::max(1.0, step_m);
  std::vector<LinePoint> points;
  points.reserve(input.size() * 2);
  appendPoint(points, input.front());
  for (std::size_t i = 1; i + 1 < input.size(); ++i) {
    const LinePoint & prev = points.back();
    const LinePoint & curr = input[i];
    const LinePoint & next = input[i + 1];
    const double dx0 = curr.x - prev.x;
    const double dy0 = curr.y - prev.y;
    const double dx1 = next.x - curr.x;
    const double dy1 = next.y - curr.y;
    const double len0 = std::hypot(dx0, dy0);
    const double len1 = std::hypot(dx1, dy1);
    if (len0 < 1e-3 || len1 < 1e-3) {
      continue;
    }
    const double yaw0 = std::atan2(dy0, dx0);
    const double yaw1 = std::atan2(dy1, dx1);
    const double turn = wrapPi(yaw1 - yaw0);
    if (std::abs(turn) < max_turn_rad) {
      appendPoint(points, curr);
      continue;
    }
    // Cut equal lengths on both sides, then sample the circular fillet.
    const double half = std::abs(turn) * 0.5;
    const double cut = std::min(
      {radius_m * std::tan(half), len0 * 0.45, len1 * 0.45});
    if (cut < 1.0) {
      appendPoint(points, curr);
      continue;
    }
    const LinePoint enter{
      curr.x - std::cos(yaw0) * cut,
      curr.y - std::sin(yaw0) * cut};
    const LinePoint leave{
      curr.x + std::cos(yaw1) * cut,
      curr.y + std::sin(yaw1) * cut};
    appendPoint(points, enter);
    const double sign = turn >= 0.0 ? 1.0 : -1.0;
    const double nx = -std::sin(yaw0) * sign;
    const double ny = std::cos(yaw0) * sign;
    // Fillet center sits radius away from the entry along the inward normal.
    const double center_x = enter.x + nx * radius_m;
    const double center_y = enter.y + ny * radius_m;
    const double start_ang = std::atan2(enter.y - center_y, enter.x - center_x);
    const double end_ang = std::atan2(leave.y - center_y, leave.x - center_x);
    double sweep = wrapPi(end_ang - start_ang);
    if (sign * sweep < 0.0) {
      sweep += sign * 2.0 * std::acos(-1.0);
    }
    const int samples = std::max(
      2, static_cast<int>(std::ceil(std::abs(sweep) * radius_m / step)));
    for (int s = 1; s < samples; ++s) {
      const double ang = start_ang + sweep * (static_cast<double>(s) / samples);
      appendPoint(
        points, LinePoint{
          center_x + radius_m * std::cos(ang),
          center_y + radius_m * std::sin(ang)});
    }
    appendPoint(points, leave);
  }
  appendPoint(points, input.back());
  return points;
}

// Drop near poses whose bearing from the boat exceeds the motion gate, so the
// controller can look past a kink that currently freezes forward speed.
inline std::vector<LinePoint> skipSharpApproach(
  double boat_x, double boat_y, double boat_yaw,
  const std::vector<LinePoint> & input, double max_bearing_rad, double min_skip_m)
{
  if (input.size() < 2) {
    return input;
  }
  // Drop only the near, sharp head of the path. Never walk keep all the way to
  // the final waypoint just because every far point is also sharp — that used
  // to collapse lawnmower recovery into "boat → survey endpoint".
  const double near = std::max(5.0, min_skip_m);
  std::size_t first_far = input.size() - 1;
  for (std::size_t i = 0; i < input.size(); ++i) {
    if (std::hypot(input[i].x - boat_x, input[i].y - boat_y) >= near) {
      first_far = i;
      break;
    }
  }
  std::size_t keep = first_far;
  for (std::size_t i = first_far; i < input.size(); ++i) {
    const double dx = input[i].x - boat_x;
    const double dy = input[i].y - boat_y;
    const double bearing = wrapPi(std::atan2(dy, dx) - boat_yaw);
    if (std::abs(bearing) <= max_bearing_rad) {
      keep = i;
      break;
    }
  }
  // Cap how far recovery may jump along the polyline (metres of chord length).
  constexpr double kMaxSkipChordM = 80.0;
  double chord = 0.0;
  std::size_t capped = keep;
  for (std::size_t i = first_far; i < keep; ++i) {
    chord += std::hypot(input[i + 1].x - input[i].x, input[i + 1].y - input[i].y);
    if (chord > kMaxSkipChordM) {
      capped = i;
      break;
    }
    capped = i + 1;
  }
  keep = std::min(keep, capped);

  std::vector<LinePoint> points;
  points.push_back(LinePoint{boat_x, boat_y});
  for (std::size_t i = keep; i < input.size(); ++i) {
    appendPoint(points, input[i]);
  }
  if (points.size() < 2 && !input.empty()) {
    appendPoint(points, input[keep]);
  }
  return points;
}

inline std::vector<LinePoint> sliceDense(
  const LinePolyline & line, double from_along, double step_m)
{
  std::vector<LinePoint> points;
  const double length = polylineLength(line);
  if (line.points.empty() || step_m <= 0.0) {
    return points;
  }
  const double start = std::clamp(from_along, 0.0, length);
  std::vector<std::pair<double, LinePoint>> samples;
  samples.emplace_back(start, pointAt(line, start));
  for (double along = start + step_m; along < length; along += step_m) {
    samples.emplace_back(along, pointAt(line, along));
  }
  for (std::size_t i = 0; i < line.points.size(); ++i) {
    if (line.cumulative[i] + 1e-6 >= start) {
      samples.emplace_back(line.cumulative[i], line.points[i]);
    }
  }
  std::sort(
    samples.begin(), samples.end(),
    [](const auto & lhs, const auto & rhs) {return lhs.first < rhs.first;});
  for (const auto & sample : samples) {
    appendPoint(points, sample.second);
  }
  return points;
}

inline std::vector<LinePoint> buildBypass(
  const LinePolyline & line, const LineProjection & progress, double rejoin_along,
  double offset_m, double side, double step_m)
{
  std::vector<LinePoint> points;
  if (!progress.valid || line.points.size() < 2) {
    return points;
  }
  const double length = polylineLength(line);
  const double rejoin = std::clamp(rejoin_along, progress.along, length);
  const auto join = pointAt(line, rejoin);
  const double yaw = progress.yaw;
  const double nx = -std::sin(yaw) * side;
  const double ny = std::cos(yaw) * side;
  appendPoint(points, LinePoint{progress.x, progress.y});
  appendPoint(points, LinePoint{progress.x + nx * offset_m, progress.y + ny * offset_m});
  appendPoint(points, LinePoint{join.x + nx * offset_m, join.y + ny * offset_m});
  appendPoint(points, join);
  for (const auto & point : sliceDense(line, rejoin, step_m)) {
    appendPoint(points, point);
  }
  return points;
}

// Rejoin is the first point on the original line past every blockage the
// costmap currently reports, plus a clearance margin. push_m slides it
// farther when the planner cannot reach the previous guess. The result is
// always clamped onto the reference line.
template<typename Index>
inline double selectRejoinAlong(
  double from_along, double line_length, const std::vector<Index> & invalid_indices,
  double step_m, double pass_margin, double push_m)
{
  const double step = step_m > 0.0 ? step_m : 5.0;
  double last_blocked = from_along;
  for (const auto index : invalid_indices) {
    if (index < 0) {
      continue;
    }
    last_blocked = std::max(
      last_blocked, from_along + static_cast<double>(index) * step);
  }
  const double margin = std::max(0.0, pass_margin);
  double rejoin = last_blocked + margin + std::max(0.0, push_m);
  rejoin = std::max(rejoin, from_along + std::max(15.0, margin));
  const double limit = std::max(from_along + 5.0, line_length - 1.0);
  return std::min(rejoin, limit);
}

// End of the blocked run that starts nearest the boat. Samples closer than
// ignore_m are the cells the hull is already on. A gap larger than gap_m
// starts a later obstacle; that later run must not drag the rejoin onto a
// far leg. Returns a negative value when nothing ahead is blocked.
template<typename Index>
inline double firstClusterEnd(
  double from_along, const std::vector<Index> & invalid_indices,
  double step_m, double ignore_m, double gap_m)
{
  const double step = step_m > 0.0 ? step_m : 5.0;
  std::vector<double> ahead;
  ahead.reserve(invalid_indices.size());
  for (const auto index : invalid_indices) {
    if (index < 0) {
      continue;
    }
    const double along = from_along + static_cast<double>(index) * step;
    if (along + 1e-3 >= from_along + std::max(0.0, ignore_m)) {
      ahead.push_back(along);
    }
  }
  if (ahead.empty()) {
    return -1.0;
  }
  std::sort(ahead.begin(), ahead.end());
  double end = ahead.front();
  const double gap = std::max(step, gap_m);
  for (std::size_t i = 1; i < ahead.size(); ++i) {
    if (ahead[i] - end > gap) {
      break;
    }
    end = ahead[i];
  }
  return end;
}

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__LINE_FOLLOW_GEOMETRY_HPP_
