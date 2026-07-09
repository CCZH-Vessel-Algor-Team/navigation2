// Copyright (c) 2023 Samsung R&D Institute Russia
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

#include "nav2_colregs_vector_object_server/vector_object_shapes.hpp"

#include <uuid/uuid.h>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_util/occ_grid_values.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_util
{
template<typename NodeT, typename ParameterT>
ParameterT declare_or_get_parameter(
  NodeT node,
  const std::string & param_name,
  const ParameterT & default_value)
{
  declare_parameter_if_not_declared(node, param_name, rclcpp::ParameterValue(default_value));
  ParameterT value{};
  node->get_parameter(param_name, value);
  return value;
}
}  // namespace nav2_util

namespace nav2_colregs_vector_object_server
{

namespace
{

bool worldToMap(
  const nav_msgs::msg::OccupancyGrid::SharedPtr & map,
  double wx, double wy, unsigned int & mx, unsigned int & my)
{
  const double ox = map->info.origin.position.x;
  const double oy = map->info.origin.position.y;
  const double r = map->info.resolution;
  if (wx < ox || wy < oy || r <= 0.0) {
    return false;
  }
  mx = static_cast<unsigned int>((wx - ox) / r);
  my = static_cast<unsigned int>((wy - oy) / r);
  return mx < map->info.width && my < map->info.height;
}

bool pointInPolygon(double x, double y, const std::vector<geometry_msgs::msg::Point32> & pts)
{
  if (pts.size() < 3) {
    return false;
  }
  bool inside = false;
  size_t j = pts.size() - 1;
  for (size_t i = 0; i < pts.size(); ++i) {
    const double xi = pts[i].x;
    const double yi = pts[i].y;
    const double xj = pts[j].x;
    const double yj = pts[j].y;
    const bool intersect = ((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi);
    if (intersect) {
      inside = !inside;
    }
    j = i;
  }
  return inside;
}

void raytraceLine(
  MapAction & action, unsigned int x0, unsigned int y0,
  unsigned int x1, unsigned int y1, unsigned int width)
{
  int ix0 = static_cast<int>(x0);
  int iy0 = static_cast<int>(y0);
  const int ix1 = static_cast<int>(x1);
  const int iy1 = static_cast<int>(y1);
  const int dx = std::abs(ix1 - ix0);
  const int sx = ix0 < ix1 ? 1 : -1;
  const int dy = -std::abs(iy1 - iy0);
  const int sy = iy0 < iy1 ? 1 : -1;
  int err = dx + dy;
  while (true) {
    action(static_cast<unsigned int>(iy0) * width + static_cast<unsigned int>(ix0));
    if (ix0 == ix1 && iy0 == iy1) {
      break;
    }
    const int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      ix0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      iy0 += sy;
    }
  }
}

}  // namespace

// ---------- Shape ----------

Shape::Shape(const nav2_util::LifecycleNode::WeakPtr & node)
: type_(UNKNOWN), node_(node)
{}

Shape::~Shape()
{}

ShapeType Shape::getType()
{
  return type_;
}

int8_t Shape::getPointValue(const double px, const double py) const
{
  // Default: binary core/outside, no inflation.
  return isPointInside(px, py) ? getValue() : static_cast<int8_t>(0);
}

bool Shape::hasInflation() const
{
  return inflation_radius_ > 0.0 && cost_scaling_factor_ > 0.0;
}

void Shape::setInflationParams(double radius, double cost_scaling, double inscribed)
{
  inflation_radius_ = radius;
  cost_scaling_factor_ = cost_scaling;
  inscribed_radius_ = inscribed;
}

bool Shape::obtainShapeUUID(const std::string & shape_name, unsigned char * out_uuid)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  try {
    // Try to get shape UUID from ROS-parameters
    std::string uuid_str = nav2_util::declare_or_get_parameter<std::string>(
      node, shape_name + ".uuid", std::string(""));
    if (uuid_str.empty()) {
      uuid_generate(out_uuid);
      return true;
    }
    if (uuid_parse(uuid_str.c_str(), out_uuid) != 0) {
      RCLCPP_ERROR(
        node->get_logger(),
        "[%s] Can not parse UUID string for shape: %s",
        shape_name.c_str(), uuid_str.c_str());
      return false;
    }
  } catch (const std::exception &) {
    // If no UUID was specified, generate a new one
    uuid_generate(out_uuid);

    char uuid_str[37];
    uuid_unparse(out_uuid, uuid_str);
    RCLCPP_INFO(
      node->get_logger(),
      "[%s] No UUID is specified for shape. Generating a new one: %s",
      shape_name.c_str(), uuid_str);
  }

  return true;
}

// ---------- Polygon ----------

Polygon::Polygon(
  const nav2_util::LifecycleNode::WeakPtr & node)
: Shape::Shape(node)
{
  type_ = POLYGON;
}

int8_t Polygon::getValue() const
{
  return params_->value;
}

std::string Polygon::getFrameID() const
{
  return params_->header.frame_id;
}

std::string Polygon::getUUID() const
{
  return unparseUUID(params_->uuid.uuid.data());
}

bool Polygon::isUUID(const unsigned char * uuid) const
{
  return uuid_compare(params_->uuid.uuid.data(), uuid) == 0;
}

bool Polygon::isFill() const
{
  return params_->closed;
}

bool Polygon::obtainParams(const std::string & shape_name)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!params_) {
    params_ = std::make_shared<nav2_colregs_msgs::msg::PolygonObject>();
  }
  if (!polygon_) {
    polygon_ = std::make_shared<geometry_msgs::msg::Polygon>();
  }

  params_->header.frame_id = nav2_util::declare_or_get_parameter(
    node, shape_name + ".frame_id", std::string{"map"});
  params_->value = nav2_util::declare_or_get_parameter(
    node, shape_name + ".value", static_cast<int>(nav2_util::OCC_GRID_OCCUPIED));
  params_->closed = nav2_util::declare_or_get_parameter(
    node, shape_name + ".closed", true);

  std::vector<double> poly_row;
  try {
    poly_row = nav2_util::declare_or_get_parameter<std::vector<double>>(
      node, shape_name + ".points", std::vector<double>{});
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[%s] Error while getting polygon parameters: %s",
      shape_name.c_str(), ex.what());
    return false;
  }
  // Check for points format correctness
  if (poly_row.size() < 6 || poly_row.size() % 2 != 0) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[%s] Polygon has incorrect points description",
      shape_name.c_str());
    return false;
  }

  // Obtain polygon vertices
  geometry_msgs::msg::Point32 point;
  bool first = true;
  for (double val : poly_row) {
    if (first) {
      point.x = val;
    } else {
      point.y = val;
      params_->points.push_back(point);
    }
    first = !first;
  }

  // Filling the polygon_ with obtained points in map's frame
  polygon_->points = params_->points;

  // Getting shape UUID
  return obtainShapeUUID(shape_name, params_->uuid.uuid.data());
}

nav2_colregs_msgs::msg::PolygonObject::SharedPtr Polygon::getParams() const
{
  return params_;
}

bool Polygon::setParams(const nav2_colregs_msgs::msg::PolygonObject::SharedPtr params)
{
  params_ = params;

  if (!polygon_) {
    polygon_ = std::make_shared<geometry_msgs::msg::Polygon>();
  }
  polygon_->points = params_->points;

  // If no UUID was specified, generate a new one
  if (uuid_is_null(params_->uuid.uuid.data())) {
    uuid_generate(params_->uuid.uuid.data());
  }

  return checkConsistency();
}

bool Polygon::toFrame(
  const std::string & to_frame,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const double transform_tolerance)
{
  geometry_msgs::msg::PoseStamped from_pose, to_pose;
  from_pose.header = params_->header;
  for (unsigned int i = 0; i < params_->points.size(); i++) {
    from_pose.pose.position.x = params_->points[i].x;
    from_pose.pose.position.y = params_->points[i].y;
    from_pose.pose.position.z = params_->points[i].z;
    if (
      nav2_util::transformPoseInTargetFrame(
        from_pose, to_pose, *tf_buffer, to_frame, transform_tolerance))
    {
      polygon_->points[i].x = to_pose.pose.position.x;
      polygon_->points[i].y = to_pose.pose.position.y;
      polygon_->points[i].z = to_pose.pose.position.z;
    } else {
      return false;
    }
  }

  return true;
}

void Polygon::getBoundaries(double & min_x, double & min_y, double & max_x, double & max_y)
{
  min_x = std::numeric_limits<double>::max();
  min_y = std::numeric_limits<double>::max();
  max_x = std::numeric_limits<double>::lowest();
  max_y = std::numeric_limits<double>::lowest();

  for (auto point : polygon_->points) {
    min_x = std::min(min_x, static_cast<double>(point.x));
    min_y = std::min(min_y, static_cast<double>(point.y));
    max_x = std::max(max_x, static_cast<double>(point.x));
    max_y = std::max(max_y, static_cast<double>(point.y));
  }
}

bool Polygon::isPointInside(const double px, const double py) const
{
  return pointInPolygon(px, py, polygon_->points);
}

void Polygon::putBorders(
  nav_msgs::msg::OccupancyGrid::SharedPtr map, const OverlayType overlay_type)
{
  unsigned int mx0, my0, mx1, my1;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!worldToMap(map, polygon_->points[0].x, polygon_->points[0].y, mx1, my1)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[UUID: %s] Can not convert (%f, %f) point to map",
      getUUID().c_str(), polygon_->points[0].x, polygon_->points[0].y);
    return;
  }

  MapAction ma(map, params_->value, overlay_type);
  for (unsigned int i = 1; i < polygon_->points.size(); i++) {
    mx0 = mx1;
    my0 = my1;
    if (!worldToMap(map, polygon_->points[i].x, polygon_->points[i].y, mx1, my1)) {
      RCLCPP_ERROR(
        node->get_logger(),
        "[UUID: %s] Can not convert (%f, %f) point to map",
        getUUID().c_str(), polygon_->points[i].x, polygon_->points[i].y);
      return;
    }
    raytraceLine(ma, mx0, my0, mx1, my1, map->info.width);
  }
}

bool Polygon::checkConsistency()
{
  if (params_->points.size() < 3) {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }

    RCLCPP_ERROR(
      node->get_logger(),
      "[UUID: %s] Polygon has incorrect number of vertices: %li",
      getUUID().c_str(), params_->points.size());
    return false;
  }

  return true;
}

// ---------- Circle ----------

Circle::Circle(
  const nav2_util::LifecycleNode::WeakPtr & node)
: Shape::Shape(node)
{
  type_ = CIRCLE;
}

int8_t Circle::getValue() const
{
  return params_->value;
}

std::string Circle::getFrameID() const
{
  return params_->header.frame_id;
}

std::string Circle::getUUID() const
{
  return unparseUUID(params_->uuid.uuid.data());
}

bool Circle::isUUID(const unsigned char * uuid) const
{
  return uuid_compare(params_->uuid.uuid.data(), uuid) == 0;
}

bool Circle::isFill() const
{
  return params_->fill;
}

bool Circle::obtainParams(const std::string & shape_name)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!params_) {
    params_ = std::make_shared<nav2_colregs_msgs::msg::CircleObject>();
  }
  if (!center_) {
    center_ = std::make_shared<geometry_msgs::msg::Point32>();
  }

  params_->header.frame_id = nav2_util::declare_or_get_parameter(
    node, shape_name + ".frame_id", std::string{"map"});
  params_->value = nav2_util::declare_or_get_parameter(
    node, shape_name + ".value", static_cast<int>(nav2_util::OCC_GRID_OCCUPIED));
  params_->fill = nav2_util::declare_or_get_parameter(
    node, shape_name + ".fill", true);

  std::vector<double> center_row;
  try {
    center_row = nav2_util::declare_or_get_parameter<std::vector<double>>(
      node, shape_name + ".center", std::vector<double>{0.0, 0.0});
    params_->radius = nav2_util::declare_or_get_parameter<double>(
      node, shape_name + ".radius", 0.0);
    if (params_->radius < 0) {
      RCLCPP_ERROR(
        node->get_logger(),
        "[%s] Circle has incorrect radius less than zero",
        shape_name.c_str());
      return false;
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[%s] Error while getting circle parameters: %s",
      shape_name.c_str(), ex.what());
    return false;
  }
  // Check for points format correctness
  if (center_row.size() != 2) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[%s] Circle has incorrect center description",
      shape_name.c_str());
    return false;
  }

  // Obtain circle center
  params_->center.x = center_row[0];
  params_->center.y = center_row[1];
  // Setting the center_ with obtained circle center in map's frame
  *center_ = params_->center;

  // Getting shape UUID
  return obtainShapeUUID(shape_name, params_->uuid.uuid.data());
}

nav2_colregs_msgs::msg::CircleObject::SharedPtr Circle::getParams() const
{
  return params_;
}

bool Circle::setParams(const nav2_colregs_msgs::msg::CircleObject::SharedPtr params)
{
  params_ = params;

  if (!center_) {
    center_ = std::make_shared<geometry_msgs::msg::Point32>();
  }
  *center_ = params_->center;

  // If no UUID was specified, generate a new one
  if (uuid_is_null(params_->uuid.uuid.data())) {
    uuid_generate(params_->uuid.uuid.data());
  }

  return checkConsistency();
}

bool Circle::toFrame(
  const std::string & to_frame,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const double transform_tolerance)
{
  geometry_msgs::msg::PoseStamped from_pose, to_pose;
  from_pose.header = params_->header;
  from_pose.pose.position.x = params_->center.x;
  from_pose.pose.position.y = params_->center.y;
  from_pose.pose.position.z = params_->center.z;
  if (
    nav2_util::transformPoseInTargetFrame(
      from_pose, to_pose, *tf_buffer, to_frame, transform_tolerance))
  {
    center_->x = to_pose.pose.position.x;
    center_->y = to_pose.pose.position.y;
    center_->z = to_pose.pose.position.z;
  } else {
    return false;
  }

  return true;
}

void Circle::getBoundaries(double & min_x, double & min_y, double & max_x, double & max_y)
{
  min_x = center_->x - params_->radius;
  min_y = center_->y - params_->radius;
  max_x = center_->x + params_->radius;
  max_y = center_->y + params_->radius;
}

bool Circle::isPointInside(const double px, const double py) const
{
  return ( (px - center_->x) * (px - center_->x) + (py - center_->y) * (py - center_->y) ) <=
         params_->radius * params_->radius;
}

void Circle::putBorders(
  nav_msgs::msg::OccupancyGrid::SharedPtr map, const OverlayType overlay_type)
{
  unsigned int mcx, mcy;
  if (!centerToMap(map, mcx, mcy)) {
    return;
  }

  // Implementation of the circle generation algorithm, based on the following work:
  // Berthold K.P. Horn "Circle generators for display devices"
  // Computer Graphics and Image Processing 5.2 (1976): 280-288.

  // Inputs initialization
  const int r = static_cast<int>(std::round(params_->radius / map->info.resolution));
  int x = r;
  int y = 1;

  // Error initialization
  int s = -r;

  // Calculation algorithm
  while (x > y) {  // Calculating only first circle octant
    // Put 8 points in each octant reflecting symmetrically
    putPoint(mcx + x, mcy + y, map, overlay_type);
    putPoint(mcx + y, mcy + x, map, overlay_type);
    putPoint(mcx - x + 1, mcy + y, map, overlay_type);
    putPoint(mcx + y, mcy - x + 1, map, overlay_type);
    putPoint(mcx - x + 1, mcy - y + 1, map, overlay_type);
    putPoint(mcx - y + 1, mcy - x + 1, map, overlay_type);
    putPoint(mcx + x, mcy - y + 1, map, overlay_type);
    putPoint(mcx - y + 1, mcy + x, map, overlay_type);

    s = s + 2 * y + 1;
    y++;
    if (s > 0) {
      s = s - 2 * x + 2;
      x--;
    }
  }

  // Corner case for x == y: do not put end points twice
  if (x == y) {
    putPoint(mcx + x, mcy + y, map, overlay_type);
    putPoint(mcx - x + 1, mcy + y, map, overlay_type);
    putPoint(mcx - x + 1, mcy - y + 1, map, overlay_type);
    putPoint(mcx + x, mcy - y + 1, map, overlay_type);
  }
}

bool Circle::checkConsistency()
{
  if (params_->radius < 0.0) {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }

    RCLCPP_ERROR(
      node->get_logger(),
      "[UUID: %s] Circle has incorrect radius less than zero",
      getUUID().c_str());
    return false;
  }
  return true;
}

bool Circle::centerToMap(
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map,
  unsigned int & mcx, unsigned int & mcy)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Get center of circle in map coordinates
  if (center_->x < map->info.origin.position.x || center_->y < map->info.origin.position.y) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[UUID: %s] Can not convert (%f, %f) circle center to map",
      getUUID().c_str(), center_->x, center_->y);
    return false;
  }
  // We need the circle center to be always shifted one cell less its logical center
  // and to avoid any FP-accuracy losing on small values, so we are using another
  // than nav2_util::worldToMap() approach
  mcx = static_cast<unsigned int>(
    std::round((center_->x - map->info.origin.position.x) / map->info.resolution)) - 1;
  mcy = static_cast<unsigned int>(
    std::round((center_->y - map->info.origin.position.y) / map->info.resolution)) - 1;
  if (mcx >= map->info.width || mcy >= map->info.height) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[UUID: %s] Can not convert (%f, %f) point to map",
      getUUID().c_str(), center_->x, center_->y);
    return false;
  }

  return true;
}

inline void Circle::putPoint(
  unsigned int mx, unsigned int my,
  nav_msgs::msg::OccupancyGrid::SharedPtr map,
  const OverlayType overlay_type)
{
  processCell(map, my * map->info.width + mx, params_->value, overlay_type);
}

}  // namespace nav2_colregs_vector_object_server
