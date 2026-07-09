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

#include "nav2_colregs_vector_object_server/vector_object_server.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

#include "tf2_ros/create_timer_ros.hpp"

#include "nav2_util/node_utils.hpp"
#include "nav2_util/occ_grid_values.hpp"

namespace nav2_util
{
template<typename ParameterT, typename NodeT>
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

using namespace std::placeholders;

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

void mapToWorld(
  const nav_msgs::msg::OccupancyGrid::SharedPtr & map,
  unsigned int mx, unsigned int my, double & wx, double & wy)
{
  wx = map->info.origin.position.x + (static_cast<double>(mx) + 0.5) * map->info.resolution;
  wy = map->info.origin.position.y + (static_cast<double>(my) + 0.5) * map->info.resolution;
}

}  // namespace

VectorObjectServer::VectorObjectServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("vector_object_server", "", options), process_map_(false)
{}

nav2_util::CallbackReturn
VectorObjectServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  // Obtaining ROS parameters
  if (!obtainParams()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  if (!enforce_global_frame_id_) {
    // Transform buffer and listener initialization
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(),
      this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  } else {
    RCLCPP_INFO(
      get_logger(),
      "Parameter enforce_global_frame_id is true. TF listener is disabled. "
      "All incoming shapes must have frame_id empty or equal to global_frame_id '%s'.",
      global_frame_id_.c_str());
  }

  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

  add_shapes_service_ = create_service<nav2_colregs_msgs::srv::AddShapes>(
    "~/add_shapes",
    std::bind(&VectorObjectServer::addShapesCallback, this, _1, _2, _3));

  get_shapes_service_ = create_service<nav2_colregs_msgs::srv::GetShapes>(
    "~/get_shapes",
    std::bind(&VectorObjectServer::getShapesCallback, this, _1, _2, _3));

  remove_shapes_service_ = create_service<nav2_colregs_msgs::srv::RemoveShapes>(
    "~/remove_shapes",
    std::bind(&VectorObjectServer::removeShapesCallback, this, _1, _2, _3));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
VectorObjectServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  map_pub_->on_activate();

  // Trigger map to be published
  process_map_ = true;
  switchMapUpdate();

  // Creating bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
VectorObjectServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  if (map_timer_) {
    map_timer_->cancel();
    map_timer_.reset();
  }
  process_map_ = false;

  map_pub_->on_deactivate();

  // Destroying bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
VectorObjectServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  add_shapes_service_.reset();
  get_shapes_service_.reset();
  remove_shapes_service_.reset();

  map_pub_.reset();
  map_.reset();

  shapes_.clear();

  tf_listener_.reset();
  tf_buffer_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
VectorObjectServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool VectorObjectServer::obtainParams()
{
  auto node = shared_from_this();

  // Main ROS-parameters
  map_topic_ = nav2_util::declare_or_get_parameter(node, "map_topic", std::string{"vo_map"});
  global_frame_id_ = nav2_util::declare_or_get_parameter(node, "global_frame_id", std::string{"map"});
  enforce_global_frame_id_ = nav2_util::declare_or_get_parameter(node, "enforce_global_frame_id", false);
  resolution_ = nav2_util::declare_or_get_parameter(node, "resolution", 0.05);
  default_value_ = nav2_util::declare_or_get_parameter(
    node, "default_value",
    static_cast<int>(nav2_util::OCC_GRID_UNKNOWN));
  overlay_type_ = static_cast<OverlayType>(nav2_util::declare_or_get_parameter(
      node, "overlay_type",
      static_cast<int>(OverlayType::OVERLAY_SEQ)));
  update_frequency_ = nav2_util::declare_or_get_parameter(node, "update_frequency", 1.0);
  transform_tolerance_ = nav2_util::declare_or_get_parameter(node, "transform_tolerance", 0.1);

  // Fixed map dimensions (when configured, skip dynamic sizing)
  map_width_ = nav2_util::declare_or_get_parameter(node, "width", 0.0);
  map_height_ = nav2_util::declare_or_get_parameter(node, "height", 0.0);
  map_origin_x_ = nav2_util::declare_or_get_parameter(node, "origin_x", 0.0);
  map_origin_y_ = nav2_util::declare_or_get_parameter(node, "origin_y", 0.0);
  use_fixed_map_ = (map_width_ > 0.0 && map_height_ > 0.0);

  // Shape inflation — shared by all shapes
  inflation_radius_ = nav2_util::declare_or_get_parameter(
    node, "inflation_radius", 0.0);
  cost_scaling_factor_ = nav2_util::declare_or_get_parameter(
    node, "cost_scaling_factor", 3.0);
  inscribed_radius_ = nav2_util::declare_or_get_parameter(
    node, "inscribed_radius", 0.0);

  // Shapes
  auto shape_names = nav2_util::declare_or_get_parameter(node, "shapes", std::vector<std::string>());
  for (std::string shape_name : shape_names) {
    std::string shape_type;
    try {
      shape_type = nav2_util::declare_or_get_parameter<std::string>(
        node, shape_name + ".type", std::string(""));
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        get_logger(), "Error while getting shape %s type: %s", shape_name.c_str(), ex.what());
      return false;
    }

    if (shape_type == "polygon") {
      auto polygon = std::make_shared<Polygon>(node);
      if (!polygon->obtainParams(shape_name)) {
        return false;
      }
      shapes_.push_back(polygon);
    } else if (shape_type == "circle") {
      auto circle = std::make_shared<Circle>(node);
      if (!circle->obtainParams(shape_name)) {
        return false;
      }
      shapes_.push_back(circle);
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "Please specify the correct type for shape %s. Supported types are 'polygon' and 'circle'",
        shape_name.c_str());
      return false;
    }
  }

  // if any shapes has non-global frame id, no shapes will be added and return false.
  if (enforce_global_frame_id_) {
    for (const auto & shape : shapes_) {
      const std::string & frame_id = shape->getFrameID();
      if (!frame_id.empty() && frame_id != global_frame_id_) {
        RCLCPP_ERROR(
          get_logger(),
          "Shape '%s' has frame_id '%s' which differs from global_frame_id '%s'. "
          "All shapes must have frame_id empty or equal to global_frame_id "
          "when enforce_global_frame_id is true.",
          shape->getUUID().c_str(), frame_id.c_str(), global_frame_id_.c_str());
        shapes_.clear();
        return false;
      }
    }
  }

  return true;
}

std::vector<std::shared_ptr<Shape>>::iterator
VectorObjectServer::findShape(const unsigned char * uuid)
{
  for (auto it = shapes_.begin(); it != shapes_.end(); it++) {
    if ((*it)->isUUID(uuid)) {
      return it;
    }
  }
  return shapes_.end();
}

bool VectorObjectServer::transformVectorObjects()
{
  for (auto shape : shapes_) {
    if (shape->getFrameID() != global_frame_id_ && !shape->getFrameID().empty()) {
      // Shape to be updated dynamically
      if (!shape->toFrame(global_frame_id_, tf_buffer_, transform_tolerance_)) {
        RCLCPP_ERROR(
          get_logger(), "Can not transform vector object from %s to %s frame",
          shape->getFrameID().c_str(), global_frame_id_.c_str());
        return false;
      }
    }
  }

  return true;
}

void VectorObjectServer::getMapBoundaries(
  double & min_x, double & min_y, double & max_x, double & max_y) const
{
  min_x = std::numeric_limits<double>::max();
  min_y = std::numeric_limits<double>::max();
  max_x = std::numeric_limits<double>::lowest();
  max_y = std::numeric_limits<double>::lowest();

  double min_p_x, min_p_y, max_p_x, max_p_y;
  for (auto shape : shapes_) {
    shape->getBoundaries(min_p_x, min_p_y, max_p_x, max_p_y);
    min_x = std::min(min_x, min_p_x);
    min_y = std::min(min_y, min_p_y);
    max_x = std::max(max_x, max_p_x);
    max_y = std::max(max_y, max_p_y);
  }

  // Pad map boundaries to accommodate the inflation band around all shapes.
  min_x -= inflation_radius_;
  min_y -= inflation_radius_;
  max_x += inflation_radius_;
  max_y += inflation_radius_;

  if (
    min_x == std::numeric_limits<double>::max() ||
    min_y == std::numeric_limits<double>::max() ||
    max_x == std::numeric_limits<double>::lowest() ||
    max_y == std::numeric_limits<double>::lowest())
  {
    throw std::runtime_error("Can not obtain map boundaries");
  }
}

void VectorObjectServer::updateMap(
  const double & min_x, const double & min_y, const double & max_x, const double & max_y)
{
  // Calculate size of update map
  int size_x = static_cast<int>((max_x - min_x) / resolution_) + 1;
  int size_y = static_cast<int>((max_y - min_y) / resolution_) + 1;

  if (size_x < 0) {
    throw std::runtime_error("Incorrect map x-size");
  }

  if (size_y < 0) {
    throw std::runtime_error("Incorrect map y-size");
  }

  if (!map_) {
    map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  }

  if (
    map_->info.width != static_cast<unsigned int>(size_x) ||
    map_->info.height != static_cast<unsigned int>(size_y))
  {
    // Map size was changed
    map_->data = std::vector<int8_t>(size_x * size_y, default_value_);
    map_->info.width = size_x;
    map_->info.height = size_y;
  } else if (size_x > 0 && size_y > 0) {
    // Map size was not changed
    memset(map_->data.data(), default_value_, size_x * size_y * sizeof(int8_t));
  }

  map_->header.frame_id = global_frame_id_;
  map_->info.resolution = resolution_;
  map_->info.origin.position.x = min_x;
  map_->info.origin.position.y = min_y;
}

void VectorObjectServer::putVectorObjectsOnMap()
{
  // Propagate global inflation params to all shapes before rasterization
  for (auto shape : shapes_) {
    shape->setInflationParams(inflation_radius_, cost_scaling_factor_, inscribed_radius_);
  }

  // Filling the shapes
  for (auto shape : shapes_) {
    if (shape->isFill()) {
      // Put filled shape on map
      double wx1 = std::numeric_limits<double>::max();
      double wy1 = std::numeric_limits<double>::max();
      double wx2 = std::numeric_limits<double>::lowest();
      double wy2 = std::numeric_limits<double>::lowest();
      unsigned int mx1 = 0;
      unsigned int my1 = 0;
      unsigned int mx2 = 0;
      unsigned int my2 = 0;

      shape->getBoundaries(wx1, wy1, wx2, wy2);
      if (
        !worldToMap(map_, wx1, wy1, mx1, my1) ||
        !worldToMap(map_, wx2, wy2, mx2, my2))
      {
        RCLCPP_ERROR(
          get_logger(),
          "Error to get shape boundaries on map (UUID: %s)", shape->getUUID().c_str());
        return;
      }

      unsigned int it;
      for (unsigned int my = my1; my <= my2; my++) {
        for (unsigned int mx = mx1; mx <= mx2; mx++) {
          it = my * map_->info.width + mx;
          double wx, wy;
          mapToWorld(map_, mx, my, wx, wy);
          if (shape->isPointInside(wx, wy)) {
            processVal(map_->data[it], shape->getValue(), overlay_type_);
          }
        }
      }
    } else {
      // Put shape borders on map
      shape->putBorders(map_, overlay_type_);
    }
  }

  // Post-rasterization: uniform multi-pass dilation for all shapes
  // with exponential cost gradient (replicating Nav2 InflationLayer model).
  if (inflation_radius_ > 0.0 && cost_scaling_factor_ > 0.0) {
    const int dilation_steps = static_cast<int>(
      std::ceil(inflation_radius_ / map_->info.resolution));
    const unsigned int w = map_->info.width;
    const unsigned int h = map_->info.height;

    std::vector<uint8_t> initial_frontier(w * h, 0);
    for (unsigned int i = 0; i < w * h; i++) {
      if (map_->data[i] > 0) {
        initial_frontier[i] = 1;
      }
    }

    for (int step = 0; step < dilation_steps; step++) {
      std::vector<unsigned int> step_frontier;
      for (unsigned int i = 0; i < w * h; i++) {
        if (initial_frontier[i]) {
          step_frontier.push_back(i);
        }
      }

      std::vector<uint8_t> next_frontier(w * h, 0);
      for (auto idx : step_frontier) {
        unsigned int x = idx % w;
        unsigned int y = idx / w;

        for (int dy = -1; dy <= 1; dy++) {
          for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            int nx = static_cast<int>(x) + dx;
            int ny = static_cast<int>(y) + dy;
            if (nx < 0 || ny < 0 ||
                nx >= static_cast<int>(w) || ny >= static_cast<int>(h))
            {
              continue;
            }
            unsigned int nidx = ny * w + nx;
            if (map_->data[nidx] == 0) {
              // Exponential gradient: cost based on step distance from edge.
              int8_t inf_val = computeInflationCost(
                (step + 0.5), map_->info.resolution,
                inscribed_radius_, cost_scaling_factor_);
              processVal(map_->data[nidx], inf_val, overlay_type_);
              next_frontier[nidx] = 1;
            }
          }
        }
      }
      initial_frontier = std::move(next_frontier);
    }
  }
}

void VectorObjectServer::publishMap()
{
  if (map_) {
    auto map = std::make_unique<nav_msgs::msg::OccupancyGrid>(*map_);
    map_pub_->publish(std::move(map));
  }
}

void VectorObjectServer::processMap()
{
  if (!process_map_) {
    return;
  }

  try {
    if (shapes_.size() > 0) {
      if (!transformVectorObjects()) {
        return;
      }
      double min_x, min_y, max_x, max_y;

      if (use_fixed_map_) {
        min_x = map_origin_x_;
        min_y = map_origin_y_;
        max_x = map_origin_x_ + map_width_;
        max_y = map_origin_y_ + map_height_;
      } else {
        getMapBoundaries(min_x, min_y, max_x, max_y);
      }
      updateMap(min_x, min_y, max_x, max_y);
      putVectorObjectsOnMap();
    } else {
      if (use_fixed_map_) {
        updateMap(map_origin_x_, map_origin_y_,
                  map_origin_x_ + map_width_,
                  map_origin_y_ + map_height_);
      } else {
        updateMap(0.0, 0.0, 0.0, 0.0);
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Can not update map: %s", ex.what());
    return;
  }

  publishMap();
}

void VectorObjectServer::switchMapUpdate()
{
  for (auto shape : shapes_) {
    if (shape->getFrameID() != global_frame_id_ && !shape->getFrameID().empty()) {
      if (!map_timer_) {
        map_timer_ = this->create_wall_timer(
          std::chrono::duration<double>(1.0 / update_frequency_),
          std::bind(&VectorObjectServer::processMap, this));
      }
      RCLCPP_INFO(get_logger(), "Publishing map dynamically at %f Hz rate", update_frequency_);
      return;
    }
  }

  if (map_timer_) {
    map_timer_->cancel();
    map_timer_.reset();
  }
  RCLCPP_INFO(get_logger(), "Publishing map once");
  processMap();
}

void VectorObjectServer::addShapesCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_colregs_msgs::srv::AddShapes::Request> request,
  std::shared_ptr<nav2_colregs_msgs::srv::AddShapes::Response> response)
{
  // Initialize result with true. If one of the required vector object was not added properly,
  // set it to false.
  response->success = true;

  if (enforce_global_frame_id_) {
    // Lambda for checking frame_id consistency
    auto check_frame_id = [this](
      const auto & shapes, const std::string & shape_type_name)
      {
        for (const auto & shape : shapes) {
          if (!shape.header.frame_id.empty() && shape.header.frame_id != global_frame_id_) {
            RCLCPP_ERROR(
            get_logger(),
            "%s frame_id '%s' must be empty or equal to global_frame_id '%s' "
            "when enforce_global_frame_id is true. Rejecting request.",
            shape_type_name.c_str(), shape.header.frame_id.c_str(), global_frame_id_.c_str());
            return false;
          }
        }
        return true;
      };

    if (!check_frame_id(request->polygons, "Polygon") ||
      !check_frame_id(request->circles, "Circle"))
    {
      response->success = false;
      return;
    }
  }

  auto node = shared_from_this();

  // Process polygons
  for (auto req_poly : request->polygons) {
    nav2_colregs_msgs::msg::PolygonObject::SharedPtr new_params =
      std::make_shared<nav2_colregs_msgs::msg::PolygonObject>(req_poly);

    auto it = findShape(new_params->uuid.uuid.data());
    if (it != shapes_.end()) {
      // Vector Object with given UUID was found: updating it
      // Check that found shape has correct type
      if ((*it)->getType() != POLYGON) {
        RCLCPP_ERROR(
          get_logger(),
          "Shape (UUID: %s) is not a polygon type for a polygon update. Not adding shape.",
          (*it)->getUUID().c_str());
        response->success = false;
        // Do not add this shape
        continue;
      }

      std::shared_ptr<Polygon> polygon = std::static_pointer_cast<Polygon>(*it);

      // Preserving old parameters for the case, if new ones to be incorrect
      nav2_colregs_msgs::msg::PolygonObject::SharedPtr old_params = polygon->getParams();
      if (!polygon->setParams(new_params)) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to update existing polygon object (UUID: %s) with new params. "
          "Reverting to old polygon params.",
          (*it)->getUUID().c_str());
        // Restore old parameters
        polygon->setParams(old_params);
        // ... and set the failure to return
        response->success = false;
      }
    } else {
      // Vector Object with given UUID was not found: creating a new one
      std::shared_ptr<Polygon> polygon = std::make_shared<Polygon>(node);
      if (polygon->setParams(new_params)) {
        shapes_.push_back(polygon);
      } else {
        RCLCPP_ERROR(
          get_logger(), "Failed to create a new polygon object using the provided params.");
        response->success = false;
      }
    }
  }

  // Process circles
  for (auto req_crcl : request->circles) {
    nav2_colregs_msgs::msg::CircleObject::SharedPtr new_params =
      std::make_shared<nav2_colregs_msgs::msg::CircleObject>(req_crcl);

    auto it = findShape(new_params->uuid.uuid.data());
    if (it != shapes_.end()) {
      // Vector object with given UUID was found: updating it
      // Check that found shape has correct type
      if ((*it)->getType() != CIRCLE) {
        RCLCPP_ERROR(
          get_logger(),
          "Shape (UUID: %s) is not a circle type for a circle update. Not adding shape.",
          (*it)->getUUID().c_str());
        response->success = false;
        // Do not add this shape
        continue;
      }

      std::shared_ptr<Circle> circle = std::static_pointer_cast<Circle>(*it);

      // Preserving old parameters for the case, if new ones to be incorrect
      nav2_colregs_msgs::msg::CircleObject::SharedPtr old_params = circle->getParams();
      if (!circle->setParams(new_params)) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to update existing circle object (UUID: %s) with new params. "
          "Reverting to old circle params.",
          (*it)->getUUID().c_str());
        // Restore old parameters
        circle->setParams(old_params);
        // ... and set the failure to return
        response->success = false;
      }
    } else {
      // Vector Object with given UUID was not found: creating a new one
      std::shared_ptr<Circle> circle = std::make_shared<Circle>(node);
      if (circle->setParams(new_params)) {
        shapes_.push_back(circle);
      } else {
        RCLCPP_ERROR(
          get_logger(), "Failed to create a new circle object using the provided params.");
        response->success = false;
      }
    }
  }

  switchMapUpdate();
}

void VectorObjectServer::getShapesCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_colregs_msgs::srv::GetShapes::Request>/*request*/,
  std::shared_ptr<nav2_colregs_msgs::srv::GetShapes::Response> response)
{
  std::shared_ptr<Polygon> polygon;
  std::shared_ptr<Circle> circle;

  for (auto shape : shapes_) {
    switch (shape->getType()) {
      case POLYGON:
        polygon = std::static_pointer_cast<Polygon>(shape);
        response->polygons.push_back(*(polygon->getParams()));
        break;
      case CIRCLE:
        circle = std::static_pointer_cast<Circle>(shape);
        response->circles.push_back(*(circle->getParams()));
        break;
      default:
        RCLCPP_WARN(get_logger(), "Unknown shape type (UUID: %s)", shape->getUUID().c_str());
    }
  }
}

void VectorObjectServer::removeShapesCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_colregs_msgs::srv::RemoveShapes::Request> request,
  std::shared_ptr<nav2_colregs_msgs::srv::RemoveShapes::Response> response)
{
  // Initialize result with true. If one of the required vector object was not found,
  // set it to false.
  response->success = true;

  if (request->all_objects) {
    // Clear all objects
    shapes_.clear();
  } else {
    // Find objects to remove
    for (auto req_uuid : request->uuids) {
      auto it = findShape(req_uuid.uuid.data());
      if (it != shapes_.end()) {
        // Shape with given UUID was found: remove it
        (*it).reset();
        shapes_.erase(it);
      } else {
        // Required vector object was not found
        RCLCPP_ERROR(
          get_logger(),
          "Can not find shape to remove with UUID: %s",
          unparseUUID(req_uuid.uuid.data()).c_str());
        response->success = false;
      }
    }
  }

  switchMapUpdate();
}

}  // namespace nav2_colregs_vector_object_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_colregs_vector_object_server::VectorObjectServer)
