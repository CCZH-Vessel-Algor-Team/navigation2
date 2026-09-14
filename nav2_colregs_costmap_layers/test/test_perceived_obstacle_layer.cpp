#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>

#include "nav2_colregs_costmap_layers/perceived_obstacle_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "rcl/time.h"
#include "tf2_ros/buffer.h"

using nav2_colregs_costmap_layers::PerceivedObstacleLayer;
using usv_interfaces::msg::TrackedObstacleList;
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;

class AccessibleLayer : public PerceivedObstacleLayer
{
public:
  using PerceivedObstacleLayer::obstacleCallback;
};

class FixedMarkLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override {enabled_ = true; current_ = true;}
  void reset() override {}
  bool isClearable() override {return false;}
  void updateBounds(double, double, double, double * x0, double * y0, double * x1, double * y1)
  override
  {
    *x0 = std::min(*x0, 3.0); *y0 = std::min(*y0, 3.0);
    *x1 = std::max(*x1, 4.0); *y1 = std::max(*y1, 4.0);
  }
  void updateCosts(nav2_costmap_2d::Costmap2D & map, int, int, int, int) override
  {
    map.setCost(3, 3, LETHAL_OBSTACLE);
  }
};

class PerceivedLayerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override {initialize();}
  void TearDown() override
  {
    map_.reset();
    layer_.reset();
    tf_.reset();
    node_.reset();
  }

  void initialize(
    const std::string & frame = "map", bool rolling = false,
    const std::vector<rclcpp::Parameter> & overrides = {})
  {
    map_.reset(); layer_.reset(); tf_.reset(); node_.reset();
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    node_ = std::make_shared<nav2_util::LifecycleNode>("perceived_layer_test", "", options);
    ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(node_->get_clock()->get_clock_handle()));
    setTime(10.0);
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    map_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>(frame, rolling, false);
    map_->resizeMap(20, 20, 1.0, 0.0, 0.0);
    layer_ = std::make_shared<AccessibleLayer>();
    auto group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    layer_->initialize(map_.get(), "perceived", tf_.get(), node_, group);
    map_->addPlugin(layer_);
  }

  void setTime(double seconds)
  {
    ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(
        node_->get_clock()->get_clock_handle(), static_cast<int64_t>(seconds * 1e9)));
  }

  TrackedObstacleList::SharedPtr observation(
    uint8_t id, double x, double y, double radius = 0.8, const std::string & frame = "map")
  {
    auto msg = std::make_shared<TrackedObstacleList>();
    msg->header.frame_id = frame;
    msg->header.stamp = node_->get_clock()->now();
    usv_interfaces::msg::TrackedObstacle obj;
    obj.target_id.uuid[0] = id;
    obj.type = id == 1 ? "buoy" : "storm";
    obj.pose.position.x = x; obj.pose.position.y = y;
    obj.radius = radius;
    msg->obstacles.push_back(obj);
    return msg;
  }

  void transform(
    const std::string & target, const std::string & source, double x, double y,
    double yaw = 0.0, bool is_static = true)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = target; t.child_frame_id = source;
    t.header.stamp = node_->get_clock()->now();
    t.transform.translation.x = x; t.transform.translation.y = y;
    t.transform.rotation.z = std::sin(yaw / 2.0);
    t.transform.rotation.w = std::cos(yaw / 2.0);
    ASSERT_TRUE(tf_->setTransform(t, "test", is_static));
  }

  unsigned char cost(double x, double y)
  {
    unsigned int mx, my;
    EXPECT_TRUE(map_->getCostmap()->worldToMap(x, y, mx, my));
    return map_->getCostmap()->getCost(mx, my);
  }

  std::shared_ptr<nav2_util::LifecycleNode> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> map_;
  std::shared_ptr<AccessibleLayer> layer_;
};

TEST_F(PerceivedLayerTest, IndependentPublishersAndEmptyListsHoldUntilTimeout)
{
  layer_->obstacleCallback(observation(1, 3.5, 3.5));
  setTime(11.0);
  layer_->obstacleCallback(observation(2, 12.5, 12.5, 2.0));
  auto empty = observation(9, 0, 0);
  empty->obstacles.clear();
  layer_->obstacleCallback(empty);
  map_->updateMap(10, 10, 0);
  EXPECT_EQ(LETHAL_OBSTACLE, cost(3.5, 3.5));
  EXPECT_EQ(LETHAL_OBSTACLE, cost(12.5, 12.5));
  setTime(13.1);
  map_->updateMap(10, 10, 0);
  EXPECT_EQ(FREE_SPACE, cost(3.5, 3.5));
  EXPECT_EQ(LETHAL_OBSTACLE, cost(12.5, 12.5));
  setTime(14.1);
  map_->updateMap(10, 10, 0);
  EXPECT_EQ(FREE_SPACE, cost(12.5, 12.5));
}

TEST_F(PerceivedLayerTest, MoveAndShrinkWithdrawOldMarks)
{
  layer_->obstacleCallback(observation(1, 4.5, 4.5, 2.0));
  map_->updateMap(10, 10, 0);
  EXPECT_EQ(LETHAL_OBSTACLE, cost(6.5, 4.5));
  setTime(10.1);
  layer_->obstacleCallback(observation(1, 4.5, 4.5, 0.2));
  map_->updateMap(10, 10, 0);
  EXPECT_EQ(FREE_SPACE, cost(6.5, 4.5));
  setTime(10.2);
  layer_->obstacleCallback(observation(1, 12.5, 12.5, 0.2));
  map_->updateMap(10, 10, 0);
  EXPECT_EQ(FREE_SPACE, cost(4.5, 4.5));
  EXPECT_EQ(LETHAL_OBSTACLE, cost(12.5, 12.5));
}

TEST_F(PerceivedLayerTest, ExpiryDoesNotEraseAnotherLayer)
{
  auto fixed = std::make_shared<FixedMarkLayer>();
  fixed->initialize(map_.get(), "fixed", tf_.get(), node_, nullptr);
  map_->addPlugin(fixed);
  layer_->obstacleCallback(observation(1, 3.5, 3.5, 1.0));
  map_->updateMap(10, 10, 0);
  EXPECT_EQ(LETHAL_OBSTACLE, cost(4.5, 3.5));
  setTime(14.0);
  map_->updateMap(10, 10, 0);
  EXPECT_EQ(LETHAL_OBSTACLE, cost(3.5, 3.5));
  EXPECT_EQ(FREE_SPACE, cost(4.5, 3.5));
}

TEST_F(PerceivedLayerTest, DuplicateOldAndInvalidObservationsDoNotRefresh)
{
  auto msg = observation(1, 4.5, 4.5);
  layer_->obstacleCallback(msg);
  map_->updateMap(10, 10, 0);
  setTime(12.0);
  layer_->obstacleCallback(msg);
  auto invalid = observation(1, 5, 5, -1);
  layer_->obstacleCallback(invalid);
  invalid = observation(2, std::numeric_limits<double>::quiet_NaN(), 5);
  layer_->obstacleCallback(invalid);
  setTime(13.1);
  map_->updateMap(10, 10, 0);
  EXPECT_EQ(FREE_SPACE, cost(4.5, 4.5));
  EXPECT_EQ(FREE_SPACE, cost(5.5, 5.5));
}

TEST_F(PerceivedLayerTest, SensorObservationsStayAtMeasurementTimePosition)
{
  transform("map", "sensor", 5, 5, 0, false);
  layer_->obstacleCallback(observation(1, 1.5, 1.5, 0.2, "sensor"));
  setTime(11.0);
  transform("map", "sensor", 10, 10, 0, false);
  map_->updateMap(10, 10, 0);
  EXPECT_EQ(LETHAL_OBSTACLE, cost(6.5, 6.5));
  EXPECT_EQ(FREE_SPACE, cost(11.5, 11.5));
}

TEST_F(PerceivedLayerTest, MissingTFDoesNotPaintRawCoordinates)
{
  layer_->obstacleCallback(observation(1, 3.5, 3.5, 2.0, "missing_sensor"));
  map_->updateMap(10, 10, 0);
  EXPECT_EQ(FREE_SPACE, cost(3.5, 3.5));
}

TEST_F(PerceivedLayerTest, RotatedCostmapFrameHasConservativeBounds)
{
  initialize("odom");
  transform("odom", "map", 10, 5, 0.7853981633974483);
  layer_->obstacleCallback(observation(1, 4, 4, 1.0));
  map_->updateMap(10, 10, 0);
  EXPECT_EQ(LETHAL_OBSTACLE, cost(9.5, 10.5));
  EXPECT_EQ(LETHAL_OBSTACLE, cost(10.5, 10.5));
  double x0, y0, x1, y1;
  map_->getUpdatedBounds(x0, y0, x1, y1);
  EXPECT_LE(x0, 9.0);
  EXPECT_GE(x1, 11.0);
}

TEST_F(PerceivedLayerTest, CostmapTransformChangeClearsPreviousProjection)
{
  initialize("odom");
  transform("odom", "map", 0, 0, 0, false);
  layer_->obstacleCallback(observation(1, 3.5, 3.5, 0.2));
  map_->updateMap(10, 10, 0);
  setTime(11.0);
  transform("odom", "map", 8, 0, 0, false);
  map_->updateMap(10, 10, 0);
  EXPECT_EQ(FREE_SPACE, cost(3.5, 3.5));
  EXPECT_EQ(LETHAL_OBSTACLE, cost(11.5, 3.5));
}

TEST_F(PerceivedLayerTest, CircleIntersectsMapEvenWhenCenterIsOutside)
{
  layer_->obstacleCallback(observation(1, -0.5, 4.5, 1.0));
  map_->updateMap(10, 10, 0);
  EXPECT_EQ(LETHAL_OBSTACLE, cost(0.5, 4.5));
  EXPECT_EQ(FREE_SPACE, cost(2.5, 4.5));
}

TEST_F(PerceivedLayerTest, RollingWindowReprojectsEnteringObjects)
{
  initialize("map", true);
  layer_->obstacleCallback(observation(1, 24.5, 10.5, 1.0));
  map_->updateMap(10, 10, 0);
  map_->updateMap(20, 10, 0);
  EXPECT_EQ(LETHAL_OBSTACLE, cost(24.5, 10.5));
  map_->resizeMap(40, 40, 0.5, 10, 0);
  map_->updateMap(20, 10, 0);
  EXPECT_EQ(LETHAL_OBSTACLE, cost(24.5, 10.5));
}

TEST_F(PerceivedLayerTest, UpdatesOnlyRequestedWindow)
{
  layer_->obstacleCallback(observation(1, 4.5, 4.5, 20.0));
  double x0 = 1e30, y0 = 1e30, x1 = -1e30, y1 = -1e30;
  layer_->updateBounds(10, 10, 0, &x0, &y0, &x1, &y1);
  layer_->updateCosts(*map_->getCostmap(), 0, 0, 2, 2);
  EXPECT_EQ(LETHAL_OBSTACLE, cost(0.5, 0.5));
  EXPECT_EQ(FREE_SPACE, cost(2.5, 0.5));
  EXPECT_EQ(FREE_SPACE, cost(4.5, 4.5));
}

TEST_F(PerceivedLayerTest, CallbackBetweenHooksDoesNotChangeRenderSnapshot)
{
  layer_->obstacleCallback(observation(1, 3.5, 3.5, 0.2));
  double x0 = 1e30, y0 = 1e30, x1 = -1e30, y1 = -1e30;
  layer_->updateBounds(10, 10, 0, &x0, &y0, &x1, &y1);
  setTime(10.1);
  layer_->obstacleCallback(observation(1, 12.5, 12.5, 0.2));
  layer_->updateCosts(*map_->getCostmap(), 0, 0, 20, 20);
  EXPECT_EQ(LETHAL_OBSTACLE, cost(3.5, 3.5));
  EXPECT_EQ(FREE_SPACE, cost(12.5, 12.5));
  map_->updateMap(10, 10, 0);
  EXPECT_EQ(FREE_SPACE, cost(3.5, 3.5));
  EXPECT_EQ(LETHAL_OBSTACLE, cost(12.5, 12.5));
}

TEST_F(PerceivedLayerTest, ResetAndClockRewindWithdrawMarks)
{
  layer_->obstacleCallback(observation(1, 3.5, 3.5));
  map_->updateMap(10, 10, 0);
  layer_->reset();
  map_->updateMap(10, 10, 0);
  EXPECT_EQ(FREE_SPACE, cost(3.5, 3.5));
  layer_->obstacleCallback(observation(1, 3.5, 3.5));
  map_->updateMap(10, 10, 0);
  setTime(1.0);
  map_->updateMap(10, 10, 0);
  EXPECT_EQ(FREE_SPACE, cost(3.5, 3.5));
}

TEST_F(PerceivedLayerTest, DisabledLayerDoesNotMark)
{
  initialize("map", false, {rclcpp::Parameter("perceived.enabled", false)});
  layer_->obstacleCallback(observation(1, 3.5, 3.5));
  map_->updateMap(10, 10, 0);
  EXPECT_EQ(FREE_SPACE, cost(3.5, 3.5));
}

TEST_F(PerceivedLayerTest, ParametersAreValidatedAndReadOnly)
{
  EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("perceived.observation_timeout", 9.0)).successful);
  EXPECT_THROW(initialize("map", false, {
      rclcpp::Parameter("perceived.observation_timeout", 0.0)}), std::invalid_argument);
}

TEST_F(PerceivedLayerTest, PluginIsDiscoverable)
{
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader("nav2_costmap_2d", "nav2_costmap_2d::Layer");
  EXPECT_NE(nullptr, loader.createSharedInstance(
      "nav2_colregs_costmap_layers::PerceivedObstacleLayer"));
}

TEST_F(PerceivedLayerTest, ReceivesRealRosMessage)
{
  auto publisher = node_->create_publisher<TrackedObstacleList>("/tracked_obstacles", 10);
  publisher->on_activate();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_->get_node_base_interface());
  auto msg = observation(1, 3.5, 3.5);
  for (int i = 0; i < 100; ++i) {
    publisher->publish(*msg);
    executor.spin_some();
    map_->updateMap(10, 10, 0);
    if (cost(3.5, 3.5) == LETHAL_OBSTACLE) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_EQ(LETHAL_OBSTACLE, cost(3.5, 3.5));
}
