#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <thread>

#include "nav2_colregs_costmap_layers/ts_projection_layer.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

using nav2_colregs_msgs::msg::TrackedShipList;
using usv_interfaces::msg::MaritimeRuleDecision;

class StandOnProjectionTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("ts.tracked_ship_topic", "/stand_on_projection/tracks"),
      rclcpp::Parameter("ts.stand_on_policy_topic", "/stand_on_projection/policy")});
    node = std::make_shared<nav2_util::LifecycleNode>("stand_on_projection_test", "", options);
    peer = std::make_shared<rclcpp::Node>("stand_on_projection_peer");
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    map = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    map->resizeMap(40, 40, 1., 0., 0.);
    layer = std::make_shared<nav2_colregs_costmap_layers::TSProjectionLayer>();
    auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    layer->initialize(map.get(), "ts", tf.get(), node, group);
    map->addPlugin(layer);
    tracks = peer->create_publisher<TrackedShipList>("/stand_on_projection/tracks", 10);
    policy = peer->create_publisher<MaritimeRuleDecision>("/stand_on_projection/policy", 10);
    executor.add_node(node->get_node_base_interface());
    executor.add_node(peer);
    spinFor(.3);
  }

  void spinFor(double seconds)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  void publish(bool stand_on, bool valid = true, bool handover = false, bool second = false)
  {
    TrackedShipList ships;
    ships.header.frame_id = "map";
    ships.header.stamp = peer->now();
    nav2_colregs_msgs::msg::TrackedShip ship;
    ship.target_id.uuid[0] = 1;
    ship.pose.position.x = ship.pose.position.y = 10.;
    ship.radius = 1.;
    ships.ships.push_back(ship);
    if (second) {
      ship.target_id.uuid[0] = 2;
      ship.pose.position.x = ship.pose.position.y = 20.;
      ships.ships.push_back(ship);
    }
    MaritimeRuleDecision decision;
    decision.header = ships.header;
    decision.target_id.uuid[0] = 1;
    decision.stand_on = stand_on;
    decision.valid = valid;
    decision.handover_required = handover;
    tracks->publish(ships);
    policy->publish(decision);
    spinFor(.1);
    map->updateMap(0., 0., 0.);
  }

  unsigned char cost() {return map->getCostmap()->getCost(10, 10);}
  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<nav2_util::LifecycleNode> node;
  rclcpp::Node::SharedPtr peer;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> map;
  std::shared_ptr<nav2_colregs_costmap_layers::TSProjectionLayer> layer;
  rclcpp::Publisher<TrackedShipList>::SharedPtr tracks;
  rclcpp::Publisher<MaritimeRuleDecision>::SharedPtr policy;
};

TEST_F(StandOnProjectionTest, suppresses_only_a_fresh_valid_single_target_policy)
{
  publish(false);
  EXPECT_EQ(cost(), nav2_costmap_2d::LETHAL_OBSTACLE);
  publish(true);
  EXPECT_EQ(cost(), nav2_costmap_2d::FREE_SPACE);
  publish(false);
  EXPECT_EQ(cost(), nav2_costmap_2d::LETHAL_OBSTACLE);
  publish(true, false);
  EXPECT_EQ(cost(), nav2_costmap_2d::LETHAL_OBSTACLE);
  publish(true, true, true);
  EXPECT_EQ(cost(), nav2_costmap_2d::LETHAL_OBSTACLE);
  publish(true, true, false, true);
  EXPECT_EQ(cost(), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(StandOnProjectionTest, stale_policy_restores_target_projection)
{
  publish(true);
  EXPECT_EQ(cost(), nav2_costmap_2d::FREE_SPACE);
  spinFor(1.1);
  map->updateMap(0., 0., 0.);
  EXPECT_EQ(cost(), nav2_costmap_2d::LETHAL_OBSTACLE);
}
