#include <memory>

#include "nav2_colregs_ts_manager/barrier_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_colregs_ts_manager::BarrierNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
