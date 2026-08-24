#include <memory>

#include "nav2_colregs_ts_manager/ts_state_manager.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_colregs_ts_manager::TSStateManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
