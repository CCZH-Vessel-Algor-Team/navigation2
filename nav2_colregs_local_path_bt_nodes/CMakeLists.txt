cmake_minimum_required(VERSION 3.5)
project(nav2_colregs_local_path_bt_nodes)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_colregs_local_path_behavior REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  rclcpp_action
  nav2_behavior_tree
  nav2_colregs_local_path_behavior
  nav_msgs
  behaviortree_cpp
)

add_library(nav2_create_local_path_action_bt_node SHARED
  src/create_local_path_action.cpp
)

ament_target_dependencies(nav2_create_local_path_action_bt_node ${dependencies})
target_compile_definitions(nav2_create_local_path_action_bt_node PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS
  nav2_create_local_path_action_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
