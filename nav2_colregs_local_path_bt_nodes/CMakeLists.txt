cmake_minimum_required(VERSION 3.5)
project(nav2_colregs_local_path_bt_nodes)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_colregs_local_path_behavior REQUIRED)
find_package(nav2_colregs_msgs REQUIRED)
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

set(compute_local_path_dependencies
  ${dependencies}
  nav2_colregs_msgs
)

add_library(nav2_create_local_path_action_bt_node SHARED
  src/create_local_path_action.cpp
)

ament_target_dependencies(nav2_create_local_path_action_bt_node ${dependencies})
target_compile_definitions(nav2_create_local_path_action_bt_node PRIVATE BT_PLUGIN_EXPORT)

add_library(nav2_compute_local_path_action_bt_node SHARED
  src/compute_local_path_action.cpp
)

ament_target_dependencies(
  nav2_compute_local_path_action_bt_node ${compute_local_path_dependencies})
target_compile_definitions(nav2_compute_local_path_action_bt_node PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS
  nav2_create_local_path_action_bt_node
  nav2_compute_local_path_action_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies} nav2_colregs_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_compute_local_path_action test/test_compute_local_path_action.cpp)
  target_link_libraries(test_compute_local_path_action nav2_compute_local_path_action_bt_node)
  target_compile_definitions(test_compute_local_path_action PRIVATE
    COMPUTE_LOCAL_PATH_BT_PLUGIN_PATH="$<TARGET_FILE:nav2_compute_local_path_action_bt_node>")
  ament_target_dependencies(test_compute_local_path_action ${compute_local_path_dependencies})
endif()

ament_package()
