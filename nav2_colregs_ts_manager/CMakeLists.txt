cmake_minimum_required(VERSION 3.5)
project(nav2_colregs_ts_manager)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_colregs_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)

set(ts_core_library ts_core)
set(ts_ros_library colregs_ts_state_ros)

add_library(${ts_core_library} SHARED src/ts_core.cpp)
target_include_directories(${ts_core_library} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${ts_core_library} rclcpp geometry_msgs)

add_library(${ts_ros_library} SHARED src/colregs_ts_state_ros.cpp)
target_include_directories(${ts_ros_library} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${ts_ros_library} ${ts_core_library})
ament_target_dependencies(
  ${ts_ros_library}
  rclcpp rclcpp_lifecycle nav2_colregs_msgs nav_msgs geometry_msgs tf2 tf2_ros
  tf2_geometry_msgs visualization_msgs nav2_util)

# Legacy standalone TS nodes (restored from feat/colregs-humble): they back the
# classic planner_server + VORRTStar stack via /processed_ts_list,
# /get_avoidance_point and /get_barrier_lines. Self-contained lib, independent
# of ts_core/colregs_ts_state_ros used by colregs_local_planner_server.
set(legacy_ts_library ts_manager_core)
include_directories(include)
add_library(${legacy_ts_library} SHARED
  src/ts_state_manager.cpp
  src/avoidance_point.cpp
  src/barrier_node.cpp
)
ament_target_dependencies(
  ${legacy_ts_library}
  rclcpp nav2_colregs_msgs nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs visualization_msgs)

add_executable(ts_state_manager src/main.cpp)
target_link_libraries(ts_state_manager ${legacy_ts_library})
ament_target_dependencies(ts_state_manager rclcpp)

add_executable(avoidance_point_node src/avoidance_point_main.cpp)
target_link_libraries(avoidance_point_node ${legacy_ts_library})
ament_target_dependencies(avoidance_point_node rclcpp)

add_executable(barrier_node src/barrier_node_main.cpp)
target_link_libraries(barrier_node ${legacy_ts_library})
ament_target_dependencies(barrier_node rclcpp)

install(TARGETS ${ts_core_library} ${ts_ros_library}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS ts_state_manager avoidance_point_node barrier_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ts_core test/test_ts_core.cpp)
  target_link_libraries(test_ts_core ${ts_core_library})
  ament_target_dependencies(test_ts_core rclcpp geometry_msgs)
endif()

ament_export_include_directories(include)
ament_export_libraries(${ts_core_library} ${ts_ros_library})
ament_export_dependencies(
  rclcpp rclcpp_lifecycle nav2_colregs_msgs nav_msgs geometry_msgs tf2 tf2_ros
  tf2_geometry_msgs visualization_msgs nav2_util)
ament_package()
