cmake_minimum_required(VERSION 3.5)
project(nav2_colregs_ts_manager)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_colregs_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

set(library_name ts_manager_core)

add_library(${library_name} SHARED
  src/ts_state_manager.cpp
  src/avoidance_point.cpp
  src/barrier_node.cpp
)

ament_target_dependencies(${library_name}
  rclcpp nav2_colregs_msgs nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs visualization_msgs)

add_executable(ts_state_manager src/main.cpp)
target_link_libraries(ts_state_manager ${library_name})
ament_target_dependencies(ts_state_manager rclcpp)

add_executable(avoidance_point_node src/avoidance_point_main.cpp)
target_link_libraries(avoidance_point_node ${library_name})
ament_target_dependencies(avoidance_point_node rclcpp)

add_executable(barrier_node src/barrier_node_main.cpp)
target_link_libraries(barrier_node ${library_name})
ament_target_dependencies(barrier_node rclcpp)

install(TARGETS ${library_name} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS ts_state_manager avoidance_point_node barrier_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(rclcpp nav2_colregs_msgs visualization_msgs)
ament_package()
