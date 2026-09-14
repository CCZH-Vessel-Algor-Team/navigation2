cmake_minimum_required(VERSION 3.5)
project(nav2_colregs_costmap_layers)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_colregs_msgs REQUIRED)
find_package(usv_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

set(library_name ${PROJECT_NAME}_plugin)

add_library(${library_name} SHARED
  src/ts_projection_layer.cpp
  src/perceived_obstacle_layer.cpp
)

ament_target_dependencies(${library_name}
  nav2_costmap_2d
  nav2_colregs_msgs
  usv_interfaces
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rclcpp
  pluginlib
)

pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(
  nav2_costmap_2d nav2_colregs_msgs usv_interfaces geometry_msgs tf2 tf2_ros tf2_geometry_msgs
  rclcpp pluginlib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_perceived_obstacle_layer_ros
    test/test_perceived_obstacle_layer_ros.py TIMEOUT 90)
  ament_add_gtest(test_perceived_obstacle_layer test/test_perceived_obstacle_layer.cpp)
  target_link_libraries(test_perceived_obstacle_layer ${library_name})
  ament_target_dependencies(test_perceived_obstacle_layer
    nav2_costmap_2d usv_interfaces geometry_msgs tf2_ros rclcpp pluginlib)
  set_tests_properties(test_perceived_obstacle_layer PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=92;FASTDDS_BUILTIN_TRANSPORTS=UDPv4")
endif()
ament_package()
