cmake_minimum_required(VERSION 3.5)
project(nav2_colregs_local_planner_server)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

nav2_package()

set(library_name ${PROJECT_NAME}_core)
set(dependencies
  geometry_msgs
  lifecycle_msgs
  rcl_action
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  nav_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
)

add_library(${library_name} SHARED src/local_planner_server.cpp src/rrt_star.cpp)
target_include_directories(${library_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${library_name} ${dependencies})
rclcpp_components_register_nodes(
  ${library_name} "nav2_colregs_local_planner_server::ColregsLocalPlannerServer")

add_executable(colregs_local_planner_server src/main.cpp)
target_link_libraries(colregs_local_planner_server ${library_name})
ament_target_dependencies(colregs_local_planner_server ${dependencies})

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS colregs_local_planner_server
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_local_planner_server test/test_local_planner_server.cpp)
  target_link_libraries(test_local_planner_server ${library_name})
  target_compile_definitions(test_local_planner_server PRIVATE
    TEST_PARAMS_FILE=\"${CMAKE_CURRENT_SOURCE_DIR}/test/test_local_planner_server_params.yaml\")
  ament_target_dependencies(
    test_local_planner_server ${dependencies} geometry_msgs lifecycle_msgs)

  ament_add_gtest(test_rrt_star test/test_rrt_star.cpp)
  target_link_libraries(test_rrt_star ${library_name})
  ament_target_dependencies(test_rrt_star nav2_costmap_2d)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
