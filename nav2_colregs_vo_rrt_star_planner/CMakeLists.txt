cmake_minimum_required(VERSION 3.5)
project(nav2_colregs_vo_rrt_star_planner)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_colregs_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()
add_compile_options(-O3)

include_directories(include)

set(library_name nav2_colregs_vo_rrt_star_planner)

set(dependencies
  nav2_colregs_msgs
  geometry_msgs
  nav2_common
  nav2_core
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2_ros
)

add_library(${library_name} SHARED
  src/rrt_star.cpp
  src/vo_rrt_star_planner.cpp
)

ament_target_dependencies(${library_name} ${dependencies})

target_compile_definitions(${library_name} PUBLIC "PLUGINLIB_DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(nav2_core vo_rrt_star_planner.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES vo_rrt_star_planner.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_planner_actions test/test_planner_actions.py TIMEOUT 90)

  ament_add_gtest(test_rrt_star test/test_rrt_star.cpp)
  ament_add_gtest(test_anchor_guard test/test_anchor_guard.cpp)
  target_link_libraries(test_rrt_star ${library_name})
  ament_target_dependencies(test_rrt_star geometry_msgs nav2_costmap_2d)
  target_link_libraries(test_anchor_guard ${library_name})
  ament_target_dependencies(test_anchor_guard geometry_msgs nav2_costmap_2d)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
