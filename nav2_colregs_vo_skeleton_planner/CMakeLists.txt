cmake_minimum_required(VERSION 3.8)
project(nav2_colregs_vo_skeleton_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_colregs_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

set(dependencies
  nav2_common
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav2_colregs_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  pluginlib
)

nav2_package()

add_library(${PROJECT_NAME} SHARED
  src/skeleton_space.cpp
  src/skeleton_tree.cpp
  src/skeleton_planner.cpp
  src/vo_skeleton_planner.cpp
)
target_compile_options(${PROJECT_NAME} PUBLIC -fPIC)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(nav2_core nav2_colregs_vo_skeleton_planner.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_common REQUIRED)
  ament_add_gtest(test_skeleton_planner test/test_skeleton_planner.cpp)
  target_link_libraries(test_skeleton_planner ${PROJECT_NAME})
  ament_target_dependencies(test_skeleton_planner ${dependencies})
endif()

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
