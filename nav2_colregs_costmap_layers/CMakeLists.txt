cmake_minimum_required(VERSION 3.5)
project(nav2_colregs_costmap_layers)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_colregs_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

set(library_name ${PROJECT_NAME}_plugin)

add_library(${library_name} SHARED
  src/ts_projection_layer.cpp
)

ament_target_dependencies(${library_name}
  nav2_costmap_2d
  nav2_colregs_msgs
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
ament_export_dependencies(nav2_costmap_2d nav2_colregs_msgs rclcpp pluginlib)
ament_package()
