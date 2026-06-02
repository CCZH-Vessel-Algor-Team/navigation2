cmake_minimum_required(VERSION 3.5)
project(nav2_colregs_alos_controller)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_regulated_pure_pursuit_controller REQUIRED)
find_package(pluginlib REQUIRED)

set(dependencies
    rclcpp
    rclcpp_lifecycle
    tf2_ros
    tf2_geometry_msgs
    nav2_core
    nav2_costmap_2d
    nav2_util
    nav2_regulated_pure_pursuit_controller
    pluginlib
)

include_directories(include)

add_library(nav2_colregs_alos_controller SHARED src/alos_controller.cpp)
ament_target_dependencies(nav2_colregs_alos_controller ${dependencies})

pluginlib_export_plugin_description_file(nav2_core alos_controller_plugin.xml)

install(TARGETS nav2_colregs_alos_controller
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/)
install(FILES alos_controller_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_package()
