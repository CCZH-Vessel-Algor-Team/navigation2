cmake_minimum_required(VERSION 3.10)
project(nav2_colregs_vector_object_server)

# Suppress noisy third-party CMake deprecation warnings (e.g. gtest_vendor)
# in this package's configure output.
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "")
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)

option(NAV2_COLREGS_VO_BUILD_TESTS "Build unit tests for nav2_colregs_vector_object_server" OFF)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_colregs_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_library(UUID_LIBRARY uuid REQUIRED)

nav2_package()

include_directories(include)

add_library(vector_object_server_core SHARED
  src/vector_object_server.cpp
  src/vector_object_shapes.cpp
)
ament_target_dependencies(vector_object_server_core
  nav2_colregs_msgs
  nav2_util
  nav_msgs
  geometry_msgs
  unique_identifier_msgs
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  tf2_ros
  tf2
)
target_link_libraries(vector_object_server_core
  ${UUID_LIBRARY}
)
rclcpp_components_register_nodes(
  vector_object_server_core
  "nav2_colregs_vector_object_server::VectorObjectServer")

add_executable(vector_object_server
  src/main.cpp
)
target_link_libraries(vector_object_server
  vector_object_server_core
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS vector_object_server_core
  vector_object_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(vector_object_server_core)

ament_package()
