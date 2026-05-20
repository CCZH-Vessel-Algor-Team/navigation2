cmake_minimum_required(VERSION 3.10)
project(nav2_colregs_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds params maps models rviz scripts configs behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/target_ship_state_publisher.py
    scripts/target_ship_motion_commander.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
