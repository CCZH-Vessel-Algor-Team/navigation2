cmake_minimum_required(VERSION 3.10)
project(nav2_colregs_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY params behavior_trees launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ts_parameter_contract test/test_ts_parameter_contract.py TIMEOUT 90)
  ament_add_pytest_test(test_ts_decisions test/test_ts_decisions.py TIMEOUT 120)
  ament_add_pytest_test(test_heading_smoothing test/test_heading_smoothing.py TIMEOUT 90)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
