cmake_minimum_required(VERSION 3.8)
project(scorpio_simulator)

find_package(ament_cmake_auto REQUIRED)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv.in")

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  rviz
  resource
)
