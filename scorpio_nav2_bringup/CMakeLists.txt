cmake_minimum_required(VERSION 3.8)
project(scorpio_nav2_bringup)

find_package(ament_cmake_auto REQUIRED)

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  maps
  params
  rviz
)
