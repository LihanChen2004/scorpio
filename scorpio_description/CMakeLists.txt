cmake_minimum_required(VERSION 3.5)
project(scorpio_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  resource
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
