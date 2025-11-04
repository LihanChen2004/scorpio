cmake_minimum_required(VERSION 3.11)
project(scorpio_base)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(generate_parameter_library REQUIRED)

###########
## Build ##
###########

generate_parameter_library(
  scorpio_base_parameters
  src/scorpio_base_parameters.yaml
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_link_libraries(${PROJECT_NAME}
  scorpio_base_parameters
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "scorpio_base::ScorpioBaseNode"
  "scorpio_base::CmdVelToAckermannNode"
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN scorpio_base::ScorpioBaseNode
  EXECUTABLE scorpio_base_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN scorpio_base::CmdVelToAckermannNode
  EXECUTABLE cmd_vel_to_ackermann_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  set(ament_cmake_clang_tidy_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-tidy")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

install(TARGETS scorpio_base_parameters
  EXPORT ${PROJECT_NAME}Targets)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
)
