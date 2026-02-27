cmake_minimum_required(VERSION 3.11)
project(latch_can_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(latch_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(latch_can_bridge src/latch_can_bridge.cpp)
ament_target_dependencies(latch_can_bridge rclcpp std_msgs ament_index_cpp latch_interfaces)

add_executable(hw_bridge_main hw_bridges/hw_bridge_main.cpp src/can_yaml_utils.cpp)
ament_target_dependencies(hw_bridge_main rclcpp std_msgs latch_interfaces)

target_include_directories(hw_bridge_main PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(hw_bridge_main yaml-cpp)

install(TARGETS
  latch_can_bridge
  hw_bridge_main
  DESTINATION lib/${PROJECT_NAME})

install(FILES
        config/can_mappings.yaml
        DESTINATION share/${PROJECT_NAME}/config/)

# insatlling for launch files
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

target_include_directories(latch_can_bridge PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
