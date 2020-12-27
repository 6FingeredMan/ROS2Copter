cmake_minimum_required(VERSION 3.5)
project(simulator_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(picopter_interfaces REQUIRED)

include_directories(
  include/simulator_pkg
)

add_executable(simulator
  src/simulator_node.cpp
  src/simulator.cpp
)

ament_target_dependencies(simulator rclcpp picopter_interfaces)

install(TARGETS
  simulator
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
