cmake_minimum_required(VERSION 3.5)
project(ahrs_sim_pkg)

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
  include/ahrs_sim_pkg
)

add_executable(ahrs_sim
  src/ahrs_sim_node.cpp
  src/ahrs_sim.cpp
)

ament_target_dependencies(ahrs_sim rclcpp picopter_interfaces)

install(TARGETS
  ahrs_sim
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
