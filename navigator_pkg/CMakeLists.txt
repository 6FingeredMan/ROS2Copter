cmake_minimum_required(VERSION 3.5)
project(navigator_pkg)

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
  include/navigator_pkg
)

add_executable(navigator
  src/navigator_node.cpp
  src/navigator.cpp
)

ament_target_dependencies(navigator rclcpp picopter_interfaces)

install(TARGETS
  navigator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
