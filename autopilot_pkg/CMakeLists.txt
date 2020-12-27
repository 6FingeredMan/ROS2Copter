cmake_minimum_required(VERSION 3.5)
project(autopilot_pkg)

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
  include/autopilot_pkg
)

add_executable(autopilot
  src/autopilot_node.cpp
  src/autopilot.cpp
  src/controllers.cpp
  src/standardPID.cpp
  src/standardAltitude.cpp
  src/superTwistingSMC.cpp
)

ament_target_dependencies(autopilot rclcpp picopter_interfaces)

install(TARGETS
  autopilot
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
