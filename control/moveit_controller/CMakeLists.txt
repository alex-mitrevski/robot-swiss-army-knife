cmake_minimum_required(VERSION 3.5)
project(moveit_controller)

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
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tf2_eigen REQUIRED)

include_directories(include)

add_executable(cartesian_controller src/cartesian_controller.cpp)
ament_target_dependencies(cartesian_controller
                          rclcpp
                          geometry_msgs
                          moveit_core
                          moveit_ros_planning
                          moveit_ros_planning_interface)

install(TARGETS
  cartesian_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
