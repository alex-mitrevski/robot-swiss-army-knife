cmake_minimum_required(VERSION 3.5)
project(point_cloud_utils)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(robot_swiss_knife_msgs REQUIRED)

include_directories(include)

add_executable(roi_3d_point_extractor src/roi_3d_point_extractor.cpp)
ament_target_dependencies(roi_3d_point_extractor
                          rclcpp
                          std_msgs
                          geometry_msgs
                          sensor_msgs
                          pcl_ros
                          robot_swiss_knife_msgs)

add_executable(pose_calculator src/pose_calculator.cpp)
ament_target_dependencies(pose_calculator
                          rclcpp
                          std_msgs
                          geometry_msgs
                          sensor_msgs
                          pcl_ros
                          tf2_ros
                          tf2_eigen
                          robot_swiss_knife_msgs)

install(TARGETS
  roi_3d_point_extractor
  pose_calculator
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
