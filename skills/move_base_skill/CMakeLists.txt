cmake_minimum_required(VERSION 3.5)
project(move_base_skill)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tts_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveBaseSkill.action"
  DEPENDENCIES
  action_msgs
  geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(include)
add_executable(move_base
               src/start_skill.cpp
               src/move_base_skill.cpp)
ament_target_dependencies(move_base
                          rclcpp
                          rclcpp_action
                          rclcpp_lifecycle
                          diagnostic_msgs
                          lifecycle_msgs
                          geometry_msgs
                          nav2_msgs
                          tts_msgs
                          diagnostic_updater
                          tf2
                          tf2_ros
                          tf2_geometry_msgs)

target_link_libraries(move_base
                      "${cpp_typesupport_target}")

install(TARGETS
  move_base
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    test
  DESTINATION share/${PROJECT_NAME}
)

ament_package()