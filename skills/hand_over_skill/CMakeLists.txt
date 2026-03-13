cmake_minimum_required(VERSION 3.5)
project(hand_over_skill)

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tts_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(robot_swiss_knife_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/HandOverSkill.action"
  DEPENDENCIES action_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(include)
add_executable(hand_over
               src/start_skill.cpp
               src/hand_over_skill.cpp)
ament_target_dependencies(hand_over
                          rclcpp
                          rclcpp_action
                          rclcpp_lifecycle
                          diagnostic_msgs
                          lifecycle_msgs
                          std_msgs
                          geometry_msgs
                          trajectory_msgs
                          sensor_msgs
                          tts_msgs
                          diagnostic_updater
                          robot_swiss_knife_msgs)

target_link_libraries(hand_over
                      "${cpp_typesupport_target}")

install(TARGETS
  hand_over
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()