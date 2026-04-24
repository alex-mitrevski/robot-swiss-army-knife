cmake_minimum_required(VERSION 3.5)
project(behaviour_library)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_swiss_knife_msgs REQUIRED)
find_package(move_base_skill REQUIRED)

set(THIS_PACKAGE_DEPS
    behaviortree_ros2::behaviortree_ros2
    ${btcpp_ros2_interfaces_TARGETS}
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${robot_swiss_knife_msgs_TARGETS}
)

include_directories(include)

###############################
# Image subscription behaviour
###############################
add_library(image_sub_plugin SHARED src/image_sub_behaviour.cpp)
target_compile_definitions(image_sub_plugin PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(image_sub_plugin ${THIS_PACKAGE_DEPS})

#####################################
# Point cloud subscription behaviour
#####################################
add_library(cloud_sub_plugin SHARED src/cloud_sub_behaviour.cpp)
target_compile_definitions(cloud_sub_plugin PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(cloud_sub_plugin ${THIS_PACKAGE_DEPS})

###########################################
# Behaviour for checking object visibility
###########################################
add_library(check_visibility_plugin SHARED src/check_visibility_behaviour.cpp)
target_compile_definitions(check_visibility_plugin PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(check_visibility_plugin ${THIS_PACKAGE_DEPS})

################################
# Object segmentation behaviour
################################
add_library(segment_objects_plugin SHARED src/segment_objects_behaviour.cpp)
target_compile_definitions(segment_objects_plugin PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(segment_objects_plugin ${THIS_PACKAGE_DEPS})

#######################################
# ROI point cloud extraction behaviour
#######################################
add_library(extract_roi_3d_points_plugin SHARED src/extract_roi_3d_points_behaviour.cpp)
target_compile_definitions(extract_roi_3d_points_plugin PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(extract_roi_3d_points_plugin ${THIS_PACKAGE_DEPS})

########################################################
# Behaviour for moving a robot's base between locations
########################################################
add_library(move_base_plugin SHARED src/move_base_behaviour.cpp)
target_compile_definitions(move_base_plugin PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(move_base_plugin ${THIS_PACKAGE_DEPS} ${move_base_skill_TARGETS})

##################
# Behaviour trees
##################
add_executable(example_tree src/trees/example_tree_executor.cpp)
target_link_libraries(example_tree ${THIS_PACKAGE_DEPS})

install(TARGETS
  image_sub_plugin
  cloud_sub_plugin
  check_visibility_plugin
  segment_objects_plugin
  extract_roi_3d_points_plugin
  move_base_plugin
  example_tree
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  image_sub_plugin
  cloud_sub_plugin
  check_visibility_plugin
  segment_objects_plugin
  extract_roi_3d_points_plugin
  move_base_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)

install(DIRECTORY
  behaviour_trees
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
