# moveit_controller

A component exposing a MoveIt-based Cartesian controller for a manipulator. The component is configurable, so it can be used with different manipulators (I have tested it with both TIAGo and a UR3e arm).

Note that the component has been developed and tested on ROS2 Humble; I cannot guarantee that it works on other ROS2 distributions.

## Dependencies

The controller assumes that MoveIt is already set up for the robot of interest, as it creates a move group interface instance through which it interacts with the robot.

## Short usage description

The component receives execution requests of type `geometry_msgs/msg/PoseArray` (by default, on the topic `/get_trajectory`). After execution, the component publishes a result message of type `std_msgs/msg/Bool` (by default, on the topic `/trajectory_execution_result`).
