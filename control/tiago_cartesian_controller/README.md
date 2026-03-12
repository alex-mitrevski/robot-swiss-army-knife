# tiago_cartesian_controller

A component exposing a Cartesian controller for TIAGo's arm+torso.

Note that the component has been developed and tested on ROS2 Humble; I cannot guarantee that it works on other ROS2 distributions.

## Dependencies

* The controller builds on the ROS control [KDL kinematics interface](https://github.com/ros-controls/kinematics_interface/tree/humble/kinematics_interface_kdl).
* The component also requires a joint velocity controller to be exposed on the robot.
