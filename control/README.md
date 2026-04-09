# control

Components used for robot control.

The following ROS 2 packages are included here:
* `moveit_controller`: A C++-based, robot-independent MoveIt interface for Cartesian control (i.e. moving the manipulator through a sequence of poses).
* `tiago_cartesian_controller`: A basic inverse kinematics controller for TIAGo's arm + torso. The component builds on the [ROS control KDL kinematics interface](https://github.com/ros-controls/kinematics_interface/tree/humble/kinematics_interface_kdl), and requires a joint velocity controller to be exposed on the robot.