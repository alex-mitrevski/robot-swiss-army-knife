from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="moveit_controller",
            executable="cartesian_controller",
            name="moveit_cartesian_controller",
            parameters = [
                {"max_moveit_initialisation_attempts": 10,
                 "move_group_name": "arm_torso",
                 "planner_id": "RRTstarkConfigDefault",
                 "number_of_planning_attempts": 10,
                 "max_velocity_scaling_factor": 0.1,
                 "max_acceleration_scaling_factor": 0.1,
                 "request_topic": "get_trajectory",
                 "result_topic": "trajectory_execution_result"}
            ]
        )
    ])
