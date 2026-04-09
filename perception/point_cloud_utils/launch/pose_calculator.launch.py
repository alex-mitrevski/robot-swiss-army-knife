from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="point_cloud_utils",
            executable="pose_calculator",
            name="pose_calculator_node",
            parameters = [
                {"pose_calculation_srv_name": "/calculate_pose_from_cloud"}
            ]
        )
    ])
