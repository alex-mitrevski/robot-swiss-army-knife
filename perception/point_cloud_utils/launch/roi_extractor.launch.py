from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="point_cloud_utils",
            executable="roi_3d_point_extractor",
            name="roi_3d_point_extractor_node",
            parameters = [
                {"extraction_srv_name": "/extract_3d_rois"}
            ]
        )
    ])
