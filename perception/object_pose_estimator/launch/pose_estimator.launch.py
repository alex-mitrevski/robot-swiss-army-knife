#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = get_package_share_directory('object_pose_estimator')

    return LaunchDescription([
        Node(
            package='object_pose_estimator',
            executable='pose_estimator',
            name='pose_estimator',
            parameters=[{
                'object_category':      'apple',
                'pose_srv_name':        'get_object_pose',
                'segment_srv_name':     'segment_objects',
                'extract_roi_srv_name': 'extract_3d_rois',
                'pose_calc_srv_name':   'calculate_pose_from_cloud',
            }]
        )
    ])
