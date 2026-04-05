#!/usr/bin/env python3
# Author: Alex Mitrevski

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    object_categories_file = os.path.join(
        get_package_share_directory('semantic_segmentation'),
        'config',
        'object_classes.yaml'
    ) 

    return LaunchDescription([
        Node(package='semantic_segmentation',
             executable='segment_objects',
             name='object_segmenter',
             parameters=[
                 {'object_categories_file': object_categories_file,
                  'segmentation_srv_name': 'segment_objects'}
             ]
        )
    ])