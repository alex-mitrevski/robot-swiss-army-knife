#!/usr/bin/env python3
# Author: Alex Mitrevski

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    object_categories_file = os.path.join(
        get_package_share_directory('object_visibility_checker'),
        'config',
        'object_classes.yaml'
    ) 

    return LaunchDescription([
        Node(package='object_visibility_checker',
             executable='visibility_checker',
             name='visibility_checker',
             parameters=[
                 {'object_categories_file': object_categories_file,
                  'visibility_checking_srv_name': 'check_objects_visible'}
             ]
        )
    ])