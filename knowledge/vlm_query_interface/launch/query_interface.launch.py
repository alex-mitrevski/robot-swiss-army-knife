#!/usr/bin/env python3
# Author: Alex Mitrevski

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='vlm_query_interface',
             executable='query_interface_node',
             name='vlm_query_interface',
             parameters=[
                 {'vlm_framework_name': 'transformers',
                  'vlm_name': 'remyxai/SpaceOm',
                  'system_prompt': '',
                  'vlm_query_interface_action_name': 'query_vlm'}
             ]
        )
    ])