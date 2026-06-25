#!/usr/bin/env python3
# Author: Alex Mitrevski

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='kb_interface',
             executable='kb_interface_node',
             name='kb_interface',
             namespace='kb',
             parameters=[
                 {'kb_db_name': 'robot_store',
                  'get_fluent_assertions_srv_name': 'get_fluent_assertions',
                  'update_kb_srv_name': 'update_kb'}
             ]
        )
    ])