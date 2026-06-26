#!/usr/bin/env python3
# Author: Alex Mitrevski

import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('task_planner_interface')

    planner_config_file = os.path.join(package_dir, 'config', 'planner_config.yaml')
    with open(planner_config_file, 'r') as f:
        planner_config = yaml.safe_load(f)

    return LaunchDescription([
        Node(package='task_planner_interface',
             executable='task_planner_interface_node',
             name='task_planner_interface',
             namespace='planner',
             parameters=[
                 {'kb_db_name': 'robot_store',
                  'planner_name': planner_config['planner_name'], 
                  'domain_file_path': os.path.join(package_dir, 'config', 'task_domains', planner_config['domain_file']),
                  'planner_cmd': planner_config['planner_cmd'],
                  'plan_file_path': os.path.join(package_dir, planner_config['plan_file_path']),
                  'get_fluent_assertions_srv_name': 'get_fluent_assertions',
                  'update_kb_srv_name': 'update_kb',
                  'update_goals_srv_name': 'update_goals',
                  'plan_action_name': 'plan'}
             ]
        )
    ])