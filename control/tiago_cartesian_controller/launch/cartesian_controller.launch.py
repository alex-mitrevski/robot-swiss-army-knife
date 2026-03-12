from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cartesian_controller_node = Node(package="tiago_cartesian_controller",
                                     executable="cartesian_controller",
                                     name="cartesian_controller_node",
                                     parameters = [{"goal_topic": "/cartesian_pose_goal",
                                                    "arm_joint_controller_topic": "/arm_vel_controller/joint_trajectory",
                                                    "torso_joint_controller_topic": "/torso_controller/joint_trajectory",
                                                    "joint_states_topic": "/joint_states",
                                                    "base_link_frame_name": "base_footprint",
                                                    "end_effector_frame_name": "arm_tool_link",
                                                    "joint_names": ["torso_lift_joint", "arm_1_joint", "arm_2_joint", "arm_3_joint",
                                                                    "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"],
                                                    "arm_joint_names": ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint",
                                                                        "arm_5_joint", "arm_6_joint", "arm_7_joint"],
                                                    "torso_joint_names": ["torso_lift_joint"],
                                                    "min_arm_joint_vel_limits": [-0.2]*7,
                                                    "max_arm_joint_vel_limits": [0.2]*7,
                                                    "goal_threshold_m": 0.1,
                                                    "goal_threshold_rad": 0.1,
                                                    "debug_mode": False}])

    ld = LaunchDescription([
        cartesian_controller_node
    ])

    return ld
