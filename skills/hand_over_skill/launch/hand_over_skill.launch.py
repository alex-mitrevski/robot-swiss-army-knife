from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    pkg = 'hand_over_skill'
    node = 'hand_over_skill'
    executable = 'hand_over'
    ld = LaunchDescription()

    node = LifecycleNode(package=pkg,
                         executable=executable,
                         namespace='',
                         name=node,
                         parameters = [
                             {"trajectory_execution_topic": "get_trajectory",
                              "trajectory_execution_result_topic": "trajectory_execution_result",
                              "gripper_trajectory_topic": "/gripper_controller/joint_trajectory",
                              "camera_image_topic": "/head_front_camera/rgb/image_raw",
                              "ft_sensor_topic": "/ft_sensor_controller/wrench",
                              "estimate_anthropometric_parameters": True,
                              "anthropometric_parameter_estimation_srv": "estimate_anthropometric_parameters",
                              "say_action_name": "/tts_engine/tts",
                              "min_wrench_measurements_for_handover_detection": 10,
                              "max_wrench_measurements": 100,
                              "wrench_filter_window_size": 5,
                              "handover_force_threshold_N": 5.,
                              "handover_timeout_s": 20.,
                              "gripper_joint_names": ["gripper_left_finger_joint", "gripper_right_finger_joint"],
                              "gripper_joint_opening_angles": [0.025, 0.045]
                             }
                         ],
                         output='both')

    ld.add_action(node)

    configure_event = EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node),
                                                  transition_id=Transition.TRANSITION_CONFIGURE))
    ld.add_action(configure_event)

    activate_event = RegisterEventHandler(OnStateTransition(target_lifecycle_node=node,
                                                            goal_state='inactive',
                                                            entities=[EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node),
                                                                                                  transition_id=Transition.TRANSITION_ACTIVATE))],
                                                            handle_once=True))
    ld.add_action(activate_event)

    return ld
