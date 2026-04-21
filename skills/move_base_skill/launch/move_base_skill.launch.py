from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    pkg = 'move_base_skill'
    node = 'move_base_skill'
    executable = 'move_base'
    ld = LaunchDescription()

    node = LifecycleNode(package=pkg,
                         executable=executable,
                         namespace='',
                         name=node,
                         parameters = [
                             {"nav_action_name": "/navigate_to_pose",
                              "say_action_name": "/tts_engine/tts",
                              "base_link_frame_name": "base_footprint",
                              "map_frame_name": "map",
                              "max_pose_timeout_s": 30.0,
                              "debug_mode": False
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
