import threading
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import Image

from robot_swiss_knife_msgs.action import QueryVLM
from vlm_query_interface.vlm_interfaces import VLMFrameworks
from vlm_query_interface.vlm_interfaces.query_interface_factory import QueryInterfaceFactory

class VLMQueryInterface(Node):
    """A component exposing a VLM query interface.

    @author Alex Mitrevski
    @contact alemitr@chalmers.se

    """
    # name assigned to the object
    name = None

    # OpenCV bridge instance
    cv_bridge = None

    # VLM query interface instance
    vlm_interface = None

    # ROS2 query interface action instance
    query_interface_server = None

    def __init__(self, name='vlm_query_interface'):
        super().__init__(name,
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.cv_bridge = CvBridge()

        vlm_framework_name = self.get_parameter_or('vlm_framework_name',
                                                   rclpy.Parameter('vlm_framework_name', rclpy.Parameter.Type.STRING, VLMFrameworks.transformers)).value
        vlm_name = self.get_parameter_or('vlm_name', rclpy.Parameter('vlm_name', rclpy.Parameter.Type.STRING, 'remyxai/SpaceOm')).value
        system_prompt = self.get_parameter_or('system_prompt', rclpy.Parameter('system_prompt', rclpy.Parameter.Type.STRING, '')).value
        vlm_query_interface_action_name = self.get_parameter_or('vlm_query_interface_action_name',
                                                                rclpy.Parameter('vlm_query_interface_action_name', rclpy.Parameter.Type.STRING, 'query_vlm')).value

        self.init_vlm_interface(vlm_framework_name=vlm_framework_name,
                                vlm_name=vlm_name,
                                system_prompt=system_prompt)

        self.get_logger().info(f'[{self.name}] Exposing query interface action "{vlm_query_interface_action_name}"')
        self.query_interface_server = ActionServer(self, QueryVLM, vlm_query_interface_action_name, self.query_vlm_cb)
        self.get_logger().info(f'[{self.name}] VLM query interface ready')

    def init_vlm_interface(self, vlm_framework_name: str,
                           vlm_name: str,
                           system_prompt: str) -> None:
        """Initialises a VLM query interface.        

        Keyword arguments:
        vlm_framework: str -- The name of the model framework that should be used (such as ollama or transformers)
        vlm_name: str -- The name of the VLM model to use for queries
        system_prompt: tuple[str] -- The system prompt to use in queries

        """
        self.get_logger().info(f'[{self.name}] Initialising query interface for model {vlm_name} from framework {vlm_framework_name}')
        self.vlm_interface = QueryInterfaceFactory.get_vlm_interface(framework=vlm_framework_name,
                                                                     vlm_name=vlm_name,
                                                                     system_prompt=system_prompt)
        self.get_logger().info(f'[{self.name}] Query interface initialised successfully')

    def query_vlm_cb(self, goal_handle: QueryVLM.Goal) -> QueryVLM.Result:
        """VLM query callback. The result contains the query result and
        a flag indicating whether there were any errors in the query processing
        (which might indicate that the results are incomplete).

        Keyword arguments:
        goal_handle: Action goal handle

        """
        self.get_logger().info(f'[{self.name}] Received VLM query request with query {goal_handle.request.query}')

        # we only process the goal image message if it actually contains any data
        image = None
        if goal_handle.request.image.data:
            image = self.convert_ros_img_to_array(goal_handle.request.image)

        self.get_logger().info(f'[{self.name}] Sending query and waiting for result...')
        (response, query_success) = self.vlm_interface.query_model(query=goal_handle.request.query,
                                                                   image=image)
        self.get_logger().info(f'[{self.name}] Query result received')

        goal_handle.succeed()
        result = QueryVLM.Result()
        if query_success:
            result.query_status = QueryVLM.Result.QUERY_SUCCESS
        else:
            result.query_status = QueryVLM.Result.QUERY_FAILURE
        result.answer = response
        return result

    def convert_ros_img_to_array(self, img_msg: Image) -> np.ndarray:
        """Converts the ROS image message to a numpy array of type uint8.

        Keyword arguments;
        img_msg: sensor_msgs.msg.Image -- The image message to be converted

        """
        image_array = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        image_array = np.array(image_array, dtype=np.uint8)
        return image_array

def main(args=None):
    rclpy.init(args=args)
    query_interface = VLMQueryInterface()
    rate = query_interface.create_rate(5, query_interface.get_clock())

    thread = threading.Thread(target=rclpy.spin, args=(query_interface,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            rate.sleep()
    except:
        pass

    print('Destroying node')
    query_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()