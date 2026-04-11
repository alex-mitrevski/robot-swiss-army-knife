import os
import yaml
import threading
import numpy as np

from cv_bridge import CvBridge
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from robot_swiss_knife_msgs.srv import CheckObjectVisibility

class VisibilityChecker(Node):
    """A component for checking whether objects are visible in the currently visible scene.

    @author Alex Mitrevski
    @contact alemitr@chalmers.se

    """
    # name assigned to the object
    name = None

    # OpenCV bridge instance
    cv_bridge = None

    # object detection model
    object_detector = None

    # map of object category indices to category names
    object_category_map = dict()

    # ROS2 visibility checking server instance
    visibility_checking_server = None

    def __init__(self, name='visibility_checker',
                 object_detection_model="yolov8n.pt"):
        super().__init__(name,
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.cv_bridge = CvBridge()

        object_categories_file = self.get_parameter_or('object_categories_file',
                                                       rclpy.Parameter('object_categories_file', rclpy.Parameter.Type.STRING, '')).value
        self.object_category_map = self.read_object_categories_map(object_categories_file)

        visibility_checking_srv_name = self.get_parameter_or('visibility_checking_srv_name',
                                                             rclpy.Parameter('visibility_checking_srv_name', rclpy.Parameter.Type.STRING, 'check_objects_visible')).value

        self.init_detection_model(object_detection_model=object_detection_model)

        self.get_logger().info(f'[{self.name}] Exposing visibility checking service "{visibility_checking_srv_name}"')
        self.visibility_checking_server = self.create_service(CheckObjectVisibility,
                                                              visibility_checking_srv_name,
                                                              self.check_object_visibility_cb)
        self.get_logger().info(f'[{self.name}] Visibility checking component ready')

    def init_detection_model(self, object_detection_model: str) -> None:
        """Loads a model for object detection.

        Keyword arguments:
        object_detection_model: str -- Name of a detection model to be loaded

        """
        self.get_logger().info(f'[{self.name}] Loading object detection model {object_detection_model}')
        self.object_detector = YOLO(object_detection_model, verbose=False)
        self.get_logger().info(f'[{self.name}] Model loaded successfully')

    def check_object_visibility_cb(self, request: CheckObjectVisibility.Request,
                                   response: CheckObjectVisibility.Response) -> CheckObjectVisibility.Response:
        """Checks whether the object categories specified in the request are visible in the image
        passed in the request. The response consists of a list of Boolean values corresponding to each
        category in the request (True if an object of the category is visible, false otherwise)

        Keyword arguments:
        request: robot_swiss_army_msgs.srv.CheckObjectVisibility.Request
        response: robot_swiss_army_msgs.srv.CheckObjectVisibility.Response

        """
        self.get_logger().info(f'[{self.name}] Received new visibility checking request for categories {request.object_categories}')
        image = self.convert_ros_img_to_array(request.image)

        # we get the indices of the object categories in the request
        object_category_indices = [self.get_category_index_for_class(x) for x in request.object_categories]

        # we initialise the result assuming that none of the requested object categories are visible
        request.objects_visible = [False for _ in request.object_categories]

        self.get_logger().info(f'[{self.name}] Detecting objects...')
        detection_results = self.object_detector(image)
        for result in detection_results:
            obj_classes = result.boxes.cls.cpu().numpy()
            for class_idx in obj_classes:
                class_idx = int(class_idx)

                # if the current class is one of the query classes,
                # we set its visibility in the response accordingly
                if class_idx in object_category_indices:
                    response.objects_visible[object_category_indices.index(class_idx)] = True

        self.get_logger().info(f'[{self.name}] Visibility checking complete')
        return response

    def convert_ros_img_to_array(self, img_msg: Image) -> np.ndarray:
        """Converts the ROS image message to a numpy array of type uint8.

        Keyword arguments;
        img_msg: sensor_msgs.msg.Image -- The image message to be converted

        """
        image_array = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        image_array = np.array(image_array, dtype=np.uint8)
        return image_array

    def get_category_index_for_class(self, object_category: str) -> int:
        """Returns the index corresponding to 'object_category' in 'self.object_category_map'.
        Returns -1 if 'object_category' is not found in the map.

        Keyword arguments:
        object_category: str -- Name of an object category

        """
        for idx, category_name in self.object_category_map.items():
            if category_name == object_category:
                return idx
        self.get_logger().warn(f'[{self.name}] {object_category} unknown; will be ignored during detection')
        return -1

    def read_object_categories_map(self, object_classes_file: str) -> dict[int, str]:
        """Loads a dictionary that maps detection class indices to object category names.
        Function taken from https://github.com/b-it-bots/mas_tools/blob/master/src/mas_tools/file_utils.py

        Keyword arguments:
        object_classes_file: str -- Path to the YAML file to be loaded

        """
        if not os.path.isfile(object_classes_file):
            raise OSError(f'{object_classes_file} is not a valid file'.format(object_classes_file))

        _, file_extension = os.path.splitext(object_classes_file)
        if not file_extension:
            raise ValueError('Could not determine the file extension of {0}'.format(object_classes_file))

        allowed_yaml_extension = ['.yaml', '.yml']
        if file_extension not in allowed_yaml_extension:
            raise ValueError('Only extensions {0} are allowed'.format(allowed_yaml_extension))

        object_categories_map = None
        with open(object_classes_file, 'r') as yaml_file:
            object_categories_map = yaml.safe_load(yaml_file)
        return object_categories_map

def main(args=None):
    rclpy.init(args=args)
    visibility_checker = VisibilityChecker()
    rate = visibility_checker.create_rate(5, visibility_checker.get_clock())

    thread = threading.Thread(target=rclpy.spin, args=(visibility_checker,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            rate.sleep()
    except:
        pass

    print('Destroying node')
    visibility_checker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()