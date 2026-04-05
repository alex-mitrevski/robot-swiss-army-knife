import os
import yaml
import threading
import numpy as np

from PIL import Image as PILImage
from cv_bridge import CvBridge
from ultralytics import YOLO
from sam2.sam2_image_predictor import SAM2ImagePredictor

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from robot_swiss_knife_msgs.srv import SegmentObjects

class ObjectSegmenter(Node):
    """Component for performing semantic segmentation on images.
    Some parts of this component are based on snippets by Zhiyuan Jia,
    as well as the Ultralytics YOLO and SAM 2 docs.

    @author Alex Mitrevski
    @contact alemitr@chalmers.se

    """
    # name assigned to the object
    name = None

    # OpenCV bridge instance
    cv_bridge = None

    # object detection model
    object_detector = None

    # object segmentation model
    segmenter = None

    # map of object category indices to category names
    object_category_map = dict()

    # ROS2 segmentation server instance
    segmentation_server = None

    def __init__(self, name='object_segmenter',
                 object_detection_model="yolov8n.pt",
                 segmentation_model='facebook/sam2-hiera-tiny'):
        super().__init__(name,
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.cv_bridge = CvBridge()

        object_categories_file = self.get_parameter_or('object_categories_file',
                                                       rclpy.Parameter('object_categories_file', rclpy.Parameter.Type.STRING, '')).value
        self.object_category_map = self.read_object_categories_map(object_categories_file)

        segmentation_srv_name = self.get_parameter_or('segmentation_srv_name',
                                                      rclpy.Parameter('segmentation_srv_name', rclpy.Parameter.Type.STRING, 'segment_objects')).value

        self.init_models(object_detection_model=object_detection_model,
                         segmentation_model=segmentation_model)

        self.get_logger().info(f'[{self.name}] Exposing segmentation service "{segmentation_srv_name}"')
        self.segmentation_server = self.create_service(SegmentObjects,
                                                       segmentation_srv_name,
                                                       self.segment_objects_cb)
        self.get_logger().info(f'[{self.name}] Segmentation component ready')

    def init_models(self, object_detection_model: str,
                    segmentation_model: str) -> None:
        """Loads models for object detection and object segmentation.

        Keyword arguments:
        object_detection_model: str -- Name of a detection model to be loaded
        segmentation_model: str -- Name of a segmentation model to be loaded

        """
        self.get_logger().info(f'[{self.name}] Loading object detection model {object_detection_model}')
        self.object_detector = YOLO(object_detection_model, verbose=False)

        self.get_logger().info(f'[{self.name}] Loading object segmentation model {segmentation_model}')
        self.segmenter = SAM2ImagePredictor.from_pretrained(segmentation_model, device='cpu')

        self.get_logger().info(f'[{self.name}] Models loaded successfully')

    def segment_objects_cb(self, request: SegmentObjects.Request,
                           response: SegmentObjects.Response) -> SegmentObjects.Response:
        """Segments the requested object categories in the request image.
        The mask of each segmented category is returned as a separate image.

        Keyword arguments:
        request: robot_swiss_army_msgs.srv.SegmentObjects.Request
        response: robot_swiss_army_msgs.srv.SegmentObjects.Response

        """
        self.get_logger().info(f'[{self.name}] Received new segmentation request for categories {request.object_categories}')
        image = self.convert_ros_img_to_array(request.image)

        # we get the indices of the object categories in the request
        object_category_indices = [self.get_category_index_for_class(x) for x in request.object_categories]

        self.get_logger().info(f'[{self.name}] Detecting objects...')
        detection_results = self.object_detector(image)
        boxes = {}
        for result in detection_results:
            boxes_xy = result.boxes.xyxy.cpu().numpy()
            obj_classes = result.boxes.cls.cpu().numpy()
            for box, class_idx in zip(boxes_xy, obj_classes):
                if int(class_idx) in object_category_indices:
                    boxes[class_idx] = box

        if not boxes:
            self.get_logger().error(f'[{self.name}] No objects detected; returning empty result')
            response.masks = []
            return response

        self.get_logger().info(f'[{self.name}] Segmenting objects...')
        self.segmenter.set_image(np.array(PILImage.fromarray(image).convert('RGB')))
        for class_idx, (x1, y1, x2, y2) in boxes.items():
            self.get_logger().info(f'[{self.name}] Segmenting objects of category {self.object_category_map[class_idx]}...')
            masks, scores, _ = self.segmenter.predict(box=np.array([x1, y1, x2, y2], dtype=np.int32))
            mask = masks[np.argmax(scores)].astype(np.uint8)
            response.mask_categories.append(self.object_category_map[class_idx])
            response.segmentation_scores.append(np.max(scores).item())
            response.masks.append(self.convert_array_to_ros_img(mask))

        self.get_logger().info(f'[{self.name}] Segmentation complete')
        return response

    def convert_ros_img_to_array(self, img_msg: Image) -> np.ndarray:
        """Converts the ROS image message to a numpy array of type uint8.

        Keyword arguments;
        img_msg: sensor_msgs.msg.Image -- The image message to be converted

        """
        image_array = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        image_array = np.array(image_array, dtype=np.uint8)
        return image_array

    def convert_array_to_ros_img(self, img_array: np.ndarray) -> Image:
        """Converts the numpy array of type uint8 to a ROS image message.

        Keyword arguments;
        img_array: np.ndarray -- The image to be converted

        """
        image_msg = self.cv_bridge.cv2_to_imgmsg(img_array, encoding='passthrough')
        image_msg.header.stamp = self.get_clock().now().to_msg()
        return image_msg

    def get_category_index_for_class(self, object_category: str) -> int:
        """Returns the index corresponding to 'object_category' in 'self.object_category_map'.
        Returns -1 if 'object_category' is not found in the map.

        Keyword arguments:
        object_category: str -- Name of an object category

        """
        for idx, category_name in self.object_category_map.items():
            if category_name == object_category:
                return idx
        self.get_logger().warn(f'[{self.name}] {object_category} unknown; will be ignored during segmentation')
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
    object_segmenter = ObjectSegmenter()
    rate = object_segmenter.create_rate(5, object_segmenter.get_clock())

    thread = threading.Thread(target=rclpy.spin, args=(object_segmenter,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            rate.sleep()
    except:
        pass

    print('Destroying node')
    object_segmenter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
