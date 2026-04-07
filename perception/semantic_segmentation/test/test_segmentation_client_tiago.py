import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image

from robot_swiss_knife_msgs.srv import SegmentObjects

class TestSegmentationClient(Node):
    def __init__(self):
        super().__init__('test_segmentation_client')
        self.latest_image_msg = None

        try:
            qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             history=QoSHistoryPolicy.KEEP_LAST,
                             depth=5)
            self.image_sub = self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.image_cb, qos)
        except Exception as exc:
            print(f'{exc}')

        self.client = self.create_client(SegmentObjects, 'segment_objects')
        while not self.client.wait_for_service(timeout_sec=1.):
            self.get_logger().info(f'Service "segment_objects" not available, waiting...')

    def get_segmentation_masks(self):
        if self.latest_image_msg is None:
            self.get_logger().info(f'An image has not been received yet; cannot send service request')
            return

        request = SegmentObjects.Request()
        request.object_categories = ["person"]
        request.image = self.latest_image_msg

        self.get_logger().info(f'Calling client to extract segmentation masks of classes {request.object_categories}')
        response = self.client.call(request)
        return response

    def image_cb(self, image_msg):
        self.latest_image_msg = image_msg

def main(args=None):
    rclpy.init(args=args)
    segmentation_client = TestSegmentationClient()
    rate = segmentation_client.create_rate(5, segmentation_client.get_clock())

    thread = threading.Thread(target=rclpy.spin, args=(segmentation_client,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            if segmentation_client.latest_image_msg is not None:
                segmentation_client_response = segmentation_client.get_segmentation_masks()
                if segmentation_client_response is not None:
                    segmentation_client.get_logger().info(f'Detected mask categories: {segmentation_client_response.mask_categories}')
                    segmentation_client.get_logger().info(f'Segmentation scores: {segmentation_client_response.segmentation_scores}')
                    break
            rate.sleep()
    except Exception as exc:
        print(f'{exc}')

    print('Destroying node')
    segmentation_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
