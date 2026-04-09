import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped

from robot_swiss_knife_msgs.srv import SegmentObjects, ExtractROI3DPoints, CalculatePoseFromCloud

class TestPoseEstimation(Node):
    def __init__(self):
        super().__init__('test_pose_estimation')
        self.latest_image_msg = None
        self.latest_cloud_msg = None

        try:
            qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             history=QoSHistoryPolicy.KEEP_LAST,
                             depth=5)
            self.image_sub = self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.image_cb, qos)

            qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             history=QoSHistoryPolicy.KEEP_LAST,
                             depth=5)
            self.cloud_sub = self.create_subscription(PointCloud2, '/head_front_camera/depth/rgb/points', self.cloud_cb, qos)
        except Exception as exc:
            self.get_logger().error(f'{exc}')
            raise

        # object cloud publisher for debugging purposes
        self.cloud_pub = self.create_publisher(PointCloud2, 'debugging_cloud', 10)

        # calculated pose publisher for debugging purposes
        self.pose_pub = self.create_publisher(PoseStamped, 'debugging_pose', 10)

        self.segmentation_client = self.create_client(SegmentObjects, 'segment_objects')
        while not self.segmentation_client.wait_for_service(timeout_sec=1.):
            self.get_logger().info(f'Service "segment_objects" not available, waiting...')

        self.roi_client = self.create_client(ExtractROI3DPoints, 'extract_3d_rois')
        while not self.roi_client.wait_for_service(timeout_sec=1.):
            self.get_logger().info(f'Service "extract_3d_rois" not available, waiting...')

        self.pose_client = self.create_client(CalculatePoseFromCloud, 'calculate_pose_from_cloud')
        while not self.pose_client.wait_for_service(timeout_sec=1.):
            self.get_logger().info(f'Service "calculate_pose_from_cloud" not available, waiting...')

        self.get_logger().info(f'Initialisation complete')

    def get_segmentation_masks(self, object_categories):
        if self.latest_image_msg is None:
            self.get_logger().info(f'An image has not been received yet; cannot send service request')
            return

        request = SegmentObjects.Request()
        request.object_categories = object_categories
        request.image = self.latest_image_msg

        self.get_logger().info(f'Calling client to extract segmentation masks of classes {request.object_categories}')
        response = self.segmentation_client.call(request)
        return response

    def get_roi_points(self, masks):
        if self.latest_cloud_msg is None:
            self.get_logger().info(f'A cloud has not been received yet; cannot send service request')
            return

        request = ExtractROI3DPoints.Request()
        request.point_cloud = self.latest_cloud_msg
        request.object_masks = masks

        self.get_logger().info(f'Calling client to extract object point clouds')
        response = self.roi_client.call(request)
        return response

    def get_pose(self, cloud):
        request = CalculatePoseFromCloud.Request()
        request.point_cloud = cloud

        self.get_logger().info(f'Calling client to calculate pose')
        response = self.pose_client.call(request)
        return response

    def image_cb(self, image_msg):
        self.latest_image_msg = image_msg

    def cloud_cb(self, cloud_msg):
        self.latest_cloud_msg = cloud_msg

def main(args=None):
    rclpy.init(args=args)
    pose_estimator = TestPoseEstimation()
    rate = pose_estimator.create_rate(5, pose_estimator.get_clock())

    thread = threading.Thread(target=rclpy.spin, args=(pose_estimator,), daemon=True)
    thread.start()

    object_categories = ["cup"]

    try:
        while rclpy.ok():
            time.sleep(5)
            if pose_estimator.latest_image_msg is not None:
                pose_estimator.get_logger().info(f'Attempting to get segmentation masks of categories {object_categories}')
                pose_estimator_response = pose_estimator.get_segmentation_masks(object_categories)
                if pose_estimator_response is not None:
                    pose_estimator.get_logger().info(f'Segmented objects of categories {pose_estimator_response.mask_categories}')
                    pose_estimator.get_logger().info(f'Segmentation scores: {pose_estimator_response.segmentation_scores}')

                    if len(pose_estimator_response.masks) != 0:
                        pose_estimator.get_logger().info('Attempting to get ROI point clouds for the segmented objects')
                        roi_extraction_response = pose_estimator.get_roi_points(pose_estimator_response.masks)
                        if roi_extraction_response is not None:
                            pose_estimator.get_logger().info(f'Extracted {len(roi_extraction_response.object_clouds)} point clouds')
                            if len(roi_extraction_response.object_clouds) != 0:
                                pose_estimator.get_logger().info(f'Publishing the cloud for the first object with {len(roi_extraction_response.object_clouds[0].data)} number of points on topic /debugging_cloud')
                                pose_estimator.cloud_pub.publish(roi_extraction_response.object_clouds[0])

                                pose_estimator.get_logger().info('Attempting to calculate the pose of the first object')
                                pose_calculation_response = pose_estimator.get_pose(roi_extraction_response.object_clouds[0])
                                if pose_calculation_response is not None:
                                    pose_estimator.get_logger().info(f'Found pose: {pose_calculation_response.pose}')
                                    pose_estimator.get_logger().info(f'Publishing the pose on topic /debugging_pose')
                                    pose_estimator.pose_pub.publish(pose_calculation_response.pose)
                    break
            rate.sleep()
    except Exception as exc:
        print(f'{exc}')

    print('Destroying node')
    pose_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
