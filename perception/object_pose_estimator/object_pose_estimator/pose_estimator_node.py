"""Object pose estimator with Kalman filtering and occlusion-aware state machine.

Architecture
────────────
Subscriptions
  /xtion/rgb/image_raw               latest RGB frame
  /xtion/depth_registered/points     latest aligned organised point cloud
  /xtion/rgb/camera_info             intrinsic matrix K (latched, read once)
  /joint_states                      gripper finger positions

Service clients
  SegmentObjects → Object (mask + category + SAM2 score) from ObjectSegmenter node

Service server
  GetObjectPose → caller queries current filtered pose for a category

Pipeline timer
  Each frame: segment → mask-area occlusion ratio → depth (PC + DA) →
             centroid correction → Kalman update

D1  TRACKING           - full Kalman predict + update
D2  PARTIALLY_OCCLUDED - Kalman predict + update with inflated R
D3a FULLY_OCCLUDED     - Kalman predict only (dead reckoning)
D3* GRASPED            - track via TF2 end-effector + stored grasp offset (beta)
"""

import time
import threading
import numpy as np
import torch
import cv2
from sensor_msgs.msg import Image

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, JointState
from sensor_msgs_py import point_cloud2 as pc2
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from robot_swiss_knife_msgs.srv import SegmentObjects, GetObjectPose

from .constants import (
    PROCESS_NOISE_POSITION, PROCESS_NOISE_VELOCITY, MEASUREMENT_NOISE_POSITION,
    OCCLUSION_R_SCALE, VELOCITY_ZERO_THRESH,
    PARTIAL_OCCLUSION_THRESHOLD, FULL_OCCLUSION_THRESHOLD, MAX_OCCLUDED_AGE_S,
    GRIPPER_GRASP_THRESHOLD, GRIPPER_JOINT_NAMES,
    EEF_FRAME, CAMERA_FRAME,
    ESTIMATION_RATE_HZ, MIN_SAM_SCORE, MIN_MASK_AREA_PX,
    ROTATION_PROCESS_NOISE_Q, ROTATION_PROCESS_NOISE_W,
    ROTATION_MEASUREMENT_NOISE,
    ANGULAR_VELOCITY_ZERO_THRESH,
)
from .kalman_tracker import KalmanTracker
from .occlusion_state_machine import OcclusionStateMachine, TrackingState
from .rotation_tracker import RotationTracker
from .depth_estimator import DepthEstimator
from .pose_helpers import (
    _correct_centroid_partial_occ, _select_z,
    _pca_axes, _z_from_pointcloud, _z_from_depth_map,
)

def _pick_device() -> str:
    if torch.cuda.is_available():
        return "cuda"
    if torch.backends.mps.is_available():
        return "mps"
    return "cpu"

def _quat_to_rotation_matrix(x, y, z, w) -> np.ndarray:
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
        [  2*(x*y + z*w), 1 - 2*(x*x + z*z),   2*(y*z - x*w)],
        [  2*(x*z - y*w),   2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])

class PoseEstimatorNode(Node):
    """ROS2 node that maintains a Kalman-filtered pose estimate for one object
    category and serves it on demand via the GetObjectPose service."""

    name = None

    def __init__(self, name='pose_estimator'):
        super().__init__(name,
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.name = name

        # ROS parameters 
        self.object_category = self.get_parameter_or(
            'object_category',
            rclpy.Parameter('object_category', rclpy.Parameter.Type.STRING, 'apple')
        ).value

        pose_srv_name = self.get_parameter_or(
            'pose_srv_name',
            rclpy.Parameter('pose_srv_name', rclpy.Parameter.Type.STRING, 'get_object_pose')
        ).value

        segment_srv = self.get_parameter_or(
            'segment_srv_name',
            rclpy.Parameter('segment_srv_name', rclpy.Parameter.Type.STRING, 'segment_objects')
        ).value

        # Shared state (subscribers write, timer reads)
        self._lock = threading.Lock()
        self.latest_image: Image | None = None
        self.latest_cloud: PointCloud2 | None = None
        self.camera_K: np.ndarray | None = None
        self.gripper_positions: dict = {}

        # ML models
        self._device = _pick_device()
        self.get_logger().info(f'[{self.name}] Using device: {self._device}')
        self._bridge = CvBridge()
        self._depth_est = DepthEstimator(device=self._device)

        # Tracking state
        self.kalman = KalmanTracker(
            dt=1.0 / ESTIMATION_RATE_HZ,
            process_noise_pos=PROCESS_NOISE_POSITION,
            process_noise_vel=PROCESS_NOISE_VELOCITY,
            measurement_noise=MEASUREMENT_NOISE_POSITION,
        )

        self._rot_tracker = RotationTracker(
            dt=1.0 / ESTIMATION_RATE_HZ,
            process_noise_q=ROTATION_PROCESS_NOISE_Q,
            process_noise_w=ROTATION_PROCESS_NOISE_W,
            measurement_noise=ROTATION_MEASUREMENT_NOISE,
        )

        self.state_machine = OcclusionStateMachine(
            partial_threshold=PARTIAL_OCCLUSION_THRESHOLD,
            full_threshold=FULL_OCCLUSION_THRESHOLD,
        )
        self._da_scale_state = {"ratio": 1.0, "calibrated": False}

        # Occlusion ratio: mask area with depth correction
        self._ref_mask_area = 0         
        self._ref_z: float | None = None   
        self._last_z: float | None = None  

        # Reference clean bbox for centroid correction
        self._clean_bbox = None   # (x1, y1, x2, y2) from last clean frame
        self._ref_box_W  = 0.0   
        self._ref_box_H  = 0.0   

        self.last_known_pose: PoseStamped | None = None
        self.last_known_orientation = None
        self.occluded_since: float | None = None #beta
        self.grasp_offset: np.ndarray | None = None

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribers 
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, '/xtion/rgb/image_raw',
                                 self._image_cb, sensor_qos)
        self.create_subscription(PointCloud2, '/xtion/depth_registered/points',
                                 self._cloud_cb, sensor_qos)
        self.create_subscription(CameraInfo, '/xtion/rgb/camera_info',
                                 self._camera_info_cb, sensor_qos)
        self.create_subscription(JointState, '/joint_states',
                                 self._joint_states_cb, 10)

        # Service clients
        self.segment_client = self.create_client(SegmentObjects, segment_srv)

        # Service server
        self.create_service(GetObjectPose, pose_srv_name, self._get_pose_cb)

        self.get_logger().info(
            f'[{self.name}] Tracking "{self.object_category}" | '
            f'serving "{pose_srv_name}"'
        )

        #Pulishers
        self._pose_pub = self.create_publisher(PoseStamped, 'object_pose', 10)
        self._mask_pub = self.create_publisher(Image, 'object_mask', 10)
        self._viz_pub = self.create_publisher(Image, 'kalman_viz', 10)

    # Subscriber callbacks 

    def _image_cb(self, msg: Image) -> None:
        with self._lock:
            self.latest_image = msg

    def _cloud_cb(self, msg: PointCloud2) -> None:
        with self._lock:
            self.latest_cloud = msg

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        if self.camera_K is None:
            self.camera_K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.get_logger().info(f'[{self.name}] Camera intrinsics loaded')

    def _joint_states_cb(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name in GRIPPER_JOINT_NAMES:
                self.gripper_positions[name] = pos

    # Service server

    def _get_pose_cb(self, request: GetObjectPose.Request,
                     response: GetObjectPose.Response) -> GetObjectPose.Response:
        if request.object_category != self.object_category:
            response.success = False
            response.tracking_state = 'UNKNOWN_CATEGORY'
            return response

        if self.last_known_pose is None:
            response.success = False
            response.tracking_state = self.state_machine.state.name
            return response

        response.pose = self.last_known_pose
        response.tracking_state = self.state_machine.state.name
        response.success = True
        return response

    # Pipeline

    def run_pipeline(self) -> None:
        with self._lock:
            image_msg = self.latest_image
            cloud_msg = self.latest_cloud

        if image_msg is None or cloud_msg is None:
            return

        gripper_closed = self._gripper_is_closed()

        # Step 1: segment via ObjectSegmenter service
        detection_succeeded, mask_np, bbox_xyxy = self._run_segmentation(image_msg)

        # Step 2: occlusion ratio from mask area
        occlusion_ratio = 0.0
        if detection_succeeded and mask_np is not None:
            current_area = int(mask_np.sum())
            if current_area > self._ref_mask_area:
                self._ref_mask_area = current_area
                self._ref_z = self._last_z
            if self._ref_mask_area > 0:
                if self._ref_z and self._last_z:
                    z_ratio = float(np.clip(self._last_z / self._ref_z, 0.8, 1.2))
                    expected_area = self._ref_mask_area / (z_ratio ** 2)
                else:
                    expected_area = self._ref_mask_area
                occlusion_ratio = max(0.0, 1.0 - current_area / expected_area)

        # Step 3: state machine transition
        state = self.state_machine.update(occlusion_ratio,
                                          detection_succeeded,
                                          gripper_closed)

        # Step 4: pose update based on state
        if state == TrackingState.GRASPED:
            self._handle_grasped()

        elif state in (TrackingState.TRACKING, TrackingState.PARTIALLY_OCCLUDED):
            self.occluded_since = None

            xyz = self._cloud_to_xyz(cloud_msg)

            position, orientation_q = self._run_pose_estimation(
                image_msg, xyz, mask_np, bbox_xyxy, occlusion_ratio
            )

            if position is not None:
                # Update clean reference bbox when unoccluded
                if bbox_xyxy is not None and occlusion_ratio <= 0.05:
                    x1, y1, x2, y2 = bbox_xyxy
                    self._clean_bbox = tuple(bbox_xyxy)
                    self._ref_box_W  = max(self._ref_box_W, float(x2 - x1))
                    self._ref_box_H  = max(self._ref_box_H, float(y2 - y1))

                if orientation_q is not None:
                    self.last_known_orientation = orientation_q

                if not self.kalman.initialized:
                    self.kalman.initialize(position)
                    predicted = position
                else:
                    self.kalman.predict()
                    predicted = self.kalman.update(position, occlusion_ratio,
                                                   OCCLUSION_R_SCALE)

                self._publish_and_store_pose(predicted, self.last_known_orientation,
                                             CAMERA_FRAME)
            else:
                if self.kalman.initialized:
                    predicted = self.kalman.predict()
                    if self._rot_tracker.initialized:
                        self._rot_tracker.predict()
                    self._publish_and_store_pose(predicted, self.last_known_orientation,
                                                 CAMERA_FRAME)

        elif state == TrackingState.FULLY_OCCLUDED:
            now = time.monotonic()
            if self.occluded_since is None:
                self.occluded_since = now

            if now - self.occluded_since > MAX_OCCLUDED_AGE_S:
                self.get_logger().warn(
                    f'[{self.name}] Object lost for >{MAX_OCCLUDED_AGE_S}s, resetting'
                )
                self.kalman.reset()
                self._rot_tracker.initialized = False
                self._ref_mask_area = 0
                self._ref_z = None
                self._last_z = None
                self.occluded_since = None
                return

            if self.kalman.initialized:
                vel = self.kalman.get_velocity()
                if np.linalg.norm(vel) < VELOCITY_ZERO_THRESH:
                    self.kalman.zero_velocity()
                predicted = self.kalman.predict()

                if self._rot_tracker.initialized:
                    w = self._rot_tracker.get_angular_velocity()
                    if np.linalg.norm(w) < ANGULAR_VELOCITY_ZERO_THRESH:
                        self._rot_tracker.zero_angular_velocity()
                    self._rot_tracker.predict()

                self._publish_and_store_pose(predicted, self.last_known_orientation,
                                             CAMERA_FRAME)

        self._publish_viz(image_msg, mask_np)
        self.get_logger().debug(
            f'[{self.name}] state={state.name} occ={occlusion_ratio:.3f} '
            f'ref_area={self._ref_mask_area}'
        )

    # Segmentation

    def _run_segmentation(self, image_msg: Image):
        """Call ObjectSegmenter service. Returns (succeeded, mask_np, bbox_xyxy)."""
        req = SegmentObjects.Request()
        req.image = image_msg
        req.object_categories = [self.object_category]

        result = self._call_service_sync(self.segment_client, req)
        if result is None or not result.objects:
            return False, None, None

        obj = max(result.objects, key=lambda o: o.probability)
        if obj.probability < MIN_SAM_SCORE:
            return False, None, None

        arr = self._bridge.imgmsg_to_cv2(obj.view.mask, desired_encoding='passthrough')
        mask_np = (arr > 0).astype(np.uint8)
        img_h, img_w = image_msg.height, image_msg.width
        if mask_np.shape != (img_h, img_w):
            mask_np = cv2.resize(mask_np, (img_w, img_h), interpolation=cv2.INTER_NEAREST)

        if int(mask_np.sum()) < MIN_MASK_AREA_PX:
            return False, None, None

        bbox_xyxy = self._bbox_from_mask_np(mask_np)

        mask_msg = self._bridge.cv2_to_imgmsg(mask_np * 255, encoding='mono8')
        mask_msg.header = image_msg.header
        self._mask_pub.publish(mask_msg)

        return True, mask_np, bbox_xyxy

    # Pose estimation

    def _run_pose_estimation(self, image_msg, xyz, mask_np, bbox_xyxy,
                              occlusion_ratio):
        if mask_np is None or not mask_np.any():
            return None, None

        K = self.camera_K
        if K is None:
            return None, None
        fx, fy   = K[0, 0], K[1, 1]
        cx_cam, cy_cam = K[0, 2], K[1, 2]

        # Depth from point cloud (sparse, accurate scale reference)
        z_pc = _z_from_pointcloud(xyz, mask_np) if xyz is not None else None

        # Depth from Depth Anything V2 (dense, calibrated via EMA ratio)
        rgb = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding='rgb8')
        depth_map = self._depth_est.estimate(rgb)
        z_da = _z_from_depth_map(mask_np, depth_map)

        z_selected, _ = _select_z(z_pc, z_da, self._da_scale_state)
        if z_selected is None or z_selected <= 0:
            return None, None
        self._last_z = z_selected

        # Corrected 2D centroid (u, v) in pixels
        img_H, img_W = mask_np.shape[:2]
        if (bbox_xyxy is not None
                and self._clean_bbox is not None
                and self._ref_box_W > 0
                and occlusion_ratio > 0.05):
            u, v = _correct_centroid_partial_occ(
                *bbox_xyxy, self._clean_bbox,
                self._ref_box_W, self._ref_box_H,
                img_W, img_H,
            )
        elif bbox_xyxy is not None:
            x1, y1, x2, y2 = bbox_xyxy
            u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        else:
            rows, cols = np.where(mask_np > 0)
            if len(rows) == 0:
                return None, None
            u, v = float(cols.mean()), float(rows.mean())

        # Back-project corrected 2D centroid to 3D using selected depth
        position = np.array([
            (u - cx_cam) * z_selected / fx,
            (v - cy_cam) * z_selected / fy,
            z_selected,
        ])

        # Rotation from PCA on the organised cloud masked to object pixels
        orientation_q = None
        if xyz is not None:
            _, axes = _pca_axes(xyz, mask_np)
            if axes is not None:
                if not self._rot_tracker.initialized:
                    axes[2] = np.cross(axes[0], axes[1])
                    self._rot_tracker.initialize(axes)
                else:
                    R_pred = self._rot_tracker.predict()
                    if R_pred is not None:
                        for i in range(2):
                            if np.dot(axes[i], R_pred[i]) < 0:
                                axes[i] = -axes[i]
                        axes[2] = np.cross(axes[0], axes[1])
                    self._rot_tracker.update(axes, occlusion_ratio, OCCLUSION_R_SCALE)

                    w_post = self._rot_tracker.get_angular_velocity()
                    if np.linalg.norm(w_post) > ANGULAR_VELOCITY_ZERO_THRESH:
                        self._rot_tracker.zero_angular_velocity()

                q = self._rot_tracker.get_quaternion()   # [w, x, y, z]
                orientation_q = (float(q[1]), float(q[2]),
                                 float(q[3]), float(q[0]))  # xyzw

        return position, orientation_q

    # GRASPED state

    def _handle_grasped(self) -> None:
        try:
            t = self.tf_buffer.lookup_transform(
                CAMERA_FRAME, EEF_FRAME, rclpy.time.Time()
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'[{self.name}] TF2 eef lookup failed: {e}')
            return

        eef_pos = np.array([
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
        ])
        q = t.transform.rotation
        R_eef = _quat_to_rotation_matrix(q.x, q.y, q.z, q.w)

        if self.grasp_offset is None and self.kalman.initialized:
            obj_pos = self.kalman.get_position()
            self.grasp_offset = R_eef.T @ (obj_pos - eef_pos)
            self.get_logger().info(
                f'[{self.name}] Grasp offset recorded: {self.grasp_offset}'
            )

        if self.grasp_offset is None:
            return

        obj_pos = eef_pos + R_eef @ self.grasp_offset
        self.kalman.predict()
        self.kalman.update(obj_pos, occlusion_ratio=0.0)
        self._publish_and_store_pose(obj_pos, self.last_known_orientation, CAMERA_FRAME)

    def _call_service_sync(self, client, request):
        """Send a service request and block until the response arrives."""
        if not client.wait_for_service(timeout_sec=0.1):
            return None
        future = client.call_async(request)
        while not future.done():
            time.sleep(0.005)
        return future.result()

    def _cloud_to_xyz(self, cloud_msg: PointCloud2) -> np.ndarray | None:
        """Convert an organised PointCloud2 to (H, W, 3) float32 in metres."""
        try:
            H, W = cloud_msg.height, cloud_msg.width
            if H <= 1:
                return None  
            pts = pc2.read_points_numpy(
                cloud_msg, field_names=('x', 'y', 'z'), skip_nans=False
            )
            if pts.dtype.names:
                pts = np.stack([pts['x'], pts['y'], pts['z']], axis=-1)
            return pts.reshape(H, W, 3).astype(np.float32)
        except Exception as e:
            self.get_logger().warn(f'[{self.name}] Cloud conversion failed: {e}')
            return None

    def _bbox_from_mask_np(self, mask_np: np.ndarray) -> np.ndarray | None:
        """Derive [x1,y1,x2,y2] bounding box from a binary numpy mask."""
        ys, xs = np.where(mask_np > 0)
        if len(xs) == 0:
            return None
        return np.array([xs.min(), ys.min(), xs.max(), ys.max()], dtype=np.float32)

    def _gripper_is_closed(self) -> bool:
        if not self.gripper_positions:
            return False
        return all(
            self.gripper_positions.get(j, 1.0) < GRIPPER_GRASP_THRESHOLD
            for j in GRIPPER_JOINT_NAMES
        )

    def _publish_and_store_pose(self, position: np.ndarray,
                                orientation,
                                frame_id: str) -> None:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])

        if orientation is not None:
            pose.pose.orientation.x = float(orientation[0])
            pose.pose.orientation.y = float(orientation[1])
            pose.pose.orientation.z = float(orientation[2])
            pose.pose.orientation.w = float(orientation[3])
        else:
            pose.pose.orientation.w = 1.0

        self.last_known_pose = pose
        self._pose_pub.publish(pose)

        tf_msg = TransformStamped()
        tf_msg.header = pose.header
        tf_msg.child_frame_id = f'estimated_{self.object_category.replace(" ", "_")}'
        tf_msg.transform.translation.x = pose.pose.position.x
        tf_msg.transform.translation.y = pose.pose.position.y
        tf_msg.transform.translation.z = pose.pose.position.z
        tf_msg.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

    def _publish_viz(self, image_msg: Image, mask_np) -> None:
        try:
            vis = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding='rgb8')
        except Exception as exc:
            self.get_logger().error(str(exc))

        if mask_np is not None and mask_np.any():
            overlay = vis.copy()
            overlay[mask_np > 0] = [0, 200, 0]
            vis_h, vis_w = vis.shape[:2]
            mask_draw = (cv2.resize(mask_np, (vis_w, vis_h), interpolation=cv2.INTER_NEAREST) if mask_np.shape[:2] != (vis_h, vis_w) else mask_np)
            overlay[mask_draw > 0] = [0, 200, 0]
            cv2.addWeighted(overlay, 0.35, vis, 0.65, 0, vis)
        if self.last_known_pose is not None and self.camera_K is not None:
            K = self.camera_K
            p = self.last_known_pose.pose.position
            if p.z > 0:
                u = int(K[0, 0] * p.x / p.z + K[0, 2])
                v = int(K[1, 1] * p.y / p.z + K[1, 2])
                H, W = vis.shape[:2]
                if 0 <= u < W and 0 <= v < H:
                    cv2.circle(vis, (u, v), 10, (0, 100, 255), -1)
                    cv2.circle(vis, (u, v), 12, (255, 255, 255), 2)

        viz_msg = Image()
        viz_msg.header = image_msg.header
        viz_msg.height, viz_msg.width = vis.shape[:2]
        viz_msg.encoding = 'rgb8'
        viz_msg.step = viz_msg.width * 3
        viz_msg.data = vis.astype(np.uint8).tobytes()
        self._viz_pub.publish(viz_msg)
        viz_msg.header = image_msg.header
        self._viz_pub.publish(viz_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorNode()
    rate = node.create_rate(ESTIMATION_RATE_HZ, node.get_clock())

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            node.run_pipeline()
            rate.sleep()
    except Exception as exc:
        node.get_logger().error(str(exc))

    node.get_logger().info('Destroying pose estimator node')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
