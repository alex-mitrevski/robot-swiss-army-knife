import threading
import numpy as np
import cv2
import torch
from object_pose_estimator.constants import SEGMENTATION_WIDTH, SEGMENTATION_HEIGHT

from PIL import Image as PILImage
from cv_bridge import CvBridge
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
from sam2.sam2_image_predictor import SAM2ImagePredictor
from sensor_msgs.msg import Image as RosImage



import rclpy
from rclpy.node import Node

from robot_swiss_knife_msgs.srv import SegmentObjects
from robot_swiss_knife_msgs.msg import Object as RobotObject


def _pick_device() -> str:
    if torch.cuda.is_available():
        return "cuda"
    if torch.backends.mps.is_available():
        return "mps"
    return "cpu"


class ObjectSegmenter(Node):
    name = None

    def __init__(self, name='object_segmenter',
                 gdino_model=None,
                 sam2_model=None,
                 box_thresh=0.25):
        super().__init__(name,
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.name = name
        self._bridge = CvBridge()
        self._box_thresh = box_thresh

        segmentation_srv_name = self.get_parameter_or(
            'segmentation_srv_name',
            rclpy.Parameter('segmentation_srv_name',
                            rclpy.Parameter.Type.STRING, 'segment_objects')
        ).value

        gdino_model = self.get_parameter_or(
            'gdino_model',
            rclpy.Parameter('gdino_model', rclpy.Parameter.Type.STRING,
                            'IDEA-Research/grounding-dino-tiny')
        ).value
        sam2_model = self.get_parameter_or(
            'sam2_model',
            rclpy.Parameter('sam2_model', rclpy.Parameter.Type.STRING,
                            'facebook/sam2.1-hiera-base-plus')
        ).value

        self._device = _pick_device()
        self.get_logger().info(f'[{self.name}] Using device: {self._device}')

        gdino_model = gdino_model or 'IDEA-Research/grounding-dino-tiny'
        sam2_model = sam2_model or 'facebook/sam2.1-hiera-base-plus'
        self._init_models(gdino_model, sam2_model)

        self.get_logger().info(
            f'[{self.name}] Exposing segmentation service "{segmentation_srv_name}"'
        )
        self.create_service(SegmentObjects, segmentation_srv_name,
                            self.segment_objects_cb)
        self.get_logger().info(f'[{self.name}] Segmentation component ready')

    # ── Model loading ─────────────────────────────────────────────────────────

    def _init_models(self, gdino_model: str, sam2_model: str) -> None:
        self.get_logger().info(f'[{self.name}] Loading Grounding-DINO ({gdino_model})')
        self._processor = AutoProcessor.from_pretrained(gdino_model)
        self._gdino = AutoModelForZeroShotObjectDetection.from_pretrained(
            gdino_model).to(self._device)
        self._gdino.eval()

        self.get_logger().info(f'[{self.name}] Loading SAM2 ({sam2_model})')
        self._sam2 = SAM2ImagePredictor.from_pretrained(sam2_model,
                                                        device=self._device)
        self.get_logger().info(f'[{self.name}] Models loaded')

    # ── Service callback ──────────────────────────────────────────────────────

    def segment_objects_cb(self, request: SegmentObjects.Request,
                           response: SegmentObjects.Response) -> SegmentObjects.Response:
        self.get_logger().info(
            f'[{self.name}] Segmentation request for {request.object_categories}'
        )
        image = self._bridge.imgmsg_to_cv2(request.image, desired_encoding='rgb8')
        image = np.asarray(image, dtype=np.uint8)
        import cv2
        image = cv2.resize(image, (SEGMENTATION_WIDTH, SEGMENTATION_HEIGHT))
        self._sam2.set_image(image)

        for category in request.object_categories:
            detections = self._gdino_detect(image, category)
            if not detections:
                self.get_logger().warn(
                    f'[{self.name}] GDino found nothing for "{category}"'
                )
                continue

            for obj_idx, (box, _) in enumerate(detections):
                x1, y1, x2, y2 = box
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

                with torch.inference_mode():
                    masks, sam_scores, _ = self._sam2.predict(
                        point_coords=np.array([[cx, cy]], dtype=np.float32),
                        point_labels=np.array([1]),
                        box=np.array([x1, y1, x2, y2], dtype=np.float32),
                        multimask_output=True,
                    )

                best = int(np.argmax(sam_scores))
                mask = masks[best].astype(np.uint8)
                stamp = self.get_clock().now().to_msg()

                obj = RobotObject()
                obj.name = f'{category}_{obj_idx}'
                obj.category = category
                obj.probability = float(sam_scores[best])
                obj.roi.x_offset = int(x1)
                obj.roi.y_offset = int(y1)
                obj.roi.width = int(x2 - x1)
                obj.roi.height = int(y2 - y1)
                #append mask
                mask_msg = RosImage()
                mask_msg.header.stamp = stamp
                mask_msg.height, mask_msg.width = mask.shape[:2]
                mask_msg.encoding = 'mono8'
                mask_msg.step = mask_msg.width
                mask_msg.data = mask.tobytes()
                obj.view.mask = mask_msg

                obj.view.mask.header.stamp = stamp

                crop = image[y1:y2, x1:x2]
                if crop.size > 0:
                    #append cropped image
                    crop = np.ascontiguousarray(crop, dtype=np.uint8)
                    crop_msg = RosImage()
                    crop_msg.header.stamp = stamp
                    crop_msg.height, crop_msg.width = crop.shape[:2]
                    crop_msg.encoding = 'rgb8'
                    crop_msg.step = crop_msg.width * 3
                    crop_msg.data = crop.tobytes()
                    obj.view.image = crop_msg
                    obj.view.image.header.stamp = stamp

                response.objects.append(obj)

        self.get_logger().info(
            f'[{self.name}] Returning {len(response.objects)} object(s)'
        )
        return response

    # GDino detection

    def _gdino_detect(self, image_rgb: np.ndarray, category: str):
        """Run Grounding-DINO on image_rgb for category text prompt.

        Returns a list of ((x1,y1,x2,y2), score) tuples, one per box that
        passes the confidence threshold (empty list if none do).
        """
        H, W = image_rgb.shape[:2]
        pil_img = PILImage.fromarray(image_rgb)
        text = category.rstrip('.') + '.'

        inputs = self._processor(images=pil_img, text=text, return_tensors='pt')
        inputs = {k: v.to(self._device) for k, v in inputs.items()}

        with torch.no_grad():
            outputs = self._gdino(**inputs)

        scores = outputs.logits[0].sigmoid().max(dim=-1).values.cpu().numpy()
        pred_boxes = outputs.pred_boxes[0].cpu().numpy()

        keep = scores >= self._box_thresh
        scores = scores[keep]
        pred_boxes = pred_boxes[keep]

        detections = []
        for score, (cx, cy, bw, bh) in zip(scores, pred_boxes):
            x1 = int((cx - bw / 2) * W)
            y1 = int((cy - bh / 2) * H)
            x2 = int((cx + bw / 2) * W)
            y2 = int((cy + bh / 2) * H)
            detections.append(((x1, y1, x2, y2), float(score)))
        return detections


def main(args=None):
    rclpy.init(args=args)
    node = ObjectSegmenter()
    rate = node.create_rate(5, node.get_clock())

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            rate.sleep()
    except Exception:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
