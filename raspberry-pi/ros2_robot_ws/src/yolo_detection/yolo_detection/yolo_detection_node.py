#!/usr/bin/env python3
"""
YOLO Object Detection Node for Precision Farming Robot

Subscribes to camera/raw topic and publishes:
1. Annotated image frames to /camera/detection
2. Detection results to detections/results

The YOLO model should be provided and placed in the package directory.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import cv2
import numpy as np
import json
import os
import glob


class YOLODetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')

        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('input_size', 640)
        self.declare_parameter('camera_topic', '/camera/color_jpeg')
        self.declare_parameter('compressed_camera_topic', '/camera/color_jpeg')
        self.declare_parameter('annotated_topic', '/camera/detection')
        self.declare_parameter('results_topic', '/detections/results')
        self.declare_parameter('legacy_results_topic', '/detections/results/legacy')
        self.declare_parameter('enable_visualization', True)
        self.declare_parameter('frame_timeout_sec', 5.0)

        # Get parameters
        configured_model_path = self.get_parameter('model_path').value
        model_path = self._resolve_model_path(configured_model_path)
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.input_size = self.get_parameter('input_size').value
        camera_topic = self.get_parameter('camera_topic').value
        compressed_camera_topic = self.get_parameter('compressed_camera_topic').value
        self.annotated_topic = self.get_parameter('annotated_topic').value
        if self.annotated_topic in ('/detections/annotated', '/camera/detectios'):
            self.annotated_topic = '/camera/detection'
        self.results_topic = self.get_parameter('results_topic').value
        self.enable_viz = self.get_parameter('enable_visualization').value
        self.frame_timeout_sec = float(self.get_parameter('frame_timeout_sec').value)

        self.cv_bridge = CvBridge()
        self.model = None
        self.class_names = []
        self.frame_count = 0
        self.last_frame_time = self.get_clock().now()

        # Initialize YOLO model
        if model_path and os.path.exists(model_path):
            try:
                self.load_yolo_model(model_path)
                self.get_logger().info(f'Successfully loaded YOLO model from: {model_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to load YOLO model: {e}')
                self.get_logger().warn('Node initialized but detection will be skipped')
        else:
            self.get_logger().warn(
                'Model path not set or does not exist. Please provide a valid YOLO model path '
                '(parameter `model_path` or env `YOLO_MODEL_PATH`). '
                f'Configured path: {configured_model_path}, resolved path: {model_path}'
            )

        # Setup QoS
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)

        # Create exactly one subscription for camera_topic to avoid incompatible
        # message-type conflicts on the same topic name.
        image_type = 'sensor_msgs/msg/Image'
        compressed_type = 'sensor_msgs/msg/CompressedImage'
        topic_types = {name: types for name, types in self.get_topic_names_and_types()}
        camera_topic_types = topic_types.get(camera_topic, [])
        prefer_compressed_input = (
            camera_topic == compressed_camera_topic
            or 'jpeg' in camera_topic
            or 'compressed' in camera_topic
        )
        use_compressed_input = (
            (compressed_type in camera_topic_types and image_type not in camera_topic_types)
            or (prefer_compressed_input and image_type not in camera_topic_types)
        )

        self.camera_image_sub = None
        self.camera_compressed_sub = None

        if use_compressed_input:
            self.camera_compressed_sub = self.create_subscription(
                CompressedImage,
                camera_topic,
                self.camera_compressed_callback,
                qos
            )
            selected_input_desc = f'{camera_topic} ({compressed_type})'
            self.annotated_is_compressed = True
        else:
            # Default to Image when type is Image, mixed, or not yet discoverable.
            self.camera_image_sub = self.create_subscription(
                Image,
                camera_topic,
                self.camera_image_callback,
                qos
            )
            selected_input_desc = f'{camera_topic} ({image_type})'
            self.annotated_is_compressed = False

        # Optional alternate compressed topic (e.g., /camera/color_jpeg).
        # Only subscribe when the primary input is Image.
        self.camera_compressed_jpeg_sub = None
        if self.camera_image_sub is not None and compressed_camera_topic and compressed_camera_topic != camera_topic:
            self.camera_compressed_jpeg_sub = self.create_subscription(
                CompressedImage,
                compressed_camera_topic,
                self.camera_compressed_callback,
                qos
            )

        # Create publisher for annotated images in the same transport as input
        if self.annotated_is_compressed:
            self.annotated_pub = self.create_publisher(
                CompressedImage,
                self.annotated_topic,
                1
            )
        else:
            self.annotated_pub = self.create_publisher(
                Image,
                self.annotated_topic,
                1
            )

        # Create publisher for detection results
        self.results_pub = self.create_publisher(
            String,
            self.results_topic,
            1
        )

        # Legacy topic compatibility (some clients expect /camera/detection)
        self.legacy_results_topic = self.get_parameter('legacy_results_topic').value
        self.legacy_results_pub = None
        if self.legacy_results_topic and self.legacy_results_topic != self.results_topic:
            self.legacy_results_pub = self.create_publisher(
                String,
                self.legacy_results_topic,
                1
            )

        legacy_info = f' and legacy results to: {self.legacy_results_topic}' if self.legacy_results_pub is not None else ''
        compressed_info = ''
        if self.camera_compressed_jpeg_sub is not None:
            compressed_info = f'\n  Also subscribed (compressed): {compressed_camera_topic}'
        self.get_logger().info(
            f'YOLO Detection Node initialized\n'
            f'  Subscribed to: {selected_input_desc}{compressed_info}\n'
            f'  Publishing annotated images to: {self.annotated_topic} '
            f'({"sensor_msgs/msg/CompressedImage" if self.annotated_is_compressed else "sensor_msgs/msg/Image"})\n'
            f'  Publishing detection results to: {self.results_topic}{legacy_info}\n'
            f'  Confidence threshold: {self.conf_threshold}\n'
            f'  IoU threshold: {self.iou_threshold}'
        )

        self.frame_watchdog_timer = self.create_timer(1.0, self._frame_watchdog_callback)

    def _resolve_model_path(self, configured_model_path):
        """Resolve the YOLO model path from parameter/env/common workspace locations."""
        if configured_model_path:
            expanded = os.path.expanduser(str(configured_model_path))
            if os.path.exists(expanded):
                return expanded

        env_model_path = os.environ.get('YOLO_MODEL_PATH', '').strip()
        if env_model_path:
            expanded = os.path.expanduser(env_model_path)
            if os.path.exists(expanded):
                self.get_logger().info(f'Using YOLO model from YOLO_MODEL_PATH: {expanded}')
                return expanded

        candidate_dirs = []

        cwd = os.getcwd()
        candidate_dirs.append(cwd)

        file_dir = os.path.dirname(os.path.abspath(__file__))
        candidate_dirs.append(file_dir)

        for base_dir in [cwd, file_dir]:
            probe_dir = base_dir
            for _ in range(8):
                parent = os.path.dirname(probe_dir)
                if parent == probe_dir:
                    break
                probe_dir = parent
                candidate_dirs.append(probe_dir)

        seen_dirs = set()
        ordered_dirs = []
        for directory in candidate_dirs:
            if directory not in seen_dirs:
                seen_dirs.add(directory)
                ordered_dirs.append(directory)

        explicit_names = ('yolo26n.pt', 'best.pt')
        for directory in ordered_dirs:
            for model_name in explicit_names:
                candidate = os.path.join(directory, model_name)
                if os.path.exists(candidate):
                    self.get_logger().info(f'Auto-resolved YOLO model: {candidate}')
                    return candidate

        for directory in ordered_dirs:
            wildcard_matches = sorted(glob.glob(os.path.join(directory, 'yolo*.pt')))
            if wildcard_matches:
                self.get_logger().info(f'Auto-resolved YOLO model: {wildcard_matches[0]}')
                return wildcard_matches[0]

        return os.path.expanduser(str(configured_model_path)) if configured_model_path else ''

    def _frame_watchdog_callback(self):
        elapsed = (self.get_clock().now() - self.last_frame_time).nanoseconds / 1e9
        if elapsed > self.frame_timeout_sec:
            self.get_logger().warn(
                f'No camera frames received for {elapsed:.1f}s. '
                f'Check camera topic type and QoS compatibility.'
            )
            self.last_frame_time = self.get_clock().now()

    def load_yolo_model(self, model_path):
        """
        Load YOLOv8 model using Ultralytics.
        """
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            # Get class names from the model
            self.class_names = self.model.names
            self.get_logger().info(f'Loaded YOLO model with {len(self.class_names)} classes')
        except ImportError:
            self.get_logger().error('ultralytics package not installed. Install with: pip install ultralytics')
            self.model = None

    def _get_coco_classes(self):
        """Get COCO dataset class names"""
        return [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck',
            'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench',
            'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe',
            'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis',
            'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
            'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
            'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
            'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
            'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
            'remote', 'keyboard', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator',
            'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

    def _decode_compressed_camera_msg(self, msg: CompressedImage):
        """Decode compressed camera messages into BGR OpenCV images."""
        format_value = (msg.format or '').strip().lower()
        raw_bytes = bytes(msg.data)

        if format_value.startswith('gray4;') or format_value.startswith('gray8;'):
            fmt, dims = format_value.split(';', 1)
            width_str, height_str = dims.split('x', 1)
            width = int(width_str)
            height = int(height_str)
            payload = np.frombuffer(raw_bytes, dtype=np.uint8)

            if fmt == 'gray4':
                hi = ((payload >> 4) & 0x0F).astype(np.uint8) * 17
                lo = (payload & 0x0F).astype(np.uint8) * 17
                gray = np.empty(payload.size * 2, dtype=np.uint8)
                gray[0::2] = hi
                gray[1::2] = lo
                gray = gray[:width * height]
            else:
                gray = payload[:width * height]

            gray = gray.reshape((height, width))
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        if format_value in ('jpeg', 'jpg', 'mjpeg'):
            image = cv2.imdecode(np.frombuffer(raw_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is not None:
                return image

        fallback_image = cv2.imdecode(np.frombuffer(raw_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
        if fallback_image is not None:
            return fallback_image

        raise ValueError(f'Unsupported compressed image format: {msg.format}')

    def _process_camera_frame(self, cv_image, header):
        """Run detection and publish outputs for one BGR frame."""
        try:
            self.frame_count += 1
            self.last_frame_time = self.get_clock().now()

            if self.model is not None:
                detections = self.run_inference(cv_image)
                annotated_image = self.draw_detections(cv_image, detections)

                if self.annotated_is_compressed:
                    ok, encoded = cv2.imencode('.jpg', annotated_image)
                    if not ok:
                        raise ValueError('Failed to JPEG-encode annotated frame')
                    annotated_msg = CompressedImage()
                    annotated_msg.header = header
                    annotated_msg.format = 'jpeg'
                    annotated_msg.data = encoded.tobytes()
                else:
                    annotated_msg = self.cv_bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
                    annotated_msg.header = header

                results_msg = String()
                results_msg.data = json.dumps(self.format_detections(detections))

                publish_actions = [
                    ('annotated image', self.annotated_pub, annotated_msg),
                    ('detection results', self.results_pub, results_msg),
                ]

                if self.legacy_results_pub is not None:
                    publish_actions.append(('legacy detection results', self.legacy_results_pub, results_msg))

                for label, publisher, message in publish_actions:
                    publisher.publish(message)
                    print(f'Published {label}', flush=True)

                if self.enable_viz:
                    self.get_logger().debug(f'Detected {len(detections)} objects')
            else:
                # If no model, just republish the original image
                if self.annotated_is_compressed:
                    ok, encoded = cv2.imencode('.jpg', cv_image)
                    if not ok:
                        raise ValueError('Failed to JPEG-encode passthrough frame')
                    annotated_msg = CompressedImage()
                    annotated_msg.header = header
                    annotated_msg.format = 'jpeg'
                    annotated_msg.data = encoded.tobytes()
                else:
                    annotated_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                    annotated_msg.header = header
                for label, publisher, message in [('annotated image', self.annotated_pub, annotated_msg)]:
                    publisher.publish(message)
                    print(f'Published {label}', flush=True)

        except Exception as e:
            self.get_logger().error(f'Error processing camera frame: {e}')

    def camera_image_callback(self, msg: Image):
        """Process incoming camera frame from sensor_msgs/Image."""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._process_camera_frame(cv_image, msg.header)
        except Exception as e:
            self.get_logger().error(f'Error processing image frame: {e}')

    def camera_compressed_callback(self, msg: CompressedImage):
        """Process incoming camera frame from sensor_msgs/CompressedImage."""
        try:
            cv_image = self._decode_compressed_camera_msg(msg)
            self._process_camera_frame(cv_image, msg.header)
        except Exception as e:
            self.get_logger().error(f'Error processing compressed frame: {e}')

    def run_inference(self, cv_image):
        """
        Run YOLOv8 inference on the image.

        Returns:
            List of detections, where each detection is a dict with:
            {
                'class': class_name,
                'class_id': int,
                'confidence': float,
                'bbox': [x1, y1, x2, y2]  # pixel coordinates
            }
        """
        detections = []

        try:
            # Run inference
            results = self.model(cv_image, conf=self.conf_threshold, iou=self.iou_threshold, verbose=False)

            # Process detections
            for result in results:
                if result.boxes is not None:
                    for box in result.boxes:
                        # Get bounding box coordinates
                        xyxy = box.xyxy[0].tolist()  # [x1, y1, x2, y2]

                        # Get class info
                        class_id = int(box.cls[0])
                        class_name = self.class_names[class_id] if class_id < len(self.class_names) else f'class_{class_id}'
                        confidence = float(box.conf[0])

                        detections.append({
                            'class': class_name,
                            'class_id': class_id,
                            'confidence': confidence,
                            'bbox': xyxy
                        })
        except Exception as e:
            self.get_logger().debug(f'Inference error: {e}')

        return detections

    def draw_detections(self, cv_image, detections):
        """Draw bounding boxes and labels on the image"""
        annotated = cv_image.copy()

        for detection in detections:
            bbox = detection['bbox']
            x1, y1, x2, y2 = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
            confidence = detection['confidence']
            class_name = detection['class']

            # Draw bounding box
            color = self._get_color_for_class(detection['class_id'])
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

            # Draw label
            label = f'{class_name} {confidence:.2f}'
            label_size, baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(
                annotated,
                (x1, y1 - label_size[1] - baseline),
                (x1 + label_size[0], y1),
                color,
                -1
            )
            cv2.putText(
                annotated,
                label,
                (x1, y1 - baseline),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1
            )

        return annotated

    def format_detections(self, detections):
        """Format detections for JSON output"""
        return {
            'detections': [
                {
                    'class': det['class'],
                    'class_id': det['class_id'],
                    'confidence': float(det['confidence']),
                    'bbox': [float(c) for c in det['bbox']]
                }
                for det in detections
            ],
            'timestamp': self.get_clock().now().to_msg().sec
        }

    def _get_color_for_class(self, class_id):
        """Get a consistent color for each class"""
        # Use class ID to generate color
        np.random.seed(class_id)
        color = tuple(np.random.randint(0, 255, 3).tolist())
        return color


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
