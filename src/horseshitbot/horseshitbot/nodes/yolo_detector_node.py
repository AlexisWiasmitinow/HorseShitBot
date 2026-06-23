"""
YOLO Detector Node — inference Ultralytics + publication d'obstacles virtuels
pour le local_costmap Nav2 via PointCloud2.

Abonne à :
  /camera/color/image_raw
  /camera/aligned_depth_to_color/image_raw
  /camera/aligned_depth_to_color/camera_info

Publie :
  /yolo/detections   (std_msgs/String, JSON)
  /yolo/obstacles    (sensor_msgs/PointCloud2) — injectable dans obstacle_layer
"""
from __future__ import annotations

import json
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import String

try:
    from ultralytics import YOLO
    _HAS_ULTRALYTICS = True
except ImportError:
    _HAS_ULTRALYTICS = False


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_detector_node")

        self.declare_parameter("model_path", "/home/user/models/best.pt")
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("publish_pointcloud", True)
        self.declare_parameter("depth_max_m", 3.0)
        self.declare_parameter("target_classes", [""])

        self._conf = (
            self.get_parameter("confidence_threshold")
            .get_parameter_value()
            .double_value
        )
        self._publish_pc = (
            self.get_parameter("publish_pointcloud").get_parameter_value().bool_value
        )
        self._depth_max = (
            self.get_parameter("depth_max_m").get_parameter_value().double_value
        )
        self._target_classes = [
            c
            for c in self.get_parameter("target_classes")
            .get_parameter_value()
            .string_array_value
            if c
        ]

        self._pub_detections = self.create_publisher(String, "/yolo/detections", 10)
        if self._publish_pc:
            self._pub_obstacles = self.create_publisher(
                PointCloud2, "/yolo/obstacles", 10
            )

        self._latest_color: Image | None = None
        self._latest_depth: Image | None = None
        self._latest_info: CameraInfo | None = None
        self._lock = threading.Lock()

        self.create_subscription(
            Image, "/camera/color/image_raw", self._cb_color, qos_profile=1
        )
        self.create_subscription(
            Image,
            "/camera/aligned_depth_to_color/image_raw",
            self._cb_depth,
            qos_profile=1,
        )
        self.create_subscription(
            CameraInfo,
            "/camera/aligned_depth_to_color/camera_info",
            self._cb_info,
            qos_profile=1,
        )

        if not _HAS_ULTRALYTICS:
            self.get_logger().error(
                "ultralytics n'est pas installé. YOLO node inactif. "
                "pip install ultralytics && télécharger le modèle best.pt"
            )
            return

        model_path = (
            self.get_parameter("model_path").get_parameter_value().string_value
        )
        self.get_logger().info(f"Chargement du modèle YOLO : {model_path}")
        self._model = YOLO(model_path)

        # Boucle d'inférence à ~5 Hz pour ne pas saturer le CPU/GPU Jetson
        self.create_timer(0.2, self._process)
        self.get_logger().info("YOLO detector node actif (5 Hz).")

    # ------------------------------------------------------------------ #
    # Callbacks (pas de traitement ici, on stocke pour le timer)
    # ------------------------------------------------------------------ #
    def _cb_color(self, msg: Image):
        with self._lock:
            self._latest_color = msg

    def _cb_depth(self, msg: Image):
        with self._lock:
            self._latest_depth = msg

    def _cb_info(self, msg: CameraInfo):
        with self._lock:
            self._latest_info = msg

    # ------------------------------------------------------------------ #
    # Helpers conversion
    # ------------------------------------------------------------------ #
    @staticmethod
    def _imgmsg_to_cv2(msg: Image):
        """Convertit sensor_msgs/Image (rgb8/bgr8) en ndarray (H, W, 3)."""
        if msg.encoding in ("rgb8", "bgr8"):
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3
            )
            return img[:, :, ::-1]  # RGB → BGR pour Ultralytics/OpenCV
        if msg.encoding == "mono8":
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width
            )
        return None

    @staticmethod
    def _build_pointcloud(header, points: list[tuple[float, float, float]]):
        if not points:
            return None
        arr = np.array(points, dtype=np.float32)
        pc = PointCloud2()
        pc.header = header
        pc.height = 1
        pc.width = len(points)
        pc.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pc.is_bigendian = False
        pc.point_step = 12
        pc.row_step = 12 * len(points)
        pc.is_dense = False
        pc.data = arr.tobytes()
        return pc

    @staticmethod
    def _pixel_to_3d(u: int, v: int, depth_m: float, K: list[float]):
        fx = K[0]
        fy = K[4]
        cx = K[2]
        cy = K[5]
        x = (u - cx) * depth_m / fx
        y = (v - cy) * depth_m / fy
        z = depth_m
        return x, y, z

    # ------------------------------------------------------------------ #
    # Boucle d'inférence
    # ------------------------------------------------------------------ #
    def _process(self):
        if not _HAS_ULTRALYTICS:
            return

        with self._lock:
            color_msg = self._latest_color
            depth_msg = self._latest_depth
            info_msg = self._latest_info
            self._latest_color = None

        if color_msg is None:
            return
        if self._publish_pc and (depth_msg is None or info_msg is None):
            self.get_logger().warn(
                "Depth ou CameraInfo manquant — pas de projection 3D",
                throttle_duration_sec=10,
            )

        img = self._imgmsg_to_cv2(color_msg)
        if img is None:
            self.get_logger().warning(
                f"Image encoding non supporté : {color_msg.encoding}"
            )
            return

        results = self._model(img, verbose=False)[0]

        detections: list[dict] = []
        points: list[tuple[float, float, float]] = []

        depth_img: np.ndarray | None = None
        if self._publish_pc and depth_msg is not None:
            depth_img = np.frombuffer(
                depth_msg.data, dtype=np.uint16
            ).reshape(depth_msg.height, depth_msg.width)

        K = info_msg.k if info_msg else []

        for box in results.boxes:
            conf = float(box.conf[0])
            if conf < self._conf:
                continue
            cls_id = int(box.cls[0])
            cls_name = results.names[cls_id]
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            det = {
                "class": cls_name,
                "confidence": round(conf, 3),
                "bbox": [x1, y1, x2, y2],
            }

            if self._publish_pc and depth_img is not None and K:
                if 0 <= cx < depth_img.shape[1] and 0 <= cy < depth_img.shape[0]:
                    depth_mm = float(depth_img[cy, cx])
                    if depth_mm > 0:
                        depth_m = depth_mm / 1000.0
                        if depth_m <= self._depth_max:
                            if (
                                not self._target_classes
                                or cls_name in self._target_classes
                            ):
                                x, y, z = self._pixel_to_3d(cx, cy, depth_m, K)
                                det["xyz"] = [round(v, 3) for v in (x, y, z)]
                                points.append((x, y, z))
            detections.append(det)

        payload = {"timestamp": time.time(), "detections": detections}
        self._pub_detections.publish(String(data=json.dumps(payload)))

        if self._publish_pc and self._pub_obstacles is not None:
            header = color_msg.header
            header.frame_id = (
                info_msg.header.frame_id
                if info_msg is not None
                else color_msg.header.frame_id
            )
            pc = self._build_pointcloud(header, points)
            if pc is not None:
                self._pub_obstacles.publish(pc)
                self.get_logger().debug(
                    f"Publié {len(points)} obstacles sur /yolo/obstacles"
                )


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
