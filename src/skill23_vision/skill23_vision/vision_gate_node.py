"""Orbbec color vision gate — idle until a mission camera block activates it."""

from __future__ import annotations

import threading
import time
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from skill23_vision.srv import FindColor, SetGate


# OpenCV HSV ranges (H: 0-179). Tunable for competition lighting.
COLOR_HSV: Dict[str, List[Tuple[Tuple[int, int, int], Tuple[int, int, int]]]] = {
    'red': [
        ((0, 80, 60), (10, 255, 255)),
        ((160, 80, 60), (179, 255, 255)),
    ],
    'orange': [((10, 80, 60), (25, 255, 255))],
    'yellow': [((25, 80, 60), (35, 255, 255))],
    'green': [((35, 60, 40), (85, 255, 255))],
    'blue': [((90, 60, 40), (130, 255, 255))],
    'purple': [((130, 50, 40), (160, 255, 255))],
    'white': [((0, 0, 180), (179, 40, 255))],
    'black': [((0, 0, 0), (179, 255, 50))],
}


class VisionGateNode(Node):
    """
    CPU-saving design:
      - IDLE: no camera subscription → no OpenCV
      - ACTIVE: subscribe, detect colors, optional JPEG preview for dashboard
    Orbbec may still run; this node simply ignores it until needed.
    """

    def __init__(self):
        super().__init__('adev_vision_gate')
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('min_pixel_ratio', 0.008)
        self.declare_parameter('preview_jpeg_quality', 50)
        self.declare_parameter('publish_preview', True)

        self._color_topic = self.get_parameter('color_topic').value
        self._min_ratio = float(self.get_parameter('min_pixel_ratio').value)
        self._jpeg_quality = int(self.get_parameter('preview_jpeg_quality').value)
        self._publish_preview = bool(self.get_parameter('publish_preview').value)

        self._bridge = CvBridge()
        self._active = False
        self._lock = threading.Lock()
        self._latest_bgr: Optional[np.ndarray] = None
        self._latest_stamp = 0.0
        self._image_sub = None

        self._status_pub = self.create_publisher(String, '/adev/vision/status', 10)
        self._preview_pub = self.create_publisher(
            CompressedImage, '/adev/vision/preview/compressed', 1)

        srv_group = ReentrantCallbackGroup()
        self._set_gate_srv = self.create_service(
            SetGate, '/adev/vision/set_gate', self._on_set_gate,
            callback_group=srv_group)
        self._find_srv = self.create_service(
            FindColor, '/adev/vision/find_color', self._on_find_color,
            callback_group=srv_group)

        self._status_timer = self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            'Vision gate IDLE — camera unsubscribed. '
            'Activate via /adev/vision/set_gate or find_color.')

    # ------------------------------------------------------------------ gate
    def _on_set_gate(self, request: SetGate.Request, response: SetGate.Response):
        if request.data:
            self._activate()
            response.success = True
            response.message = 'ACTIVE — subscribed to camera'
        else:
            self._deactivate()
            response.success = True
            response.message = 'IDLE — unsubscribed'
        return response

    def _activate(self):
        with self._lock:
            if self._active:
                return
            self._active = True
            self._image_sub = self.create_subscription(
                Image, self._color_topic, self._on_image,
                qos_profile_sensor_data)
        self.get_logger().info(f'Vision ACTIVE on {self._color_topic}')

    def _deactivate(self):
        with self._lock:
            self._active = False
            if self._image_sub is not None:
                self.destroy_subscription(self._image_sub)
                self._image_sub = None
            self._latest_bgr = None
        self.get_logger().info('Vision IDLE (unsubscribed)')

    def _on_image(self, msg: Image):
        # Fast path: if somehow still receiving while idle, drop immediately.
        if not self._active:
            return
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge failed: {exc}')
            return
        with self._lock:
            self._latest_bgr = bgr
            self._latest_stamp = time.time()
        if self._publish_preview:
            self._emit_preview(bgr)

    def _emit_preview(self, bgr: np.ndarray):
        try:
            small = cv2.resize(bgr, (320, 180), interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(
                '.jpg', small,
                [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality])
            if not ok:
                return
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_color_optical_frame'
            msg.format = 'jpeg'
            msg.data = buf.tobytes()
            self._preview_pub.publish(msg)
        except Exception:
            pass

    # -------------------------------------------------------------- detect
    def _detect_colors(
        self, bgr: np.ndarray, wanted: List[str]
    ) -> Tuple[Optional[str], float, int, int]:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        h, w = hsv.shape[:2]
        area = float(h * w)
        best_name = None
        best_ratio = 0.0
        best_uv = (0, 0)

        for name in wanted:
            key = name.strip().lower()
            ranges = COLOR_HSV.get(key)
            if not ranges:
                continue
            mask = np.zeros((h, w), dtype=np.uint8)
            for lo, hi in ranges:
                mask = cv2.bitwise_or(
                    mask, cv2.inRange(hsv, np.array(lo), np.array(hi)))
            # Light morphology to kill noise
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            ratio = float(cv2.countNonZero(mask)) / area
            if ratio > best_ratio:
                best_ratio = ratio
                best_name = key
                moments = cv2.moments(mask)
                if moments['m00'] > 0:
                    best_uv = (
                        int(moments['m10'] / moments['m00']),
                        int(moments['m01'] / moments['m00']),
                    )
        if best_name is None or best_ratio < self._min_ratio:
            return None, best_ratio, 0, 0
        return best_name, best_ratio, best_uv[0], best_uv[1]

    def _on_find_color(
        self, request: FindColor.Request, response: FindColor.Response
    ):
        colors = [c for c in request.colors if c.strip()]
        if not colors:
            response.found = False
            response.message = 'No colors requested'
            return response

        was_idle = not self._active
        self._activate()
        timeout = max(0.5, float(request.timeout_sec) if request.timeout_sec > 0 else 5.0)
        deadline = time.time() + timeout
        found_name = None
        conf = 0.0
        u = v = 0

        try:
            while time.time() < deadline and rclpy.ok():
                frame = None
                with self._lock:
                    if self._latest_bgr is not None:
                        frame = self._latest_bgr.copy()
                if frame is not None:
                    found_name, conf, u, v = self._detect_colors(frame, colors)
                    if found_name is not None:
                        break
                time.sleep(0.05)
        finally:
            if was_idle:
                self._deactivate()

        if found_name is not None:
            response.found = True
            response.color_name = found_name
            response.confidence = float(conf)
            response.u = int(u)
            response.v = int(v)
            response.message = f'Found {found_name} ({conf:.3f})'
            self.get_logger().info(response.message)
        else:
            response.found = False
            response.color_name = ''
            response.confidence = float(conf)
            response.message = (
                f'No match for {colors} within {timeout:.1f}s '
                f'(best_ratio={conf:.4f})')
            self.get_logger().info(response.message)
        return response

    def _publish_status(self):
        import json
        msg = String()
        with self._lock:
            age = (time.time() - self._latest_stamp) if self._latest_stamp else -1.0
            active = self._active
        msg.data = json.dumps({
            'active': active,
            'frame_age_sec': round(age, 2),
            'colors': list(COLOR_HSV.keys()),
        })
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionGateNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
