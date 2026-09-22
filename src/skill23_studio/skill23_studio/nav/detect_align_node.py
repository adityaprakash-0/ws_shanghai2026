#!/usr/bin/env python3
"""Detect+Align: lightweight snapshot YOLO + absolute OMS rotation + depth.

Idle until /studio/detect_align/cmd.

Flow (CPU-light — one frame per inference, streams off between steps):
  1. Toggle color ON, verify frames, grab ONE snapshot, YOLO, color OFF
  2. Gripper forward = image center (camera mounted above gripper, rotate together)
  3. Pinhole bearing from color intrinsics → OMS target (+ACW / −CW, ±90°)
  4. /oms/rotation/cmd goto {absolute_deg}; after move, 3–4 verify snapshots
  5. When confirmed centered → depth ON briefly → distance from camera → all OFF

Debug: /studio/detect_align/debug_image (bbox + target angle) for rqt_image_view.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time as RosTime
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Empty, Float64, String
from std_srvs.srv import SetBool

from skill23_studio.detect_align_paths import yolov8n_ncnn_dir
from skill23_studio.nav.node_status import NodeStatusPub
from skill23_studio.nav.detect_align_geometry import (
    CameraInfoCache,
    bbox_center_px,
    bearing_deg_from_pixel,
    compute_target_geometry,
    oms_goto_target_deg,
    pitch_rad_from_param,
    u_norm_from_pixel,
    yaw_direction,
)
from skill23_studio.nav.oms_drop_sequence import run_drop_sequence
from skill23_studio.nav.oms_pick_sequence import (
    depth_to_linear_mm,
    run_pick_post_depth,
    run_pick_pre_detect,
)
from skill23_studio.nav.wp_align_common import result_msg


def _studio_log_path(name: str) -> Path:
    for root in (
            Path('/home/vmx/skill23_ws/studio_maps/_logs'),
            Path(os.path.expanduser('~/skill23_ws/studio_maps/_logs'))):
        try:
            root.mkdir(parents=True, exist_ok=True)
            return root / name
        except OSError:
            continue
    return Path('/home/vmx/skill23_ws/studio_maps/_logs') / name


_DETECT_LOG = _studio_log_path('detect_align.log')


def _image_to_rgb(msg: Image) -> Optional[np.ndarray]:
    enc = (msg.encoding or '').lower()
    h, w = int(msg.height), int(msg.width)
    if h <= 0 or w <= 0:
        return None
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ('rgb8', 'rgb8; compressed'):
        return buf.reshape(h, w, 3).copy()
    if enc in ('bgr8',):
        img = buf.reshape(h, w, 3)
        return img[:, :, ::-1].copy()
    if enc in ('rgba8',):
        return buf.reshape(h, w, 4)[:, :, :3].copy()
    if enc in ('bgra8',):
        img = buf.reshape(h, w, 4)
        return img[:, :, 2::-1].copy()
    if enc in ('mono8',):
        gray = buf.reshape(h, w)
        return np.repeat(gray[:, :, None], 3, axis=2)
    if buf.size == h * w * 3:
        return buf.reshape(h, w, 3).copy()
    return None


def _rgb_to_bgr_msg(rgb: np.ndarray, stamp: RosTime, frame_id: str = 'camera') -> Image:
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = bgr.shape[:2]
    msg.encoding = 'bgr8'
    msg.step = int(msg.width * 3)
    msg.data = bgr.tobytes()
    return msg


def _depth_to_meters(msg: Image) -> Optional[np.ndarray]:
    enc = (msg.encoding or '').lower()
    h, w = int(msg.height), int(msg.width)
    if h <= 0 or w <= 0:
        return None
    if enc in ('16uc1', 'mono16'):
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
        return raw.astype(np.float32) / 1000.0
    if enc in ('32fc1',):
        return np.frombuffer(msg.data, dtype=np.float32).reshape(h, w).copy()
    if '16' in enc:
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
        return raw.astype(np.float32) / 1000.0
    return None


def _sample_depth(
        depth_m: np.ndarray, u_norm: float, v_norm: float,
        roi_frac: float) -> Optional[float]:
    h, w = depth_m.shape[:2]
    cx = int(round(max(0.0, min(1.0, u_norm)) * (w - 1)))
    cy = int(round(max(0.0, min(1.0, v_norm)) * (h - 1)))
    half = max(1, int(round(min(h, w) * max(0.02, min(0.25, roi_frac)) * 0.5)))
    x1, x2 = max(0, cx - half), min(w, cx + half + 1)
    y1, y2 = max(0, cy - half), min(h, cy + half + 1)
    patch = depth_m[y1:y2, x1:x2]
    valid = patch[np.isfinite(patch) & (patch > 0.05) & (patch < 8.0)]
    if valid.size == 0:
        return None
    return float(np.median(valid))


class DetectAlignNode(Node):
    def __init__(self) -> None:
        super().__init__('detect_align')
        self.declare_parameter('cmd_topic', '/studio/detect_align/cmd')
        self.declare_parameter('result_topic', '/studio/detect_align/result')
        self.declare_parameter('cancel_topic', '/studio/detect_align/cancel')
        self.declare_parameter('status_topic', '/studio/detect_align/status')
        self.declare_parameter('debug_image_topic', '/studio/detect_align/debug_image')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('color_camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('depth_camera_info_topic', '/camera/depth/camera_info')
        self.declare_parameter('model_dir', '')
        self.declare_parameter('input_size', 320)
        self.declare_parameter('conf_threshold', 0.35)
        self.declare_parameter('nms_iou', 0.45)
        self.declare_parameter('num_threads', 1)
        self.declare_parameter('target_class', -1)
        self.declare_parameter('target_name', '')
        self.declare_parameter('min_box_area_frac', 0.002)
        self.declare_parameter('center_tol_norm', 0.05)
        self.declare_parameter('center_u_offset', 0.0)
        self.declare_parameter('confirm_frames', 3)
        self.declare_parameter('detect_wait_sec', 8.0)
        self.declare_parameter('align_timeout_sec', 30.0)
        self.declare_parameter('depth_after_align_sec', 0.8)
        self.declare_parameter('depth_samples', 4)
        self.declare_parameter('depth_sample_timeout_sec', 5.0)
        self.declare_parameter('depth_roi_frac', 0.08)
        self.declare_parameter('camera_tilt_deg', 35.0)
        self.declare_parameter('camera_tilt_from_vertical', False)
        self.declare_parameter('hold_after_done_sec', 0.5)
        self.declare_parameter('retries', 1)
        self.declare_parameter('retry_pause_sec', 0.3)
        self.declare_parameter('angular_sign', 1.0)
        self.declare_parameter('log_hz', 2.0)
        self.declare_parameter('skip_frames', 3)
        self.declare_parameter('verify_skip_frames', 3)
        self.declare_parameter('hfov_deg', 69.0)
        self.declare_parameter('gripper_u_norm', 0.0)
        self.declare_parameter('oms_min_deg', -90.0)
        self.declare_parameter('oms_max_deg', 90.0)
        self.declare_parameter('oms_cmd_topic', '/oms/rotation/cmd')
        self.declare_parameter('oms_result_topic', '/oms/rotation/result')
        self.declare_parameter('oms_status_topic', '/oms/rotation/status')
        self.declare_parameter('oms_timeout_sec', 25.0)
        self.declare_parameter('max_oms_moves', 6)
        self.declare_parameter('oms_rotation_enabled', False)
        self.declare_parameter('stream_verify_frames', 3)
        self.declare_parameter('stream_verify_timeout_sec', 12.0)
        # Commissioning oms_pick / oms_drop
        self.declare_parameter('pick_lift_mm', 0.0)
        self.declare_parameter('camera_to_gripper_mm', 350.0)
        self.declare_parameter('linear_pre_extend_mm', 100.0)
        self.declare_parameter('linear_fwd_offset_mm', 0.0)
        self.declare_parameter('linear_max_mm', 150.0)
        self.declare_parameter('oms_align_tol_deg', 1.0)
        self.declare_parameter('align_yaw_tol_deg', 2.0)
        self.declare_parameter('align_refine_max', 2)
        self.declare_parameter('need_frames', 3)
        self.declare_parameter('linear_cmd_enabled', True)
        self.declare_parameter('lift_timeout_sec', 40.0)
        self.declare_parameter('linear_timeout_sec', 45.0)
        self.declare_parameter('servo_timeout_sec', 12.0)
        self.declare_parameter('oms_home_timeout_sec', 180.0)

        self._center_tol = abs(float(self.get_parameter('center_tol_norm').value))
        self._u_off = float(self.get_parameter('center_u_offset').value)
        self._confirm_need = max(1, int(self.get_parameter('confirm_frames').value))
        self._detect_wait = max(0.5, float(self.get_parameter('detect_wait_sec').value))
        self._align_timeout = max(2.0, float(self.get_parameter('align_timeout_sec').value))
        self._depth_wait = max(0.0, float(self.get_parameter('depth_after_align_sec').value))
        self._depth_samples = max(1, int(self.get_parameter('depth_samples').value))
        self._depth_timeout = max(0.5, float(self.get_parameter('depth_sample_timeout_sec').value))
        self._depth_roi = float(self.get_parameter('depth_roi_frac').value)
        self._tilt = pitch_rad_from_param(
            float(self.get_parameter('camera_tilt_deg').value),
            bool(self.get_parameter('camera_tilt_from_vertical').value))
        self._hold = max(0.0, float(self.get_parameter('hold_after_done_sec').value))
        self._retries = max(0, min(5, int(self.get_parameter('retries').value)))
        self._retry_pause = max(0.0, float(self.get_parameter('retry_pause_sec').value))
        asig = float(self.get_parameter('angular_sign').value)
        self._asign = 1.0 if asig >= 0.0 else -1.0
        self._log_period = 1.0 / max(0.5, float(self.get_parameter('log_hz').value))
        self._skip_frames = max(0, int(self.get_parameter('skip_frames').value))
        self._verify_skip = max(0, int(self.get_parameter('verify_skip_frames').value))
        self._hfov = max(10.0, float(self.get_parameter('hfov_deg').value))
        self._gripper_u = float(self.get_parameter('gripper_u_norm').value)
        self._oms_min = float(self.get_parameter('oms_min_deg').value)
        self._oms_max = float(self.get_parameter('oms_max_deg').value)
        self._oms_timeout = max(4.0, float(self.get_parameter('oms_timeout_sec').value))
        self._max_oms_moves = max(1, min(10, int(self.get_parameter('max_oms_moves').value)))
        self._oms_rotation_enabled = bool(
            self.get_parameter('oms_rotation_enabled').value)
        self._stream_need = max(1, int(self.get_parameter('stream_verify_frames').value))
        self._stream_timeout = max(2.0, float(self.get_parameter('stream_verify_timeout_sec').value))
        self._pick_lift_mm = max(0.0, float(self.get_parameter('pick_lift_mm').value))
        self._cam_grip_mm = max(0.0, float(self.get_parameter('camera_to_gripper_mm').value))
        self._pre_extend_mm = max(0.0, float(self.get_parameter('linear_pre_extend_mm').value))
        self._linear_off_mm = float(self.get_parameter('linear_fwd_offset_mm').value)
        self._linear_max_mm = max(1.0, float(self.get_parameter('linear_max_mm').value))
        self._oms_align_tol = max(0.1, float(self.get_parameter('oms_align_tol_deg').value))
        self._align_yaw_tol = max(0.1, float(self.get_parameter('align_yaw_tol_deg').value))
        self._align_refine_max = max(0, int(self.get_parameter('align_refine_max').value))
        self._need_frames = max(1, int(self.get_parameter('need_frames').value))
        self._linear_cmd_enabled = bool(self.get_parameter('linear_cmd_enabled').value)
        self._lift_timeout = max(4.0, float(self.get_parameter('lift_timeout_sec').value))
        self._linear_timeout = max(4.0, float(self.get_parameter('linear_timeout_sec').value))
        self._servo_timeout = max(2.0, float(self.get_parameter('servo_timeout_sec').value))
        self._oms_home_timeout = max(10.0, float(self.get_parameter('oms_home_timeout_sec').value))
        self._target_class = int(self.get_parameter('target_class').value)
        self._target_name = str(self.get_parameter('target_name').value or '').strip()
        self._min_area_frac = max(0.0, float(self.get_parameter('min_box_area_frac').value))
        self._publish_debug = bool(self.get_parameter('publish_debug_image').value)
        model_dir = str(self.get_parameter('model_dir').value or '').strip()
        self._model_dir = model_dir or yolov8n_ncnn_dir()
        self._debug_topic = str(self.get_parameter('debug_image_topic').value)

        self._lock = threading.Lock()
        self._busy = False
        self._cancel = threading.Event()
        self._job_gen = 0
        self._detector = None
        self._color: Optional[Image] = None
        self._color_stamp = 0.0
        self._color_n = 0
        self._color_topic = str(self.get_parameter('color_topic').value)
        self._depth_topic = str(self.get_parameter('depth_topic').value)
        self._cam_info = CameraInfoCache(self._hfov)
        self._log_path = _DETECT_LOG
        self._log_write_failed = False
        self._depth: Optional[Image] = None
        self._depth_stamp = 0.0
        self._oms_angle: Optional[float] = None
        self._oms_tracked_deg = 0.0
        self._oms_result: Optional[dict] = None
        self._oms_homed = False
        self._axis_results: Dict[str, Any] = {}
        self._home_result: Optional[dict] = None
        self._status_payload = {
            'state': 'idle',
            'message': 'waiting for Detect+Align',
            'aligned': False,
            'u_norm': None,
            'bbox': None,
            'conf': None,
            'class_name': '',
            'depth_m': None,
            'confirm': 0,
            'confirm_need': self._confirm_need,
            'oms_angle_deg': None,
            'target_oms_deg': None,
        }

        self._result_pub = self.create_publisher(
            String, str(self.get_parameter('result_topic').value), 10)
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10)
        if self._publish_debug:
            self._debug_pub = self.create_publisher(Image, self._debug_topic, 10)
        else:
            self._debug_pub = None
        self._cam_need = self.create_publisher(Bool, '/studio/camera/action_need', 10)
        self._cam_depth = self.create_publisher(Bool, '/studio/camera/need_depth', 10)
        self._toggle_color_clis = [
            self.create_client(SetBool, name) for name in (
                '/camera/toggle_color',
                '/camera/camera/toggle_color',
                '/toggle_color',
            )
        ]
        self._toggle_depth_clis = [
            self.create_client(SetBool, name) for name in (
                '/camera/toggle_depth',
                '/camera/camera/toggle_depth',
                '/toggle_depth',
            )
        ]
        color_topics = []
        for topic in (
                self._color_topic,
                '/camera/color/image_raw',
                '/camera/camera/color/image_raw',
                '/color/image_raw'):
            if topic and topic not in color_topics:
                color_topics.append(topic)
        self._color_topics = color_topics
        reliable = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        for topic in color_topics:
            cb = lambda msg, t=topic: self._on_color(msg, t)
            self.create_subscription(Image, topic, cb, qos_profile_sensor_data)
            self.create_subscription(Image, topic, cb, reliable)
        depth_topics = []
        for topic in (
                self._depth_topic,
                '/camera/depth/image_raw',
                '/camera/camera/depth/image_raw',
                '/depth/image_raw'):
            if topic and topic not in depth_topics:
                depth_topics.append(topic)
        for topic in depth_topics:
            self.create_subscription(
                Image, topic, self._on_depth, qos_profile_sensor_data)
            self.create_subscription(Image, topic, self._on_depth, reliable)
        color_info_topic = str(self.get_parameter('color_camera_info_topic').value)
        depth_info_topic = str(self.get_parameter('depth_camera_info_topic').value)
        if color_info_topic:
            self.create_subscription(
                CameraInfo, color_info_topic, self._on_color_info, qos_profile_sensor_data)
        if depth_info_topic:
            self.create_subscription(
                CameraInfo, depth_info_topic, self._on_depth_info, qos_profile_sensor_data)
        self.create_subscription(
            String, str(self.get_parameter('cmd_topic').value), self._on_cmd, 10)
        self.create_subscription(
            Empty, str(self.get_parameter('cancel_topic').value),
            self._on_cancel, 10)
        self._oms_cmd_pub = self.create_publisher(
            String, str(self.get_parameter('oms_cmd_topic').value), 10)
        self.create_subscription(
            String, str(self.get_parameter('oms_result_topic').value),
            self._on_oms_result, 10)
        self.create_subscription(
            String, str(self.get_parameter('oms_status_topic').value),
            self._on_oms_status, 10)
        self.create_subscription(
            Float64, '/oms/rotation/angle_deg', self._on_oms_angle, 10)

        self._lift_cmd_pub = self.create_publisher(String, '/oms/lift/cmd', 10)
        self._linear_cmd_pub = self.create_publisher(String, '/oms/linear/cmd', 10)
        self._rot_servo_pub = self.create_publisher(String, '/oms/rotation_servo/cmd', 10)
        self._vert_pub = self.create_publisher(String, '/oms/vertical/cmd', 10)
        self._grip_pub = self.create_publisher(String, '/oms/gripper/cmd', 10)
        self._home_cmd_pub = self.create_publisher(String, '/oms/home/cmd', 10)
        for topic, key in (
                ('/oms/lift/result', 'lift'),
                ('/oms/linear/result', 'linear'),
                ('/oms/rotation_servo/result', 'rotation_servo'),
                ('/oms/vertical/result', 'vertical'),
                ('/oms/gripper/result', 'gripper'),
        ):
            self.create_subscription(
                String, topic,
                lambda msg, k=key: self._on_axis_result(k, msg), 10)
        self.create_subscription(
            String, '/oms/home/result', self._on_home_result, 10)

        self.status = NodeStatusPub(self, 'detect_align')
        self.status.set('idle', 'waiting for Detect+Align (YOLO off)')
        self._publish_status()
        self.get_logger().info(
            f'detect_align idle  model={self._model_dir}  '
            f'tilt={math.degrees(self._tilt):.0f}°  hfov_fallback={self._hfov:.0f}°  '
            f'confirm={self._confirm_need}  OMS abs ±90°  '
            f'debug={self._debug_topic if self._publish_debug else "off"}  '
            f'log={self._log_path}')
        self._flog(
            'info',
            f'==== detect_align start {time.strftime("%Y-%m-%d %H:%M:%S")} '
            f'snapshot-only YOLO  absolute OMS  no vertical move')

    def _flog(self, level: str, msg: str) -> None:
        line = f'{time.strftime("%H:%M:%S")} [{level}] {msg}'
        log = self.get_logger()
        if level == 'error':
            log.error(f'[detect_align] {msg}')
        elif level == 'warn':
            log.warn(f'[detect_align] {msg}')
        else:
            log.info(f'[detect_align] {msg}')
        try:
            self._log_path.parent.mkdir(parents=True, exist_ok=True)
            with open(self._log_path, 'a', encoding='utf-8') as handle:
                handle.write(line + '\n')
                handle.flush()
        except Exception as exc:
            if not self._log_write_failed:
                self._log_write_failed = True
                log.error(f'[detect_align] cannot write {self._log_path}: {exc}')

    def _cam_graph_note(self) -> str:
        try:
            nodes = [n for n in self.get_node_names() if 'camera' in n.lower()]
        except Exception:
            nodes = ['?']
        return f'camera_nodes={nodes or ["(none)"]} color_n={self._color_n}'

    def _on_color(self, msg: Image, topic: str = '') -> None:
        first = self._color_n == 0
        self._color = msg
        self._color_stamp = time.time()
        self._color_n += 1
        if topic:
            self._color_topic = topic
        if first:
            self._flog(
                'info',
                f'first color {int(msg.width)}x{int(msg.height)} '
                f'topic={self._color_topic}')

    def _on_depth(self, msg: Image) -> None:
        self._depth = msg
        self._depth_stamp = time.time()

    def _on_color_info(self, msg: CameraInfo) -> None:
        self._cam_info.set_color_info(msg)

    def _on_depth_info(self, msg: CameraInfo) -> None:
        self._cam_info.set_depth_info(msg)

    def _on_cancel(self, _msg: Empty) -> None:
        self._cancel.set()

    def _set_cam(self, color: bool, depth: bool = False) -> None:
        try:
            self._cam_need.publish(Bool(data=bool(color)))
            self._cam_depth.publish(Bool(data=bool(depth)))
        except Exception:
            pass
        if color:
            self._call_toggle('color', self._toggle_color_clis, True)
        else:
            self._call_toggle('color', self._toggle_color_clis, False)
            if not depth:
                self._call_toggle('depth', self._toggle_depth_clis, False)
        if depth:
            self._call_toggle('depth', self._toggle_depth_clis, True)

    def _call_toggle(self, kind: str, clients, wanted: bool, wait_s: float = 8.0) -> bool:
        req = SetBool.Request()
        req.data = bool(wanted)
        seen = set()
        for cli in list(clients):
            name = cli.srv_name
            if name in seen:
                continue
            seen.add(name)
            if not cli.service_is_ready() and not cli.wait_for_service(timeout_sec=2.0):
                continue
            fut = cli.call_async(req)
            t0 = time.time()
            while rclpy.ok() and not fut.done() and (time.time() - t0) < wait_s:
                time.sleep(0.05)
            if not fut.done():
                continue
            try:
                res = fut.result()
            except Exception:
                continue
            msg = str(getattr(res, 'message', '') or '')
            ok = bool(getattr(res, 'success', False)) or (
                ('already on' in msg.lower()) if wanted else ('already off' in msg.lower()))
            self._flog(
                'info' if ok else 'warn',
                f'toggle_{kind} {"ON" if wanted else "OFF"} ok={ok} {msg}'.strip())
            return ok
        return False

    def _on_oms_angle(self, msg: Float64) -> None:
        ang = float(msg.data)
        self._oms_angle = ang
        self._oms_tracked_deg = ang

    def _on_oms_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if isinstance(data, dict):
            if data.get('angle_deg') is not None:
                try:
                    ang = float(data['angle_deg'])
                    self._oms_angle = ang
                    self._oms_tracked_deg = ang
                except (TypeError, ValueError):
                    pass
            self._oms_homed = bool(data.get('homed'))

    def _on_oms_result(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if isinstance(data, dict):
            self._oms_result = data

    def _on_axis_result(self, name: str, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if isinstance(data, dict):
            self._axis_results[name] = data

    def _on_home_result(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if isinstance(data, dict):
            self._home_result = data

    def _oms_goto_tol(self, angle_deg: float, gen: int) -> Tuple[bool, str]:
        """Goto with retries; success only when tracked within oms_align_tol."""
        target = max(self._oms_min, min(self._oms_max, float(angle_deg)))
        last_msg = ''
        for attempt in range(3):
            if self._cancel.is_set() or gen != self._job_gen:
                return False, 'canceled'
            cmd_id = f'daoms_{int(time.time() * 1000)}'
            self._oms_result = None
            payload = {'cmd_id': cmd_id, 'mode': 'goto', 'angle_deg': round(target, 3)}
            self._oms_cmd_pub.publish(String(data=json.dumps(payload)))
            self._flog(
                'info',
                f'OMS goto {target:+.2f}° try={attempt + 1}/3 '
                f'(now {self._oms_now():+.1f}°)')
            deadline = time.time() + self._oms_timeout
            ok = False
            msg = 'timeout'
            while time.time() < deadline:
                if self._cancel.is_set() or gen != self._job_gen:
                    return False, 'canceled'
                res = self._oms_result
                if isinstance(res, dict) and str(res.get('cmd_id') or '') == cmd_id:
                    ok = bool(res.get('success'))
                    msg = str(res.get('message') or '')
                    if ok:
                        self._commit_oms_position(target, res)
                    break
                time.sleep(0.05)
            tracked = self._oms_now()
            err = abs(tracked - target)
            if ok and msg == 'ok' and err <= self._oms_align_tol:
                self._flog(
                    'info',
                    f'OMS ok tracked={tracked:+.1f}° err={err:.2f}°')
                return True, msg
            last_msg = msg or 'timeout'
            self._flog(
                'warn',
                f'OMS miss try={attempt + 1} ok={ok} msg={last_msg} '
                f'err={err:.1f}° (tol {self._oms_align_tol:.1f}°)')
            time.sleep(0.2)
        return False, last_msg

    def _wait_oms_home(self, timeout: float) -> Tuple[bool, str]:
        self._home_result = None
        deadline = time.time() + timeout
        last_pub = 0.0
        while time.time() < deadline:
            now = time.time()
            if now - last_pub >= 2.0:
                self._home_cmd_pub.publish(
                    String(data=json.dumps({'mode': 'home'})))
                last_pub = now
            res = self._home_result
            if isinstance(res, dict) and res.get('message'):
                ok = bool(res.get('success'))
                msg = str(res.get('message') or '')
                if ok or 'fail' in msg.lower():
                    return ok, msg
            time.sleep(0.1)
        return False, 'OMS home timeout'

    def _oms_now(self) -> float:
        return float(self._oms_tracked_deg)

    def _commit_oms_position(self, target_deg: float, result: Optional[dict] = None) -> None:
        if result is not None and result.get('angle_deg') is not None:
            try:
                ang = float(result['angle_deg'])
                self._oms_angle = ang
                self._oms_tracked_deg = ang
                return
            except (TypeError, ValueError):
                pass
        ang = float(target_deg)
        self._oms_angle = ang
        self._oms_tracked_deg = ang

    def _u_norm_of(self, picked, size: Tuple[int, int]) -> float:
        """Object center vs image center. +right, −left."""
        w = int(size[0])
        u_px, _v = bbox_center_px(picked)
        return u_norm_from_pixel(u_px, w, self._u_off)

    def _oms_target_deg(
            self, u_px: float, oms_now: float,
            color_intr) -> float:
        """Absolute OMS goto = current tracked angle + relative bearing."""
        bearing = bearing_deg_from_pixel(
            u_px, color_intr, self._asign, self._oms_min, self._oms_max)
        return oms_goto_target_deg(oms_now, bearing, self._oms_min, self._oms_max)

    def _wait_stream_frames(self, gen: int, need: int, timeout: float) -> bool:
        start = self._color_n
        deadline = time.time() + timeout
        while rclpy.ok() and not self._cancel.is_set() and gen == self._job_gen:
            if self._color_n - start >= need:
                self._flog('info', f'stream verified {self._color_n - start}/{need} frames')
                return True
            if time.time() >= deadline:
                break
            time.sleep(0.05)
        self._flog(
            'warn',
            f'stream verify failed got={self._color_n - start}/{need} '
            f'{self._cam_graph_note()}')
        return False

    def _grab_snapshot(self, gen: int, skip: int, timeout: float) -> Optional[Image]:
        skip = max(0, int(skip))
        start_n = self._color_n
        need = start_n + skip + 1
        deadline = time.time() + max(2.0, timeout)
        while rclpy.ok() and not self._cancel.is_set() and gen == self._job_gen:
            if self._color is not None and self._color_n >= need:
                snap = self._color
                self._flog(
                    'info',
                    f'snapshot taken skip={skip} total_frames={self._color_n}')
                return snap
            if time.time() >= deadline:
                return None
            time.sleep(0.05)
        return None

    def _oms_goto(self, angle_deg: float, gen: int) -> Tuple[bool, str]:
        cmd_id = f'da_{int(time.time() * 1000)}'
        self._oms_result = None
        payload = {'cmd_id': cmd_id, 'mode': 'goto', 'angle_deg': float(angle_deg)}
        self._oms_cmd_pub.publish(String(data=json.dumps(payload)))
        self._flog(
            'info',
            f'OMS goto ABS {angle_deg:+.2f}° '
            f'(tracked now {self._oms_now():+.1f}°) id={cmd_id}')
        deadline = time.time() + self._oms_timeout
        while rclpy.ok() and not self._cancel.is_set() and gen == self._job_gen:
            res = self._oms_result
            if res and str(res.get('cmd_id') or '') == cmd_id:
                ok = bool(res.get('success'))
                msg = str(res.get('message') or '')
                if ok:
                    self._commit_oms_position(angle_deg, res)
                self._flog(
                    'info',
                    f'OMS result ok={ok} tracked={self._oms_now():+.1f}° {msg}')
                return ok, msg
            if time.time() >= deadline:
                return False, 'OMS rotation timeout'
            time.sleep(0.05)
        return False, 'canceled'

    def _publish_debug_image(
            self, rgb: np.ndarray, stamp: RosTime, obj, u_err: float,
            oms_now: float, target_deg: float, label: str,
            floor=None) -> None:
        if self._debug_pub is None:
            return
        img = rgb.copy()
        h, w = img.shape[:2]
        cx = w // 2
        cv2.line(img, (cx, 0), (cx, h - 1), (0, 255, 255), 1)
        if obj is not None:
            x1, y1, x2, y2 = int(obj.x1), int(obj.y1), int(obj.x2), int(obj.y2)
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            oc = int(0.5 * (obj.x1 + obj.x2))
            oc_y = int(0.5 * (obj.y1 + obj.y2))
            cv2.circle(img, (oc, oc_y), 4, (255, 0, 0), -1)
            cv2.line(img, (cx, oc_y), (oc, oc_y), (255, 255, 0), 1)
        lines = [
            label,
            f'u_err={u_err:+.3f}  tol={self._center_tol:.3f}',
            f'oms={oms_now:+.1f}  target={target_deg:+.1f}',
            f'tilt={math.degrees(self._tilt):.0f}',
        ]
        if floor is not None:
            lat_side = 'center'
            if floor.lateral_m > 0.005:
                lat_side = 'L'
            elif floor.lateral_m < -0.005:
                lat_side = 'R'
            lines.append(
                f'Z={floor.z_depth_m:.3f}m  fwd={floor.forward_m:.3f}m  '
                f'lat={floor.lateral_m:+.3f}m ({lat_side})')
            lines.append(f'yaw={floor.bearing_deg:+.1f} deg (turn to center)')
        for i, text in enumerate(lines):
            cv2.putText(
                img, text, (8, 22 + i * 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(
                img, text, (8, 22 + i * 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 128, 255), 1, cv2.LINE_AA)
        try:
            self._debug_pub.publish(_rgb_to_bgr_msg(img, stamp))
        except Exception as exc:
            self._flog('warn', f'debug image publish failed: {exc}')

    def _publish_status(self, **extra) -> None:
        payload = dict(self._status_payload)
        payload.update(extra)
        if extra.get('oms_angle_deg') is None and self._oms_angle is not None:
            payload['oms_angle_deg'] = round(float(self._oms_angle), 2)
        payload['ts'] = time.time()
        self._status_payload = payload
        try:
            self._status_pub.publish(String(data=json.dumps(payload)))
        except Exception:
            pass
        state = str(payload.get('state') or 'idle')
        msg = str(payload.get('message') or '')
        self.status.set(state, msg, log=False)

    def _on_cmd(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except Exception as exc:
            self._result_pub.publish(result_msg('', False, 'detect_align', str(exc)))
            return
        self._flog('info', f'cmd id={cmd.get("cmd_id", "")} {cmd}')
        self._job_gen += 1
        gen = self._job_gen
        self._cancel.set()
        threading.Thread(target=self._run, args=(cmd, gen), daemon=True).start()

    def _timeout_of(self, cmd: dict) -> float:
        try:
            raw = float(cmd.get('timeout_sec', 0.0) or 0.0)
        except (TypeError, ValueError):
            raw = 0.0
        timeout = raw if raw > 0.0 and math.isfinite(raw) else self._align_timeout
        return min(90.0, max(2.0, timeout))

    def _sleep_or_cancel(self, seconds: float, gen: int) -> bool:
        until = time.time() + max(0.0, seconds)
        while rclpy.ok() and not self._cancel.is_set() and gen == self._job_gen:
            if time.time() >= until:
                return True
            time.sleep(0.05)
        return False

    def _ensure_detector(self):
        if self._detector is not None:
            return self._detector
        from skill23_studio.nav.yolov8_ncnn import YoloV8Ncnn
        self._flog('info', f'loading NCNN {self._model_dir}')
        self._detector = YoloV8Ncnn(
            self._model_dir,
            input_size=int(self.get_parameter('input_size').value),
            conf_threshold=float(self.get_parameter('conf_threshold').value),
            nms_iou=float(self.get_parameter('nms_iou').value),
            num_threads=int(self.get_parameter('num_threads').value),
        )
        return self._detector

    def _drop_detector(self) -> None:
        det = self._detector
        self._detector = None
        if det is None:
            return
        try:
            det.close()
        except Exception:
            pass

    def _detect_on_frame(self, msg: Image):
        rgb = _image_to_rgb(msg)
        if rgb is None:
            return None, None, None, None, f'bad encoding {msg.encoding}'
        dets = self._detector.detect(rgb)
        h, w = rgb.shape[:2]
        min_area = self._min_area_frac * float(w * h)
        obj = self._detector.pick(
            dets,
            class_id=self._target_class,
            class_name=self._target_name,
            min_area=min_area,
        )
        grip_u = self._gripper_u
        if obj is None:
            return None, (w, h), grip_u, rgb, 'no target'
        return obj, (w, h), grip_u, rgb, ''

    def _one_snapshot_detect(
            self, gen: int, skip: int, wait_s: float
    ) -> Tuple[Optional[Image], Optional[object], Optional[tuple], Optional[np.ndarray], str]:
        """Color ON → wait frames → one snapshot → color OFF → YOLO."""
        self._set_cam(True, False)
        if not self._wait_stream_frames(gen, 1, min(wait_s, self._stream_timeout)):
            self._set_cam(False, False)
            return None, None, None, None, 'no color frames'
        snap = self._grab_snapshot(gen, skip, wait_s)
        self._set_cam(False, False)
        if snap is None:
            return None, None, None, None, 'snapshot timeout'
        obj, size, grip_u, rgb, note = self._detect_on_frame(snap)
        return snap, obj, size, rgb, note

    def _one_attempt(self, timeout: float, gen: int, attempt: int, attempts: int):
        tag = f'{attempt}/{attempts}'
        t0 = time.time()
        confirm = 0
        moves = 0
        extra = None
        first_pass = True
        self._flog(
            'info',
            f'{tag} begin  oms={self._oms_now():+.1f}° homed={self._oms_homed}')
        self._publish_status(
            state='align', message=f'{tag} snapshot OMS align', confirm=0)

        while rclpy.ok() and not self._cancel.is_set() and gen == self._job_gen:
            if time.time() - t0 >= timeout:
                return False, 'align timeout', extra
            skip = self._skip_frames if first_pass else self._verify_skip
            snap, obj, size, rgb, note = self._one_snapshot_detect(
                gen, skip, max(6.0, self._detect_wait))
            first_pass = False
            if snap is None:
                return False, note or 'snapshot failed', extra
            if obj is None:
                self._flog('warn', f'{tag} {note}')
                self._publish_status(state='detect', message=f'{tag} {note}', confirm=0)
                if time.time() - t0 >= self._detect_wait:
                    return False, note or 'no detection', extra
                if not self._sleep_or_cancel(0.4, gen):
                    return False, 'canceled', extra
                continue

            w, h = size
            u_px, v_px = bbox_center_px(obj)
            color_intr = self._cam_info.color_or_fallback(w, h)
            obj_u = self._u_norm_of(obj, size)
            u_err = obj_u - float(grip_u)
            oms_now = self._oms_now()
            target = self._oms_target_deg(u_px, oms_now, color_intr)
            bearing = bearing_deg_from_pixel(
                u_px, color_intr, self._asign, self._oms_min, self._oms_max)
            bbox = [
                round(obj.x1, 1), round(obj.y1, 1),
                round(obj.x2, 1), round(obj.y2, 1)]
            extra = {
                'u_norm': round(u_err, 4),
                'bbox': bbox,
                'conf': round(float(obj.conf), 3),
                'class_id': int(obj.class_id),
                'class_name': obj.name,
                'size': [w, h],
                'u_px': round(u_px, 1),
                'v_px': round(v_px, 1),
                'bearing_deg': round(bearing, 2),
                'oms_angle_deg': round(oms_now, 2),
                'target_oms_deg': round(target, 2),
            }
            self._publish_debug_image(
                rgb, snap.header.stamp, obj, u_err, oms_now, target,
                f'{obj.name} conf={float(obj.conf):.2f}')
            self._flog(
                'info',
                f'{tag} {obj.name} u_obj={obj_u:+.3f} u_err={u_err:+.3f} '
                f'bearing={bearing:+.1f}° + oms={oms_now:+.1f}° → goto={target:+.1f}°')
            aligned = abs(u_err) <= self._center_tol
            self._publish_status(
                state='detect',
                message=(
                    f'{tag} {obj.name} err={u_err:+.3f} '
                    f'oms={oms_now:+.1f}° tgt={target:+.1f}°'),
                aligned=aligned,
                u_norm=round(u_err, 4),
                bbox=bbox,
                conf=round(float(obj.conf), 3),
                class_name=obj.name,
                confirm=confirm,
                oms_angle_deg=round(oms_now, 2),
                target_oms_deg=round(target, 2))

            if aligned:
                confirm += 1
                self._flog(
                    'info',
                    f'{tag} centered confirm={confirm}/{self._confirm_need}')
                if confirm >= self._confirm_need:
                    return True, f'aligned u_err={u_err:+.3f} oms={oms_now:+.1f}°', extra
                continue

            # Vision-only bench: skip OMS rotation — one detect then depth.
            if not self._oms_rotation_enabled:
                self._flog(
                    'info',
                    f'{tag} OMS rotation disabled — vision-only '
                    f'angle={target:+.1f}° u_err={u_err:+.3f}')
                return True, (
                    f'vision-only angle={target:+.1f}° u_err={u_err:+.3f}'), extra

            confirm = 0
            if moves >= self._max_oms_moves:
                return False, f'OMS moves exhausted u_err={u_err:+.3f}', extra
            self._publish_status(
                state='oms_goto',
                message=f'{tag} OMS goto {target:+.1f}° (now {oms_now:+.1f}°)',
                u_norm=round(u_err, 4),
                target_oms_deg=round(target, 2),
                oms_angle_deg=round(oms_now, 2))
            ok, message = self._oms_goto(target, gen)
            moves += 1
            if not ok:
                return False, f'OMS goto failed: {message}', extra
            if not self._sleep_or_cancel(0.25, gen):
                return False, 'canceled', extra
        return False, 'canceled', extra

    def _read_depth(self, extra: dict, gen: int) -> Tuple[Optional[float], str]:
        bbox = extra.get('bbox') or [0, 0, 0, 0]
        size = extra.get('size') or [1, 1]
        w = max(1.0, float(size[0]))
        h = max(1.0, float(size[1]))
        cx = 0.5 * (float(bbox[0]) + float(bbox[2]))
        cy = 0.5 * (float(bbox[1]) + float(bbox[3]))
        u = cx / w
        v = cy / h
        samples: List[float] = []
        deadline = time.time() + self._depth_timeout
        last_log = 0.0
        while rclpy.ok() and not self._cancel.is_set() and gen == self._job_gen:
            if time.time() >= deadline:
                break
            msg = self._depth
            if msg is None or time.time() - self._depth_stamp > 1.0:
                if time.time() - last_log >= self._log_period:
                    self._publish_status(state='read_depth', message='waiting depth frame')
                    last_log = time.time()
                time.sleep(0.05)
                continue
            depth = _depth_to_meters(msg)
            if depth is None:
                time.sleep(0.05)
                continue
            val = _sample_depth(depth, u, v, self._depth_roi)
            if val is None:
                time.sleep(0.05)
                continue
            samples.append(val)
            self._flog('info', f'depth sample {len(samples)}/{self._depth_samples} = {val:.3f}m')
            self._publish_status(
                state='read_depth',
                message=f'depth {len(samples)}/{self._depth_samples} = {val:.3f}m',
                depth_m=round(val, 3))
            if len(samples) >= self._depth_samples:
                break
            time.sleep(0.05)
        if not samples:
            return None, 'no valid depth samples'
        return float(np.median(np.array(samples, dtype=np.float32))), ''

    def _run(self, cmd: dict, gen: int) -> None:
        with self._lock:
            if self._busy and gen != self._job_gen:
                return
            self._busy = True
        self._cancel.clear()
        self._color = None
        self._depth = None
        cmd_id = str(cmd.get('cmd_id', ''))
        mode = str(cmd.get('mode') or 'detect_align').strip().lower()
        if mode in ('oms_pick', 'pick'):
            self._run_oms_pick(cmd, gen, cmd_id)
            return
        if mode in ('oms_drop', 'drop'):
            self._run_oms_drop(cmd, gen, cmd_id)
            return
        # Vision-only: color ON → YOLO class → color OFF. No OMS / depth / arm.
        if mode in ('classify', 'identify', 'detect_only'):
            self._run_classify(cmd, gen, cmd_id)
            return
        self._run_detect_align(cmd, gen, cmd_id)

    def _run_classify(self, cmd: dict, gen: int, cmd_id: str) -> None:
        """Commission vision ID only — enable frames at pick, return class_name."""
        del cmd  # no extra fields required
        extra_out: Dict[str, Any] = {}
        try:
            self._ensure_detector()
        except Exception as exc:
            self._finish(
                cmd_id, False, f'model load failed: {exc}', gen, extra_out,
                mode='classify')
            return

        self._publish_status(state='classify', message='enabling color stream')
        self._flog('info', 'CLASSIFY begin (vision only, no OMS)')
        self._set_cam(True, False)
        if not self._wait_stream_frames(gen, self._stream_need, self._stream_timeout):
            self._finish(
                cmd_id, False,
                f'no color stream ({self._color_n} frames)',
                gen, extra_out, mode='classify')
            return

        best_extra = None
        best_obj = None
        frames = 0
        t0 = time.time()
        while frames < self._need_frames and time.time() - t0 < self._detect_wait:
            if self._cancel.is_set() or gen != self._job_gen:
                self._finish(
                    cmd_id, False, 'canceled', gen, extra_out, mode='classify')
                return
            snap, obj, size, rgb, note = self._one_snapshot_detect(
                gen, self._skip_frames, max(4.0, self._detect_wait))
            if snap is None or obj is None or size is None:
                self._flog('warn', f'classify frame: {note}')
                continue
            frames += 1
            w, h = size
            u_px, v_px = bbox_center_px(obj)
            cand = {
                'bbox': [round(obj.x1, 1), round(obj.y1, 1),
                         round(obj.x2, 1), round(obj.y2, 1)],
                'conf': round(float(obj.conf), 3),
                'class_id': int(obj.class_id),
                'class_name': obj.name,
                'size': [w, h],
                'u_px': round(u_px, 1),
                'v_px': round(v_px, 1),
            }
            self._flog(
                'info',
                f'classify frame {frames}/{self._need_frames}: {obj.name} '
                f'{float(obj.conf) * 100:.0f}%')
            if best_obj is None or float(obj.conf) > float(best_obj.conf):
                best_obj = obj
                best_extra = cand

        self._set_cam(False, False)
        self._drop_detector()
        if best_extra is None:
            self._finish(
                cmd_id, False, 'no detection', gen, extra_out, mode='classify')
            return
        extra_out.update(best_extra)
        name = str(best_extra.get('class_name') or '')
        conf = float(best_extra.get('conf') or 0.0)
        self._finish(
            cmd_id, True,
            f'classified {name} ({conf * 100:.0f}%)',
            gen, extra_out, mode='classify')

    def _run_oms_drop(self, cmd: dict, gen: int, cmd_id: str) -> None:
        self._publish_status(state='oms_drop', message='drop: OMS home stub')
        try:
            timeout = float(cmd.get('timeout_sec') or self._oms_home_timeout)
        except (TypeError, ValueError):
            timeout = self._oms_home_timeout

        def _wait(t: float) -> Tuple[bool, str]:
            return self._wait_oms_home(t)

        ok, message, extra = run_drop_sequence(
            home_cmd_pub=self._home_cmd_pub,
            wait_home=_wait,
            log=lambda m: self._flog('info', m),
            timeout_sec=max(10.0, timeout),
        )
        self._finish(cmd_id, ok, message, gen, extra, mode='oms_drop')

    def _run_oms_pick(self, cmd: dict, gen: int, cmd_id: str) -> None:
        extra_out: Dict[str, Any] = {}
        try:
            lift_mm = float(cmd.get('lift_mm', self._pick_lift_mm))
        except (TypeError, ValueError):
            lift_mm = self._pick_lift_mm
        lift_mm = max(0.0, min(300.0, lift_mm))
        try:
            self._ensure_detector()
        except Exception as exc:
            self._finish(
                cmd_id, False, f'model load failed: {exc}', gen, extra_out,
                mode='oms_pick')
            return

        self._publish_status(state='oms_pick', message=f'pick lift={lift_mm:.0f}mm')
        self._flog('info', f'OMS_PICK begin lift={lift_mm:.0f}mm')

        # 1) Full home (lift up → linear rear → gripper close → rot servo in)
        self._publish_status(state='oms_pick', message='OMS home')
        ok_home, msg_home = self._wait_oms_home(self._oms_home_timeout)
        if not ok_home:
            self._finish(
                cmd_id, False, f'OMS home failed: {msg_home}', gen, extra_out,
                mode='oms_pick')
            return

        # 2) rotation_servo out → linear 10 cm → lift to pick height
        def _plog(m: str, level: str = 'info') -> None:
            self._flog(level, m)

        self._publish_status(state='oms_pick', message='pre-detect arm')
        ok_pre, msg_pre, pre_extra = run_pick_pre_detect(
            lift_pub=self._lift_cmd_pub,
            linear_pub=self._linear_cmd_pub,
            rot_servo_pub=self._rot_servo_pub,
            result_store=self._axis_results,
            lift_mm=lift_mm,
            pre_extend_mm=self._pre_extend_mm,
            linear_max_mm=self._linear_max_mm,
            lift_timeout=self._lift_timeout,
            linear_timeout=self._linear_timeout,
            servo_timeout=self._servo_timeout,
            linear_enabled=self._linear_cmd_enabled,
            log=_plog,
        )
        extra_out.update(pre_extra)
        if not ok_pre:
            self._finish(
                cmd_id, False, msg_pre, gen, extra_out, mode='oms_pick')
            return

        self._set_cam(True, False)
        if not self._wait_stream_frames(gen, self._stream_need, self._stream_timeout):
            self._finish(
                cmd_id, False, f'no color stream ({self._color_n} frames)',
                gen, extra_out, mode='oms_pick')
            return

        best_extra = None
        best_obj = None
        frames = 0
        t0 = time.time()
        while frames < self._need_frames and time.time() - t0 < self._detect_wait:
            if self._cancel.is_set() or gen != self._job_gen:
                self._finish(cmd_id, False, 'canceled', gen, extra_out, mode='oms_pick')
                return
            snap, obj, size, rgb, note = self._one_snapshot_detect(
                gen, self._skip_frames, max(4.0, self._detect_wait))
            if snap is None or obj is None or size is None:
                self._flog('warn', f'pick detect frame: {note}')
                continue
            frames += 1
            w, h = size
            u_px, v_px = bbox_center_px(obj)
            color_intr = self._cam_info.color_or_fallback(w, h)
            bearing = bearing_deg_from_pixel(
                u_px, color_intr, self._asign, self._oms_min, self._oms_max)
            target = oms_goto_target_deg(
                self._oms_now(), bearing, self._oms_min, self._oms_max)
            cand = {
                'bbox': [round(obj.x1, 1), round(obj.y1, 1),
                         round(obj.x2, 1), round(obj.y2, 1)],
                'conf': round(float(obj.conf), 3),
                'class_id': int(obj.class_id),
                'class_name': obj.name,
                'size': [w, h],
                'u_px': round(u_px, 1),
                'v_px': round(v_px, 1),
                'bearing_deg': round(bearing, 2),
                'target_oms_deg': round(target, 2),
                'oms_angle_deg': round(self._oms_now(), 2),
            }
            self._flog(
                'info',
                f'pick frame {frames}/{self._need_frames}: {obj.name} '
                f'{float(obj.conf) * 100:.0f}% yaw={bearing:+.1f}°')
            if best_obj is None or float(obj.conf) > float(best_obj.conf):
                best_obj = obj
                best_extra = cand

        if best_extra is None:
            self._set_cam(False, False)
            self._finish(cmd_id, False, 'no detection', gen, extra_out, mode='oms_pick')
            return

        extra_out.update(best_extra)
        target = float(best_extra['target_oms_deg'])
        self._flog('info', f'BEST {best_extra["class_name"]} → OMS {target:+.1f}°')
        ok_rot, msg_rot = self._oms_goto_tol(target, gen)
        if not ok_rot:
            self._finish(
                cmd_id, False, f'OMS rotate failed: {msg_rot}', gen, extra_out,
                mode='oms_pick')
            return

        linear_mm = None
        for refine in range(self._align_refine_max + 1):
            if self._cancel.is_set() or gen != self._job_gen:
                self._finish(cmd_id, False, 'canceled', gen, extra_out, mode='oms_pick')
                return
            self._set_cam(True, True)
            if self._depth_wait > 0:
                self._sleep_or_cancel(self._depth_wait, gen)
            # Snapshot detect without killing depth stream
            self._set_cam(True, True)
            if not self._wait_stream_frames(gen, 1, min(6.0, self._stream_timeout)):
                self._set_cam(False, False)
                self._finish(
                    cmd_id, False, 'post-rotate no color frames', gen,
                    extra_out, mode='oms_pick')
                return
            snap = self._grab_snapshot(gen, self._skip_frames, max(4.0, self._detect_wait))
            if snap is None:
                self._set_cam(False, False)
                self._finish(
                    cmd_id, False, 'post-rotate snapshot timeout', gen,
                    extra_out, mode='oms_pick')
                return
            obj, size, grip_u, rgb, note = self._detect_on_frame(snap)
            if obj is None or size is None:
                self._set_cam(False, False)
                self._finish(
                    cmd_id, False, f'post-rotate detect failed: {note}', gen,
                    extra_out, mode='oms_pick')
                return
            w, h = size
            u_px, v_px = bbox_center_px(obj)
            color_intr = self._cam_info.color_or_fallback(w, h)
            bearing = bearing_deg_from_pixel(
                u_px, color_intr, self._asign, self._oms_min, self._oms_max)
            extra_out.update({
                'bbox': [round(obj.x1, 1), round(obj.y1, 1),
                         round(obj.x2, 1), round(obj.y2, 1)],
                'conf': round(float(obj.conf), 3),
                'class_name': obj.name,
                'class_id': int(obj.class_id),
                'u_px': round(u_px, 1),
                'v_px': round(v_px, 1),
                'bearing_deg': round(bearing, 2),
                'size': [w, h],
                'oms_angle_deg': round(self._oms_now(), 2),
            })
            depth_m, dnote = self._read_depth(extra_out, gen)
            self._set_cam(False, False)
            if depth_m is None:
                self._finish(
                    cmd_id, False, dnote or 'depth failed', gen, extra_out,
                    mode='oms_pick')
                return
            linear_mm = depth_to_linear_mm(
                depth_m, self._cam_grip_mm, self._linear_off_mm,
                self._linear_max_mm, self._pre_extend_mm)
            remain = max(0.0, float(linear_mm) - self._pre_extend_mm)
            extra_out['depth_z_m'] = round(depth_m, 4)
            extra_out['depth_m'] = round(depth_m, 4)
            extra_out['linear_pos_mm'] = round(linear_mm, 2)
            extra_out['linear_remain_mm'] = round(remain, 2)
            extra_out['pick_lift_mm'] = round(lift_mm, 2)
            self._flog(
                'info',
                f'post-rotate yaw={bearing:+.1f}° Z={depth_m:.3f}m '
                f'remain={remain:.0f}mm '
                f'(depth−({self._cam_grip_mm:.0f}+{self._pre_extend_mm:.0f})'
                f'+{self._linear_off_mm:.0f}) → abs {linear_mm:.0f}mm')
            if abs(bearing) <= self._align_yaw_tol:
                break
            if refine >= self._align_refine_max:
                self._flog(
                    'warn',
                    f'bearing {bearing:+.1f}° still > {self._align_yaw_tol:.1f}°')
                break
            fix = oms_goto_target_deg(
                self._oms_now(), bearing, self._oms_min, self._oms_max)
            self._flog('info', f'ALIGN refine → {fix:+.1f}°')
            ok_rot, msg_rot = self._oms_goto_tol(fix, gen)
            if not ok_rot:
                self._finish(
                    cmd_id, False, f'align refine failed: {msg_rot}', gen,
                    extra_out, mode='oms_pick')
                return

        if linear_mm is None:
            self._finish(
                cmd_id, False, 'no linear after measure', gen, extra_out,
                mode='oms_pick')
            return

        # gripper open → linear to target → close → linear home
        ok, message, pick_extra = run_pick_post_depth(
            linear_pub=self._linear_cmd_pub,
            grip_pub=self._grip_pub,
            result_store=self._axis_results,
            linear_mm=linear_mm,
            pre_extend_mm=self._pre_extend_mm,
            linear_max_mm=self._linear_max_mm,
            linear_timeout=self._linear_timeout,
            servo_timeout=self._servo_timeout,
            linear_enabled=self._linear_cmd_enabled,
            log=_plog,
        )
        extra_out.update(pick_extra)
        self._drop_detector()
        self._finish(cmd_id, ok, message, gen, extra_out, mode='oms_pick')

    def _run_detect_align(self, cmd: dict, gen: int, cmd_id: str) -> None:
        timeout = self._timeout_of(cmd)
        attempts = self._retries + 1
        extra_out = {}
        ok = False
        message = 'failed'
        try:
            self._ensure_detector()
        except Exception as exc:
            message = f'model load failed: {exc}'
            self._finish(cmd_id, False, message, gen, extra_out)
            return

        self._flog(
            'info',
            f'OMS at {self._oms_now():+.1f}° homed={self._oms_homed} — '
            'vertical unchanged')

        self._publish_status(state='wait_color', message='enabling color stream')
        self._set_cam(True, False)
        if not self._wait_stream_frames(gen, self._stream_need, self._stream_timeout):
            message = (
                f'no color stream ({self._color_n} frames; {self._cam_graph_note()})')
            self._finish(cmd_id, False, message, gen, extra_out)
            return
        self._flog('info', f'color stream OK  oms={self._oms_now():+.1f}°')

        for attempt in range(1, attempts + 1):
            if not rclpy.ok() or self._cancel.is_set() or gen != self._job_gen:
                message = 'canceled'
                break
            ok, message, extra = self._one_attempt(timeout, gen, attempt, attempts)
            if extra:
                extra_out.update(extra)
            if ok:
                break
            self._flog('warn', f'attempt {attempt}/{attempts} failed: {message}')
            if attempt < attempts:
                self._publish_status(state='retry', message=f'pause {self._retry_pause:.1f}s')
                if not self._sleep_or_cancel(self._retry_pause, gen):
                    message = 'canceled'
                    break

        depth_m = None
        depth_h = None
        if ok and extra_out:
            self._drop_detector()
            self._publish_status(
                state='wait_depth',
                message=f'aligned — wait {self._depth_wait:.1f}s then depth ON')
            if not self._sleep_or_cancel(self._depth_wait, gen):
                ok = False
                message = 'canceled before depth'
            else:
                self._set_cam(False, True)
                self._sleep_or_cancel(0.5, gen)
                self._flog('info', 'depth stream ON — sampling distance from camera')
                depth_m, dnote = self._read_depth(extra_out, gen)
                if depth_m is None:
                    ok = False
                    message = dnote or 'depth failed'
                else:
                    size = extra_out.get('size') or [640, 360]
                    cw, ch = int(size[0]), int(size[1])
                    dw = int(self._depth.width) if self._depth is not None else cw
                    dh = int(self._depth.height) if self._depth is not None else ch
                    color_intr = self._cam_info.color_or_fallback(cw, ch)
                    depth_intr = self._cam_info.depth_or_fallback(dw, dh)
                    u_px = float(extra_out.get('u_px') or 0.0)
                    v_px = float(extra_out.get('v_px') or 0.0)
                    floor = compute_target_geometry(
                        u_px, v_px, depth_m,
                        color_intr, depth_intr, self._tilt,
                        self._asign, self._oms_min, self._oms_max)
                    extra_out['depth_m'] = round(depth_m, 4)
                    extra_out['depth_z_m'] = round(floor.z_depth_m, 4)
                    extra_out['depth_forward_m'] = round(floor.forward_m, 4)
                    extra_out['depth_horizontal_m'] = round(floor.forward_m, 4)
                    extra_out['lateral_m'] = round(floor.lateral_m, 4)
                    extra_out['floor_dist_m'] = round(floor.floor_dist_m, 4)
                    extra_out['bearing_deg'] = round(floor.bearing_deg, 2)
                    extra_out['yaw_deg'] = round(floor.yaw_deg, 2)
                    extra_out['camera_tilt_deg'] = round(math.degrees(self._tilt), 1)
                    depth_h = floor.forward_m
                    lat_side = 'center'
                    if floor.lateral_m > 0.005:
                        lat_side = 'L'
                    elif floor.lateral_m < -0.005:
                        lat_side = 'R'
                    message = (
                        f'aligned  Z={depth_m:.3f}m  fwd={floor.forward_m:.3f}m  '
                        f'lat={floor.lateral_m:+.3f}m ({lat_side})  '
                        f'bearing={floor.bearing_deg:+.1f}°  oms={self._oms_now():+.1f}°')
                    self._flog('info', message)

        self._drop_detector()
        self._set_cam(False, False)

        if ok and self._hold > 0.0:
            self._publish_status(state='hold', message=f'hold {self._hold:.1f}s')
            if not self._sleep_or_cancel(self._hold, gen):
                ok = False
                message = 'canceled during hold'

        extra_out['depth_m'] = None if depth_m is None else round(depth_m, 4)
        extra_out['depth_horizontal_m'] = (
            None if depth_h is None else round(depth_h, 4))
        if self._oms_angle is not None:
            extra_out.setdefault('oms_angle_deg', round(float(self._oms_angle), 3))
        self._finish(cmd_id, ok, message, gen, extra_out)

    def _finish(
            self, cmd_id: str, ok: bool, message: str, gen: int,
            extra: dict, mode: str = 'detect_align') -> None:
        if gen != self._job_gen:
            return
        self._drop_detector()
        self._set_cam(False, False)
        self._busy = False
        self._publish_status(
            state='done' if ok else 'failed',
            message=message,
            aligned=bool(ok),
            depth_m=extra.get('depth_m'),
            depth_horizontal_m=extra.get('depth_horizontal_m'),
            lateral_m=extra.get('lateral_m'),
            floor_dist_m=extra.get('floor_dist_m'),
            yaw_deg=extra.get('yaw_deg'),
            bbox=extra.get('bbox'),
            conf=extra.get('conf'),
            class_name=extra.get('class_name') or '',
            u_norm=extra.get('u_norm'),
            oms_angle_deg=extra.get('oms_angle_deg', self._oms_angle),
            target_oms_deg=extra.get('target_oms_deg'))
        safe_extra = {
            k: v for k, v in (extra or {}).items()
            if v is None or isinstance(v, (bool, int, float, str, list, dict))
        }
        self._result_pub.publish(
            result_msg(cmd_id, ok, mode, message, **safe_extra))
        self._flog('info' if ok else 'error', f'RESULT mode={mode} ok={ok} {message}')
        self.status.set(
            'idle' if ok else 'error',
            'waiting for Detect+Align' if ok else message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectAlignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._set_cam(False, False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
