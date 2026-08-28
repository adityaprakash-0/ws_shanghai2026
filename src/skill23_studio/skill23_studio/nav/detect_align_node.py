#!/usr/bin/env python3
"""Detect+Align: snapshot YOLO, OMS rotation, then one-shot depth.

Idle until /studio/detect_align/cmd. Color is toggled only for snapshots
(skip the first N frames, infer on the next). The base does not yaw —
the camera sits on the OMS at camera_tilt_deg and /oms/rotation/cmd
turns the mechanism. Depth is enabled after the object is confirmed
aligned with the gripper under center_tol_norm.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Empty, Float64, String
from std_srvs.srv import SetBool

from skill23_studio.detect_align_paths import yolov8n_ncnn_dir
from skill23_studio.nav.node_status import NodeStatusPub
from skill23_studio.nav.wp_align_common import result_msg


def _studio_log_path(name: str) -> Path:
    """Prefer the Pi workspace even when launched with sudo."""
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
    # Orbbec sometimes leaves encoding blank for rgb8.
    if buf.size == h * w * 3:
        return buf.reshape(h, w, 3).copy()
    return None


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
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_topic', '/studio/detect_align/cmd')
        self.declare_parameter('result_topic', '/studio/detect_align/result')
        self.declare_parameter('cancel_topic', '/studio/detect_align/cancel')
        self.declare_parameter('status_topic', '/studio/detect_align/status')
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('model_dir', '')
        self.declare_parameter('input_size', 320)
        self.declare_parameter('conf_threshold', 0.35)
        self.declare_parameter('nms_iou', 0.45)
        self.declare_parameter('num_threads', 1)
        self.declare_parameter('target_class', -1)
        self.declare_parameter('target_name', '')
        self.declare_parameter('min_box_area_frac', 0.002)
        self.declare_parameter('control_rate_hz', 8.0)
        self.declare_parameter('center_tol_norm', 0.12)
        self.declare_parameter('center_u_offset', 0.0)
        self.declare_parameter('confirm_frames', 3)
        self.declare_parameter('detect_wait_sec', 8.0)
        self.declare_parameter('align_timeout_sec', 20.0)
        self.declare_parameter('depth_after_align_sec', 1.0)
        self.declare_parameter('depth_samples', 5)
        self.declare_parameter('depth_sample_timeout_sec', 4.0)
        self.declare_parameter('depth_roi_frac', 0.08)
        self.declare_parameter('camera_tilt_deg', 35.0)
        self.declare_parameter('hold_after_done_sec', 1.0)
        self.declare_parameter('retries', 1)
        self.declare_parameter('retry_pause_sec', 0.4)
        self.declare_parameter('yaw_kp', 0.70)
        self.declare_parameter('yaw_ki', 0.0)
        self.declare_parameter('yaw_kd', 0.0)
        self.declare_parameter('yaw_i_limit', 0.2)
        self.declare_parameter('yaw_duty_min', 0.10)
        self.declare_parameter('yaw_duty_max', 0.13)
        self.declare_parameter('yaw_pulse_sec', 0.07)
        self.declare_parameter('angular_sign', 1.0)
        self.declare_parameter('log_hz', 2.0)
        self.declare_parameter('skip_frames', 4)
        self.declare_parameter('verify_skip_frames', 2)
        self.declare_parameter('hfov_deg', 69.0)
        self.declare_parameter('gripper_name', '')
        self.declare_parameter('gripper_u_norm', 0.0)
        self.declare_parameter('oms_cmd_topic', '/oms/rotation/cmd')
        self.declare_parameter('oms_result_topic', '/oms/rotation/result')
        self.declare_parameter('oms_status_topic', '/oms/rotation/status')
        self.declare_parameter('oms_timeout_sec', 25.0)
        self.declare_parameter('max_oms_moves', 5)
        self.declare_parameter('oms_vertical_cmd_topic', '/oms/vertical/cmd')
        self.declare_parameter('oms_vertical_result_topic', '/oms/vertical/result')
        self.declare_parameter('oms_vertical_timeout_sec', 8.0)

        self._dt = 1.0 / max(5.0, float(self.get_parameter('control_rate_hz').value))
        self._center_tol = abs(float(self.get_parameter('center_tol_norm').value))
        self._u_off = float(self.get_parameter('center_u_offset').value)
        self._confirm_need = max(1, int(self.get_parameter('confirm_frames').value))
        self._detect_wait = max(0.5, float(self.get_parameter('detect_wait_sec').value))
        self._align_timeout = max(2.0, float(self.get_parameter('align_timeout_sec').value))
        self._depth_wait = max(0.0, float(self.get_parameter('depth_after_align_sec').value))
        self._depth_samples = max(1, int(self.get_parameter('depth_samples').value))
        self._depth_timeout = max(0.5, float(self.get_parameter('depth_sample_timeout_sec').value))
        self._depth_roi = float(self.get_parameter('depth_roi_frac').value)
        self._tilt = math.radians(float(self.get_parameter('camera_tilt_deg').value))
        self._hold = max(0.0, float(self.get_parameter('hold_after_done_sec').value))
        self._retries = max(0, min(5, int(self.get_parameter('retries').value)))
        self._retry_pause = max(0.0, float(self.get_parameter('retry_pause_sec').value))
        self._wmin = abs(float(self.get_parameter('yaw_duty_min').value))
        self._wmax = abs(float(self.get_parameter('yaw_duty_max').value))
        if self._wmin > self._wmax:
            self._wmin, self._wmax = self._wmax, self._wmin
        self._pulse_sec = max(0.03, min(0.20, float(self.get_parameter('yaw_pulse_sec').value)))
        asig = float(self.get_parameter('angular_sign').value)
        self._asign = 1.0 if asig >= 0.0 else -1.0
        self._log_period = 1.0 / max(0.5, float(self.get_parameter('log_hz').value))
        self._skip_frames = max(0, int(self.get_parameter('skip_frames').value))
        self._verify_skip = max(0, int(self.get_parameter('verify_skip_frames').value))
        self._hfov = max(10.0, float(self.get_parameter('hfov_deg').value))
        self._gripper_name = str(self.get_parameter('gripper_name').value or '').strip()
        self._gripper_u = float(self.get_parameter('gripper_u_norm').value)
        self._oms_timeout = max(4.0, float(self.get_parameter('oms_timeout_sec').value))
        self._max_oms_moves = max(1, min(8, int(self.get_parameter('max_oms_moves').value)))
        self._vert_timeout = max(2.0, float(self.get_parameter('oms_vertical_timeout_sec').value))
        self._target_class = int(self.get_parameter('target_class').value)
        self._target_name = str(self.get_parameter('target_name').value or '').strip()
        self._min_area_frac = max(0.0, float(self.get_parameter('min_box_area_frac').value))
        model_dir = str(self.get_parameter('model_dir').value or '').strip()
        self._model_dir = model_dir or yolov8n_ncnn_dir()

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
        self._log_path = _DETECT_LOG
        self._log_write_failed = False
        self._det_cache = None
        self._det_stamp = 0.0
        self._depth: Optional[Image] = None
        self._depth_stamp = 0.0
        self._oms_angle: Optional[float] = None
        self._oms_result: Optional[dict] = None
        self._oms_homed = False
        self._vert_result: Optional[dict] = None
        self._status_payload = {
            'state': 'idle',
            'message': 'waiting for Detect+Align',
            'aligned': False,
            'stopped': True,
            'u_norm': None,
            'bbox': None,
            'conf': None,
            'class_id': None,
            'class_name': '',
            'depth_m': None,
            'depth_horizontal_m': None,
            'confirm': 0,
            'confirm_need': self._confirm_need,
            'oms_angle_deg': None,
            'gripper_u': None,
        }

        self._cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10)
        self._result_pub = self.create_publisher(
            String, str(self.get_parameter('result_topic').value), 10)
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10)
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
        self._vert_cmd_pub = self.create_publisher(
            String, str(self.get_parameter('oms_vertical_cmd_topic').value), 10)
        self.create_subscription(
            String, str(self.get_parameter('oms_vertical_result_topic').value),
            self._on_vert_result, 10)

        self.status = NodeStatusPub(self, 'detect_align')
        self.status.set('idle', 'waiting for Detect+Align (YOLO off)')
        self._publish_status()
        self.get_logger().info(
            f'detect_align idle  model={self._model_dir}  '
            f'center_tol={self._center_tol:.3f}  confirm={self._confirm_need}  '
            f'detect_wait={self._detect_wait:.1f}s  align_timeout={self._align_timeout:.0f}s  '
            f'depth_after_align={self._depth_wait:.1f}s  tilt={math.degrees(self._tilt):.1f}deg  '
            f'threads={int(self.get_parameter("num_threads").value)}  '
            f'target={self._target_name or self._target_class}  '
            f'snapshot skip={self._skip_frames}  hfov={self._hfov:.0f}deg  OMS yaw  '
            f'color={",".join(self._color_topics)}  log={self._log_path}')
        self._flog(
            'info',
            f'==== detect_align start {time.strftime("%Y-%m-%d %H:%M:%S")} '
            f'color={",".join(self._color_topics)} depth={self._depth_topic} '
            f'target={self._target_name or self._target_class} '
            f'log={self._log_path}')

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
        try:
            topics = [
                name for name, _types in self.get_topic_names_and_types()
                if 'image' in name and ('color' in name or 'depth' in name)
            ]
        except Exception:
            topics = []
        return (
            f'camera_nodes={nodes or ["(none)"]} '
            f'image_topics={topics[:8] or ["(none)"]} '
            f'color_n={self._color_n}')

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
                f'enc={msg.encoding or "?"} topic={self._color_topic} '
                f'{self._cam_graph_note()}')

    def _on_depth(self, msg: Image) -> None:
        self._depth = msg
        self._depth_stamp = time.time()

    def _on_cancel(self, _msg: Empty) -> None:
        self._cancel.set()
        self._stop()

    def _stop(self) -> None:
        try:
            if rclpy.ok():
                self._cmd_pub.publish(Twist())
        except Exception:
            pass

    def _set_cam(self, color: bool, depth: bool = False) -> None:
        try:
            self._cam_need.publish(Bool(data=bool(color)))
            self._cam_depth.publish(Bool(data=bool(depth)))
        except Exception:
            pass
        self._call_toggle('color', self._toggle_color_clis, bool(color))
        if depth or not color:
            self._call_toggle('depth', self._toggle_depth_clis, bool(depth))

    def _call_toggle(self, kind: str, clients, wanted: bool, wait_s: float = 8.0) -> bool:
        discovered = []
        try:
            for name, types in self.get_service_names_and_types():
                if f'toggle_{kind}' in name and any('SetBool' in t for t in types):
                    discovered.append(name)
        except Exception:
            pass
        if discovered:
            self._flog('info', f'toggle_{kind} services={discovered}')
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
                self._flog('warn', f'{name} call timeout')
                continue
            try:
                res = fut.result()
            except Exception as exc:
                self._flog('warn', f'{name} error: {exc}')
                continue
            msg = str(getattr(res, 'message', '') or '')
            ok = bool(getattr(res, 'success', False)) or (
                ('already on' in msg.lower()) if wanted else ('already off' in msg.lower()))
            self._flog(
                'info' if ok else 'warn',
                f'{name} {"ON" if wanted else "OFF"} ok={ok} {msg}'.strip())
            return ok
        for name in discovered:
            if name in seen:
                continue
            cli = self.create_client(SetBool, name)
            clients.append(cli)
            if not cli.wait_for_service(timeout_sec=2.0):
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
            ok = bool(getattr(res, 'success', False))
            self._flog(
                'info' if ok else 'warn',
                f'{name} {"ON" if wanted else "OFF"} ok={ok} {msg}'.strip())
            return ok
        self._flog(
            'error',
            f'toggle_{kind}={"ON" if wanted else "OFF"} FAILED '
            f'(no ready service; {self._cam_graph_note()})')
        return False

    def _on_oms_angle(self, msg: Float64) -> None:
        self._oms_angle = float(msg.data)

    def _on_oms_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if isinstance(data, dict):
            if data.get('angle_deg') is not None:
                try:
                    self._oms_angle = float(data['angle_deg'])
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

    def _on_vert_result(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if isinstance(data, dict):
            self._vert_result = data

    def _grab_snapshot(self, gen: int, skip: int, timeout: float):
        skip = max(0, int(skip))
        start_n = self._color_n
        need = start_n + skip + 1
        deadline = time.time() + max(2.0, timeout)
        last_note = -1
        while rclpy.ok() and not self._cancel.is_set() and gen == self._job_gen:
            if self._color is not None and self._color_n >= need:
                return self._color
            if time.time() >= deadline:
                return None
            elapsed = int(time.time() - (deadline - max(2.0, timeout)))
            if elapsed != last_note:
                last_note = elapsed
                self._flog(
                    'info',
                    f'snapshot skip {self._color_n - start_n}/{skip + 1} '
                    f'n={self._color_n}')
            time.sleep(0.05)
        return None

    def _oms_goto(self, angle_deg: float, gen: int) -> Tuple[bool, str]:
        cmd_id = f'da_{int(time.time() * 1000)}'
        self._oms_result = None
        payload = {'cmd_id': cmd_id, 'mode': 'goto', 'angle_deg': float(angle_deg)}
        self._oms_cmd_pub.publish(String(data=json.dumps(payload)))
        self._flog('info', f'OMS goto {angle_deg:+.2f}° id={cmd_id}')
        deadline = time.time() + self._oms_timeout
        while rclpy.ok() and not self._cancel.is_set() and gen == self._job_gen:
            res = self._oms_result
            if res and str(res.get('cmd_id') or '') == cmd_id:
                ok = bool(res.get('success'))
                msg = str(res.get('message') or '')
                if res.get('angle_deg') is not None:
                    try:
                        self._oms_angle = float(res['angle_deg'])
                    except (TypeError, ValueError):
                        pass
                return ok, msg
            if time.time() >= deadline:
                return False, 'OMS rotation timeout'
            time.sleep(0.05)
        return False, 'canceled'

    def _vertical_pose(self, pose: str, gen: int) -> Tuple[bool, str]:
        pose = 'up' if str(pose).strip().lower() == 'up' else 'down'
        cmd_id = f'da_v_{int(time.time() * 1000)}'
        self._vert_result = None
        payload = {'cmd_id': cmd_id, 'mode': pose}
        self._vert_cmd_pub.publish(String(data=json.dumps(payload)))
        self._flog('info', f'vertical {pose} id={cmd_id}')
        deadline = time.time() + self._vert_timeout
        while rclpy.ok() and not self._cancel.is_set() and gen == self._job_gen:
            res = self._vert_result
            if res and str(res.get('cmd_id') or '') == cmd_id:
                ok = bool(res.get('success'))
                return ok, str(res.get('message') or pose)
            if time.time() >= deadline:
                return False, f'vertical {pose} timeout'
            time.sleep(0.05)
        return False, 'canceled'

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
            self.get_logger().warn(f'[detect_align] bad command JSON: {exc}')
            self._result_pub.publish(
                result_msg('', False, 'detect_align', str(exc)))
            return
        self.get_logger().info(
            f'[detect_align] command id={cmd.get("cmd_id", "")}')
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
        return min(60.0, max(2.0, timeout))

    def _sleep_or_cancel(self, seconds: float, gen: int) -> bool:
        until = time.time() + max(0.0, seconds)
        while rclpy.ok() and not self._cancel.is_set() and gen == self._job_gen:
            self._stop()
            if time.time() >= until:
                return True
            time.sleep(self._dt)
        return False

    def _ensure_detector(self):
        if self._detector is not None:
            return self._detector
        from skill23_studio.nav.yolov8_ncnn import YoloV8Ncnn
        self.get_logger().info(f'[detect_align] loading NCNN {self._model_dir}')
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
        self._det_cache = None
        self._det_stamp = 0.0
        if det is None:
            return
        try:
            det.close()
        except Exception:
            pass

    def _wait_new_color(self, gen: int, seen: float, timeout: float) -> bool:
        deadline = time.time() + max(0.05, timeout)
        while rclpy.ok() and not self._cancel.is_set() and gen == self._job_gen:
            if (
                self._color is not None
                and self._color_stamp != seen
                and (time.time() - self._color_stamp) < 1.2
            ):
                return True
            if time.time() >= deadline:
                return False
            time.sleep(0.03)
        return False

    def _latest_det(self):
        msg = self._color
        if msg is None or time.time() - self._color_stamp > 1.2:
            return None, None, 'no color frame'
        if self._det_stamp == self._color_stamp and self._det_cache is not None:
            return self._det_cache
        rgb = _image_to_rgb(msg)
        if rgb is None:
            out = (None, None, f'bad color encoding {msg.encoding}')
            self._det_cache = out
            self._det_stamp = self._color_stamp
            return out
        dets = self._detector.detect(rgb)
        h, w = rgb.shape[:2]
        min_area = self._min_area_frac * float(w * h)
        picked = self._detector.pick(
            dets,
            class_id=self._target_class,
            class_name=self._target_name,
            min_area=min_area,
        )
        if picked is None:
            out = (None, (w, h), 'no target')
        else:
            out = (picked, (w, h), '')
        self._det_cache = out
        self._det_stamp = self._color_stamp
        return out

    def _u_norm_of(self, picked, size: Tuple[int, int]) -> float:
        """Horizontal error of the *object center* vs the image center.

        Object center = midpoint of the YOLO box (x1+x2)/2. We do not try
        to fit the whole box into the middle — only that point.
        u_norm = 0 on the vertical midline, −1 left edge, +1 right edge.
        """
        w = float(size[0])
        if w <= 1.0:
            return 0.0
        obj_cx = 0.5 * (float(picked.x1) + float(picked.x2))
        frame_cx = 0.5 * w
        return (obj_cx - frame_cx) / (0.5 * w) - self._u_off

    def _detect_msg(self, msg: Image):
        rgb = _image_to_rgb(msg)
        if rgb is None:
            return None, None, None, f'bad color encoding {msg.encoding}'
        dets = self._detector.detect(rgb)
        h, w = rgb.shape[:2]
        min_area = self._min_area_frac * float(w * h)
        obj = self._detector.pick(
            dets,
            class_id=self._target_class,
            class_name=self._target_name,
            min_area=min_area,
        )
        grip = None
        if self._gripper_name:
            grip = self._detector.pick(
                dets, class_name=self._gripper_name, min_area=min_area)
        grip_u = (
            self._u_norm_of(grip, (w, h)) if grip is not None
            else self._gripper_u)
        if obj is None:
            return None, (w, h), grip_u, 'no target'
        return obj, (w, h), grip_u, ''

    def _one_attempt(self, timeout: float, gen: int, attempt: int, attempts: int):
        tag = f'{attempt}/{attempts}'
        t0 = time.time()
        confirm = 0
        moves = 0
        extra = None
        self._flog('info', f'attempt {tag} snapshot skip={self._skip_frames} OMS align')
        self._publish_status(
            state='snapshot', message=f'{tag} snapshot',
            aligned=False, stopped=True, confirm=0)

        while rclpy.ok() and not self._cancel.is_set() and gen == self._job_gen:
            if time.time() - t0 >= timeout:
                return False, 'align timeout', extra
            self._set_cam(True, False)
            skip = self._skip_frames if extra is None else self._verify_skip
            snap = self._grab_snapshot(gen, skip, max(6.0, self._detect_wait))
            self._set_cam(False, False)
            if snap is None:
                return False, (
                    f'no snapshot ({self._color_n} frames; {self._cam_graph_note()})'
                ), extra
            obj, size, grip_u, note = self._detect_msg(snap)
            if obj is None:
                self._flog('warn', f'{tag} {note}')
                self._publish_status(
                    state='detect', message=f'{tag} {note}',
                    aligned=False, stopped=True, confirm=0)
                if time.time() - t0 >= self._detect_wait:
                    return False, note or 'no detection', extra
                if not self._sleep_or_cancel(0.3, gen):
                    return False, 'canceled', extra
                continue
            w, h = size
            obj_u = self._u_norm_of(obj, size)
            u_err = obj_u - float(grip_u)
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
                'oms_angle_deg': self._oms_angle,
                'gripper_u': round(float(grip_u), 4),
            }
            self._flog(
                'info',
                f'{tag} {obj.name} u_obj={obj_u:+.3f} u_grip={grip_u:+.3f} '
                f'err={u_err:+.3f} oms={self._oms_angle}')
            self._publish_status(
                state='detect',
                message=(
                    f'{tag} {obj.name} err={u_err:+.3f} '
                    f'oms={self._oms_angle}'),
                aligned=abs(u_err) <= self._center_tol,
                stopped=True,
                u_norm=round(u_err, 4),
                bbox=bbox,
                conf=round(float(obj.conf), 3),
                class_id=int(obj.class_id),
                class_name=obj.name,
                confirm=confirm,
                size=[w, h],
                oms_angle_deg=self._oms_angle,
                gripper_u=round(float(grip_u), 4))
            if abs(u_err) <= self._center_tol:
                confirm += 1
                if confirm >= self._confirm_need:
                    return True, f'aligned u_err={u_err:+.3f}', extra
                continue
            confirm = 0
            if moves >= self._max_oms_moves:
                return False, f'OMS moves exhausted u_err={u_err:+.3f}', extra
            delta = -self._asign * u_err * (self._hfov * 0.5)
            current = float(self._oms_angle) if self._oms_angle is not None else 0.0
            target = max(-90.0, min(90.0, current + delta))
            self._publish_status(
                state='oms_goto',
                message=f'{tag} OMS {current:+.1f}° → {target:+.1f}°',
                aligned=False, stopped=True, u_norm=round(u_err, 4),
                oms_angle_deg=current)
            ok, message = self._oms_goto(target, gen)
            moves += 1
            if not ok:
                return False, f'OMS goto failed: {message}', extra
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
                    self._publish_status(
                        state='read_depth', message='waiting for depth frame',
                        aligned=True, stopped=True)
                    last_log = time.time()
                time.sleep(self._dt)
                continue
            depth = _depth_to_meters(msg)
            if depth is None:
                time.sleep(self._dt)
                continue
            val = _sample_depth(depth, u, v, self._depth_roi)
            if val is None:
                if time.time() - last_log >= self._log_period:
                    self._publish_status(
                        state='read_depth', message='no valid depth in ROI',
                        aligned=True, stopped=True)
                    last_log = time.time()
                time.sleep(self._dt)
                continue
            samples.append(val)
            self._publish_status(
                state='read_depth',
                message=f'depth sample {len(samples)}/{self._depth_samples} = {val:.3f}m',
                aligned=True, stopped=True, depth_m=round(val, 3))
            if len(samples) >= self._depth_samples:
                break
            time.sleep(self._dt)
        if not samples:
            return None, 'no valid depth samples'
        optical = float(np.median(np.array(samples, dtype=np.float32)))
        return optical, ''

    def _run(self, cmd: dict, gen: int) -> None:
        with self._lock:
            if self._busy and gen != self._job_gen:
                return
            self._busy = True
        self._cancel.clear()
        self._color = None
        self._depth = None
        self._det_cache = None
        self._det_stamp = 0.0
        cmd_id = str(cmd.get('cmd_id', ''))
        timeout = self._timeout_of(cmd)
        attempts = self._retries + 1
        extra_out = {}
        ok = False
        message = 'failed'
        try:
            self._ensure_detector()
        except Exception as exc:
            message = f'model load failed: {exc}'
            self.get_logger().error(f'[detect_align] {message}')
            self._finish(cmd_id, False, message, gen, extra_out)
            return

        self._publish_status(
            state='vertical_up', message='vertical servo up (camera can see gripper)',
            aligned=False, stopped=True)
        vok, vmsg = self._vertical_pose('up', gen)
        self._flog('info' if vok else 'warn', f'vertical up ok={vok} {vmsg}')
        if not vok:
            message = f'vertical up failed: {vmsg}'
            self._finish(cmd_id, False, message, gen, extra_out)
            return

        self._set_cam(True, False)
        self._publish_status(
            state='wait_color', message='color stream requested',
            aligned=False, stopped=True, depth_m=None)
        self._flog(
            'info',
            f'color ON requested topics={",".join(self._color_topics)} '
            f'frames_so_far={self._color_n} {self._cam_graph_note()}')
        wait_s = 15.0
        t_wait = time.time()
        last_note = -1
        while (
            rclpy.ok() and not self._cancel.is_set() and gen == self._job_gen
            and self._color is None and (time.time() - t_wait) < wait_s
        ):
            elapsed = int(time.time() - t_wait)
            if elapsed != last_note:
                last_note = elapsed
                note = self._cam_graph_note()
                self._flog(
                    'info',
                    f'waiting for first color {elapsed:.0f}/{wait_s:.0f}s '
                    f'{self._color_topic} {note}')
                self._publish_status(
                    state='wait_color',
                    message=f'waiting for color {elapsed:.0f}s',
                    aligned=False, stopped=True)
                if elapsed >= 4:
                    try:
                        cam_nodes = [
                            n for n in self.get_node_names()
                            if 'camera' in n.lower()]
                    except Exception:
                        cam_nodes = ['?']
                    if not cam_nodes:
                        message = (
                            'camera node gone — Gemini E crashed after '
                            'toggle_color ON (see nav.log Check failed / SIGABRT)')
                        self._flog('error', message)
                        self._finish(cmd_id, False, message, gen, extra_out)
                        return
            time.sleep(0.05)
        if self._color is None:
            message = (
                f'no color frame after {wait_s:.0f}s '
                f'({self._color_n} frames on {self._color_topic}; '
                f'{self._cam_graph_note()})')
            self._flog('error', message)
            self._finish(cmd_id, False, message, gen, extra_out)
            return
        else:
            msg = self._color
            self._flog(
                'info',
                f'color ready {int(msg.width)}x{int(msg.height)} '
                f'enc={msg.encoding or "?"} n={self._color_n}')

        for attempt in range(1, attempts + 1):
            if not rclpy.ok() or self._cancel.is_set() or gen != self._job_gen:
                message = 'canceled'
                break
            ok, message, extra = self._one_attempt(timeout, gen, attempt, attempts)
            if extra:
                extra_out.update(extra)
            if ok:
                break
            self.get_logger().warn(
                f'[detect_align] attempt {attempt}/{attempts} failed: {message}')
            self._flog('warn', f'attempt {attempt}/{attempts} failed: {message}')
            if attempt < attempts:
                self._publish_status(
                    state='retry', message=f'pause {self._retry_pause:.1f}s',
                    aligned=False, stopped=True)
                if not self._sleep_or_cancel(self._retry_pause, gen):
                    message = 'canceled'
                    break

        depth_m = None
        depth_h = None
        if ok and extra_out:
            self._stop()
            self._drop_detector()
            self._publish_status(
                state='wait_depth',
                message=f'aligned — wait {self._depth_wait:.1f}s then depth',
                aligned=True, stopped=True)
            if not self._sleep_or_cancel(self._depth_wait, gen):
                ok = False
                message = 'canceled before depth'
            else:
                self._set_cam(True, True)
                self._publish_status(
                    state='read_depth', message='depth stream on',
                    aligned=True, stopped=True)
                # Give Orbbec a moment after toggle_depth.
                self._sleep_or_cancel(0.4, gen)
                depth_m, dnote = self._read_depth(extra_out, gen)
                if depth_m is None:
                    ok = False
                    message = dnote or 'depth failed'
                else:
                    depth_h = float(depth_m) * math.cos(self._tilt)
                    extra_out['depth_m'] = round(depth_m, 4)
                    extra_out['depth_horizontal_m'] = round(depth_h, 4)
                    extra_out['camera_tilt_deg'] = round(math.degrees(self._tilt), 1)
                    message = (
                        f'aligned depth={depth_m:.3f}m '
                        f'horizontal={depth_h:.3f}m')

        # Streams off as soon as depth is done — hold does not need the camera.
        self._drop_detector()
        self._set_cam(False, False)

        if ok and self._hold > 0.0:
            self._publish_status(
                state='hold', message=f'{self._hold:.1f}s before next waypoint',
                aligned=True, stopped=True,
                depth_m=None if depth_m is None else round(depth_m, 3),
                depth_horizontal_m=None if depth_h is None else round(depth_h, 3))
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
            extra: dict) -> None:
        self._stop()
        if gen != self._job_gen:
            return
        self._drop_detector()
        self._set_cam(False, False)
        self._busy = False
        self._publish_status(
            state='done' if ok else 'failed',
            message=message,
            aligned=bool(ok),
            stopped=True,
            depth_m=extra.get('depth_m'),
            depth_horizontal_m=extra.get('depth_horizontal_m'),
            bbox=extra.get('bbox'),
            conf=extra.get('conf'),
            class_name=extra.get('class_name') or '',
            u_norm=extra.get('u_norm'),
            oms_angle_deg=extra.get('oms_angle_deg', self._oms_angle),
            gripper_u=extra.get('gripper_u'))
        self._result_pub.publish(
            result_msg(cmd_id, ok, 'detect_align', message, **extra))
        self.get_logger().info(
            f'[detect_align] result ok={ok} {message}')
        self._flog('info' if ok else 'error', f'result ok={ok} {message}')
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
        node._stop()
        node._set_cam(False, False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
