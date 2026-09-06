#!/usr/bin/env python3
"""Detect+Align OMS pick bench — home, detect yaw, rotate, depth, lift, linear, close.

Sequence:
  1. OMS full home
  2. Color ON → YOLO (3 frames) → debug image → color OFF
  3. OMS rotate to calculated yaw
  4. Color+depth ON → detect + depth → streams OFF
  5. Rotation servo in → vertical up + gripper open
  6. Lift → linear → gripper close → linear home
  7. Post-pick debug image
"""

from __future__ import annotations

import json
import os
import threading
import time
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty, Float64, String
from std_srvs.srv import SetBool

from skill23_studio.detect_align_paths import yolov8n_ncnn_dir
from skill23_studio.nav.detect_align_geometry import (
    CameraInfoCache,
    TargetGeometry,
    bbox_center_px,
    bearing_deg_from_pixel,
    oms_goto_target_deg,
    pitch_rad_from_param,
    u_norm_from_pixel,
    yaw_direction,
)
from skill23_studio.nav.detect_align_node import (
    _depth_to_meters,
    _image_to_rgb,
    _rgb_to_bgr_msg,
    _sample_depth,
)
from skill23_studio.ros_dds import apply_studio_dds_env


def _ok_toggle(res, wanted: bool) -> bool:
    if bool(getattr(res, 'success', False)):
        return True
    msg = str(getattr(res, 'message', '') or '').lower()
    if wanted and 'already on' in msg:
        return True
    if (not wanted) and 'already off' in msg:
        return True
    return False


class DetectAlignOmsTestNode(Node):
    def __init__(self) -> None:
        super().__init__('detect_align_oms_test')
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('color_camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('depth_camera_info_topic', '/camera/depth/camera_info')
        self.declare_parameter('debug_image_topic', '/studio/detect_align/debug_image')
        self.declare_parameter('start_topic', '/studio/detect_align_oms_test/start')
        self.declare_parameter('model_dir', '')
        self.declare_parameter('input_size', 320)
        self.declare_parameter('conf_threshold', 0.35)
        self.declare_parameter('nms_iou', 0.45)
        self.declare_parameter('num_threads', 1)
        self.declare_parameter('target_class', -1)
        self.declare_parameter('target_name', '')
        self.declare_parameter('min_box_area_frac', 0.002)
        self.declare_parameter('hfov_deg', 69.0)
        self.declare_parameter('angular_sign', 1.0)
        self.declare_parameter('center_u_offset', 0.0)
        self.declare_parameter('camera_tilt_deg', 35.0)
        self.declare_parameter('camera_tilt_from_vertical', False)
        self.declare_parameter('depth_samples', 4)
        self.declare_parameter('depth_roi_frac', 0.08)
        self.declare_parameter('depth_sample_timeout_sec', 6.0)
        self.declare_parameter('depth_after_align_sec', 0.4)
        self.declare_parameter('need_frames', 3)
        self.declare_parameter('skip_frames', 3)
        self.declare_parameter('stream_verify_timeout_sec', 12.0)
        self.declare_parameter('detect_wait_sec', 10.0)
        self.declare_parameter('oms_min_deg', -90.0)
        self.declare_parameter('oms_max_deg', 90.0)
        self.declare_parameter('oms_cmd_topic', '/oms/rotation/cmd')
        self.declare_parameter('oms_result_topic', '/oms/rotation/result')
        self.declare_parameter('oms_timeout_sec', 25.0)
        self.declare_parameter('oms_align_tol_deg', 1.0)
        self.declare_parameter('align_yaw_tol_deg', 2.0)
        self.declare_parameter('align_refine_max', 2)
        self.declare_parameter('oms_home_cmd_topic', '/oms/home/cmd')
        self.declare_parameter('oms_home_result_topic', '/oms/home/result')
        self.declare_parameter('oms_home_timeout_sec', 180.0)
        self.declare_parameter('home_on_start', True)
        self.declare_parameter('pick_lift_mm', 100.0)
        self.declare_parameter('camera_to_gripper_mm', 455.0)
        self.declare_parameter('linear_fwd_offset_mm', 0.0)
        self.declare_parameter('linear_max_mm', 100.0)
        self.declare_parameter('linear_cmd_enabled', True)
        self.declare_parameter('linear_cmd_topic', '/oms/linear/cmd')
        self.declare_parameter('linear_result_topic', '/oms/linear/result')
        self.declare_parameter('linear_timeout_sec', 45.0)
        self.declare_parameter('lift_cmd_topic', '/oms/lift/cmd')
        self.declare_parameter('lift_result_topic', '/oms/lift/result')
        self.declare_parameter('lift_timeout_sec', 40.0)
        self.declare_parameter('rotation_servo_cmd_topic', '/oms/rotation_servo/cmd')
        self.declare_parameter('rotation_servo_result_topic', '/oms/rotation_servo/result')
        self.declare_parameter('vertical_cmd_topic', '/oms/vertical/cmd')
        self.declare_parameter('vertical_result_topic', '/oms/vertical/result')
        self.declare_parameter('gripper_cmd_topic', '/oms/gripper/cmd')
        self.declare_parameter('gripper_result_topic', '/oms/gripper/result')
        self.declare_parameter('servo_timeout_sec', 12.0)
        self.declare_parameter('pick_mode', True)
        self.declare_parameter('post_pick_frames', 3)

        raw_model = str(self.get_parameter('model_dir').value or '').strip()
        self._model_dir = raw_model or yolov8n_ncnn_dir()
        self._target_class = int(self.get_parameter('target_class').value)
        self._target_name = str(self.get_parameter('target_name').value or '').strip()
        self._min_area_frac = float(self.get_parameter('min_box_area_frac').value)
        self._hfov = float(self.get_parameter('hfov_deg').value)
        asig = float(self.get_parameter('angular_sign').value)
        self._asign = 1.0 if asig >= 0.0 else -1.0
        self._u_off = float(self.get_parameter('center_u_offset').value)
        self._tilt = pitch_rad_from_param(
            float(self.get_parameter('camera_tilt_deg').value),
            bool(self.get_parameter('camera_tilt_from_vertical').value))
        self._depth_samples = max(1, int(self.get_parameter('depth_samples').value))
        self._depth_roi = float(self.get_parameter('depth_roi_frac').value)
        self._depth_timeout = max(1.0, float(
            self.get_parameter('depth_sample_timeout_sec').value))
        self._depth_after_align = max(0.0, float(self.get_parameter('depth_after_align_sec').value))
        self._need_frames = max(1, int(self.get_parameter('need_frames').value))
        self._skip_frames = max(0, int(self.get_parameter('skip_frames').value))
        self._stream_wait = max(2.0, float(
            self.get_parameter('stream_verify_timeout_sec').value))
        self._detect_wait = max(1.0, float(self.get_parameter('detect_wait_sec').value))
        self._oms_min = float(self.get_parameter('oms_min_deg').value)
        self._oms_max = float(self.get_parameter('oms_max_deg').value)
        self._oms_timeout = max(4.0, float(self.get_parameter('oms_timeout_sec').value))
        self._oms_align_tol = max(0.1, float(self.get_parameter('oms_align_tol_deg').value))
        self._align_yaw_tol = max(0.1, float(self.get_parameter('align_yaw_tol_deg').value))
        self._align_refine_max = max(0, int(self.get_parameter('align_refine_max').value))
        self._home_on_start = bool(self.get_parameter('home_on_start').value)
        self._home_timeout = max(10.0, float(self.get_parameter('oms_home_timeout_sec').value))
        self._pick_lift_mm = max(0.0, float(self.get_parameter('pick_lift_mm').value))
        self._cam_grip_mm = max(0.0, float(self.get_parameter('camera_to_gripper_mm').value))
        self._linear_off_mm = float(self.get_parameter('linear_fwd_offset_mm').value)
        self._linear_max_mm = max(1.0, float(self.get_parameter('linear_max_mm').value))
        self._linear_cmd_enabled = bool(self.get_parameter('linear_cmd_enabled').value)
        self._linear_timeout = max(4.0, float(self.get_parameter('linear_timeout_sec').value))
        self._lift_timeout = max(4.0, float(self.get_parameter('lift_timeout_sec').value))
        self._servo_timeout = max(2.0, float(self.get_parameter('servo_timeout_sec').value))
        self._pick_mode = bool(self.get_parameter('pick_mode').value)
        self._post_pick_frames = max(1, int(self.get_parameter('post_pick_frames').value))
        self._debug_topic = str(self.get_parameter('debug_image_topic').value)

        self._oms_home_cmd_topic = str(self.get_parameter('oms_home_cmd_topic').value)
        self._oms_home_result_topic = str(self.get_parameter('oms_home_result_topic').value)
        self._lift_cmd_topic = str(self.get_parameter('lift_cmd_topic').value)
        self._lift_result_topic = str(self.get_parameter('lift_result_topic').value)
        self._linear_cmd_topic = str(self.get_parameter('linear_cmd_topic').value)
        self._linear_result_topic = str(self.get_parameter('linear_result_topic').value)
        self._rot_servo_cmd = str(self.get_parameter('rotation_servo_cmd_topic').value)
        self._rot_servo_res = str(self.get_parameter('rotation_servo_result_topic').value)
        self._vert_cmd = str(self.get_parameter('vertical_cmd_topic').value)
        self._vert_res = str(self.get_parameter('vertical_result_topic').value)
        self._grip_cmd = str(self.get_parameter('gripper_cmd_topic').value)
        self._grip_res = str(self.get_parameter('gripper_result_topic').value)

        self._cam_info = CameraInfoCache(self._hfov)
        self._color_n = 0
        self._color: Optional[Image] = None
        self._depth: Optional[Image] = None
        self._depth_stamp = 0.0
        self._detector = None
        self._busy = False
        self._start_evt = threading.Event()
        self._oms_angle: Optional[float] = None
        self._oms_tracked_deg = 0.0
        self._oms_result: Optional[dict] = None
        self._home_result: Optional[dict] = None
        self._home_status: Optional[dict] = None
        self._axis_results: Dict[str, dict] = {}
        self._linear_pos_mm: Optional[float] = None

        self._debug_pub = self.create_publisher(Image, self._debug_topic, 10)
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
        self._oms_cmd_pub = self.create_publisher(
            String, str(self.get_parameter('oms_cmd_topic').value), 10)
        self._home_cmd_pub = self.create_publisher(String, self._oms_home_cmd_topic, 10)
        self._lift_cmd_pub = self.create_publisher(String, self._lift_cmd_topic, 10)
        self._linear_cmd_pub = self.create_publisher(String, self._linear_cmd_topic, 10)
        self._rot_servo_pub = self.create_publisher(String, self._rot_servo_cmd, 10)
        self._vert_pub = self.create_publisher(String, self._vert_cmd, 10)
        self._grip_pub = self.create_publisher(String, self._grip_cmd, 10)

        color_topic = str(self.get_parameter('color_topic').value)
        depth_topic = str(self.get_parameter('depth_topic').value)
        color_topics: List[str] = []
        for topic in (
                color_topic,
                '/camera/color/image_raw',
                '/camera/camera/color/image_raw',
                '/color/image_raw'):
            if topic and topic not in color_topics:
                color_topics.append(topic)
        reliable = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        for topic in color_topics:
            self.create_subscription(
                Image, topic, self._on_color, qos_profile_sensor_data)
            self.create_subscription(Image, topic, self._on_color, reliable)
        depth_topics: List[str] = []
        for topic in (
                depth_topic,
                '/camera/depth/image_raw',
                '/camera/camera/depth/image_raw',
                '/depth/image_raw'):
            if topic and topic not in depth_topics:
                depth_topics.append(topic)
        for topic in depth_topics:
            self.create_subscription(
                Image, topic, self._on_depth, qos_profile_sensor_data)
            self.create_subscription(Image, topic, self._on_depth, reliable)
        for topic, cb in (
                (str(self.get_parameter('color_camera_info_topic').value),
                 self._on_color_info),
                (str(self.get_parameter('depth_camera_info_topic').value),
                 self._on_depth_info),
        ):
            if topic:
                self.create_subscription(CameraInfo, topic, cb, qos_profile_sensor_data)
        self.create_subscription(
            Empty, str(self.get_parameter('start_topic').value),
            self._on_start, 10)
        self.create_subscription(
            Float64, '/oms/rotation/angle_deg', self._on_oms_angle, 10)
        self.create_subscription(
            String, str(self.get_parameter('oms_result_topic').value),
            self._on_oms_result, 10)
        self.create_subscription(
            String, self._oms_home_result_topic, self._on_home_result, 10)
        self.create_subscription(
            String, '/oms/home/status', self._on_home_status, 10)
        self.create_subscription(
            String, self._lift_result_topic, self._on_lift_result, 10)
        self.create_subscription(
            String, self._linear_result_topic, self._on_linear_result, 10)
        self.create_subscription(
            String, self._rot_servo_res, self._on_rot_servo_result, 10)
        self.create_subscription(
            String, self._vert_res, self._on_vert_result, 10)
        self.create_subscription(
            String, self._grip_res, self._on_grip_result, 10)
        self.create_subscription(
            Float64, '/oms/linear/pos_mm', self._on_linear_pos, 10)

        domain = os.environ.get('ROS_DOMAIN_ID', '0')
        rmw = os.environ.get('RMW_IMPLEMENTATION', '?')
        transport = os.environ.get('FASTDDS_BUILTIN_TRANSPORTS', '?')
        if os.geteuid() == 0:
            self._log(
                'running as root — use: source install/setup.bash && '
                'ros2 run ... (no sudo). DDS may not see the launch stack.',
                'warn')
        self._log(
            'ready — OMS pick test. '
            f'domain={domain}  rmw={rmw}  fastdds={transport}  '
            f'target={self._target_name or "any (excl. class 0)"}  '
            f'pick_mode={self._pick_mode}  lift={self._pick_lift_mm:.0f}mm  '
            f'cam→grip={self._cam_grip_mm:.0f}mm  fwd_offset={self._linear_off_mm:.0f}mm  '
            f'linear_max={self._linear_max_mm:.0f}mm  '
            f'frames={self._need_frames}  '
            f'align_tol={self._align_yaw_tol:.1f}°  '
            f'oms_tol={self._oms_align_tol:.1f}°')
        self._log('Wait for launch stack (camera + oms_all), then type start + Enter')
        self._log(f'VIEW: rqt_image_view {self._debug_topic}')
        threading.Thread(target=self._stdin_loop, daemon=True).start()

    def _log(self, msg: str, level: str = 'info') -> None:
        text = f'[detect_align_oms_test] {msg}'
        if level == 'error':
            self.get_logger().error(text)
        elif level == 'warn':
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

    def _on_start(self, _msg: Empty) -> None:
        self._request_start('topic')

    def _request_start(self, source: str) -> None:
        if self._busy:
            self._log(f'busy — ignore START from {source}', 'warn')
            return
        self._log(f'START received ({source})')
        self._start_evt.set()

    def _stdin_loop(self) -> None:
        import sys
        while rclpy.ok():
            try:
                line = sys.stdin.readline()
            except Exception:
                break
            if not line:
                time.sleep(0.2)
                continue
            cmd = line.strip().lower()
            if cmd in ('start', 'go', 'run', 's'):
                self._request_start('keyboard')
            elif cmd in ('q', 'quit', 'exit'):
                rclpy.shutdown()
                break

    def _on_color(self, msg: Image) -> None:
        self._color_n += 1
        self._color = msg

    def _on_depth(self, msg: Image) -> None:
        self._depth = msg
        self._depth_stamp = time.time()

    def _on_color_info(self, msg: CameraInfo) -> None:
        self._cam_info.set_color_info(msg)

    def _on_depth_info(self, msg: CameraInfo) -> None:
        self._cam_info.set_depth_info(msg)

    def _on_oms_angle(self, msg: Float64) -> None:
        ang = float(msg.data)
        self._oms_angle = ang
        self._oms_tracked_deg = ang

    def _on_oms_result(self, msg: String) -> None:
        self._store_json(msg, '_oms_result')

    def _on_home_result(self, msg: String) -> None:
        self._store_json(msg, '_home_result')

    def _on_home_status(self, msg: String) -> None:
        self._store_json(msg, '_home_status')

    def _on_lift_result(self, msg: String) -> None:
        self._store_axis('lift', msg)

    def _on_linear_result(self, msg: String) -> None:
        self._store_axis('linear', msg)

    def _on_rot_servo_result(self, msg: String) -> None:
        self._store_axis('rotation_servo', msg)

    def _on_vert_result(self, msg: String) -> None:
        self._store_axis('vertical', msg)

    def _on_grip_result(self, msg: String) -> None:
        self._store_axis('gripper', msg)

    def _on_linear_pos(self, msg: Float64) -> None:
        self._linear_pos_mm = float(msg.data)

    def _store_json(self, msg: String, attr: str) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if isinstance(data, dict):
            setattr(self, attr, data)

    def _store_axis(self, name: str, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if isinstance(data, dict):
            self._axis_results[name] = data

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

    def _oms_target_deg(self, bearing_deg: float, oms_now: float) -> float:
        return oms_goto_target_deg(oms_now, bearing_deg, self._oms_min, self._oms_max)

    def _wait_cmd_result(
            self, cmd_id: str, getter, timeout: float,
            label: str) -> Tuple[bool, str]:
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            res = getter()
            if not isinstance(res, dict):
                time.sleep(0.05)
                continue
            rid = str(res.get('cmd_id') or '')
            if rid and rid != cmd_id:
                time.sleep(0.05)
                continue
            if rid == cmd_id or (not rid and label == 'home'):
                ok = bool(res.get('success'))
                msg = str(res.get('message') or '')
                return ok, msg
            time.sleep(0.05)
        return False, f'{label} timeout'

    def _oms_home_all(self) -> Tuple[bool, str]:
        self._home_result = None
        self._home_status = None
        payload = {'mode': 'home'}
        self._log('OMS HOME — full sequence via /oms/home/cmd')
        deadline = time.time() + self._home_timeout
        last_pub = 0.0
        last_step = ''
        last_wait_log = time.time()
        while rclpy.ok() and time.time() < deadline:
            now = time.time()
            if now - last_pub >= 2.0:
                self._home_cmd_pub.publish(String(data=json.dumps(payload)))
                last_pub = now
            if now - last_wait_log >= 15.0:
                self._log('still waiting for OMS home result...', 'warn')
                last_wait_log = now
            st = self._home_status
            if isinstance(st, dict) and st.get('state') == 'homing':
                cur = str(st.get('current') or '')
                if cur and cur != last_step:
                    self._log(f'OMS HOME running: {cur}')
                    last_step = cur
            res = self._home_result
            if isinstance(res, dict) and res.get('message'):
                ok = bool(res.get('success'))
                msg = str(res.get('message') or '')
                if ok or 'fail' in msg.lower():
                    self._log(f'OMS HOME result ok={ok} {msg}')
                    return ok, msg
            time.sleep(0.1)
        return False, (
            'OMS home timeout — is oms_all running? '
            'Use: source install/setup.bash (no sudo on ros2 run)')

    def _oms_goto(self, angle_deg: float) -> Tuple[bool, str]:
        target = max(self._oms_min, min(self._oms_max, float(angle_deg)))
        max_tries = 3
        last_msg = ''
        for attempt in range(max_tries):
            cmd_id = f'daoms_{int(time.time() * 1000)}'
            self._oms_result = None
            payload = {'cmd_id': cmd_id, 'mode': 'goto', 'angle_deg': round(target, 3)}
            self._oms_cmd_pub.publish(String(data=json.dumps(payload)))
            self._log(
                f'OMS rotate goto {target:+.2f}° '
                f'(now {self._oms_now():+.1f}°) id={cmd_id} '
                f'try={attempt + 1}/{max_tries}')
            ok, msg = self._wait_cmd_result(
                cmd_id, lambda: self._oms_result, self._oms_timeout, 'rotation')
            tracked = self._oms_now()
            err = abs(tracked - target)
            if ok and msg == 'ok' and err <= self._oms_align_tol:
                self._commit_oms_position(tracked, self._oms_result)
                self._log(
                    f'OMS rotate ok tracked={tracked:+.1f}° err={err:.2f}° '
                    f'(tol {self._oms_align_tol:.1f}°)')
                return True, msg
            last_msg = msg or 'timeout'
            self._log(
                f'OMS rotate miss try={attempt + 1} ok={ok} msg={last_msg} '
                f'tracked={tracked:+.1f}° target={target:+.1f}° err={err:.1f}° '
                f'(need ≤{self._oms_align_tol:.1f}°)',
                'warn')
            time.sleep(0.25)
        return False, last_msg

    def _axis_cmd(
            self, pub, axis: str, mode: str, timeout: float,
            extra: Optional[dict] = None) -> Tuple[bool, str]:
        cmd_id = f'da_{axis}_{int(time.time() * 1000)}'
        self._axis_results.pop(axis, None)
        payload = {'cmd_id': cmd_id, 'mode': mode}
        if extra:
            payload.update(extra)
        pub.publish(String(data=json.dumps(payload)))
        self._log(f'{axis.upper()} mode={mode} id={cmd_id}')
        ok, msg = self._wait_cmd_result(
            cmd_id, lambda: self._axis_results.get(axis), timeout, axis)
        self._log(f'{axis.upper()} ok={ok} {msg}')
        return ok, msg

    def _lift_goto(self, pos_mm: float) -> Tuple[bool, str]:
        pos_mm = max(0.0, min(300.0, float(pos_mm)))
        return self._axis_cmd(
            self._lift_cmd_pub, 'lift', 'goto', self._lift_timeout,
            {'pos_mm': pos_mm})

    def _linear_goto_mm(self, pos_mm: float) -> Tuple[bool, str]:
        pos_mm = max(0.0, min(self._linear_max_mm, float(pos_mm)))
        return self._axis_cmd(
            self._linear_cmd_pub, 'linear', 'goto', self._linear_timeout,
            {'pos_mm': pos_mm})

    def _depth_to_linear_mm(self, z_depth_m: float) -> float:
        """linear mm = depth_z (cam→object) − cam→gripper + fwd_offset, clamped [0, linear_max]."""
        need_mm = (
            float(z_depth_m) * 1000.0
            - self._cam_grip_mm
            + self._linear_off_mm)
        return max(0.0, min(self._linear_max_mm, need_mm))

    def _call_toggle(
            self, label: str, clients, wanted: bool, wait_s: float = 8.0) -> bool:
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
            ok = _ok_toggle(res, wanted)
            self._log(
                f'camera {label} → {"ON" if wanted else "OFF"} '
                f'via {name} ok={ok}')
            return ok
        self._log(f'{label} toggle service not ready', 'warn')
        return False

    def _streams_off(self) -> None:
        self._call_toggle('color', self._toggle_color_clis, False, wait_s=3.0)
        self._call_toggle('depth', self._toggle_depth_clis, False, wait_s=3.0)

    def _camera_service_ready(self) -> bool:
        for cli in self._toggle_color_clis:
            if cli.service_is_ready():
                return True
        return False

    def _wait_stack_ready(self, timeout: float = 45.0) -> Tuple[bool, str]:
        """Wait for camera toggle + give DDS time to see oms_all (main thread spins)."""
        self._log(f'waiting for stack (camera service, up to {timeout:.0f}s)...')
        t0 = time.time()
        last_log = t0
        while rclpy.ok() and (time.time() - t0) < timeout:
            if self._camera_service_ready():
                self._log('camera toggle service ready')
                return True, 'ready'
            now = time.time()
            if now - last_log >= 10.0:
                self._log(
                    f'still waiting for /camera/toggle_color '
                    f'({now - t0:.0f}s) — is launch running? same ROS_DOMAIN_ID? no sudo?',
                    'warn')
                last_log = now
            time.sleep(0.2)
        return False, (
            'camera service not found — launch test_detect_align_oms first, '
            'source install/setup.bash, do not use sudo on ros2 run')

    def _wait_color_frames(self, need: int, timeout: float) -> bool:
        start = self._color_n
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < timeout:
            if self._color_n - start >= need and self._color is not None:
                return True
            time.sleep(0.05)
        return False

    def _grab_snapshot(self, skip: int) -> Optional[Image]:
        need = self._color_n + max(0, skip)
        deadline = time.time() + self._stream_wait
        while rclpy.ok() and time.time() < deadline:
            if self._color is not None and self._color_n >= need:
                return self._color
            time.sleep(0.05)
        return None

    def _ensure_detector(self):
        if self._detector is not None:
            return self._detector
        from skill23_studio.nav.yolov8_ncnn import YoloV8Ncnn
        self._log(f'loading NCNN model from {self._model_dir}')
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
        if det is not None:
            try:
                det.close()
            except Exception:
                pass

    def _detect(self, snap: Image):
        rgb = _image_to_rgb(snap)
        if rgb is None:
            return None, None, None, f'bad encoding {snap.encoding}'
        det = self._ensure_detector()
        dets = det.detect(rgb)
        h, w = rgb.shape[:2]
        min_area = self._min_area_frac * float(w * h)
        obj = det.pick(
            dets,
            class_id=self._target_class,
            class_name=self._target_name,
            min_area=min_area,
        )
        if obj is None:
            note = 'no target detected'
            if dets:
                top = sorted(dets, key=lambda d: d.conf, reverse=True)[:5]
                note = 'filter miss — top: ' + ', '.join(
                    f'{d.name}({float(d.conf) * 100:.0f}%)' for d in top)
            else:
                note = (
                    f'YOLO zero boxes conf>'
                    f'{self.get_parameter("conf_threshold").value}')
            return None, (w, h), rgb, note
        return obj, (w, h), rgb, ''

    def _read_depth_z(self, u_norm: float, v_norm: float) -> Optional[float]:
        samples: List[float] = []
        deadline = time.time() + self._depth_timeout
        while rclpy.ok() and time.time() < deadline:
            msg = self._depth
            if msg is None or time.time() - self._depth_stamp > 1.0:
                time.sleep(0.05)
                continue
            depth = _depth_to_meters(msg)
            if depth is None:
                time.sleep(0.05)
                continue
            val = _sample_depth(depth, u_norm, v_norm, self._depth_roi)
            if val is None:
                time.sleep(0.05)
                continue
            samples.append(val)
            if len(samples) >= self._depth_samples:
                break
            time.sleep(0.05)
        if not samples:
            return None
        return float(np.median(np.array(samples, dtype=np.float32)))

    def _publish_debug(
            self, rgb: np.ndarray, stamp, obj, u_err: float,
            bearing: float, oms_now: float, oms_target: float,
            lateral_px: float, geom: Optional[TargetGeometry],
            linear_mm: Optional[float], label: str) -> None:
        img = rgb.copy()
        h, w = img.shape[:2]
        cx_line = w // 2
        cv2.line(img, (cx_line, 0), (cx_line, h - 1), (0, 255, 255), 2)
        if obj is not None:
            x1, y1, x2, y2 = int(obj.x1), int(obj.y1), int(obj.x2), int(obj.y2)
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            oc = int(0.5 * (obj.x1 + obj.x2))
            oc_y = int(0.5 * (obj.y1 + obj.y2))
            cv2.circle(img, (oc, oc_y), 5, (255, 0, 0), -1)
            cv2.line(img, (cx_line, oc_y), (oc, oc_y), (255, 255, 0), 2)
            conf_pct = float(getattr(obj, 'conf', 0.0)) * 100.0
            cv2.putText(
                img, f'{obj.name} {conf_pct:.0f}%', (x1, max(18, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
        lines = [
            label,
            f'yaw={bearing:+.1f}° {yaw_direction(bearing)}  '
            f'lateral={lateral_px:+.0f}px  u_err={u_err:+.3f}',
            f'oms={oms_now:+.1f}° → tgt={oms_target:+.1f}°',
        ]
        if geom is not None:
            lines.append(
                f'depth Z={geom.z_depth_m:.3f}m  fwd={geom.forward_m:.3f}m  '
                f'lat={geom.lateral_m:+.3f}m  floor={geom.floor_dist_m:.3f}m')
        if linear_mm is not None:
            lines.append(f'linear cmd {linear_mm:.1f} mm')
        for i, text in enumerate(lines):
            cv2.putText(img, text, (8, 22 + i * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(img, text, (8, 22 + i * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 128, 255), 1, cv2.LINE_AA)
        try:
            self._debug_pub.publish(_rgb_to_bgr_msg(img, stamp))
            self._log(f'debug image → {self._debug_topic}  ({label})')
        except Exception as exc:
            self._log(f'debug publish failed: {exc}', 'warn')

    def _detect_from_frames(
            self,
    ) -> Tuple[bool, str, dict, Optional[np.ndarray], Optional[Image], Optional[object]]:
        """Color ON → verify → YOLO on need_frames snapshots → best conf → color OFF."""
        if not self._call_toggle('color', self._toggle_color_clis, True):
            return False, 'color toggle failed', {}, None, None, None
        if not self._wait_color_frames(1, self._stream_wait):
            self._call_toggle('color', self._toggle_color_clis, False)
            return False, 'no color frames after toggle ON', {}, None, None, None

        self._log(f'verifying stream — need {self._need_frames} detection frames')
        t0 = time.time()
        best_obj = None
        best_snap = None
        best_rgb = None
        best_extra: dict = {}
        frames_tried = 0

        while rclpy.ok() and frames_tried < self._need_frames:
            if time.time() - t0 >= self._detect_wait:
                break
            snap = self._grab_snapshot(self._skip_frames)
            if snap is None:
                break
            frames_tried += 1
            obj, size, rgb, note = self._detect(snap)
            if obj is None:
                self._log(f'frame {frames_tried}/{self._need_frames}: {note}', 'warn')
                continue
            w, h = size
            u_px, v_px = bbox_center_px(obj)
            color_intr = self._cam_info.color_or_fallback(w, h)
            u_err = u_norm_from_pixel(u_px, w, self._u_off)
            bearing = bearing_deg_from_pixel(
                u_px, color_intr, self._asign, self._oms_min, self._oms_max)
            oms_now = self._oms_now()
            target = self._oms_target_deg(bearing, oms_now)
            lateral_px = u_px - color_intr.cx
            conf_pct = float(obj.conf) * 100.0
            self._log(
                f'frame {frames_tried}/{self._need_frames}: {obj.name} '
                f'{conf_pct:.0f}%  yaw={bearing:+.1f}°  lateral={lateral_px:+.0f}px')
            if best_obj is None or float(obj.conf) > float(best_obj.conf):
                best_obj = obj
                best_snap = snap
                best_rgb = rgb
                best_extra = {
                    'bbox': [round(obj.x1, 1), round(obj.y1, 1),
                             round(obj.x2, 1), round(obj.y2, 1)],
                    'conf': round(float(obj.conf), 3),
                    'conf_pct': round(conf_pct, 1),
                    'class_id': int(obj.class_id),
                    'class_name': obj.name,
                    'size': [w, h],
                    'u_px': round(u_px, 1),
                    'v_px': round(v_px, 1),
                    'u_norm': round(u_err, 4),
                    'lateral_px': round(lateral_px, 1),
                    'bearing_deg': round(bearing, 2),
                    'yaw_to_center_deg': round(bearing, 2),
                    'oms_angle_deg': round(oms_now, 2),
                    'target_oms_deg': round(target, 2),
                }

        self._call_toggle('color', self._toggle_color_clis, False)

        if best_obj is None or best_snap is None or best_rgb is None:
            return False, 'no detection in any frame', best_extra, None, None, None

        self._log(
            f'BEST {best_extra["class_name"]} {best_extra["conf_pct"]:.0f}%  '
            f'yaw={best_extra["bearing_deg"]:+.1f}°  '
            f'target_oms={best_extra["target_oms_deg"]:+.1f}°')
        return True, 'detected', best_extra, best_rgb, best_snap, best_obj

    def _detect_and_measure_depth(
            self, extra: dict,
    ) -> Tuple[bool, str, dict, Optional[np.ndarray], Optional[Image],
               Optional[object], Optional[TargetGeometry], Optional[float]]:
        """After OMS yaw — color+depth ON, YOLO, depth sample, streams OFF."""
        self._log('POST-ROTATE — color+depth ON, detect and measure depth')
        if not self._call_toggle('color', self._toggle_color_clis, True):
            return False, 'color toggle failed', extra, None, None, None, None, None
        if not self._call_toggle('depth', self._toggle_depth_clis, True):
            self._call_toggle('color', self._toggle_color_clis, False)
            return False, 'depth toggle failed', extra, None, None, None, None, None
        if not self._wait_color_frames(1, self._stream_wait):
            self._streams_off()
            return False, 'no color frames after rotate', extra, None, None, None, None, None
        if self._depth_after_align > 0.0:
            time.sleep(self._depth_after_align)

        t0 = time.time()
        best_obj = None
        best_snap = None
        best_rgb = None
        best_extra = dict(extra)
        frames_tried = 0

        while rclpy.ok() and frames_tried < self._need_frames:
            if time.time() - t0 >= self._detect_wait:
                break
            snap = self._grab_snapshot(self._skip_frames)
            if snap is None:
                break
            frames_tried += 1
            obj, size, rgb, note = self._detect(snap)
            if obj is None:
                self._log(f'post-rotate frame {frames_tried}: {note}', 'warn')
                continue
            w, h = size
            u_px, v_px = bbox_center_px(obj)
            color_intr = self._cam_info.color_or_fallback(w, h)
            u_err = u_norm_from_pixel(u_px, w, self._u_off)
            bearing = bearing_deg_from_pixel(
                u_px, color_intr, self._asign, self._oms_min, self._oms_max)
            lateral_px = u_px - color_intr.cx
            conf_pct = float(obj.conf) * 100.0
            self._log(
                f'post-rotate frame {frames_tried}: {obj.name} {conf_pct:.0f}%  '
                f'yaw={bearing:+.1f}°  lateral={lateral_px:+.0f}px')
            if best_obj is None or float(obj.conf) > float(best_obj.conf):
                best_obj = obj
                best_snap = snap
                best_rgb = rgb
                best_extra.update({
                    'bbox': [round(obj.x1, 1), round(obj.y1, 1),
                             round(obj.x2, 1), round(obj.y2, 1)],
                    'conf': round(float(obj.conf), 3),
                    'conf_pct': round(conf_pct, 1),
                    'class_id': int(obj.class_id),
                    'class_name': obj.name,
                    'size': [w, h],
                    'u_px': round(u_px, 1),
                    'v_px': round(v_px, 1),
                    'u_norm': round(u_err, 4),
                    'lateral_px': round(lateral_px, 1),
                    'bearing_deg': round(bearing, 2),
                    'yaw_to_center_deg': round(bearing, 2),
                })

        if best_obj is None or best_snap is None or best_rgb is None:
            self._streams_off()
            return False, 'no detection after rotate', best_extra, None, None, None, None, None

        w, h = best_extra['size']
        u_px = float(best_extra['u_px'])
        v_px = float(best_extra['v_px'])
        u_norm = u_px / max(1.0, float(w))
        v_norm = v_px / max(1.0, float(h))
        z_depth = self._read_depth_z(u_norm, v_norm)
        self._streams_off()

        if z_depth is None:
            return False, 'no valid depth after rotate', best_extra, best_rgb, best_snap, best_obj, None, None

        linear_mm = self._depth_to_linear_mm(z_depth)
        oms_now = self._oms_now()
        target = float(best_extra.get('target_oms_deg') or oms_now)
        best_extra.update({
            'depth_z_m': round(z_depth, 4),
            'linear_fwd_offset_mm': round(self._linear_off_mm, 2),
            'linear_pos_mm': round(linear_mm, 2),
            'oms_angle_deg': round(oms_now, 2),
        })
        self._log(
            f'DEPTH Z={z_depth:.3f}m (cam→object)  '
            f'cam→grip={self._cam_grip_mm:.0f}mm  '
            f'fwd_offset=+{self._linear_off_mm:.0f}mm  '
            f'linear cmd={linear_mm:.1f}mm (max {self._linear_max_mm:.0f}mm)')
        self._publish_debug(
            best_rgb, best_snap.header.stamp, best_obj,
            float(best_extra['u_norm']), float(best_extra['bearing_deg']),
            oms_now, target, float(best_extra['lateral_px']),
            None, linear_mm,
            f'DEPTH Z={z_depth:.3f}m  linear→{linear_mm:.0f}mm')
        return True, 'depth measured', best_extra, best_rgb, best_snap, best_obj, None, linear_mm

    def _pick_arm_servos(self) -> Tuple[bool, str]:
        """Rotation servo in, then vertical up + gripper open."""
        self._log('ARM PREP — rotation_servo in, vertical up, gripper open')
        steps = (
            (self._rot_servo_pub, 'rotation_servo', 'in'),
            (self._vert_pub, 'vertical', 'up'),
            (self._grip_pub, 'gripper', 'open'),
        )
        for pub, axis, mode in steps:
            ok, msg = self._axis_cmd(pub, axis, mode, self._servo_timeout)
            if not ok:
                return False, f'{axis} {mode} failed: {msg}'
            time.sleep(0.15)
        return True, 'arm prep done'

    def _pick_execute(self, extra: dict) -> Tuple[bool, str, dict]:
        linear_mm = extra.get('linear_pos_mm')
        if linear_mm is None:
            return False, 'linear_pos_mm missing — run depth measure first', extra

        self._log(f'LIFT goto {self._pick_lift_mm:.0f} mm down from home')
        ok, msg = self._lift_goto(self._pick_lift_mm)
        if not ok:
            return False, f'lift failed: {msg}', extra

        if self._linear_cmd_enabled:
            ok, msg = self._linear_goto_mm(float(linear_mm))
            if not ok:
                return False, f'linear failed: {msg}', extra
        else:
            self._log(f'linear_cmd_enabled=false — would goto {float(linear_mm):.1f} mm')

        self._log('GRIPPER close')
        ok, msg = self._axis_cmd(
            self._grip_pub, 'gripper', 'close', self._servo_timeout)
        if not ok:
            ok2, msg2 = self._axis_cmd(
                self._grip_pub, 'gripper', 'home', self._servo_timeout)
            if not ok2:
                return False, f'gripper close failed: {msg} / {msg2}', extra

        if self._linear_cmd_enabled:
            self._log('LINEAR home — retract after grip')
            ok, msg = self._axis_cmd(
                self._linear_cmd_pub, 'linear', 'home', self._linear_timeout)
            if not ok:
                return False, f'linear home failed: {msg}', extra

        extra['pick_ok'] = True
        return True, 'pick complete', extra

    def _post_pick_view(self, extra: dict) -> None:
        try:
            self._log('POST-PICK — color ON, publish debug with centerline')
            if not self._call_toggle('color', self._toggle_color_clis, True):
                self._log('post-pick color toggle failed', 'warn')
                return
            if not self._wait_color_frames(self._post_pick_frames, self._stream_wait):
                self._log('post-pick: no frames', 'warn')
                return
            snap = self._grab_snapshot(self._skip_frames)
            if snap is None:
                return
            obj, size, rgb, note = self._detect(snap)
            if obj is None:
                self._log(f'post-pick detect: {note} — overlay stored values', 'warn')
                obj = type('O', (), {
                    'x1': extra['bbox'][0], 'y1': extra['bbox'][1],
                    'x2': extra['bbox'][2], 'y2': extra['bbox'][3],
                    'name': extra.get('class_name', '?'),
                    'conf': extra.get('conf', 0.0),
                })()
                if rgb is None:
                    converted = _image_to_rgb(snap)
                    if converted is not None:
                        rgb = converted
            if rgb is None:
                self._log('post-pick: no rgb frame', 'warn')
                return
            bearing = float(extra.get('bearing_deg') or 0.0)
            oms_now = self._oms_now()
            target = float(extra.get('target_oms_deg') or oms_now)
            u_err = float(extra.get('u_norm') or 0.0)
            lateral_px = float(extra.get('lateral_px') or 0.0)
            linear_mm = extra.get('linear_pos_mm')
            self._publish_debug(
                rgb, snap.header.stamp, obj, u_err, bearing, oms_now, target,
                lateral_px, None, linear_mm,
                f'POST-PICK {extra.get("class_name", "?")} '
                f'{extra.get("conf_pct", 0):.0f}%')
        except Exception as exc:
            self._log(f'post-pick view failed: {exc}', 'error')
        finally:
            self._call_toggle('color', self._toggle_color_clis, False)

    def _run_once(self) -> None:
        self._busy = True
        try:
            self._log('=== START OMS pick run ===')
            ok, msg = self._wait_stack_ready()
            if not ok:
                self._log(msg, 'error')
                return
            self._color_n = 0
            self._depth = None
            self._streams_off()

            if self._home_on_start:
                ok, msg = self._oms_home_all()
                if not ok:
                    self._log(f'OMS home failed: {msg}', 'error')
                    return

            self._ensure_detector()
            ok, msg, extra, rgb, snap, obj = self._detect_from_frames()
            if not ok or snap is None or rgb is None or obj is None:
                self._log(msg or 'detection failed', 'error')
                return

            bearing = float(extra['bearing_deg'])
            oms_now = float(extra['oms_angle_deg'])
            target = float(extra['target_oms_deg'])
            u_err = float(extra['u_norm'])
            lateral_px = float(extra['lateral_px'])
            self._publish_debug(
                rgb, snap.header.stamp, obj, u_err, bearing, oms_now, target,
                lateral_px, None, None,
                f'DETECT {extra["class_name"]} {extra["conf_pct"]:.0f}%')

            if not self._pick_mode:
                self._log(f'DONE (detect only) {json.dumps(extra)}')
                self._drop_detector()
                return

            target = float(extra['target_oms_deg'])
            self._log(f'PICK — OMS rotate to {target:+.1f}°')
            ok, msg = self._oms_goto(target)
            if not ok:
                self._log(f'rotation failed: {msg}', 'error')
                return

            for refine in range(self._align_refine_max + 1):
                ok, msg, extra, _rgb2, _snap2, _obj2, _geom, _lin = (
                    self._detect_and_measure_depth(extra))
                if not ok:
                    self._log(msg, 'error')
                    return
                bearing = float(extra.get('bearing_deg', 0.0))
                if abs(bearing) <= self._align_yaw_tol:
                    if refine > 0:
                        self._log(
                            f'ALIGN ok after refine {refine}: '
                            f'bearing={bearing:+.2f}°')
                    break
                if refine >= self._align_refine_max:
                    self._log(
                        f'ALIGN warn: bearing={bearing:+.2f}° still > '
                        f'{self._align_yaw_tol:.1f}° after {refine + 1} measure(s)',
                        'warn')
                    break
                fix_target = self._oms_target_deg(bearing, self._oms_now())
                self._log(
                    f'ALIGN refine {refine + 1}: bearing={bearing:+.2f}° > '
                    f'{self._align_yaw_tol:.1f}° → rotate to {fix_target:+.1f}°')
                ok, msg = self._oms_goto(fix_target)
                if not ok:
                    self._log(f'align refine failed: {msg}', 'error')
                    return

            ok, msg = self._pick_arm_servos()
            if not ok:
                self._log(msg, 'error')
                return

            ok, msg, extra = self._pick_execute(extra)
            if not ok:
                self._log(msg, 'error')
                return

            self._streams_off()
            self._log(f'PICK DONE {json.dumps(extra)}')
            self._post_pick_view(extra)
            self._drop_detector()
        finally:
            self._streams_off()
            self._drop_detector()
            self._busy = False

    def spin_loop(self) -> None:
        while rclpy.ok():
            if self._start_evt.wait(timeout=0.1):
                self._start_evt.clear()
                self._run_once()


def main(args=None) -> None:
    apply_studio_dds_env()
    rclpy.init(args=args)
    node = DetectAlignOmsTestNode()
    worker = threading.Thread(target=node.spin_loop, daemon=True)
    worker.start()
    try:
        while rclpy.ok() and worker.is_alive():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
