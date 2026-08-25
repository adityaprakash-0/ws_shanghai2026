#!/usr/bin/env python3
"""Laptop vision hub.

Idle until Pi publishes /studio/vision/request. Then:
  ACK started → process color (+ depth if *_depth) → align → approach → result.

Does not open the camera. Subscribes straight to the Orbbec driver topics on
the Pi (``/camera/...``); the Pi does not relay or re-encode frames. Frames only
flow while the Pi has the colour stream toggled on for that waypoint.
"""

from __future__ import annotations

import argparse
import json
import time
from typing import Any, Dict, Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from detectors import depth_util

DETECTORS = {
    'aruco': 'detectors.aruco',
    'color': 'detectors.color',
    'yolo': 'detectors.yolo_det',
    'ocr': 'detectors.ocr',
    'barcode': 'detectors.barcode',
    'qr': 'detectors.qr',
    'aruco_depth': 'detectors.aruco_depth',
    'color_depth': 'detectors.color_depth',
    'yolo_depth': 'detectors.yolo_depth',
    'ocr_depth': 'detectors.ocr_depth',
    'barcode_depth': 'detectors.barcode_depth',
    'qr_depth': 'detectors.qr_depth',
}

# Enabling a base detector also loads its depth variant.
ALIASES = {
    'aruco': ('aruco', 'aruco_depth'),
    'color': ('color', 'color_depth'),
    'yolo': ('yolo', 'yolo_depth'),
    'ocr': ('ocr', 'ocr_depth'),
    'barcode': ('barcode', 'barcode_depth'),
    'qr': ('qr', 'qr_depth'),
    'aruco_depth': ('aruco', 'aruco_depth'),
    'color_depth': ('color', 'color_depth'),
    'yolo_depth': ('yolo', 'yolo_depth'),
    'ocr_depth': ('ocr', 'ocr_depth'),
    'barcode_depth': ('barcode', 'barcode_depth'),
    'qr_depth': ('qr', 'qr_depth'),
}


# Orbbec driver topics (no Studio relay in between).
COLOR_TOPIC = '/camera/color/image_raw'
COLOR_COMPRESSED_TOPIC = '/camera/color/image_raw/compressed'
DEPTH_TOPIC = '/camera/depth/image_raw'


def _load(name: str):
    mod = __import__(DETECTORS[name], fromlist=['detect'])
    return mod.detect


def _color_to_bgr(msg: Image):
    """Orbbec colour Image → BGR frame (MJPG, RGB/BGR, YUYV or mono)."""
    enc = (msg.encoding or '').lower().replace('-', '').replace('_', '')
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ('jpeg', 'jpg', 'mjpeg', 'mjpg'):
        return cv2.imdecode(data, cv2.IMREAD_COLOR)
    h, w = int(msg.height), int(msg.width)
    if h <= 0 or w <= 0 or data.size == 0:
        return None
    try:
        if enc in ('bgr8', '8uc3'):
            return data.reshape((h, w, 3))
        if enc == 'rgb8':
            return cv2.cvtColor(data.reshape((h, w, 3)), cv2.COLOR_RGB2BGR)
        if enc == 'bgra8':
            return cv2.cvtColor(data.reshape((h, w, 4)), cv2.COLOR_BGRA2BGR)
        if enc == 'rgba8':
            return cv2.cvtColor(data.reshape((h, w, 4)), cv2.COLOR_RGBA2BGR)
        if enc in ('mono8', '8uc1'):
            return cv2.cvtColor(data.reshape((h, w)), cv2.COLOR_GRAY2BGR)
        if enc in ('yuv422', 'yuyv', 'yuy2', 'yuyv422'):
            return cv2.cvtColor(data.reshape((h, w, 2)), cv2.COLOR_YUV2BGR_YUY2)
    except ValueError:
        return None
    return None


def _image_to_array(msg: Image):
    h, w = msg.height, msg.width
    enc = (msg.encoding or '').lower()
    if enc in ('16uc1', 'mono16'):
        return np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
    if enc in ('32fc1',):
        return np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
    if enc in ('8uc1', 'mono8'):
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
    try:
        return np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
    except Exception:
        return None


class VisionHub(Node):
    def __init__(self, enabled: list) -> None:
        super().__init__('studio_vision_hub')
        names = set()
        for raw in enabled:
            names.update(ALIASES.get(raw, (raw,)))
        self._fns = {}
        for name in names:
            if name not in DETECTORS:
                self.get_logger().warn(f'unknown detector {name}')
                continue
            try:
                self._fns[name] = _load(name)
                self.get_logger().info(f'loaded {name} (idle)')
            except Exception as exc:
                self.get_logger().error(f'{name} import failed: {exc}')

        self._job: Optional[Dict[str, Any]] = None
        self._frame = None
        self._depth = None
        self._last_ok: Optional[Dict[str, Any]] = None
        self._ok_streak = 0
        self._subs: list = []
        self._depth_sub = None
        self._frame_src = ''

        self._ack = self.create_publisher(String, '/studio/vision/ack', 10)
        self._result = self.create_publisher(String, '/studio/vision/result', 10)
        self._track = self.create_publisher(String, '/studio/vision/track', 10)
        self._cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/studio/vision/request', self._on_req, 10)
        self.create_subscription(String, '/studio/vision/done', self._on_done, 10)
        self.create_timer(0.08, self._tick)
        self.get_logger().info(
            f'Vision hub ready. Enabled: {sorted(self._fns)} — waiting for Pi')

    def _on_req(self, msg: String) -> None:
        try:
            req = json.loads(msg.data)
        except Exception:
            return
        kind = str(req.get('type') or '').lower()
        rid = str(req.get('id') or '')
        if kind not in self._fns:
            ack = String()
            ack.data = json.dumps({
                'id': rid, 'status': 'error',
                'message': f'{kind} not launched on laptop',
            })
            self._ack.publish(ack)
            out = String()
            out.data = json.dumps({
                'id': rid, 'ok': False,
                'message': f'{kind} not launched — start.sh that detector',
            })
            self._result.publish(out)
            return
        params = req.get('params') or {}
        use_depth = kind.endswith('_depth') or float(params.get('approach_cm') or 0) > 0 \
            or float(params.get('stop_at_cm') or 0) > 0
        self._job = {
            'id': rid,
            'type': kind,
            'params': params,
            'timeout': max(3.0, float(req.get('timeout_sec', 20.0))),
            't0': time.time(),
            'align': bool(params.get('align', True)),
            'use_depth': use_depth,
            'phase': 'align',
            'target_m': None,
            'hold': 0.0,
        }
        self._last_ok = None
        self._ok_streak = 0
        self._depth = None
        self._frame_src = ''
        # Subscribe only for the duration of the job, and take whichever of the
        # two colour topics the driver actually publishes.
        if not self._subs:
            self._subs.append(self.create_subscription(
                CompressedImage, COLOR_COMPRESSED_TOPIC,
                self._on_compressed, qos_profile_sensor_data))
            self._subs.append(self.create_subscription(
                Image, COLOR_TOPIC, self._on_raw, qos_profile_sensor_data))
        if use_depth and self._depth_sub is None:
            self._depth_sub = self.create_subscription(
                Image, DEPTH_TOPIC, self._on_depth, qos_profile_sensor_data)
        ack = String()
        ack.data = json.dumps({'id': rid, 'status': 'started', 'type': kind})
        self._ack.publish(ack)
        self.get_logger().info(f'ACK started {kind} id={rid} depth={use_depth}')

    def _on_done(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            data = {}
        if self._job and str(data.get('id', '')) in ('', self._job['id']):
            self._stop_job('pi done')

    def _set_frame(self, frame, src: str) -> None:
        if frame is None:
            return
        self._frame = frame
        if self._frame_src != src:
            self._frame_src = src
            self.get_logger().info(f'frames from {src}')

    def _on_compressed(self, msg: CompressedImage) -> None:
        if self._job is None:
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        self._set_frame(cv2.imdecode(arr, cv2.IMREAD_COLOR), COLOR_COMPRESSED_TOPIC)

    def _on_raw(self, msg: Image) -> None:
        # Skip decode work when the compressed topic is already feeding us.
        if self._job is None or self._frame_src == COLOR_COMPRESSED_TOPIC:
            return
        self._set_frame(_color_to_bgr(msg), COLOR_TOPIC)

    def _on_depth(self, msg: Image) -> None:
        if self._job is None:
            return
        self._depth = _image_to_array(msg)

    def _stop_job(self, why: str) -> None:
        self.get_logger().info(f'stop job: {why}')
        self._job = None
        self._frame = None
        self._depth = None
        self._frame_src = ''
        self._cmd.publish(Twist())
        for sub in self._subs:
            self.destroy_subscription(sub)
        self._subs = []
        if self._depth_sub is not None:
            self.destroy_subscription(self._depth_sub)
            self._depth_sub = None

    def _finish_ok(self, job: dict, res: dict) -> None:
        payload = dict(res)
        payload['id'] = job['id']
        payload['ok'] = True
        out = String()
        out.data = json.dumps(payload)
        self._result.publish(out)
        self.get_logger().info(payload.get('message', 'result'))
        self._stop_job('result sent')

    def _tick(self) -> None:
        job = self._job
        if job is None:
            return
        if time.time() - job['t0'] > job['timeout']:
            out = String()
            out.data = json.dumps({
                'id': job['id'], 'ok': False, 'message': 'laptop timeout',
            })
            self._result.publish(out)
            self._stop_job('timeout')
            return
        if self._frame is None:
            return
        fn = self._fns.get(job['type'])
        if fn is None:
            return
        try:
            if job['use_depth']:
                res = fn(self._frame, job['params'], depth=self._depth)
            else:
                res = fn(self._frame, job['params'])
        except TypeError:
            res = fn(self._frame, job['params'])
        except Exception as exc:
            res = {'ok': False, 'message': str(exc)}

        if res.get('ok'):
            self._last_ok = res
            track = String()
            track.data = json.dumps({
                'id': job['id'],
                'type': job['type'],
                'phase': job['phase'],
                'depth_cm': res.get('depth_cm'),
                'target_m': job.get('target_m'),
                'offset_x': res.get('offset_x'),
                'message': res.get('message'),
            })
            self._track.publish(track)

        if job['align'] and res.get('ok') and 'offset_x' in res and job['phase'] == 'align':
            ox = float(res['offset_x'])
            tw = Twist()
            if abs(ox) > 0.12:
                tw.angular.z = max(-0.35, min(0.35, -0.6 * ox))
                self._cmd.publish(tw)
                job['hold'] = 0.0
                return
            job['hold'] += 0.08
            self._cmd.publish(Twist())
            if job['hold'] < 0.35:
                return
            if job['use_depth']:
                job['phase'] = 'approach'
                job['hold'] = 0.0
                self.get_logger().info('aligned — starting depth approach')
                return
            self._finish_ok(job, res)
            return

        if job['use_depth'] and job['phase'] in ('align', 'approach') and res.get('ok'):
            job['phase'] = 'approach'
            d = res.get('depth_m')
            if d is None:
                return
            if job['target_m'] is None:
                job['target_m'] = depth_util.target_m(float(d), job['params'])
                self.get_logger().info(
                    f'depth {d*100:.0f}cm → stop at {job["target_m"]*100:.0f}cm')
            remain = float(d) - float(job['target_m'])
            tw = Twist()
            if 'offset_x' in res:
                tw.angular.z = max(-0.25, min(0.25, -0.4 * float(res['offset_x'])))
            if remain <= 0.015:
                job['hold'] += 0.08
                self._cmd.publish(Twist())
                if job['hold'] >= 0.3:
                    res['message'] = (
                        f"{res.get('message', '')} stopped at {d*100:.0f}cm "
                        f"(target {job['target_m']*100:.0f}cm)").strip()
                    self._finish_ok(job, res)
                return
            job['hold'] = 0.0
            tw.linear.x = max(0.04, min(0.16, 0.7 * remain))
            self._cmd.publish(tw)
            return

        if res.get('ok'):
            self._ok_streak += 1
        else:
            self._ok_streak = 0
        if self._last_ok and self._ok_streak >= 2 and not job['use_depth']:
            self._finish_ok(job, self._last_ok)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--enable', default='aruco,color',
        help='comma list: aruco,yolo,ocr,barcode,qr,color (depth variants included)')
    args, ros_args = parser.parse_known_args()
    enabled = [x.strip().lower() for x in args.enable.split(',') if x.strip()]
    rclpy.init(args=ros_args)
    node = VisionHub(enabled)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
