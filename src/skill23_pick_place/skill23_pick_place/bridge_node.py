#!/usr/bin/env python3
"""Pick-place HTTP/SSE bridge on port 8081 (separate from ADEV 8080)."""

from __future__ import annotations

import base64
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path as FilePath
from typing import Any, Dict, Optional
from urllib.parse import parse_qs, urlparse

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Empty, Float32MultiArray, String

try:
    import yaml
except ImportError:
    yaml = None

try:
    from cv_bridge import CvBridge
    import cv2
    _HAS_CV = True
except ImportError:
    CvBridge = None
    cv2 = None
    _HAS_CV = False


def yaw_of(msg: Odometry) -> float:
    import math
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


class PpBridgeNode(Node):
    def __init__(self):
        super().__init__('pp_bridge')
        self.declare_parameter('robot', 'mecanum')
        self.declare_parameter('http_port', 8081)
        self.declare_parameter('maps_dir', os.path.expanduser('~/skill23_ws'))
        self.declare_parameter('relay_cmd_vel', True)

        self.robot = self.get_parameter('robot').value
        self.http_port = int(self.get_parameter('http_port').value)
        self.maps_dir = self.get_parameter('maps_dir').value

        self._estop = False
        self._mission_status: Dict[str, Any] = {}
        self._odom: Optional[Odometry] = None
        self._ranges = [0.0, 0.0, 0.0, 0.0]
        self._arm: Dict[str, Any] = {}
        self._preview_jpeg: Optional[bytes] = None
        self._cv = CvBridge() if _HAS_CV else None

        self.create_subscription(Odometry, '/pp/odom', self._on_odom, 10)
        self.create_subscription(Float32MultiArray, '/pp/dis_data', self._on_ranges, 10)
        self.create_subscription(String, '/pp/mission/status', self._on_mission, 10)
        self.create_subscription(String, '/pp/manipulator/status', self._on_arm, 10)
        self.create_subscription(
            CompressedImage, '/camera/color/image_raw/compressed',
            self._on_compressed, 10)
        if _HAS_CV:
            self.create_subscription(Image, '/adev/vision/preview', self._on_preview, 10)

        self._mission_cmd = self.create_publisher(String, '/pp/mission/cmd', 10)
        self._arm_cmd = self.create_publisher(String, '/pp/manipulator/cmd', 10)
        self._align_cobra = self.create_publisher(String, '/pp/align/cobra', 10)
        self._align_cam = self.create_publisher(String, '/pp/align/camera', 10)
        self._goto = self.create_publisher(String, '/pp/nav/goto', 10)
        self._rel = self.create_publisher(String, '/pp/nav/relative', 10)
        self._odom_reset = self.create_publisher(Empty, '/pp/odom/reset', 10)
        self._pp_cmd = self.create_publisher(Twist, '/pp/cmd_vel', 10)
        self._drive_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # Relay /pp/cmd_vel → /cmd_vel (absolute drivetrain topic)
        if bool(self.get_parameter('relay_cmd_vel').value):
            self.create_subscription(Twist, '/pp/cmd_vel', self._on_pp_cmd, 10)

        self._http_ready = threading.Event()
        threading.Thread(target=self._run_http, daemon=True).start()
        self.get_logger().info(
            f'PP bridge HTTP :{self.http_port} robot={self.robot}')

    def _on_odom(self, msg: Odometry):
        self._odom = msg

    def _on_ranges(self, msg: Float32MultiArray):
        self._ranges = list(msg.data)[:4]

    def _on_mission(self, msg: String):
        try:
            self._mission_status = json.loads(msg.data)
        except Exception:
            pass

    def _on_arm(self, msg: String):
        try:
            self._arm = json.loads(msg.data)
        except Exception:
            pass

    def _on_pp_cmd(self, msg: Twist):
        if self._estop:
            z = Twist()
            self._drive_cmd.publish(z)
            return
        self._drive_cmd.publish(msg)

    def _on_compressed(self, msg: CompressedImage):
        self._preview_jpeg = bytes(msg.data)

    def _on_preview(self, msg: Image):
        if self._cv is None or cv2 is None:
            return
        try:
            bgr = self._cv.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ok, buf = cv2.imencode('.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if ok:
                self._preview_jpeg = buf.tobytes()
        except Exception:
            pass

    def snapshot(self) -> dict:
        pose = None
        if self._odom is not None:
            p = self._odom.pose.pose.position
            pose = {
                'x': round(float(p.x), 4),
                'y': round(float(p.y), 4),
                'yaw': round(yaw_of(self._odom), 4),
            }
        return {
            'ok': True,
            'robot': self.robot,
            'estop': self._estop,
            'pose': pose,
            'ranges': self._ranges,
            'arm': self._arm,
            'mission': self._mission_status,
            'ts': time.time(),
        }

    def apply_estop(self, active: bool):
        self._estop = bool(active)
        if self._estop:
            z = Twist()
            self._pp_cmd.publish(z)
            self._drive_cmd.publish(z)
            # Also stop mission
            self._pub_mission({'action': 'stop'})

    def _pub_mission(self, payload: dict):
        msg = String()
        msg.data = json.dumps(payload)
        self._mission_cmd.publish(msg)

    def load_map_info(self, map_path: str) -> dict:
        if not yaml:
            raise RuntimeError('PyYAML not available on Pi')
        path = FilePath(map_path)
        if not path.is_absolute():
            path = FilePath(self.maps_dir) / path
        with open(path, 'r') as f:
            meta = yaml.safe_load(f)
        pgm_name = meta.get('image', '')
        pgm_path = path.parent / pgm_name
        if not pgm_path.exists():
            pgm_path = FilePath(self.maps_dir) / pgm_name
        with open(pgm_path, 'rb') as f:
            pgm_b64 = base64.b64encode(f.read()).decode('ascii')
        return {
            'yaml_path': str(path),
            'image': pgm_name,
            'resolution': float(meta.get('resolution', 0.05)),
            'origin': [float(v) for v in meta.get('origin', [0, 0, 0])],
            'occupied_thresh': float(meta.get('occupied_thresh', 0.65)),
            'free_thresh': float(meta.get('free_thresh', 0.25)),
            'negate': int(meta.get('negate', 0)),
            'mode': meta.get('mode', 'trinary'),
            'pgm_base64': pgm_b64,
        }

    def get_preview_jpeg(self) -> Optional[bytes]:
        return self._preview_jpeg

    def _run_http(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                pass

            def _cors(self):
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')

            def _json_response(self, code, payload):
                body = json.dumps(payload).encode('utf-8')
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self._cors()
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _read_json(self):
                length = int(self.headers.get('Content-Length', 0))
                if length == 0:
                    return {}
                return json.loads(self.rfile.read(length).decode('utf-8'))

            def do_OPTIONS(self):
                self.send_response(204)
                self._cors()
                self.end_headers()

            def do_GET(self):
                parsed = urlparse(self.path)
                path = parsed.path

                if path == '/api/status':
                    self._json_response(200, {
                        'ok': True,
                        'robot': node.robot,
                        'module': 'pick_place',
                        'http_port': node.http_port,
                        'estop': node._estop,
                    })
                    return

                if path == '/api/telemetry':
                    self._json_response(200, node.snapshot())
                    return

                if path == '/api/stream':
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/event-stream')
                    self.send_header('Cache-Control', 'no-cache')
                    self._cors()
                    self.end_headers()
                    try:
                        while rclpy.ok():
                            chunk = f'data: {json.dumps(node.snapshot())}\n\n'.encode('utf-8')
                            self.wfile.write(chunk)
                            self.wfile.flush()
                            time.sleep(0.2)
                    except (BrokenPipeError, ConnectionResetError):
                        return
                    return

                if path == '/api/maps':
                    maps = []
                    root = FilePath(node.maps_dir)
                    for yml in sorted(root.glob('*.yaml')):
                        maps.append({'name': yml.stem, 'path': str(yml)})
                    # Also search common map folders one level deep
                    for sub in root.glob('*/*.yaml'):
                        maps.append({'name': sub.stem, 'path': str(sub)})
                    self._json_response(200, {'maps': maps, 'robot': node.robot})
                    return

                if path == '/api/map':
                    qs = parse_qs(parsed.query)
                    map_path = qs.get('path', [''])[0]
                    if not map_path:
                        self._json_response(400, {'error': 'path query required'})
                        return
                    try:
                        self._json_response(200, node.load_map_info(map_path))
                    except Exception as exc:
                        self._json_response(500, {'error': str(exc)})
                    return

                if path == '/api/vision/preview':
                    jpeg = node.get_preview_jpeg()
                    if not jpeg:
                        self.send_response(204)
                        self._cors()
                        self.end_headers()
                        return
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Cache-Control', 'no-cache')
                    self._cors()
                    self.send_header('Content-Length', str(len(jpeg)))
                    self.end_headers()
                    self.wfile.write(jpeg)
                    return

                self._json_response(404, {
                    'error': 'PP API only — open pick_place_dashboard on laptop',
                    'hint': 'Run laptop/pick_place_dashboard/start.sh ; Pi port 8081',
                })

            def do_POST(self):
                parsed = urlparse(self.path)
                path = parsed.path
                body = self._read_json()

                if path == '/api/estop':
                    node.apply_estop(bool(body.get('active', True)))
                    self._json_response(200, {'estop': node._estop})
                    return

                if path == '/api/mission/task':
                    task = body.get('task', body)
                    node._pub_mission({'action': 'set_task', 'task': task})
                    steps = list((task or {}).get('steps') or [])
                    self._json_response(200, {
                        'ok': True,
                        'name': (task or {}).get('name', ''),
                        'steps': len(steps),
                    })
                    return

                if path == '/api/mission/start':
                    node._pub_mission({'action': 'start'})
                    self._json_response(200, {'ok': True})
                    return

                if path == '/api/mission/stop':
                    node._pub_mission({'action': 'stop'})
                    self._json_response(200, {'ok': True})
                    return

                if path == '/api/odom/reset':
                    node._odom_reset.publish(Empty())
                    self._json_response(200, {'ok': True})
                    return

                if path == '/api/nav/goto':
                    msg = String()
                    msg.data = json.dumps(body)
                    node._goto.publish(msg)
                    self._json_response(200, {'ok': True})
                    return

                if path == '/api/nav/relative':
                    msg = String()
                    msg.data = json.dumps(body)
                    node._rel.publish(msg)
                    self._json_response(200, {'ok': True})
                    return

                if path == '/api/align/cobra':
                    msg = String()
                    msg.data = json.dumps(body)
                    node._align_cobra.publish(msg)
                    self._json_response(200, {'ok': True})
                    return

                if path == '/api/align/camera':
                    msg = String()
                    msg.data = json.dumps(body)
                    node._align_cam.publish(msg)
                    self._json_response(200, {'ok': True})
                    return

                if path == '/api/manipulator/cmd':
                    msg = String()
                    msg.data = json.dumps(body)
                    node._arm_cmd.publish(msg)
                    self._json_response(200, {'ok': True})
                    return

                if path == '/api/robot/info':
                    self._json_response(200, {
                        'robot': node.robot,
                        'module': 'pick_place',
                        'launch_hint': (
                            '~/skill23_ws/src/studica_ws/scripts/ros2_pi.sh launch '
                            f'skill23_pick_place pick_place.launch.py robot:={node.robot}'),
                    })
                    return

                self._json_response(404, {'error': 'not found'})

        class ReuseHTTPServer(ThreadingHTTPServer):
            allow_reuse_address = True

        try:
            server = ReuseHTTPServer(('0.0.0.0', node.http_port), Handler)
        except OSError as exc:
            node.get_logger().error(
                f'Port {node.http_port} already in use ({exc}). '
                'Stop other pp_bridge instances: pkill -f pp_bridge')
            return

        node._http_ready.set()
        server.serve_forever()


def main(args=None):
    rclpy.init(args=args)
    node = PpBridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, Exception):
        pass
    try:
        node.destroy_node()
    except Exception:
        pass
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
