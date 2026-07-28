#!/usr/bin/env python3
"""Case-nav HTTP/SSE bridge on port 8082 (separate from ADEV 8080 / PP 8081)."""

from __future__ import annotations

import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path as FilePath
from typing import Any, Dict, Optional
from urllib.parse import parse_qs, urlparse

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import String


def yaw_of_pose(pose: PoseStamped) -> float:
    q = pose.pose.orientation
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


class CaseNavBridge(Node):
    def __init__(self):
        super().__init__('cn_bridge')
        self.declare_parameter('robot', 'mecanum')
        self.declare_parameter('http_port', 8082)
        self.declare_parameter('maps_dir', os.path.expanduser('~/skill23_ws'))
        self.declare_parameter('robot_length', 0.40)
        self.declare_parameter('robot_width', 0.40)
        self.declare_parameter('lidar_fov_deg', 90.0)
        self.declare_parameter('front_scan_topic', '/scan/front')
        self.declare_parameter('scan_topic', '/scan')

        self.robot = self.get_parameter('robot').value
        self.http_port = int(self.get_parameter('http_port').value)
        self.maps_dir = self.get_parameter('maps_dir').value
        self.robot_length = float(self.get_parameter('robot_length').value)
        self.robot_width = float(self.get_parameter('robot_width').value)
        self.lidar_half_fov = (
            float(self.get_parameter('lidar_fov_deg').value) * math.pi / 360.0)

        self._estop = False
        self._pose: Optional[PoseStamped] = None
        self._watch: Dict[str, Any] = {}
        self._fsm: Dict[str, Any] = {}
        self._plan: Dict[str, Any] = {}
        self._planner_status: Dict[str, Any] = {}
        self._lidar: Dict[str, Any] = {}
        self._map: Optional[OccupancyGrid] = None
        self._path: Optional[Path] = None
        # Dashboard green rays: FOV front continuously; full /scan when lidar awake
        self._scan_front_fov: list = []
        self._scan_full: list = []
        self._battery: Dict[str, Any] = {
            'voltage': 0.0, 'percentage': 0.0, 'valid': False, 'source': 'none',
        }
        self._battery_rx_count = 0

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        battery_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(PoseStamped, '/case_nav/pose', self._on_pose, 10)
        self.create_subscription(String, '/case_nav/watchdog', self._on_watch, 10)
        self.create_subscription(String, '/case_nav/fsm_state', self._on_fsm, 10)
        self.create_subscription(String, '/case_nav/plan', self._on_plan, 10)
        self.create_subscription(String, '/case_nav/planner_status', self._on_pstatus, 10)
        self.create_subscription(String, '/case_nav/lidar_status', self._on_lidar, 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, qos)
        self.create_subscription(Path, '/case_nav/path', self._on_path, 10)
        self.create_subscription(BatteryState, '/battery_state', self._on_battery, battery_qos)
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('front_scan_topic').value),
            self._on_front_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self._on_full_scan,
            qos_profile_sensor_data,
        )
        self.create_timer(5.0, self._check_battery)

        self._initialpose = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self._goal_pub = self.create_publisher(String, '/case_nav/goal', 10)
        self._cmd_pub = self.create_publisher(String, '/case_nav/cmd', 10)
        self._map_load = self.create_publisher(String, '/case_nav/map/load', 10)
        self._cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        threading.Thread(target=self._run_http, daemon=True).start()
        self.get_logger().info(
            f'Case-nav bridge HTTP :{self.http_port} robot={self.robot}')

    def _on_pose(self, msg: PoseStamped):
        self._pose = msg

    def _on_watch(self, msg: String):
        try:
            self._watch = json.loads(msg.data)
        except Exception:
            pass

    def _on_fsm(self, msg: String):
        try:
            self._fsm = json.loads(msg.data)
        except Exception:
            pass

    def _on_plan(self, msg: String):
        try:
            self._plan = json.loads(msg.data)
        except Exception:
            pass

    def _on_pstatus(self, msg: String):
        try:
            self._planner_status = json.loads(msg.data)
        except Exception:
            pass

    def _on_lidar(self, msg: String):
        try:
            self._lidar = json.loads(msg.data)
        except Exception:
            pass

    def _on_map(self, msg: OccupancyGrid):
        self._map = msg

    def _on_path(self, msg: Path):
        self._path = msg

    @staticmethod
    def _scan_to_points(msg: LaserScan, half_fov: Optional[float] = None) -> list:
        """Downsample LaserScan to [{a, r}] for dashboard (ADEV-style)."""
        n = len(msg.ranges)
        if n == 0:
            return []
        step = max(1, n // 120)
        points = []
        for i in range(0, n, step):
            r = float(msg.ranges[i])
            a = float(msg.angle_min + i * msg.angle_increment)
            if half_fov is not None and abs(a) > half_fov:
                continue
            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                points.append({'a': round(a, 4), 'r': round(r, 3)})
        return points

    def _on_front_scan(self, msg: LaserScan):
        # Continuous green FOV cone (± lidar_fov/2)
        self._scan_front_fov = self._scan_to_points(msg, self.lidar_half_fov)

    def _on_full_scan(self, msg: LaserScan):
        self._scan_full = self._scan_to_points(msg, None)

    def _dashboard_scan(self) -> list:
        if bool((self._lidar or {}).get('awake')) and self._scan_full:
            return self._scan_full
        return self._scan_front_fov

    def _on_battery(self, msg: BatteryState):
        self._battery_rx_count += 1
        v = float(msg.voltage)
        p = float(msg.percentage)
        if math.isnan(v) and math.isnan(p):
            self._battery = {
                'voltage': 0.0, 'percentage': 0.0, 'valid': False,
                'source': 'vmx_read_failed',
            }
            return
        pct = 0.0
        if not math.isnan(p):
            pct = p * 100.0 if p <= 1.0 else p
        elif not math.isnan(v):
            pct = max(0.0, min(100.0, (v - 10.0) / 4.0 * 100.0))
        self._battery = {
            'voltage': round(v, 2) if not math.isnan(v) else 0.0,
            'percentage': round(pct, 1),
            'valid': not math.isnan(v),
            'source': '/battery_state',
        }

    def _check_battery(self):
        if self._battery_rx_count > 0:
            return
        self._battery = {
            'voltage': 0.0, 'percentage': 0.0, 'valid': False,
            'source': 'waiting_for_control_server',
        }

    def _mode(self) -> str:
        fsm = self._fsm or {}
        state = str(fsm.get('state') or 'IDLE').upper()
        if self._estop or state == 'E_STOP':
            return 'E_STOP'
        if state in ('WAITING_POSE', 'RECOVERY', 'PLANNING', 'NAVIGATING', 'IDLE'):
            return state
        if self._pose is None:
            return 'WAITING_POSE'
        return state or 'IDLE'

    def _remaining_steps(self) -> list:
        fsm = self._fsm or {}
        steps = fsm.get('remaining_steps')
        if isinstance(steps, list):
            return steps
        prims = (self._plan or {}).get('primitives') or []
        idx = int(fsm.get('primitive_idx') or 0)
        if isinstance(prims, list) and idx < len(prims):
            return prims[idx:]
        return []

    def snapshot(self) -> dict:
        pose = None
        if self._pose is not None:
            pose = {
                'x': round(float(self._pose.pose.position.x), 4),
                'y': round(float(self._pose.pose.position.y), 4),
                'yaw': round(yaw_of_pose(self._pose), 4),
            }
        path_pts = []
        if self._path is not None:
            for p in self._path.poses[:200]:
                path_pts.append({
                    'x': round(float(p.pose.position.x), 3),
                    'y': round(float(p.pose.position.y), 3),
                })
        return {
            'ok': True,
            'robot': self.robot,
            'estop': self._estop,
            'mode': self._mode(),
            'pose': pose,
            'watchdog': self._watch,
            'fsm': self._fsm,
            'plan': self._plan,
            'remaining_steps': self._remaining_steps(),
            'planner_status': self._planner_status,
            'lidar': self._lidar,
            'scan': self._dashboard_scan(),
            'all_steps': (self._fsm or {}).get('all_steps') or (self._plan or {}).get('primitives') or [],
            'step_statuses': (self._fsm or {}).get('step_statuses') or [],
            'step_progress': (self._fsm or {}).get('step_progress') or {},
            'battery': self._battery,
            'path': path_pts,
            'robot_dims': {
                'length': self.robot_length,
                'width': self.robot_width,
            },
            'ts': time.time(),
        }

    def set_pose(self, x: float, y: float, yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(float(yaw) * 0.5)
        msg.pose.pose.orientation.w = math.cos(float(yaw) * 0.5)
        self._initialpose.publish(msg)

    def set_goal(self, x: float, y: float, yaw: float = 0.0):
        msg = String()
        msg.data = json.dumps({
            'x': float(x),
            'y': float(y),
            'yaw': float(yaw),
        })
        self._goal_pub.publish(msg)

    def set_goals(self, waypoints: list):
        """waypoints: [{x,y,yaw}, ...]"""
        msg = String()
        msg.data = json.dumps({'waypoints': waypoints})
        self._goal_pub.publish(msg)

    def send_cmd(self, cmd: str, **extra):
        payload = {'cmd': cmd, **extra}
        msg = String()
        msg.data = json.dumps(payload)
        self._cmd_pub.publish(msg)

    def set_estop(self, active: bool):
        self._estop = bool(active)
        self.send_cmd('estop', active=self._estop)
        if self._estop:
            self._cmd_vel.publish(Twist())

    def load_map(self, path: str):
        msg = String()
        msg.data = path
        self._map_load.publish(msg)

    def map_payload(self) -> Optional[dict]:
        if self._map is None:
            return None
        m = self._map
        return {
            'resolution': m.info.resolution,
            'width': m.info.width,
            'height': m.info.height,
            'origin': {
                'x': m.info.origin.position.x,
                'y': m.info.origin.position.y,
            },
            'data': list(m.data),
        }

    def list_maps(self) -> list:
        root = FilePath(os.path.expanduser(self.maps_dir))
        out = []
        if not root.is_dir():
            return out
        for p in sorted(root.glob('*.yaml')):
            out.append(str(p))
        return out

    def _run_http(self):
        bridge = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                pass

            def _cors(self):
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')

            def _json(self, code: int, obj: dict):
                body = json.dumps(obj).encode('utf-8')
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self._cors()
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_OPTIONS(self):
                self.send_response(204)
                self._cors()
                self.end_headers()

            def do_GET(self):
                u = urlparse(self.path)
                if u.path == '/api/status' or u.path == '/api/telemetry':
                    self._json(200, bridge.snapshot())
                    return
                if u.path == '/api/maps':
                    self._json(200, {'maps': bridge.list_maps()})
                    return
                if u.path == '/api/map':
                    qs = parse_qs(u.query)
                    path = (qs.get('path') or [''])[0]
                    if path:
                        bridge.load_map(path)
                        time.sleep(0.3)
                    payload = bridge.map_payload()
                    if payload is None:
                        self._json(404, {'error': 'no map'})
                    else:
                        self._json(200, payload)
                    return
                if u.path == '/api/stream':
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/event-stream')
                    self.send_header('Cache-Control', 'no-cache')
                    self._cors()
                    self.end_headers()
                    try:
                        while True:
                            data = json.dumps(bridge.snapshot())
                            self.wfile.write(f'data: {data}\n\n'.encode('utf-8'))
                            self.wfile.flush()
                            time.sleep(0.25)
                    except Exception:
                        return
                self._json(404, {'error': 'not found'})

            def do_POST(self):
                u = urlparse(self.path)
                length = int(self.headers.get('Content-Length', 0))
                raw = self.rfile.read(length) if length else b'{}'
                try:
                    body = json.loads(raw.decode('utf-8') or '{}')
                except Exception:
                    body = {}

                if u.path == '/api/estop':
                    bridge.set_estop(bool(body.get('active', True)))
                    self._json(200, {'ok': True, 'estop': bridge._estop})
                    return
                if u.path == '/api/set_pose':
                    bridge.set_pose(
                        float(body.get('x', 0)),
                        float(body.get('y', 0)),
                        float(body.get('yaw', 0)))
                    self._json(200, {'ok': True})
                    return
                if u.path == '/api/goal':
                    if isinstance(body.get('waypoints'), list):
                        bridge.set_goals(body['waypoints'])
                    else:
                        bridge.set_goal(
                            float(body['x']),
                            float(body['y']),
                            float(body.get('yaw', 0.0)))
                    self._json(200, {'ok': True})
                    return
                if u.path == '/api/start':
                    bridge.send_cmd('start')
                    self._json(200, {'ok': True})
                    return
                if u.path == '/api/stop':
                    bridge.send_cmd('stop')
                    self._json(200, {'ok': True})
                    return
                if u.path == '/api/map/load':
                    bridge.load_map(str(body.get('path', '')))
                    self._json(200, {'ok': True})
                    return
                self._json(404, {'error': 'not found'})

        server = ThreadingHTTPServer(('0.0.0.0', self.http_port), Handler)
        server.serve_forever()


def main(args=None):
    rclpy.init(args=args)
    node = CaseNavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
