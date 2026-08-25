#!/usr/bin/env python3
"""Custom-nav HTTP/SSE bridge on port 8082."""

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

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Bool, Empty, String

import rclpy.duration

from skill23_studio.nav.planning import decode_edt_cells, plan_path
from skill23_studio.nav.smoothing import smooth_path_poses
from skill23_studio.nav.utils import pose_cov_to_stamped


def _yaw_of_pose(pose: PoseStamped) -> float:
    q = pose.pose.orientation
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


class CustomNavBridge(Node):
    def __init__(self) -> None:
        super().__init__('cn_bridge')
        self.declare_parameter('robot', 'mecanum')
        self.declare_parameter('http_port', 8082)
        self.declare_parameter('maps_dir', os.path.expanduser('~/skill23_ws'))
        self.declare_parameter('robot_length', 0.40)
        self.declare_parameter('robot_width', 0.40)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('path_style', 'smoothed')

        self.robot = str(self.get_parameter('robot').value)
        self.http_port = int(self.get_parameter('http_port').value)
        self.maps_dir = str(self.get_parameter('maps_dir').value)
        self.robot_length = float(self.get_parameter('robot_length').value)
        self.robot_width = float(self.get_parameter('robot_width').value)
        style = str(self.get_parameter('path_style').value).strip().lower()
        self._path_style = 'smoothed' if style not in ('normal', 'raw', 'planner') else 'normal'
        if style in ('smoothed', 'smooth'):
            self._path_style = 'smoothed'

        self._estop = False
        self._pose: Optional[PoseStamped] = None
        self._map: Optional[OccupancyGrid] = None
        self._static_edt: Optional[np.ndarray] = None
        self._path: Optional[Path] = None
        self._planner_path: Optional[Path] = None
        self._smoothed_path: Optional[Path] = None
        self._mission: Dict[str, Any] = {}
        self._recovery: Dict[str, Any] = {}
        self._collision: Dict[str, Any] = {}
        self._nodes: Dict[str, Any] = {}
        self._goal_status = 'NOT_REACHED'
        self._world_ready = False
        self._amcl_ok = False
        self._amcl_last_t = 0.0
        self._scan: list = []
        self._battery: Dict[str, Any] = {
            'voltage': 0.0, 'percentage': 0.0, 'valid': False, 'source': 'none',
        }

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, qos)
        self.create_subscription(
            OccupancyGrid, '/custom_nav/static_edt', self._on_static_edt, qos)
        self.create_subscription(Path, '/custom_nav/path', self._on_path, 10)
        self.create_subscription(Path, '/custom_nav/planner_path', self._on_planner_path, 10)
        self.create_subscription(Path, '/custom_nav/smoothed_path', self._on_smoothed_path, 10)
        self.create_subscription(String, '/custom_nav/mission/status', self._on_mission, 10)
        self.create_subscription(String, '/custom_nav/recovery', self._on_recovery, 10)
        self.create_subscription(String, '/custom_nav/collision', self._on_collision, 10)
        self.create_subscription(String, '/custom_nav/goal_status', self._on_goal_status, 10)
        self.create_subscription(String, '/custom_nav/node_status', self._on_node_status, 10)
        self.create_subscription(Bool, '/custom_nav/world/ready', self._on_ready, qos)
        self.create_subscription(BatteryState, '/battery_state', self._on_battery, 10)
        self.create_subscription(
            LaserScan, str(self.get_parameter('scan_topic').value),
            self._on_scan, qos_profile_sensor_data)

        self._initialpose = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self._goal_pub = self.create_publisher(PoseStamped, '/custom_nav/goal', 10)
        self._goals_pub = self.create_publisher(String, '/custom_nav/goals', 10)
        self._start_pub = self.create_publisher(Empty, '/custom_nav/mission/start', 10)
        self._stop_pub = self.create_publisher(Empty, '/custom_nav/mission/stop', 10)
        self._map_load = self.create_publisher(String, '/custom_nav/map/load', 10)
        self._cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self._ctrl_pub = self.create_publisher(Bool, '/custom_nav/controller/enable', 10)
        self._path_style_pub = self.create_publisher(String, '/custom_nav/path_style', 10)

        threading.Thread(target=self._run_http, daemon=True).start()
        self.get_logger().info(f'Custom-nav bridge HTTP :{self.http_port} robot={self.robot}')
        # Publish default style so smoother + dashboard stay in sync.
        self.set_path_style(self._path_style)

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        ps = pose_cov_to_stamped(msg)
        x = float(ps.pose.position.x)
        y = float(ps.pose.position.y)
        # Reject exploded AMCL particles (wrong map / zero cov)
        if abs(x) > 50.0 or abs(y) > 50.0 or not math.isfinite(x) or not math.isfinite(y):
            self.get_logger().warn(
                f'Ignoring bogus /amcl_pose x={x:.1f} y={y:.1f} — set pose again',
                throttle_duration_sec=2.0)
            return
        self._pose = ps
        self._amcl_last_t = time.time()
        self._amcl_ok = True

    def _on_node_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            name = str(data.get('node', ''))
            if name:
                self._nodes[name] = data
        except Exception:
            pass

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg
        if self._static_edt is None and msg.data:
            # Fallback EDT from occupancy until static_edt arrives
            try:
                from skill23_studio.nav.utils import compute_edt
                occ = np.array(msg.data, dtype=np.int16).reshape(
                    msg.info.height, msg.info.width)
                self._static_edt = compute_edt(occ, msg.info.resolution, 50)
            except Exception:
                pass

    def _on_static_edt(self, msg: OccupancyGrid) -> None:
        if msg.info.width == 0 or self._map is None:
            return
        occ = np.array(self._map.data, dtype=np.int16).reshape(
            self._map.info.height, self._map.info.width)
        self._static_edt = decode_edt_cells(
            msg.data, msg.info.height, msg.info.width,
            msg.info.resolution, 50, occ)

    def _on_path(self, msg: Path) -> None:
        self._path = msg

    def _on_planner_path(self, msg: Path) -> None:
        self._planner_path = msg

    def _on_smoothed_path(self, msg: Path) -> None:
        self._smoothed_path = msg

    @staticmethod
    def _path_to_pts(path: Optional[Path], limit: int = 250) -> list:
        if path is None:
            return []
        pts = []
        for p in path.poses[:limit]:
            pts.append({
                'x': round(float(p.pose.position.x), 3),
                'y': round(float(p.pose.position.y), 3),
            })
        return pts

    def set_path_style(self, style: str) -> str:
        raw = str(style).strip().lower()
        if raw in ('normal', 'raw', 'planner'):
            self._path_style = 'normal'
        else:
            self._path_style = 'smoothed'
        msg = String()
        msg.data = self._path_style
        self._path_style_pub.publish(msg)
        return self._path_style

    def _on_mission(self, msg: String) -> None:
        try:
            self._mission = json.loads(msg.data)
        except Exception:
            pass

    def _on_recovery(self, msg: String) -> None:
        try:
            self._recovery = json.loads(msg.data)
        except Exception:
            pass

    def _on_collision(self, msg: String) -> None:
        try:
            self._collision = json.loads(msg.data)
        except Exception:
            pass

    def _on_goal_status(self, msg: String) -> None:
        self._goal_status = msg.data.strip()

    def _on_ready(self, msg: Bool) -> None:
        self._world_ready = bool(msg.data)

    def _on_battery(self, msg: BatteryState) -> None:
        v = float(msg.voltage)
        p = float(msg.percentage)
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

    @staticmethod
    def _scan_to_points(msg: LaserScan) -> list:
        n = len(msg.ranges)
        if n == 0:
            return []
        step = max(1, n // 120)
        points = []
        for i in range(0, n, step):
            r = float(msg.ranges[i])
            a = float(msg.angle_min + i * msg.angle_increment)
            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                points.append({'a': round(a, 4), 'r': round(r, 3)})
        return points

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = self._scan_to_points(msg)

    def snapshot(self) -> dict:
        pose = None
        if self._pose is not None:
            pose = {
                'x': round(float(self._pose.pose.position.x), 4),
                'y': round(float(self._pose.pose.position.y), 4),
                'yaw': round(_yaw_of_pose(self._pose), 4),
            }
        path_pts = self._path_to_pts(self._path)
        planner_pts = self._path_to_pts(self._planner_path)
        smoothed_pts = self._path_to_pts(self._smoothed_path)
        mode = str(self._mission.get('state', 'idle'))
        if self._estop:
            mode = 'E_STOP'
        amcl_age = (time.time() - self._amcl_last_t) if self._amcl_last_t else None
        # Fresh publish vs still robot: AMCL often stops republishing when idle.
        # Pose is still valid; "live" = recent msg, "held" = have pose but quiet.
        amcl_live = bool(self._amcl_ok and amcl_age is not None and amcl_age < 2.0)
        amcl_held = bool(self._amcl_ok and self._pose is not None and not amcl_live)
        planner = self._nodes.get('planner', {})
        return {
            'ok': True,
            'robot': self.robot,
            'estop': self._estop,
            'mode': mode,
            'pose': pose,
            'mission': self._mission,
            'recovery': self._recovery,
            'collision': self._collision,
            'goal_status': self._goal_status,
            'goal': {'state': self._goal_status, 'status': self._goal_status},
            'planner_status': {
                'state': planner.get('state', '—'),
                'message': planner.get('detail') or planner.get('state') or '—',
            },
            'nodes': self._nodes,
            'amcl': {
                'ok': amcl_live,
                'held': amcl_held,
                'age_s': round(amcl_age, 2) if amcl_age is not None else None,
                'has_pose': self._pose is not None,
            },
            'world_ready': self._world_ready,
            'scan': self._scan,
            'battery': self._battery,
            'path': path_pts,
            'planner_path': planner_pts,
            'smoothed_path': smoothed_pts,
            'path_style': self._path_style,
            'robot_dims': {'length': self.robot_length, 'width': self.robot_width},
            'ts': time.time(),
        }

    def plan_preview(self, start: dict, goal: dict) -> dict:
        """Dashboard simulation: plan + smooth (no mission / motion)."""
        if self._map is None:
            return {'ok': False, 'error': 'no map', 'path': []}
        if self._static_edt is None:
            return {'ok': False, 'error': 'no static EDT', 'path': []}
        info = self._map.info
        occ = np.array(self._map.data, dtype=np.int16).reshape(
            info.height, info.width)
        poses = plan_path(
            float(start.get('x', 0.0)),
            float(start.get('y', 0.0)),
            float(goal.get('x', 0.0)),
            float(goal.get('y', 0.0)),
            float(goal.get('yaw', 0.0)),
            occ,
            self._static_edt,
            info.origin.position.x,
            info.origin.position.y,
            float(info.resolution),
            local=None,
            occupied_threshold=50,
            clearance_weight=12.0,
            min_clearance_m=0.14,
            corner_clearance_m=0.20,
            ridge_snap_radius_m=0.10,
        )
        if not poses:
            return {'ok': False, 'error': 'no path found', 'path': []}
        planner_path = [{'x': round(x, 3), 'y': round(y, 3)} for x, y, _yaw in poses]
        smoothed = smooth_path_poses(
            poses,
            self._static_edt,
            info.origin.position.x,
            info.origin.position.y,
            float(info.resolution),
            occupancy=occ,
            occupied_thresh=50,
            min_turn_rad=0.35,
            fillet_radius_m=0.06,
            sample_step_m=0.04,
            min_clearance_m=0.14,
            max_offset_m=0.0,
            simplify_dev_m=0.03,
        )
        smoothed_path = [
            {'x': round(x, 3), 'y': round(y, 3)} for x, y, _yaw in smoothed]
        active = smoothed_path if self._path_style == 'smoothed' else planner_path
        return {
            'ok': True,
            'path': active,
            'planner_path': planner_path,
            'smoothed_path': smoothed_path,
            'path_style': self._path_style,
            'poses': len(active),
        }

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        # Stamp 0 → AMCL skips odom time-correction (past stamps shift the
        # pose by intervening odom motion). Tight cov keeps the click pose
        # from snapping to a nearby lidar match on the next scan.
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(float(yaw) * 0.5)
        msg.pose.pose.orientation.w = math.cos(float(yaw) * 0.5)
        # Non-zero but tight (σ≈5 cm / ≈6°) — zeros make the filter diverge
        msg.pose.covariance[0] = 0.0025
        msg.pose.covariance[7] = 0.0025
        msg.pose.covariance[35] = 0.01
        self._initialpose.publish(msg)
        # Optimistic local pose until AMCL republishes
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self._pose = ps
        self._amcl_last_t = time.time()
        self._amcl_ok = True
        self.get_logger().info(
            f'Set initial pose x={x:.3f} y={y:.3f} yaw={yaw:.3f}')

    def set_goal(self, x: float, y: float, yaw: float = 0.0) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(float(yaw) * 0.5)
        msg.pose.orientation.w = math.cos(float(yaw) * 0.5)
        self._goal_pub.publish(msg)

    def set_goals(self, waypoints: list) -> None:
        msg = String()
        msg.data = json.dumps({'waypoints': waypoints})
        self._goals_pub.publish(msg)

    def set_estop(self, active: bool) -> None:
        self._estop = bool(active)
        if self._estop:
            self._cmd_vel.publish(Twist())
            self._ctrl_pub.publish(Bool(data=False))
            self._stop_pub.publish(Empty())

    def load_map(self, path: str) -> None:
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
            'origin': {'x': m.info.origin.position.x, 'y': m.info.origin.position.y},
            'data': list(m.data),
        }

    def list_maps(self) -> list:
        root = FilePath(os.path.expanduser(self.maps_dir))
        if not root.is_dir():
            return []
        return sorted(str(p) for p in root.glob('*.yaml'))

    def _run_http(self) -> None:
        bridge = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                pass

            def _cors(self) -> None:
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')

            def _json(self, code: int, obj: dict) -> None:
                body = json.dumps(obj).encode('utf-8')
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self._cors()
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_OPTIONS(self) -> None:
                self.send_response(204)
                self._cors()
                self.end_headers()

            def do_GET(self) -> None:
                u = urlparse(self.path)
                if u.path in ('/api/status', '/api/telemetry'):
                    self._json(200, bridge.snapshot())
                    return
                if u.path == '/api/maps':
                    self._json(200, {'maps': bridge.list_maps()})
                    return
                if u.path == '/api/map':
                    qs = parse_qs(u.query)
                    # Display only — do NOT reload robot map on every GET
                    # (that was swapping maps under AMCL and exploding pose)
                    path = (qs.get('path') or [''])[0]
                    force = (qs.get('load') or ['0'])[0] in ('1', 'true', 'yes')
                    if path and force:
                        bridge.load_map(path)
                        time.sleep(0.4)
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

            def do_POST(self) -> None:
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
                        float(body.get('x', 0)), float(body.get('y', 0)),
                        float(body.get('yaw', 0)))
                    self._json(200, {'ok': True})
                    return
                if u.path == '/api/goal':
                    if isinstance(body.get('waypoints'), list):
                        bridge.set_goals(body['waypoints'])
                    else:
                        bridge.set_goal(
                            float(body['x']), float(body['y']),
                            float(body.get('yaw', 0.0)))
                    self._json(200, {'ok': True})
                    return
                if u.path == '/api/start':
                    bridge._start_pub.publish(Empty())
                    self._json(200, {'ok': True})
                    return
                if u.path == '/api/stop':
                    bridge._stop_pub.publish(Empty())
                    bridge._ctrl_pub.publish(Bool(data=False))
                    self._json(200, {'ok': True})
                    return
                if u.path == '/api/map/load':
                    bridge.load_map(str(body.get('path', '')))
                    self._json(200, {'ok': True})
                    return
                if u.path == '/api/plan':
                    start = body.get('start') or {}
                    goal = body.get('goal') or {}
                    result = bridge.plan_preview(start, goal)
                    self._json(200 if result.get('ok') else 400, result)
                    return
                if u.path == '/api/path_style':
                    style = bridge.set_path_style(str(body.get('style', 'smoothed')))
                    self._json(200, {'ok': True, 'path_style': style})
                    return
                self._json(404, {'error': 'not found'})

        server = ThreadingHTTPServer(('0.0.0.0', self.http_port), Handler)
        server.serve_forever()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CustomNavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
