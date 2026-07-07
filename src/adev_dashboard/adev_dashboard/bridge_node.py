#!/usr/bin/env python3
"""ADEV Dashboard — lightweight HTTP/SSE bridge (no rosbridge)."""

import base64
import json
import math
import os
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path as FilePath
from urllib.parse import parse_qs, urlparse

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

try:
    import yaml
except ImportError:
    yaml = None


def yaw_to_quat(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(qz: float, qw: float) -> float:
    return math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)


GOAL_STATUS_NAMES = {
    0: 'UNKNOWN',
    1: 'ACCEPTED',
    2: 'EXECUTING',
    3: 'CANCELING',
    4: 'SUCCEEDED',
    5: 'CANCELED',
    6: 'ABORTED',
}


ROBOT_DIMS = {
    'allterrain': {'length': 0.43, 'width': 0.40},
    'mecanum': {'length': 0.40, 'width': 0.40},
    'xdrive': {'length': 0.43, 'width': 0.40},
}


def path_to_points(msg: Path, max_pts: int = 120):
    if not msg.poses:
        return []
    step = max(1, len(msg.poses) // max_pts)
    return [
        {'x': round(p.pose.position.x, 3), 'y': round(p.pose.position.y, 3)}
        for p in msg.poses[::step]
    ]


class TelemetryStore:
    def __init__(self):
        self._lock = threading.Lock()
        self._data = {
            'connected': True,
            'robot': 'allterrain',
            'simulation': False,
            'pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            'battery': {'voltage': 0.0, 'percentage': 0.0, 'valid': False, 'source': 'none'},
            'estop_hw': False,
            'estop_sw': False,
            'nav2': {
                'available': False,
                'state': 'unknown',
                'mission': 'idle',
            },
            'mission': {
                'active': False,
                'index': 0,
                'total': 0,
                'status': 'idle',
                'retries': 0,
                'current': None,
                'last_error': '',
                'dwell_remaining': 0.0,
                'dwell_total': 0.0,
            },
            'scan': [],
            'global_path': [],
            'local_path': [],
            'robot_dims': {'length': 0.43, 'width': 0.40},
        }

    def update(self, **kwargs):
        with self._lock:
            for key, value in kwargs.items():
                if isinstance(value, dict) and isinstance(self._data.get(key), dict):
                    self._data[key].update(value)
                else:
                    self._data[key] = value

    def snapshot(self):
        with self._lock:
            return json.loads(json.dumps(self._data))


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def densify_path(points, step: float = 0.04):
    """Return dense (x, y) list along polyline."""
    if not points:
        return []
    if len(points) == 1:
        return [points[0]]
    dense = [points[0]]
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg < 1e-6:
            continue
        n = max(1, int(seg / step))
        for k in range(1, n + 1):
            t = k / n
            dense.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
    return dense


class MissionRunner:
    """Waypoint mission — Nav2 in real mode, planner + path-follow point in simulation."""

    XY_TOL = 0.10
    YAW_TOL = 0.18
    SIM_SPEED = 0.14
    SIM_ROT_SPEED = 0.55
    WAYPOINT_DWELL_SEC = 4.0

    def __init__(self, bridge: 'AdevBridgeNode', telemetry: TelemetryStore):
        self.bridge = bridge
        self.node = bridge
        self.telemetry = telemetry
        self.nav_client = ActionClient(bridge, NavigateToPose, 'navigate_to_pose')
        self.plan_client = ActionClient(bridge, ComputePathToPose, 'compute_path_to_pose')
        self.waypoints = []
        self.index = 0
        self.active = False
        self.retries = 0
        self._goal_handle = None
        self._pending_timer = None
        self._poll_timer = None
        self._follow_timer = None
        self._path_pts = []
        self._path_idx = 0
        self._follow_wp = None
        self._rotate_to_yaw = None
        self._ignore_yaw_cancel = False
        self._dwell_timer = None
        self._dwell_remaining = 0.0

    def set_waypoints(self, waypoints):
        self.waypoints = waypoints
        self.index = 0
        self.retries = 0
        self.telemetry.update(mission={
            'total': len(waypoints),
            'index': 0,
            'status': 'armed',
            'current': waypoints[0] if waypoints else None,
            'mode': 'sim_path' if self.bridge.simulation_enabled else 'nav2',
        })

    def start(self):
        if not self.waypoints:
            return False, 'No waypoints defined'
        self._cancel_timers()
        self.active = True
        self.index = 0
        self.retries = 0
        self._ignore_yaw_cancel = False
        self.telemetry.update(mission={
            'active': True,
            'status': 'running',
            'mode': 'sim_path' if self.bridge.simulation_enabled else 'nav2',
            'dwell_remaining': 0.0,
            'dwell_total': 0.0,
        })
        self._dispatch()
        mode = 'sim path-follow' if self.bridge.simulation_enabled else 'Nav2'
        return True, f'Mission started ({mode})'

    def stop(self):
        self.active = False
        self._cancel_timers()
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._rotate_to_yaw = None
        self.telemetry.update(mission={
            'active': False,
            'status': 'stopped',
            'dwell_remaining': 0.0,
            'dwell_total': 0.0,
        })

    def _cancel_nav_timers(self):
        """Cancel navigation helpers only — never the inter-waypoint dwell timer."""
        for attr in ('_pending_timer', '_poll_timer', '_follow_timer'):
            timer = getattr(self, attr)
            if timer is not None:
                timer.cancel()
                setattr(self, attr, None)

    def _cancel_timers(self):
        self._cancel_nav_timers()
        if self._dwell_timer is not None:
            self._dwell_timer.cancel()
            self._dwell_timer = None

    def _current_pose(self):
        p = self.bridge._cached_pose
        return float(p['x']), float(p['y']), float(p['yaw'])

    def _dispatch(self):
        self._cancel_nav_timers()
        if not self.active:
            return
        if self.index >= len(self.waypoints):
            self.active = False
            self.telemetry.update(mission={
                'active': False,
                'status': 'complete',
                'index': self.index,
            })
            self.node.get_logger().info('ADEV mission complete')
            return

        wp = self.waypoints[self.index]
        self.node.get_logger().info(
            f'Waypoint {self.index + 1}/{len(self.waypoints)}: '
            f'({wp["x"]:.3f}, {wp["y"]:.3f}) '
            f'yaw={math.degrees(wp.get("yaw", 0)):.0f}° '
            f'ignore_yaw={bool(wp.get("ignore_yaw", False))}')
        self.telemetry.update(mission={
            'index': self.index,
            'total': len(self.waypoints),
            'status': 'navigating',
            'current': wp,
            'retries': self.retries,
        })

        if self.bridge.simulation_enabled:
            self._dispatch_sim_plan(wp)
        else:
            self._dispatch_nav2(wp)

    def _dispatch_sim_plan(self, wp):
        if not self.plan_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().warn('Planner not ready — retry in 2s')
            self._pending_timer = self.node.create_timer(2.0, self._timer_dispatch)
            return

        cx, cy, _ = self._current_pose()
        goal = ComputePathToPose.Goal()
        goal.planner_id = 'GridBased'
        goal.use_start = True

        goal.start = PoseStamped()
        goal.start.header.frame_id = 'map'
        goal.start.header.stamp = self.node.get_clock().now().to_msg()
        goal.start.pose.position.x = cx
        goal.start.pose.position.y = cy
        _, _, sqz, sqw = yaw_to_quat(self.bridge._cached_pose['yaw'])
        goal.start.pose.orientation.z = sqz
        goal.start.pose.orientation.w = sqw

        goal.goal = PoseStamped()
        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = goal.start.header.stamp
        goal.goal.pose.position.x = float(wp['x'])
        goal.goal.pose.position.y = float(wp['y'])
        _, _, gqz, gqw = yaw_to_quat(float(wp.get('yaw', 0.0)))
        goal.goal.pose.orientation.z = gqz
        goal.goal.pose.orientation.w = gqw

        self.telemetry.update(nav2={'available': True, 'state': 'planning'})
        future = self.plan_client.send_goal_async(goal)
        future.add_done_callback(self._on_plan_goal_sent)

    def _on_plan_goal_sent(self, future):
        if not self.active:
            return
        handle = future.result()
        if not handle.accepted:
            self.node.get_logger().warn('ComputePathToPose rejected — retry')
            self._schedule_retry()
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_plan_result)

    def _on_plan_result(self, future):
        if not self.active:
            return
        wp = self.waypoints[self.index]
        try:
            result = future.result().result
            pts = [
                (p.pose.position.x, p.pose.position.y)
                for p in result.path.poses
            ]
        except Exception as exc:
            self.node.get_logger().warn(f'Plan failed: {exc} — straight line')
            cx, cy, _ = self._current_pose()
            pts = [(cx, cy), (float(wp['x']), float(wp['y']))]

        if len(pts) < 2:
            cx, cy, _ = self._current_pose()
            pts = [(cx, cy), (float(wp['x']), float(wp['y']))]

        self.telemetry.update(
            nav2={'state': 'sim_following'},
            global_path=[{'x': round(x, 3), 'y': round(y, 3)} for x, y in pts[::max(1, len(pts) // 80)]],
            local_path=[],
        )
        self._start_path_follow(densify_path(pts), wp)

    def _start_path_follow(self, path_pts, wp):
        self._path_pts = path_pts
        self._path_idx = 1 if len(path_pts) > 1 else 0
        self._follow_wp = wp
        self._rotate_to_yaw = None
        if self._follow_timer is None:
            self._follow_timer = self.node.create_timer(0.05, self._path_tick)

    def _path_tick(self):
        if not self.active or self._follow_wp is None:
            return

        wp = self._follow_wp
        cx, cy, cyaw = self._current_pose()
        dt = 0.05

        if self._rotate_to_yaw is not None:
            err = normalize_angle(self._rotate_to_yaw - cyaw)
            if abs(err) < self.YAW_TOL:
                self._waypoint_reached()
                return
            step = self.SIM_ROT_SPEED * dt
            new_yaw = cyaw + max(-step, min(step, err))
            self.bridge._publish_sim_seed(cx, cy, new_yaw)
            return

        if self._path_idx < len(self._path_pts):
            tx, ty = self._path_pts[self._path_idx]
            dx, dy = tx - cx, ty - cy
            dist = math.hypot(dx, dy)
            step = self.SIM_SPEED * dt
            if dist <= step:
                self._path_idx += 1
                if dist > 1e-6:
                    face = math.atan2(dy, dx)
                    self.bridge._publish_sim_seed(tx, ty, face)
                return
            scale = step / dist
            nx = cx + dx * scale
            ny = cy + dy * scale
            face = math.atan2(dy, dx)
            self.bridge._publish_sim_seed(nx, ny, face)
            return

        gx, gy = float(wp['x']), float(wp['y'])
        if math.hypot(gx - cx, gy - cy) > self.XY_TOL:
            dx, dy = gx - cx, gy - cy
            dist = math.hypot(dx, dy)
            step = min(self.SIM_SPEED * dt, dist)
            scale = step / max(dist, 1e-6)
            nx = cx + dx * scale
            ny = cy + dy * scale
            self.bridge._publish_sim_seed(nx, ny, math.atan2(dy, dx))
            return

        if wp.get('ignore_yaw', False):
            self._waypoint_reached()
            return

        target_yaw = float(wp.get('yaw', 0.0))
        if abs(normalize_angle(target_yaw - cyaw)) < self.YAW_TOL:
            self._waypoint_reached()
            return
        self._rotate_to_yaw = target_yaw

    def _on_waypoint_success(self):
        """Goal reached — dwell before the next waypoint (if any)."""
        if self._dwell_timer is not None:
            return

        self.node.get_logger().info(f'Waypoint {self.index + 1} reached')
        self._follow_wp = None
        self._rotate_to_yaw = None
        self._path_pts = []
        self._goal_handle = None
        self.retries = 0
        self.telemetry.update(mission={'last_error': ''})

        if self.index >= len(self.waypoints) - 1:
            self.index += 1
            self.telemetry.update(mission={'dwell_remaining': 0.0, 'dwell_total': 0.0})
            self._dispatch()
            return

        self._start_dwell(self.WAYPOINT_DWELL_SEC)

    def _start_dwell(self, seconds: float):
        self._cancel_nav_timers()
        self._dwell_remaining = float(seconds)
        self.node.get_logger().info(
            f'Waiting {seconds:.0f}s before waypoint {self.index + 2}/{len(self.waypoints)}')
        self.telemetry.update(mission={
            'status': 'dwelling',
            'dwell_remaining': round(self._dwell_remaining, 1),
            'dwell_total': seconds,
        })
        self._dwell_timer = self.node.create_timer(0.25, self._dwell_tick)

    def _dwell_tick(self):
        if not self.active:
            return
        self._dwell_remaining = max(0.0, self._dwell_remaining - 0.25)
        self.telemetry.update(mission={
            'status': 'dwelling',
            'dwell_remaining': round(self._dwell_remaining, 1),
        })
        if self._dwell_remaining <= 0.0:
            if self._dwell_timer is not None:
                self._dwell_timer.cancel()
                self._dwell_timer = None
            self.index += 1
            self.telemetry.update(mission={'dwell_remaining': 0.0, 'dwell_total': 0.0})
            self.node.get_logger().info(
                f'Dwell complete — advancing to waypoint {self.index + 1}/{len(self.waypoints)}')
            self._dispatch()

    def _waypoint_reached(self):
        self._on_waypoint_success()

    def _dispatch_nav2(self, wp):
        if not self.nav_client.wait_for_server(timeout_sec=1.5):
            self.telemetry.update(nav2={'available': False, 'state': 'unavailable'})
            self._pending_timer = self.node.create_timer(2.0, self._timer_dispatch)
            return

        self.telemetry.update(nav2={'available': True, 'state': 'navigating'})

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = rclpy.time.Time().to_msg()
        goal.pose.pose.position.x = float(wp['x'])
        goal.pose.pose.position.y = float(wp['y'])
        _, _, qz, qw = yaw_to_quat(float(wp.get('yaw', 0.0)))
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_sent)

        if wp.get('ignore_yaw', False) and self._poll_timer is None:
            self._poll_timer = self.node.create_timer(0.25, self._poll_xy_goal)

    def _poll_xy_goal(self):
        if not self.active or self._goal_handle is None:
            return
        wp = self.waypoints[self.index]
        if not wp.get('ignore_yaw', False):
            return
        cx, cy, _ = self._current_pose()
        if math.hypot(float(wp['x']) - cx, float(wp['y']) - cy) <= self.XY_TOL:
            self.node.get_logger().info(
                f'Waypoint {self.index + 1} XY reached (yaw ignored)')
            handle = self._goal_handle
            self._goal_handle = None
            self._ignore_yaw_cancel = True
            if self._poll_timer is not None:
                self._poll_timer.cancel()
                self._poll_timer = None
            handle.cancel_goal_async()
            self._on_waypoint_success()

    def _timer_dispatch(self):
        self._cancel_nav_timers()
        self._dispatch()

    def _on_goal_sent(self, future):
        if not self.active:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().warn('NavigateToPose goal rejected — retrying')
            self._schedule_retry()
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        if not self.active:
            return
        self._goal_handle = None
        status = future.result().status
        status_name = GOAL_STATUS_NAMES.get(status, str(status))
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._cancel_nav_timers()
            self._on_waypoint_success()
        elif status == GoalStatus.STATUS_CANCELED:
            if self._ignore_yaw_cancel:
                self._ignore_yaw_cancel = False
                return
            self._cancel_nav_timers()
        else:
            self._cancel_nav_timers()
            err = f'Nav2 goal {status_name} (code {status})'
            self.node.get_logger().warn(
                f'Waypoint {self.index + 1} failed: {err} — retrying in 2s')
            self.telemetry.update(mission={'last_error': err, 'status': 'retrying'})
            self._schedule_retry()

    def _schedule_retry(self):
        self.retries += 1
        self.telemetry.update(mission={'status': 'retrying', 'retries': self.retries})
        self._pending_timer = self.node.create_timer(2.0, self._timer_dispatch)


class AdevBridgeNode(Node):
  def __init__(self):
    super().__init__('adev_bridge')
    self.declare_parameter('robot', 'allterrain')
    self.declare_parameter('http_port', 8080)
    self.declare_parameter('maps_dir', os.path.expanduser('~/skill23_ws'))

    self.robot = self.get_parameter('robot').value
    self.http_port = int(self.get_parameter('http_port').value)
    self.maps_dir = self.get_parameter('maps_dir').value

    self.telemetry = TelemetryStore()
    self.telemetry.update(robot=self.robot)
    self.mission = MissionRunner(self, self.telemetry)
    self.estop_active = False
    self.simulation_enabled = False
    self.pose_initialized = True  # AMCL set_initial_pose in nav2 yaml
    self._battery_rx_count = 0
    self._cached_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    self._sim_pending = None
    self._sim_apply_timer = None
    dims = ROBOT_DIMS.get(self.robot, ROBOT_DIMS['allterrain'])
    self.telemetry.update(robot_dims=dims)

    self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    self.estop_pub = self.create_publisher(Bool, '/adev/estop', 10)
    self.sim_pub = self.create_publisher(Bool, '/adev/simulation', 10)
    self.sim_seed_pub = self.create_publisher(
        PoseWithCovarianceStamped, '/adev/sim_seed', 10)
    self.initial_pose_pub = self.create_publisher(
        PoseWithCovarianceStamped, '/initialpose', 10)
    init_sim = Bool()
    init_sim.data = False
    self.sim_pub.publish(init_sim)

    scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    path_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    battery_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    self.create_subscription(Bool, '/estop/state', self._estop_hw_cb, 10)
    self.create_subscription(BatteryState, '/battery_state', self._battery_cb, battery_qos)
    self.create_subscription(LaserScan, '/scan', self._scan_cb, scan_qos)
    self.create_subscription(LaserScan, '/scan_filtered', self._scan_cb, scan_qos)
    self.create_subscription(Path, '/plan', self._global_plan_cb, path_qos)
    self.create_subscription(Path, '/local_plan', self._local_plan_cb, path_qos)
    self.create_subscription(
        PoseWithCovarianceStamped, '/amcl_pose', self._amcl_pose_cb, 10)
    self.create_subscription(PoseStamped, '/adev/sim_pose', self._sim_pose_cb, 10)

    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
    self.create_timer(0.2, self._update_pose)
    self.create_timer(5.0, self._check_battery)

    self._http_ready = threading.Event()
    self._http_thread = threading.Thread(target=self._run_http, daemon=True)
    self._http_thread.start()
    if self._http_ready.wait(timeout=5.0):
        self.get_logger().info(
            f'ADEV API bridge on :{self.http_port} (API only — UI runs on laptop) | robot={self.robot}')
    else:
        self.get_logger().error(
            f'HTTP bridge failed to start on port {self.http_port}. '
            'Kill stale process: pkill -f adev_bridge  then relaunch.')

  def _estop_hw_cb(self, msg: Bool):
    self.telemetry.update(estop_hw=bool(msg.data))
    if msg.data:
        self._apply_estop(True, source='hardware')

  def _battery_cb(self, msg: BatteryState):
    self._battery_rx_count += 1
    v = float(msg.voltage)
    p = float(msg.percentage)
    if math.isnan(v) and math.isnan(p):
      self.telemetry.update(battery={
          'voltage': 0.0, 'percentage': 0.0, 'valid': False,
          'source': 'vmx_read_failed',
      })
      return
    pct = 0.0
    if not math.isnan(p):
      pct = p * 100.0 if p <= 1.0 else p
    elif not math.isnan(v):
      pct = max(0.0, min(100.0, (v - 10.0) / 4.0 * 100.0))
    self.telemetry.update(battery={
        'voltage': round(v, 2) if not math.isnan(v) else 0.0,
        'percentage': round(pct, 1),
        'valid': not math.isnan(v),
        'source': '/battery_state',
    })

  def _check_battery(self):
    if self._battery_rx_count > 0:
      return
    self.get_logger().warn(
        'No /battery_state yet — is control_server running? '
        'Launch with: ~/skill23_ws/src/studica_ws/scripts/ros2_pi.sh launch adev_dashboard ... '
        'Check: ros2_pi.sh topic hz /battery_state')
    self.telemetry.update(battery={
        'voltage': 0.0, 'percentage': 0.0, 'valid': False,
        'source': 'waiting_for_control_server',
    })

  def _global_plan_cb(self, msg: Path):
    self.telemetry.update(global_path=path_to_points(msg))

  def _local_plan_cb(self, msg: Path):
    self.telemetry.update(local_path=path_to_points(msg))

  def _store_pose(self, x: float, y: float, yaw: float):
    self._cached_pose = {
        'x': round(float(x), 3),
        'y': round(float(y), 3),
        'yaw': round(float(yaw), 3),
    }
    self.pose_initialized = True
    self.telemetry.update(pose=dict(self._cached_pose))

  def _lookup_map_pose(self):
    try:
        tf = self.tf_buffer.lookup_transform(
            'map', 'base_footprint', rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.2))
        t = tf.transform.translation
        yaw = quat_to_yaw(tf.transform.rotation.z, tf.transform.rotation.w)
        return t.x, t.y, yaw
    except Exception:
        if self.pose_initialized:
            p = self._cached_pose
            return p['x'], p['y'], p['yaw']
        return None

  def _publish_sim_seed(self, x: float, y: float, yaw: float):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    _, _, qz, qw = yaw_to_quat(float(yaw))
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.068
    self.sim_seed_pub.publish(msg)
    self._store_pose(x, y, yaw)

  def _set_localization_tf(self, enabled: bool):
    val = 'true' if enabled else 'false'
    for node, param in (('amcl', 'tf_broadcast'), ('ekf_filter_node', 'publish_tf')):
        try:
            subprocess.run(
                ['ros2', 'param', 'set', f'/{node}', param, val],
                capture_output=True, timeout=2.0, check=False)
        except Exception as exc:
            self.get_logger().warn(f'param set {node}/{param} failed: {exc}')

  def _amcl_pose_cb(self, msg: PoseWithCovarianceStamped):
    if self.simulation_enabled:
      return
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    self._store_pose(p.x, p.y, quat_to_yaw(q.z, q.w))

  def _sim_pose_cb(self, msg: PoseStamped):
    if not self.simulation_enabled:
      return
    p = msg.pose.position
    q = msg.pose.orientation
    self._store_pose(p.x, p.y, quat_to_yaw(q.z, q.w))

  def _scan_cb(self, msg: LaserScan):
    points = []
    angle = msg.angle_min
    for r in msg.ranges[::max(1, len(msg.ranges) // 120)]:
        if math.isfinite(r) and msg.range_min < r < msg.range_max:
            points.append({'a': angle, 'r': r})
        angle += msg.angle_increment * max(1, len(msg.ranges) // 120)
    self.telemetry.update(scan=points)

  def _update_pose(self):
    if self.simulation_enabled:
      return
    try:
        tf = self.tf_buffer.lookup_transform(
            'map', 'base_footprint', rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.05))
        t = tf.transform.translation
        self._store_pose(
            t.x, t.y, quat_to_yaw(tf.transform.rotation.z, tf.transform.rotation.w))
    except Exception:
        if self.pose_initialized:
            self.telemetry.update(pose=dict(self._cached_pose))

  def set_simulation(self, enabled: bool):
    self._sim_pending = bool(enabled)
    if self._sim_apply_timer is None:
        self._sim_apply_timer = self.create_timer(0.05, self._apply_simulation_pending)

  def _apply_simulation_pending(self):
    if self._sim_pending is None:
        return
    enabled = self._sim_pending
    self._sim_pending = None
    if enabled == self.simulation_enabled:
        return

    if enabled:
        p = self._cached_pose
        x, y, yaw = float(p['x']), float(p['y']), float(p['yaw'])
        self._set_localization_tf(False)
        self._publish_sim_seed(x, y, yaw)
        self.simulation_enabled = True
        self.telemetry.update(simulation=True)
        self.sim_pub.publish(Bool(data=True))
        self.get_logger().info(
            f'Simulation ON — pose kept at ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)')
        return

    pose = dict(self._cached_pose)
    self.simulation_enabled = False
    self.sim_pub.publish(Bool(data=False))
    self._set_localization_tf(True)
    self.set_initial_pose(pose['x'], pose['y'], pose['yaw'])
    self.telemetry.update(simulation=False)
    self.get_logger().info(
        f'Simulation OFF — AMCL set to ({pose["x"]:.2f}, {pose["y"]:.2f})')

  def _apply_estop(self, active: bool, source: str = 'software'):
    self.estop_active = active
    self.telemetry.update(estop_sw=active)
    self.estop_pub.publish(Bool(data=active))
    twist = Twist()
    self.cmd_vel_pub.publish(twist)
    if active:
        self.mission.stop()
        self.get_logger().warn(f'E-STOP activated ({source})')
    else:
        self.get_logger().info('E-STOP released')

  def set_initial_pose(self, x, y, yaw):
    self._store_pose(x, y, yaw)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    _, _, qz, qw = yaw_to_quat(float(yaw))
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.068
    self.initial_pose_pub.publish(msg)
    if self.simulation_enabled:
        self._publish_sim_seed(x, y, yaw)
        self.mission.stop()

  def load_map_info(self, map_path: str):
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

  def _run_http(self):
    node = self
    store = self.telemetry

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
                self._json_response(200, {'ok': True, 'robot': node.robot})
                return

            if path == '/api/telemetry':
                self._json_response(200, store.snapshot())
                return

            if path == '/api/stream':
                self.send_response(200)
                self.send_header('Content-Type', 'text/event-stream')
                self.send_header('Cache-Control', 'no-cache')
                self._cors()
                self.end_headers()
                try:
                    while rclpy.ok():
                        data = store.snapshot()
                        chunk = f'data: {json.dumps(data)}\n\n'.encode('utf-8')
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
                    if yml.name.endswith('.yaml'):
                        maps.append({'name': yml.stem, 'path': str(yml)})
                self._json_response(200, {'maps': maps, 'robot': node.robot})
                return

            if path == '/api/map':
                qs = parse_qs(parsed.query)
                map_path = qs.get('path', [''])[0]
                if not map_path:
                    self._json_response(400, {'error': 'path query required'})
                    return
                try:
                    info = node.load_map_info(map_path)
                    self._json_response(200, info)
                except Exception as exc:
                    self._json_response(500, {'error': str(exc)})
                return

            self._json_response(404, {
                'error': 'API only — open ADEV dashboard on your laptop, not the Pi browser',
                'hint': 'Run ~/Music/adev_dashboard/start.sh on laptop',
            })

        def do_POST(self):
            parsed = urlparse(self.path)
            path = parsed.path
            body = self._read_json()

            if path == '/api/estop':
                active = bool(body.get('active', True))
                node._apply_estop(active)
                self._json_response(200, {'estop': active})
                return

            if path == '/api/mission/waypoints':
                wps = body.get('waypoints', [])
                for wp in wps:
                    wp.setdefault('ignore_yaw', False)
                node.mission.set_waypoints(wps)
                self._json_response(200, {'count': len(wps)})
                return

            if path == '/api/mission/start':
                ok, msg = node.mission.start()
                payload = {'ok': ok, 'message': msg}
                if not ok:
                    payload['error'] = msg
                self._json_response(200 if ok else 400, payload)
                return

            if path == '/api/mission/stop':
                node.mission.stop()
                self._json_response(200, {'ok': True})
                return

            if path == '/api/initial_pose':
                node.set_initial_pose(
                    body.get('x', 0), body.get('y', 0), body.get('yaw', 0))
                self._json_response(200, {'ok': True})
                return

            if path == '/api/simulation':
                enabled = bool(body.get('enabled', False))
                node.set_simulation(enabled)
                self._json_response(200, {'simulation': enabled})
                return

            if path == '/api/robot/info':
                self._json_response(200, {
                    'robot': node.robot,
                    'launch_hint': (
                        '~/skill23_ws/src/studica_ws/scripts/ros2_pi.sh launch '
                        f'adev_dashboard adev_dashboard.launch.py robot:={node.robot}'),
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
            'Stop other adev_bridge instances: pkill -f adev_bridge')
        return

    node._http_ready.set()
    server.serve_forever()


def main(args=None):
    rclpy.init(args=args)
    node = AdevBridgeNode()
    try:
        rclpy.spin(node)
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
