#!/usr/bin/env python3
"""WorldSkills Studio HTTP/SSE bridge on port 8083."""

from __future__ import annotations

import atexit
import json
import math
import os
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path as FsPath
from typing import Any, Dict, List, Optional
from urllib.parse import parse_qs, urlparse

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateThroughPoses, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from skill23_studio import map_pipeline as maps
from skill23_studio.process_manager import ProcessManager
from skill23_studio.nav.utils import pose_cov_to_stamped, quat_from_yaw, yaw_from_quat


# Plenty for the ~40 lines the dashboard shows, without reading whole logs.
_LOG_TAIL_BYTES = 16_384


def _yaw_of(ps: PoseStamped) -> float:
    q = ps.pose.orientation
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


class StudioBridge(Node):
    def __init__(self) -> None:
        super().__init__('studio_bridge')
        self.declare_parameter('http_port', 8083)
        self.declare_parameter('maps_dir', os.path.expanduser('~/skill23_ws/studio_maps'))
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        self.http_port = int(self.get_parameter('http_port').value)
        self.pm = ProcessManager()
        n_orphans = self.pm.reap_orphans()
        if n_orphans:
            self.get_logger().warn(
                f'Reaped {n_orphans} leftover studio nav/camera process(es) '
                'from a previous launch')

        # Session
        self.session: Dict[str, Any] = {
            'map_name': '',
            'robot': 'grip',
            'mode': 'offline',  # arena | offline
            'phase': 'idle',    # idle|mapping|improve|waypoints|deploy
            'nav_stack': 'custom',  # custom | nav2
            'warnings': [],
            'slam': {
                'resolution': 0.03,
                'max_laser_range': 10.0,
                'min_laser_range': 0.05,
            },
        }
        self._estop = False
        self._pose: Optional[PoseStamped] = None
        self._pose_override: Optional[Dict[str, float]] = None
        self._pose_override_until: float = 0.0
        self._odom_pose: Optional[PoseStamped] = None
        self._map: Optional[OccupancyGrid] = None
        self._map_seq = 0
        self._map_built: Optional[Dict[str, Any]] = None
        self._map_built_seq = -1
        self._map_payload_last: Optional[Dict[str, Any]] = None
        self._map_rev = 0
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
        self._amcl_cov: float = 99.0
        self._scan: list = []
        self._trail: List[Dict[str, float]] = []
        self._trail_last = (None, None)
        self._battery: Dict[str, Any] = {
            'voltage': 0.0, 'percentage': 0.0, 'valid': False,
            'source': 'waiting_for_control_server',
        }
        self._reloc_active = False
        self._reloc_motion = ''
        self._reloc_then_task = False
        self._reloc_t0 = 0.0
        self._reloc_ok_streak = 0
        self._reloc_start_after: Optional[float] = None
        self._reloc_pending_wps: Optional[list] = None
        self._reloc_result = ''  # '', 'running', 'ok', 'failed'
        self._path_style = 'smoothed'
        self._amcl_stamp = 0.0
        self._scan_stamp = 0.0
        self._scan_msg_stamp: Optional[Time] = None
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._cam_ui = False
        self._cam_action_need = False
        self._cam_forbid_autostart = False  # set by Kill camera node
        self._cam_stop_after = 0.0
        self._cam_logs: List[str] = []
        self._cam_launch_backoff_until = 0.0
        self._cam_grace_until = 0.0
        self._cam_process_started_at = 0.0
        self._cam_deploy_requested = False
        self._cam_deploy_launch_after = 0.0
        self._sim_path: Optional[Path] = None
        self._sim_path_smooth: Optional[Path] = None
        self._sim_result: Dict[str, Any] = {}
        self._sim_req_id = 0
        self._pending_nav2_wps: Optional[list] = None
        self._armed_wps: List[dict] = []
        self._armed_seq_name = ''
        self._mission_started_from = ''  # 'dashboard' | 'physical' | ''
        self._hw_alert = ''
        self._hw_start_prev = False
        self._hw_stop_prev = False
        self._hw_start_cool = 0.0
        self._hw_stop_cool = 0.0
        self._NAV2_WP_TRIES = 2
        self._nav2_seq_gen = 0
        self._nav2_seq_active = False
        self._nav2_seq_wps: List[dict] = []
        self._nav2_seq_index = 0
        self._nav2_seq_attempt = 1
        self._nav2_seq_ok = 0
        self._nav2_seq_skipped = 0
        self._nav2_wait_until = 0.0

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, qos)
        self.create_subscription(Path, '/custom_nav/path', self._on_path, 10)
        self.create_subscription(Path, '/plan', self._on_path, 10)
        self.create_subscription(Path, '/custom_nav/planner_path', self._on_planner, 10)
        self.create_subscription(Path, '/custom_nav/smoothed_path', self._on_smoothed, 10)
        self.create_subscription(Path, '/custom_nav/sim/path', self._on_sim_path, 10)
        self.create_subscription(
            Path, '/custom_nav/sim/path_smooth', self._on_sim_path_smooth, 10)
        self.create_subscription(String, '/custom_nav/sim/result', self._on_sim_result, 10)
        self.create_subscription(String, '/custom_nav/mission/status', self._on_mission, 10)
        self.create_subscription(String, '/custom_nav/recovery', self._on_recovery, 10)
        self.create_subscription(String, '/custom_nav/collision', self._on_collision, 10)
        self.create_subscription(String, '/custom_nav/goal_status', self._on_goal_status, 10)
        self.create_subscription(
            String, '/custom_nav/node_status', self._on_node_status,
            QoSProfile(
                depth=20,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ))
        self.create_subscription(Bool, '/custom_nav/world/ready', self._on_ready, qos)
        self.create_subscription(BatteryState, '/battery_state', self._on_battery, 10)
        self.create_subscription(
            LaserScan, str(self.get_parameter('scan_topic').value),
            self._on_scan, qos_profile_sensor_data)

        self._initialpose = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self._goal_pub = self.create_publisher(PoseStamped, '/custom_nav/goal', 10)
        self._goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self._nav_to_pose = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._nav_through = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses')
        self._compute_path = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self._clear_local_cli = self.create_client(
            ClearEntireCostmap, 'local_costmap/clear_entirely_local_costmap')
        self._clear_global_cli = self.create_client(
            ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap')
        self._nav2_goal_handle = None
        self._goals_pub = self.create_publisher(String, '/custom_nav/goals', 10)
        self._start_pub = self.create_publisher(Empty, '/custom_nav/mission/start', 10)
        self._stop_pub = self.create_publisher(Empty, '/custom_nav/mission/stop', 10)
        self._map_load = self.create_publisher(String, '/custom_nav/map/load', 10)
        self._cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self._ctrl_pub = self.create_publisher(
            Bool, '/custom_nav/controller/enable',
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ))
        self._path_style_pub = self.create_publisher(String, '/custom_nav/path_style', 10)
        self._sim_req_pub = self.create_publisher(
            String, '/custom_nav/sim/plan_request', 10)
        # Public intent signal: subscribers (laptop vision) can use it to know
        # when Studio wants frames. Studio itself never subscribes to images.
        self._cam_need_pub = self.create_publisher(Bool, '/studio/camera/need', 10)
        self._toggle_color_cli = self.create_client(SetBool, '/camera/toggle_color')
        self._toggle_depth_cli = self.create_client(SetBool, '/camera/toggle_depth')
        self._cam_need_last: Optional[bool] = None
        self._cam_need_depth = False
        self._cam_depth_running = False
        self._cam_stream = None
        self._cam_depth_stream = None
        self._toggle_inflight = False

        # No image subscriptions here on purpose: Orbbec publishes on its own
        # topics (/camera/...) and consumers subscribe to those directly, so the
        # Pi never decodes or re-encodes frames for the dashboard.
        self.create_subscription(String, '/studio/vision/track', self._on_vision_track, 10)
        # Momentary Titan1 limit switches (false → true pulse → false).
        # limit_rev = physical START, limit_fwd = physical STOP.
        self.create_subscription(
            Bool, '/titan1/m_3/limit_rev', self._on_hw_start_btn, 10)
        self.create_subscription(
            Bool, '/titan1/m_3/limit_fwd', self._on_hw_stop_btn, 10)
        self.create_subscription(Bool, '/studio/camera/action_need', self._on_cam_action, 10)
        self.create_subscription(Bool, '/studio/camera/need_depth', self._on_cam_need_depth, 10)

        self.create_timer(0.1, self._reloc_tick)
        self.create_timer(0.1, self._nav2_seq_tick)
        self.create_timer(0.2, self._trail_tick)
        self.create_timer(0.5, self._cam_tick)
        maps.ensure_root()
        threading.Thread(target=self._run_http, daemon=True).start()
        self.get_logger().info(f'Studio bridge HTTP :{self.http_port}')

    # ── ROS callbacks ───────────────────────────────────────────────────
    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        ps = pose_cov_to_stamped(msg)
        x = float(ps.pose.position.x)
        y = float(ps.pose.position.y)
        if abs(x) > 50.0 or abs(y) > 50.0:
            return
        self._pose = ps
        self._amcl_ok = True
        cov = msg.pose.covariance
        self._amcl_cov = float(cov[0] + cov[7]) if len(cov) > 7 else 99.0
        self._amcl_stamp = time.time()

    def _on_odom(self, msg: Odometry) -> None:
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self._odom_pose = ps

    def _append_trail(self, x: float, y: float) -> None:
        lx, ly = self._trail_last
        if lx is None or math.hypot(x - lx, y - ly) > 0.05:
            self._trail.append({'x': round(x, 3), 'y': round(y, 3)})
            if len(self._trail) > 4000:
                self._trail = self._trail[-3000:]
            self._trail_last = (x, y)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg
        self._map_seq += 1

    def _clear_nav_paths(self) -> None:
        self._path = None
        self._planner_path = None
        self._smoothed_path = None

    def _on_path(self, msg: Path) -> None:
        self._path = msg

    def _on_planner(self, msg: Path) -> None:
        if self._using_nav2():
            return
        self._planner_path = msg

    def _on_smoothed(self, msg: Path) -> None:
        if self._using_nav2():
            return
        self._smoothed_path = msg

    def _on_sim_path(self, msg: Path) -> None:
        self._sim_path = msg

    def _on_sim_path_smooth(self, msg: Path) -> None:
        self._sim_path_smooth = msg

    def _on_sim_result(self, msg: String) -> None:
        try:
            self._sim_result = json.loads(msg.data)
        except Exception:
            pass

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
        self._goal_status = msg.data

    def _using_nav2(self) -> bool:
        return str(self.session.get('nav_stack') or '') == 'nav2'

    def _nav2_compute_path_sync(
        self,
        sx: float,
        sy: float,
        syaw: float,
        gx: float,
        gy: float,
        gyaw: float,
        *,
        timeout_sec: float = 12.0,
    ) -> Dict[str, Any]:
        """One-shot Theta* preview via Nav2 compute_path_to_pose."""
        if not self._compute_path.wait_for_server(timeout_sec=2.0):
            return {'ok': False, 'detail': 'compute_path_to_pose not ready', 'points': 0}

        goal = ComputePathToPose.Goal()
        goal.planner_id = 'GridBased'
        goal.use_start = True
        stamp = self.get_clock().now().to_msg()

        goal.start = PoseStamped()
        goal.start.header.frame_id = 'map'
        goal.start.header.stamp = stamp
        goal.start.pose.position.x = float(sx)
        goal.start.pose.position.y = float(sy)
        qx, qy, qz, qw = quat_from_yaw(float(syaw))
        goal.start.pose.orientation.x = qx
        goal.start.pose.orientation.y = qy
        goal.start.pose.orientation.z = qz
        goal.start.pose.orientation.w = qw

        goal.goal = PoseStamped()
        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = stamp
        goal.goal.pose.position.x = float(gx)
        goal.goal.pose.position.y = float(gy)
        qx, qy, qz, qw = quat_from_yaw(float(gyaw))
        goal.goal.pose.orientation.x = qx
        goal.goal.pose.orientation.y = qy
        goal.goal.pose.orientation.z = qz
        goal.goal.pose.orientation.w = qw

        done = threading.Event()
        box: Dict[str, Any] = {'path': None, 'error': None}

        def _on_sent(fut):
            try:
                handle = fut.result()
            except Exception as exc:
                box['error'] = str(exc)
                done.set()
                return
            if handle is None or not handle.accepted:
                box['error'] = 'rejected'
                done.set()
                return

            def _on_result(res_fut):
                try:
                    box['path'] = res_fut.result().result.path
                except Exception as exc:
                    box['error'] = str(exc)
                done.set()

            handle.get_result_async().add_done_callback(_on_result)

        self._compute_path.send_goal_async(goal).add_done_callback(_on_sent)
        if not done.wait(timeout_sec):
            return {'ok': False, 'detail': 'plan timeout', 'points': 0}
        if box['error']:
            return {'ok': False, 'detail': str(box['error']), 'points': 0}
        path = box['path']
        if path is None or len(path.poses) < 2:
            return {'ok': False, 'detail': 'no path', 'points': 0}
        return {
            'ok': True,
            'detail': f'Nav2 Theta* ({len(path.poses)} pts)',
            'points': len(path.poses),
            'path': path,
        }

    def _pose_stamped(self, x: float, y: float, yaw: float = 0.0) -> PoseStamped:
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        qx, qy, qz, qw = quat_from_yaw(float(yaw))
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        return ps

    def _cancel_nav2(self) -> None:
        """Stop the whole Nav2 mission sequence and cancel the active goal."""
        self._nav2_seq_active = False
        self._nav2_seq_gen += 1
        self._nav2_wait_until = 0.0
        self._cancel_nav2_goal_only()
        if str((self._mission or {}).get('state', '')).lower() in (
                'running', 'wait_goal', 'wait_action', 'recovering'):
            self._nav2_set_mission('idle', 'stopped')

    def _cancel_nav2_goal_only(self) -> None:
        """Cancel current navigate_to_pose handle without ending the mission."""
        handle = self._nav2_goal_handle
        self._nav2_goal_handle = None
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass

    @staticmethod
    def _nav2_wait_seconds(wp: dict) -> float:
        action = (wp or {}).get('action')
        if not isinstance(action, dict) or action.get('type') != 'wait':
            return 0.0
        try:
            seconds = float(action.get('seconds', 0.0))
        except (TypeError, ValueError):
            return 0.0
        if not math.isfinite(seconds):
            return 0.0
        return min(3600.0, max(0.0, seconds))

    def _nav2_set_mission(self, state: str, detail: str = '') -> None:
        n = len(self._nav2_seq_wps)
        idx = self._nav2_seq_index
        remaining = 0.0
        if self._nav2_wait_until > 0.0:
            remaining = max(0.0, self._nav2_wait_until - time.time())
        current = self._nav2_seq_wps[idx] if 0 <= idx < n else None
        self._mission = {
            'state': state,
            'index': idx,
            'total': n,
            'wp_idx': idx,
            'wp_total': n,
            'current': current,
            'wait_remaining': round(remaining, 1),
            'detail': detail,
            'started_from': self._mission_started_from,
            'sequence': self._armed_seq_name,
            'armed_count': len(self._armed_wps),
            'hw_alert': self._hw_alert,
        }

    def _send_nav2_pose(self, ps: PoseStamped) -> None:
        # Action client only — do NOT also publish /goal_pose (bt_navigator
        # listens to that topic and would start a second NavigateToPose).
        if not self._nav_to_pose.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn('Nav2 navigate_to_pose not ready')
            return
        goal = NavigateToPose.Goal()
        goal.pose = ps
        self._cancel_nav2()
        fut = self._nav_to_pose.send_goal_async(goal)

        def _store(done):
            try:
                self._nav2_goal_handle = done.result()
            except Exception:
                self._nav2_goal_handle = None
        fut.add_done_callback(_store)

    def _send_nav2_through(self, waypoints: list) -> None:
        """One NavigateToPose at a time. Wait at the waypoint, then plan the next."""
        wps = [w for w in waypoints if isinstance(w, dict)]
        if not wps:
            raise ValueError('no valid waypoints')
        self._cancel_nav2()
        self._nav2_seq_wps = wps
        self._nav2_seq_index = 0
        self._nav2_seq_attempt = 1
        self._nav2_seq_ok = 0
        self._nav2_seq_skipped = 0
        self._nav2_wait_until = 0.0
        self._nav2_seq_active = True
        self.get_logger().info(
            f'Nav2 mission {len(wps)} waypoint(s) — one pose at a time, '
            f'{self._NAV2_WP_TRIES} tries each, wait-then-plan next')
        self._send_nav2_seq_pose()

    def _wp_pose(self, wp: dict) -> PoseStamped:
        return self._pose_stamped(
            float(wp.get('x', 0)), float(wp.get('y', 0)), float(wp.get('yaw', 0)))

    def _send_nav2_seq_pose(self) -> None:
        if not self._nav2_seq_active:
            return
        wps = self._nav2_seq_wps
        idx = self._nav2_seq_index
        if idx >= len(wps):
            self._nav2_seq_active = False
            self._nav2_wait_until = 0.0
            self._nav2_set_mission('done', 'mission complete')
            self.get_logger().info(
                f'Nav2 mission complete — {self._nav2_seq_ok} reached, '
                f'{self._nav2_seq_skipped} skipped')
            return
        wp = wps[idx]
        ps = self._wp_pose(wp)
        n = len(wps)
        att = self._nav2_seq_attempt
        # Invalidate any in-flight result callbacks from a previous send/preempt.
        self._nav2_seq_gen += 1
        gen = self._nav2_seq_gen
        self._cancel_nav2_goal_only()
        self._nav2_set_mission('running', f'planning wp {idx + 1}/{n} only')
        self.get_logger().info(
            f'Nav2 planning waypoint {idx + 1}/{n} only '
            f'(attempt {att}/{self._NAV2_WP_TRIES}) '
            f'({ps.pose.position.x:.2f}, {ps.pose.position.y:.2f})')
        if not self._nav_to_pose.server_is_ready():
            self.get_logger().warn(
                'Nav2 navigate_to_pose not ready — counting as a failed try')
            self._nav2_seq_on_outcome(gen, GoalStatus.STATUS_ABORTED)
            return
        goal = NavigateToPose.Goal()
        goal.pose = ps
        fut = self._nav_to_pose.send_goal_async(goal)

        def _on_sent(done):
            if not self._nav2_seq_active or gen != self._nav2_seq_gen:
                return
            try:
                handle = done.result()
            except Exception:
                handle = None
            if handle is None or not handle.accepted:
                self.get_logger().warn(
                    'Nav2 navigate_to_pose goal rejected — counting as failed try')
                self._nav2_seq_on_outcome(gen, GoalStatus.STATUS_ABORTED)
                return
            self._nav2_goal_handle = handle
            handle.get_result_async().add_done_callback(
                lambda rfut, g=gen: self._on_nav2_seq_result(g, rfut))

        fut.add_done_callback(_on_sent)

    def _nav2_begin_wait_then_next(self, reached_idx: int) -> None:
        wp = self._nav2_seq_wps[reached_idx]
        wait_s = self._nav2_wait_seconds(wp)
        self._nav2_seq_index = reached_idx + 1
        self._nav2_seq_attempt = 1
        if wait_s <= 0.0:
            self._send_nav2_seq_pose()
            return
        self._nav2_wait_until = time.time() + wait_s
        n = len(self._nav2_seq_wps)
        more = self._nav2_seq_index < n
        self._nav2_set_mission('wait_action', f'wp {reached_idx + 1}/{n} wait {wait_s:g}s')
        if more:
            self.get_logger().info(
                f'Waypoint {reached_idx + 1}/{n} reached — waiting {wait_s:g}s '
                'then planning the next waypoint')
        else:
            self.get_logger().info(
                f'Waypoint {reached_idx + 1}/{n} reached — waiting {wait_s:g}s '
                'then mission complete')

    def _nav2_seq_tick(self) -> None:
        if not self._nav2_seq_active or self._nav2_wait_until <= 0.0:
            return
        remaining = self._nav2_wait_until - time.time()
        self._nav2_set_mission('wait_action', f'waiting {max(0.0, remaining):.1f}s')
        if remaining > 0.0:
            return
        self._nav2_wait_until = 0.0
        self.get_logger().info(
            'Waypoint wait complete — generating path to next waypoint')
        self._send_nav2_seq_pose()

    def _on_nav2_seq_result(self, gen: int, rfut) -> None:
        if not self._nav2_seq_active or gen != self._nav2_seq_gen:
            return
        status = GoalStatus.STATUS_ABORTED
        try:
            wrapped = rfut.result()
            status = int(wrapped.status)
        except Exception as exc:
            self.get_logger().warn(f'Nav2 waypoint result error: {exc}')
        self._nav2_seq_on_outcome(gen, status)

    def _nav2_seq_on_outcome(self, gen: int, status: int) -> None:
        if not self._nav2_seq_active or gen != self._nav2_seq_gen:
            return
        # Canceled by us (new send / stop) — ignore; only the latest gen counts.
        if status == GoalStatus.STATUS_CANCELED:
            return
        n = len(self._nav2_seq_wps)
        idx = self._nav2_seq_index
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._nav2_seq_ok += 1
            self.get_logger().info(f'Nav2 waypoint {idx + 1}/{n} reached')
            self._nav2_begin_wait_then_next(idx)
            return
        att = self._nav2_seq_attempt
        if att < self._NAV2_WP_TRIES:
            self.get_logger().warn(
                f'waypoint {idx + 1}/{n} failed attempt {att}/'
                f'{self._NAV2_WP_TRIES} — sending again')
            self._nav2_seq_attempt = att + 1
            self._send_nav2_seq_pose()
            return
        self._nav2_seq_skipped += 1
        self.get_logger().warn(
            f'waypoint skipped after {self._NAV2_WP_TRIES} Nav2 failures')
        self._nav2_seq_index += 1
        self._nav2_seq_attempt = 1
        self._send_nav2_seq_pose()

    def _call_clear_costmap(self, client) -> bool:
        if not client.wait_for_service(timeout_sec=0.5):
            return False
        req = ClearEntireCostmap.Request()
        fut = client.call_async(req)
        deadline = time.time() + 8.0
        while not fut.done() and time.time() < deadline:
            time.sleep(0.05)
        if not fut.done():
            return False
        try:
            res = fut.result()
            return bool(res.success)
        except Exception:
            return False

    def clear_nav2_costmaps(self) -> Dict[str, Any]:
        """Clear local + global Nav2 costmaps (no-op if Nav2 not running)."""
        local_ok = self._call_clear_costmap(self._clear_local_cli)
        global_ok = self._call_clear_costmap(self._clear_global_cli)
        ok = local_ok and global_ok
        if ok:
            self.get_logger().info('Nav2 costmaps cleared (local + global)')
        else:
            self.get_logger().warn(
                f'Nav2 costmap clear partial/failed local={local_ok} global={global_ok}')
        return {'ok': ok, 'local': local_ok, 'global': global_ok}

    def _on_node_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            name = str(data.get('node', ''))
            if name:
                self._nodes[name] = data
        except Exception:
            pass

    def _on_ready(self, msg: Bool) -> None:
        self._world_ready = bool(msg.data)

    def _on_battery(self, msg: BatteryState) -> None:
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

    def _cam_log(self, line: str) -> None:
        stamp = time.strftime('%H:%M:%S')
        self._cam_logs.append(f'{stamp} {line}')
        if len(self._cam_logs) > 80:
            self._cam_logs = self._cam_logs[-60:]
        self.get_logger().info(f'[camera] {line}')

    def _mission_active(self) -> bool:
        if self._nav2_seq_active:
            return True
        st = str((self._mission or {}).get('state', '')).lower()
        return st in ('running', 'wait_goal', 'wait_action', 'recovering')

    def _arm_mission(self, waypoints: Any, seq_name: str = '') -> int:
        wps = [w for w in (waypoints or []) if isinstance(w, dict)]
        self._armed_wps = wps
        self._armed_seq_name = str(seq_name or '')
        self._pending_nav2_wps = wps
        self._hw_alert = ''
        return len(wps)

    def _start_mission(self, source: str) -> Dict[str, Any]:
        """Start the armed mission. No-op if a mission is already running."""
        source = 'physical' if source == 'physical' else 'dashboard'
        if self._estop:
            raise RuntimeError('E-STOP active — release before start')
        if self._mission_active():
            self.get_logger().info(
                f'Mission start from {source} ignored — already running '
                f'(from {self._mission_started_from or "unknown"})')
            return {
                'ok': True,
                'started': False,
                'reason': 'already_running',
                'source': self._mission_started_from,
            }
        wps = list(self._armed_wps or self._pending_nav2_wps or [])
        if not wps:
            self._hw_alert = (
                'No sequence armed — select or save a sequence in Deploy')
            self._nav2_set_mission(
                str((self._mission or {}).get('state') or 'idle'),
                self._hw_alert)
            self.get_logger().warn(
                f'Mission start from {source} ignored — no sequence armed')
            return {
                'ok': False,
                'started': False,
                'reason': 'no_sequence',
                'error': self._hw_alert,
            }
        self._hw_alert = ''
        self._mission_started_from = source
        if self._using_nav2():
            self._send_nav2_through(wps)
        else:
            msg = String()
            msg.data = json.dumps({'waypoints': wps})
            self._goals_pub.publish(msg)
            self._start_pub.publish(Empty())
            self._nav2_set_mission('running', f'started from {source}')
        self.get_logger().info(
            f'Mission START from {source} — {len(wps)} waypoint(s) '
            f'seq={self._armed_seq_name or "(unnamed)"}')
        return {
            'ok': True,
            'started': True,
            'source': source,
            'count': len(wps),
            'sequence': self._armed_seq_name,
        }

    def _stop_mission(self, source: str) -> Dict[str, Any]:
        if self._using_nav2():
            self._cancel_nav2()
        self._stop_pub.publish(Empty())
        self._enable_ctrl(False)
        self._publish_cmd(0, 0, 0)
        self._hw_alert = ''
        self._nav2_set_mission('idle', f'stopped ({source})')
        self.get_logger().info(f'Mission STOP from {source}')
        return {'ok': True, 'source': source}

    def _on_hw_start_btn(self, msg: Bool) -> None:
        pressed = bool(msg.data)
        rising = pressed and not self._hw_start_prev
        self._hw_start_prev = pressed
        if not rising:
            return
        now = time.time()
        if now < self._hw_start_cool:
            return
        self._hw_start_cool = now + 0.4
        try:
            self._start_mission('physical')
        except Exception as exc:
            self._hw_alert = str(exc)
            self.get_logger().warn(f'Physical START ignored: {exc}')

    def _on_hw_stop_btn(self, msg: Bool) -> None:
        pressed = bool(msg.data)
        rising = pressed and not self._hw_stop_prev
        self._hw_stop_prev = pressed
        if not rising:
            return
        now = time.time()
        if now < self._hw_stop_cool:
            return
        self._hw_stop_cool = now + 0.4
        if not self._mission_active():
            self.get_logger().info('Physical STOP ignored — no mission running')
            return
        self._stop_mission('physical')

    def _cam_wanted(self) -> bool:
        return bool(self._cam_ui or self._cam_action_need)

    def _cam_stream_allowed(self) -> bool:
        """Defer color stream briefly after Orbbec process start during nav."""
        if not self._mission_active():
            return True
        if self._cam_action_need:
            return True
        return time.time() >= self._cam_grace_until

    def _on_vision_track(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        d = data.get('depth_cm')
        tgt = data.get('target_m')
        line = (
            f"vision {data.get('type', '')} {data.get('phase', '')}"
            + (f' depth={d}cm' if d is not None else '')
            + (f' stop={round(float(tgt)*100)}cm' if tgt else '')
        )
        self._cam_log(line)

    def _on_cam_action(self, msg: Bool) -> None:
        self._cam_action_need = bool(msg.data)
        self._cam_need_last = None
        self._cam_log('action camera ' + ('ON' if msg.data else 'OFF'))

    def _on_cam_need_depth(self, msg: Bool) -> None:
        self._cam_need_depth = bool(msg.data)

    def _orbbec_ok(self, res, wanted: bool) -> bool:
        if bool(getattr(res, 'success', False)):
            return True
        msg = str(getattr(res, 'message', '') or '').lower()
        if wanted and 'already on' in msg:
            return True
        if (not wanted) and 'already off' in msg:
            return True
        return False

    def _toggle_orbbec(self, *, color: bool, depth: Optional[bool] = None) -> None:
        """Start/stop Orbbec hardware streams via /camera/toggle_color (and depth)."""
        if self._toggle_inflight:
            return
        color_same = self._cam_stream is not None and color == self._cam_stream
        depth_same = (
            depth is None
            or (self._cam_depth_stream is not None and depth == self._cam_depth_stream)
        )
        if color_same and depth_same:
            return
        if not self._toggle_color_cli.service_is_ready():
            return

        def _done(fut, kind: str, wanted: bool) -> None:
            try:
                res = fut.result()
                msg = str(getattr(res, 'message', '') or '')
                ok = self._orbbec_ok(res, wanted)
                if kind == 'color':
                    prev = self._cam_stream
                    if ok:
                        self._cam_stream = bool(wanted)
                    if ok and prev is not wanted:
                        self._cam_log(
                            f'toggle_color={"ON" if wanted else "OFF"} ok {msg}'.strip())
                    elif not ok:
                        self._cam_log(
                            f'toggle_color={"ON" if wanted else "OFF"} FAIL {msg}'.strip())
                elif kind == 'depth':
                    if ok:
                        self._cam_depth_stream = bool(wanted)
                    if not ok:
                        self._cam_log(
                            f'toggle_depth={"ON" if wanted else "OFF"} FAIL {msg}'.strip())
            except Exception as extra:
                self._cam_log(f'toggle_{kind} error: {extra}')
            finally:
                self._toggle_inflight = False

        if not color_same:
            self._toggle_inflight = True
            req = SetBool.Request()
            req.data = bool(color)
            fut = self._toggle_color_cli.call_async(req)
            fut.add_done_callback(lambda f, w=bool(color): _done(f, 'color', w))
        if depth is not None and not depth_same and self._toggle_depth_cli.service_is_ready():
            dreq = SetBool.Request()
            dreq.data = bool(depth)
            dfut = self._toggle_depth_cli.call_async(dreq)
            dfut.add_done_callback(lambda f, w=bool(depth): _done(f, 'depth', w))

    def _ensure_orbbec(self, *, force: bool = False) -> None:
        """Launch Orbbec node on demand (never at deploy unless camera wanted)."""
        if self._cam_forbid_autostart:
            return
        if not force and not self._cam_wanted():
            return
        if time.time() < self._cam_launch_backoff_until:
            return
        if self.pm.running('camera'):
            return
        try:
            mp = self.pm.start_camera(enable_depth=False)
            self._cam_depth_running = False
            self._cam_stream = None
            self._cam_depth_stream = None
            now = time.time()
            self._cam_process_started_at = now
            # USB init + Orbbec startup can starve lidar/AMCL — grace before stream.
            self._cam_grace_until = now + (4.0 if self._mission_active() else 1.5)
            self._cam_log(
                f'Orbbec node launched pid={mp.popen.pid} log={mp.log_path}')
            if self._mission_active() and not self._cam_action_need:
                self._cam_log(
                    f'stream deferred {self._cam_grace_until - now:.0f}s '
                    '(mission active — protect AMCL/lidar)')
            else:
                self._cam_log('waiting for /camera/toggle_color')
        except Exception as exc:
            self._cam_launch_backoff_until = time.time() + 20.0
            self._cam_log(f'Orbbec launch failed: {exc}')

    def _stop_orbbec(self) -> None:
        if self.pm.running('camera'):
            self._toggle_orbbec(color=False, depth=False)
            self.pm.stop_camera()
            self._cam_log('Orbbec node stopped')
        self._cam_stream = None
        self._cam_depth_stream = None
        self._cam_depth_running = False

    def _cam_tick(self) -> None:
        deploy = str(self.session.get('phase') or '') == 'deploy'
        wanted = self._cam_wanted()
        if (
            deploy
            and self._cam_deploy_requested
            and time.time() >= self._cam_deploy_launch_after
        ):
            self._ensure_orbbec(force=True)
            if self.pm.running('camera'):
                self._cam_deploy_requested = False
                self._cam_log(
                    'Deploy with camera — Orbbec node ready; stream remains OFF '
                    'until Camera ON or a vision action')
        if self._cam_need_last is not wanted:
            self._cam_need_pub.publish(Bool(data=wanted))
            self._cam_need_last = wanted

        if deploy:
            if wanted:
                self._ensure_orbbec()
            if wanted and self._cam_stream_allowed():
                self._toggle_orbbec(
                    color=True,
                    depth=True if self._cam_need_depth else None)
            else:
                self._toggle_orbbec(color=False)
            return

        if wanted:
            self._ensure_orbbec()
            if self._cam_stream_allowed():
                self._toggle_orbbec(color=True)

    def _on_scan(self, msg: LaserScan) -> None:
        pts = []
        angle = msg.angle_min
        step = max(1, len(msg.ranges) // 180)
        for i in range(0, len(msg.ranges), step):
            r = float(msg.ranges[i])
            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                pts.append({
                    'x': round(r * math.cos(angle), 3),
                    'y': round(r * math.sin(angle), 3),
                })
            angle += msg.angle_increment * step
        self._scan = pts
        self._scan_stamp = time.time()
        self._scan_msg_stamp = Time.from_msg(msg.header.stamp)

    @staticmethod
    def _pose_from_transform(t) -> PoseStamped:
        ps = PoseStamped()
        ps.header = t.header
        ps.pose.position.x = t.transform.translation.x
        ps.pose.position.y = t.transform.translation.y
        ps.pose.position.z = t.transform.translation.z
        ps.pose.orientation = t.transform.rotation
        return ps

    def _try_map_pose(self, stamp: Optional[Time] = None) -> Optional[PoseStamped]:
        """Robot pose in map frame — same chain RViz uses (map→base_footprint)."""
        when = stamp if stamp is not None else Time()
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                when,
                timeout=Duration(seconds=0.05))
        except TransformException:
            return None
        x = float(t.transform.translation.x)
        y = float(t.transform.translation.y)
        if abs(x) > 50.0 or abs(y) > 50.0:
            return None
        return self._pose_from_transform(t)

    def _trail_tick(self) -> None:
        ps = self._try_map_pose()
        if ps is None:
            return
        self._append_trail(
            float(ps.pose.position.x), float(ps.pose.position.y))

    def _publish_inflated_initialpose(self) -> None:
        """Re-seed AMCL with large cov at current pose so reloc must re-converge."""
        ps = self._pose or self._odom_pose
        if ps is None:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose = ps.pose
        # Large uncertainty — particles spread; spin must tighten cov again
        msg.pose.covariance[0] = 0.55
        msg.pose.covariance[7] = 0.55
        msg.pose.covariance[35] = 0.40
        self._initialpose.publish(msg)
        self._amcl_ok = False
        self._amcl_cov = 99.0

    def _reloc_tick(self) -> None:
        now = time.time()
        if self._reloc_start_after is not None and now >= self._reloc_start_after:
            self._reloc_start_after = None
            try:
                self._start_mission('dashboard')
            except Exception as exc:
                self.get_logger().warn(f'Reloc then-task start failed: {exc}')
            return

        if not self._reloc_active or self._estop:
            return

        elapsed = now - self._reloc_t0
        self._reloc_result = 'running'

        # Drive motion
        tw = Twist()
        m = self._reloc_motion
        if m == 'spin':
            tw.angular.z = 0.40
        elif m == 'front':
            tw.linear.x = 0.12
        elif m == 'back':
            tw.linear.x = -0.12
        self._cmd_vel.publish(tw)

        # Timeout
        if elapsed > 45.0:
            self._reloc_active = False
            self._reloc_then_task = False
            self._reloc_pending_wps = None
            self._reloc_result = 'failed'
            self._publish_cmd(0, 0, 0)
            self.get_logger().warn(
                f'Relocalization FAILED after {elapsed:.1f}s '
                f'(cov={self._amcl_cov:.3f})')
            return

        # Must move first so laser updates reshape the particle cloud
        min_motion = 3.0 if m == 'spin' else 2.0
        if elapsed < min_motion:
            self._reloc_ok_streak = 0
            return

        amcl_fresh = (now - self._amcl_stamp) < 1.0
        scan_fresh = (now - self._scan_stamp) < 1.0 and len(self._scan) >= 8
        # xx+yy covariance; well-localized typically << 0.1
        cov_ok = self._amcl_ok and self._amcl_cov < 0.10
        good = cov_ok and amcl_fresh and scan_fresh
        if good:
            self._reloc_ok_streak += 1
        else:
            self._reloc_ok_streak = 0

        # ~1.5 s of sustained good pose @ 10 Hz
        if self._reloc_ok_streak < 15:
            return

        self._reloc_active = False
        self._reloc_result = 'ok'
        self._publish_cmd(0, 0, 0)
        self.get_logger().info(
            f'Relocalization succeeded after {elapsed:.1f}s '
            f'(cov={self._amcl_cov:.4f})')

        if self._reloc_then_task:
            self._reloc_then_task = False
            wps = self._reloc_pending_wps
            self._reloc_pending_wps = None
            if wps:
                self._arm_mission(wps, self._armed_seq_name or 'reloc')
                self.get_logger().info(
                    f'Reloc OK — armed {len(wps)} waypoint(s), starting soon')
            # Brief delay so mission manager / Nav2 receive goals
            self._reloc_start_after = now + 0.35
        return

    def _begin_reloc(
        self,
        motion: str,
        *,
        then_task: bool = False,
        waypoints: Optional[list] = None,
    ) -> Dict[str, Any]:
        if motion not in ('spin', 'front', 'back'):
            motion = 'spin'
        self._enable_ctrl(False)
        self._publish_cmd(0, 0, 0)
        self._publish_inflated_initialpose()
        self._reloc_motion = motion
        self._reloc_active = True
        self._reloc_then_task = then_task
        self._reloc_pending_wps = list(waypoints) if waypoints else None
        self._reloc_t0 = time.time()
        self._reloc_ok_streak = 0
        self._reloc_start_after = None
        self._reloc_result = 'running'
        self.get_logger().info(
            f'Reloc START motion={motion} then_task={then_task} '
            f'wps={len(waypoints or [])}')
        return {
            'ok': True,
            'motion': motion,
            'then_task': then_task,
            'result': 'running',
        }

    def _publish_cmd(self, vx: float, vy: float, wz: float) -> None:
        if self._estop:
            vx = vy = wz = 0.0
        tw = Twist()
        tw.linear.x = float(vx)
        tw.linear.y = float(vy)
        tw.angular.z = float(wz)
        self._cmd_vel.publish(tw)

    def _enable_ctrl(self, on: bool) -> None:
        msg = Bool()
        msg.data = bool(on)
        self._ctrl_pub.publish(msg)

    def _live_pose(self) -> Optional[Dict[str, float]]:
        # After Set pose, briefly prefer the clicked pose so the HUD does not
        # keep drawing the stale map→base TF until AMCL finishes updating.
        if (
            self._pose_override is not None
            and time.time() < self._pose_override_until
        ):
            return dict(self._pose_override)
        self._pose_override = None
        # Prefer TF map→base so scan overlay matches RViz. During SLAM mapping
        # /odom is in odom frame while /map is in map frame — using odom here
        # made green scan dots drift off walls even when RViz looked correct.
        stamp = None
        if self._scan_msg_stamp is not None and (time.time() - self._scan_stamp) < 0.5:
            stamp = self._scan_msg_stamp
        ps = self._try_map_pose(stamp) or self._try_map_pose() or self._pose or self._odom_pose
        if ps is None:
            return None
        return {
            'x': round(float(ps.pose.position.x), 3),
            'y': round(float(ps.pose.position.y), 3),
            'yaw': round(_yaw_of(ps), 3),
        }

    @staticmethod
    def _path_pts(path: Optional[Path], limit: int = 250) -> list:
        if path is None:
            return []
        return [
            {
                'x': round(float(p.pose.position.x), 3),
                'y': round(float(p.pose.position.y), 3),
            }
            for p in path.poses[:limit]
        ]

    def _map_payload(self) -> Optional[Dict[str, Any]]:
        """Build the UI map payload, reusing the last build when unchanged.

        Serialising the grid is by far the most expensive part of a telemetry
        frame, and the map only changes while SLAM is running.
        """
        if self._map is not None:
            if self._map_seq != self._map_built_seq:
                self._map_built = self._build_live_map(self._map)
                self._map_built_seq = self._map_seq
            return self._map_built
        # Offline / no live /map — serve project files (cached per file version)
        name = self.session.get('map_name') or ''
        if name:
            return maps.load_map_grid(name)
        return None

    def _map_revision(self, payload: Optional[Dict[str, Any]]) -> int:
        """Bump a counter whenever the payload object itself changes."""
        if payload is not self._map_payload_last:
            self._map_payload_last = payload
            self._map_rev += 1
        return self._map_rev

    @staticmethod
    def _build_live_map(m: OccupancyGrid) -> Dict[str, Any]:
        # Downsample for SSE bandwidth
        w, h = m.info.width, m.info.height
        stride = 2 if w * h > 120_000 else 1
        arr = np.asarray(m.data, dtype=np.int16).reshape(h, w)
        if stride > 1:
            arr = arr[::stride, ::stride]
        height, width = arr.shape
        return {
            'width': width,
            'height': height,
            'resolution': float(m.info.resolution) * stride,
            'origin': {
                'x': float(m.info.origin.position.x),
                'y': float(m.info.origin.position.y),
            },
            'data': arr.reshape(-1).tolist(),
        }

    def snapshot(self, last_map_rev: Optional[int] = None) -> Dict[str, Any]:
        map_payload = self._map_payload()
        map_rev = self._map_revision(map_payload)
        # The grid dominates the frame size; a stream that already has this
        # revision keeps drawing its copy.
        if last_map_rev is not None and last_map_rev == map_rev:
            map_payload = None

        warnings = list(self.session.get('warnings') or [])
        if self.session.get('phase') == 'mapping':
            warnings = warnings or [
                'Drive slowly while mapping',
                'Keep lidars clear of people and clutter',
                'Loop-close corridors for a better map',
            ]
        return {
            'ts': time.time(),
            'session': dict(self.session),
            'processes': self.pm.list_running(),
            'stacks': self.pm.stacks_alive(),
            'pose': self._live_pose(),
            'amcl': {
                'ok': self._amcl_ok,
                'cov': round(self._amcl_cov, 4),
                'age': round(max(0.0, time.time() - self._amcl_stamp), 2)
                if self._amcl_stamp else None,
            },
            'world_ready': self._world_ready,
            'map': map_payload,
            'map_rev': map_rev,
            'scan': self._scan,
            'scan_count': len(self._scan),
            'scan_age': round(max(0.0, time.time() - self._scan_stamp), 2)
            if self._scan_stamp else None,
            'trail': self._trail[-500:],
            'nav_stack': self.session.get('nav_stack', 'custom'),
            'path': self._path_pts(self._path),
            'planner_path': (
                [] if self._using_nav2() else self._path_pts(self._planner_path)
            ),
            'smoothed_path': (
                [] if self._using_nav2() else self._path_pts(self._smoothed_path)
            ),
            'sim': {
                'path': self._path_pts(self._sim_path),
                'path_smooth': (
                    [] if self._using_nav2()
                    else self._path_pts(self._sim_path_smooth)
                ),
                'result': dict(self._sim_result),
                'stack': self.session.get('nav_stack', 'custom'),
            },
            'mission': {
                **(self._mission or {}),
                'started_from': self._mission_started_from,
                'sequence': self._armed_seq_name,
                'armed_count': len(self._armed_wps),
                'hw_alert': self._hw_alert,
            },
            'recovery': self._recovery,
            'collision': self._collision,
            'goal_status': self._goal_status,
            'nodes': (
                self.pm.nav2_component_status()
                if self._using_nav2()
                else dict(self._nodes)
            ),
            'battery': self._battery,
            'estop': self._estop,
            'reloc': {
                'active': self._reloc_active,
                'motion': self._reloc_motion,
                'result': self._reloc_result,
                'elapsed': round(time.time() - self._reloc_t0, 1)
                if self._reloc_active and self._reloc_t0 else 0.0,
                'then_task': self._reloc_then_task,
            },
            'path_style': self._path_style,
            'warnings': warnings,
            'robot_dims': self._robot_dims(),
            'logs': self._process_logs(),
            'camera': {
                'on': self._cam_wanted(),
                'ui': self._cam_ui,
                'action': self._cam_action_need,
                'killed': bool(self._cam_forbid_autostart),
                'process': self.pm.running('camera'),
                'stream': bool(self._cam_stream),
                'depth_stream': bool(self._cam_depth_stream),
                'toggle_ready': self._toggle_color_cli.service_is_ready(),
                # Consumers subscribe to these Orbbec topics directly.
                'topics': {
                    'color': '/camera/color/image_raw',
                    'color_compressed': '/camera/color/image_raw/compressed',
                    'depth': '/camera/depth/image_raw',
                },
                'logs': list(self._cam_logs[-40:]),
            },
        }

    def _robot_dims(self) -> Dict[str, float]:
        robot = self.session.get('robot') or 'grip'
        if robot == 'mecanum':
            return {'length': 0.40, 'width': 0.40}
        return {'length': 0.43, 'width': 0.40}

    def _process_logs(self, max_lines: int = 40) -> Dict[str, str]:
        out: Dict[str, str] = {}
        log_dir = FsPath(os.path.expanduser('~/skill23_ws/studio_maps/_logs'))
        for name in ('slam', 'nav', 'localize', 'camera'):
            path = log_dir / f'{name}.log'
            try:
                # Read only the tail: nav.log grows to megabytes during a run
                # and telemetry asks for this ~3x/s.
                with open(path, 'rb') as fh:
                    fh.seek(0, os.SEEK_END)
                    size = fh.tell()
                    fh.seek(max(0, size - _LOG_TAIL_BYTES))
                    raw = fh.read().decode('utf-8', errors='replace')
                lines = raw.replace('\x00', '').splitlines()
                if size > _LOG_TAIL_BYTES and lines:
                    lines = lines[1:]  # drop the partial first line
                out[name] = '\n'.join(lines[-max_lines:])
            except OSError:
                out[name] = ''
        running = self.pm.list_running()
        out['running'] = ', '.join(running) if running else '(none)'
        return out

    # ── HTTP ─────────────────────────────────────────────────────────────
    def _run_http(self) -> None:
        bridge = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):  # noqa: N802
                return

            def _cors(self) -> None:
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')

            def _json(self, code: int, payload: Any) -> None:
                body = json.dumps(payload).encode()
                self.send_response(code)
                self._cors()
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _read_json(self) -> Dict[str, Any]:
                n = int(self.headers.get('Content-Length') or 0)
                if n <= 0:
                    return {}
                raw = self.rfile.read(n)
                try:
                    return json.loads(raw.decode() or '{}')
                except Exception:
                    return {}

            def do_OPTIONS(self):  # noqa: N802
                self.send_response(204)
                self._cors()
                self.end_headers()

            def do_GET(self):  # noqa: N802
                path = urlparse(self.path).path
                if path in ('/api/status', '/api/telemetry'):
                    self._json(200, bridge.snapshot())
                elif path == '/api/stream':
                    self.send_response(200)
                    self._cors()
                    self.send_header('Content-Type', 'text/event-stream')
                    self.send_header('Cache-Control', 'no-cache')
                    self.send_header('Connection', 'keep-alive')
                    self.end_headers()
                    try:
                        last_map_rev: Optional[int] = None
                        while True:
                            snap = bridge.snapshot(last_map_rev=last_map_rev)
                            last_map_rev = snap.get('map_rev')
                            data = json.dumps(snap)
                            self.wfile.write(f'data: {data}\n\n'.encode())
                            self.wfile.flush()
                            time.sleep(0.35)
                    except Exception:
                        return
                elif path == '/api/maps':
                    self._json(200, {'maps': maps.list_projects()})
                elif path == '/api/map':
                    qs = parse_qs(urlparse(self.path).query)
                    # Prefer live map; else load from project file metadata only
                    payload = bridge._map_payload()
                    if payload is None and qs.get('name'):
                        self._json(200, {'ok': True, 'map': None, 'file': qs['name'][0]})
                    else:
                        self._json(200, {'ok': True, 'map': payload})
                elif path == '/api/waypoints':
                    name = bridge.session.get('map_name') or ''
                    self._json(200, {'waypoints': maps.load_waypoints(name) if name else []})
                elif path == '/api/restrictions':
                    name = bridge.session.get('map_name') or ''
                    self._json(200, {
                        'rectangles': maps.load_restrictions(name) if name else [],
                    })
                elif path == '/api/logs':
                    self._json(200, bridge._process_logs(80))
                else:
                    self._json(404, {'error': 'not found'})

            def do_POST(self):  # noqa: N802
                path = urlparse(self.path).path
                body = self._read_json()
                try:
                    result = bridge.handle_post(path, body)
                    self._json(200, result)
                except Exception as exc:
                    bridge.get_logger().error(f'API {path}: {exc}')
                    self._json(400, {'ok': False, 'error': str(exc)})

        server = ThreadingHTTPServer(('0.0.0.0', self.http_port), Handler)
        server.serve_forever()

    def _parse_slam_config(self, body: Dict[str, Any]) -> Dict[str, float]:
        prev = self.session.get('slam') or {}
        resolution = float(body.get('resolution', prev.get('resolution', 0.03)))
        max_laser_range = float(
            body.get('max_laser_range', prev.get('max_laser_range', 10.0)))
        min_laser_range = float(
            body.get('min_laser_range', prev.get('min_laser_range', 0.05)))
        return {
            'resolution': resolution,
            'max_laser_range': max_laser_range,
            'min_laser_range': min_laser_range,
        }

    def handle_post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        if path == '/api/session':
            name = str(body.get('map_name', '')).strip()
            robot = str(body.get('robot', 'grip')).strip()
            mode = str(body.get('mode', 'offline')).strip()
            if mode not in ('arena', 'offline'):
                mode = 'offline'
            if robot not in ('allterrain', 'grip', 'mecanum', 'xdrive'):
                raise ValueError('invalid robot')
            if mode == 'offline':
                # Backend guarantee: offline mode is file-only. Tear down any
                # live ROS stack, but skip the expensive reap when nothing is
                # running (map-list clicks used to wait many seconds on empty
                # pkill sweeps).
                self._cam_deploy_requested = False
                alive = self.pm.stacks_alive()
                if any(alive.get(k) for k in ('nav', 'localize', 'slam', 'camera')):
                    self.pm.stop_all(timeout=0.4)
                self._cam_ui = False
                self._cam_action_need = False
                self._pose = None
                self._amcl_ok = False
                self._scan = []
                self.session['phase'] = 'mode'
            if name:
                maps.write_meta(name, robot=robot, mode=mode, created_at=time.time())
            self.session.update({
                'map_name': name or self.session.get('map_name', ''),
                'robot': robot,
                'mode': mode,
            })
            return {'ok': True, 'session': self.session}

        if path == '/api/mapping/start':
            robot = str(body.get('robot') or self.session.get('robot') or 'grip')
            name = str(body.get('map_name') or self.session.get('map_name') or '').strip()
            if not name:
                raise ValueError('map_name required')
            slam_cfg = self._parse_slam_config(body)
            slam_path = maps.write_slam_params(name, **slam_cfg)
            self.session.update({
                'map_name': name, 'robot': robot, 'phase': 'mapping',
                'slam': slam_cfg,
            })
            maps.write_meta(name, robot=robot, phase='mapping', **slam_cfg)
            self._trail = []
            self._trail_last = (None, None)
            mp = self.pm.start_slam(robot, str(slam_path))
            self.get_logger().info(
                f'Mapping start robot={robot} map={name} '
                f'pid={mp.popen.pid} log={mp.log_path} cmd={" ".join(mp.cmd)}')
            return {
                'ok': True,
                'phase': 'mapping',
                'slam': slam_cfg,
                'slam_params': str(slam_path),
                'pid': mp.popen.pid,
                'log': mp.log_path,
            }

        if path == '/api/mapping/stop':
            self.pm.stop_slam()
            self.session['phase'] = 'idle'
            return {'ok': True}

        if path == '/api/mapping/remap':
            robot = self.session.get('robot') or 'grip'
            name = self.session.get('map_name') or ''
            slam_path = maps.slam_params_path(name) if name else None
            if name and (not slam_path or not slam_path.is_file()):
                slam_cfg = self.session.get('slam') or {
                    'resolution': 0.03,
                    'max_laser_range': 10.0,
                    'min_laser_range': 0.05,
                }
                slam_path = maps.write_slam_params(name, **slam_cfg)
            self.pm.stop_slam()
            time.sleep(1.0)
            self._trail = []
            self._map = None
            mp = self.pm.start_slam(robot, str(slam_path) if slam_path else None)
            self.session['phase'] = 'mapping'
            self.get_logger().info(
                f'Mapping remap robot={robot} pid={mp.popen.pid} log={mp.log_path}')
            return {'ok': True, 'remapped': True, 'log': mp.log_path}

        if path == '/api/map/save':
            name = str(body.get('map_name') or self.session.get('map_name') or '').strip()
            if not name:
                raise ValueError('map_name required')
            prefix = maps.map_prefix(name)
            # Ensure project dir is writable by bridge user before map_saver writes
            try:
                os.makedirs(os.path.dirname(prefix), exist_ok=True)
                probe = prefix + '._wprobe'
                with open(probe, 'w') as f:
                    f.write('ok')
                os.unlink(probe)
            except PermissionError as exc:
                d = maps.project_dir(name)
                raise PermissionError(
                    f'Cannot write in {d} (likely root-owned). Run:\n'
                    f'  sudo chown -R $USER:$USER ~/skill23_ws/studio_maps'
                ) from exc
            proc = self.pm.save_map(prefix)
            if proc.returncode != 0:
                raise RuntimeError(proc.stderr or proc.stdout or 'map_saver failed')
            # Fresh pixels — drop the pre-burn snapshot of the previous map.pgm
            maps.invalidate_base_snapshot(name)
            # Mapping session ends on save — free HW for Arena localize / Deploy
            self.pm.stop_slam()
            self.session['phase'] = 'mode'
            maps.write_meta(name, robot=self.session.get('robot'), saved_at=time.time(),
                            phase='mode')
            self._trail = []
            self._trail_last = (None, None)
            self.get_logger().info(
                f'Map saved → {maps.map_yaml_path(name)} (SLAM stopped, trail cleared)')
            return {
                'ok': True,
                'path': str(maps.map_yaml_path(name)),
                'trail_cleared': True,
                'slam_stopped': True,
            }

        if path == '/api/map/delete':
            name = str(body.get('map_name') or body.get('name') or '').strip()
            if not name:
                raise ValueError('map_name required')
            # Stop stacks that might be using this map
            if self.session.get('map_name') == name:
                self.pm.stop_slam()
                self.pm.stop_localize()
                self.pm.stop_nav()
                self.session['map_name'] = ''
                self.session['phase'] = 'idle'
            maps.delete_project(name)
            self.get_logger().info(f'Deleted project {name}')
            return {'ok': True, 'deleted': name}

        if path == '/api/map/import':
            name = str(body.get('map_name', '')).strip()
            yaml_src = str(body.get('yaml_path', '')).strip()
            pgm_src = body.get('pgm_path')
            robot = str(body.get('robot') or 'grip')
            path_out = maps.import_map_files(name, yaml_src, pgm_src, robot=robot)
            self.session.update({'map_name': name, 'robot': robot})
            return {'ok': True, 'path': str(path_out)}

        if path == '/api/teleop':
            vx = float(body.get('vx', 0))
            vy = float(body.get('vy', 0))
            wz = float(body.get('wz', 0))
            # Clamp for safety
            vx = max(-0.35, min(0.35, vx))
            vy = max(-0.35, min(0.35, vy))
            wz = max(-0.6, min(0.6, wz))
            self._publish_cmd(vx, vy, wz)
            return {'ok': True}

        if path == '/api/map/edit':
            name = self.session.get('map_name') or ''
            maps.apply_occupancy_edits(name, list(body.get('ops') or []))
            return {'ok': True}

        if path == '/api/map/restrictions':
            name = self.session.get('map_name') or ''
            rects = list(body.get('rectangles') or [])
            maps.save_restrictions(name, rects)
            if body.get('burn'):
                maps.burn_restrictions_into_map(name)
            return {'ok': True}

        if path == '/api/waypoints':
            name = self.session.get('map_name') or ''
            wps = list(body.get('waypoints') or [])
            maps.save_waypoints(name, wps)
            return {'ok': True, 'count': len(wps)}

        if path == '/api/deploy/start':
            name = self.session.get('map_name') or ''
            robot = self.session.get('robot') or 'grip'
            start_camera = bool(body.get('start_camera', False))
            stack = 'nav2'
            yaml_path = maps.map_yaml_path(name)
            if not yaml_path.is_file():
                raise FileNotFoundError(str(yaml_path))
            # Normal deploy explicitly frees camera USB/CPU. Camera deploy starts
            # Orbbec after nav/AMCL have had time to stabilize.
            self._cam_deploy_requested = False
            if not start_camera:
                self._cam_ui = False
                self._stop_orbbec()
            # Optionally burn restrictions before nav
            if body.get('burn_restrictions', True):
                maps.burn_restrictions_into_map(name)
            mp = self.pm.start_nav(
                robot, str(yaml_path), with_camera=start_camera, stack=stack)
            self.session['phase'] = 'deploy'
            self.session['mode'] = 'arena'
            self.session['nav_stack'] = stack
            self._clear_nav_paths()
            self._sim_path = None
            self._sim_path_smooth = None
            self._sim_result = {}
            self._cam_forbid_autostart = False
            if start_camera:
                self._cam_deploy_requested = True
                # Keep camera startup away from the navigation process burst.
                self._cam_deploy_launch_after = time.time() + 8.0
                self._cam_log(
                    'Deploy with camera selected — starting Orbbec after '
                    'the basic nav startup delay')
            # Orbbec launches only when camera UI or vision action needs it.
            self.get_logger().info(
                f'Deploy START stack={stack} robot={robot} camera={start_camera} '
                f'map={yaml_path} pid={mp.popen.pid} log={mp.log_path} '
                f'cmd={" ".join(mp.cmd)}')
            return {
                'ok': True,
                'stack': stack,
                'map': str(yaml_path),
                'camera': start_camera,
                'camera_after_nav_startup': start_camera,
                'pid': mp.popen.pid,
                'log': mp.log_path,
                'cmd': ' '.join(mp.cmd),
            }

        if path == '/api/deploy/stop':
            self.get_logger().info('Deploy STOP — shutting down nav stack')
            self._cam_deploy_requested = False
            self._cancel_nav2()
            self.pm.stop_nav()
            self._stop_orbbec()
            self._enable_ctrl(False)
            self.session['phase'] = 'idle'
            self.session['nav_stack'] = 'nav2'
            return {'ok': True}

        if path == '/api/nav2/clear_costmaps':
            return self.clear_nav2_costmaps()

        if path == '/api/estop':
            self._estop = bool(body.get('active', True))
            self._publish_cmd(0, 0, 0)
            self._enable_ctrl(False)
            if self._estop:
                self._stop_mission('dashboard')
                self._reloc_active = False
            return {'ok': True, 'estop': self._estop}

        if path == '/api/set_pose':
            x = float(body['x'])
            y = float(body['y'])
            yaw = float(body.get('yaw', 0))
            msg = PoseWithCovarianceStamped()
            # Stamp 0 → AMCL skips odom time-correction (a "now" stamp shifts the
            # click by intervening odom). Tight cov keeps the click from snapping
            # to a nearby lidar match on the next scan update.
            msg.header.stamp.sec = 0
            msg.header.stamp.nanosec = 0
            msg.header.frame_id = 'map'
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            qx, qy, qz, qw = quat_from_yaw(yaw)
            msg.pose.pose.orientation.x = qx
            msg.pose.pose.orientation.y = qy
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw
            # σ≈5 cm / ≈6° — zeros make the filter diverge
            msg.pose.covariance[0] = 0.0025
            msg.pose.covariance[7] = 0.0025
            msg.pose.covariance[35] = 0.01
            self._initialpose.publish(msg)
            # Optimistic pose until AMCL / TF catch up (UI was preferring stale TF)
            self._pose = self._pose_stamped(x, y, yaw)
            self._pose_override = {
                'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3),
            }
            self._pose_override_until = time.time() + 2.5
            self._amcl_ok = True
            self._amcl_cov = 0.005
            self._amcl_stamp = time.time()
            self.get_logger().info(
                f'Set initial pose x={x:.3f} y={y:.3f} yaw={yaw:.3f}')
            return {'ok': True}

        if path == '/api/goal':
            if 'waypoints' in body:
                wps = body['waypoints']
                if not isinstance(wps, list):
                    raise ValueError('waypoints must be a list')
                seq = str(body.get('sequence') or '')
                self._arm_mission(wps, seq)
                if self._using_nav2():
                    self.get_logger().info(
                        f'Nav2 armed {len(self._armed_wps)} waypoint(s) '
                        f'seq={self._armed_seq_name or "(unnamed)"} — '
                        'Start mission / physical START to run')
                else:
                    msg = String()
                    # Mission manager expects {"waypoints": [...]} (not a bare list)
                    msg.data = json.dumps({'waypoints': wps})
                    self._goals_pub.publish(msg)
                    self.get_logger().info(f'Published {len(wps)} mission waypoint(s)')
            else:
                ps = self._pose_stamped(
                    float(body['x']), float(body['y']), float(body.get('yaw', 0)))
                if self._using_nav2():
                    self._send_nav2_pose(ps)
                else:
                    self._goal_pub.publish(ps)
            return {'ok': True}

        if path == '/api/mission/arm':
            wps = body.get('waypoints')
            if not isinstance(wps, list):
                raise ValueError('waypoints must be a list')
            n = self._arm_mission(wps, str(body.get('sequence') or ''))
            return {
                'ok': True,
                'count': n,
                'sequence': self._armed_seq_name,
            }

        if path == '/api/start':
            source = str(body.get('source') or 'dashboard')
            if isinstance(body.get('waypoints'), list):
                self._arm_mission(
                    body['waypoints'], str(body.get('sequence') or ''))
            return self._start_mission(source)

        if path == '/api/stop':
            return self._stop_mission(str(body.get('source') or 'dashboard'))

        if path == '/api/path_style':
            style = str(body.get('style', 'smoothed')).lower()
            self._path_style = 'normal' if style in ('normal', 'raw', 'planner') else 'smoothed'
            msg = String()
            msg.data = self._path_style
            self._path_style_pub.publish(msg)
            return {'ok': True, 'style': self._path_style}

        if path == '/api/reloc/start':
            motion = str(body.get('motion', 'spin'))
            return self._begin_reloc(motion, then_task=False)

        if path == '/api/reloc/stop':
            self._reloc_active = False
            self._reloc_then_task = False
            self._reloc_pending_wps = None
            self._reloc_start_after = None
            self._reloc_result = 'stopped'
            self._publish_cmd(0, 0, 0)
            return {'ok': True, 'result': 'stopped'}

        if path == '/api/reloc/then_task':
            motion = str(body.get('motion', 'spin'))
            wps = body.get('waypoints')
            if not isinstance(wps, list):
                wps = None
            return self._begin_reloc(motion, then_task=True, waypoints=wps)

        if path == '/api/sim/plan':
            sx = float(body['sx'])
            sy = float(body['sy'])
            gx = float(body['gx'])
            gy = float(body['gy'])
            gyaw = float(body.get('gyaw', 0.0))
            self._sim_req_id += 1
            req_id = self._sim_req_id
            self._sim_path = None
            self._sim_path_smooth = None
            self._sim_result = {
                'id': req_id, 'ok': None, 'detail': 'planning…',
                'stack': self.session.get('nav_stack', 'custom'),
            }
            if self._using_nav2():
                out = self._nav2_compute_path_sync(
                    sx, sy, float(body.get('syaw', 0.0)), gx, gy, gyaw)
                if out.get('ok'):
                    self._sim_path = out['path']
                    self._sim_result = {
                        'id': req_id,
                        'ok': True,
                        'detail': out['detail'],
                        'points': out['points'],
                        'points_smooth': 0,
                        'stack': 'nav2',
                    }
                else:
                    self._sim_result = {
                        'id': req_id,
                        'ok': False,
                        'detail': out.get('detail', 'plan failed'),
                        'points': 0,
                        'stack': 'nav2',
                    }
                return {'ok': True, 'id': req_id}
            msg = String()
            msg.data = json.dumps({
                'id': req_id,
                'sx': sx, 'sy': sy, 'gx': gx, 'gy': gy, 'gyaw': gyaw,
            })
            self._sim_req_pub.publish(msg)
            return {'ok': True, 'id': req_id}

        if path == '/api/sim/clear':
            self._sim_path = None
            self._sim_path_smooth = None
            self._sim_result = {}
            return {'ok': True}

        if path == '/api/trail/clear':
            self._trail = []
            self._trail_last = (None, None)
            self._path = None
            self._planner_path = None
            self._smoothed_path = None
            return {'ok': True}

        if path == '/api/phase':
            phase = str(body.get('phase', 'idle'))
            self.session['phase'] = phase
            if phase == 'deploy':
                self._cam_forbid_autostart = False
            elif phase != 'deploy' and not self._cam_ui and not self._cam_action_need:
                self._stop_orbbec()
            return {'ok': True, 'session': self.session}

        if path == '/api/localize/start':
            name = str(body.get('map_name') or self.session.get('map_name') or '').strip()
            robot = str(body.get('robot') or self.session.get('robot') or 'grip')
            if not name:
                raise ValueError('map_name required')
            yaml_path = maps.map_yaml_path(name)
            if not yaml_path.is_file():
                raise FileNotFoundError(
                    f'No saved map for "{name}". Create/save a map first.')
            self.session.update({
                'map_name': name,
                'robot': robot,
                'mode': 'arena',
                'phase': 'improve',
            })
            self._trail = []
            self._trail_last = (None, None)
            self._map = None
            self._scan = []
            self._pose = None
            self._amcl_ok = False
            self._amcl_cov = 99.0
            mp = self.pm.start_localize(robot, str(yaml_path))
            self.get_logger().info(
                f'Localize START robot={robot} map={yaml_path} '
                f'pid={mp.popen.pid} log={mp.log_path} cmd={" ".join(mp.cmd)}')
            return {
                'ok': True,
                'map': str(yaml_path),
                'pid': mp.popen.pid,
                'log': mp.log_path,
                'cmd': ' '.join(mp.cmd),
            }

        if path == '/api/localize/stop':
            self.get_logger().info('Localize STOP')
            self.pm.stop_localize()
            self._pose = None
            self._amcl_ok = False
            self._scan = []
            return {'ok': True}

        if path == '/api/camera/on':
            self._cam_ui = True
            self._cam_forbid_autostart = False
            self._cam_need_last = None
            self._cam_log('UI camera ON → /camera/toggle_color true')
            self._ensure_orbbec()
            if self._cam_stream_allowed():
                self._toggle_orbbec(color=True)
            else:
                wait = max(0.0, self._cam_grace_until - time.time())
                self._cam_log(
                    f'stream will enable in ~{wait:.0f}s (mission active — AMCL guard)')
            return {'ok': True, 'on': True, 'stream': self._cam_stream}

        if path == '/api/camera/off':
            self._cam_ui = False
            self._cam_need_last = None
            self._cam_log('UI camera OFF → /camera/toggle_color false')
            self._toggle_orbbec(color=False)
            return {'ok': True, 'on': False, 'stream': self._cam_stream}

        if path == '/api/camera/kill':
            # Fully stop Orbbec process (not just toggle stream off).
            self._cam_ui = False
            self._cam_action_need = False
            self._cam_need_depth = False
            self._cam_need_last = None
            self._cam_deploy_requested = False
            self._cam_forbid_autostart = True
            self._cam_log('UI Kill camera node — stopping Orbbec process')
            self._stop_orbbec()
            return {
                'ok': True,
                'on': False,
                'process': self.pm.running('camera'),
                'stacks': self.pm.stacks_alive(),
            }

        raise ValueError(f'unknown endpoint {path}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StudioBridge()
    cleaned = {'done': False}

    def cleanup() -> None:
        if cleaned['done']:
            return
        cleaned['done'] = True
        try:
            node.get_logger().info('Studio shutdown — stopping nav/camera children')
        except Exception:
            pass
        try:
            node.pm.stop_all(timeout=0.4)
        except Exception:
            pass
        # Bash pkill backup (handles root-owned sudo children).
        try:
            stop_sh = os.path.expanduser(
                '~/skill23_ws/src/skill23_studio/scripts/stop_studio_stack.sh')
            if os.path.isfile(stop_sh):
                subprocess.call(
                    ['bash', stop_sh],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    timeout=8)
        except Exception:
            pass

    atexit.register(cleanup)

    def on_signal(signum, _frame):
        cleanup()
        raise SystemExit(0)

    for sig in (signal.SIGTERM, signal.SIGINT):
        try:
            signal.signal(sig, on_signal)
        except Exception:
            pass

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        cleanup()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
