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
from sensor_msgs.msg import BatteryState, Imu, LaserScan, Range
from std_msgs.msg import Bool, Empty, Float32, Float64, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from skill23_studio import map_pipeline as maps
from skill23_studio.hw_test_monitor import (
    HwLiveStore,
    build_channel_specs,
    laser_min_range,
    yaw_from_quat,
)
from skill23_studio.nav.light_tower_patterns import (
    get_pattern,
    list_patterns,
    pattern_duration,
)
from skill23_studio.commission_mission import (
    COMMISSION_OBJECTS,
    default_config,
    drop_sticker_for_object,
    index_waypoints,
    next_pick_index,
    normalize_config,
    normalize_object_name,
    remaining_after_visit,
    resolve_drop_wp,
)
from skill23_studio.process_manager import ProcessManager
from skill23_studio.nav.utils import pose_cov_to_stamped, quat_from_yaw, yaw_from_quat


OMS_HARDWARE = [
    {
        'name': 'Rotation',
        'axis': 'rotation',
        'hw': 'Titan1 M2 Maverick 71.2:1',
        'topics': '/titan1/m_2/cmd  /titan1/m_2/encoder',
        'home': 'Titan1 M0 limit_rev = 0° (front / +X)',
        'range': '−90° CW … +90° ACW',
    },
    {
        'name': 'Lift',
        'axis': 'lift',
        'hw': 'Titan1 M1 Maverick 99.5:1 + GT2 belt',
        'topics': '/titan1/m_1/cmd  /titan1/m_1/encoder',
        'home': 'M1 limit_fwd = up (0 mm). limit_rev = floor',
        'range': '0 mm (top) … 250 mm down, X=0 Y=0, Z only',
    },
    {
        'name': 'Linear',
        'axis': 'linear',
        'hw': 'Cypher Max on Titan1 M3 + Smart Servo 200',
        'topics': '/linear_servo/cmd  /titan1/m_3/encoder',
        'home': 'Titan1 M0 limit_fwd (rear). Encoder reset on home',
        'range': '0 mm retracted … +X forward. Y fixed, Z follows lift',
    },
    {
        'name': 'Vertical servo',
        'axis': 'vertical',
        'hw': 'VMX pin 13 /vertical_servo/set_servo',
        'topics': '/oms/vertical/cmd',
        'home': 'down −40°',
        'range': 'down −40° / up 60°',
    },
    {
        'name': 'Rotation servo',
        'axis': 'rotation_servo',
        'hw': 'VMX pin 14 /rotation_servo/set_servo',
        'topics': '/oms/rotation_servo/cmd',
        'home': 'out 0°',
        'range': 'out 0° (home) / in 110°',
    },
    {
        'name': 'Gripper',
        'axis': 'gripper',
        'hw': 'VMX pin 15 /gripper_servo/set_servo',
        'topics': '/oms/gripper/cmd',
        'home': 'closed 150°',
        'range': 'closed 150° / open −150°',
    },
]


# Plenty for the ~40 lines the dashboard shows, without reading whole logs.
_LOG_TAIL_BYTES = 16_384
# Telemetry SSE: longer TTL = less JSON/ps work on the Pi while dashboards idle.
_SNAPSHOT_CACHE_TTL = 1.5
_LOG_CACHE_TTL = 3.0
_SSE_SLEEP_S = 1.5
_ODOM_MIN_DT = 0.10   # ≤10 Hz odom bookkeeping
_AMCL_MIN_DT = 0.10
_SCAN_IDLE_EVERY = 5  # process 1 of N scans when not mapping/hw_test


def _ros_string(data: Any) -> str:
    """Strip embedded nulls from std_msgs/String (avoids DDS deserialize warnings)."""
    if data is None:
        return ''
    text = data if isinstance(data, str) else str(data)
    return text.split('\0', 1)[0]


def _json_from_ros_string(data: Any) -> Any:
    text = _ros_string(data).strip()
    if not text:
        return None
    return json.loads(text)


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
            'nav_stack': 'nav2',  # nav2 | gc_planner
            'deploy_module': 'nav',  # nav | commission
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
        self._odom_stamp = 0.0
        self._odom_speed = 0.0
        self._odom_frame = ''
        self._scan_raw_count = 0
        self._scan_valid_count = 0
        self._map: Optional[OccupancyGrid] = None
        self._map_seq = 0
        self._map_built: Optional[Dict[str, Any]] = None
        self._map_built_seq = -1
        self._map_payload_last: Optional[Dict[str, Any]] = None
        self._map_rev = 0
        self._path: Optional[Path] = None
        self._mission: Dict[str, Any] = {}
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
        self._cam_in_nav_launch = False
        self._sim_path: Optional[Path] = None
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
        self._NAV2_WP_TRIES = 4
        self._nav2_seq_gen = 0
        self._nav2_seq_active = False
        self._nav2_seq_wps: List[dict] = []
        self._nav2_seq_index = 0
        self._nav2_seq_attempt = 1
        self._nav2_seq_ok = 0
        self._nav2_seq_skipped = 0
        self._nav2_wait_until = 0.0
        self._nav2_action_cmd_id = ''
        self._nav2_action_deadline = 0.0
        self._nav2_action_kind = ''
        self._nav2_oms_wp: Optional[dict] = None
        self._nav2_oms_cancel = threading.Event()
        self._nav2_oms_thread: Optional[threading.Thread] = None
        self._nav2_cpu_paused = False
        self._detect_align: Dict[str, Any] = {
            'state': 'idle',
            'message': '',
            'aligned': False,
            'stopped': True,
            'depth_m': None,
            'depth_horizontal_m': None,
            'u_norm': None,
            'bbox': None,
            'conf': None,
            'class_name': '',
            'confirm': 0,
            'confirm_need': 0,
            'obj_cx': None,
            'frame_cx': None,
            'last_ok': None,
            'last_result': '',
        }
        self._commission: Dict[str, Any] = self._commission_idle_state()
        self._oms: Dict[str, Any] = {
            'ready': False,
            'source': 'none',
            'standalone': False,
            'rotation': {
                'angle_deg': None, 'homed': False, 'state': 'offline',
                'home_switch': False,
            },
            'lift': {
                'pos_mm': None, 'homed': False, 'state': 'offline',
                'up_sw': False, 'down_sw': False,
            },
            'linear': {
                'pos_mm': None, 'homed': False, 'state': 'offline',
                'home_switch': False,
            },
            'vertical': {'pose': '', 'angle_deg': None, 'homed': False, 'state': 'offline'},
            'rotation_servo': {
                'pose': '', 'angle_deg': None, 'homed': False, 'state': 'offline'},
            'gripper': {
                'pose': '', 'angle_deg': None, 'homed': False, 'state': 'offline'},
            'home': {'state': 'idle'},
            'last_result': {},
            'pose': {'x_m': 0.0, 'y_m': 0.0, 'z_m': 0.0, 'yaw_deg': 0.0},
            'stamp': 0.0,
        }

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10)
        self._odom_prefer_filtered = False
        self.create_subscription(
            Odometry, '/odometry/filtered', self._on_odom_filtered, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom_raw, 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, qos)
        self.create_subscription(Path, '/plan', self._on_path, 10)
        self.create_subscription(BatteryState, '/battery_state', self._on_battery, 10)
        # /scan is subscribed lazily — constantly decoding LaserScan when Nav2
        # lidars are up was a major studio_bridge CPU cost.
        self._scan_sub = None
        self._scan_sub_wanted = False

        self._initialpose = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self._goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self._nav_to_pose = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._nav_through = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses')
        self._compute_path = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self._clear_local_cli = self.create_client(
            ClearEntireCostmap, 'local_costmap/clear_entirely_local_costmap')
        self._clear_global_cli = self.create_client(
            ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap')
        # GC walls publisher is created only while GC+Planner runs (otherwise
        # /gc_planner/walls stays advertised forever and confuses topic lists).
        self._gc_walls_pub = None
        self._gc_walls_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._gc_walls_cells: set = set()  # {(gx, gy), ...} occupied overlay
        self._gc_walls_meta: Optional[Dict[str, Any]] = None
        self._nav2_goal_handle = None
        self._cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        # Public intent signal: subscribers (laptop vision) can use it to know
        # when Studio wants frames. Studio itself never subscribes to images.
        self._cam_need_pub = self.create_publisher(Bool, '/studio/camera/need', 10)
        self._toggle_color_cli = self.create_client(SetBool, '/camera/toggle_color')
        self._toggle_color_cli_ns = self.create_client(
            SetBool, '/camera/camera/toggle_color')
        self._toggle_color_cli_root = self.create_client(SetBool, '/toggle_color')
        self._toggle_depth_cli = self.create_client(SetBool, '/camera/toggle_depth')
        self._toggle_depth_cli_ns = self.create_client(
            SetBool, '/camera/camera/toggle_depth')
        self._toggle_depth_cli_root = self.create_client(SetBool, '/toggle_depth')
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
        self._align_cmd_pub = self.create_publisher(String, '/studio/align/cmd', 10)
        self._align_cancel_pub = self.create_publisher(Empty, '/studio/align/cancel', 10)
        self.create_subscription(
            String, '/studio/align/result', self._on_studio_align_result, 10)
        self._lookup_cmd_pub = self.create_publisher(String, '/studio/lookup/cmd', 10)
        self._lookup_cancel_pub = self.create_publisher(Empty, '/studio/lookup/cancel', 10)
        self.create_subscription(
            String, '/studio/lookup/result', self._on_studio_lookup_result, 10)
        self._lookup: Dict[str, Any] = {
            'state': 'idle', 'message': '', 'last_ok': None, 'last_next_wp': None,
        }
        self._cobra_cmd_pub = self.create_publisher(String, '/studio/cobra/cmd', 10)
        self._cobra_cancel_pub = self.create_publisher(Empty, '/studio/cobra/cancel', 10)
        self.create_subscription(
            String, '/studio/cobra/result', self._on_studio_align_result, 10)
        self._light_gen = 0
        self._light_cli = None
        try:
            from studica_control.srv import SetData  # type: ignore
            self._light_cli = self.create_client(SetData, '/light_tower/set')
        except Exception as exc:
            self.get_logger().warn(f'light_tower SetData unavailable: {exc}')
        self._detect_cmd_pub = self.create_publisher(
            String, '/studio/detect_align/cmd', 10)
        self._detect_cancel_pub = self.create_publisher(
            Empty, '/studio/detect_align/cancel', 10)
        self.create_subscription(
            String, '/studio/detect_align/result',
            self._on_studio_detect_align_result, 10)
        self.create_subscription(
            String, '/studio/detect_align/status',
            self._on_studio_detect_align_status, 10)
        self.create_subscription(
            String, '/oms/rotation/status', self._on_oms_status, 10)
        self.create_subscription(
            Float64, '/oms/rotation/angle_deg', self._on_oms_angle, 10)
        self.create_subscription(
            String, '/oms/lift/status',
            lambda m: self._on_oms_axis_status('lift', m), 10)
        self.create_subscription(
            Float64, '/oms/lift/pos_mm',
            lambda m: self._on_oms_mm('lift', m), 10)
        self.create_subscription(
            String, '/oms/linear/status',
            lambda m: self._on_oms_axis_status('linear', m), 10)
        self.create_subscription(
            Float64, '/oms/linear/pos_mm',
            lambda m: self._on_oms_mm('linear', m), 10)
        self.create_subscription(
            String, '/oms/vertical/status',
            lambda m: self._on_oms_axis_status('vertical', m), 10)
        self.create_subscription(
            String, '/oms/rotation_servo/status',
            lambda m: self._on_oms_axis_status('rotation_servo', m), 10)
        self.create_subscription(
            String, '/oms/gripper/status',
            lambda m: self._on_oms_axis_status('gripper', m), 10)
        self.create_subscription(
            String, '/oms/home/status',
            lambda m: self._on_oms_axis_status('home', m), 10)
        self._oms_cmd_pubs = {
            'rotation': self.create_publisher(String, '/oms/rotation/cmd', 10),
            'lift': self.create_publisher(String, '/oms/lift/cmd', 10),
            'linear': self.create_publisher(String, '/oms/linear/cmd', 10),
            'vertical': self.create_publisher(String, '/oms/vertical/cmd', 10),
            'rotation_servo': self.create_publisher(
                String, '/oms/rotation_servo/cmd', 10),
            'gripper': self.create_publisher(String, '/oms/gripper/cmd', 10),
            'home': self.create_publisher(String, '/oms/home/cmd', 10),
        }
        self._oms_power_pub = self.create_publisher(String, '/oms/power', 10)
        for axis in (
                'rotation', 'lift', 'linear', 'vertical',
                'rotation_servo', 'gripper', 'home'):
            self.create_subscription(
                String, f'/oms/{axis}/result',
                lambda m, a=axis: self._on_oms_result(a, m), 10)

        self.create_timer(0.25, self._reloc_tick)
        self.create_timer(0.25, self._nav2_seq_tick)
        # Trail is only shipped during mapping — no TF spam otherwise.
        self.create_timer(1.0, self._trail_tick)
        self.create_timer(1.0, self._cam_tick)
        self.create_timer(2.0, self._scan_sub_tick)
        self._snapshot_cache: Optional[Dict[str, Any]] = None
        self._snapshot_cache_t = 0.0
        self._logs_cache: Optional[Dict[str, str]] = None
        self._logs_cache_t = 0.0
        self._sse_clients = 0
        self._toggle_ready_cache = False
        self._toggle_ready_cache_t = 0.0
        self._scan_cb_n = 0
        self._path_pts_cached = None
        # Hardware Testing Software (lazy subscriptions while stack is up).
        self._hw_store = HwLiveStore()
        self._hw_subs: List[Any] = []
        self._hw_topic_ids: Dict[str, List[str]] = {}
        maps.ensure_root()
        threading.Thread(target=self._run_http, daemon=True).start()
        self.get_logger().info(f'Studio bridge HTTP :{self.http_port}')

    # ── ROS callbacks ───────────────────────────────────────────────────
    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        now = time.time()
        # Do not bump stamp on skip — otherwise continuous AMCL never fully updates.
        if self._amcl_stamp > 0.0 and (now - self._amcl_stamp) < _AMCL_MIN_DT:
            return
        ps = pose_cov_to_stamped(msg)
        x = float(ps.pose.position.x)
        y = float(ps.pose.position.y)
        if abs(x) > 50.0 or abs(y) > 50.0:
            return
        self._pose = ps
        self._amcl_ok = True
        cov = msg.pose.covariance
        self._amcl_cov = float(cov[0] + cov[7]) if len(cov) > 7 else 99.0
        self._amcl_stamp = now

    def _on_odom_filtered(self, msg: Odometry) -> None:
        self._odom_prefer_filtered = True
        self._on_odom(msg)

    def _on_odom_raw(self, msg: Odometry) -> None:
        # Ignore wheel /odom while EKF filtered is fresh (halves callback load).
        if (
            self._odom_prefer_filtered
            and self._odom_stamp > 0.0
            and (time.time() - self._odom_stamp) < 1.0
        ):
            return
        self._on_odom(msg)

    def _on_odom(self, msg: Odometry) -> None:
        now = time.time()
        if self._odom_stamp > 0.0 and (now - self._odom_stamp) < _ODOM_MIN_DT:
            tw = msg.twist.twist
            self._odom_speed = math.hypot(float(tw.linear.x), float(tw.linear.y))
            return
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self._odom_pose = ps
        self._odom_stamp = now
        tw = msg.twist.twist
        self._odom_speed = math.hypot(float(tw.linear.x), float(tw.linear.y))
        self._odom_frame = str(msg.header.frame_id or '')

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
        self._path_pts_cached = None

    def _on_path(self, msg: Path) -> None:
        self._path = msg
        self._path_pts_cached = None

    def _using_nav2(self) -> bool:
        stack = str(self.session.get('nav_stack') or '')
        return stack in ('nav2', 'gc_planner')

    def _nav2_compute_path_sync(
        self,
        sx: float,
        sy: float,
        syaw: float,
        gx: float,
        gy: float,
        gyaw: float,
        *,
        planner_id: str = 'GridBased',
        timeout_sec: float = 12.0,
    ) -> Dict[str, Any]:
        """One-shot ClearancePlanner preview via Nav2 compute_path_to_pose."""
        if not self._compute_path.wait_for_server(timeout_sec=2.0):
            return {'ok': False, 'detail': 'compute_path_to_pose not ready', 'points': 0}

        pid = str(planner_id or 'GridBased').strip()
        if pid not in ('GridBased',):
            pid = 'GridBased'

        goal = ComputePathToPose.Goal()
        goal.planner_id = pid
        goal.use_start = True
        stamp = Time().to_msg()

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
            return {'ok': False, 'detail': 'no path', 'points': 0, 'planner_id': pid}
        return {
            'ok': True,
            'detail': f'{pid} ({len(path.poses)} pts)',
            'points': len(path.poses),
            'planner_id': pid,
            'path': path,
        }

    def _pose_stamped(self, x: float, y: float, yaw: float = 0.0) -> PoseStamped:
        ps = PoseStamped()
        # Stamp 0 = latest TF. A wall-clock stamp goes stale under Pi load and
        # Nav2 then fails every tick with "extrapolation into the past".
        ps.header.stamp.sec = 0
        ps.header.stamp.nanosec = 0
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
        self._commission_reset()
        self._nav2_seq_active = False
        self._nav2_seq_gen += 1
        self._nav2_wait_until = 0.0
        self._nav2_action_cmd_id = ''
        self._nav2_action_deadline = 0.0
        self._nav2_action_kind = ''
        try:
            self._align_cancel_pub.publish(Empty())
        except Exception:
            pass
        try:
            self._cobra_cancel_pub.publish(Empty())
        except Exception:
            pass
        try:
            self._detect_cancel_pub.publish(Empty())
        except Exception:
            pass
        self._light_gen += 1
        self._light_tower_set('off')
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
        if not isinstance(action, dict):
            return 0.0
        if action.get('type') not in ('wait', 'wait_light'):
            return 0.0
        try:
            seconds = float(action.get('seconds', 0.0))
        except (TypeError, ValueError):
            return 0.0
        if not math.isfinite(seconds):
            return 0.0
        return min(3600.0, max(0.0, seconds))

    def _light_tower_set(self, cmd: str) -> bool:
        cli = getattr(self, '_light_cli', None)
        if cli is None:
            return False
        try:
            if not cli.service_is_ready():
                return False
            from studica_control.srv import SetData  # type: ignore
            req = SetData.Request()
            req.params = str(cmd)
            cli.call_async(req)
            return True
        except Exception as exc:
            self.get_logger().warn(f'light_tower set "{cmd}" failed: {exc}')
            return False

    def _play_light_pattern(self, pattern: str, gen: int) -> None:
        for cmd, dwell in get_pattern(pattern):
            if gen != self._light_gen:
                break
            self._light_tower_set(cmd)
            end = time.time() + max(0.0, float(dwell))
            while time.time() < end and gen == self._light_gen:
                time.sleep(0.05)
        if gen == self._light_gen:
            self._light_tower_set('off')

    def _nav2_start_wait_light(self, wp: dict, reached_idx: int) -> None:
        n = len(self._nav2_seq_wps)
        action = self._wp_action(wp)
        pattern = str(action.get('pattern') or 'red_blink_3x')
        seconds = self._nav2_wait_seconds(wp)
        wait_s = max(seconds, pattern_duration(pattern), 0.5)
        self._light_gen += 1
        gen = self._light_gen
        threading.Thread(
            target=self._play_light_pattern, args=(pattern, gen), daemon=True).start()
        self._nav2_wait_until = time.time() + wait_s
        self._nav2_action_cmd_id = ''
        self._nav2_action_kind = 'wait_light'
        self._nav2_action_deadline = 0.0
        self._nav2_set_mission(
            'wait_action',
            f'wp {reached_idx + 1}/{n} wait_light {pattern} {wait_s:g}s')
        self.get_logger().info(
            f'Waypoint {reached_idx + 1}/{n} reached — wait_light '
            f'pattern={pattern} for {wait_s:g}s '
            '(enable light_tower in hw_params if no flash)')

    def _nav2_start_cobra_action(self, wp: dict, reached_idx: int) -> None:
        n = len(self._nav2_seq_wps)
        action = self._wp_action(wp)
        timeout = self._nav2_action_timeout('cobra_align', action)
        cmd_id = f'nav2_{self._nav2_seq_gen}_{reached_idx}_{int(time.time() * 1000)}'
        payload: Dict[str, Any] = {
            'cmd_id': cmd_id,
            'mode': 'cobra_align',
        }
        try:
            cmd_timeout = float(action.get('timeout_sec', 0.0) or 0.0)
        except (TypeError, ValueError):
            cmd_timeout = 0.0
        if cmd_timeout > 0.0 and math.isfinite(cmd_timeout):
            payload['timeout_sec'] = min(60.0, max(2.0, cmd_timeout))
        self._cobra_cmd_pub.publish(String(data=json.dumps(payload)))
        self._nav2_action_cmd_id = cmd_id
        self._nav2_action_kind = 'cobra_align'
        self._nav2_action_deadline = time.time() + timeout + 15.0
        self._nav2_set_mission(
            'wait_action', f'wp {reached_idx + 1}/{n} cobra_align')
        self.get_logger().info(
            f'Waypoint {reached_idx + 1}/{n} reached — cobra_align '
            '(enable cobra in hw_params)')

    def _nav2_set_mission(self, state: str, detail: str = '') -> None:
        n = len(self._nav2_seq_wps)
        idx = self._nav2_seq_index
        remaining = 0.0
        if self._nav2_wait_until > 0.0:
            remaining = max(0.0, self._nav2_wait_until - time.time())
        elif self._nav2_action_deadline > 0.0:
            remaining = max(0.0, self._nav2_action_deadline - time.time())
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
        self._nav2_action_cmd_id = ''
        self._nav2_action_deadline = 0.0
        self._nav2_action_kind = ''
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
            if self._commission.get('active'):
                return
            self._nav2_seq_active = False
            self._nav2_wait_until = 0.0
            self._nav2_action_cmd_id = ''
            self._nav2_action_deadline = 0.0
            self._nav2_action_kind = ''
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

    @staticmethod
    def _wp_action(wp: dict) -> dict:
        action = (wp or {}).get('action')
        return action if isinstance(action, dict) else {}

    def _nav2_action_timeout(self, atype: str, action: dict) -> float:
        """Watchdog if the align node never replies (covers YAML timeout + hold)."""
        try:
            raw = float(action.get('timeout_sec', 0.0) or 0.0)
        except (TypeError, ValueError):
            raw = 0.0
        if raw > 0.0 and math.isfinite(raw):
            return min(120.0, max(2.0, raw))
        return 180.0 if atype == 'detect_align' else 150.0

    def _nav2_start_detect_align(self, wp: dict, reached_idx: int) -> None:
        n = len(self._nav2_seq_wps)
        action = self._wp_action(wp)
        timeout = self._nav2_action_timeout('detect_align', action)
        cmd_id = f'nav2_{self._nav2_seq_gen}_{reached_idx}_{int(time.time() * 1000)}'
        payload: Dict[str, Any] = {
            'cmd_id': cmd_id,
            'mode': 'detect_align',
        }
        try:
            cmd_timeout = float(action.get('timeout_sec', 0.0) or 0.0)
        except (TypeError, ValueError):
            cmd_timeout = 0.0
        if cmd_timeout > 0.0 and math.isfinite(cmd_timeout):
            payload['timeout_sec'] = min(90.0, max(2.0, cmd_timeout))
        detail = f'wp {reached_idx + 1}/{n} detect+align'
        self._detect_cmd_pub.publish(String(data=json.dumps(payload)))
        self._nav2_action_cmd_id = cmd_id
        self._nav2_action_kind = 'detect_align'
        self._nav2_action_deadline = time.time() + timeout + 15.0
        self._nav2_set_mission('wait_action', detail)
        self.get_logger().info(f'Waypoint {reached_idx + 1}/{n} reached — {detail}')

    def _oms_set_power(self, mode: str) -> None:
        """Wake/sleep OMS process started with Nav2 (start_asleep)."""
        payload = {'mode': 'wake' if mode == 'wake' else 'sleep'}
        try:
            self._oms_power_pub.publish(String(data=json.dumps(payload)))
        except Exception as exc:
            self.get_logger().warn(f'/oms/power {mode} failed: {exc}')

    def _nav2_start_oms_action(self, wp: dict, reached_idx: int) -> None:
        """Run dashboard OMS sequence at this waypoint (wake → work → sleep)."""
        n = len(self._nav2_seq_wps)
        action = self._wp_action(wp)
        seq = action.get('sequence') or []
        try:
            timeout = float(action.get('timeout_sec', 120.0) or 120.0)
        except (TypeError, ValueError):
            timeout = 120.0
        timeout = min(600.0, max(10.0, timeout))
        cmd_id = f'nav2_oms_{self._nav2_seq_gen}_{reached_idx}_{int(time.time() * 1000)}'
        self._nav2_oms_wp = wp
        self._nav2_action_cmd_id = cmd_id
        self._nav2_action_kind = 'oms'
        self._nav2_action_deadline = time.time() + timeout + 15.0
        detail = f'wp {reached_idx + 1}/{n} OMS ({len(seq) if isinstance(seq, list) else "?"} steps)'
        self._nav2_set_mission('wait_action', detail)
        self.get_logger().info(f'Waypoint {reached_idx + 1}/{n} reached — {detail}')
        self._nav2_oms_cancel.clear()
        self._nav2_oms_thread = threading.Thread(
            target=self._nav2_run_oms_sequence_job,
            args=(cmd_id, wp),
            daemon=True,
            name='nav2_oms_seq',
        )
        self._nav2_oms_thread.start()

    def _nav2_run_oms_sequence_job(self, cmd_id: str, wp: dict) -> None:
        from skill23_studio.nav.wp_oms_sequence import run_wp_oms_sequence
        action = self._wp_action(wp)
        result_store = self._oms.setdefault('last_result', {})
        self._oms_set_power('wake')
        time.sleep(0.15)

        def _log(msg: str, level: str = 'info') -> None:
            if level == 'warn':
                self.get_logger().warn(f'[wp_oms] {msg}')
            else:
                self.get_logger().info(f'[wp_oms] {msg}')

        try:
            ok, message = run_wp_oms_sequence(
                pubs=self._oms_cmd_pubs,
                result_store=result_store,
                sequence=action.get('sequence'),
                lift_mm=float(action.get('lift_mm', 0.0) or 0.0),
                linear_mm=float(action.get('linear_mm', 0.0) or 0.0),
                rotation_deg=float(action.get('rotation_deg', 0.0) or 0.0),
                wait_sec=float(action.get('wait_sec', 1.0) or 1.0),
                axis_timeout_sec=float(action.get('axis_timeout_sec', 60.0) or 60.0),
                home_timeout_sec=float(action.get('home_timeout_sec', 180.0) or 180.0),
                log=_log,
                cancel_check=self._nav2_oms_cancel.is_set,
            )
        except Exception as exc:
            ok, message = False, f'OMS sequence error: {exc}'
            self.get_logger().error(f'[wp_oms] {message}')
        finally:
            self._oms_set_power('sleep')

        if self._nav2_action_cmd_id != cmd_id:
            return
        self.get_logger().info(
            f'OMS sequence result ok={ok} {message}')
        self._nav2_finish_action_then_next(
            f'oms {"ok" if ok else "failed"}')

    def _nav2_cancel_oms_job(self) -> None:
        self._nav2_oms_cancel.set()
        try:
            self._oms_set_power('sleep')
        except Exception:
            pass

    def _nav2_start_align_action(
            self, wp: dict, reached_idx: int, atype: str) -> None:
        n = len(self._nav2_seq_wps)
        action = self._wp_action(wp)
        # align_move_oms → run align_move first, then OMS on success.
        run_oms_after = atype == 'align_move_oms'
        align_type = 'align_move' if run_oms_after else atype
        timeout = self._nav2_action_timeout(align_type, action)
        if run_oms_after:
            try:
                oms_t = float(action.get('timeout_sec', 120.0) or 120.0)
            except (TypeError, ValueError):
                oms_t = 120.0
            timeout = timeout + min(600.0, max(10.0, oms_t))
        cmd_id = f'nav2_{self._nav2_seq_gen}_{reached_idx}_{int(time.time() * 1000)}'
        try:
            yaw = float(wp.get('yaw', 0.0) or 0.0)
        except (TypeError, ValueError):
            yaw = 0.0
        payload: Dict[str, Any] = {
            'cmd_id': cmd_id,
            'mode': align_type,
            'yaw': yaw,
        }
        if align_type == 'align_move':
            try:
                dist = float(action.get('distance_m', 0.0) or 0.0)
            except (TypeError, ValueError):
                dist = 0.0
            if math.isfinite(dist) and dist > 0.0:
                payload['distance_m'] = min(5.0, max(0.03, dist))
            else:
                self.get_logger().warn(
                    f'Waypoint {reached_idx + 1}/{n} align+move missing distance — '
                    'running yaw align only')
                align_type = 'align'
                payload['mode'] = 'align'
        detail = (
            f'wp {reached_idx + 1}/{n} '
            f'{"align_move→oms" if run_oms_after else align_type} '
            f'yaw={math.degrees(yaw):.1f}deg')
        if 'distance_m' in payload:
            detail += f' dist={payload["distance_m"]:.3f}m'
        self._align_cmd_pub.publish(String(data=json.dumps(payload)))
        self._nav2_action_cmd_id = cmd_id
        self._nav2_action_kind = (
            'align_move_oms' if run_oms_after else align_type)
        self._nav2_oms_wp = wp if run_oms_after else None
        self._nav2_action_deadline = time.time() + timeout + 15.0
        self._nav2_set_mission('wait_action', detail)
        self.get_logger().info(
            f'Waypoint {reached_idx + 1}/{n} reached — {detail} (YAML timeouts)')

    def _nav2_finish_action_then_next(self, reason: str) -> None:
        if self._commission.get('active'):
            self._commission_on_action_done(reason)
            return
        self._nav2_wait_until = 0.0
        self._nav2_action_cmd_id = ''
        self._nav2_action_deadline = 0.0
        self._nav2_action_kind = ''
        self._nav2_oms_wp = None
        self._lookup_camera_off()
        self.get_logger().info(f'{reason} — generating path to next waypoint')
        self._nav2_resume_heavy()
        self._send_nav2_seq_pose()

    def _lookup_camera_off(self) -> None:
        """Force color stream off after lookup (success, fail, or timeout)."""
        self._cam_action_need = False
        self._cam_need_last = None
        try:
            self._toggle_orbbec(color=False, depth=False)
        except Exception as exc:
            self.get_logger().warn(f'lookup camera off failed: {exc}')

    def _nav2_start_lookup_action(
            self, wp: dict, reached_idx: int, atype: str) -> None:
        """OCR / ArUco lookup → optional jump to 1-based waypoint index."""
        n = len(self._nav2_seq_wps)
        action = self._wp_action(wp)
        mode = 'ocr' if atype == 'lookup_ocr' else 'aruco'
        try:
            confirm = int(action.get('confirm_frames', 3) or 3)
        except (TypeError, ValueError):
            confirm = 3
        try:
            retries = int(action.get('retries', 3) or 3)
        except (TypeError, ValueError):
            retries = 3
        timeout = self._nav2_action_timeout(atype, action)
        if timeout >= 149.0:
            # Default watchdog when WP action omits timeout_sec.
            timeout = 40.0
        cmd_id = f'nav2_lookup_{self._nav2_seq_gen}_{reached_idx}_{int(time.time() * 1000)}'
        payload: Dict[str, Any] = {
            'cmd_id': cmd_id,
            'mode': mode,
            'confirm_frames': max(1, min(20, confirm)),
            'retries': max(1, min(10, retries)),
            'timeout_sec': max(5.0, min(90.0, timeout - 5.0)),
        }
        self._cam_action_need = True
        self._cam_need_last = None
        try:
            self._ensure_orbbec()
            if self._cam_stream_allowed():
                self._toggle_orbbec(color=True, depth=False)
        except Exception as exc:
            self.get_logger().warn(f'lookup camera on failed: {exc}')
        self._lookup_cmd_pub.publish(String(data=json.dumps(payload)))
        self._nav2_action_cmd_id = cmd_id
        self._nav2_action_kind = atype
        self._nav2_action_deadline = time.time() + timeout + 10.0
        detail = f'wp {reached_idx + 1}/{n} {atype}'
        self._nav2_set_mission('wait_action', detail)
        self._lookup.update({
            'state': 'running', 'message': detail, 'last_ok': None,
            'last_next_wp': None,
        })
        self.get_logger().info(f'Waypoint {reached_idx + 1}/{n} reached — {detail}')

    def _on_studio_lookup_result(self, msg: String) -> None:
        try:
            data = _json_from_ros_string(msg.data)
        except Exception:
            return
        if not self._nav2_seq_active or not self._nav2_action_cmd_id:
            return
        if str(data.get('cmd_id', '')) != self._nav2_action_cmd_id:
            return
        if self._nav2_action_kind not in ('lookup_ocr', 'lookup_aruco'):
            return
        ok = bool(data.get('success'))
        kind = self._nav2_action_kind
        text = str(data.get('message') or '')
        next_wp = data.get('next_wp')
        try:
            next_wp_i = int(next_wp) if next_wp is not None else None
        except (TypeError, ValueError):
            next_wp_i = None
        self._lookup.update({
            'state': 'idle',
            'message': text,
            'last_ok': ok,
            'last_next_wp': next_wp_i,
        })
        self._lookup_camera_off()

        n = len(self._nav2_seq_wps)
        # _nav2_seq_index was already advanced to reached+1 before the action.
        if ok and next_wp_i is not None and 1 <= next_wp_i <= n:
            jump_idx = next_wp_i - 1  # 1-based → 0-based
            self._nav2_seq_index = jump_idx
            self._nav2_seq_attempt = 1
            self.get_logger().info(
                f'{kind} ok — jump to waypoint {next_wp_i}/{n} ({text})')
            self._nav2_finish_action_then_next(
                f'{kind} ok → wp {next_wp_i}')
            return

        reason = f'{kind} failed'
        if ok and next_wp_i is not None:
            reason = f'{kind} failed — wp {next_wp_i} not in sequence'
        elif text:
            reason = f'{kind} failed — {text}'
        self.get_logger().warn(f'{reason} — continuing normal sequence')
        self._nav2_finish_action_then_next(reason)

    def _on_studio_detect_align_status(self, msg: String) -> None:
        try:
            data = _json_from_ros_string(msg.data)
        except Exception:
            return
        if not isinstance(data, dict):
            return
        bbox = data.get('bbox')
        obj_cx = None
        frame_cx = None
        if isinstance(bbox, (list, tuple)) and len(bbox) >= 4:
            try:
                obj_cx = round(0.5 * (float(bbox[0]) + float(bbox[2])), 1)
            except (TypeError, ValueError):
                obj_cx = None
        size = data.get('size')
        if isinstance(size, (list, tuple)) and size:
            try:
                frame_cx = round(0.5 * float(size[0]), 1)
            except (TypeError, ValueError):
                frame_cx = None
        self._detect_align = {
            'state': str(data.get('state') or 'idle'),
            'message': str(data.get('message') or ''),
            'aligned': bool(data.get('aligned')),
            'stopped': bool(data.get('stopped', True)),
            'depth_m': data.get('depth_m'),
            'depth_horizontal_m': data.get('depth_horizontal_m'),
            'u_norm': data.get('u_norm'),
            'bbox': bbox,
            'conf': data.get('conf'),
            'class_name': str(data.get('class_name') or ''),
            'confirm': int(data.get('confirm') or 0),
            'confirm_need': int(data.get('confirm_need') or 0),
            'obj_cx': obj_cx,
            'frame_cx': frame_cx,
            'oms_angle_deg': data.get('oms_angle_deg'),
            'gripper_u': data.get('gripper_u'),
            'last_ok': self._detect_align.get('last_ok'),
            'last_result': self._detect_align.get('last_result') or '',
        }

    def _on_oms_angle(self, msg: Float64) -> None:
        try:
            self._oms['rotation']['angle_deg'] = round(float(msg.data), 2)
            self._oms['stamp'] = time.time()
            self._refresh_oms_pose()
        except (TypeError, ValueError):
            pass

    def _on_oms_mm(self, axis: str, msg: Float64) -> None:
        try:
            self._oms[axis]['pos_mm'] = round(float(msg.data), 1)
            self._oms['stamp'] = time.time()
            self._refresh_oms_pose()
        except (TypeError, ValueError, KeyError):
            pass

    def _on_oms_axis_status(self, axis: str, msg: String) -> None:
        try:
            data = _json_from_ros_string(msg.data)
        except Exception:
            return
        if not isinstance(data, dict):
            return
        slot = self._oms.setdefault(axis, {})
        if data.get('angle_deg') is not None:
            try:
                slot['angle_deg'] = round(float(data['angle_deg']), 2)
            except (TypeError, ValueError):
                pass
        if data.get('pos_mm') is not None:
            try:
                slot['pos_mm'] = round(float(data['pos_mm']), 1)
            except (TypeError, ValueError):
                pass
        if 'homed' in data:
            slot['homed'] = bool(data.get('homed'))
        if data.get('state') is not None:
            slot['state'] = str(data.get('state') or '')
        if data.get('pose') is not None:
            slot['pose'] = str(data.get('pose') or '')
        if 'home_switch' in data:
            slot['home_switch'] = bool(data.get('home_switch'))
        if 'up_sw' in data or 'up_switch' in data:
            slot['up_sw'] = bool(data.get('up_sw', data.get('up_switch')))
        if 'down_sw' in data or 'down_switch' in data:
            slot['down_sw'] = bool(data.get('down_sw', data.get('down_switch')))
        if axis == 'home':
            if isinstance(data.get('steps'), dict):
                slot['steps'] = data['steps']
            if data.get('sequence') is not None:
                slot['sequence'] = data.get('sequence')
            if data.get('current') is not None:
                slot['current'] = str(data.get('current') or '')
            if data.get('failed_step') is not None:
                slot['failed_step'] = str(data.get('failed_step') or '')
            if data.get('message') is not None:
                slot['message'] = str(data.get('message') or '')
        self._oms['stamp'] = time.time()
        if axis in ('rotation', 'lift', 'linear'):
            self._refresh_oms_pose()

    def _on_oms_result(self, axis: str, msg: String) -> None:
        try:
            data = _json_from_ros_string(msg.data)
        except Exception:
            data = {'raw': _ros_string(msg.data)}
        if not isinstance(data, dict):
            data = {'raw': _ros_string(msg.data)}
        slot = self._oms.setdefault('last_result', {})
        slot[axis] = data
        self._oms['stamp'] = time.time()

    def _on_oms_status(self, msg: String) -> None:
        self._on_oms_axis_status('rotation', msg)

    def _refresh_oms_pose(self) -> None:
        rot = self._oms.get('rotation') or {}
        lift = self._oms.get('lift') or {}
        linear = self._oms.get('linear') or {}
        yaw = float(rot.get('angle_deg') or 0.0)
        lift_mm = float(lift.get('pos_mm') or 0.0)
        lin_mm = float(linear.get('pos_mm') or 0.0)
        try:
            from oms.kinematics import oms_analysis
            viz = getattr(self, '_oms_viz_cache', None)
            if viz is None:
                try:
                    from oms.config_io import public_config
                    viz = public_config().get('oms_viz') or {}
                    self._oms_viz_cache = viz
                except Exception:
                    viz = {}
            self._oms['pose'] = oms_analysis(
                yaw, lift_mm, lin_mm,
                lift_travel_mm=float(viz.get('lift_travel_mm') or 300.0),
                linear_home_x_mm=float(viz.get('linear_home_x_mm') or 220.0),
                linear_travel_mm=float(viz.get('linear_travel_mm') or 110.0),
                linear_max_reach_mm=float(viz.get('linear_max_reach_mm') or 330.0),
                z_top_mm=float(viz.get('z_top_mm') or 455.0),
                z_floor_mm=float(viz.get('z_floor_mm') or 155.0),
            )
        except Exception:
            import math
            r = (220.0 + lin_mm) / 1000.0
            rad = math.radians(yaw)
            z = max(0.155, (455.0 - lift_mm) / 1000.0)
            self._oms['pose'] = {
                'x_m': round(r * math.cos(rad), 4),
                'y_m': round(r * math.sin(rad), 4),
                'z_m': round(z, 4),
                'x_mm': round(r * math.cos(rad) * 1000.0, 1),
                'y_mm': round(r * math.sin(rad) * 1000.0, 1),
                'z_mm': round(z * 1000.0, 1),
                'yaw_deg': round(yaw, 2),
                'lift_mm': round(lift_mm, 1),
                'linear_mm': round(lin_mm, 1),
                'front': '+X',
            }

    def _detect_align_telemetry(self) -> Dict[str, Any]:
        """Lean Detect+Align fields for SSE (no live bbox spam)."""
        d = self._detect_align or {}
        return {
            'state': d.get('state') or 'idle',
            'message': d.get('message') or '',
            'aligned': bool(d.get('aligned')),
            'class_name': d.get('class_name') or '',
            'depth_m': d.get('depth_m'),
            'last_ok': d.get('last_ok'),
            'last_result': d.get('last_result') or '',
            'oms_angle_deg': d.get('oms_angle_deg'),
        }

    def _oms_snapshot_lean(self) -> Dict[str, Any]:
        snap = self._oms_snapshot()
        # Drop bulky per-axis dicts from SSE when navigating.
        if self.session.get('phase') == 'deploy':
            lean = {
                'ready': snap.get('ready'),
                'source': snap.get('source'),
                'homed': snap.get('homed'),
                'state': snap.get('state'),
                'angle_deg': snap.get('angle_deg'),
            }
            return lean
        return snap

    def _oms_snapshot(self) -> Dict[str, Any]:
        standalone = bool(self.pm.oms_standalone_alive())
        nodes = bool(self.pm.oms_nodes_alive())
        fresh = (time.time() - float(self._oms.get('stamp') or 0.0)) < 3.0
        ready = nodes or fresh
        if standalone:
            source = 'standalone'
        elif self.pm.stacks_alive().get('nav') and (nodes or fresh):
            source = 'commission'
        elif ready:
            source = 'external'
        else:
            source = 'none'
        payload = dict(self._oms)
        payload['ready'] = ready
        payload['source'] = source
        payload['standalone'] = standalone
        # Keep old keys so Detect+Align HUD still works.
        rot = self._oms.get('rotation') or {}
        payload['angle_deg'] = rot.get('angle_deg')
        payload['homed'] = bool(rot.get('homed'))
        payload['state'] = str(rot.get('state') or '')
        payload['home_switch'] = bool(rot.get('home_switch'))
        payload['hardware'] = OMS_HARDWARE
        return payload

    def _oms_api_status(self) -> Dict[str, Any]:
        return {
            'ok': True,
            'stacks': self.pm.stacks_alive(),
            'oms': self._oms_snapshot(),
        }

    def _oms_config(self) -> Dict[str, Any]:
        try:
            from oms.config_io import public_config, yaml_paths
            return {
                'ok': True,
                'config': public_config(),
                'paths': [str(p) for p in yaml_paths()],
            }
        except Exception as exc:
            return {'ok': False, 'error': str(exc), 'config': {}}

    def _oms_home_and_sleep(self) -> Dict[str, Any]:
        """Lazy-start OMS (with Nav Titan if needed), full home, then sleep.

        Dashboard Home OMS button — keeps CPU low until mission WP wakes OMS.
        """
        if (
            self.session.get('deploy_module') == 'nav'
            and not self.session.get('oms_enabled', False)
        ):
            msg = (
                'Tick "Enable OMS" before Start Nav2, then press Home OMS. '
                'The tick does not start OMS — only Home OMS / a WP does.')
            return {'ok': False, 'error': msg, 'message': msg}
        # Commission always allowed; Home OMS also marks enabled for this session.
        self.session['oms_enabled'] = True
        robot = str(self.session.get('robot') or 'allterrain')
        try:
            self.pm.ensure_oms_with_nav(robot)
        except Exception as exc:
            return {'ok': False, 'error': str(exc), 'message': str(exc)}
        # Wait until home axis status appears (node up).
        t0 = time.time()
        while time.time() - t0 < 12.0:
            if self.pm.oms_nodes_alive():
                break
            time.sleep(0.25)
        if not self.pm.oms_nodes_alive():
            msg = 'OMS node did not start — check logs'
            self._append_oms_log(msg)
            return {'ok': False, 'error': msg, 'message': msg}

        self._oms_set_power('wake')
        time.sleep(0.3)
        result_store = self._oms.setdefault('last_result', {})
        result_store.pop('home', None)
        cmd_id = f'studio_home_{int(time.time() * 1000)}'
        payload = {'cmd_id': cmd_id, 'mode': 'home'}
        self._oms_cmd_pubs['home'].publish(String(data=json.dumps(payload)))
        self.get_logger().info(f'OMS Home OMS — cmd {cmd_id}')
        self._append_oms_log(f'Home OMS started id={cmd_id}')

        deadline = time.time() + 180.0
        ok = False
        message = 'timeout'
        while time.time() < deadline:
            res = result_store.get('home')
            if isinstance(res, dict) and str(res.get('cmd_id') or '') == cmd_id:
                ok = bool(res.get('success'))
                message = str(res.get('message') or '')
                break
            # Also accept a home result without matching id (older OMS).
            if isinstance(res, dict) and res.get('success') is not None and (
                    time.time() - t0) > 2.0:
                ok = bool(res.get('success'))
                message = str(res.get('message') or 'home')
                if 'cmd_id' not in res or not res.get('cmd_id'):
                    break
            time.sleep(0.1)

        self._oms_set_power('sleep')
        summary = (
            f'Home OMS {"OK" if ok else "FAILED"} — {message}; OMS sleeping')
        self._append_oms_log(summary)
        self.get_logger().info(summary)
        return {
            'ok': bool(ok),
            'homed': bool(ok),
            'message': summary,
            'oms': self._oms_snapshot(),
        }

    def _oms_send_cmd(self, body: Dict[str, Any]) -> Dict[str, Any]:
        axis = str(body.get('axis') or body.get('mode') or '').strip().lower()
        aliases = {
            'rotate': 'rotation', 'yaw': 'rotation',
            'belt': 'lift',
            'slide': 'linear',
            'vert': 'vertical',
            'wrist': 'rotation_servo', 'rot_servo': 'rotation_servo',
            'grip': 'gripper',
            'allhome': 'home', 'boot': 'home',
        }
        axis = aliases.get(axis, axis)
        if axis not in self._oms_cmd_pubs:
            raise ValueError(
                f'unknown axis {axis!r}. Use rotation, lift, linear, '
                'vertical, rotation_servo, gripper, home')
        # Full Home OMS flow (lazy start + sleep) for dashboard button.
        if axis == 'home' and str(body.get('mode') or 'home').lower() in (
                'home', 'allhome', 'boot'):
            if body.get('sleep_after', True):
                return self._oms_home_and_sleep()
        payload = dict(body)
        payload.pop('axis', None)
        mode = str(payload.get('mode') or 'goto').strip().lower()
        payload['mode'] = mode
        payload.setdefault('cmd_id', f'studio_{axis}_{int(time.time() * 1000)}')
        if axis == 'rotation' and mode == 'goto' and 'angle_deg' not in payload:
            raise ValueError('rotation goto needs angle_deg')
        if axis in ('lift', 'linear') and mode == 'goto' and 'pos_mm' not in payload:
            raise ValueError(f'{axis} goto needs pos_mm')
        if axis in ('vertical', 'rotation_servo', 'gripper') and mode == 'goto':
            if 'angle_deg' not in payload:
                raise ValueError(f'{axis} goto needs angle_deg')
        self._ensure_oms_ready_for_work()
        self._oms_cmd_pubs[axis].publish(String(data=json.dumps(payload)))
        self.get_logger().info(f'OMS cmd {axis} {payload}')
        return {'ok': True, 'axis': axis, 'sent': payload}


    def _on_studio_detect_align_result(self, msg: String) -> None:
        try:
            data = _json_from_ros_string(msg.data)
        except Exception:
            data = {}
        if isinstance(data, dict):
            ok = bool(data.get('success'))
            text = str(data.get('message') or '')
            self._detect_align['last_ok'] = ok
            self._detect_align['last_result'] = text
            if data.get('depth_m') is not None:
                self._detect_align['depth_m'] = data.get('depth_m')
            if data.get('depth_horizontal_m') is not None:
                self._detect_align['depth_horizontal_m'] = data.get(
                    'depth_horizontal_m')
            if data.get('u_norm') is not None:
                self._detect_align['u_norm'] = data.get('u_norm')
            if data.get('class_name'):
                self._detect_align['class_name'] = str(data.get('class_name'))
            if data.get('oms_angle_deg') is not None:
                self._detect_align['oms_angle_deg'] = data.get('oms_angle_deg')
            if data.get('gripper_u') is not None:
                self._detect_align['gripper_u'] = data.get('gripper_u')
        self._on_studio_align_result(msg)

    def _on_studio_align_result(self, msg: String) -> None:
        try:
            data = _json_from_ros_string(msg.data)
        except Exception:
            return
        if self._commission.get('active'):
            if not self._nav2_action_cmd_id:
                return
            if str(data.get('cmd_id', '')) != self._nav2_action_cmd_id:
                return
            ok = bool(data.get('success'))
            mode = str(data.get('mode') or 'align')
            text = str(data.get('message') or '')
            if mode in ('detect_align', 'oms_pick'):
                self._commission['last_detect_ok'] = ok
                if data.get('class_name'):
                    self._commission['detected_object'] = normalize_object_name(
                        data.get('class_name'))
            if mode == 'oms_drop':
                self._commission['last_detect_ok'] = ok
            self.get_logger().info(
                f'commission {mode} result ok={ok} {text}')
            self._commission_on_action_done(
                f'{mode} {"ok" if ok else "failed"}')
            return
        if not self._nav2_seq_active or not self._nav2_action_cmd_id:
            return
        if str(data.get('cmd_id', '')) != self._nav2_action_cmd_id:
            return
        ok = bool(data.get('success'))
        mode = str(data.get('mode') or 'align')
        text = str(data.get('message') or '')
        self.get_logger().info(
            f'{mode} result ok={ok} {text}')
        # Align+distance+OMS: after successful align_move, run OMS sequence.
        if self._nav2_action_kind == 'align_move_oms' and ok and self._nav2_oms_wp:
            reached_idx = max(0, self._nav2_seq_index - 1)
            self._nav2_start_oms_action(self._nav2_oms_wp, reached_idx)
            return
        if self._nav2_action_kind == 'align_move_oms' and not ok:
            self._nav2_oms_wp = None
        self._nav2_finish_action_then_next(
            f'{mode} {"ok" if ok else "failed"}')

    def _nav2_begin_wait_then_next(self, reached_idx: int) -> None:
        if self._commission.get('active'):
            self._nav2_seq_index = reached_idx + 1
            self._nav2_seq_attempt = 1
            self._commission_on_nav_reached(reached_idx)
            return
        wp = self._nav2_seq_wps[reached_idx]
        self._nav2_seq_index = reached_idx + 1
        self._nav2_seq_attempt = 1
        atype = str(self._wp_action(wp).get('type') or '')
        heavy = atype in (
            'detect_align', 'align', 'align_move', 'align_move_oms',
            'oms', 'cobra_align', 'wait_light', 'lookup_ocr', 'lookup_aruco')
        if heavy and not self._commission.get('active'):
            self._nav2_pause_heavy(f'wp action {atype}')
        if atype == 'detect_align':
            self._nav2_start_detect_align(wp, reached_idx)
            return
        if atype in ('align', 'align_move', 'align_move_oms'):
            self._nav2_start_align_action(wp, reached_idx, atype)
            return
        if atype == 'oms':
            self._ensure_oms_ready_for_work()
            self._nav2_start_oms_action(wp, reached_idx)
            return
        if atype == 'cobra_align':
            self._nav2_start_cobra_action(wp, reached_idx)
            return
        if atype == 'wait_light':
            self._nav2_start_wait_light(wp, reached_idx)
            return
        if atype in ('lookup_ocr', 'lookup_aruco'):
            self._nav2_start_lookup_action(wp, reached_idx, atype)
            return
        wait_s = self._nav2_wait_seconds(wp)
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
        if not self._nav2_seq_active:
            return
        if self._nav2_action_cmd_id:
            remaining = self._nav2_action_deadline - time.time()
            kind = self._nav2_action_kind or 'action'
            self._nav2_set_mission(
                'wait_action',
                f'{kind} {max(0.0, remaining):.1f}s')
            if remaining > 0.0:
                return
            self.get_logger().warn(
                f'Waypoint {kind} timed out — continuing mission')
            try:
                self._align_cancel_pub.publish(Empty())
            except Exception:
                pass
            try:
                self._cobra_cancel_pub.publish(Empty())
            except Exception:
                pass
            try:
                self._detect_cancel_pub.publish(Empty())
            except Exception:
                pass
            if kind in ('lookup_ocr', 'lookup_aruco'):
                try:
                    self._lookup_cancel_pub.publish(Empty())
                except Exception:
                    pass
                self._lookup_camera_off()
            if kind in ('oms', 'align_move_oms'):
                self._nav2_cancel_oms_job()
            self._nav2_finish_action_then_next(f'{kind} timeout')
            return
        if self._nav2_wait_until <= 0.0:
            return
        remaining = self._nav2_wait_until - time.time()
        self._nav2_set_mission('wait_action', f'waiting {max(0.0, remaining):.1f}s')
        if remaining > 0.0:
            return
        self._light_gen += 1
        self._light_tower_set('off')
        self._nav2_finish_action_then_next('Waypoint wait complete')

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
            fut.result()
            return True
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

    def _hw_test_stop_subs(self) -> None:
        for sub in self._hw_subs:
            try:
                self.destroy_subscription(sub)
            except Exception:
                pass
        self._hw_subs = []
        self._hw_topic_ids = {}
        self._hw_store.clear()

    def _hw_bind_topic(self, topic: str, channel_id: str) -> None:
        self._hw_topic_ids.setdefault(topic, []).append(channel_id)

    def _hw_test_start_subs(self, robot: str) -> Dict[str, Any]:
        self._hw_test_stop_subs()
        specs = build_channel_specs(robot, include_lidar=True)
        self._hw_store.reset(robot, specs)
        for spec in specs:
            self._hw_bind_topic(spec['topic'], spec['id'])

        subscribed = set()

        def sub_once(topic: str, msg_type, cb, qos=10) -> None:
            if topic in subscribed or topic in ('/battery_state', '/scan'):
                return
            subscribed.add(topic)
            self._hw_subs.append(
                self.create_subscription(msg_type, topic, cb, qos))

        for topic, ids in list(self._hw_topic_ids.items()):
            if topic in ('/battery_state', '/scan'):
                continue
            msg_hint = ''
            for spec in specs:
                if spec['topic'] == topic:
                    msg_hint = str(spec.get('msg') or '')
                    break
            if 'LaserScan' in msg_hint:
                # YDLidar offers BEST_EFFORT SensorDataQoS. A default depth-10
                # subscription is RELIABLE and therefore incompatible.
                sub_once(
                    topic, LaserScan,
                    lambda m, t=topic: self._hw_on_laser(t, m),
                    qos_profile_sensor_data)
            elif 'Imu' in msg_hint:
                sub_once(topic, Imu, lambda m, t=topic: self._hw_on_imu(t, m))
            elif 'Range' in msg_hint:
                sub_once(topic, Range, lambda m, t=topic: self._hw_on_range(t, m))
            elif 'Bool' in msg_hint:
                sub_once(topic, Bool, lambda m, t=topic: self._hw_on_bool(t, m))
            elif 'Float32' in msg_hint:
                sub_once(topic, Float32, lambda m, t=topic: self._hw_on_f32(t, m))
            elif 'String' in msg_hint:
                sub_once(topic, String, lambda m, t=topic: self._hw_on_string(t, m))
            else:
                sub_once(topic, Float64, lambda m, t=topic: self._hw_on_f64(t, m))

        return {
            'ok': True,
            'robot': robot,
            'channels': len(specs),
            'topics': sorted(self._hw_topic_ids.keys()),
        }

    def _hw_on_f64(self, topic: str, msg: Float64) -> None:
        for cid in self._hw_topic_ids.get(topic, []):
            self._hw_store.set_value(cid, round(float(msg.data), 3), topic)

    def _hw_on_f32(self, topic: str, msg: Float32) -> None:
        for cid in self._hw_topic_ids.get(topic, []):
            self._hw_store.set_value(cid, round(float(msg.data), 4), topic)

    def _hw_on_bool(self, topic: str, msg: Bool) -> None:
        for cid in self._hw_topic_ids.get(topic, []):
            self._hw_store.set_value(cid, bool(msg.data), topic)

    def _hw_on_string(self, topic: str, msg: String) -> None:
        for cid in self._hw_topic_ids.get(topic, []):
            self._hw_store.set_value(cid, _ros_string(msg.data)[:64], topic)

    def _hw_on_range(self, topic: str, msg: Range) -> None:
        r = float(msg.range)
        val = None if not math.isfinite(r) else round(r, 3)
        for cid in self._hw_topic_ids.get(topic, []):
            self._hw_store.set_value(cid, val, topic)

    def _hw_on_imu(self, topic: str, msg: Imu) -> None:
        q = msg.orientation
        yaw = yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))
        deg = round(math.degrees(yaw), 2)
        for cid in self._hw_topic_ids.get(topic, []):
            if cid.endswith('.yaw') or 'yaw' in cid:
                self._hw_store.set_value(cid, deg, topic)

    def _hw_on_laser(self, topic: str, msg: LaserScan) -> None:
        d = laser_min_range(msg.ranges, msg.range_min, msg.range_max)
        if d is None:
            return
        for cid in self._hw_topic_ids.get(topic, []):
            self._hw_store.set_value(cid, round(d, 3), topic)

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
        volt = 0.0 if math.isnan(v) else v
        self._battery = {
            'voltage': round(volt, 2),
            'percentage': round(pct, 1),
            'valid': True,
            'source': 'battery_state',
        }
        if self._hw_store.active:
            for cid in self._hw_topic_ids.get('/battery_state', []):
                if 'percent' in cid:
                    self._hw_store.set_value(cid, round(pct, 1), '/battery_state')
                else:
                    self._hw_store.set_value(cid, round(volt, 2), '/battery_state')

    def _cam_log(self, line: str) -> None:
        stamp = time.strftime('%H:%M:%S')
        self._cam_logs.append(f'{stamp} {line}')
        if len(self._cam_logs) > 80:
            self._cam_logs = self._cam_logs[-60:]
        self.get_logger().info(f'[camera] {line}')

    def _mission_active(self) -> bool:
        if self._commission.get('active'):
            return True
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

    # ── Module G commissioning mission ─────────────────────────────────────

    def _commission_idle_state(self) -> Dict[str, Any]:
        return {
            'active': False,
            'phase': 'idle',
            'pick_index': 0,
            'pick_retries': 0,
            'pick_total': 0,
            'current_pick': '',
            'detected_object': '',
            'target_drop': '',
            'last_detect_ok': False,
            'remaining': {},
            'lift_mm': 0.0,
            'nav_paused': False,
            'config': default_config(),
            'indexed': {},
            'events': [],
            'detail': '',
        }

    def _commission_reset(self) -> None:
        if self._commission.get('active'):
            self.get_logger().info('[commission] reset')
        if self._commission.get('nav_paused'):
            self._nav2_lifecycle_set(True)
        self._commission = self._commission_idle_state()

    def _commission_snapshot(self) -> Dict[str, Any]:
        snap = dict(self._commission)
        snap['objects'] = list(COMMISSION_OBJECTS)
        return snap

    def _commission_log(self, msg: str) -> None:
        events = list(self._commission.get('events') or [])
        events.append(f'{time.strftime("%H:%M:%S")} {msg}')
        if len(events) > 40:
            events = events[-30:]
        self._commission['events'] = events
        self.get_logger().info(f'[commission] {msg}')

    def _commission_set_phase(self, phase: str, detail: str = '') -> None:
        self._commission['phase'] = phase
        self._commission['detail'] = detail
        label = f'commission {phase}'
        if detail:
            label += f': {detail}'
        self._nav2_set_mission('running', label)

    def _commission_yaw_tol(self) -> float:
        cfg = self._commission.get('config') or {}
        try:
            return float(cfg.get('yaw_tolerance_deg', 1.0))
        except (TypeError, ValueError):
            return 1.0

    def _commission_max_retries(self) -> int:
        cfg = self._commission.get('config') or {}
        try:
            return max(1, int(cfg.get('detect_retries', 3)))
        except (TypeError, ValueError):
            return 3

    def _nav2_lifecycle_set(self, activate: bool) -> None:
        """Pause/resume heavy Nav2 nodes at pick/drop to save CPU."""
        try:
            from lifecycle_msgs.msg import Transition
            from lifecycle_msgs.srv import ChangeState
        except Exception as exc:
            self._commission_log(f'lifecycle msgs unavailable: {exc}')
            return
        nodes = (
            'controller_server',
            'planner_server',
            'bt_navigator',
        )
        transition_id = (
            Transition.TRANSITION_ACTIVATE if activate
            else Transition.TRANSITION_DEACTIVATE)
        label = 'activate' if activate else 'deactivate'
        for name in nodes:
            srv = f'/{name}/change_state'
            cli = self.create_client(ChangeState, srv)
            if not cli.wait_for_service(timeout_sec=0.8):
                self._commission_log(f'{label} skip {name} (no service)')
                continue
            req = ChangeState.Request()
            req.transition.id = transition_id
            try:
                fut = cli.call_async(req)
                t0 = time.time()
                while not fut.done() and time.time() - t0 < 2.0:
                    time.sleep(0.02)
                ok = bool(fut.done() and fut.result() and fut.result().success)
                self._commission_log(
                    f'Nav2 {label} {name}: {"ok" if ok else "fail"}')
            except Exception as exc:
                self._commission_log(f'Nav2 {label} {name} error: {exc}')
        self._commission['nav_paused'] = not activate
        self._nav2_cpu_paused = not activate

    def _nav2_pause_heavy(self, reason: str = '') -> None:
        """Deactivate planner/controller/bt while a WP action runs (CPU save)."""
        if self._nav2_cpu_paused:
            return
        self._cancel_nav2_goal_only()
        self.get_logger().info(
            f'Pause Nav2 heavy nodes{(": " + reason) if reason else ""}')
        self._nav2_lifecycle_set(False)

    def _nav2_resume_heavy(self) -> None:
        if not self._nav2_cpu_paused and not self._commission.get('nav_paused'):
            return
        self.get_logger().info('Resume Nav2 heavy nodes')
        self._nav2_lifecycle_set(True)
        time.sleep(0.35)

    def _commission_pause_nav(self) -> None:
        if self._commission.get('nav_paused'):
            return
        self._commission_log('Pause Nav2 planner/controller (CPU save)')
        self._nav2_pause_heavy('commission WP')

    def _commission_resume_nav(self) -> None:
        if not self._commission.get('nav_paused') and not self._nav2_cpu_paused:
            return
        self._commission_log('Resume Nav2 planner/controller')
        self._nav2_resume_heavy()
        time.sleep(0.1)

    def _commission_current_pick_wp(self) -> Optional[dict]:
        indexed = self._commission.get('indexed') or {}
        picks = indexed.get('picks') or []
        idx = int(self._commission.get('pick_index') or 0)
        if 0 <= idx < len(picks):
            return picks[idx]
        return None

    def _commission_current_label(self) -> str:
        indexed = self._commission.get('indexed') or {}
        labels = indexed.get('pick_labels') or []
        idx = int(self._commission.get('pick_index') or 0)
        if 0 <= idx < len(labels):
            return str(labels[idx])
        return str(self._commission.get('current_pick') or '')

    def _commission_nav_to(self, wp: dict) -> None:
        self._commission_resume_nav()
        self._nav2_seq_wps = [wp]
        self._nav2_seq_index = 0
        self._nav2_seq_attempt = 1
        self._nav2_wait_until = 0.0
        self._nav2_action_cmd_id = ''
        self._nav2_action_deadline = 0.0
        self._nav2_action_kind = ''
        self._nav2_seq_active = True
        self._send_nav2_seq_pose()

    def _commission_begin_pick(self) -> None:
        indexed = self._commission.get('indexed') or {}
        picks = indexed.get('picks') or []
        labels = indexed.get('pick_labels') or []
        remaining = dict(self._commission.get('remaining') or {})
        idx = int(self._commission.get('pick_index') or 0)
        if idx < 0 or idx >= len(picks):
            idx2 = next_pick_index(labels, remaining, -1)
            if idx2 is None:
                self._commission_go_home()
                return
            idx = idx2
            self._commission['pick_index'] = idx
        label = labels[idx] if idx < len(labels) else f'pick{idx + 1}'
        if int(remaining.get(label, 0)) <= 0:
            nxt = next_pick_index(labels, remaining, idx)
            if nxt is None:
                self._commission_go_home()
                return
            self._commission['pick_index'] = nxt
            self._commission_begin_pick()
            return
        lifts = indexed.get('pick_lifts_mm') or {}
        try:
            lift_mm = float(lifts.get(label, 0.0))
        except (TypeError, ValueError):
            lift_mm = 0.0
        self._commission['current_pick'] = label
        self._commission['lift_mm'] = lift_mm
        self._commission['pick_retries'] = 0
        self._commission['yaw_retries'] = 0
        self._commission['detected_object'] = ''
        self._commission['target_drop'] = ''
        self._commission['last_detect_ok'] = False
        left = int(remaining.get(label, 0))
        self._commission_log(
            f'Navigate to {label} (left {left}, lift {lift_mm:.0f}mm)')
        self._commission_set_phase('nav_pick', label)
        self._commission_nav_to(picks[idx])

    def _commission_on_nav_reached(self, reached_idx: int) -> None:
        del reached_idx
        phase = str(self._commission.get('phase') or '')
        if phase == 'nav_pick':
            self._commission_pause_nav()
            mode = str(
                (self._commission.get('config') or {}).get('pick_mode')
                or 'detect_oms')
            if mode == 'align_move_detect_oms':
                self._commission_start_align_move_pick()
            else:
                # Mode 1: Detect+Align OMS pick (optional brief yaw first).
                self._commission_start_yaw_align(kind='pick')
        elif phase == 'nav_drop':
            self._commission_pause_nav()
            self._commission_start_yaw_align(kind='drop')
        elif phase == 'nav_home':
            self._commission_finish()
        else:
            self._commission_log(f'unexpected nav reach in phase {phase}')
            self._commission_finish()

    def _commission_start_align_move_pick(self) -> None:
        """Mode 2: Align + distance, then oms_pick."""
        wp = self._commission_current_pick_wp()
        if wp is None:
            self._commission_skip_pick('no pick waypoint')
            return
        pick = str(self._commission.get('current_pick') or 'pick')
        self._commission_set_phase('align_move_pick', pick)
        cfg = self._commission.get('config') or {}
        try:
            dist = float(wp.get('pick_distance_m', cfg.get('pick_distance_m', 0.15)))
        except (TypeError, ValueError):
            dist = 0.15
        dist = min(5.0, max(0.03, dist))
        try:
            yaw = float(wp.get('yaw', 0.0) or 0.0)
        except (TypeError, ValueError):
            yaw = 0.0
        cmd_id = f'cm_am_{self._nav2_seq_gen}_{int(time.time() * 1000)}'
        payload = {
            'cmd_id': cmd_id,
            'mode': 'align_move',
            'yaw': yaw,
            'distance_m': dist,
            'yaw_tolerance_deg': self._commission_yaw_tol(),
        }
        timeout = self._nav2_action_timeout('align_move', {'distance_m': dist})
        self._align_cmd_pub.publish(String(data=json.dumps(payload)))
        self._nav2_action_cmd_id = cmd_id
        self._nav2_action_kind = 'align_move'
        self._nav2_action_deadline = time.time() + timeout + 15.0
        self._commission_log(
            f'{pick} align+move dist={dist:.3f}m yaw={math.degrees(yaw):.1f}°')

    def _commission_start_yaw_align(self, kind: str = 'pick') -> None:
        if kind == 'drop':
            indexed = self._commission.get('indexed') or {}
            drop_sticker = str(self._commission.get('target_drop') or '')
            wp = resolve_drop_wp(indexed.get('drops') or {}, drop_sticker)
            phase = 'yaw_drop'
            tag = drop_sticker or 'drop'
        else:
            wp = self._commission_current_pick_wp()
            phase = 'yaw_pick'
            tag = str(self._commission.get('current_pick') or 'pick')
        if wp is None:
            self._commission_skip_pick('no waypoint for yaw align')
            return
        self._commission_set_phase(phase, tag)
        try:
            yaw = float(wp.get('yaw', 0.0) or 0.0)
        except (TypeError, ValueError):
            yaw = 0.0
        cmd_id = f'cm_{self._nav2_seq_gen}_{phase}_{int(time.time() * 1000)}'
        payload: Dict[str, Any] = {
            'cmd_id': cmd_id,
            'mode': 'align',
            'yaw': yaw,
            'yaw_tolerance_deg': self._commission_yaw_tol(),
        }
        timeout = self._nav2_action_timeout('align', {})
        self._align_cmd_pub.publish(String(data=json.dumps(payload)))
        self._nav2_action_cmd_id = cmd_id
        self._nav2_action_kind = 'align'
        self._nav2_action_deadline = time.time() + timeout + 15.0
        self._commission_log(
            f'{tag} yaw align {math.degrees(yaw):.1f}° '
            f'(±{self._commission_yaw_tol():.1f}°)')

    def _commission_start_oms_pick(self) -> None:
        wp = self._commission_current_pick_wp()
        if wp is None:
            self._commission_skip_pick('no pick waypoint')
            return
        pick = str(self._commission.get('current_pick') or '')
        lift_mm = float(self._commission.get('lift_mm') or 0.0)
        attempt = int(self._commission.get('pick_retries') or 0) + 1
        self._commission_set_phase('oms_pick', f'{pick} try {attempt}')
        self._commission['last_detect_ok'] = False
        self._ensure_oms_ready_for_work()
        cmd_id = f'cm_pick_{self._nav2_seq_gen}_{int(time.time() * 1000)}'
        payload = {
            'cmd_id': cmd_id,
            'mode': 'oms_pick',
            'lift_mm': lift_mm,
            'timeout_sec': 180.0,
        }
        self._detect_cmd_pub.publish(String(data=json.dumps(payload)))
        self._nav2_action_cmd_id = cmd_id
        self._nav2_action_kind = 'oms_pick'
        self._nav2_action_deadline = time.time() + 200.0
        self._commission_log(
            f'OMS pick at {pick} lift={lift_mm:.0f}mm (try {attempt})')

    def _commission_start_oms_drop(self) -> None:
        """Run per-drop OMS sequence from the drop waypoint (dashboard)."""
        drop = str(self._commission.get('target_drop') or 'drop')
        indexed = self._commission.get('indexed') or {}
        drop_wp = resolve_drop_wp(indexed.get('drops') or {}, drop) or {}
        action = drop_wp.get('action') if isinstance(drop_wp, dict) else {}
        if not isinstance(action, dict):
            action = {}
        sequence = action.get('sequence')
        if not sequence:
            # Fallback: home only (legacy stub behaviour).
            sequence = ['home']
        self._commission_set_phase('oms_drop', drop)
        cmd_id = f'cm_drop_{self._nav2_seq_gen}_{int(time.time() * 1000)}'
        self._nav2_action_cmd_id = cmd_id
        self._nav2_action_kind = 'oms_drop'
        try:
            timeout = float(action.get('timeout_sec', 120.0) or 120.0)
        except (TypeError, ValueError):
            timeout = 120.0
        timeout = min(600.0, max(10.0, timeout))
        self._nav2_action_deadline = time.time() + timeout + 15.0
        self._commission_log(
            f'Drop {drop} — OMS sequence ({len(sequence) if isinstance(sequence, list) else "?"} steps)')
        self._nav2_oms_cancel.clear()
        self._ensure_oms_ready_for_work()
        self._nav2_oms_thread = threading.Thread(
            target=self._commission_run_drop_oms_job,
            args=(cmd_id, drop_wp if isinstance(drop_wp, dict) else {}),
            daemon=True,
            name='cm_drop_oms',
        )
        self._nav2_oms_thread.start()

    def _commission_run_drop_oms_job(self, cmd_id: str, drop_wp: dict) -> None:
        from skill23_studio.nav.wp_oms_sequence import run_wp_oms_sequence
        action = drop_wp.get('action') if isinstance(drop_wp.get('action'), dict) else {}
        result_store = self._oms.setdefault('last_result', {})
        self._oms_set_power('wake')
        time.sleep(0.2)

        def _log(msg: str, level: str = 'info') -> None:
            if level == 'warn':
                self.get_logger().warn(f'[cm_drop_oms] {msg}')
            else:
                self.get_logger().info(f'[cm_drop_oms] {msg}')

        try:
            ok, message = run_wp_oms_sequence(
                pubs=self._oms_cmd_pubs,
                result_store=result_store,
                sequence=action.get('sequence') or ['home'],
                lift_mm=float(action.get('lift_mm', 0.0) or 0.0),
                linear_mm=float(action.get('linear_mm', 0.0) or 0.0),
                rotation_deg=float(action.get('rotation_deg', 0.0) or 0.0),
                wait_sec=float(action.get('wait_sec', 1.0) or 1.0),
                axis_timeout_sec=float(action.get('axis_timeout_sec', 60.0) or 60.0),
                home_timeout_sec=float(action.get('home_timeout_sec', 180.0) or 180.0),
                log=_log,
                cancel_check=self._nav2_oms_cancel.is_set,
            )
        except Exception as exc:
            ok, message = False, f'drop OMS error: {exc}'
            self.get_logger().error(f'[cm_drop_oms] {message}')
        finally:
            self._oms_set_power('sleep')

        if self._nav2_action_cmd_id != cmd_id:
            return
        self._commission['last_detect_ok'] = bool(ok)
        self.get_logger().info(f'drop OMS ok={ok} {message}')
        self._commission_on_action_done(
            f'oms_drop {"ok" if ok else "failed"}')

    def _ensure_oms_ready_for_work(self) -> None:
        """Lazy-start OMS alongside Nav Titan if needed, then wake."""
        if (
            self.session.get('deploy_module') == 'nav'
            and not self.session.get('oms_enabled', False)
        ):
            raise RuntimeError(
                'OMS not enabled — tick "Enable OMS" before Start Nav2 '
                '(tick does not start OMS; Home OMS / WP starts it)')
        robot = str(self.session.get('robot') or 'allterrain')
        try:
            if not self.pm.oms_nodes_alive():
                self.pm.ensure_oms_with_nav(robot)
                # Give node a moment to advertise.
                t0 = time.time()
                while time.time() - t0 < 8.0:
                    if self.pm.oms_nodes_alive():
                        break
                    time.sleep(0.2)
            self._oms_set_power('wake')
        except Exception as exc:
            self.get_logger().warn(f'ensure OMS failed: {exc}')
            raise

    def _commission_go_to_drop(self) -> None:
        indexed = self._commission.get('indexed') or {}
        drop_sticker = str(self._commission.get('target_drop') or '')
        drop_wp = resolve_drop_wp(indexed.get('drops') or {}, drop_sticker)
        if drop_wp is None:
            self._commission_skip_pick(f'no waypoint for {drop_sticker or "?"}')
            return
        self._commission_set_phase('nav_drop', drop_sticker)
        self._commission_log(f'Navigate to {drop_sticker}')
        self._commission_nav_to(drop_wp)

    def _commission_skip_pick(self, reason: str) -> None:
        pick = str(self._commission.get('current_pick') or '')
        self._commission_log(f'Skip {pick}: {reason}')
        labels = (self._commission.get('indexed') or {}).get('pick_labels') or []
        remaining = dict(self._commission.get('remaining') or {})
        idx = int(self._commission.get('pick_index') or 0)
        nxt = next_pick_index(labels, remaining, idx)
        if nxt is None:
            self._commission_go_home()
            return
        self._commission['pick_index'] = nxt
        self._commission_begin_pick()

    def _commission_after_drop_success(self) -> None:
        label = self._commission_current_label()
        remaining = remaining_after_visit(
            dict(self._commission.get('remaining') or {}), label)
        self._commission['remaining'] = remaining
        left = int(remaining.get(label, 0))
        self._commission_log(f'{label} done — remaining {left}')
        labels = (self._commission.get('indexed') or {}).get('pick_labels') or []
        idx = int(self._commission.get('pick_index') or 0)
        nxt = next_pick_index(labels, remaining, idx)
        if nxt is None:
            self._commission_go_home()
            return
        self._commission['pick_index'] = nxt
        self._commission_begin_pick()

    def _commission_go_home(self) -> None:
        indexed = self._commission.get('indexed') or {}
        home = indexed.get('home')
        if isinstance(home, dict):
            self._commission_set_phase('nav_home', 'home')
            self._commission_log('All pick counters done — return home')
            self._commission_nav_to(home)
        else:
            self._commission_log('All pick counters done — no home waypoint')
            self._commission_finish()

    def _commission_finish(self) -> None:
        self._commission_resume_nav()
        self._commission_set_phase('done', 'complete')
        self._commission_log('Module G complete')
        self._nav2_seq_active = False
        self._nav2_wait_until = 0.0
        self._nav2_action_cmd_id = ''
        self._nav2_action_deadline = 0.0
        self._nav2_action_kind = ''
        self._nav2_set_mission('done', 'commission complete')
        self._commission['active'] = False

    def _commission_on_action_done(self, reason: str) -> None:
        phase = str(self._commission.get('phase') or '')
        self._nav2_wait_until = 0.0
        self._nav2_action_cmd_id = ''
        self._nav2_action_deadline = 0.0
        self._nav2_action_kind = ''

        if phase == 'yaw_pick':
            if 'failed' in reason.lower() or 'timeout' in reason.lower():
                self._commission_log(f'Pick yaw align failed: {reason}')
                # one retry then continue anyway so mission doesn't stick
                retries = int(self._commission.get('yaw_retries') or 0)
                if retries < 1:
                    self._commission['yaw_retries'] = retries + 1
                    self._commission_start_yaw_align(kind='pick')
                    return
            self._commission['yaw_retries'] = 0
            self._ensure_oms_ready_for_work()
            self._commission_start_oms_pick()
            return
        if phase == 'align_move_pick':
            if 'failed' in reason.lower() or 'timeout' in reason.lower():
                self._commission_log(f'Align+move failed: {reason} — trying oms_pick anyway')
            self._ensure_oms_ready_for_work()
            self._commission_start_oms_pick()
            return
        if phase == 'yaw_drop':
            if 'failed' in reason.lower() or 'timeout' in reason.lower():
                retries = int(self._commission.get('yaw_retries') or 0)
                if retries < 1:
                    self._commission['yaw_retries'] = retries + 1
                    self._commission_start_yaw_align(kind='drop')
                    return
            self._commission['yaw_retries'] = 0
            self._commission_start_oms_drop()
            return
        if phase == 'oms_pick':
            detect_ok = bool(self._commission.get('last_detect_ok'))
            obj = str(self._commission.get('detected_object') or '')
            # Sleep OMS after pick until drop sequence.
            try:
                self._oms_set_power('sleep')
            except Exception:
                pass
            if detect_ok and obj:
                drop_sticker = drop_sticker_for_object(
                    self._commission.get('config') or {}, obj)
                self._commission['target_drop'] = drop_sticker
                self._commission_log(
                    f'Picked {obj} → route to {drop_sticker or "?"}')
                if not drop_sticker:
                    self._commission_skip_pick('no drop assigned for object')
                    return
                indexed = self._commission.get('indexed') or {}
                if resolve_drop_wp(indexed.get('drops') or {}, drop_sticker) is None:
                    self._commission_skip_pick(
                        f'drop waypoint {drop_sticker} not on map')
                    return
                self._commission_go_to_drop()
                return
            retries = int(self._commission.get('pick_retries') or 0) + 1
            self._commission['pick_retries'] = retries
            max_r = self._commission_max_retries()
            self._commission_log(f'Pick failed ({retries}/{max_r}): {reason}')
            if retries < max_r:
                self._ensure_oms_ready_for_work()
                self._commission_start_oms_pick()
            else:
                self._commission_skip_pick('pick failed after retries')
            return
        if phase == 'oms_drop':
            self._commission_after_drop_success()
            return
        self._commission_log(f'action done in phase {phase}: {reason}')

    def _start_commission_mission(self, source: str) -> Dict[str, Any]:
        source = 'physical' if source == 'physical' else 'dashboard'
        if self._estop:
            raise RuntimeError('E-STOP active — release before start')
        if self._mission_active():
            return {
                'ok': True,
                'started': False,
                'reason': 'already_running',
                'source': self._mission_started_from,
            }
        name = self.session.get('map_name') or ''
        if not name:
            raise RuntimeError('No map loaded')
        wps = maps.load_waypoints(name)
        cfg = maps.load_commission_config(name)
        indexed = index_waypoints(wps, cfg)
        picks = indexed.get('picks') or []
        if not picks:
            msg = (
                'No pick locations — mark waypoints as pick1, pick2, … '
                'in commissioning mode')
            self._hw_alert = msg
            self._nav2_set_mission('idle', msg)
            return {
                'ok': False,
                'started': False,
                'reason': 'no_picks',
                'error': msg,
            }
        self._hw_alert = ''
        self._mission_started_from = source
        self._armed_wps = []
        self._armed_seq_name = 'commission'
        self._commission_reset()
        self._commission['active'] = True
        self._commission['config'] = cfg
        self._commission['indexed'] = indexed
        self._commission['pick_total'] = len(picks)
        self._commission['remaining'] = dict(indexed.get('pick_counts') or {})
        self._commission['pick_index'] = 0
        counts = self._commission['remaining']
        self._commission_log(
            f'Module G START from {source} — {len(picks)} pick(s) '
            f'counts={counts}')
        self._commission_begin_pick()
        return {
            'ok': True,
            'started': True,
            'source': source,
            'mode': 'commission',
            'picks': len(picks),
            'remaining': counts,
        }

    def _start_mission(self, source: str) -> Dict[str, Any]:
        """Start the armed mission. No-op if a mission is already running."""
        if self.session.get('deploy_module') == 'commission':
            return self._start_commission_mission(source)
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
        self._send_nav2_through(wps)
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
        self._cancel_nav2()
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
            data = _json_from_ros_string(msg.data)
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

    def _cached_toggle_ready(self) -> bool:
        now = time.time()
        if (now - self._toggle_ready_cache_t) < 2.0:
            return self._toggle_ready_cache
        try:
            ready = bool(self._orbbec_color_cli().service_is_ready())
        except Exception:
            ready = False
        self._toggle_ready_cache = ready
        self._toggle_ready_cache_t = now
        return ready

    def _orbbec_color_cli(self):
        if self._toggle_color_cli.service_is_ready():
            return self._toggle_color_cli
        if self._toggle_color_cli_ns.service_is_ready():
            return self._toggle_color_cli_ns
        if self._toggle_color_cli_root.service_is_ready():
            return self._toggle_color_cli_root
        return self._toggle_color_cli

    def _orbbec_depth_cli(self):
        if self._toggle_depth_cli.service_is_ready():
            return self._toggle_depth_cli
        if self._toggle_depth_cli_ns.service_is_ready():
            return self._toggle_depth_cli_ns
        if self._toggle_depth_cli_root.service_is_ready():
            return self._toggle_depth_cli_root
        return self._toggle_depth_cli

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
        color_cli = self._orbbec_color_cli()
        if not color_cli.service_is_ready() and not color_cli.wait_for_service(timeout_sec=0.3):
            now = time.time()
            if now - getattr(self, '_toggle_not_ready_log', 0.0) >= 2.0:
                self._toggle_not_ready_log = now
                self._cam_log(
                    f'toggle skipped — {color_cli.srv_name} not ready')
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
            fut = color_cli.call_async(req)
            fut.add_done_callback(lambda f, w=bool(color): _done(f, 'color', w))
        depth_cli = self._orbbec_depth_cli()
        if depth is not None and not depth_same and depth_cli.service_is_ready():
            dreq = SetBool.Request()
            dreq.data = bool(depth)
            dfut = depth_cli.call_async(dreq)
            dfut.add_done_callback(lambda f, w=bool(depth): _done(f, 'depth', w))

    def _ensure_orbbec(self, *, force: bool = False) -> None:
        """Launch Orbbec node on demand (never at deploy unless camera wanted)."""
        if self._cam_forbid_autostart:
            return
        if self._cam_in_nav_launch:
            return
        if not force and not self._cam_wanted():
            return
        if time.time() < self._cam_launch_backoff_until:
            return
        if (
            self.pm.running('camera')
            or self._orbbec_color_cli().service_is_ready()
            or bool(self.pm.stacks_alive().get('camera'))
        ):
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
        try:
            self._toggle_orbbec(color=False, depth=False)
        except Exception:
            pass
        if self.pm.running('camera') or bool(self.pm.stacks_alive().get('camera')):
            self.pm.stop_camera()
            self._cam_log('Orbbec node stopped')
        else:
            self.pm.stop_camera()
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
                    depth=bool(self._cam_need_depth))
            elif self._cam_stream or self._cam_depth_stream:
                # Do not toggle OFF the moment /camera/toggle_* appears —
                # that races Orbbec setupPublishers and kills the color
                # publisher (then Detect+Align crashes the camera).
                self._toggle_orbbec(color=False, depth=False)
            return

        if wanted:
            self._ensure_orbbec()
            if self._cam_stream_allowed():
                self._toggle_orbbec(color=True)

    def _scan_sub_tick(self) -> None:
        """Subscribe to /scan only while lidar stacks (or HW test) need it."""
        stacks = self.pm.stacks_alive()
        want = bool(
            stacks.get('nav')
            or stacks.get('localize')
            or stacks.get('slam')
            or stacks.get('hw_test')
            or self.session.get('phase') == 'mapping'
        )
        if want == self._scan_sub_wanted:
            return
        self._scan_sub_wanted = want
        topic = str(self.get_parameter('scan_topic').value)
        if want and self._scan_sub is None:
            self._scan_sub = self.create_subscription(
                LaserScan, topic, self._on_scan, qos_profile_sensor_data)
            self.get_logger().info(f'Subscribed {topic} (lidar stack up)')
        elif (not want) and self._scan_sub is not None:
            try:
                self.destroy_subscription(self._scan_sub)
            except Exception:
                pass
            self._scan_sub = None
            self._scan = []
            self.get_logger().info(f'Unsubscribed {topic} (no lidar stack)')

    def _on_scan(self, msg: LaserScan) -> None:
        # Always track freshness; avoid heavy point cloud work unless mapping
        # or Hardware Testing needs lidar scalars.
        now = time.time()
        self._scan_stamp = now
        self._scan_msg_stamp = Time.from_msg(msg.header.stamp)
        self._scan_raw_count = len(msg.ranges)
        self._scan_cb_n += 1

        hw_active = bool(self._hw_store.active)
        mapping = self.session.get('phase') == 'mapping'
        # Downsample callbacks: lidar is ~10 Hz; keep health cheap under Nav2.
        if not mapping and not hw_active:
            if self._scan_cb_n % _SCAN_IDLE_EVERY != 0:
                return
            step = max(1, len(msg.ranges) // 60)
            valid = 0
            for i in range(0, len(msg.ranges), step):
                r = float(msg.ranges[i])
                if math.isfinite(r) and msg.range_min < r < msg.range_max:
                    valid += 1
            self._scan_valid_count = valid
            self._scan = []
            return

        pts = []
        angle = msg.angle_min
        step = max(1, len(msg.ranges) // 180)
        valid = 0
        for i in range(0, len(msg.ranges), step):
            r = float(msg.ranges[i])
            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                valid += 1
                if mapping and len(pts) < 32:
                    pts.append({
                        'x': round(r * math.cos(angle), 3),
                        'y': round(r * math.sin(angle), 3),
                    })
            angle += msg.angle_increment * step
        self._scan = pts if mapping else []
        self._scan_valid_count = valid

        if hw_active:
            d = laser_min_range(msg.ranges, msg.range_min, msg.range_max)
            if d is not None:
                for cid in self._hw_topic_ids.get('/scan', []):
                    self._hw_store.set_value(cid, round(d, 3), '/scan')

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
        # Dashboard only draws trail while mapping; skip TF lookups otherwise.
        if self.session.get('phase') != 'mapping':
            return
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
        scan_fresh = (
            (now - self._scan_stamp) < 1.0
            and (self._scan_valid_count >= 8 or len(self._scan) >= 8)
        )
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
        del on

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
            'z': round(float(ps.pose.position.z), 3),
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

    def _nav_path_pts(self) -> list:
        if self._path is None:
            return []
        cached = getattr(self, '_path_pts_cached', None)
        if cached is None:
            self._path_pts_cached = self._path_pts(self._path)
        return list(self._path_pts_cached or [])

    def _clear_live_map(self) -> None:
        """Drop in-memory /map so the UI loads project files (full resolution)."""
        self._map = None
        self._map_built = None
        self._map_built_seq = -1
        self._map_payload_last = object()  # force map_rev bump on next snapshot

    def _map_payload(self) -> Optional[Dict[str, Any]]:
        """Build the UI map payload, reusing the last build when unchanged.

        Serialising the grid is by far the most expensive part of a telemetry
        frame, and the map only changes while SLAM is running.
        Prefer live OccupancyGrid only while a mapping/localize/nav stack is
        actually up — otherwise serve map.pgm so edits and Offline mode are
        not overwritten by a stale downsampled live grid.
        """
        # Only SLAM's live OccupancyGrid is authoritative while mapping.
        # During localize/nav the /map topic is map_server's static map, but we
        # downsample it for SSE — painting against that half-res grid writes the
        # wrong cells into map.pgm. Always serve the on-disk project file then.
        stacks = self.pm.stacks_alive()
        live_ok = (
            self._map is not None
            and (
                stacks.get('slam')
                or self.session.get('phase') == 'mapping'
            )
        )
        if live_ok:
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
        now = time.time()
        if (
            self._snapshot_cache is not None
            and (now - self._snapshot_cache_t) < _SNAPSHOT_CACHE_TTL
        ):
            snap = dict(self._snapshot_cache)
            snap['ts'] = now
            if last_map_rev is not None and last_map_rev == snap.get('map_rev'):
                snap['map'] = None
            return snap

        map_payload = self._map_payload()
        map_rev = self._map_revision(map_payload)

        warnings = list(self.session.get('warnings') or [])
        if self.session.get('phase') == 'mapping':
            warnings = warnings or [
                'Drive slowly while mapping',
                'Keep lidars clear of people and clutter',
                'Loop-close corridors for a better map',
            ]
        stacks = self.pm.stacks_alive()
        nodes = self.pm.nav2_component_status()
        # Enrich lidar / EKF with live rates already tracked — no extra pubs.
        scan_age = (
            round(max(0.0, time.time() - self._scan_stamp), 2)
            if self._scan_stamp else None)
        odom_age = (
            round(max(0.0, time.time() - self._odom_stamp), 2)
            if self._odom_stamp else None)
        lidar = nodes.get('lidar') or {}
        if lidar.get('state') == 'ready' and self._scan_stamp:
            lidar = {
                **lidar,
                'state': 'running' if (scan_age is not None and scan_age < 1.0) else 'stale',
                'detail': (
                    f'merger /scan beams={self._scan_raw_count} '
                    f'valid≈{self._scan_valid_count} age={scan_age}s'
                ),
            }
            nodes['lidar'] = lidar
        elif lidar.get('state') == 'ready':
            nodes['lidar'] = {**lidar, 'detail': 'drivers up · waiting for /scan'}
        ekf = nodes.get('ekf') or {}
        if ekf.get('state') == 'ready' and self._odom_stamp:
            ekf = {
                **ekf,
                'state': 'running' if (odom_age is not None and odom_age < 1.0) else 'stale',
                'detail': (
                    f'/odometry/filtered age={odom_age}s '
                    f'speed={self._odom_speed:.2f}m/s'
                    + (f' frame={self._odom_frame}' if self._odom_frame else '')
                ),
            }
            nodes['ekf'] = ekf
        elif ekf.get('state') == 'ready':
            nodes['ekf'] = {**ekf, 'detail': 'ekf up · waiting for odom'}

        # Do not ship scan points / trail over SSE — UI draws map+path+WPs only.
        phase = self.session.get('phase') or ''
        send_overlay = phase == 'mapping'
        snap = {
            'ts': now,
            'session': dict(self.session),
            'processes': self.pm.list_running(),
            'stacks': stacks,
            'pose': self._live_pose(),
            'amcl': {
                'ok': self._amcl_ok,
                'cov': round(self._amcl_cov, 4),
                'age': round(max(0.0, time.time() - self._amcl_stamp), 2)
                if self._amcl_stamp else None,
            },
            'world_ready': self._map is not None or bool(
                self.session.get('map_name')),
            'map': map_payload,
            'map_rev': map_rev,
            'scan': self._scan if send_overlay else [],
            'scan_count': self._scan_valid_count or len(self._scan),
            'scan_raw_count': self._scan_raw_count,
            'scan_age': scan_age,
            'trail': self._trail[-500:] if send_overlay else [],
            'nav_stack': self.session.get('nav_stack', 'nav2'),
            'path': self._nav_path_pts(),
            'planner_path': [],
            'smoothed_path': [],
            'sim': {
                'path': self._path_pts(self._sim_path),
                'path_smooth': [],
                'result': dict(self._sim_result),
                'stack': self.session.get('nav_stack', 'nav2'),
            },
            'mission': {
                **(self._mission or {}),
                'started_from': self._mission_started_from,
                'sequence': self._armed_seq_name,
                'armed_count': len(self._armed_wps),
                'hw_alert': self._hw_alert,
            },
            'recovery': {},
            'collision': {},
            'collision_avoidance': {
                'type': 'path_ahead_lethal',
                'label': 'Path-ahead footprint (local cost ≥254)',
                'controller': 'FollowPath / PathAheadCollision',
            },
            'goal_status': '',
            'nodes': nodes,
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
            'path_style': 'nav2',
            'warnings': warnings,
            'robot_dims': self._robot_dims(),
            'logs': self._process_logs(),
            'detect_align': self._detect_align_telemetry(),
            'commission': self._commission_snapshot(),
            'oms': self._oms_snapshot_lean(),
            'hw_test': self._hw_store.snapshot(only_live=True) if self._hw_store.active else {
                'ok': True, 'active': False, 'groups': {}, 'live_channels': 0,
            },
            'camera': {
                'on': self._cam_wanted(),
                'ui': self._cam_ui,
                'action': self._cam_action_need,
                'killed': bool(self._cam_forbid_autostart),
                'process': (
                    self.pm.running('camera')
                    or bool(stacks.get('camera'))),
                'stream': bool(self._cam_stream),
                'depth_stream': bool(self._cam_depth_stream),
                'toggle_ready': self._cached_toggle_ready(),
                # Consumers subscribe to these Orbbec topics directly.
                'topics': {
                    'color': '/camera/color/image_raw',
                    'color_compressed': '/camera/color/image_raw/compressed',
                    'depth': '/camera/depth/image_raw',
                },
                'logs': list(self._cam_logs[-40:]),
            },
        }
        self._snapshot_cache = dict(snap)
        self._snapshot_cache_t = now
        out = dict(snap)
        if last_map_rev is not None and last_map_rev == map_rev:
            out['map'] = None
        return out

    def _robot_dims(self) -> Dict[str, float]:
        robot = self.session.get('robot') or 'grip'
        if robot == 'mecanum':
            return {'length': 0.40, 'width': 0.40}
        return {'length': 0.43, 'width': 0.40}

    def _process_logs(self, max_lines: int = 40) -> Dict[str, str]:
        now = time.time()
        if (
            self._logs_cache is not None
            and (now - self._logs_cache_t) < _LOG_CACHE_TTL
            and max_lines <= 40
        ):
            return dict(self._logs_cache)
        out: Dict[str, str] = {}
        log_dir = FsPath(os.path.expanduser('~/skill23_ws/studio_maps/_logs'))
        for name in ('slam', 'nav', 'localize', 'camera', 'detect_align', 'oms'):
            path = log_dir / f'{name}.log'
            try:
                # Read only the tail: nav.log grows to megabytes during a run.
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
        if max_lines <= 40:
            self._logs_cache = out
            self._logs_cache_t = now
        return dict(out)

    def _append_oms_log(self, line: str) -> None:
        path = FsPath(os.path.expanduser('~/skill23_ws/studio_maps/_logs/oms.log'))
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime('%H:%M:%S')
            with open(path, 'a', encoding='utf-8') as handle:
                handle.write(f'{stamp} [studio] {line}\n')
        except OSError:
            pass

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
                    bridge._sse_clients += 1
                    try:
                        last_map_rev: Optional[int] = None
                        while True:
                            snap = bridge.snapshot(last_map_rev=last_map_rev)
                            last_map_rev = snap.get('map_rev')
                            data = json.dumps(snap)
                            self.wfile.write(f'data: {data}\n\n'.encode())
                            self.wfile.flush()
                            # ~0.67 Hz enough for pose/path UI under Nav2 load.
                            time.sleep(_SSE_SLEEP_S)
                    except Exception:
                        return
                    finally:
                        bridge._sse_clients = max(0, bridge._sse_clients - 1)
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
                elif path == '/api/sequences':
                    name = bridge.session.get('map_name') or ''
                    self._json(200, {
                        'sequences': maps.load_sequences(name) if name else {},
                    })
                elif path == '/api/commission/config':
                    name = bridge.session.get('map_name') or ''
                    self._json(200, {
                        'config': maps.load_commission_config(name) if name else default_config(),
                    })
                elif path == '/api/restrictions':
                    name = bridge.session.get('map_name') or ''
                    self._json(200, {
                        'rectangles': maps.load_restrictions(name) if name else [],
                    })
                elif path == '/api/logs':
                    self._json(200, bridge._process_logs(80))
                elif path == '/api/gc_planner/params':
                    self._json(200, bridge._gc_planner_params({}))
                elif path == '/api/gc_planner/walls':
                    self._json(200, bridge._gc_walls_status())
                elif path == '/api/hw_test/live':
                    self._json(200, bridge._hw_store.snapshot(only_live=True))
                elif path == '/api/hw_test/channels':
                    qs = parse_qs(urlparse(self.path).query)
                    robot = (qs.get('robot') or [bridge.session.get('robot') or 'allterrain'])[0]
                    specs = build_channel_specs(str(robot), include_lidar=True)
                    self._json(200, {
                        'ok': True,
                        'robot': robot,
                        'expected': specs,
                        'live': bridge._hw_store.snapshot(only_live=True),
                    })
                elif path == '/api/light_tower/patterns':
                    self._json(200, {'ok': True, 'patterns': list_patterns()})
                elif path in ('/api/oms', '/api/oms/status'):
                    self._json(200, bridge._oms_api_status())
                elif path == '/api/oms/config':
                    self._json(200, bridge._oms_config())
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

    _GC_PLANNER_KEYS = (
        'GridBased.smooth_path',
        'GridBased.smooth_iterations',
        'GridBased.max_deviation',
        'GridBased.straight_line_only',
        'GridBased.round_l_corners',
        'GridBased.l_corner_radius',
        'GridBased.l_corner_arc_points',
        'GridBased.w_distance',
        'GridBased.w_cell_cost',
        'GridBased.w_inflation',
        'GridBased.turn_penalty',
        'GridBased.narrow_wall_penalty',
        'GridBased.edt_narrow_weight',
        'GridBased.allow_unknown',
        'GridBased.allow_narrow',
        'GridBased.use_inscribed_width_gate',
        'GridBased.robot_width',
        'GridBased.inscribed_width_margin',
        'GridBased.shift_goal_off_inscribed',
        'GridBased.use_final_approach_orientation',
    )

    def _gc_planner_params(self, body: Dict[str, Any]) -> Dict[str, Any]:
        """Get or set ClearancePlanner params on /planner_server (live, no restart)."""
        from rcl_interfaces.msg import Parameter as ParamMsg
        from rcl_interfaces.msg import ParameterType, ParameterValue
        from rcl_interfaces.srv import GetParameters, SetParameters

        node_name = 'planner_server'
        updates = body.get('params') if isinstance(body.get('params'), dict) else body
        updates = {
            k: v for k, v in (updates or {}).items()
            if k in self._GC_PLANNER_KEYS or k.startswith('GridBased.')
        }

        if updates:
            cli = self.create_client(SetParameters, f'/{node_name}/set_parameters')
            if not cli.wait_for_service(timeout_sec=1.5):
                raise RuntimeError('planner_server set_parameters not ready (is GC+Planner up?)')
            req = SetParameters.Request()
            for key, raw in updates.items():
                p = ParamMsg()
                p.name = key
                pv = ParameterValue()
                if isinstance(raw, bool):
                    pv.type = ParameterType.PARAMETER_BOOL
                    pv.bool_value = bool(raw)
                elif isinstance(raw, int) and not isinstance(raw, bool):
                    pv.type = ParameterType.PARAMETER_INTEGER
                    pv.integer_value = int(raw)
                else:
                    pv.type = ParameterType.PARAMETER_DOUBLE
                    pv.double_value = float(raw)
                p.value = pv
                req.parameters.append(p)
            fut = cli.call_async(req)
            done = threading.Event()

            def _done(_f):
                done.set()

            fut.add_done_callback(_done)
            if not done.wait(3.0):
                raise RuntimeError('set_parameters timed out')
            res = fut.result()
            if res is None:
                raise RuntimeError('set_parameters failed')
            results = []
            for i, r in enumerate(res.results):
                results.append({
                    'name': req.parameters[i].name if i < len(req.parameters) else '?',
                    'successful': bool(r.successful),
                    'reason': str(r.reason or ''),
                })
            # Fall through to read-back
            _ = results

        gcli = self.create_client(GetParameters, f'/{node_name}/get_parameters')
        if not gcli.wait_for_service(timeout_sec=1.5):
            raise RuntimeError('planner_server get_parameters not ready (is GC+Planner up?)')
        greq = GetParameters.Request()
        greq.names = list(self._GC_PLANNER_KEYS)
        gfut = gcli.call_async(greq)
        gdone = threading.Event()
        gfut.add_done_callback(lambda _f: gdone.set())
        if not gdone.wait(3.0):
            raise RuntimeError('get_parameters timed out')
        gres = gfut.result()
        out: Dict[str, Any] = {}
        if gres is not None:
            for name, val in zip(self._GC_PLANNER_KEYS, gres.values):
                if val.type == ParameterType.PARAMETER_BOOL:
                    out[name] = bool(val.bool_value)
                elif val.type == ParameterType.PARAMETER_INTEGER:
                    out[name] = int(val.integer_value)
                elif val.type == ParameterType.PARAMETER_DOUBLE:
                    out[name] = float(val.double_value)
                elif val.type == ParameterType.PARAMETER_STRING:
                    out[name] = str(val.string_value)
        return {
            'ok': True,
            'params': out,
            'updated': bool(updates),
            'keys': list(self._GC_PLANNER_KEYS),
        }

    def _gc_walls_ensure_pub(self) -> None:
        if self._gc_walls_pub is not None:
            return
        self._gc_walls_pub = self.create_publisher(
            OccupancyGrid, '/gc_planner/walls', self._gc_walls_qos)
        self.get_logger().info('Advertising /gc_planner/walls')

    def _gc_walls_destroy_pub(self) -> None:
        if self._gc_walls_pub is None:
            return
        try:
            self.destroy_publisher(self._gc_walls_pub)
        except Exception:
            pass
        self._gc_walls_pub = None
        self.get_logger().info('Removed /gc_planner/walls publisher')

    def _gc_walls_running(self) -> bool:
        try:
            return bool(self.pm.stacks_alive().get('gc_planner'))
        except Exception:
            return False

    def _gc_walls_refresh_meta(self) -> Dict[str, Any]:
        """Map grid size/origin for walls OccupancyGrid (from project or live /map)."""
        name = str(self.session.get('map_name') or '')
        grid = maps.load_map_grid(name) if name else None
        if grid is None and self._map is not None:
            # Full-resolution /map info only — never the SSE-downsampled grid.
            m = self._map
            grid = {
                'width': int(m.info.width),
                'height': int(m.info.height),
                'resolution': float(m.info.resolution),
                'origin': {
                    'x': float(m.info.origin.position.x),
                    'y': float(m.info.origin.position.y),
                },
            }
        if grid is None:
            raise RuntimeError('no map loaded — open a project before drawing GC walls')
        meta = {
            'width': int(grid['width']),
            'height': int(grid['height']),
            'resolution': float(grid['resolution']),
            'origin': dict(grid['origin']),
        }
        self._gc_walls_meta = meta
        return meta

    def _gc_walls_load_from_disk(self) -> None:
        name = str(self.session.get('map_name') or '')
        self._gc_walls_cells = set()
        if not name:
            return
        for cell in maps.load_gc_walls(name):
            try:
                self._gc_walls_cells.add((int(cell[0]), int(cell[1])))
            except (TypeError, ValueError, IndexError):
                continue

    def _gc_walls_persist(self) -> None:
        name = str(self.session.get('map_name') or '')
        if not name:
            return
        cells = [[gx, gy] for gx, gy in sorted(self._gc_walls_cells)]
        maps.save_gc_walls(name, cells)

    def _gc_walls_build_msg(self) -> OccupancyGrid:
        meta = self._gc_walls_meta or self._gc_walls_refresh_meta()
        w = int(meta['width'])
        h = int(meta['height'])
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = float(meta['resolution'])
        msg.info.width = w
        msg.info.height = h
        ox = float(meta['origin'].get('x', 0.0))
        oy = float(meta['origin'].get('y', 0.0))
        msg.info.origin.position.x = ox
        msg.info.origin.position.y = oy
        msg.info.origin.orientation.w = 1.0
        # Unknown (-1) ignored by StaticLayer use_maximum; walls = lethal 100.
        data = [-1] * (w * h)
        for gx, gy in self._gc_walls_cells:
            if 0 <= gx < w and 0 <= gy < h:
                data[gy * w + gx] = 100
        msg.data = data
        return msg

    def _gc_walls_publish(self, clear_costmap: bool = True) -> Dict[str, Any]:
        """Publish overlay and optionally clear global costmap so inflation refreshes."""
        # Only advertise the topic while GC+Planner is (or was just) running.
        if self._gc_walls_pub is None and not self._gc_walls_running():
            return {
                'ok': True,
                'cells': len(self._gc_walls_cells),
                'published': False,
                'costmap_cleared': False,
            }
        self._gc_walls_ensure_pub()
        msg = self._gc_walls_build_msg()
        self._gc_walls_pub.publish(msg)
        cleared = False
        if clear_costmap:
            # use_maximum layers need a clear so erased cells leave the master grid.
            cleared = self._call_clear_costmap(self._clear_global_cli)
        return {
            'ok': True,
            'cells': len(self._gc_walls_cells),
            'width': int(msg.info.width),
            'height': int(msg.info.height),
            'published': True,
            'costmap_cleared': cleared,
        }

    def _gc_walls_edit(self, ops: List[Any]) -> Dict[str, Any]:
        self._gc_walls_refresh_meta()
        meta = self._gc_walls_meta or {}
        w = int(meta.get('width') or 0)
        h = int(meta.get('height') or 0)
        for op in ops or []:
            if not isinstance(op, dict):
                continue
            action = str(op.get('action') or '')
            for cell in op.get('cells') or []:
                try:
                    gx, gy = int(cell[0]), int(cell[1])
                except (TypeError, ValueError, IndexError):
                    continue
                if not (0 <= gx < w and 0 <= gy < h):
                    continue
                if action == 'wall':
                    self._gc_walls_cells.add((gx, gy))
                elif action == 'erase':
                    self._gc_walls_cells.discard((gx, gy))
        self._gc_walls_persist()
        out = self._gc_walls_publish(clear_costmap=True)
        out['ops'] = len(ops or [])
        return out

    def _gc_walls_clear(self) -> Dict[str, Any]:
        self._gc_walls_cells = set()
        try:
            self._gc_walls_refresh_meta()
        except RuntimeError:
            pass
        self._gc_walls_persist()
        return self._gc_walls_publish(clear_costmap=True)

    def _gc_walls_status(self) -> Dict[str, Any]:
        meta = self._gc_walls_meta
        try:
            if meta is None:
                meta = self._gc_walls_refresh_meta()
        except RuntimeError as exc:
            return {'ok': False, 'error': str(exc), 'cells': 0, 'walls': []}
        return {
            'ok': True,
            'cells': len(self._gc_walls_cells),
            'walls': [[gx, gy] for gx, gy in sorted(self._gc_walls_cells)],
            'width': int(meta['width']),
            'height': int(meta['height']),
            'resolution': float(meta['resolution']),
            'origin': dict(meta['origin']),
            'topic_active': self._gc_walls_pub is not None,
        }

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
                self._cam_in_nav_launch = False
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
            self._clear_live_map()
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
            self._clear_live_map()
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
            # Prefer fresh map.pgm over any leftover live OccupancyGrid.
            self._clear_live_map()
            return {'ok': True}

        if path == '/api/map/restrictions':
            name = self.session.get('map_name') or ''
            rects = list(body.get('rectangles') or [])
            maps.save_restrictions(name, rects)
            if body.get('burn'):
                maps.burn_restrictions_into_map(name)
            self._clear_live_map()
            return {'ok': True}

        if path == '/api/waypoints':
            name = self.session.get('map_name') or ''
            wps = list(body.get('waypoints') or [])
            maps.save_waypoints(name, wps)
            return {'ok': True, 'count': len(wps), 'path': str(maps.project_dir(name) / 'waypoints.json')}

        if path == '/api/sequences':
            name = self.session.get('map_name') or ''
            seqs = body.get('sequences') if isinstance(body.get('sequences'), dict) else body
            path_out = maps.save_sequences(name, seqs or {})
            return {
                'ok': True,
                'count': len(maps.load_sequences(name)),
                'path': str(path_out),
            }

        if path == '/api/pi_save':
            """Save waypoints + sequences to studio_maps/<map>/ for mission_runner."""
            name = self.session.get('map_name') or ''
            if not name:
                raise ValueError('no map loaded')
            wps = list(body.get('waypoints') or [])
            seqs = body.get('sequences') if isinstance(body.get('sequences'), dict) else {}
            # Default wait 3s on WPs that have no action (mission_runner friendly)
            for i, wp in enumerate(wps):
                if not isinstance(wp, dict):
                    continue
                if not wp.get('action'):
                    wp['action'] = {'type': 'wait', 'seconds': 3.0}
                if not wp.get('name'):
                    wp['name'] = f'wp{i + 1}'
            wp_path = maps.save_waypoints(name, wps)
            seq_path = maps.save_sequences(name, seqs)
            return {
                'ok': True,
                'map': name,
                'waypoints': len(wps),
                'sequences': len(maps.load_sequences(name)),
                'waypoints_path': str(wp_path),
                'sequences_path': str(seq_path),
                'hint': (
                    f'Copied on Pi under studio_maps/{name}/. '
                    f'For skill23_mission: copy into src/skill23_mission/maps/<name>/ '
                    f'or run mission with map:={name} (uses studio_maps).'
                ),
            }

        if path == '/api/commission/config':
            name = self.session.get('map_name') or ''
            if not name:
                raise ValueError('no map loaded')
            cfg = normalize_config(body.get('config') or body)
            maps.save_commission_config(name, cfg)
            return {'ok': True, 'config': cfg}

        if path == '/api/deploy/start':
            name = self.session.get('map_name') or ''
            robot = self.session.get('robot') or 'grip'
            start_camera = bool(body.get('start_camera', False))
            module = str(body.get('module') or '').strip().lower()
            if module in ('commission', 'commissioning', 'detect'):
                start_camera = True
                with_align = True  # yaw / align_move for pick modes
                with_cobra = False
                # Tick meaning for OMS: enabled for Home OMS / WP — never launch with stack.
                oms_enabled = True
                module = 'commission'
            elif module in ('nav', 'navigation', 'localisation', 'localization'):
                # Honor dashboard camera tick (streams off until lookup WP).
                start_camera = bool(body.get('start_camera', False))
                # Dashboard checkboxes (default off).
                with_align = bool(body.get('with_align')) if 'with_align' in body else False
                with_cobra = bool(body.get('with_cobra')) if 'with_cobra' in body else False
                oms_enabled = bool(body.get('with_oms')) if 'with_oms' in body else False
                module = 'nav'
            else:
                module = 'commission' if start_camera else 'nav'
                with_align = (
                    bool(body.get('with_align'))
                    if 'with_align' in body else False)
                with_cobra = (
                    bool(body.get('with_cobra'))
                    if 'with_cobra' in body else False)
                oms_enabled = (
                    bool(body.get('with_oms'))
                    if 'with_oms' in body else bool(start_camera))
            if module == 'commission' or start_camera:
                with_cobra = False
                if module == 'commission':
                    oms_enabled = True
                    with_align = True
            if 'start_detect_align' in body:
                with_detect_align = bool(body.get('start_detect_align'))
            else:
                # Commission defaults Detect+Align on; nav camera tick does not.
                with_detect_align = bool(start_camera) and module == 'commission'
            with_detect_align = with_detect_align and start_camera
            with_lookup = bool(start_camera) and module == 'nav'
            if start_camera and module != 'commission':
                with_cobra = False
            if start_camera and module == 'commission':
                with_cobra = False
                oms_enabled = True
                with_align = True
            # Never launch OMS inside Nav2/commission — Home OMS / WP lazy-starts it.
            with_oms_launch = False
            stack = 'nav2'
            yaml_path = maps.map_yaml_path(name)
            if not yaml_path.is_file():
                raise FileNotFoundError(str(yaml_path))
            # Nav module: Align/Cobra YAML, no camera. Commissioning: camera + optional Detect+Align.
            self._cam_deploy_requested = False
            if not start_camera:
                self._cam_ui = False
                self._stop_orbbec()
            else:
                self.pm.stop_camera()
            # Optionally burn restrictions before nav
            if body.get('burn_restrictions', True):
                maps.burn_restrictions_into_map(name)
            mp = self.pm.start_nav(
                robot, str(yaml_path),
                with_camera=start_camera, with_align=with_align,
                with_cobra=with_cobra, with_oms=with_oms_launch,
                with_detect_align=with_detect_align,
                with_lookup=with_lookup, stack=stack)
            self.session['phase'] = 'deploy'
            self.session['mode'] = 'arena'
            self.session['nav_stack'] = stack
            self.session['deploy_module'] = module
            self.session['oms_enabled'] = bool(oms_enabled)
            self._clear_nav_paths()
            self._sim_path = None
            self._sim_result = {}
            self._cam_forbid_autostart = False
            self._cam_in_nav_launch = bool(start_camera)
            if start_camera:
                da_msg = (
                    'Detect+Align idle until waypoint'
                    if with_detect_align else
                    'Detect+Align node NOT started (checkbox off)')
                self._cam_log(
                    f'Commissioning — Gemini E in launch (streams OFF); {da_msg}')
            self.get_logger().info(
                f'Deploy START module={module} stack={stack} robot={robot} '
                f'camera={start_camera} align={with_align} cobra={with_cobra} '
                f'oms_enabled={oms_enabled} oms_launch={with_oms_launch} '
                f'detect_align={with_detect_align} map={yaml_path} '
                f'pid={mp.popen.pid} log={mp.log_path} cmd={" ".join(mp.cmd)}')
            return {
                'ok': True,
                'stack': stack,
                'module': module,
                'map': str(yaml_path),
                'camera': start_camera,
                'with_align': with_align,
                'with_cobra': with_cobra,
                'with_oms': bool(oms_enabled),
                'oms_lazy': True,
                'oms_launched': False,
                'with_detect_align': with_detect_align,
                'camera_after_nav_startup': False,
                'pid': mp.popen.pid,
                'log': mp.log_path,
                'cmd': ' '.join(mp.cmd),
            }

        if path == '/api/deploy/stop':
            self.get_logger().info('Deploy STOP — shutting down nav stack')
            self._cam_deploy_requested = False
            self._cam_in_nav_launch = False
            self._cancel_nav2()
            self.pm.stop_nav()
            self._stop_orbbec()
            self._enable_ctrl(False)
            self.session['phase'] = 'idle'
            self.session['nav_stack'] = 'nav2'
            self.session['deploy_module'] = 'nav'
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
                'x': round(x, 3), 'y': round(y, 3), 'z': 0.0, 'yaw': round(yaw, 3),
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
                self.get_logger().info(
                    f'Nav2 armed {len(self._armed_wps)} waypoint(s) '
                    f'seq={self._armed_seq_name or "(unnamed)"} — '
                    'Start mission / physical START to run')
            else:
                ps = self._pose_stamped(
                    float(body['x']), float(body['y']), float(body.get('yaw', 0)))
                self._send_nav2_pose(ps)
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
            return {'ok': True, 'style': 'nav2'}

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
            planner_id = str(body.get('planner_id') or 'GridBased').strip()
            if planner_id not in ('GridBased',):
                planner_id = 'GridBased'
            self._sim_req_id += 1
            req_id = self._sim_req_id
            self._sim_path = None
            stack_name = (
                'gc_planner'
                if str(self.session.get('nav_stack') or '') == 'gc_planner'
                else 'nav2')
            if not self._using_nav2():
                self._sim_result = {
                    'id': req_id,
                    'ok': False,
                    'detail': 'Start Nav2 or GC+Planner before simulating a path',
                    'points': 0,
                    'stack': stack_name,
                    'planner_id': planner_id,
                }
                return {'ok': False, 'id': req_id, 'error': self._sim_result['detail']}
            self._sim_result = {
                'id': req_id, 'ok': None, 'detail': f'planning with {planner_id}…',
                'stack': stack_name,
                'planner_id': planner_id,
            }
            out = self._nav2_compute_path_sync(
                sx, sy, float(body.get('syaw', 0.0)), gx, gy, gyaw,
                planner_id=planner_id)
            if out.get('ok'):
                self._sim_path = out['path']
                self._sim_result = {
                    'id': req_id,
                    'ok': True,
                    'detail': out['detail'],
                    'points': out['points'],
                    'points_smooth': 0,
                    'stack': stack_name,
                    'planner_id': out.get('planner_id', planner_id),
                }
            else:
                self._sim_result = {
                    'id': req_id,
                    'ok': False,
                    'detail': out.get('detail', 'plan failed'),
                    'points': 0,
                    'stack': stack_name,
                    'planner_id': planner_id,
                }
            return {'ok': True, 'id': req_id}

        if path == '/api/sim/clear':
            self._sim_path = None
            self._sim_result = {}
            return {'ok': True}

        if path == '/api/gc_planner/start':
            name = str(body.get('map_name') or self.session.get('map_name') or '')
            robot = str(body.get('robot') or self.session.get('robot') or 'grip')
            yaml_path = maps.map_yaml_path(name)
            if not yaml_path.is_file():
                raise FileNotFoundError(str(yaml_path))
            mp = self.pm.start_gc_planner(robot, str(yaml_path))
            self.session['nav_stack'] = 'gc_planner'
            self.session['phase'] = self.session.get('phase') or 'deploy'
            self._sim_path = None
            self._sim_result = {}
            # Advertise walls topic only while GC is up; load saved overlay.
            self._gc_walls_ensure_pub()
            try:
                self._gc_walls_load_from_disk()
                self._gc_walls_refresh_meta()
            except Exception as exc:
                self.get_logger().warn(f'GC walls meta: {exc}')
            # Defer publish slightly so StaticLayer can subscribe (latched QoS).
            def _pub_walls_later():
                time.sleep(1.5)
                try:
                    if self._gc_walls_pub is None:
                        return
                    self._gc_walls_publish(clear_costmap=True)
                except Exception as exc:
                    self.get_logger().warn(f'GC walls publish: {exc}')

            threading.Thread(target=_pub_walls_later, daemon=True).start()
            return {
                'ok': True,
                'message': (
                    'GC+Planner started (map + walls layer + inflation + planner). '
                    'Draw Wall/Erase in the GC dashboard — /global_costmap updates live. '
                    'RViz: /map, /gc_planner/walls, /global_costmap/costmap.'
                ),
                'map': name,
                'robot': robot,
                'pid': mp.popen.pid,
                'walls': len(self._gc_walls_cells),
                'stacks': self.pm.stacks_alive(),
            }

        if path == '/api/gc_planner/stop':
            self.pm.stop_gc_planner()
            if str(self.session.get('nav_stack') or '') == 'gc_planner':
                self.session['nav_stack'] = ''
            self._gc_walls_destroy_pub()
            return {
                'ok': True,
                'message': 'GC+Planner stopped',
                'stacks': self.pm.stacks_alive(),
            }

        if path == '/api/gc_planner/params':
            return self._gc_planner_params(body or {})

        if path == '/api/gc_planner/walls':
            return self._gc_walls_status()

        if path == '/api/gc_planner/walls/edit':
            return self._gc_walls_edit(list(body.get('ops') or []))

        if path == '/api/gc_planner/walls/clear':
            return self._gc_walls_clear()

        if path == '/api/light_tower/patterns':
            return {'ok': True, 'patterns': list_patterns()}

        if path == '/api/trail/clear':
            self._trail = []
            self._trail_last = (None, None)
            self._path = None
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
            if self.pm.stacks_alive().get('localize'):
                return {
                    'ok': True,
                    'already': True,
                    'message': 'localize already running',
                    'map': str(yaml_path),
                }
            self.session.update({
                'map_name': name,
                'robot': robot,
                'mode': 'arena',
                'phase': 'waypoints',
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
                'already': False,
                'map': str(yaml_path),
                'pid': mp.popen.pid,
                'log': mp.log_path,
                'cmd': ' '.join(mp.cmd),
            }

        if path == '/api/localize/stop':
            if not self.pm.stacks_alive().get('localize'):
                return {
                    'ok': True,
                    'already': True,
                    'message': 'localize already stopped',
                }
            self.get_logger().info('Localize STOP')
            self.pm.stop_localize()
            self._pose = None
            self._amcl_ok = False
            self._scan = []
            self._clear_live_map()
            return {'ok': True, 'already': False}

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
            self._toggle_orbbec(color=False, depth=False)
            return {'ok': True, 'on': False, 'stream': self._cam_stream}

        if path == '/api/camera/kill':
            # Fully stop Orbbec process (not just toggle stream off).
            self._cam_ui = False
            self._cam_action_need = False
            self._cam_need_depth = False
            self._cam_need_last = None
            self._cam_deploy_requested = False
            self._cam_in_nav_launch = False
            self._cam_forbid_autostart = True
            self._cam_log('UI Kill camera node — stopping Orbbec process')
            self._stop_orbbec()
            return {
                'ok': True,
                'on': False,
                'process': self.pm.running('camera'),
                'stacks': self.pm.stacks_alive(),
            }

        if path == '/api/oms/start':
            robot = str(body.get('robot') or self.session.get('robot') or 'allterrain')
            if robot not in ('allterrain', 'grip', 'mecanum', 'xdrive'):
                raise ValueError('invalid robot')
            self.session['robot'] = robot
            snap = self._oms_snapshot()
            if snap.get('ready'):
                message = (
                    'OMS already running — attached '
                    f'({snap.get("source")}). Press Run is not needed.')
                self._append_oms_log(message)
                return {
                    'ok': True,
                    'already': True,
                    'ready': True,
                    'source': snap.get('source'),
                    'message': message,
                    'oms': snap,
                }
            mp = self.pm.start_oms(robot)
            log_path = None if mp is None else mp.log_path
            cmd = ' '.join(mp.cmd) if mp is not None else ''
            message = (
                'OMS launching (Titan + nodes). Idle until you press Home. '
                f'Logs → {log_path or "studio_maps/_logs/oms.log"} '
                '(shown on OMS Software dashboard).')
            self._append_oms_log(message)
            self.get_logger().info(
                f'OMS start robot={robot} pid='
                f'{None if mp is None else mp.popen.pid} '
                f'log={log_path} cmd={cmd}')
            return {
                'ok': True,
                'already': False,
                'ready': False,
                'source': 'standalone',
                'message': message,
                'pid': None if mp is None else mp.popen.pid,
                'log': log_path,
                'oms': self._oms_snapshot(),
            }

        if path == '/api/oms/stop':
            was = bool(
                self.pm.oms_standalone_alive() or self.pm.oms_nodes_alive())
            stop_error = ''
            try:
                self.pm.stop_oms()
            except RuntimeError as exc:
                stop_error = str(exc)
                self.get_logger().error(f'OMS stop: {stop_error}')
                # Still force-clear bridge OMS state so UI is not stuck Ready.
            self._oms['stamp'] = 0.0
            self._oms['ready'] = False
            self._oms['source'] = 'none'
            self._oms['standalone'] = False
            for key in (
                    'rotation', 'lift', 'linear', 'vertical',
                    'rotation_servo', 'gripper', 'home'):
                slot = self._oms.get(key)
                if isinstance(slot, dict):
                    slot.clear()
                    slot['state'] = 'stopped'
            still = bool(self.pm.oms_nodes_alive())
            if stop_error and still:
                message = stop_error
                self._append_oms_log(message)
                snap = self._oms_snapshot()
                snap['ready'] = False
                snap['source'] = 'none'
                snap['standalone'] = False
                return {
                    'ok': False,
                    'error': message,
                    'message': message,
                    'oms': snap,
                }
            if was and not still:
                message = 'OMS stopped. Titan + OMS nodes were killed.'
            elif was and still:
                message = (
                    'OMS stop requested but some processes are still alive. '
                    'Try Stop OMS again.')
            else:
                message = 'Stop requested — no standalone OMS was running.'
            self._append_oms_log(message)
            self.get_logger().info(message)
            snap = self._oms_snapshot()
            snap['ready'] = False
            snap['source'] = 'none'
            snap['standalone'] = False
            return {
                'ok': not still,
                'message': message,
                'oms': snap,
            }

        if path == '/api/oms/cmd':
            return self._oms_send_cmd(body)

        if path == '/api/hw_test/start':
            robot = str(body.get('robot') or self.session.get('robot') or 'allterrain')
            if robot not in ('allterrain', 'grip', 'mecanum', 'xdrive'):
                raise ValueError('invalid robot')
            enable_lidar = bool(body.get('enable_lidar', True))
            self.session['robot'] = robot
            if self.pm.stacks_alive().get('hw_test'):
                self._hw_test_start_subs(robot)
                return {
                    'ok': True,
                    'already': True,
                    'message': 'HW test already running — live channels attached.',
                    'robot': robot,
                    'stacks': self.pm.stacks_alive(),
                    'channels': self._hw_store.snapshot(only_live=False),
                }
            mp = self.pm.start_hw_test(robot, enable_lidar=enable_lidar)
            # Delay subs so Titan has time to advertise.
            def _later():
                time.sleep(2.0)
                try:
                    self._hw_test_start_subs(robot)
                except Exception as exc:
                    self.get_logger().warn(f'HW test subs: {exc}')

            threading.Thread(target=_later, daemon=True).start()
            return {
                'ok': True,
                'message': (
                    'HW test starting (control_server + lidars). '
                    f'Log → {mp.log_path}. Only publishing topics appear live.'
                ),
                'robot': robot,
                'pid': mp.popen.pid,
                'log': str(mp.log_path) if mp.log_path else '',
                'stacks': self.pm.stacks_alive(),
            }

        if path == '/api/hw_test/stop':
            self.pm.stop_hw_test()
            self._hw_test_stop_subs()
            return {
                'ok': True,
                'message': 'HW test stopped.',
                'stacks': self.pm.stacks_alive(),
            }

        if path == '/api/hw_test/channels':
            robot = str(body.get('robot') or self.session.get('robot') or 'allterrain')
            specs = build_channel_specs(robot, include_lidar=True)
            return {
                'ok': True,
                'robot': robot,
                'channels': specs,
                'live': self._hw_store.snapshot(only_live=True),
            }

        if path == '/api/oms/home':
            return self._oms_send_cmd({'axis': 'home', 'mode': 'home'})

        if path == '/api/oms/config':
            from oms.config_io import save_oms_yaml
            updates = body.get('updates') or body
            if not isinstance(updates, dict):
                raise ValueError('updates object required')
            result = save_oms_yaml(updates)
            self._oms_viz_cache = None
            return result if result.get('ok') else {'ok': False, **result}

        raise ValueError(f'unknown endpoint {path}')


def main(args=None) -> None:
    from skill23_studio.ros_dds import apply_studio_dds_env
    apply_studio_dds_env()
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
