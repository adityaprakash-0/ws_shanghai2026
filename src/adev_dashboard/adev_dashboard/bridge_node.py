#!/usr/bin/env python3
"""ADEV Dashboard — lightweight HTTP/SSE bridge (no rosbridge)."""

import base64
import json
import math
import os
import queue
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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import BatteryState, CompressedImage, LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

try:
    import yaml
except ImportError:
    yaml = None

try:
    from skill23_vision.srv import FindColor, SetGate
    _VISION_SRVS = True
except ImportError:
    FindColor = None  # type: ignore
    SetGate = None  # type: ignore
    _VISION_SRVS = False


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

NAV_MODE_LABELS = {
    'nav2': 'Nav2',
    'nav2_secret_mode': 'Secret Mode',
}

PATH_DEBUG_TOPICS = (
    '/plan',
    '/local_plan',
    '/route_path',
)


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
            'nav_mode': 'nav2',
            'simulation': False,
            'pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            'battery': {'voltage': 0.0, 'percentage': 0.0, 'valid': False, 'source': 'none'},
            'estop_hw': False,
            'estop_sw': False,
            'nav2': {
                'available': False,
                'state': 'unknown',
                'action': '',
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
                'task_name': '',
            },
            'vision': {
                'available': False,
                'active': False,
                'stream_enabled': None,
                'last_result': None,
                'colors': [],
            },
            'scan': [],
            'global_path': [],
            'local_path': [],
            'route_path': [],
            'path_info': {
                'plan_poses': 0,
                'local_poses': 0,
                'route_poses': 0,
                'last_topic': '',
            },
            'debug_log': [],
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


class DebugLog:
    """Ring buffer of recent nav debug lines — streamed to dashboard."""

    def __init__(self, telemetry: TelemetryStore, max_lines: int = 40):
        self._telemetry = telemetry
        self._max_lines = max_lines
        self._lock = threading.Lock()

    def add(self, msg: str):
        line = f'{time.strftime("%H:%M:%S")} {msg}'
        with self._lock:
            lines = list(self._telemetry._data.get('debug_log', []))
            lines.append(line)
            lines = lines[-self._max_lines:]
            self._telemetry.update(debug_log=lines)


def normalize_mission_steps(raw_steps):
    """Normalize task-builder / waypoint payloads into typed steps."""
    steps = []
    for item in raw_steps or []:
        if not isinstance(item, dict):
            continue
        stype = str(item.get('type', 'navigate')).strip().lower()
        if stype in ('camera', 'look', 'look_for_color'):
            stype = 'look_color'
        if stype == 'navigate' or ('x' in item and 'y' in item and stype not in (
                'look_color', 'wait')):
            steps.append({
                'type': 'navigate',
                'x': float(item['x']),
                'y': float(item['y']),
                'yaw': float(item.get('yaw', 0.0)),
                'ignore_yaw': bool(item.get('ignore_yaw', False)),
                'label': str(item.get('label', '')),
            })
        elif stype == 'look_color':
            colors = item.get('colors') or item.get('color') or []
            if isinstance(colors, str):
                colors = [c.strip() for c in colors.split(',') if c.strip()]
            else:
                colors = [str(c).strip().lower() for c in colors if str(c).strip()]
            steps.append({
                'type': 'look_color',
                'colors': colors,
                'timeout_sec': float(item.get('timeout_sec', 5.0)),
                'retry_count': max(
                    0, int(item.get('retry_count', item.get('retries', 0)))),
                'on_found': str(item.get('on_found', 'continue')),
                'on_miss': str(item.get('on_miss', 'continue')),
                'label': str(item.get('label', '')),
            })
        elif stype == 'wait':
            steps.append({
                'type': 'wait',
                'seconds': float(item.get('seconds', item.get('dwell_sec', 2.0))),
                'label': str(item.get('label', '')),
            })
    return steps


class MissionRunner:
    """Typed mission steps — navigate / look_color / wait. Works in all nav modes."""

    XY_TOL = 0.10
    YAW_TOL = 0.18
    SIM_SPEED = 0.14
    SIM_ROT_SPEED = 0.55
    WAYPOINT_DWELL_SEC = 4.0

    def __init__(self, bridge: 'AdevBridgeNode', telemetry: TelemetryStore):
        self.bridge = bridge
        self.node = bridge
        self.telemetry = telemetry
        self._work_queue = queue.Queue()
        self.steps = []
        self.waypoints = []  # navigate poses only (map display / legacy)
        self.task_name = ''
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
        self._topic_goal_mode = False
        self._dwell_advance = True  # False when wait-step owns the dwell timer
        self._vision_retry_attempt = 0

    def enqueue(self, fn):
        self._work_queue.put(fn)

    def process_work(self):
        while True:
            try:
                fn = self._work_queue.get_nowait()
            except queue.Empty:
                break
            fn()

    def check_nav2_server(self):
        if self.bridge.simulation_enabled:
            plan_ready = self.bridge.plan_is_ready()
            self.bridge.dbg(
                f'[nav-check] sim mode planner ready={plan_ready}')
            self.telemetry.update(nav2={
                'available': plan_ready,
                'state': 'planning' if plan_ready else 'planner_unavailable',
                'action': 'compute_path_to_pose' if plan_ready else '',
            })
            return plan_ready

        nav_ready = self.bridge.nav_is_ready()
        if nav_ready != self.bridge._last_nav_ready:
            self.bridge.dbg(
                f'[nav-check] navigate_to_pose ready={nav_ready} '
                f'mission_active={self.active}')
            if self.bridge._last_nav_ready and not nav_ready:
                self.bridge.dbg(
                    '[nav] ALERT: navigate_to_pose server LOST — '
                    'wait or relaunch stack before mission')
        self.bridge._last_nav_ready = nav_ready
        if self.active and (self._goal_handle is not None or self._topic_goal_mode):
            self.telemetry.update(nav2={
                'available': True,
                'state': 'navigating',
                'action': (
                    'navigate_to_pose' if self._goal_handle is not None
                    else 'goal_pose'),
            })
            return True

        if self.active and not nav_ready:
            self.telemetry.update(nav2={
                'available': False,
                'state': 'navigating',
                'action': 'navigate_to_pose',
            })
            return False

        if nav_ready:
            nav2_update = {
                'available': True,
                'action': 'navigate_to_pose',
                'state': 'navigating' if self.active else 'ready',
            }
            self.telemetry.update(nav2=nav2_update)
            return True

        nav2_update = {'available': False, 'action': ''}
        if not self.active:
            nav2_update['state'] = 'unavailable'
        self.telemetry.update(nav2=nav2_update)
        return False

    def set_waypoints(self, waypoints):
        """Legacy API — plain waypoints become navigate-only steps."""
        for wp in waypoints or []:
            if isinstance(wp, dict):
                wp.setdefault('ignore_yaw', False)
        self.set_task({
            'name': '',
            'steps': [{'type': 'navigate', **wp} for wp in (waypoints or [])],
        })

    def set_task(self, task):
        """Typed task from Task Builder: {name, steps:[{type,...}, ...]}."""
        if isinstance(task, list):
            task = {'name': '', 'steps': task}
        steps = normalize_mission_steps(task.get('steps', []))
        self.steps = steps
        self.waypoints = [s for s in steps if s.get('type') == 'navigate']
        self.task_name = str(task.get('name', '') or '')
        self.index = 0
        self.retries = 0
        self.telemetry.update(mission={
            'total': len(steps),
            'index': 0,
            'status': 'armed',
            'current': steps[0] if steps else None,
            'task_name': self.task_name,
            'mode': 'sim_path' if self.bridge.simulation_enabled else self.bridge.nav_mode,
        })
        self.bridge.dbg(
            f'[mission] armed task="{self.task_name}" steps={len(steps)} '
            f'(nav={len(self.waypoints)})')

    def start(self):
        if not self.steps:
            return False, 'No mission steps defined'
        self._cancel_timers()
        self.active = True
        self.index = 0
        self.retries = 0
        self._ignore_yaw_cancel = False
        self.bridge._smoother_seen = False
        self.telemetry.update(mission={
            'active': True,
            'status': 'running',
            'mode': 'sim_path' if self.bridge.simulation_enabled else self.bridge.nav_mode,
            'dwell_remaining': 0.0,
            'dwell_total': 0.0,
            'task_name': self.task_name,
            'total': len(self.steps),
        })
        self.enqueue(self._dispatch)
        self.bridge.dbg(
            f'[mission] START {len(self.steps)} steps mode={self.bridge.nav_mode} '
            f'task="{self.task_name}"')
        self.telemetry.update(
            global_path=[], local_path=[], route_path=[],
            path_info={'plan_poses': 0, 'local_poses': 0, 'route_poses': 0, 'last_topic': ''},
            vision={'last_result': None})
        mode = 'sim path-follow' if self.bridge.simulation_enabled else 'Nav2'
        return True, f'Mission started ({mode}, {len(self.steps)} steps)'

    def stop(self):
        self.enqueue(self._stop_impl)

    def _stop_impl(self):
        self.active = False
        self._topic_goal_mode = False
        self._cancel_timers()
        if self._goal_handle is not None:
            handle = self._goal_handle
            self._goal_handle = None
            self.bridge.dbg('[nav] canceling active goal (mission stop)')
            handle.cancel_goal_async()
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
        if self.index >= len(self.steps):
            self.active = False
            self.telemetry.update(mission={
                'active': False,
                'status': 'complete',
                'index': self.index,
            })
            self.node.get_logger().info('ADEV mission complete')
            return

        step = self.steps[self.index]
        stype = step.get('type', 'navigate')
        self.telemetry.update(mission={
            'index': self.index,
            'total': len(self.steps),
            'current': step,
            'retries': self.retries,
        })

        if stype == 'look_color':
            self._vision_retry_attempt = 0
            self._dispatch_look_color(step)
            return
        if stype == 'wait':
            self._dispatch_wait(step)
            return

        # navigate (default)
        wp = step
        cx, cy, cyaw = self._current_pose()
        self.bridge.dbg(
            f'[mission] dispatch step {self.index + 1}/{len(self.steps)} navigate '
            f'goal=({wp["x"]:.3f},{wp["y"]:.3f}) '
            f'robot=({cx:.3f},{cy:.3f},{math.degrees(cyaw):.0f}°) '
            f'mode={self.bridge.nav_mode}')
        self.bridge._log_tf_debug()
        self.node.get_logger().info(
            f'Step {self.index + 1}/{len(self.steps)} navigate: '
            f'({wp["x"]:.3f}, {wp["y"]:.3f}) '
            f'yaw={math.degrees(wp.get("yaw", 0)):.0f}° '
            f'ignore_yaw={bool(wp.get("ignore_yaw", False))}')
        self.telemetry.update(mission={'status': 'navigating'})

        if self.bridge.simulation_enabled:
            self._dispatch_sim_plan(wp)
        else:
            self._dispatch_nav2(wp)

    def _dispatch_look_color(self, step):
        colors = step.get('colors') or []
        timeout = float(step.get('timeout_sec', 5.0))
        retry_count = max(0, int(step.get('retry_count', 0)))
        attempt = self._vision_retry_attempt + 1
        total_attempts = retry_count + 1
        self.bridge.dbg(
            f'[vision] look_color colors={colors} timeout={timeout:.1f}s '
            f'attempt={attempt}/{total_attempts}')
        self.telemetry.update(mission={
            'status': 'looking',
            'retries': self._vision_retry_attempt,
        })
        self.node.get_logger().info(
            f'Step {self.index + 1}/{len(self.steps)} look_color: {colors} '
            f'(attempt {attempt}/{total_attempts})')

        def worker():
            result = self.bridge.vision_find_color(colors, timeout)
            self.enqueue(lambda: self._on_look_color_done(step, result))

        threading.Thread(target=worker, daemon=True, name='adev_look_color').start()

    def _on_look_color_done(self, step, result):
        if not self.active:
            return
        found = bool(result.get('found'))
        color = str(result.get('color_name', '') or '')
        msg = str(result.get('message', '') or '')
        self.telemetry.update(vision={
            'last_result': {
                'found': found,
                'color_name': color,
                'confidence': float(result.get('confidence', 0.0) or 0.0),
                'message': msg,
                'step': self.index + 1,
            },
        })
        self.bridge.dbg(f'[vision] result found={found} color={color} — {msg}')
        self.node.get_logger().info(f'Look color: {msg}')

        retry_count = max(0, int(step.get('retry_count', 0)))
        if not found and self._vision_retry_attempt < retry_count:
            self._vision_retry_attempt += 1
            self.telemetry.update(mission={
                'status': 'retrying_camera',
                'retries': self._vision_retry_attempt,
                'last_error': msg,
            })
            self.bridge.dbg(
                f'[vision] no match — retry '
                f'{self._vision_retry_attempt}/{retry_count}')
            self._dispatch_look_color(step)
            return

        action = str(step.get('on_found' if found else 'on_miss', 'continue')).lower()
        if action == 'stop':
            self.active = False
            self._vision_retry_attempt = 0
            self.telemetry.update(mission={
                'active': False,
                'status': 'stopped' if not found else 'found_stop',
                'last_error': '' if found else msg,
            })
            return

        self.retries = 0
        self._vision_retry_attempt = 0
        self.index += 1
        self._dispatch()

    def _dispatch_wait(self, step):
        seconds = max(0.0, float(step.get('seconds', 2.0)))
        self.bridge.dbg(f'[mission] wait {seconds:.1f}s')
        self._dwell_advance = True
        self._start_dwell(seconds, label='waiting')

    def _dispatch_sim_plan(self, wp):
        if not self.bridge.plan_is_ready():
            self.bridge.dbg('[nav] compute_path_to_pose NOT ready — retry 2s')
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

        self.bridge.dbg(
            f'[nav] sim ComputePathToPose '
            f'({cx:.2f},{cy:.2f})->({wp["x"]:.2f},{wp["y"]:.2f})')
        self.telemetry.update(nav2={'available': True, 'state': 'planning'})
        self.bridge.send_plan_goal(goal, self._on_plan_goal_sent_bridge)

    def _on_plan_goal_sent_bridge(self, handle, reason='ok'):
        if not self.active:
            return
        if handle is None or reason != 'accepted':
            self.bridge.dbg(f'[nav] ComputePathToPose failed: {reason}')
            self.node.get_logger().warn(
                f'ComputePathToPose failed ({reason}) — retry')
            self._schedule_retry()
            return
        self.bridge.dbg('[nav] ComputePathToPose goal accepted')
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda fut: self.enqueue(lambda: self._on_plan_result(fut)))

    def _on_plan_result(self, future):
        if not self.active:
            return
        wp = self.steps[self.index]
        try:
            result = future.result().result
            pts = [
                (p.pose.position.x, p.pose.position.y)
                for p in result.path.poses
            ]
            self.bridge.dbg(
                f'[path] sim plan result: {len(pts)} poses')
        except Exception as exc:
            self.bridge.dbg(f'[path] sim plan exception: {exc}')
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
        """Navigate goal reached — dwell only before another navigate step."""
        if self._dwell_timer is not None:
            return

        self.node.get_logger().info(f'Step {self.index + 1} navigate reached')
        self._follow_wp = None
        self._rotate_to_yaw = None
        self._path_pts = []
        self._goal_handle = None
        self._topic_goal_mode = False
        self.retries = 0
        self.telemetry.update(mission={'last_error': ''})

        next_idx = self.index + 1
        if next_idx >= len(self.steps):
            self.index = next_idx
            self.telemetry.update(mission={'dwell_remaining': 0.0, 'dwell_total': 0.0})
            self._dispatch()
            return

        nxt = self.steps[next_idx]
        # Camera / wait right after arrival — no inter-waypoint pause
        if nxt.get('type') in ('look_color', 'wait'):
            self.index = next_idx
            self.telemetry.update(mission={'dwell_remaining': 0.0, 'dwell_total': 0.0})
            self._dispatch()
            return

        self._dwell_advance = True
        self._start_dwell(self.WAYPOINT_DWELL_SEC, label='dwelling')

    def _start_dwell(self, seconds: float, label: str = 'dwelling'):
        self._cancel_nav_timers()
        self._dwell_remaining = float(seconds)
        self.node.get_logger().info(
            f'{label.capitalize()} {seconds:.0f}s '
            f'(step {self.index + 1}/{len(self.steps)})')
        self.telemetry.update(mission={
            'status': label,
            'dwell_remaining': round(self._dwell_remaining, 1),
            'dwell_total': seconds,
        })
        self._dwell_timer = self.node.create_timer(0.25, self._dwell_tick)

    def _dwell_tick(self):
        if not self.active:
            return
        self._dwell_remaining = max(0.0, self._dwell_remaining - 0.25)
        status = self.telemetry.snapshot().get('mission', {}).get('status', 'dwelling')
        self.telemetry.update(mission={
            'status': status if status in ('dwelling', 'waiting') else 'dwelling',
            'dwell_remaining': round(self._dwell_remaining, 1),
        })
        if self._dwell_remaining <= 0.0:
            if self._dwell_timer is not None:
                self._dwell_timer.cancel()
                self._dwell_timer = None
            self.index += 1
            self.telemetry.update(mission={'dwell_remaining': 0.0, 'dwell_total': 0.0})
            self.node.get_logger().info(
                f'Dwell complete — advancing to step {self.index + 1}/{len(self.steps)}')
            self._dispatch()

    def _waypoint_reached(self):
        self._on_waypoint_success()

    def _make_goal_pose(self, wp):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x = float(wp['x'])
        msg.pose.position.y = float(wp['y'])
        _, _, qz, qw = yaw_to_quat(float(wp.get('yaw', 0.0)))
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def _dispatch_nav2_goal_topic(self, wp):
        """Publish goal_pose — bt_navigator re-sends via navigate_to_pose internally."""
        self._topic_goal_mode = True
        self._goal_handle = None
        msg = self._make_goal_pose(wp)
        self.bridge.publish_nav_goal(msg)
        self.bridge.dbg(
            '[nav] published /goal_pose + /bt_navigator/goal_pose '
            f'({wp["x"]:.3f},{wp["y"]:.3f}) — waiting for /plan or /local_plan')
        self.node.get_logger().info(
            'Navigate via /goal_pose topic (watch for /plan in logs)')
        self.telemetry.update(nav2={
            'available': True,
            'state': 'navigating',
            'action': 'goal_pose',
        })
        if self._poll_timer is None:
            self._poll_timer = self.node.create_timer(0.25, self._poll_topic_goal)

    def _poll_topic_goal(self):
        if not self.active or not self._topic_goal_mode:
            return
        wp = self.steps[self.index]
        cx, cy, cyaw = self._current_pose()
        dist = math.hypot(float(wp['x']) - cx, float(wp['y']) - cy)
        if dist > self.XY_TOL:
            return
        if wp.get('ignore_yaw', False):
            self._topic_goal_mode = False
            self.bridge.dbg(f'[nav] topic goal XY reached dist={dist:.3f}')
            self._on_waypoint_success()
            return
        target_yaw = float(wp.get('yaw', 0.0))
        if abs(normalize_angle(target_yaw - cyaw)) < self.YAW_TOL:
            self._topic_goal_mode = False
            self.bridge.dbg('[nav] topic goal pose reached')
            self._on_waypoint_success()

    def _dispatch_nav2(self, wp):
        if not self.bridge.nav_wait_ready(timeout_sec=15.0):
            self.bridge.dbg(
                '[nav] navigate_to_pose unavailable (15s) — retry in 3s')
            self.bridge._log_action_discovery()
            self.telemetry.update(mission={
                'status': 'retrying',
                'last_error': 'navigate_to_pose unavailable — is Nav2 still active?',
            })
            self._pending_timer = self.node.create_timer(3.0, self._timer_dispatch)
            return

        self._topic_goal_mode = False
        goal = NavigateToPose.Goal()
        goal.pose = self._make_goal_pose(wp)
        cx, cy, cyaw = self._current_pose()
        self.bridge.dbg(
            f'[nav] send NavigateToPose action '
            f'robot=({cx:.2f},{cy:.2f}) goal=({wp["x"]:.2f},{wp["y"]:.2f}) '
            f'nav_mode={self.bridge.nav_mode}')
        # #region agent log
        try:
            import json as _json
            import time as _time
            route_ready = False
            path_ready = False
            try:
                from rclpy.action import ActionClient as _AC
                from nav2_msgs.action import ComputeRoute as _CR, ComputePathToPose as _CP
                rc = getattr(self.bridge, '_dbg_route_client', None)
                if rc is None:
                    self.bridge._dbg_route_client = _AC(self.bridge, _CR, 'compute_route')
                    self.bridge._dbg_path_client = _AC(self.bridge, _CP, 'compute_path_to_pose')
                route_ready = self.bridge._dbg_route_client.wait_for_server(timeout_sec=0.2)
                path_ready = self.bridge._dbg_path_client.wait_for_server(timeout_sec=0.2)
            except Exception as _exc:
                route_ready = f'err:{_exc}'
                path_ready = route_ready
            with open('/home/vmx/skill23_ws/.cursor/debug-8cd67e.log', 'a', encoding='utf-8') as _f:
                _f.write(_json.dumps({
                    'sessionId': '8cd67e',
                    'runId': 'post-fix',
                    'hypothesisId': 'A',
                    'location': 'bridge_node.py:_dispatch_nav2',
                    'message': 'mission dispatch action readiness',
                    'data': {
                        'nav_mode': self.bridge.nav_mode,
                        'compute_route_ready': route_ready,
                        'compute_path_to_pose_ready': path_ready,
                        'goal': {'x': wp['x'], 'y': wp['y']},
                        'robot': {'x': cx, 'y': cy, 'yaw_deg': math.degrees(cyaw)},
                    },
                    'timestamp': int(_time.time() * 1000),
                }) + '\n')
        except Exception:
            pass
        # #endregion
        self.node.get_logger().info('Sending NavigateToPose via navigate_to_pose')
        self.telemetry.update(nav2={
            'available': True,
            'state': 'navigating',
            'action': 'navigate_to_pose',
        })
        self.bridge.send_nav_goal(goal, self._on_nav_goal_sent_bridge)
        if wp.get('ignore_yaw', False) and self._poll_timer is None:
            self._poll_timer = self.node.create_timer(0.25, self._poll_xy_goal)

    def _on_nav_goal_sent_bridge(self, handle, reason='ok'):
        if not self.active:
            return
        if handle is None or reason != 'accepted':
            self.bridge.dbg(
                f'[nav] NavigateToPose send failed: {reason} — trying goal_pose')
            self.node.get_logger().warn(
                f'NavigateToPose failed ({reason}) — falling back to /goal_pose')
            wp = self.steps[self.index]
            self._dispatch_nav2_goal_topic(wp)
            return
        self.bridge.dbg('[nav] NavigateToPose goal ACCEPTED — waiting for path + result')
        self._goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda fut: self.enqueue(lambda: self._on_result(fut)))

    def _poll_xy_goal(self):
        if not self.active or self._goal_handle is None:
            return
        wp = self.steps[self.index]
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
            self.bridge.dbg('[nav] canceling goal (ignore_yaw XY reached)')
            handle.cancel_goal_async()
            self._on_waypoint_success()

    def _timer_dispatch(self):
        self._cancel_nav_timers()
        self._dispatch()

    def _on_result(self, future):
        if not self.active:
            return
        self._goal_handle = None
        status = future.result().status
        status_name = GOAL_STATUS_NAMES.get(status, str(status))
        self.bridge.dbg(f'[nav] goal result: {status_name} (code {status})')
        # #region agent log
        try:
            import json as _json
            import time as _time
            with open('/home/vmx/skill23_ws/.cursor/debug-8cd67e.log', 'a', encoding='utf-8') as _f:
                _f.write(_json.dumps({
                    'sessionId': '8cd67e',
                    'runId': 'post-fix',
                    'hypothesisId': 'A',
                    'location': 'bridge_node.py:_on_result',
                    'message': 'nav goal result',
                    'data': {
                        'status': status_name,
                        'code': status,
                        'nav_mode': self.bridge.nav_mode,
                    },
                    'timestamp': int(_time.time() * 1000),
                }) + '\n')
        except Exception:
            pass
        # #endregion
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
            self.bridge.dbg(f'[nav] goal FAILED: {err}')
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
    self.declare_parameter('nav_mode', 'nav2')
    self.declare_parameter('http_port', 8080)
    self.declare_parameter('maps_dir', os.path.expanduser('~/skill23_ws'))

    self.robot = self.get_parameter('robot').value
    self.nav_mode = self.get_parameter('nav_mode').value
    self.http_port = int(self.get_parameter('http_port').value)
    self.maps_dir = self.get_parameter('maps_dir').value

    self.telemetry = TelemetryStore()
    self.telemetry.update(
        robot=self.robot,
        nav_mode=self.nav_mode,
    )
    self.debug_log = DebugLog(self.telemetry)
    self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
    self.plan_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
    self.mission = MissionRunner(self, self.telemetry)
    self.estop_active = False
    self.simulation_enabled = False
    self.pose_initialized = True  # AMCL set_initial_pose in nav2 yaml
    self._battery_rx_count = 0
    self._cmd_vel_nav_rx = 0
    self._cmd_vel_rx = 0
    self._smoother_seen = False
    self._last_nav_ready = None
    self._cached_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    self._sim_pending = None
    self._sim_apply_timer = None
    self._vision_preview_jpeg = b''
    self._vision_preview_lock = threading.Lock()
    self._vision_cb_group = ReentrantCallbackGroup()
    dims = ROBOT_DIMS.get(self.robot, ROBOT_DIMS['allterrain'])
    self.telemetry.update(robot_dims=dims)

    self._init_vision_clients()

    self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    self.estop_pub = self.create_publisher(Bool, '/adev/estop', 10)
    self.sim_pub = self.create_publisher(Bool, '/adev/simulation', 10)
    self.sim_seed_pub = self.create_publisher(
        PoseWithCovarianceStamped, '/adev/sim_seed', 10)
    self.initial_pose_pub = self.create_publisher(
        PoseWithCovarianceStamped, '/initialpose', 10)
    self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
    self.bt_goal_pose_pub = self.create_publisher(
        PoseStamped, '/bt_navigator/goal_pose', 10)
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
    self.create_subscription(Path, '/route_path', self._route_path_cb, path_qos)
    self.create_subscription(
        PoseWithCovarianceStamped, '/amcl_pose', self._amcl_pose_cb, 10)
    self.create_subscription(PoseStamped, '/adev/sim_pose', self._sim_pose_cb, 10)
    self.create_subscription(Twist, '/cmd_vel_nav', self._cmd_vel_nav_cb, 10)
    self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_out_cb, 10)
    preview_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    self.create_subscription(
        CompressedImage, '/adev/vision/preview/compressed',
        self._vision_preview_cb, preview_qos)
    self.create_subscription(
        String, '/adev/vision/status', self._vision_status_cb, 10)

    self.tf_buffer = Buffer(
        cache_time=rclpy.duration.Duration(seconds=30.0))
    self.tf_listener = TransformListener(self.tf_buffer, self)
    self.create_timer(0.2, self._update_pose)
    self.create_timer(5.0, self._check_battery)
    self.create_timer(0.05, lambda: self.mission.process_work())
    self.create_timer(2.0, lambda: self.mission.check_nav2_server())
    self.create_timer(5.0, self._log_cmd_vel_health)

    self._http_ready = threading.Event()
    self._http_thread = threading.Thread(target=self._run_http, daemon=True)
    self._http_thread.start()
    if self._http_ready.wait(timeout=5.0):
        self.dbg(
            f'bridge ready port={self.http_port} robot={self.robot} '
            f'nav_mode={self.nav_mode}')
        if self.nav_mode == 'nav2_secret_mode':
            self.dbg(
                '[config] nav2_secret_mode uses GraphPlanner + RPP')
        self.get_logger().info(
            f'ADEV API bridge on :{self.http_port} (API only — UI runs on laptop) | '
            f'robot={self.robot} mode={self.nav_mode}')
    else:
        self.get_logger().error(
            f'HTTP bridge failed to start on port {self.http_port}. '
            'Kill stale process: pkill -f adev_bridge  then relaunch.')

  def publish_nav_goal(self, msg: PoseStamped):
    self.goal_pose_pub.publish(msg)
    self.bt_goal_pose_pub.publish(msg)

  def dbg(self, msg: str):
    self.get_logger().info(msg)
    self.debug_log.add(msg)

  def _init_vision_clients(self):
    self._find_color_cli = None
    self._set_gate_cli = None
    if not _VISION_SRVS:
      self.telemetry.update(vision={
        'available': False,
        'active': False,
        'colors': [],
        'last_result': None,
      })
      self.get_logger().info(
        'skill23_vision not installed — camera task blocks unavailable '
        '(navigation still works)')
      return
    self._find_color_cli = self.create_client(
      FindColor, '/adev/vision/find_color',
      callback_group=self._vision_cb_group)
    self._set_gate_cli = self.create_client(
      SetGate, '/adev/vision/set_gate',
      callback_group=self._vision_cb_group)
    self.telemetry.update(vision={
      'available': False,
      'active': False,
      'stream_enabled': None,
      'colors': ['red', 'orange', 'yellow', 'green', 'blue', 'purple', 'white', 'black'],
      'last_result': None,
    })

  def vision_is_ready(self) -> bool:
    return bool(
      self._find_color_cli is not None
      and self._find_color_cli.service_is_ready())

  def vision_find_color(self, colors, timeout_sec: float = 5.0) -> dict:
    """Blocking call for mission worker thread. Safe if vision stack absent."""
    if self._find_color_cli is None:
      return {
        'found': False,
        'color_name': '',
        'confidence': 0.0,
        'message': 'skill23_vision not built/sourced — enable_vision launch arg?',
      }
    if not self._find_color_cli.wait_for_service(timeout_sec=2.0):
      return {
        'found': False,
        'color_name': '',
        'confidence': 0.0,
        'message': 'Vision gate not running (launch with enable_vision:=true)',
      }
    req = FindColor.Request()
    req.colors = [str(c) for c in colors]
    req.timeout_sec = float(timeout_sec)
    future = self._find_color_cli.call_async(req)
    deadline = time.time() + float(timeout_sec) + 3.0
    while rclpy.ok() and not future.done() and time.time() < deadline:
      time.sleep(0.05)
    if not future.done():
      return {
        'found': False,
        'color_name': '',
        'confidence': 0.0,
        'message': 'find_color service timeout',
      }
    try:
      res = future.result()
      return {
        'found': bool(res.found),
        'color_name': str(res.color_name or ''),
        'confidence': float(res.confidence),
        'u': int(res.u),
        'v': int(res.v),
        'message': str(res.message or ''),
      }
    except Exception as exc:
      return {
        'found': False,
        'color_name': '',
        'confidence': 0.0,
        'message': f'find_color error: {exc}',
      }

  def vision_set_gate(self, active: bool) -> dict:
    if self._set_gate_cli is None:
      return {'success': False, 'message': 'vision unavailable'}
    if not self._set_gate_cli.wait_for_service(timeout_sec=1.0):
      return {'success': False, 'message': 'set_gate service not ready'}
    req = SetGate.Request()
    req.data = bool(active)
    future = self._set_gate_cli.call_async(req)
    deadline = time.time() + 3.0
    while rclpy.ok() and not future.done() and time.time() < deadline:
      time.sleep(0.05)
    if not future.done():
      return {'success': False, 'message': 'set_gate timeout'}
    try:
      res = future.result()
      return {'success': bool(res.success), 'message': str(res.message or '')}
    except Exception as exc:
      return {'success': False, 'message': str(exc)}

  def _vision_preview_cb(self, msg: CompressedImage):
    with self._vision_preview_lock:
      self._vision_preview_jpeg = bytes(msg.data)

  def _vision_status_cb(self, msg: String):
    try:
      data = json.loads(msg.data)
    except Exception:
      return
    self.telemetry.update(vision={
      'available': True,
      'active': bool(data.get('active', False)),
      'stream_enabled': data.get('stream_enabled'),
      'colors': data.get('colors') or self.telemetry.snapshot().get('vision', {}).get('colors', []),
    })

  def get_vision_preview_jpeg(self) -> bytes:
    with self._vision_preview_lock:
      return self._vision_preview_jpeg

  def nav_is_ready(self) -> bool:
    return self.nav_client.server_is_ready()

  def nav_wait_ready(self, timeout_sec: float = 15.0) -> bool:
    if self.nav_client.server_is_ready():
      return True
    return self.nav_client.wait_for_server(timeout_sec=timeout_sec)

  def plan_is_ready(self) -> bool:
    return self.plan_client.server_is_ready()

  def _lookup_tf_map_base(self, timeout_sec: float = 0.3):
    """Latest map→base_footprint; prefer clock-now then zero-time."""
    timeout = rclpy.duration.Duration(seconds=timeout_sec)
    errors = []
    for stamp in (self.get_clock().now(), rclpy.time.Time()):
      try:
        return self.tf_buffer.lookup_transform(
            'map', 'base_footprint', stamp, timeout=timeout)
      except Exception as exc:
        errors.append(str(exc))
    raise RuntimeError(errors[-1] if errors else 'tf lookup failed')

  def _log_action_discovery(self):
    ready = self.nav_client.server_is_ready()
    waited = self.nav_client.wait_for_server(timeout_sec=0.5)
    self.dbg(
        f'[nav] navigate_to_pose server_is_ready={ready} '
        f'wait_for_server(0.5s)={waited}')

  def _log_tf_debug(self):
    try:
      tf = self._lookup_tf_map_base(timeout_sec=0.3)
      t = tf.transform.translation
      yaw = quat_to_yaw(tf.transform.rotation.z, tf.transform.rotation.w)
      self.dbg(
          f'[tf] map->base_footprint OK ({t.x:.3f},{t.y:.3f},{math.degrees(yaw):.0f}°)')
    except Exception as exc:
      self.dbg(f'[tf] map->base_footprint FAILED: {exc}')

  def send_nav_goal(self, goal, on_sent_cb):
    if not self.nav_wait_ready(timeout_sec=15.0):
      self.dbg('[nav] wait_for_server(navigate_to_pose) TIMEOUT 15s')
      on_sent_cb(None, 'wait_timeout')
      return
    self.dbg('[nav] wait_for_server OK — send_goal_async')
    future = self.nav_client.send_goal_async(goal)
    future.add_done_callback(
        lambda f: self.mission.enqueue(
            lambda: self._on_nav_goal_future(f, on_sent_cb)))

  def _on_nav_goal_future(self, future, on_sent_cb):
    try:
      handle = future.result()
    except Exception as exc:
      self.dbg(f'[nav] send_goal_async exception: {exc}')
      on_sent_cb(None, f'exception:{exc}')
      return
    if handle is None:
      self.dbg('[nav] send_goal_async returned None')
      on_sent_cb(None, 'null_handle')
      return
    if not handle.accepted:
      self.dbg('[nav] bt_navigator REJECTED goal')
      on_sent_cb(handle, 'rejected')
      return
    on_sent_cb(handle, 'accepted')

  def send_plan_goal(self, goal, on_sent_cb):
    if not self.plan_client.wait_for_server(timeout_sec=5.0):
      self.dbg('[nav] wait_for_server(compute_path_to_pose) TIMEOUT 5s')
      on_sent_cb(None, 'wait_timeout')
      return
    future = self.plan_client.send_goal_async(goal)
    future.add_done_callback(
        lambda f: self.mission.enqueue(
            lambda: self._on_plan_goal_future(f, on_sent_cb)))

  def _on_plan_goal_future(self, future, on_sent_cb):
    try:
      handle = future.result()
    except Exception as exc:
      on_sent_cb(None, f'exception:{exc}')
      return
    if handle is None or not handle.accepted:
      on_sent_cb(handle, 'rejected')
      return
    on_sent_cb(handle, 'accepted')

  def _update_path_telemetry(self, topic: str, msg: Path, key: str):
    pts = path_to_points(msg)
    n = len(msg.poses)

    if key == 'global_path' and n >= 1:
      cx, cy = self._cached_pose['x'], self._cached_pose['y']
      a = msg.poses[0].pose.position
      if math.hypot(a.x - cx, a.y - cy) > 2.5:
        return

    info = dict(self.telemetry.snapshot().get('path_info', {}))
    info['last_topic'] = topic
    if key == 'global_path':
      info['plan_poses'] = n
    elif key == 'local_path':
      info['local_poses'] = n
    elif key == 'route_path':
      info['route_poses'] = n
    update = {key: pts, 'path_info': info}
    if key == 'route_path' and pts:
      update['global_path'] = pts
    self.telemetry.update(**update)

  def _cmd_vel_nav_cb(self, msg: Twist):
    self._cmd_vel_nav_rx += 1
    if self.mission.active and not self._smoother_seen:
      self.cmd_vel_pub.publish(msg)

  def _cmd_vel_out_cb(self, _msg: Twist):
    self._cmd_vel_rx += 1
    self._smoother_seen = True

  def _log_cmd_vel_health(self):
    if not self.mission.active:
      self._cmd_vel_nav_rx = 0
      self._cmd_vel_rx = 0
      return
    nav = self._cmd_vel_nav_rx
    out = self._cmd_vel_rx
    self._cmd_vel_nav_rx = 0
    self._cmd_vel_rx = 0
    if nav > 0 and out == 0:
      self.dbg(
          f'[cmd_vel] WARNING: /cmd_vel_nav={nav}/5s but /cmd_vel=0 '
          '(velocity_smoother likely inactive — need cmd_vel_relay)')
    elif nav > 0 or out > 0:
      self.dbg(f'[cmd_vel] /cmd_vel_nav={nav}/5s /cmd_vel={out}/5s')

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

  def _local_plan_cb(self, msg: Path):
    self._update_path_telemetry('/local_plan', msg, 'local_path')

  def _route_path_cb(self, msg: Path):
    self._update_path_telemetry('/route_path', msg, 'route_path')

  def _global_plan_cb(self, msg: Path):
    self._update_path_telemetry('/plan', msg, 'global_path')

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
        tf = self._lookup_tf_map_base(timeout_sec=0.2)
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
        tf = self._lookup_tf_map_base(timeout_sec=0.1)
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
                mode = node.nav_mode
                snap = store.snapshot()
                self._json_response(200, {
                    'ok': True,
                    'robot': node.robot,
                    'nav_mode': mode,
                    'nav_mode_label': NAV_MODE_LABELS.get(mode, mode),
                    'vision_available': bool(
                        snap.get('vision', {}).get('available')
                        or node.vision_is_ready()),
                })
                return

            if path == '/api/telemetry':
                self._json_response(200, store.snapshot())
                return

            if path == '/api/vision/preview':
                jpeg = node.get_vision_preview_jpeg()
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
                self._json_response(200, {'count': len(wps), 'steps': len(node.mission.steps)})
                return

            if path == '/api/mission/task':
                # Task Builder payload: {name, steps:[...]} or {steps:[...]}
                task = body.get('task', body)
                node.mission.set_task(task)
                self._json_response(200, {
                    'ok': True,
                    'name': node.mission.task_name,
                    'steps': len(node.mission.steps),
                    'waypoints': len(node.mission.waypoints),
                })
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

            if path == '/api/vision/gate':
                active = bool(body.get('active', False))

                def _gate_worker():
                    return node.vision_set_gate(active)

                # Run briefly on HTTP thread — service call sleeps, not on executor
                result = _gate_worker()
                self._json_response(200 if result.get('success') else 503, result)
                return

            if path == '/api/vision/find':
                colors = body.get('colors') or []
                timeout = float(body.get('timeout_sec', 5.0))
                result = node.vision_find_color(colors, timeout)
                self._json_response(200, result)
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
