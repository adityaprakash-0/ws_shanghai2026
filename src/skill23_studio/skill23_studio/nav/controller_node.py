#!/usr/bin/env python3
"""Closed-loop AMCL path executor for skill23_custom_nav.

Pure motion execution: follow planner path with /amcl_pose. Goal XY/yaw
completion is owned by cn_goal_checker (/custom_nav/goal_status).
"""

from __future__ import annotations

import json
import math
from enum import Enum
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from skill23_studio.nav.node_status import NodeStatusPub
from skill23_studio.nav.utils import pose_cov_to_stamped, wrap_angle, yaw_from_quat


class CtrlState(Enum):
    IDLE = 'idle'
    ALIGN = 'align'
    FOLLOW = 'follow'
    FINAL_YAW = 'final_yaw'
    BLOCKED = 'blocked'
    DONE = 'done'


Point2 = Tuple[float, float]


class ControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('cn_controller')

        self.declare_parameter('linear_vel', 0.35)
        self.declare_parameter('angular_vel', 0.55)
        self.declare_parameter('fine_angular_vel', 0.20)
        # Slowest in-place turn the drive still responds to; the taper never
        # goes below this or the robot stalls short of the heading.
        self.declare_parameter('min_angular_vel', 0.16)
        # While driving, the wheels are already turning, so a much smaller
        # differential steers. Using the in-place floor here is what made small
        # heading errors answer with a full-size correction and overshoot.
        self.declare_parameter('min_steer_vel', 0.05)
        # Final yaw endgame: below yaw_slow_err use yaw_slow_vel so one pose
        # period cannot turn further than the tolerance.
        self.declare_parameter('yaw_slow_err', 0.35)
        self.declare_parameter('yaw_slow_vel', 0.10)
        self.declare_parameter('final_yaw_tol', 0.05)
        # Rotate on 1 tick then rest this many ticks in the slow band. Peaks
        # stay high enough to break stiction while the average rate drops.
        self.declare_parameter('yaw_pulse_off_ticks', 1)
        self.declare_parameter('lookahead_dist', 0.30)
        self.declare_parameter('heading_align_threshold', 0.35)
        self.declare_parameter('heading_aligned_threshold', 0.12)
        self.declare_parameter('heading_deadband', 0.05)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('angular_sign', 1.0)
        # AMCL publishes ~scan rate. Under Pi load it can briefly go quieter
        # than 0.5 s — treat that as soft, not an emergency stop.
        self.declare_parameter('pose_timeout_sec', 1.0)
        self.declare_parameter('pose_warn_interval_sec', 2.0)
        self.declare_parameter('nearest_search_window', 20)
        self.declare_parameter('nearest_reseed_dist', 0.75)
        self.declare_parameter('pose_jump_dist', 0.40)
        self.declare_parameter('cmd_log_hz', 4.0)
        self.declare_parameter('disabled_brake_sec', 0.6)
        # Planning a path is the heaviest moment in the stack. Holding still
        # briefly afterwards lets AMCL land a scan update on a stationary
        # robot before motion starts, instead of diverging under peak load.
        self.declare_parameter('path_settle_sec', 0.8)

        self.linear_vel = float(self.get_parameter('linear_vel').value)
        self.angular_vel = float(self.get_parameter('angular_vel').value)
        self.fine_angular_vel = float(self.get_parameter('fine_angular_vel').value)
        self.min_angular_vel = float(self.get_parameter('min_angular_vel').value)
        self.min_steer_vel = float(self.get_parameter('min_steer_vel').value)
        self.yaw_slow_err = float(self.get_parameter('yaw_slow_err').value)
        self.yaw_slow_vel = float(self.get_parameter('yaw_slow_vel').value)
        self.final_yaw_tol = float(self.get_parameter('final_yaw_tol').value)
        self.yaw_pulse_off = max(
            0, int(self.get_parameter('yaw_pulse_off_ticks').value))
        self.lookahead_dist = float(self.get_parameter('lookahead_dist').value)
        self.heading_align = float(self.get_parameter('heading_align_threshold').value)
        self.heading_aligned = float(self.get_parameter('heading_aligned_threshold').value)
        self.heading_deadband = float(self.get_parameter('heading_deadband').value)
        sign = float(self.get_parameter('angular_sign').value)
        self.angular_sign = 1.0 if sign >= 0.0 else -1.0
        self.pose_timeout = float(self.get_parameter('pose_timeout_sec').value)
        self.pose_warn_interval = float(self.get_parameter('pose_warn_interval_sec').value)
        self.nearest_window = max(1, int(self.get_parameter('nearest_search_window').value))
        self.nearest_reseed_dist = float(self.get_parameter('nearest_reseed_dist').value)
        self.pose_jump_dist = float(self.get_parameter('pose_jump_dist').value)
        self.cmd_log_period = 1.0 / max(0.5, float(self.get_parameter('cmd_log_hz').value))
        self.brake_hold = max(0.0, float(self.get_parameter('disabled_brake_sec').value))
        self.path_settle = max(0.0, float(self.get_parameter('path_settle_sec').value))

        self.enabled = False
        self.blocked = False
        self.goal_status = 'NOT_REACHED'
        self.pose: Optional[PoseStamped] = None
        self.goal_yaw: Optional[float] = None
        self.path_pts: List[Point2] = []
        self.path_yaws: List[float] = []
        self._yaw_tick = 0
        self.nearest_idx = 0  # forward-only path progress cursor
        self.state = CtrlState.IDLE
        self._rotate_dir = 0.0
        self._last_xy: Optional[Point2] = None
        self._force_reseed = False

        self._last_pose_time: Optional[float] = None
        self._pose_count = 0
        self._pose_rate_window_start = self.get_clock().now().nanoseconds * 1e-9
        self._pose_rate_count = 0
        self._measured_pose_hz = 0.0
        self._last_pose_warn_time = 0.0
        self._last_cmd_log_time = 0.0
        self._last_logged_state = ''
        self._dbg_look: Optional[Point2] = None
        self._dbg_desired_yaw = 0.0
        self._dbg_heading_err = 0.0
        self._dbg_yaw = 0.0
        self._dbg_action = ''

        self.status = NodeStatusPub(self, 'controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.recovery_state = 'idle'
        self.recovery_active = False
        # While disabled the controller brakes for this long and then goes
        # silent, so teleop and the relocalize spin own /cmd_vel instead of
        # fighting a stream of zero twists at the control rate.
        self._brake_until = 0.0
        self._settle_until = 0.0

        self.create_subscription(
            Bool, '/custom_nav/controller/enable', self._on_enable,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ))
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, 10)
        self.create_subscription(Path, '/custom_nav/path', self._on_path, 10)
        # Final yaw has to be measured against the same pose cn_goal_checker
        # judges, otherwise the controller settles on the path's last tangent
        # while the checker calls the goal reached at a different heading.
        self.create_subscription(
            PoseStamped, '/custom_nav/current_goal', self._on_current_goal, 10)
        self.create_subscription(String, '/custom_nav/collision', self._on_collision, 10)
        self.create_subscription(String, '/custom_nav/goal_status', self._on_goal, 10)
        self.create_subscription(String, '/custom_nav/recovery', self._on_recovery, 10)

        rate = max(1.0, float(self.get_parameter('control_rate_hz').value))
        self.create_timer(1.0 / rate, self._tick)
        self.status.set('idle', 'waiting for enable + path')
        self.get_logger().info(
            'path executor only — goal XY/yaw owned by cn_goal_checker; '
            f'AMCL timeout={self.pose_timeout:.2f}s angular_sign={self.angular_sign:+.0f} '
            f'cmd_log={1.0 / self.cmd_log_period:.1f} Hz')

    def _on_enable(self, msg: Bool) -> None:
        was_enabled = self.enabled
        self.enabled = bool(msg.data)
        if not self.enabled:
            self.blocked = False
            self._brake_until = (
                self.get_clock().now().nanoseconds * 1e-9 + self.brake_hold)
            self._stop('disabled')
            if self.state != CtrlState.IDLE:
                self._set_state(CtrlState.IDLE, 'disabled')
        elif not was_enabled:
            # A fresh enable means a new goal is being pursued; drop any
            # verdict left over from the previous waypoint.
            self.goal_status = 'NOT_REACHED'

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        ps = pose_cov_to_stamped(msg)
        if abs(ps.pose.position.x) > 50.0 or abs(ps.pose.position.y) > 50.0:
            self._warn_pose(
                f'rejected out-of-bounds AMCL pose '
                f'({ps.pose.position.x:.1f}, {ps.pose.position.y:.1f})')
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        self.pose = ps
        self._last_pose_time = now
        self._pose_count += 1
        self._pose_rate_count += 1
        elapsed = now - self._pose_rate_window_start
        if elapsed >= 5.0:
            self._measured_pose_hz = self._pose_rate_count / max(elapsed, 1e-6)
            self._pose_rate_count = 0
            self._pose_rate_window_start = now
            self.get_logger().info(
                f'/amcl_pose rate ≈ {self._measured_pose_hz:.1f} Hz '
                f'(total={self._pose_count})')

    def _warn_pose(self, detail: str) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if (now - self._last_pose_warn_time) < self.pose_warn_interval:
            return
        self._last_pose_warn_time = now
        self.get_logger().warn(detail)

    def _pose_age(self) -> Optional[float]:
        if self._last_pose_time is None:
            return None
        return self.get_clock().now().nanoseconds * 1e-9 - self._last_pose_time

    def _pose_ok(self) -> bool:
        """True if we have a recent /amcl_pose; logs + stops otherwise."""
        age = self._pose_age()
        if age is None:
            self._warn_pose('no /amcl_pose received yet — waiting for AMCL')
            return False
        if age > self.pose_timeout:
            hz = self._measured_pose_hz
            self._warn_pose(
                f'/amcl_pose stale ({age:.2f}s > {self.pose_timeout:.2f}s) '
                f'— last measured rate {hz:.1f} Hz; stopping')
            return False
        return True

    def _on_path(self, msg: Path) -> None:
        pts: List[Point2] = []
        yaws: List[float] = []
        for ps in msg.poses:
            pts.append((float(ps.pose.position.x), float(ps.pose.position.y)))
            q = ps.pose.orientation
            yaws.append(yaw_from_quat(q.x, q.y, q.z, q.w))
        if len(pts) < 1:
            self.path_pts = []
            self.path_yaws = []
            self._stop('empty path')
            self._set_state(CtrlState.IDLE, 'empty path')
            return

        self.path_pts = pts
        self.path_yaws = yaws
        # A new path belongs to a new goal — an old GOAL_REACHED must not
        # short-circuit it into final yaw / done.
        if self.goal_status in ('POSITION_REACHED', 'GOAL_REACHED'):
            self.goal_status = 'NOT_REACHED'
        self._settle_until = (
            self.get_clock().now().nanoseconds * 1e-9 + self.path_settle)
        self.nearest_idx = 0
        self._rotate_dir = 0.0
        self._yaw_tick = 0
        self._last_xy = None
        self._force_reseed = True
        self.blocked = False
        if self.pose is not None:
            x = self.pose.pose.position.x
            y = self.pose.pose.position.y
            self.nearest_idx, _ = self._nearest_in_range(x, y, 0, len(pts) - 1)
            self._last_xy = (x, y)
            self._force_reseed = False
        self._set_state(CtrlState.ALIGN, f'new path ({len(pts)} pts)')

    def _on_current_goal(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        self.goal_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

    def _on_collision(self, msg: String) -> None:
        try:
            blocked = bool(json.loads(msg.data).get('blocked', False))
        except Exception:
            blocked = False
        # At the goal we only rotate in place — ignore forward bumper.
        if self.goal_status in ('POSITION_REACHED', 'GOAL_REACHED'):
            if blocked:
                self.get_logger().info(
                    'collision ignored during final yaw / goal reached',
                    throttle_duration_sec=2.0)
            return
        # Recovery owns the robot — stay latched, do not fight cmd_vel.
        if self.recovery_active:
            self.blocked = True
            return
        if blocked and not self.blocked:
            self.blocked = True
            self._stop('collision')
            self._set_state(CtrlState.BLOCKED, 'collision — waiting for recovery')
            return
        if (not blocked and self.blocked
                and not self.recovery_active
                and self.recovery_state in ('idle', 'fail', '')):
            self.blocked = False
            if self.path_pts and self.enabled:
                self._set_state(CtrlState.ALIGN, 'collision clear — resume path')
            else:
                self._set_state(CtrlState.IDLE, 'collision clear')

    def _on_goal(self, msg: String) -> None:
        prev = self.goal_status
        self.goal_status = msg.data.strip()
        if self.goal_status == 'POSITION_REACHED' and prev != 'POSITION_REACHED':
            self.blocked = False
            self.get_logger().info('POSITION_REACHED — starting final yaw (ignore bumper)')

    def _on_recovery(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self.recovery_state = data.get('state', 'idle')
            self.recovery_active = bool(data.get('active', False)) or self.recovery_state in (
                'backup', 'spin', 'wait', 'replan')
        except Exception:
            self.recovery_state = 'idle'
            self.recovery_active = False
        # When recovery finishes, unlock so the new path can be followed.
        if not self.recovery_active and self.recovery_state in ('idle', 'fail'):
            if self.path_pts and self.enabled and not self.blocked:
                if self.state == CtrlState.BLOCKED:
                    self._set_state(CtrlState.ALIGN, 'recovery done — follow path')

    def _set_state(self, state: CtrlState, detail: str = '') -> None:
        if state != self.state:
            self.state = state
            self.status.set(state.value, detail or state.value)
            self.get_logger().info(f'state → {state.value} | {detail or state.value}')
        else:
            self.state = state

    def _stop(self, reason: str = 'stop') -> None:
        self.cmd_pub.publish(Twist())
        self._log_cmd(0.0, 0.0, reason, force=False)

    def _publish_cmd(self, linear: float, angular: float, action: str) -> None:
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(self.angular_sign * angular)
        self.cmd_pub.publish(cmd)
        self._dbg_action = action
        self._log_cmd(cmd.linear.x, cmd.angular.z, action, force=False)

    def _log_cmd(
        self, lin: float, ang: float, action: str, force: bool = False
    ) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        state_key = f'{self.state.value}:{action}'
        state_changed = state_key != self._last_logged_state
        due = (now - self._last_cmd_log_time) >= self.cmd_log_period
        if not force and not state_changed and not due:
            return
        if not self.enabled and not force:
            if not state_changed:
                return
        self._last_cmd_log_time = now
        self._last_logged_state = state_key
        look = self._dbg_look
        look_s = f'({look[0]:.2f},{look[1]:.2f})' if look is not None else 'n/a'
        turn = 'CCW(+)' if ang > 1e-4 else ('CW(-)' if ang < -1e-4 else 'none')
        pose_s = 'n/a'
        if self.pose is not None:
            pose_s = (
                f'({self.pose.pose.position.x:.2f},{self.pose.pose.position.y:.2f}) '
                f'yaw={self._dbg_yaw:.2f}rad/{math.degrees(self._dbg_yaw):.0f}deg'
            )
        self.get_logger().info(
            f'cmd | state={self.state.value} action={action} | '
            f'cmd_vel lin={lin:+.3f} ang={ang:+.3f} ({turn}) sign={self.angular_sign:+.0f} | '
            f'pose {pose_s} | look={look_s} desired_yaw={self._dbg_desired_yaw:.2f} '
            f'err={self._dbg_heading_err:+.2f}rad/{math.degrees(self._dbg_heading_err):+.0f}deg '
            f'idx={self.nearest_idx}/{max(0, len(self.path_pts) - 1)} '
            f'goal_status={self.goal_status}'
        )

    def _yaw(self) -> float:
        assert self.pose is not None
        q = self.pose.pose.orientation
        return yaw_from_quat(q.x, q.y, q.z, q.w)

    @staticmethod
    def _dist(a: Point2, b: Point2) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _nearest_in_range(
        self, x: float, y: float, lo: int, hi: int
    ) -> Tuple[int, float]:
        """Return (index, distance) of nearest path point in [lo, hi] inclusive."""
        n = len(self.path_pts)
        if n == 0:
            return 0, float('inf')
        lo = max(0, min(lo, n - 1))
        hi = max(0, min(hi, n - 1))
        if hi < lo:
            lo, hi = hi, lo
        best_i = lo
        best_d = float('inf')
        xy = (x, y)
        for i in range(lo, hi + 1):
            d = self._dist(xy, self.path_pts[i])
            if d < best_d:
                best_d = d
                best_i = i
        return best_i, best_d

    def _update_progress(self, x: float, y: float) -> None:
        """Forward-only path progress. Skip waypoints behind the robot→goal axis."""
        n = len(self.path_pts)
        if n == 0:
            self.nearest_idx = 0
            self._last_xy = (x, y)
            return

        jumped = (
            self._force_reseed
            or (
                self._last_xy is not None
                and self._dist(self._last_xy, (x, y)) > self.pose_jump_dist
            )
        )
        self._force_reseed = False

        if jumped:
            idx, dist = self._nearest_in_range(x, y, 0, n - 1)
            if self._last_xy is not None:
                self.get_logger().warn(
                    f'path reseed idx={idx} (d={dist:.2f})')
            self.nearest_idx = idx

        self._skip_behind_waypoints(x, y)
        self._advance_passed_waypoints(x, y)
        self._last_xy = (x, y)

    def _skip_behind_waypoints(self, x: float, y: float) -> None:
        """Consume only nearby waypoints the robot has already passed."""
        n = len(self.path_pts)
        if n < 2:
            return
        gx, gy = self.path_pts[-1]
        gdx, gdy = gx - x, gy - y
        gL = math.hypot(gdx, gdy)
        if gL < 1e-3:
            return
        gdx, gdy = gdx / gL, gdy / gL
        yaw = self._yaw() if self.pose is not None else 0.0
        hx, hy = math.cos(yaw), math.sin(yaw)

        while self.nearest_idx < n - 1:
            px, py = self.path_pts[self.nearest_idx]
            vx, vy = px - x, py - y
            d = math.hypot(vx, vy)
            if d < 0.12:
                self.nearest_idx += 1
                continue
            # Only skip if the waypoint is close AND behind — never jump to a
            # far edt-straight corner (that causes wall-cutting chords).
            if d < 0.45 and (vx * gdx + vy * gdy) < 0.0 and (vx * hx + vy * hy) < 0.0:
                self.get_logger().info(
                    f'skip passed wp idx={self.nearest_idx} d={d:.2f}')
                self.nearest_idx += 1
                continue
            break

    def _advance_passed_waypoints(self, x: float, y: float) -> None:
        """Push progress past waypoints the robot has already gone beyond."""
        n = len(self.path_pts)
        while self.nearest_idx < n - 1:
            curr = self.path_pts[self.nearest_idx]
            nxt = self.path_pts[self.nearest_idx + 1]
            sx = nxt[0] - curr[0]
            sy = nxt[1] - curr[1]
            seg2 = sx * sx + sy * sy
            if seg2 < 1e-12:
                self.nearest_idx += 1
                continue
            t = ((x - curr[0]) * sx + (y - curr[1]) * sy) / seg2
            d_curr = self._dist((x, y), curr)
            d_next = self._dist((x, y), nxt)
            if d_curr < 0.12 or t >= 0.95 or (t > 0.4 and d_next < d_curr):
                self.nearest_idx += 1
                continue
            break

    def _closest_on_remaining_path(
        self, x: float, y: float
    ) -> Tuple[Point2, int, float]:
        """Closest point on polyline from progress→goal. Returns (pt, seg_idx, t)."""
        n = len(self.path_pts)
        if n == 1:
            return self.path_pts[0], 0, 0.0
        best_d = float('inf')
        best_pt = self.path_pts[self.nearest_idx]
        best_i = max(0, min(self.nearest_idx, n - 2))
        best_t = 0.0
        start = best_i
        for i in range(start, n - 1):
            a = self.path_pts[i]
            b = self.path_pts[i + 1]
            sx, sy = b[0] - a[0], b[1] - a[1]
            seg2 = sx * sx + sy * sy
            if seg2 < 1e-12:
                continue
            t = ((x - a[0]) * sx + (y - a[1]) * sy) / seg2
            t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
            px = a[0] + t * sx
            py = a[1] + t * sy
            d = math.hypot(px - x, py - y)
            if d < best_d:
                best_d = d
                best_pt = (px, py)
                best_i = i
                best_t = t
        return best_pt, best_i, best_t

    def _path_tangent(self, idx: int) -> Point2:
        n = len(self.path_pts)
        if n < 2:
            return (1.0, 0.0)
        i = max(0, min(idx, n - 2))
        a = self.path_pts[i]
        b = self.path_pts[i + 1]
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        L = math.hypot(dx, dy)
        if L < 1e-9:
            return (1.0, 0.0) if i == 0 else self._path_tangent(i - 1)
        return (dx / L, dy / L)

    def _walk_path_forward(
        self, seg_idx: int, t: float, dist: float
    ) -> Tuple[Point2, int]:
        """Walk `dist` meters forward along the path from segment position."""
        n = len(self.path_pts)
        if n == 0:
            return (0.0, 0.0), 0
        if n == 1:
            return self.path_pts[0], 0
        i = max(0, min(seg_idx, n - 2))
        a = self.path_pts[i]
        b = self.path_pts[i + 1]
        cx = a[0] + t * (b[0] - a[0])
        cy = a[1] + t * (b[1] - a[1])
        remaining = dist
        while i < n - 1:
            nx, ny = self.path_pts[i + 1]
            seg = math.hypot(nx - cx, ny - cy)
            if seg < 1e-6:
                i += 1
                cx, cy = nx, ny
                continue
            if seg >= remaining:
                u = remaining / seg
                return (cx + u * (nx - cx), cy + u * (ny - cy)), i
            remaining -= seg
            cx, cy = nx, ny
            i += 1
        return self.path_pts[-1], n - 2

    def _lookahead_point(self, x: float, y: float) -> Point2:
        """Pure-pursuit carrot: closest on remaining path + short forward walk."""
        n = len(self.path_pts)
        if n == 0:
            return (x, y)
        if n == 1:
            return self.path_pts[0]

        closest, seg_i, t = self._closest_on_remaining_path(x, y)
        # Far off the path (progress jumped, or the robot was pushed): steer at
        # whichever real path point is nearer. Aiming at a point interpolated
        # along the robot→waypoint ray would sit off the path, so the robot
        # would cut a chord across whatever lies between it and the path.
        if self._dist((x, y), closest) > 0.60:
            forward = self.path_pts[min(self.nearest_idx + 1, n - 1)]
            if self._dist((x, y), forward) < self._dist((x, y), closest):
                return forward
            return closest

        # Use configured lookahead as-is (no 0.20 floor) so short values
        # actually track the path tightly instead of cutting corners.
        look, _ = self._walk_path_forward(
            seg_i, t, max(self.lookahead_dist, 0.05))

        # Never aim more than ~2x lookahead away, and keep the carrot on the
        # polyline while doing it: fall back to the closest path point instead
        # of pulling the carrot off the path toward the robot.
        max_d = max(0.15, 2.0 * self.lookahead_dist)
        if self._dist((x, y), look) > max_d:
            return closest
        return look

    def _heading_err_to(self, target: Point2) -> float:
        assert self.pose is not None
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        yaw = self._yaw()

        lx, ly = target[0] - x, target[1] - y
        Ld = math.hypot(lx, ly)
        hx, hy = math.cos(yaw), math.sin(yaw)

        # If carrot is behind the robot nose, steer along path tangent instead
        # of spinning 180° to face backward.
        if Ld < 0.05 or (lx * hx + ly * hy) < 0.0:
            fx, fy = self._path_tangent(self.nearest_idx)
            # Prefer tangent that still points toward the goal.
            gx = self.path_pts[-1][0] - x
            gy = self.path_pts[-1][1] - y
            if fx * gx + fy * gy < 0.0:
                fx, fy = -fx, -fy
            desired = math.atan2(fy, fx)
        else:
            desired = math.atan2(ly, lx)

        self._dbg_look = target
        self._dbg_desired_yaw = desired
        self._dbg_yaw = yaw
        self._dbg_heading_err = wrap_angle(desired - yaw)
        return self._dbg_heading_err

    def _ang_locked(
        self,
        err: float,
        speed: float,
        floor: Optional[float] = None,
        deadband: Optional[float] = None,
    ) -> float:
        """Turn toward ``err``, easing off as the heading closes.

        AMCL yaw lands at scan rate while this loop runs at control_rate_hz,
        so a full-rate turn keeps going for one whole pose period after it has
        already crossed the aligned band — the robot then reverses and hunts
        forever instead of settling. Taper the command so the turn left inside
        one pose period stays smaller than the band.

        ``floor`` is the slowest command that still moves the drive in the
        current mode: in-place rotation needs a much larger one than steering
        while the wheels are already spinning.
        """
        band = self.heading_deadband if deadband is None else deadband
        if abs(err) < band:
            self._rotate_dir = 0.0
            return 0.0
        if self._rotate_dir == 0.0 or err * self._rotate_dir < 0.0:
            self._rotate_dir = 1.0 if err > 0.0 else -1.0
        taper = max(self.heading_align, 1e-3)
        low = self.min_angular_vel if floor is None else floor
        low = min(low, speed)
        mag = speed if abs(err) >= taper else max(low, speed * abs(err) / taper)
        return self._rotate_dir * mag

    def _tick(self) -> None:
        if not self.enabled:
            if self.get_clock().now().nanoseconds * 1e-9 < self._brake_until:
                self._stop('disabled')
            return

        # Recovery owns /cmd_vel for the whole active sequence (backup/spin/wait/replan).
        if self.recovery_active:
            return

        if self.get_clock().now().nanoseconds * 1e-9 < self._settle_until:
            self._stop('settling after new path')
            self._set_state(CtrlState.ALIGN, 'settling — letting AMCL catch up')
            return

        if not self._pose_ok():
            self._stop('no/stale amcl pose')
            age = self._pose_age()
            detail = 'no amcl pose' if age is None else f'stale amcl pose {age:.2f}s'
            self._set_state(CtrlState.IDLE, detail)
            return

        if self.blocked:
            # If XY goal was reached while blocked, finish yaw instead of waiting.
            if self.goal_status == 'POSITION_REACHED':
                self.blocked = False
                self._do_final_yaw()
                return
            self._stop('collision blocked')
            if self.state != CtrlState.BLOCKED:
                self._set_state(CtrlState.BLOCKED, 'waiting for new path')
            return

        # Goal Checker owns completion — controller only reacts.
        if self.goal_status == 'GOAL_REACHED':
            self._rotate_dir = 0.0
            self.blocked = False
            self._stop('GOAL_REACHED')
            self._set_state(CtrlState.DONE, 'GOAL_REACHED')
            return

        if not self.path_pts:
            self._stop('no path')
            if self.state not in (CtrlState.IDLE, CtrlState.DONE):
                self._set_state(CtrlState.IDLE, 'no path')
            return

        if self.goal_status == 'POSITION_REACHED':
            self._do_final_yaw()
            return

        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        self._update_progress(x, y)

        look = self._lookahead_point(x, y)
        err = self._heading_err_to(look)

        if abs(err) > self.heading_align or self.state == CtrlState.ALIGN:
            if abs(err) <= self.heading_aligned:
                self._rotate_dir = 0.0
                self._set_state(CtrlState.FOLLOW, 'aligned — start drive')
                self._publish_cmd(self.linear_vel, 0.0, 'drive_after_align')
                return
            w = self._ang_locked(err, self.angular_vel)
            self._set_state(
                CtrlState.ALIGN,
                f'rotate_in_place err={err:+.2f} → cmd_ang={self.angular_sign * w:+.2f}')
            self._publish_cmd(0.0, w, 'rotate_in_place')
            return

        self._set_state(CtrlState.FOLLOW, f'track idx={self.nearest_idx}')
        w = 0.0
        if abs(err) > self.heading_deadband:
            w = self._ang_locked(
                err, self.fine_angular_vel, floor=self.min_steer_vel)
            action = 'drive+steer'
        else:
            self._rotate_dir = 0.0
            action = 'drive_straight'
        self._publish_cmd(self.linear_vel, w, action)

    def _do_final_yaw(self) -> None:
        """Rotate to the commanded goal yaw until Goal Checker says reached.

        The target is the goal orientation, not the path's last tangent: the
        checker judges against the goal, so aiming anywhere else leaves the
        robot parked at a heading the checker happened to accept.
        """
        assert self.pose is not None
        if self.goal_yaw is not None:
            target = self.goal_yaw
        elif self.path_yaws:
            target = self.path_yaws[-1]
        else:
            target = self._yaw()
        self._dbg_yaw = self._yaw()
        self._dbg_desired_yaw = target
        self._dbg_heading_err = wrap_angle(target - self._dbg_yaw)
        self._dbg_look = self.path_pts[-1] if self.path_pts else None
        err = self._dbg_heading_err

        if abs(err) <= self.final_yaw_tol:
            self._rotate_dir = 0.0
            self._yaw_tick = 0
            self._stop('final_yaw settled — wait GOAL_REACHED')
            self._set_state(CtrlState.FINAL_YAW, 'waiting GOAL_REACHED')
            return

        if abs(err) <= self.yaw_slow_err:
            # Endgame: low speed, and only every (1 + off) ticks, so a single
            # pose period cannot turn further than the tolerance.
            self._yaw_tick += 1
            if self.yaw_pulse_off and (self._yaw_tick % (1 + self.yaw_pulse_off)):
                self._stop('final_yaw pulse rest')
                self._set_state(CtrlState.FINAL_YAW, f'yaw err={err:+.2f} (rest)')
                return
            speed = self.yaw_slow_vel
            floor = min(self.yaw_slow_vel, self.min_angular_vel)
        else:
            self._yaw_tick = 0
            speed = self.angular_vel
            floor = self.min_angular_vel

        w = self._ang_locked(
            err, speed, floor=floor, deadband=self.final_yaw_tol)
        self._set_state(CtrlState.FINAL_YAW, f'yaw err={err:+.2f}')
        self._publish_cmd(0.0, w, 'final_yaw_rotate')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop('shutdown')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
