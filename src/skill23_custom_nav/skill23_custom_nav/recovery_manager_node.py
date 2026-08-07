#!/usr/bin/env python3
"""Simple recovery executor for skill23_custom_nav.

Flow when collision.blocked while navigating:
  1. BACKUP if rear clearance allows (min..max)
  2. SPIN toward the more open side (left/right) until front clears or angle cap
  3. WAIT for local EDT
  4. REPLAN
  5. IDLE — controller may follow the new path

While active=true the controller must not follow the path. New collision
rising-edges are ignored until recovery finishes (so a spin that clears the
front does not restart recovery mid-sequence).

Stop (controller enable=false) aborts everything immediately.
"""

from __future__ import annotations

import json
import math
import time
from enum import Enum
from typing import Any, Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String

from skill23_custom_nav.node_status import NodeStatusPub
from skill23_custom_nav.utils import wrap_angle, yaw_from_quat


class RecoveryState(Enum):
    IDLE = 'idle'
    BACKUP = 'backup'
    SPIN = 'spin'
    WAIT = 'wait'
    REPLAN = 'replan'
    FAIL = 'fail'


class RecoveryManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('cn_recovery')
        self.declare_parameter('recovery_wait_sec', 1.0)
        self.declare_parameter('edt_timeout_sec', 5.0)
        self.declare_parameter('path_timeout_sec', 8.0)
        self.declare_parameter('max_recovery_attempts', 5)
        self.declare_parameter('min_backup_m', 0.05)
        self.declare_parameter('max_backup_m', 0.20)
        self.declare_parameter('backup_rear_margin_m', 0.03)
        self.declare_parameter('backup_linear_vel', -0.10)
        self.declare_parameter('backup_timeout_sec', 5.0)
        self.declare_parameter('spin_angle_rad', 1.5708)
        self.declare_parameter('spin_angular_vel', 0.30)
        self.declare_parameter('spin_timeout_sec', 8.0)
        self.declare_parameter('control_rate_hz', 20.0)

        self.wait_sec = float(self.get_parameter('recovery_wait_sec').value)
        self.edt_timeout = float(self.get_parameter('edt_timeout_sec').value)
        self.path_timeout = float(self.get_parameter('path_timeout_sec').value)
        self.max_attempts = int(self.get_parameter('max_recovery_attempts').value)
        self.min_backup = float(self.get_parameter('min_backup_m').value)
        self.max_backup = float(self.get_parameter('max_backup_m').value)
        self.backup_margin = float(self.get_parameter('backup_rear_margin_m').value)
        self.backup_vel = float(self.get_parameter('backup_linear_vel').value)
        self.backup_timeout = float(self.get_parameter('backup_timeout_sec').value)
        self.spin_angle = abs(float(self.get_parameter('spin_angle_rad').value))
        self.spin_vel = abs(float(self.get_parameter('spin_angular_vel').value))
        self.spin_timeout = float(self.get_parameter('spin_timeout_sec').value)
        period = 1.0 / max(float(self.get_parameter('control_rate_hz').value), 1.0)

        if self.backup_vel > 0.0:
            self.backup_vel = -self.backup_vel

        self.blocked = False
        self.clearance: Dict[str, float] = {
            'front': 1.0, 'rear': 1.0, 'left': 1.0, 'right': 1.0}
        self.goal_status = 'NOT_REACHED'
        self.navigating = False
        self.state = RecoveryState.IDLE
        self.attempts = 0
        self.detail = 'watching'

        self.edt_ready = False
        self.awaiting_path = False
        self.phase_started = 0.0
        self.last_edt_req = 0.0
        self.path_seq_at_replan = 0
        self.path_seq = 0

        self._pose_xy: Optional[Tuple[float, float]] = None
        self._pose_yaw: Optional[float] = None
        self._backup_start_xy: Optional[Tuple[float, float]] = None
        self._backup_target_m = 0.0
        self._spin_start_yaw: Optional[float] = None
        self._spin_dir = 1.0

        self.status = NodeStatusPub(self, 'recovery')
        self.recovery_pub = self.create_publisher(String, '/custom_nav/recovery', 10)
        self.replan_pub = self.create_publisher(Empty, '/custom_nav/replan', 10)
        self.edt_req_pub = self.create_publisher(
            Bool, '/custom_nav/local_edt/request', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(String, '/custom_nav/collision', self._on_collision, 10)
        self.create_subscription(Bool, '/custom_nav/controller/enable', self._on_enable, 10)
        self.create_subscription(
            Bool, '/custom_nav/local_edt/active', self._on_edt_active, 10)
        self.create_subscription(Path, '/custom_nav/path', self._on_path, 10)
        self.create_subscription(String, '/custom_nav/node_status', self._on_node_status, 10)
        self.create_subscription(String, '/custom_nav/goal_status', self._on_goal_status, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, 10)
        self.create_timer(period, self._tick)
        self._set_detail('watching')
        self.status.set('idle', 'watching for front collision')

    # ── helpers ─────────────────────────────────────────────────────────

    def _active(self) -> bool:
        return self.state in (
            RecoveryState.BACKUP, RecoveryState.SPIN,
            RecoveryState.WAIT, RecoveryState.REPLAN)

    def _set_detail(self, text: str) -> None:
        self.detail = text

    def _log_decision(self, text: str) -> None:
        self._set_detail(text)
        self.get_logger().info(f'[recovery] {text}')
        self.status.set(self.state.value, text, attempts=self.attempts, blocked=self.blocked)
        self._publish_state()

    def _set_state(self, state: RecoveryState, detail: str) -> None:
        changed = state != self.state
        self.state = state
        self._set_detail(detail)
        if changed:
            self.status.set(
                state.value, detail, attempts=self.attempts, blocked=self.blocked)
            self.get_logger().info(f'[recovery] {detail}')
        self._publish_state()

    def _publish_state(self) -> None:
        payload = {
            'state': self.state.value,
            'active': self._active(),
            'attempts': self.attempts,
            'blocked': self.blocked,
            'detail': self.detail,
            'action': self.detail,
            'clearance': dict(self.clearance),
            'backup_target_m': round(self._backup_target_m, 3),
            'spin_dir': int(self._spin_dir),
        }
        if self.state == RecoveryState.BACKUP and self._backup_start_xy and self._pose_xy:
            payload['backup_m'] = round(self._backup_traveled(), 3)
        if (self.state == RecoveryState.SPIN
                and self._spin_start_yaw is not None
                and self._pose_yaw is not None):
            turned = abs(wrap_angle(self._pose_yaw - self._spin_start_yaw))
            payload['spin_rad'] = round(turned, 3)
            payload['spin_deg'] = round(math.degrees(turned), 1)
        msg = String()
        msg.data = json.dumps(payload)
        self.recovery_pub.publish(msg)

    def _stop_cmd(self) -> None:
        self.cmd_pub.publish(Twist())

    def _publish_backup_cmd(self) -> None:
        tw = Twist()
        tw.linear.x = self.backup_vel
        self.cmd_pub.publish(tw)

    def _publish_spin_cmd(self) -> None:
        tw = Twist()
        tw.angular.z = self._spin_dir * self.spin_vel
        self.cmd_pub.publish(tw)

    def _request_local_edt(self, enable: bool) -> None:
        msg = Bool()
        msg.data = bool(enable)
        self.edt_req_pub.publish(msg)
        self.last_edt_req = time.time()
        if not enable:
            self.edt_ready = False

    def _backup_traveled(self) -> float:
        if not self._backup_start_xy or not self._pose_xy:
            return 0.0
        dx = self._pose_xy[0] - self._backup_start_xy[0]
        dy = self._pose_xy[1] - self._backup_start_xy[1]
        return math.hypot(dx, dy)

    def _choose_spin_dir(self) -> float:
        left = float(self.clearance.get('left', 0.0))
        right = float(self.clearance.get('right', 0.0))
        # +1 = CCW = left, -1 = CW = right
        return 1.0 if left >= right else -1.0

    def _compute_backup_target(self) -> float:
        rear = float(self.clearance.get('rear', 0.0))
        safe = max(0.0, rear - self.backup_margin)
        if safe < self.min_backup:
            return 0.0
        return min(self.max_backup, safe)

    def _abort_all(self, reason: str) -> None:
        self._stop_cmd()
        self.awaiting_path = False
        self._request_local_edt(False)
        self.attempts = 0
        self._backup_target_m = 0.0
        self._set_state(RecoveryState.IDLE, reason)

    def _begin_backup(self, meters: float) -> None:
        self.edt_ready = False
        self.awaiting_path = False
        self._request_local_edt(False)
        self.phase_started = time.time()
        self._backup_start_xy = self._pose_xy
        self._backup_target_m = meters
        self._set_state(
            RecoveryState.BACKUP,
            f'Moving backward {meters:.2f} m')

    def _begin_spin(self) -> None:
        self._stop_cmd()
        self.phase_started = time.time()
        self._spin_start_yaw = self._pose_yaw
        self._spin_dir = self._choose_spin_dir()
        side = 'left (CCW)' if self._spin_dir > 0 else 'right (CW)'
        left = float(self.clearance.get('left', 0.0))
        right = float(self.clearance.get('right', 0.0))
        self._set_state(
            RecoveryState.SPIN,
            f'Turning {side} toward open space '
            f'(L={left:.2f} m, R={right:.2f} m)')

    def _begin_wait_for_edt(self, detail: str) -> None:
        self._stop_cmd()
        self.edt_ready = False
        self.awaiting_path = False
        self.phase_started = time.time()
        self._request_local_edt(True)
        self._set_state(RecoveryState.WAIT, detail)

    def _finish_success(self) -> None:
        self._stop_cmd()
        self.awaiting_path = False
        self._request_local_edt(False)
        self._set_state(
            RecoveryState.IDLE,
            'Recovery done — controller may follow the new path')
        if self.blocked and self.navigating:
            if self.attempts >= self.max_attempts:
                self._fail('Still blocked after replan — giving up')
                return
            self._start_recovery('Still blocked after replan — trying again')
        else:
            self.attempts = 0

    def _fail(self, detail: str) -> None:
        self._stop_cmd()
        self.awaiting_path = False
        self._request_local_edt(False)
        self._set_state(RecoveryState.FAIL, detail)

    def _start_recovery(self, why: str) -> None:
        if not self.navigating or self._active():
            return
        if self.goal_status in ('POSITION_REACHED', 'GOAL_REACHED'):
            return

        self.attempts += 1
        if self.attempts > self.max_attempts:
            self._fail(f'Recovery failed after {self.max_attempts} attempts')
            return

        backup_m = self._compute_backup_target()
        self._log_decision(
            f'{why}. Clearances F={self.clearance.get("front", 0):.2f} '
            f'R={self.clearance.get("rear", 0):.2f} '
            f'L={self.clearance.get("left", 0):.2f} '
            f'Ri={self.clearance.get("right", 0):.2f} m '
            f'(attempt {self.attempts})')

        if backup_m >= self.min_backup:
            self._begin_backup(backup_m)
        else:
            self._log_decision(
                f'Not enough rear space to back up '
                f'({self.clearance.get("rear", 0):.2f} m) — turning instead')
            self._begin_spin()

    # ── inputs ──────────────────────────────────────────────────────────

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._pose_xy = (p.x, p.y)
        self._pose_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

    def _on_enable(self, msg: Bool) -> None:
        was = self.navigating
        self.navigating = bool(msg.data)
        if was and not self.navigating:
            # Dashboard Stop — abort recovery completely.
            self._abort_all('Navigation stopped — recovery aborted')
            return
        if not was and self.navigating:
            self._set_state(RecoveryState.IDLE, 'Armed — watching for front collision')
            if self.blocked:
                self._start_recovery('Already blocked when Start pressed')

    def _on_goal_status(self, msg: String) -> None:
        self.goal_status = msg.data.strip()
        if self.goal_status in ('POSITION_REACHED', 'GOAL_REACHED'):
            if self._active():
                self._abort_all('At goal — skipping recovery')

    def _on_collision(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        was_blocked = self.blocked
        self.blocked = bool(data.get('blocked', False))
        cl = data.get('clearance') or {}
        for k in ('front', 'rear', 'left', 'right'):
            try:
                self.clearance[k] = float(cl.get(k, self.clearance[k]))
            except (TypeError, ValueError):
                pass

        if not self.navigating:
            return
        if self.goal_status in ('POSITION_REACHED', 'GOAL_REACHED'):
            return

        # While recovery owns the robot, ignore rising edges so a cleared front
        # mid-spin does not restart the sequence.
        if self._active():
            return

        if self.blocked and not was_blocked and self.state == RecoveryState.IDLE:
            self._start_recovery('Front collision detected')

    def _on_edt_active(self, msg: Bool) -> None:
        self.edt_ready = bool(msg.data)

    def _on_path(self, msg: Path) -> None:
        self.path_seq += 1
        if not self.awaiting_path or self.state != RecoveryState.REPLAN:
            return
        if self.path_seq <= self.path_seq_at_replan:
            return
        if not msg.poses:
            return
        self._finish_success()

    def _on_node_status(self, msg: String) -> None:
        if self.state != RecoveryState.REPLAN or not self.awaiting_path:
            return
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if data.get('node') != 'planner' or data.get('state') != 'failed':
            return
        if self.attempts >= self.max_attempts:
            self._fail(f'Planner failed after {self.attempts} attempts')
            return
        self.attempts += 1
        self._begin_wait_for_edt(
            'Planner failed — requesting local EDT again')

    # ── state machine ───────────────────────────────────────────────────

    def _tick(self) -> None:
        self._publish_state()
        if not self.navigating:
            return
        now = time.time()

        if self.state == RecoveryState.BACKUP:
            if self._backup_start_xy is None and self._pose_xy is not None:
                self._backup_start_xy = self._pose_xy
            traveled = self._backup_traveled()
            if traveled >= self._backup_target_m:
                self._stop_cmd()
                self._log_decision(
                    f'Backup done ({traveled:.2f} m) — now turning to open side')
                self._begin_spin()
                return
            if now - self.phase_started >= self.backup_timeout:
                self._stop_cmd()
                self._log_decision(
                    f'Backup timeout after {traveled:.2f} m — turning to open side')
                self._begin_spin()
                return
            self._publish_backup_cmd()
            return

        if self.state == RecoveryState.SPIN:
            if self._spin_start_yaw is None and self._pose_yaw is not None:
                self._spin_start_yaw = self._pose_yaw
            turned = 0.0
            if self._spin_start_yaw is not None and self._pose_yaw is not None:
                turned = abs(wrap_angle(self._pose_yaw - self._spin_start_yaw))

            # Success: front is clear enough after some rotation, or angle cap.
            front_ok = not self.blocked and float(self.clearance.get('front', 0.0)) > 0.15
            if front_ok and turned >= 0.15:
                self._stop_cmd()
                self._begin_wait_for_edt(
                    f'Turn succeeded ({math.degrees(turned):.0f}°) — '
                    f'robot facing open direction. Creating local EDT')
                return
            if turned >= self.spin_angle:
                self._stop_cmd()
                self._begin_wait_for_edt(
                    f'Turned {math.degrees(turned):.0f}° — creating local EDT')
                return
            if now - self.phase_started >= self.spin_timeout:
                self._stop_cmd()
                self._begin_wait_for_edt(
                    f'Turn timeout ({math.degrees(turned):.0f}°) — creating local EDT')
                return
            self._publish_spin_cmd()
            return

        if self.state == RecoveryState.WAIT:
            elapsed = now - self.phase_started
            if not self.edt_ready and (now - self.last_edt_req) >= 1.0:
                self._request_local_edt(True)
            if elapsed < self.wait_sec:
                return
            if not self.edt_ready:
                if elapsed > self.wait_sec + self.edt_timeout:
                    if self.attempts >= self.max_attempts:
                        self._fail('Local EDT timeout')
                    else:
                        self._begin_wait_for_edt(
                            'Local EDT timeout — retrying')
                return
            if self.attempts > self.max_attempts:
                self._fail(f'Failed after {self.attempts} attempts')
                return
            self.path_seq_at_replan = self.path_seq
            self.awaiting_path = True
            self.phase_started = now
            self.replan_pub.publish(Empty())
            self._set_state(
                RecoveryState.REPLAN,
                'Local EDT ready — replanning path')

        elif self.state == RecoveryState.REPLAN:
            if now - self.phase_started > self.path_timeout:
                if self.attempts >= self.max_attempts:
                    self._fail(f'Path timeout after {self.attempts} attempts')
                else:
                    self._begin_wait_for_edt(
                        'Path timeout — requesting local EDT again')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecoveryManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
