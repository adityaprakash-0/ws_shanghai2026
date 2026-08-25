#!/usr/bin/env python3
"""Simple recovery executor for skill23_custom_nav.

Collision triggers recovery from FRONT only (0°±5°).
After backup, spin so the robot's COMPLETE FRONT faces the open side.

Easy sequence when front is blocked while navigating:
  1. BACKUP  — distance from rear hemisphere min (clamped min..max)
  2. SPIN    — rotate until front faces the most-open sector
  3. WAIT    — local EDT
  4. REPLAN  — new path
  5. IDLE    — controller follows again

Spin target (after backup):
  Pick sector with max clearance among:
    FL(+45) FR(-45) L(+90) R(-90) RL(+135) RR(-135)
  Spin by that sector's bearing so robot 0° faces that opening.
  Done early when COMPLETE FRONT is open:
    front > open_m AND front_left > open_side_m AND front_right > open_side_m

While active=true the controller must not follow the path.
"""

from __future__ import annotations

import json
import math
import time
from enum import Enum
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String

from skill23_studio.nav.node_status import NodeStatusPub
from skill23_studio.nav.utils import wrap_angle, yaw_from_quat


class RecoveryState(Enum):
    IDLE = 'idle'
    BACKUP = 'backup'
    SPIN = 'spin'
    WAIT = 'wait'
    REPLAN = 'replan'
    FAIL = 'fail'


_CLEAR_KEYS = (
    'front', 'front_left', 'front_right', 'left', 'right',
    'rear_left', 'rear_right', 'rear',
)

# Sector name -> bearing in base_footprint (spin this much to face it)
_FACE_SECTORS: Tuple[Tuple[str, float], ...] = (
    ('front_left', math.radians(45.0)),
    ('front_right', math.radians(-45.0)),
    ('left', math.radians(90.0)),
    ('right', math.radians(-90.0)),
    ('rear_left', math.radians(135.0)),
    ('rear_right', math.radians(-135.0)),
)


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
        self.declare_parameter('open_m', 0.20)
        self.declare_parameter('open_side_m', 0.12)
        self.declare_parameter('spin_fl_fr_rad', 0.7854)  # yaml compat
        self.declare_parameter('spin_side_rad', 1.5708)   # yaml compat
        self.declare_parameter('spin_angular_vel', 0.30)
        self.declare_parameter('spin_timeout_sec', 8.0)
        self.declare_parameter('spin_angle_rad', 1.5708)
        self.declare_parameter('control_rate_hz', 20.0)
        # Recovery is level-triggered: the front must stay blocked this long
        # while IDLE before recovery starts. A pure false->true edge missed
        # every case where the level was already true when IDLE was re-entered
        # (goal reached mid-recovery, block latched across a waypoint switch),
        # which is what deadlocked the controller against a silent recovery.
        self.declare_parameter('block_trigger_sec', 0.5)
        # FAIL means "give up on THIS obstacle", not "disable recovery for the
        # rest of the run". Once the front has been clear this long, FAIL falls
        # back to IDLE so a later, unrelated obstacle re-arms recovery.
        self.declare_parameter('fail_reset_clear_sec', 3.0)
        # _finish_success must see the block persist this long before spending
        # another attempt, so the checker's clear hysteresis / grid lag cannot
        # burn attempts or drop into IDLE-while-blocked.
        self.declare_parameter('still_blocked_confirm_sec', 0.4)

        self.wait_sec = float(self.get_parameter('recovery_wait_sec').value)
        self.edt_timeout = float(self.get_parameter('edt_timeout_sec').value)
        self.path_timeout = float(self.get_parameter('path_timeout_sec').value)
        self.max_attempts = int(self.get_parameter('max_recovery_attempts').value)
        self.min_backup = float(self.get_parameter('min_backup_m').value)
        self.max_backup = float(self.get_parameter('max_backup_m').value)
        self.backup_margin = float(self.get_parameter('backup_rear_margin_m').value)
        self.backup_vel = float(self.get_parameter('backup_linear_vel').value)
        self.backup_timeout = float(self.get_parameter('backup_timeout_sec').value)
        self.open_m = float(self.get_parameter('open_m').value)
        self.open_side_m = float(self.get_parameter('open_side_m').value)
        self.spin_vel = abs(float(self.get_parameter('spin_angular_vel').value))
        self.spin_timeout = float(self.get_parameter('spin_timeout_sec').value)
        self.block_trigger_sec = float(self.get_parameter('block_trigger_sec').value)
        self.fail_reset_clear_sec = float(
            self.get_parameter('fail_reset_clear_sec').value)
        self.still_blocked_confirm_sec = float(
            self.get_parameter('still_blocked_confirm_sec').value)
        period = 1.0 / max(float(self.get_parameter('control_rate_hz').value), 1.0)

        if self.backup_vel > 0.0:
            self.backup_vel = -self.backup_vel

        self.blocked = False
        # Wall-clock timestamps of the last observed blocked / clear edges,
        # used for level-triggering, FAIL auto-reset and still-blocked confirm.
        self._blocked_since: Optional[float] = None
        self._clear_since: Optional[float] = time.time()
        self.clearance: Dict[str, float] = {k: 1.0 for k in _CLEAR_KEYS}
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
        self._spin_target = math.radians(90.0)
        self._spin_face_name = ''

        self.status = NodeStatusPub(self, 'recovery')
        self.recovery_pub = self.create_publisher(String, '/custom_nav/recovery', 10)
        self.replan_pub = self.create_publisher(Empty, '/custom_nav/replan', 10)
        self.edt_req_pub = self.create_publisher(
            Bool, '/custom_nav/local_edt/request', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(String, '/custom_nav/collision', self._on_collision, 10)
        self.create_subscription(
            Bool, '/custom_nav/controller/enable', self._on_enable,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ))
        self.create_subscription(
            Bool, '/custom_nav/local_edt/active', self._on_edt_active, 10)
        self.create_subscription(Path, '/custom_nav/path', self._on_path, 10)
        self.create_subscription(String, '/custom_nav/node_status', self._on_node_status, 10)
        self.create_subscription(String, '/custom_nav/goal_status', self._on_goal_status, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, 10)
        self.create_timer(period, self._tick)
        self._last_state_pub = 0.0
        self._set_detail('watching')
        self.status.set('idle', 'watching for front collision')

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
            'spin_target_deg': round(math.degrees(self._spin_target), 1),
            'spin_face': self._spin_face_name,
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

    def _compute_backup_target(self) -> float:
        rear = float(self.clearance.get('rear', 0.0))
        safe = max(0.0, rear - self.backup_margin)
        if safe < self.min_backup:
            return 0.0
        return min(self.max_backup, safe)

    def _complete_front_open(self) -> bool:
        """True when whole front (0° + diagonals) sits in open space."""
        if self.blocked:
            return False
        f = float(self.clearance.get('front', 0.0))
        fl = float(self.clearance.get('front_left', 0.0))
        fr = float(self.clearance.get('front_right', 0.0))
        return f > self.open_m and fl > self.open_side_m and fr > self.open_side_m

    def _decide_spin(self) -> Tuple[float, float, str]:
        """Spin so robot front faces the most-open sector."""
        best_name = 'left'
        best_ang = math.radians(90.0)
        best_d = -1.0
        for name, ang in _FACE_SECTORS:
            d = float(self.clearance.get(name, 0.0))
            # Prefer larger clearance; on tie prefer smaller turn.
            if d > best_d + 1e-6 or (
                    abs(d - best_d) <= 1e-6 and abs(ang) < abs(best_ang)):
                best_d = d
                best_name = name
                best_ang = ang

        if best_d < 1e-3:
            self._spin_face_name = 'left'
            return (
                1.0, math.radians(90.0),
                'no open sector — default turn left 90°')

        if abs(best_ang) < 1e-3:
            self._spin_face_name = best_name
            return 1.0, 0.0, f'{best_name} already ahead ({best_d:.2f} m) — skip spin'

        spin_dir = 1.0 if best_ang > 0.0 else -1.0
        target = abs(best_ang)
        side = 'left' if spin_dir > 0.0 else 'right'
        self._spin_face_name = best_name
        why = (
            f'face open {best_name} ({best_d:.2f} m) — turn {side} '
            f'~{math.degrees(target):.0f}° so complete front points there '
            f'(FL={self.clearance.get("front_left", 0):.2f} '
            f'FR={self.clearance.get("front_right", 0):.2f} '
            f'L={self.clearance.get("left", 0):.2f} '
            f'Ri={self.clearance.get("right", 0):.2f} '
            f'RL={self.clearance.get("rear_left", 0):.2f} '
            f'RR={self.clearance.get("rear_right", 0):.2f})')
        return spin_dir, target, why

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
            f'Moving backward {meters:.2f} m '
            f'(rear clear {float(self.clearance.get("rear", 0)):.2f} m)')

    def _begin_spin(self) -> None:
        self._stop_cmd()
        self.phase_started = time.time()
        self._spin_start_yaw = self._pose_yaw
        self._spin_dir, self._spin_target, why = self._decide_spin()
        if self._spin_target < 1e-3:
            self._begin_wait_for_edt(why + ' — creating local EDT')
            return
        self._set_state(RecoveryState.SPIN, why)

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
        # Only treat this as "still blocked" if the block has persisted past the
        # checker's clear hysteresis / grid lag. A stale single-frame block here
        # would otherwise burn an attempt; a stale clear would drop us into IDLE
        # while actually blocked, where the level trigger in _tick re-arms us.
        now = time.time()
        confirmed_block = (
            self.blocked
            and self._blocked_since is not None
            and (now - self._blocked_since) >= self.still_blocked_confirm_sec)
        if confirmed_block and self.navigating:
            if self.attempts >= self.max_attempts:
                self._fail('Still blocked after replan — giving up')
                return
            self._start_recovery('Still blocked after replan — trying again')
        elif not self.blocked:
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

        cl = self.clearance
        self._log_decision(
            f'{why}. F={cl.get("front", 0):.2f} FL={cl.get("front_left", 0):.2f} '
            f'FR={cl.get("front_right", 0):.2f} L={cl.get("left", 0):.2f} '
            f'Ri={cl.get("right", 0):.2f} RL={cl.get("rear_left", 0):.2f} '
            f'RR={cl.get("rear_right", 0):.2f} R={cl.get("rear", 0):.2f} m '
            f'(attempt {self.attempts})')

        backup_m = self._compute_backup_target()
        if backup_m >= self.min_backup:
            self._begin_backup(backup_m)
        else:
            self._log_decision(
                f'Not enough rear space to back up '
                f'({cl.get("rear", 0):.2f} m) — turning to face open side')
            self._begin_spin()

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._pose_xy = (p.x, p.y)
        self._pose_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

    def _on_enable(self, msg: Bool) -> None:
        was = self.navigating
        self.navigating = bool(msg.data)
        if was and not self.navigating:
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
        for k in _CLEAR_KEYS:
            try:
                if k in cl:
                    self.clearance[k] = float(cl[k])
            except (TypeError, ValueError):
                pass

        # Track how long the level has held. _tick uses these for
        # level-triggering, FAIL auto-reset and the still-blocked confirm, so
        # every decision looks at the sustained level, not a single edge.
        now = time.time()
        if self.blocked:
            if not was_blocked or self._blocked_since is None:
                self._blocked_since = now
            self._clear_since = None
        else:
            if was_blocked or self._clear_since is None:
                self._clear_since = now
            self._blocked_since = None

        # Trigger and FAIL-reset are handled level-based in _tick; nothing edge
        # based here, so a block that was already latched when IDLE re-entered
        # still gets serviced.

    def _on_edt_active(self, msg: Bool) -> None:
        self.edt_ready = bool(msg.data)

    def _on_path(self, msg: Path) -> None:
        self.path_seq += 1
        # New path means a new goal: a reached verdict from the previous
        # waypoint must not keep recovery muted.
        if self.goal_status in ('POSITION_REACHED', 'GOAL_REACHED'):
            self.goal_status = 'NOT_REACHED'
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

    def _tick(self) -> None:
        now = time.time()
        if self.state != RecoveryState.IDLE or (now - self._last_state_pub) >= 0.5:
            self._publish_state()
            self._last_state_pub = now
        if not self.navigating:
            return

        # FAIL is not permanent: once the front has been clear long enough the
        # obstacle is gone, so re-arm for whatever we hit next.
        if self.state == RecoveryState.FAIL:
            if (self._clear_since is not None
                    and (now - self._clear_since) >= self.fail_reset_clear_sec):
                self.attempts = 0
                self._set_state(
                    RecoveryState.IDLE,
                    'Front clear — recovery re-armed after previous failure')
            return

        # Level-triggered start: recovery begins when the front has been blocked
        # continuously for block_trigger_sec while IDLE, regardless of whether
        # we ever saw a false->true edge.
        if (self.state == RecoveryState.IDLE
                and not self._active()
                and self.goal_status not in ('POSITION_REACHED', 'GOAL_REACHED')
                and self._blocked_since is not None
                and (now - self._blocked_since) >= self.block_trigger_sec):
            self._start_recovery('Front blocked (level trigger)')
            return

        if self.state == RecoveryState.BACKUP:
            if self._backup_start_xy is None and self._pose_xy is not None:
                self._backup_start_xy = self._pose_xy
            traveled = self._backup_traveled()
            # Live rear guard: the backup distance was computed once at trigger
            # time, but the rear grid keeps updating. Stop the moment the rear
            # closes to the safety margin instead of reversing into it.
            rear_now = float(self.clearance.get('rear', 1.0))
            if rear_now <= self.backup_margin:
                self._stop_cmd()
                self._log_decision(
                    f'Rear closed to {rear_now:.2f} m — stop backup, facing open side')
                self._begin_spin()
                return
            if traveled >= self._backup_target_m:
                self._stop_cmd()
                self._log_decision(
                    f'Backup done ({traveled:.2f} m) — facing open side')
                self._begin_spin()
                return
            if now - self.phase_started >= self.backup_timeout:
                self._stop_cmd()
                self._log_decision(
                    f'Backup timeout after {traveled:.2f} m — facing open side')
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

            # Stop when COMPLETE front (0° + FL + FR) is in open space.
            if self._complete_front_open() and turned >= 0.12:
                self._stop_cmd()
                cl = self.clearance
                self._begin_wait_for_edt(
                    f'Complete front open after {math.degrees(turned):.0f}° '
                    f'(F={cl.get("front", 0):.2f} FL={cl.get("front_left", 0):.2f} '
                    f'FR={cl.get("front_right", 0):.2f}) — creating local EDT')
                return
            if turned >= self._spin_target:
                self._stop_cmd()
                self._begin_wait_for_edt(
                    f'Faced {self._spin_face_name} '
                    f'({math.degrees(turned):.0f}°) — creating local EDT')
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
