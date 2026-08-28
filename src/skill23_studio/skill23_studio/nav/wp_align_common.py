"""Shared helpers for the waypoint align node (yaw and align+move).

/cmd_vel on this stack is Titan duty (−1…1), not SI velocity.
"""

from __future__ import annotations

import json
import math
from typing import Optional, Tuple

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String

from skill23_studio.nav.utils import wrap_angle, yaw_from_quat


def yaw_of_quat(x: float, y: float, z: float, w: float) -> float:
    return yaw_from_quat(x, y, z, w)


def bind_heading_sub(node, on_yaw) -> str:
    """Subscribe AMCL pose or /imu (NavX quaternion) and call on_yaw(rad)."""
    source = str(node.get_parameter('pose_source').value or 'amcl').strip().lower()
    topic = str(node.get_parameter('pose_topic').value or '').strip()
    if source == 'imu':
        if not topic:
            topic = '/imu'

        def _imu(msg: Imu) -> None:
            q = msg.orientation
            on_yaw(yaw_of_quat(q.x, q.y, q.z, q.w))

        node.create_subscription(Imu, topic, _imu, qos_profile_sensor_data)
        return topic
    if not topic:
        topic = '/amcl_pose'

    def _pose(msg: PoseWithCovarianceStamped) -> None:
        q = msg.pose.pose.orientation
        on_yaw(yaw_of_quat(q.x, q.y, q.z, q.w))

    node.create_subscription(PoseWithCovarianceStamped, topic, _pose, 10)
    return topic


class Pid:
    def __init__(self, kp: float, ki: float, kd: float, i_limit: float) -> None:
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.i_limit = abs(float(i_limit))
        self._i = 0.0
        self._prev: Optional[float] = None

    def reset(self) -> None:
        self._i = 0.0
        self._prev = None

    def update(self, err: float, dt: float) -> float:
        dt = max(1e-3, float(dt))
        self._i = max(-self.i_limit, min(self.i_limit, self._i + err * dt))
        deriv = 0.0 if self._prev is None else (err - self._prev) / dt
        self._prev = err
        return self.kp * err + self.ki * self._i + self.kd * deriv


def shape_duty(raw: float, umin: float, umax: float) -> float:
    """Clamp magnitude into [umin, umax] and keep sign. umin fights stiction."""
    if not math.isfinite(raw) or abs(raw) < 1e-9:
        return 0.0
    mag = max(float(umin), min(float(umax), abs(raw)))
    return math.copysign(mag, raw)


def twist_yaw(duty: float) -> Twist:
    tw = Twist()
    tw.angular.z = float(duty)
    return tw


def twist_lin(duty: float) -> Twist:
    tw = Twist()
    tw.linear.x = float(duty)
    return tw


def result_msg(cmd_id: str, ok: bool, mode: str, message: str, **extra) -> String:
    payload = {
        'cmd_id': cmd_id,
        'success': bool(ok),
        'mode': mode,
        'message': message,
    }
    payload.update(extra)
    msg = String()
    msg.data = json.dumps(payload)
    return msg


def range_at_angle(
        scan: LaserScan, angle_rad: float = 0.0,
        window_deg: float = 0.0) -> Tuple[Optional[float], float]:
    """Return (range_m, used_angle_rad) for the beam nearest ``angle_rad``.

    If that beam is inf/NaN and window_deg > 0, the nearest valid beam inside
    the window is used. Returns (None, used_angle) when nothing is valid.
    """
    n = len(scan.ranges)
    if n <= 0 or scan.angle_increment == 0.0:
        return None, angle_rad
    idx = int(round((angle_rad - scan.angle_min) / scan.angle_increment))
    idx = max(0, min(n - 1, idx))
    used = scan.angle_min + idx * scan.angle_increment
    val = float(scan.ranges[idx])
    rmin = float(scan.range_min) if scan.range_min > 0.0 else 0.02
    rmax = float(scan.range_max) if scan.range_max > 0.0 else 12.0

    def _ok(v: float) -> bool:
        return math.isfinite(v) and rmin <= v <= rmax

    if _ok(val):
        return val, used

    half = math.radians(max(0.0, float(window_deg)))
    if half <= 0.0:
        return None, used
    best: Optional[float] = None
    best_ang = used
    best_da = 1e9
    for i, raw in enumerate(scan.ranges):
        ang = scan.angle_min + i * scan.angle_increment
        da = abs(wrap_angle(ang - angle_rad))
        if da > half:
            continue
        v = float(raw)
        if _ok(v) and da < best_da:
            best = v
            best_ang = ang
            best_da = da
    return best, best_ang
