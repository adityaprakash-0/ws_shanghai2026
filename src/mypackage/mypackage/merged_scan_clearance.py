#!/usr/bin/env python3
"""Directional clearances from merged /scan at fixed bearings.

For each bearing, take beams in ±window_deg (default ±5°) and report the
minimum range. Distances are reported beyond the rectangular footprint edge
(bumper-like), not from the robot center.

Bearings (base_footprint / LaserScan convention):
  front        0°
  front_left  +45°
  front_right -45°
  left        +90°
  right       -90°
  rear       ±180°
"""

from __future__ import annotations

import json
import math
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


# name -> center angle (rad)
BEARINGS = {
    'front': 0.0,
    'front_left': math.radians(45.0),
    'front_right': math.radians(-45.0),
    'left': math.radians(90.0),
    'right': math.radians(-90.0),
    'rear': math.pi,
}

# Log / JSON order
ORDER = ('front', 'front_left', 'front_right', 'left', 'right', 'rear')


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _in_window(angle: float, center: float, half_win: float) -> bool:
    return abs(_wrap_pi(angle - center)) <= half_win + 1e-9


def _ray_exit_rect(half_l: float, half_w: float, angle: float) -> float:
    """Distance from origin to rectangle boundary along a ray."""
    c = math.cos(angle)
    s = math.sin(angle)
    candidates = []
    if abs(c) > 1e-9:
        t = half_l / c if c > 0.0 else -half_l / c
        if t > 0.0:
            y = t * s
            if abs(y) <= half_w + 1e-6:
                candidates.append(t)
    if abs(s) > 1e-9:
        t = half_w / s if s > 0.0 else -half_w / s
        if t > 0.0:
            x = t * c
            if abs(x) <= half_l + 1e-6:
                candidates.append(t)
    return min(candidates) if candidates else 0.0


class MergedScanClearance(Node):
    def __init__(self) -> None:
        super().__init__('merged_scan_clearance')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('window_deg', 5.0)  # ±5°
        self.declare_parameter('range_min_override', 0.05)
        self.declare_parameter('robot_length', 0.44)
        self.declare_parameter('robot_width', 0.363)
        self.declare_parameter('log_hz', 2.0)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.half_win = math.radians(float(self.get_parameter('window_deg').value))
        self.half_l = 0.5 * float(self.get_parameter('robot_length').value)
        self.half_w = 0.5 * float(self.get_parameter('robot_width').value)
        self.range_min_override = float(self.get_parameter('range_min_override').value)
        log_hz = max(0.2, float(self.get_parameter('log_hz').value))

        self._edge: Dict[str, float] = {k: float('inf') for k in BEARINGS}
        self._center: Dict[str, float] = {k: float('inf') for k in BEARINGS}
        self._beams: Dict[str, int] = {k: 0 for k in BEARINGS}

        self.pub_json = self.create_publisher(String, '/merged_scan/clearance', 10)
        self.create_subscription(
            LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data)
        self.create_timer(1.0 / log_hz, self._log_tick)

        self.get_logger().info(
            f'Listening {self.scan_topic} | ±{math.degrees(self.half_win):.0f}° '
            f'around 0 / +45 / -45 / +90 / -90 / 180 | '
            f'min beyond footprint {2 * self.half_l:.2f}×{2 * self.half_w:.2f} m')

    def _on_scan(self, msg: LaserScan) -> None:
        edge = {k: float('inf') for k in BEARINGS}
        center = {k: float('inf') for k in BEARINGS}
        beams = {k: 0 for k in BEARINGS}

        rmin = max(float(msg.range_min), self.range_min_override)
        rmax = float(msg.range_max) if msg.range_max > 0.0 else 1e6
        angle = float(msg.angle_min)
        step = float(msg.angle_increment)

        for r in msg.ranges:
            a = _wrap_pi(angle)
            angle += step
            if not math.isfinite(r) or r < rmin or r > rmax:
                continue
            body = _ray_exit_rect(self.half_l, self.half_w, a)
            beyond = r - body
            if beyond < 0.0:
                continue
            for name, cen in BEARINGS.items():
                if not _in_window(a, cen, self.half_win):
                    continue
                beams[name] += 1
                if r < center[name]:
                    center[name] = r
                if beyond < edge[name]:
                    edge[name] = beyond

        self._edge = edge
        self._center = center
        self._beams = beams

        def fmt(v: float) -> Optional[float]:
            return None if not math.isfinite(v) else round(v, 3)

        payload = {
            'frame_id': msg.header.frame_id,
            'window_deg': round(math.degrees(self.half_win), 1),
            'beyond_body_m': {k: fmt(edge[k]) for k in ORDER},
            'from_center_m': {k: fmt(center[k]) for k in ORDER},
            'beams': {k: beams[k] for k in ORDER},
            'bearings_deg': {
                'front': 0,
                'front_left': 45,
                'front_right': -45,
                'left': 90,
                'right': -90,
                'rear': 180,
            },
            'note': (
                'beyond_body_m = min range in ±window around bearing, '
                'minus robot body along that ray (use this vs tape). '
                'from_center_m = raw /scan min (from origin).'
            ),
        }
        # Flat aliases for quick reading
        for k in ORDER:
            payload[f'{k}_m'] = fmt(edge[k])

        out = String()
        out.data = json.dumps(payload)
        self.pub_json.publish(out)

    def _log_tick(self) -> None:
        def s(v: float) -> str:
            return '  inf' if not math.isfinite(v) else f'{v:5.3f}'

        self.get_logger().info(
            'beyond-body (±{w:.0f}°)  '
            'F0={f}  FL+45={fl}  FR-45={fr}  L+90={l}  Ri-90={ri}  R180={r} m | '
            'beams {bf}/{bfl}/{bfr}/{bl}/{bri}/{br}'.format(
                w=math.degrees(self.half_win),
                f=s(self._edge['front']),
                fl=s(self._edge['front_left']),
                fr=s(self._edge['front_right']),
                l=s(self._edge['left']),
                ri=s(self._edge['right']),
                r=s(self._edge['rear']),
                bf=self._beams['front'],
                bfl=self._beams['front_left'],
                bfr=self._beams['front_right'],
                bl=self._beams['left'],
                bri=self._beams['right'],
                br=self._beams['rear'],
            )
        )


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = MergedScanClearance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
