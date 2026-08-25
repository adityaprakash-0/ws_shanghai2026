#!/usr/bin/env python3
"""Angular-sector clearance on /custom_nav/local_grid.

Occupied cells only (100). Inflation (-99) ignored for distance.

COLLISION decision uses FRONT only (0° ± window):
  near_m (0.20) → near
  stop_m (0.12) → blocked  → recovery may start

Other sectors (± window) are for RECOVERY (face open side / backup):
  front_left   +45°
  front_right  -45°
  left         +90°
  right        -90°
  rear_left   +135°
  rear_right  -135°
  rear         full rear hemisphere min → backup distance
"""

from __future__ import annotations

import json
import math
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String

from skill23_studio.nav.node_status import NodeStatusPub

OCCUPIED_VALUE = 100

# name -> center angle (rad); rear hemisphere handled separately
BEARINGS = {
    'front': 0.0,
    'front_left': math.radians(45.0),
    'front_right': math.radians(-45.0),
    'left': math.radians(90.0),
    'right': math.radians(-90.0),
    'rear_left': math.radians(135.0),
    'rear_right': math.radians(-135.0),
}


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _ray_exit_rect(half_l: float, half_w: float, angle: float) -> float:
    c = math.cos(angle)
    s = math.sin(angle)
    cands: List[float] = []
    if abs(c) > 1e-9:
        t = half_l / c if c > 0.0 else -half_l / c
        if t > 0.0 and abs(t * s) <= half_w + 1e-6:
            cands.append(t)
    if abs(s) > 1e-9:
        t = half_w / s if s > 0.0 else -half_w / s
        if t > 0.0 and abs(t * c) <= half_l + 1e-6:
            cands.append(t)
    return min(cands) if cands else 0.0


class CollisionCheckerNode(Node):
    def __init__(self) -> None:
        super().__init__('cn_collision_checker')
        self.declare_parameter('robot_length', 0.40)
        self.declare_parameter('robot_width', 0.40)
        self.declare_parameter('safety_margin_m', 0.06)
        self.declare_parameter('near_m', 0.20)
        self.declare_parameter('stop_m', 0.12)
        self.declare_parameter('clearance_probe_m', 0.80)
        self.declare_parameter('window_deg', 5.0)
        self.declare_parameter('ray_step_deg', 2.5)
        self.declare_parameter('block_frames', 1)
        self.declare_parameter('clear_frames', 2)
        self.declare_parameter('collision_zone_depth', 0.20)  # yaml compat

        self.robot_length = float(self.get_parameter('robot_length').value)
        self.robot_width = float(self.get_parameter('robot_width').value)
        self.margin = float(self.get_parameter('safety_margin_m').value)
        self.near_m = float(self.get_parameter('near_m').value)
        self.stop_m = float(self.get_parameter('stop_m').value)
        self.probe_m = float(self.get_parameter('clearance_probe_m').value)
        self.window = math.radians(float(self.get_parameter('window_deg').value))
        self.ray_step = math.radians(float(self.get_parameter('ray_step_deg').value))
        self.block_frames = max(1, int(self.get_parameter('block_frames').value))
        self.clear_frames = max(1, int(self.get_parameter('clear_frames').value))

        self.buf_l = 0.5 * self.robot_length + self.margin
        self.buf_w = 0.5 * self.robot_width + self.margin

        self._was_blocked = False
        self._block_count = 0
        self._clear_count = 0
        self._last_ui_state = ''
        self.status = NodeStatusPub(self, 'collision')

        self.collision_pub = self.create_publisher(String, '/custom_nav/collision', 10)
        self.create_subscription(
            OccupancyGrid, '/custom_nav/local_grid', self._on_grid, 10)
        self.status.set(
            'idle',
            f'front 0°±{math.degrees(self.window):.0f}° · '
            f'stop≤{self.stop_m:.2f} · near≤{self.near_m:.2f}')

    def _cell_occupied(self, data: np.ndarray, gx: int, gy: int, w: int, h: int) -> bool:
        if gx < 0 or gy < 0 or gx >= w or gy >= h:
            return False
        return int(data[gy, gx]) == OCCUPIED_VALUE

    def _world_to_grid(
        self, x: float, y: float, ox: float, oy: float, res: float, w: int, h: int,
    ) -> Optional[Tuple[int, int]]:
        gx = int(math.floor((x - ox) / res))
        gy = int(math.floor((y - oy) / res))
        if gx < 0 or gy < 0 or gx >= w or gy >= h:
            return None
        return gx, gy

    def _ray_clearance(
        self,
        data: np.ndarray,
        ox: float,
        oy: float,
        res: float,
        w: int,
        h: int,
        angle: float,
    ) -> float:
        """Clearance beyond buffered footprint along one ray to occupied (100)."""
        step = max(res * 0.5, 0.01)
        t0 = _ray_exit_rect(self.buf_l, self.buf_w, angle)
        d = 0.0
        while d <= self.probe_m:
            d += step
            t = t0 + d
            x = t * math.cos(angle)
            y = t * math.sin(angle)
            g = self._world_to_grid(x, y, ox, oy, res, w, h)
            if g is None:
                return round(d, 3)
            if self._cell_occupied(data, g[0], g[1], w, h):
                return round(max(0.0, d - step), 3)
        return round(self.probe_m, 3)

    def _sector_min(
        self,
        data: np.ndarray,
        ox: float,
        oy: float,
        res: float,
        w: int,
        h: int,
        center: float,
        half_win: float,
    ) -> float:
        best = self.probe_m
        a = center - half_win
        while a <= center + half_win + 1e-9:
            best = min(best, self._ray_clearance(data, ox, oy, res, w, h, _wrap_pi(a)))
            a += max(self.ray_step, 1e-3)
        return round(best, 3)

    def _rear_min(
        self,
        data: np.ndarray,
        ox: float,
        oy: float,
        res: float,
        w: int,
        h: int,
    ) -> float:
        """Min clearance over the full rear hemisphere (|angle| ≥ 90°)."""
        best = self.probe_m
        a = -math.pi
        while a <= -math.pi / 2.0 + 1e-9:
            best = min(best, self._ray_clearance(data, ox, oy, res, w, h, a))
            a += max(self.ray_step, 1e-3)
        a = math.pi / 2.0
        while a <= math.pi + 1e-9:
            best = min(best, self._ray_clearance(data, ox, oy, res, w, h, _wrap_pi(a)))
            a += max(self.ray_step, 1e-3)
        return round(best, 3)

    def _best_open_name(self, clearance: Dict[str, float]) -> str:
        """Best non-front sector for recovery face hint."""
        best_name = 'left'
        best_d = -1.0
        for name in (
            'front_left', 'front_right', 'left', 'right',
            'rear_left', 'rear_right',
        ):
            d = float(clearance.get(name, 0.0))
            if d > best_d:
                best_d = d
                best_name = name
        return best_name

    def _on_grid(self, msg: OccupancyGrid) -> None:
        if msg.info.width == 0 or msg.info.height == 0:
            return

        h = int(msg.info.height)
        w = int(msg.info.width)
        res = float(msg.info.resolution)
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)
        data = np.array(msg.data, dtype=np.int16).reshape(h, w)

        clearance: Dict[str, float] = {}
        for name, cen in BEARINGS.items():
            clearance[name] = self._sector_min(
                data, ox, oy, res, w, h, cen, self.window)
        clearance['rear'] = self._rear_min(data, ox, oy, res, w, h)

        # --- collision decision: FRONT ONLY ---
        clear_front = clearance['front']
        raw_blocked = clear_front <= self.stop_m
        near = (not raw_blocked) and clear_front <= self.near_m

        if raw_blocked:
            self._block_count += 1
            self._clear_count = 0
        else:
            self._clear_count += 1
            self._block_count = 0

        blocked = self._was_blocked
        if not self._was_blocked and self._block_count >= self.block_frames:
            blocked = True
        elif self._was_blocked and self._clear_count >= self.clear_frames:
            blocked = False

        open_side = self._best_open_name(clearance)

        if blocked:
            state = 'blocked'
            reason = (
                f'FRONT only stop — {clear_front:.2f} m ≤ {self.stop_m:.2f} m '
                f'(open {open_side})')
        elif near:
            state = 'near'
            reason = f'FRONT only near — {clear_front:.2f} m ≤ {self.near_m:.2f} m'
        else:
            state = 'clear'
            reason = f'FRONT only clear — {clear_front:.2f} m'

        payload = {
            'blocked': blocked,
            'near': bool(near or blocked),
            'state': state,
            'reason': reason,
            'trigger': 'front_only',
            'open_side_hint': open_side,
            'clearance': {
                'front': clearance['front'],
                'front_left': clearance['front_left'],
                'front_right': clearance['front_right'],
                'left': clearance['left'],
                'right': clearance['right'],
                'rear_left': clearance['rear_left'],
                'rear_right': clearance['rear_right'],
                'rear': clearance['rear'],
            },
            'window_deg': round(math.degrees(self.window), 1),
            'zone': 'angular_sectors',
            'near_m': round(self.near_m, 3),
            'stop_m': round(self.stop_m, 3),
            'footprint_m': [round(self.robot_length, 3), round(self.robot_width, 3)],
            'safety_margin_m': round(self.margin, 3),
        }
        out = String()
        out.data = json.dumps(payload)
        self.collision_pub.publish(out)

        if blocked != self._was_blocked:
            self._was_blocked = blocked
            self._last_ui_state = state
            self.status.set(state, reason)
        elif state != self._last_ui_state:
            self._last_ui_state = state
            self.status.set(state, reason)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CollisionCheckerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
