#!/usr/bin/env python3
"""Simple front-clearance collision sense for skill23_custom_nav.

Uses /custom_nav/local_grid (already inflated):
  100  occupied
  -99  inflation
    0  free
   -1  unknown

Blocking uses front clearance only (sides do not stop the robot).
Publishes F/L/R/rear clearances every update for recovery.

  near_m (default 0.20)  → state=near  (advisory)
  stop_m (default 0.12)  → state=blocked → recovery may start
"""

from __future__ import annotations

import json
import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String

from skill23_custom_nav.node_status import NodeStatusPub

INFLATION_VALUE = -99
OCCUPIED_VALUE = 100


class CollisionCheckerNode(Node):
    def __init__(self) -> None:
        super().__init__('cn_collision_checker')
        self.declare_parameter('robot_length', 0.40)
        self.declare_parameter('robot_width', 0.40)
        self.declare_parameter('safety_margin_m', 0.06)
        self.declare_parameter('near_m', 0.20)
        self.declare_parameter('stop_m', 0.12)
        self.declare_parameter('clearance_probe_m', 0.80)
        self.declare_parameter('block_frames', 1)
        self.declare_parameter('clear_frames', 2)
        # Kept for yaml compatibility; not used for stop (front clearance only).
        self.declare_parameter('collision_zone_depth', 0.20)

        self.robot_length = float(self.get_parameter('robot_length').value)
        self.robot_width = float(self.get_parameter('robot_width').value)
        self.margin = float(self.get_parameter('safety_margin_m').value)
        self.near_m = float(self.get_parameter('near_m').value)
        self.stop_m = float(self.get_parameter('stop_m').value)
        self.probe_m = float(self.get_parameter('clearance_probe_m').value)
        self.block_frames = max(1, int(self.get_parameter('block_frames').value))
        self.clear_frames = max(1, int(self.get_parameter('clear_frames').value))

        self.half_l = 0.5 * self.robot_length
        self.half_w = 0.5 * self.robot_width
        self.buf_l = self.half_l + self.margin
        self.buf_w = self.half_w + self.margin

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
            f'front clearance only · stop≤{self.stop_m:.2f} · near≤{self.near_m:.2f}')

    def _cell_lethal(self, data: np.ndarray, gx: int, gy: int, w: int, h: int) -> bool:
        if gx < 0 or gy < 0 or gx >= w or gy >= h:
            return False
        v = int(data[gy, gx])
        return v == OCCUPIED_VALUE or v == INFLATION_VALUE

    def _world_to_grid(
        self, x: float, y: float, ox: float, oy: float, res: float, w: int, h: int,
    ) -> Optional[Tuple[int, int]]:
        gx = int(math.floor((x - ox) / res))
        gy = int(math.floor((y - oy) / res))
        if gx < 0 or gy < 0 or gx >= w or gy >= h:
            return None
        return gx, gy

    def _clearance(
        self,
        data: np.ndarray,
        ox: float,
        oy: float,
        res: float,
        w: int,
        h: int,
        direction: str,
    ) -> float:
        """Free distance from buffered footprint edge to first lethal cell."""
        step = max(res * 0.5, 0.01)
        max_d = self.probe_m
        if direction == 'front':
            x0, y0, dx, dy = self.buf_l, 0.0, 1.0, 0.0
            half_span = self.buf_w * 0.55
        elif direction == 'rear':
            x0, y0, dx, dy = -self.buf_l, 0.0, -1.0, 0.0
            half_span = self.buf_w * 0.55
        elif direction == 'left':
            x0, y0, dx, dy = 0.0, self.buf_w, 0.0, 1.0
            half_span = self.buf_l * 0.55
        else:
            x0, y0, dx, dy = 0.0, -self.buf_w, 0.0, -1.0
            half_span = self.buf_l * 0.55

        d = 0.0
        while d <= max_d:
            d += step
            px = x0 + dx * d
            py = y0 + dy * d
            for offset in (-half_span, 0.0, half_span):
                if direction in ('front', 'rear'):
                    sx, sy = px, py + offset
                else:
                    sx, sy = px + offset, py
                g = self._world_to_grid(sx, sy, ox, oy, res, w, h)
                if g is None:
                    return round(d, 3)
                if self._cell_lethal(data, g[0], g[1], w, h):
                    return round(max(0.0, d - step), 3)
        return round(max_d, 3)

    def _on_grid(self, msg: OccupancyGrid) -> None:
        if msg.info.width == 0 or msg.info.height == 0:
            return

        h = int(msg.info.height)
        w = int(msg.info.width)
        res = float(msg.info.resolution)
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)
        data = np.array(msg.data, dtype=np.int16).reshape(h, w)

        clear_front = self._clearance(data, ox, oy, res, w, h, 'front')
        clear_rear = self._clearance(data, ox, oy, res, w, h, 'rear')
        clear_left = self._clearance(data, ox, oy, res, w, h, 'left')
        clear_right = self._clearance(data, ox, oy, res, w, h, 'right')

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

        if blocked:
            state = 'blocked'
            reason = f'front stop — clear {clear_front:.2f} m ≤ {self.stop_m:.2f} m'
        elif near:
            state = 'near'
            reason = f'front near — clear {clear_front:.2f} m ≤ {self.near_m:.2f} m'
        else:
            state = 'clear'
            reason = f'front clear — {clear_front:.2f} m'

        payload = {
            'blocked': blocked,
            'near': bool(near or blocked),
            'state': state,
            'reason': reason,
            'clearance': {
                'front': clear_front,
                'rear': clear_rear,
                'left': clear_left,
                'right': clear_right,
            },
            'zone': 'front_clearance',
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
