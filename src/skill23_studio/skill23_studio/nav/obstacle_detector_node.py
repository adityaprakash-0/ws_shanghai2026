#!/usr/bin/env python3
"""Live local occupancy grid from /scan + on-demand Euclidean distance field.

Local grid values:
  100  occupied (lidar hit)
    0  free (ray cleared)
   -1  unknown
  -99  inflation ring around occupied (lethal for footprint / EDT)

Params:
  local_grid_width / height / resolution
  inflation_radius_m  — grow obstacles by this radius (e.g. 0.02 = 2 cm)

Always: /custom_nav/local_grid  (live, every scan)

On Bool /custom_nav/local_edt/request:
  true  → compute ONE EDT snapshot from current grid, publish once, freeze
          (no refresh on later scans — deterministic for planner)
  false → clear temporary EDT, active=false
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from skill23_studio.nav.activity import ActivityGate
from skill23_studio.nav.node_status import NodeStatusPub
from skill23_studio.nav.utils import compute_edt

# Special cell value: inflation lethal zone (int8-safe)
INFLATION_VALUE = -99


def inflate_obstacles(grid: np.ndarray, radius_m: float, resolution: float) -> np.ndarray:
    """Dilate occupied (100) by radius; mark new cells as INFLATION_VALUE (-99)."""
    if radius_m <= 0.0 or resolution <= 0.0:
        return grid
    r_cells = int(math.ceil(radius_m / resolution))
    if r_cells <= 0:
        return grid

    occ = (grid == 100)
    if not np.any(occ):
        return grid
    inf = np.zeros_like(occ)
    r2 = r_cells * r_cells
    h, w = grid.shape
    for dy in range(-r_cells, r_cells + 1):
        for dx in range(-r_cells, r_cells + 1):
            if dx * dx + dy * dy > r2:
                continue
            y0, y1 = max(0, dy), min(h, h + dy)
            x0, x1 = max(0, dx), min(w, w + dx)
            sy0, sy1 = max(0, -dy), min(h, h - dy)
            sx0, sx1 = max(0, -dx), min(w, w - dx)
            if y1 <= y0 or x1 <= x0:
                continue
            inf[y0:y1, x0:x1] |= occ[sy0:sy1, sx0:sx1]
    out = grid.copy()
    out[inf & (out != 100)] = INFLATION_VALUE
    return out


class ObstacleDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('cn_obstacle_detector')
        self.declare_parameter('local_grid_width', 3.0)
        self.declare_parameter('local_grid_height', 3.0)
        self.declare_parameter('local_grid_resolution', 0.05)
        self.declare_parameter('inflation_radius_m', 0.02)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('occupied_thresh', 50)
        self.declare_parameter('scan_max_beams', 180)
        self.declare_parameter('publish_hz', 10.0)

        self.width_m = float(self.get_parameter('local_grid_width').value)
        self.height_m = float(self.get_parameter('local_grid_height').value)
        self.resolution = float(self.get_parameter('local_grid_resolution').value)
        self.inflation_r = float(self.get_parameter('inflation_radius_m').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.occupied_thresh = int(self.get_parameter('occupied_thresh').value)
        self.scan_max_beams = max(36, int(self.get_parameter('scan_max_beams').value))
        self.declare_parameter('idle_publish_hz', 2.0)
        self.gate = ActivityGate(
            self,
            active_hz=float(self.get_parameter('publish_hz').value),
            idle_hz=float(self.get_parameter('idle_publish_hz').value))

        self.size_x = max(1, int(round(self.width_m / self.resolution)))
        self.size_y = max(1, int(round(self.height_m / self.resolution)))
        self.origin_x = -0.5 * self.size_x * self.resolution
        self.origin_y = -0.5 * self.size_y * self.resolution

        # Latest occupancy: -1 unknown, 0 free, 100 occ, -99 inflation
        self._occ: Optional[np.ndarray] = None
        self._frame_id = self.base_frame
        self._edt_active = False
        self._has_scan = False
        self.status = NodeStatusPub(self, 'obstacle_detector')

        self.grid_pub = self.create_publisher(OccupancyGrid, '/custom_nav/local_grid', 10)
        self.edt_pub = self.create_publisher(OccupancyGrid, '/custom_nav/local_edt', 10)
        self.edt_active_pub = self.create_publisher(Bool, '/custom_nav/local_edt/active', 10)

        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            Bool, '/custom_nav/local_edt/request', self._on_edt_request, 10)

        self._publish_edt_active(False)
        self.status.set('waiting', 'waiting for /scan')
        self.get_logger().info(
            f'Local grid {self.width_m:.2f}×{self.height_m:.2f} m → '
            f'{self.size_x}×{self.size_y} @ {self.resolution:.3f} m/cell | '
            f'inflation={self.inflation_r:.3f} m (-99)')

    def _world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

    def _in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.size_x and 0 <= gy < self.size_y

    def _on_scan(self, msg: LaserScan) -> None:
        if not self.gate.due():
            return

        n = len(msg.ranges)
        if n <= 0:
            self.status.set('failed', 'empty laser scan')
            return
        step = max(1, n // self.scan_max_beams)
        grid = np.full((self.size_y, self.size_x), -1, dtype=np.int8)
        cx, cy = self._world_to_grid(0.0, 0.0)
        if not self._in_bounds(cx, cy):
            return

        # Clip rays to the local grid so we never walk 10 m of empty space.
        max_r = math.hypot(self.width_m, self.height_m) * 0.55
        rmin = float(msg.range_min)
        rmax = min(float(msg.range_max), max_r)
        angle0 = float(msg.angle_min)
        dang = float(msg.angle_increment)

        # Vectorized raycast: a per-beam Bresenham loop in Python was the
        # single largest CPU consumer in the nav stack on the Pi.
        idx = np.arange(0, n, step, dtype=np.int32)
        ranges = np.asarray(msg.ranges, dtype=np.float32)[idx]
        angles = (angle0 + idx.astype(np.float32) * dang).astype(np.float32)

        finite = np.isfinite(ranges)
        beyond = finite & (ranges > rmax)
        hit = finite & (ranges > rmin) & (ranges <= rmax)
        keep = beyond | hit
        if not np.any(keep):
            self.status.set('failed', 'no usable laser returns')
            return

        ranges = ranges[keep]
        angles = angles[keep]
        hit = hit[keep]
        ray_r = np.where(ranges > rmax, np.float32(rmax), ranges)
        cos_a = np.cos(angles)
        sin_a = np.sin(angles)

        # Sample every ray at sub-cell spacing — dense enough to leave no gaps.
        # Bound the sample count by the longest ray actually seen, not by the
        # window diagonal, so short indoor returns stay cheap.
        span = max(self.resolution * 0.5, 1e-3)
        steps = int(math.ceil(float(ray_r.max()) / span)) + 1
        along = np.minimum(
            np.arange(steps, dtype=np.float32)[None, :] * span,
            ray_r[:, None])
        gx = ((along * cos_a[:, None] - self.origin_x)
              / self.resolution).astype(np.int32)
        gy = ((along * sin_a[:, None] - self.origin_y)
              / self.resolution).astype(np.int32)
        free = ((gx >= 0) & (gx < self.size_x)
                & (gy >= 0) & (gy < self.size_y))
        grid[gy[free], gx[free]] = 0

        # Endpoints last so an obstacle always wins over another ray's free space.
        hx = ((ray_r * cos_a - self.origin_x) / self.resolution).astype(np.int32)
        hy = ((ray_r * sin_a - self.origin_y) / self.resolution).astype(np.int32)
        marks = (hit & (hx >= 0) & (hx < self.size_x)
                 & (hy >= 0) & (hy < self.size_y))
        grid[hy[marks], hx[marks]] = 100

        grid = inflate_obstacles(grid, self.inflation_r, self.resolution)
        self._occ = grid
        self._frame_id = msg.header.frame_id if msg.header.frame_id else self.base_frame
        self._publish_occupancy(grid)
        if not self._has_scan:
            self._has_scan = True
            self.status.set(
                'running',
                f'local grid {self.size_x}x{self.size_y}; beams={n}')

    def _publish_occupancy(self, grid: np.ndarray) -> None:
        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._frame_id
        out.info.resolution = self.resolution
        out.info.width = self.size_x
        out.info.height = self.size_y
        out.info.origin.position.x = self.origin_x
        out.info.origin.position.y = self.origin_y
        out.info.origin.orientation.w = 1.0
        out.data = grid.ravel().tolist()
        self.grid_pub.publish(out)

    def _on_edt_request(self, msg: Bool) -> None:
        if msg.data:
            if self._occ is None:
                self.get_logger().warn('EDT requested but no local grid yet')
                self._publish_edt_active(False)
                return
            # One-shot snapshot from current grid, then freeze until cleared
            self._edt_active = True
            self._compute_and_publish_edt()
            self._publish_edt_active(True)
            self.get_logger().info('Local EDT snapshot frozen (recovery)')
        else:
            self._clear_edt()
            self.get_logger().info('Local EDT cleared')

    def _occ_for_edt(self) -> np.ndarray:
        """Copy grid with inflation (-99) mapped to occupied so EDT seeds correctly."""
        assert self._occ is not None
        occ = self._occ.astype(np.int16).copy()
        occ[occ == INFLATION_VALUE] = 100
        return occ

    def _compute_and_publish_edt(self) -> None:
        assert self._occ is not None
        occ_edt = self._occ_for_edt()
        edt_m = compute_edt(occ_edt, self.resolution, self.occupied_thresh)
        res = self.resolution
        data = []
        for y in range(self.size_y):
            for x in range(self.size_x):
                v = int(self._occ[y, x])
                if v == INFLATION_VALUE or v >= self.occupied_thresh:
                    # Lethal for planner (inflation + hits)
                    data.append(100)
                elif v < 0:
                    data.append(-1)
                else:
                    cells = int(edt_m[y, x] / res) if res > 0 else 0
                    data.append(min(100, max(0, cells)))

        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._frame_id
        out.info.resolution = self.resolution
        out.info.width = self.size_x
        out.info.height = self.size_y
        out.info.origin.position.x = self.origin_x
        out.info.origin.position.y = self.origin_y
        out.info.origin.orientation.w = 1.0
        out.data = data
        self.edt_pub.publish(out)

    def _clear_edt(self) -> None:
        self._edt_active = False
        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._frame_id
        out.info.resolution = self.resolution
        out.info.width = 0
        out.info.height = 0
        out.info.origin.orientation.w = 1.0
        out.data = []
        self.edt_pub.publish(out)
        self._publish_edt_active(False)

    def _publish_edt_active(self, active: bool) -> None:
        msg = Bool()
        msg.data = bool(active)
        self.edt_active_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
