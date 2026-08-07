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

from skill23_custom_nav.utils import compute_edt

# Special cell value: inflation lethal zone (int8-safe)
INFLATION_VALUE = -99


def _bresenham(x0: int, y0: int, x1: int, y1: int):
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        yield x, y
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy


def inflate_obstacles(grid: np.ndarray, radius_m: float, resolution: float) -> np.ndarray:
    """Dilate occupied (100) by radius; mark new cells as INFLATION_VALUE (-99)."""
    if radius_m <= 0.0 or resolution <= 0.0:
        return grid
    r_cells = int(math.ceil(radius_m / resolution))
    if r_cells <= 0:
        return grid

    out = grid.copy()
    h, w = grid.shape
    ys, xs = np.where(grid == 100)
    if ys.size == 0:
        return out

    r2 = r_cells * r_cells
    for y, x in zip(ys.tolist(), xs.tolist()):
        y0 = max(0, y - r_cells)
        y1 = min(h, y + r_cells + 1)
        x0 = max(0, x - r_cells)
        x1 = min(w, x + r_cells + 1)
        for ny in range(y0, y1):
            dy = ny - y
            for nx in range(x0, x1):
                dx = nx - x
                if dx * dx + dy * dy > r2:
                    continue
                if out[ny, nx] != 100:
                    out[ny, nx] = INFLATION_VALUE
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

        self.width_m = float(self.get_parameter('local_grid_width').value)
        self.height_m = float(self.get_parameter('local_grid_height').value)
        self.resolution = float(self.get_parameter('local_grid_resolution').value)
        self.inflation_r = float(self.get_parameter('inflation_radius_m').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.occupied_thresh = int(self.get_parameter('occupied_thresh').value)

        self.size_x = max(1, int(round(self.width_m / self.resolution)))
        self.size_y = max(1, int(round(self.height_m / self.resolution)))
        self.origin_x = -0.5 * self.size_x * self.resolution
        self.origin_y = -0.5 * self.size_y * self.resolution

        # Latest occupancy: -1 unknown, 0 free, 100 occ, -99 inflation
        self._occ: Optional[np.ndarray] = None
        self._frame_id = self.base_frame
        self._edt_active = False

        self.grid_pub = self.create_publisher(OccupancyGrid, '/custom_nav/local_grid', 10)
        self.edt_pub = self.create_publisher(OccupancyGrid, '/custom_nav/local_edt', 10)
        self.edt_active_pub = self.create_publisher(Bool, '/custom_nav/local_edt/active', 10)

        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            Bool, '/custom_nav/local_edt/request', self._on_edt_request, 10)

        self._publish_edt_active(False)
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
        grid = np.full((self.size_y, self.size_x), -1, dtype=np.int8)
        cx, cy = self._world_to_grid(0.0, 0.0)

        angle = float(msg.angle_min)
        for r in msg.ranges:
            a = angle
            angle += msg.angle_increment

            if math.isfinite(r) and msg.range_min < r <= msg.range_max:
                hit = True
                ray_r = float(r)
            elif math.isfinite(r) and r > msg.range_max:
                hit = False
                ray_r = float(msg.range_max)
            else:
                continue

            ex = ray_r * math.cos(a)
            ey = ray_r * math.sin(a)
            gx1, gy1 = self._world_to_grid(ex, ey)

            if not self._in_bounds(gx1, gy1):
                steps = max(1, int(ray_r / self.resolution))
                last = None
                for s in range(1, steps + 1):
                    t = s / steps
                    gx, gy = self._world_to_grid(t * ex, t * ey)
                    if self._in_bounds(gx, gy):
                        last = (gx, gy)
                    else:
                        break
                if last is None:
                    continue
                gx1, gy1 = last
                hit = False

            if not self._in_bounds(cx, cy):
                continue

            cells = list(_bresenham(cx, cy, gx1, gy1))
            if not cells:
                continue

            end = cells[-1]
            free_cells = cells[:-1] if hit else cells
            for gx, gy in free_cells:
                if self._in_bounds(gx, gy) and grid[gy, gx] != 100:
                    grid[gy, gx] = 0

            if hit and self._in_bounds(end[0], end[1]):
                grid[end[1], end[0]] = 100

        grid = inflate_obstacles(grid, self.inflation_r, self.resolution)
        self._occ = grid
        self._frame_id = msg.header.frame_id if msg.header.frame_id else self.base_frame
        self._publish_occupancy(grid)
        # Local EDT is frozen after request — do NOT refresh here while active.

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
        out.data = grid.astype(np.int8).flatten().tolist()
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
