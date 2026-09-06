"""Shared helpers for align / detect+align nodes."""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def pose_cov_to_stamped(msg: PoseWithCovarianceStamped) -> PoseStamped:
    """Convert AMCL /amcl_pose (PoseWithCovarianceStamped) to PoseStamped."""
    out = PoseStamped()
    out.header = msg.header
    out.pose = msg.pose.pose
    return out


def world_to_grid(x: float, y: float, origin_x: float, origin_y: float,
                  resolution: float, height: int = 0) -> Tuple[int, int]:
    gx = int((x - origin_x) / resolution)
    gy = int((y - origin_y) / resolution)
    return gx, gy


def grid_to_world(gx: int, gy: int, origin_x: float, origin_y: float,
                  resolution: float) -> Tuple[float, float]:
    x = origin_x + (gx + 0.5) * resolution
    y = origin_y + (gy + 0.5) * resolution
    return x, y


def compute_edt(occupancy: np.ndarray, resolution: float,
                occupied_threshold: int = 50) -> np.ndarray:
    """Two-pass approximate EDT. Returns clearance in meters for each cell."""
    occ = np.asarray(occupancy, dtype=np.int16)
    h, w = occ.shape
    inf = 1e6
    dist = np.full((h, w), inf, dtype=np.float64)
    occupied = occ >= occupied_threshold
    dist[occupied] = 0.0

    for y in range(h):
        for x in range(w):
            if occupied[y, x]:
                continue
            if x > 0:
                dist[y, x] = min(dist[y, x], dist[y, x - 1] + 1.0)
            if y > 0:
                dist[y, x] = min(dist[y, x], dist[y - 1, x] + 1.0)
            if x > 0 and y > 0:
                dist[y, x] = min(dist[y, x], dist[y - 1, x - 1] + 1.41421356237)
            if x + 1 < w and y > 0:
                dist[y, x] = min(dist[y, x], dist[y - 1, x + 1] + 1.41421356237)

    for y in range(h - 1, -1, -1):
        for x in range(w - 1, -1, -1):
            if occupied[y, x]:
                continue
            if x + 1 < w:
                dist[y, x] = min(dist[y, x], dist[y, x + 1] + 1.0)
            if y + 1 < h:
                dist[y, x] = min(dist[y, x], dist[y + 1, x] + 1.0)
            if x + 1 < w and y + 1 < h:
                dist[y, x] = min(dist[y, x], dist[y + 1, x + 1] + 1.41421356237)
            if x > 0 and y + 1 < h:
                dist[y, x] = min(dist[y, x], dist[y + 1, x - 1] + 1.41421356237)

    dist[occ < 0] = 0.0
    dist[dist >= 1e6] = 0.0
    return dist * float(resolution)
