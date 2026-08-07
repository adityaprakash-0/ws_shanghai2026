"""Grid planning brain (no ROS).

Simple idea:
  1) A* on the map, preferring cells with high static EDT (farthest from walls)
  2) Snap each path cell toward higher EDT nearby
  3) String-pull into long straight segments that stay clear of walls
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from skill23_custom_nav.utils import grid_to_world, world_to_grid

NEIGHBORS = [
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, 1.41421356237), (-1, 1, 1.41421356237),
    (1, -1, 1.41421356237), (1, 1, 1.41421356237),
]

Cell = Tuple[int, int]
Pose2D = Tuple[float, float, float]


@dataclass
class LocalEdtInput:
    data: np.ndarray
    origin_x: float
    origin_y: float
    resolution: float
    robot_x: float
    robot_y: float
    robot_yaw: float


def compress_collinear(cells: List[Cell]) -> List[Cell]:
    if len(cells) <= 2:
        return list(cells)
    out = [cells[0]]
    for i in range(1, len(cells) - 1):
        x0, y0 = out[-1]
        x1, y1 = cells[i]
        x2, y2 = cells[i + 1]
        if (x1 - x0) * (y2 - y1) == (y1 - y0) * (x2 - x1):
            continue
        out.append((x1, y1))
    out.append(cells[-1])
    return out


def decode_edt_cells(
    data: Sequence[int],
    height: int,
    width: int,
    resolution: float,
    occupied_thresh: int = 50,
    occupancy: Optional[np.ndarray] = None,
) -> np.ndarray:
    arr = np.array(data, dtype=np.int16).reshape(height, width)
    edt = np.zeros((height, width), dtype=np.float64)
    res = float(resolution)
    if occupancy is not None and occupancy.shape == (height, width):
        free = (occupancy >= 0) & (occupancy < occupied_thresh) & (arr >= 0)
        edt[free] = np.minimum(arr[free].astype(np.float64), 100.0) * res
    else:
        free = (arr >= 0) & (arr < 100)
        edt[free] = arr[free].astype(np.float64) * res
    return edt


def fuse_local_edt(
    static_edt: np.ndarray,
    map_origin_x: float,
    map_origin_y: float,
    map_resolution: float,
    local: LocalEdtInput,
) -> Tuple[np.ndarray, np.ndarray]:
    edt = static_edt.copy()
    h, w = edt.shape
    blocked = np.zeros((h, w), dtype=bool)
    c = math.cos(local.robot_yaw)
    s = math.sin(local.robot_yaw)
    lh, lw = local.data.shape
    res_l = float(local.resolution)
    for gy in range(lh):
        for gx in range(lw):
            v = int(local.data[gy, gx])
            if v < 0:
                continue
            lx = local.origin_x + (gx + 0.5) * res_l
            ly = local.origin_y + (gy + 0.5) * res_l
            wx = local.robot_x + c * lx - s * ly
            wy = local.robot_y + s * lx + c * ly
            mx = int((wx - map_origin_x) / map_resolution)
            my = int((wy - map_origin_y) / map_resolution)
            if not (0 <= mx < w and 0 <= my < h):
                continue
            if v >= 100:
                blocked[my, mx] = True
                edt[my, mx] = 0.0
            else:
                edt[my, mx] = min(edt[my, mx], float(v) * res_l)
    return edt, blocked


def _free_cell(
    x: int, y: int,
    occupancy: np.ndarray,
    edt: np.ndarray,
    *,
    occupied_threshold: int,
    min_clearance_m: float,
    extra_blocked: Optional[np.ndarray],
) -> bool:
    h, w = occupancy.shape
    if not (0 <= x < w and 0 <= y < h):
        return False
    v = int(occupancy[y, x])
    if v < 0 or v >= occupied_threshold:
        return False
    if extra_blocked is not None and extra_blocked[y, x]:
        return False
    if float(edt[y, x]) < min_clearance_m:
        return False
    return True


def astar(
    start: Cell,
    goal: Cell,
    occupancy: np.ndarray,
    edt: np.ndarray,
    *,
    occupied_threshold: int = 50,
    clearance_weight: float = 12.0,
    min_clearance_m: float = 0.10,
    extra_blocked: Optional[np.ndarray] = None,
) -> Optional[List[Cell]]:
    """A*: each step prefers cells farther from walls (higher EDT)."""
    h, w = occupancy.shape
    sx, sy = start
    gx, gy = goal
    if not (0 <= sx < w and 0 <= sy < h and 0 <= gx < w and 0 <= gy < h):
        return None

    def is_blocked(x: int, y: int) -> bool:
        return not _free_cell(
            x, y, occupancy, edt,
            occupied_threshold=occupied_threshold,
            min_clearance_m=min_clearance_m,
            extra_blocked=extra_blocked,
        )

    if is_blocked(sx, sy) or is_blocked(gx, gy):
        return None

    def heuristic(x: int, y: int) -> float:
        return math.hypot(x - gx, y - gy)

    def step_cost(x: int, y: int, step: float) -> float:
        # Strongly prefer cells farthest from walls
        clearance = max(float(edt[y, x]), 0.02)
        return step * (clearance_weight / (clearance * clearance))

    open_heap: List[Tuple[float, int, int]] = []
    heapq.heappush(open_heap, (heuristic(sx, sy), sx, sy))
    came_from: dict = {}
    g_score = {(sx, sy): 0.0}
    closed = set()
    path: Optional[List[Cell]] = None

    while open_heap:
        _, cx, cy = heapq.heappop(open_heap)
        if (cx, cy) in closed:
            continue
        closed.add((cx, cy))
        if cx == gx and cy == gy:
            path = [(cx, cy)]
            while (cx, cy) in came_from:
                cx, cy = came_from[(cx, cy)]
                path.append((cx, cy))
            path.reverse()
            break

        for dx, dy, step in NEIGHBORS:
            nx, ny = cx + dx, cy + dy
            if nx < 0 or ny < 0 or nx >= w or ny >= h:
                continue
            if is_blocked(nx, ny):
                continue
            if dx != 0 and dy != 0:
                if is_blocked(cx + dx, cy) or is_blocked(cx, cy + dy):
                    continue
            ng = g_score[(cx, cy)] + step_cost(nx, ny, step)
            if ng < g_score.get((nx, ny), 1e18):
                g_score[(nx, ny)] = ng
                came_from[(nx, ny)] = (cx, cy)
                heapq.heappush(open_heap, (ng + heuristic(nx, ny), nx, ny))

    return path


def snap_to_edt_ridge(
    cells: List[Cell],
    occupancy: np.ndarray,
    edt: np.ndarray,
    *,
    occupied_threshold: int,
    min_clearance_m: float,
    extra_blocked: Optional[np.ndarray],
    radius_cells: int = 2,
) -> List[Cell]:
    """At each path pixel, pick the nearby free cell farthest from walls."""
    if len(cells) <= 2:
        return list(cells)
    out: List[Cell] = [cells[0]]
    for i in range(1, len(cells) - 1):
        cx, cy = cells[i]
        best = (cx, cy)
        best_c = float(edt[cy, cx])
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                nx, ny = cx + dx, cy + dy
                if not _free_cell(
                    nx, ny, occupancy, edt,
                    occupied_threshold=occupied_threshold,
                    min_clearance_m=min_clearance_m,
                    extra_blocked=extra_blocked,
                ):
                    continue
                c = float(edt[ny, nx])
                if c > best_c:
                    best_c = c
                    best = (nx, ny)
        out.append(best)
    out.append(cells[-1])
    return out


def line_clear(
    a: Cell,
    b: Cell,
    occupancy: np.ndarray,
    edt: np.ndarray,
    *,
    occupied_threshold: int,
    min_clearance_m: float,
    extra_blocked: Optional[np.ndarray],
    map_origin_x: float,
    map_origin_y: float,
    map_resolution: float,
    los_sample_step_m: float,
    required_clearance_m: Optional[float] = None,
) -> bool:
    """True if a→b stays free and every sample has enough EDT clearance.

    required_clearance_m (if set) stops corner-cutting: a shortcut must keep
    at least the clearance the original ridge path had.
    """
    need = float(min_clearance_m if required_clearance_m is None else required_clearance_m)
    ax, ay = grid_to_world(a[0], a[1], map_origin_x, map_origin_y, map_resolution)
    bx, by = grid_to_world(b[0], b[1], map_origin_x, map_origin_y, map_resolution)
    dist = math.hypot(bx - ax, by - ay)
    if dist < 1e-6:
        return True
    step = max(los_sample_step_m, map_resolution * 0.5)
    n = max(1, int(math.ceil(dist / step)))
    h, w = occupancy.shape
    for i in range(n + 1):
        t = i / n
        wx = ax + t * (bx - ax)
        wy = ay + t * (by - ay)
        gx, gy = world_to_grid(wx, wy, map_origin_x, map_origin_y, map_resolution)
        if not (0 <= gx < w and 0 <= gy < h):
            return False
        if not _free_cell(
            gx, gy, occupancy, edt,
            occupied_threshold=occupied_threshold,
            min_clearance_m=need,
            extra_blocked=extra_blocked,
        ):
            return False
    return True


def _min_edt_along(cells: List[Cell], i: int, j: int, edt: np.ndarray) -> float:
    m = 1e18
    for k in range(i, j + 1):
        cx, cy = cells[k]
        m = min(m, float(edt[cy, cx]))
    return 0.0 if m > 1e17 else m


def string_pull(
    cells: List[Cell],
    occupancy: np.ndarray,
    edt: np.ndarray,
    *,
    occupied_threshold: int,
    min_clearance_m: float,
    extra_blocked: Optional[np.ndarray],
    map_origin_x: float,
    map_origin_y: float,
    map_resolution: float,
    los_sample_step_m: float,
    clearance_keep: float = 0.90,
) -> List[Cell]:
    """Straight segments that do not drop below the ridge path's clearance.

    A shortcut i→j is allowed only if every point on the chord has EDT >=
    clearance_keep * min(EDT on the original path cells i..j). That blocks
    orange-style corner cuts and keeps black-style outward / center paths.
    """
    if len(cells) <= 2:
        return list(cells)
    cells = compress_collinear(cells)
    out = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        best = i + 1
        j = i + 2
        while j < len(cells):
            ridge_min = _min_edt_along(cells, i, j, edt)
            need = max(min_clearance_m, clearance_keep * ridge_min)
            if line_clear(
                cells[i], cells[j], occupancy, edt,
                occupied_threshold=occupied_threshold,
                min_clearance_m=min_clearance_m,
                extra_blocked=extra_blocked,
                map_origin_x=map_origin_x,
                map_origin_y=map_origin_y,
                map_resolution=map_resolution,
                los_sample_step_m=los_sample_step_m,
                required_clearance_m=need,
            ):
                best = j
                j += 1
            else:
                break
        out.append(cells[best])
        i = best
    return out


def cells_to_poses(
    cells: List[Cell],
    map_origin_x: float,
    map_origin_y: float,
    map_resolution: float,
    goal_yaw: float,
) -> List[Pose2D]:
    if not cells:
        return []
    world = [
        grid_to_world(cx, cy, map_origin_x, map_origin_y, map_resolution)
        for cx, cy in cells
    ]
    poses: List[Pose2D] = []
    for i, (wx, wy) in enumerate(world):
        if i < len(world) - 1:
            yaw = math.atan2(world[i + 1][1] - wy, world[i + 1][0] - wx)
        else:
            yaw = goal_yaw
        poses.append((wx, wy, yaw))
    return poses


def plan_path(
    start_x: float,
    start_y: float,
    goal_x: float,
    goal_y: float,
    goal_yaw: float,
    occupancy: np.ndarray,
    static_edt: np.ndarray,
    map_origin_x: float,
    map_origin_y: float,
    map_resolution: float,
    *,
    local: Optional[LocalEdtInput] = None,
    occupied_threshold: int = 50,
    clearance_weight: float = 12.0,
    min_clearance_m: float = 0.10,
    los_sample_step_m: float = 0.05,
) -> Optional[List[Pose2D]]:
    """Max-clearance A* → snap to EDT ridge → clearance-preserving straights."""
    edt = static_edt
    extra_blocked: Optional[np.ndarray] = None
    if local is not None:
        edt, extra_blocked = fuse_local_edt(
            static_edt, map_origin_x, map_origin_y, map_resolution, local)

    sx, sy = world_to_grid(
        start_x, start_y, map_origin_x, map_origin_y, map_resolution)
    gx, gy = world_to_grid(
        goal_x, goal_y, map_origin_x, map_origin_y, map_resolution)

    cells = astar(
        (sx, sy), (gx, gy), occupancy, edt,
        occupied_threshold=occupied_threshold,
        clearance_weight=clearance_weight,
        min_clearance_m=min_clearance_m,
        extra_blocked=extra_blocked,
    )
    if cells is None:
        return None

    # Push every cell onto the local farthest-from-wall spot (corridor/room center)
    cells = snap_to_edt_ridge(
        cells, occupancy, edt,
        occupied_threshold=occupied_threshold,
        min_clearance_m=min_clearance_m,
        extra_blocked=extra_blocked,
        radius_cells=4,
    )
    # Straight lines only where they stay as far from walls as the ridge path
    cells = string_pull(
        cells, occupancy, edt,
        occupied_threshold=occupied_threshold,
        min_clearance_m=min_clearance_m,
        extra_blocked=extra_blocked,
        map_origin_x=map_origin_x,
        map_origin_y=map_origin_y,
        map_resolution=map_resolution,
        los_sample_step_m=los_sample_step_m,
        clearance_keep=0.90,
    )

    poses = cells_to_poses(
        cells, map_origin_x, map_origin_y, map_resolution, goal_yaw)
    if not poses:
        return None

    # Always include exact start so dashboard S→path connects (grid center can drift)
    sx_w, sy_w = start_x, start_y
    px, py, _ = poses[0]
    if math.hypot(px - sx_w, py - sy_w) > 1e-3:
        yaw0 = math.atan2(py - sy_w, px - sx_w)
        poses = [(sx_w, sy_w, yaw0)] + list(poses)
    else:
        poses[0] = (sx_w, sy_w, poses[0][2])

    # Exact goal XY (keep goal yaw)
    gx_w, gy_w = goal_x, goal_y
    qx, qy, _ = poses[-1]
    if math.hypot(qx - gx_w, qy - gy_w) > 1e-3:
        poses[-1] = (gx_w, gy_w, goal_yaw)
    else:
        poses[-1] = (gx_w, gy_w, goal_yaw)

    return poses
