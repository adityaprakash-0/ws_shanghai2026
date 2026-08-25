"""Grid planning brain (no ROS).

  1) A* preferring highest EDT (corridor center / ridge)
  2) Snap each path cell onto local max-EDT neighbor
  3) String-pull into long straights (keep ridge clearance)
  4) Corner clearance: go ahead past tip, then turn
  Replan: same pipeline on static∩local EDT.
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

    def step_cost(x: int, y: int, step: float, diagonal: bool) -> float:
        # Strongly prefer cells farthest from walls; prefer H/V over diagonal.
        clearance = max(float(edt[y, x]), 0.02)
        cost = step * (clearance_weight / (clearance * clearance))
        if diagonal:
            cost *= 1.35  # discourage diagonal staircases → tilted corridors
        return cost

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
            ng = g_score[(cx, cy)] + step_cost(nx, ny, step, dx != 0 and dy != 0)
            if ng < g_score.get((nx, ny), 1e18):
                g_score[(nx, ny)] = ng
                came_from[(nx, ny)] = (cx, cy)
                heapq.heappush(open_heap, (ng + heuristic(nx, ny), nx, ny))

    return path


def ridge_snap_radius_cells(
    radius_m: float,
    map_resolution: float,
    *,
    min_cells: int = 1,
    max_cells: int = 16,
) -> int:
    """Convert a world-space snap radius into grid cells for the current map.

    A* wobble is a few cells; the useful search window is a fixed distance
    (default 10 cm), not a fixed cell count. On a 5 cm map that is 2 cells;
    on a 2 cm map it is 5 cells; on a 10 cm map it is 1 cell.
    """
    res = max(float(map_resolution), 1e-6)
    cells = int(round(float(radius_m) / res))
    return max(min_cells, min(max_cells, cells))


def snap_to_edt_ridge(
    cells: List[Cell],
    occupancy: np.ndarray,
    edt: np.ndarray,
    *,
    occupied_threshold: int,
    min_clearance_m: float,
    extra_blocked: Optional[np.ndarray],
    radius_cells: int = 2,
    map_origin_x: float = 0.0,
    map_origin_y: float = 0.0,
    map_resolution: float = 0.05,
    los_sample_step_m: float = 0.05,
) -> List[Cell]:
    """Shift each interior cell sideways onto the local EDT ridge.

    Candidates are limited to the perpendicular of the local travel direction,
    must stay ahead of the cell already kept, and must be reachable from it in
    a straight line. Searching the full square instead would let a cell snap
    *backwards* onto a higher-EDT neighbour — at the first waypoint that makes
    the controller open the path with a ~180 deg turn — and could leave two
    consecutive cells on opposite sides of a thin wall.
    """
    if len(cells) <= 2:
        return list(cells)
    out: List[Cell] = [cells[0]]
    for i in range(1, len(cells) - 1):
        cx, cy = cells[i]
        prev = out[-1]
        ahead = cells[i + 1]
        tx = float(ahead[0] - prev[0])
        ty = float(ahead[1] - prev[1])
        t_len = math.hypot(tx, ty)
        if t_len < 1e-9:
            out.append((cx, cy))
            continue
        tx /= t_len
        ty /= t_len

        best = (cx, cy)
        best_c = float(edt[cy, cx])
        for k in range(-radius_cells, radius_cells + 1):
            if k == 0:
                continue
            nx = int(round(cx - ty * k))
            ny = int(round(cy + tx * k))
            if (nx, ny) == (cx, cy):
                continue
            if not _free_cell(
                nx, ny, occupancy, edt,
                occupied_threshold=occupied_threshold,
                min_clearance_m=min_clearance_m,
                extra_blocked=extra_blocked,
            ):
                continue
            if float(edt[ny, nx]) <= best_c:
                continue
            if (nx - prev[0]) * tx + (ny - prev[1]) * ty <= 0.0:
                continue
            if not line_clear(
                prev, (nx, ny), occupancy, edt,
                occupied_threshold=occupied_threshold,
                min_clearance_m=min_clearance_m,
                extra_blocked=extra_blocked,
                map_origin_x=map_origin_x,
                map_origin_y=map_origin_y,
                map_resolution=map_resolution,
                los_sample_step_m=los_sample_step_m,
            ):
                continue
            best_c = float(edt[ny, nx])
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


def _max_edt_along(cells: List[Cell], i: int, j: int, edt: np.ndarray) -> float:
    m = 0.0
    for k in range(i, j + 1):
        cx, cy = cells[k]
        m = max(m, float(edt[cy, cx]))
    return m


def _sharp_turn_between(
    cells: List[Cell],
    i: int,
    j: int,
    map_origin_x: float,
    map_origin_y: float,
    map_resolution: float,
    *,
    min_turn_rad: float = 0.40,
) -> bool:
    """True if any vertex between cells[i] and cells[j] is a sharp turn."""
    if j <= i + 1:
        return False
    for k in range(i + 1, j):
        ax, ay = grid_to_world(cells[k - 1][0], cells[k - 1][1],
                                map_origin_x, map_origin_y, map_resolution)
        bx, by = grid_to_world(cells[k][0], cells[k][1],
                                map_origin_x, map_origin_y, map_resolution)
        cx, cy = grid_to_world(cells[k + 1][0], cells[k + 1][1],
                                map_origin_x, map_origin_y, map_resolution)
        if _turn_abs((ax, ay), (bx, by), (cx, cy)) >= min_turn_rad:
            return True
    return False


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
    clearance_keep: float = 0.0,
) -> List[Cell]:
    """Collapse to long straight segments (farthest visible waypoint).

    Tries every later waypoint (does NOT stop at the first LOS failure).
    When clearance_keep > 0, a shortcut must keep that fraction of the
    *peak* EDT on the original ridge path — so pulls stay centered
    (cannot hug a wall just because min clearance is met).
    """
    if len(cells) <= 2:
        return list(cells)
    cells = compress_collinear(cells)
    out = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        best = i + 1
        for j in range(i + 2, len(cells)):
            if clearance_keep > 1e-6:
                ridge_ref = _max_edt_along(cells, i, j, edt)
                need = max(min_clearance_m, clearance_keep * ridge_ref)
            else:
                need = min_clearance_m
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
            ) and not _sharp_turn_between(
                cells, i, j, map_origin_x, map_origin_y, map_resolution):
                best = j
        out.append(cells[best])
        i = best
    return out


def _unit_xy(dx: float, dy: float) -> Tuple[float, float]:
    n = math.hypot(dx, dy)
    if n < 1e-9:
        return (1.0, 0.0)
    return (dx / n, dy / n)


def _turn_abs(a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]) -> float:
    v_in = _unit_xy(b[0] - a[0], b[1] - a[1])
    v_out = _unit_xy(c[0] - b[0], c[1] - b[1])
    cross = v_in[0] * v_out[1] - v_in[1] * v_out[0]
    dot = max(-1.0, min(1.0, v_in[0] * v_out[0] + v_in[1] * v_out[1]))
    return abs(math.atan2(cross, dot))


def _world_line_clear(
    ax: float, ay: float, bx: float, by: float,
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
) -> bool:
    ga = world_to_grid(ax, ay, map_origin_x, map_origin_y, map_resolution)
    gb = world_to_grid(bx, by, map_origin_x, map_origin_y, map_resolution)
    return line_clear(
        ga, gb, occupancy, edt,
        occupied_threshold=occupied_threshold,
        min_clearance_m=min_clearance_m,
        extra_blocked=extra_blocked,
        map_origin_x=map_origin_x,
        map_origin_y=map_origin_y,
        map_resolution=map_resolution,
        los_sample_step_m=los_sample_step_m,
        required_clearance_m=min_clearance_m,
    )


def _world_free(
    x: float, y: float,
    occupancy: np.ndarray,
    edt: np.ndarray,
    *,
    occupied_threshold: int,
    min_clearance_m: float,
    extra_blocked: Optional[np.ndarray],
    map_origin_x: float,
    map_origin_y: float,
    map_resolution: float,
) -> bool:
    gx, gy = world_to_grid(x, y, map_origin_x, map_origin_y, map_resolution)
    return _free_cell(
        gx, gy, occupancy, edt,
        occupied_threshold=occupied_threshold,
        min_clearance_m=min_clearance_m,
        extra_blocked=extra_blocked,
    )


def _edt_at_world(
    x: float, y: float,
    edt: np.ndarray,
    map_origin_x: float,
    map_origin_y: float,
    map_resolution: float,
) -> float:
    gx, gy = world_to_grid(x, y, map_origin_x, map_origin_y, map_resolution)
    h, w = edt.shape
    if gx < 0 or gy < 0 or gx >= w or gy >= h:
        return 0.0
    return float(edt[gy, gx])


def _best_const_y_edt(
    x0: float, x1: float, y_hint: float,
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
    max_shift_m: float,
    shift_step_m: float,
    target_clearance_m: float = 0.0,
) -> Optional[float]:
    """Constant-Y line (x0→x1) with best clearance (prefer ≥ target_clearance_m)."""
    xs = [x0 + t * (x1 - x0) for t in (0.15, 0.35, 0.5, 0.65, 0.85)]
    ridge_ys: List[float] = []
    for x in xs:
        local_best_y, local_best_e = y_hint, -1.0
        y = y_hint - max_shift_m
        while y <= y_hint + max_shift_m + 1e-9:
            e = _edt_at_world(
                x, y, edt, map_origin_x, map_origin_y, map_resolution)
            if e > local_best_e:
                local_best_e, local_best_y = e, y
            y += shift_step_m
        if local_best_e > 0.0:
            ridge_ys.append(local_best_y)
    seed = sorted(ridge_ys)[len(ridge_ys) // 2] if ridge_ys else y_hint

    # Body-safe floor; straight legs aim for target_clearance_m (e.g. 0.20).
    need = max(min_clearance_m, float(target_clearance_m))

    best_y, best_s = None, -1.0
    best_ok_y, best_ok_s = None, -1.0
    y = seed - max_shift_m
    while y <= seed + max_shift_m + 1e-9:
        if (_world_free(
                x0, y, occupancy, edt,
                occupied_threshold=occupied_threshold,
                min_clearance_m=min_clearance_m,
                extra_blocked=extra_blocked,
                map_origin_x=map_origin_x,
                map_origin_y=map_origin_y,
                map_resolution=map_resolution)
            and _world_free(
                x1, y, occupancy, edt,
                occupied_threshold=occupied_threshold,
                min_clearance_m=min_clearance_m,
                extra_blocked=extra_blocked,
                map_origin_x=map_origin_x,
                map_origin_y=map_origin_y,
                map_resolution=map_resolution)
            and _world_line_clear(
                x0, y, x1, y, occupancy, edt,
                occupied_threshold=occupied_threshold,
                min_clearance_m=min_clearance_m,
                extra_blocked=extra_blocked,
                map_origin_x=map_origin_x,
                map_origin_y=map_origin_y,
                map_resolution=map_resolution,
                los_sample_step_m=los_sample_step_m)):
            s = _min_edt_world_segment(
                x0, y, x1, y, edt,
                map_origin_x, map_origin_y, map_resolution, los_sample_step_m)
            if s > best_s + 1e-4:
                best_s, best_y = s, y
            if s >= need - 1e-4 and s > best_ok_s + 1e-4:
                best_ok_s, best_ok_y = s, y
        y += shift_step_m
    return best_ok_y if best_ok_y is not None else best_y


def _best_const_x_edt(
    y0: float, y1: float, x_hint: float,
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
    max_shift_m: float,
    shift_step_m: float,
    target_clearance_m: float = 0.0,
) -> Optional[float]:
    """Constant-X line (y0→y1) with best clearance (prefer ≥ target_clearance_m)."""
    ys = [y0 + t * (y1 - y0) for t in (0.15, 0.35, 0.5, 0.65, 0.85)]
    ridge_xs: List[float] = []
    for y in ys:
        local_best_x, local_best_e = x_hint, -1.0
        x = x_hint - max_shift_m
        while x <= x_hint + max_shift_m + 1e-9:
            e = _edt_at_world(
                x, y, edt, map_origin_x, map_origin_y, map_resolution)
            if e > local_best_e:
                local_best_e, local_best_x = e, x
            x += shift_step_m
        if local_best_e > 0.0:
            ridge_xs.append(local_best_x)
    seed = sorted(ridge_xs)[len(ridge_xs) // 2] if ridge_xs else x_hint

    need = max(min_clearance_m, float(target_clearance_m))

    best_x, best_s = None, -1.0
    best_ok_x, best_ok_s = None, -1.0
    x = seed - max_shift_m
    while x <= seed + max_shift_m + 1e-9:
        if (_world_free(
                x, y0, occupancy, edt,
                occupied_threshold=occupied_threshold,
                min_clearance_m=min_clearance_m,
                extra_blocked=extra_blocked,
                map_origin_x=map_origin_x,
                map_origin_y=map_origin_y,
                map_resolution=map_resolution)
            and _world_free(
                x, y1, occupancy, edt,
                occupied_threshold=occupied_threshold,
                min_clearance_m=min_clearance_m,
                extra_blocked=extra_blocked,
                map_origin_x=map_origin_x,
                map_origin_y=map_origin_y,
                map_resolution=map_resolution)
            and _world_line_clear(
                x, y0, x, y1, occupancy, edt,
                occupied_threshold=occupied_threshold,
                min_clearance_m=min_clearance_m,
                extra_blocked=extra_blocked,
                map_origin_x=map_origin_x,
                map_origin_y=map_origin_y,
                map_resolution=map_resolution,
                los_sample_step_m=los_sample_step_m)):
            s = _min_edt_world_segment(
                x, y0, x, y1, edt,
                map_origin_x, map_origin_y, map_resolution, los_sample_step_m)
            if s > best_s + 1e-4:
                best_s, best_x = s, x
            if s >= need - 1e-4 and s > best_ok_s + 1e-4:
                best_ok_s, best_ok_x = s, x
        x += shift_step_m
    return best_ok_x if best_ok_x is not None else best_x


def force_edt_axis_corridors(
    poses: List[Pose2D],
    start_xy: Tuple[float, float],
    goal_xy: Tuple[float, float],
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
    straight_clearance_m: float = 0.20,
    axis_tol_deg: float = 40.0,
    max_shift_m: float = 0.40,
    shift_step_m: float = 0.02,
    min_seg_m: float = 0.35,
) -> List[Pose2D]:
    """Nudge long near-H/V legs onto a clearance line — never through walls.

    Start/goal stay pinned. Each change is kept only if the full polyline
    stays LOS-clear; otherwise it is skipped. On total failure, revert.
    """
    if len(poses) < 2:
        return list(poses)

    goal_yaw = poses[-1][2]
    tol = math.radians(axis_tol_deg)
    sx, sy = float(start_xy[0]), float(start_xy[1])
    gx, gy = float(goal_xy[0]), float(goal_xy[1])
    original: List[Tuple[float, float]] = [(p[0], p[1]) for p in poses]
    pts: List[Tuple[float, float]] = list(original)
    pts[0] = (sx, sy)
    pts[-1] = (gx, gy)

    def _ok_seg(x0: float, y0: float, x1: float, y1: float) -> bool:
        return _world_line_clear(
            x0, y0, x1, y1, occupancy, edt,
            occupied_threshold=occupied_threshold,
            min_clearance_m=min_clearance_m,
            extra_blocked=extra_blocked,
            map_origin_x=map_origin_x,
            map_origin_y=map_origin_y,
            map_resolution=map_resolution,
            los_sample_step_m=los_sample_step_m,
        )

    def _path_clear(poly: List[Tuple[float, float]]) -> bool:
        for j in range(len(poly) - 1):
            if not _ok_seg(poly[j][0], poly[j][1], poly[j + 1][0], poly[j + 1][1]):
                return False
        return True

    kw = dict(
        occupied_threshold=occupied_threshold,
        min_clearance_m=min_clearance_m,
        extra_blocked=extra_blocked,
        map_origin_x=map_origin_x,
        map_origin_y=map_origin_y,
        map_resolution=map_resolution,
        los_sample_step_m=los_sample_step_m,
        max_shift_m=max_shift_m,
        shift_step_m=shift_step_m,
        target_clearance_m=float(straight_clearance_m),
    )

    # start→goal only: insert centerline (never move the pins themselves).
    if len(pts) == 2:
        ax, ay = pts[0]
        bx, by = pts[1]
        if math.hypot(bx - ax, by - ay) >= min_seg_m:
            ang = abs(math.atan2(by - ay, bx - ax))
            near_h = ang < tol or abs(ang - math.pi) < tol
            near_v = abs(ang - 0.5 * math.pi) < tol
            trial = None
            if near_h or abs(bx - ax) >= abs(by - ay):
                cy = _best_const_y_edt(
                    ax, bx, 0.5 * (ay + by), occupancy, edt, **kw)
                if cy is not None:
                    trial = [(sx, sy), (sx, cy), (gx, cy), (gx, gy)]
            elif near_v or abs(by - ay) > abs(bx - ax):
                cx = _best_const_x_edt(
                    ay, by, 0.5 * (ax + bx), occupancy, edt, **kw)
                if cx is not None:
                    trial = [(sx, sy), (cx, sy), (cx, gy), (gx, gy)]
            if trial is not None and _path_clear(trial):
                pts = trial
    else:
        # Only move interior vertices onto H/V clearance lines.
        for i in range(len(pts) - 1):
            ax, ay = pts[i]
            bx, by = pts[i + 1]
            dx, dy = bx - ax, by - ay
            L = math.hypot(dx, dy)
            if L < min_seg_m:
                continue
            ang = abs(math.atan2(dy, dx))
            near_h = ang < tol or abs(ang - math.pi) < tol
            near_v = abs(ang - 0.5 * math.pi) < tol
            if not near_h and not near_v:
                continue

            trial = list(pts)
            if near_h:
                cy = _best_const_y_edt(
                    ax, bx, 0.5 * (ay + by), occupancy, edt, **kw)
                if cy is None:
                    continue
                if i > 0:
                    trial[i] = (ax, cy)
                if i + 1 < len(pts) - 1:
                    trial[i + 1] = (bx, cy)
            else:
                cx = _best_const_x_edt(
                    ay, by, 0.5 * (ax + bx), occupancy, edt, **kw)
                if cx is None:
                    continue
                if i > 0:
                    trial[i] = (cx, ay)
                if i + 1 < len(pts) - 1:
                    trial[i + 1] = (cx, by)

            trial[0] = (sx, sy)
            trial[-1] = (gx, gy)
            if trial == pts:
                continue
            if _path_clear(trial):
                pts = trial

        # Clearance entry jog from start (only if LOS-safe).
        if len(pts) >= 2:
            nx, ny = pts[1]
            if abs(nx - sx) >= abs(ny - sy) and abs(sy - ny) > 0.05:
                mid = (sx, ny)
            elif abs(ny - sy) > abs(nx - sx) and abs(sx - nx) > 0.05:
                mid = (nx, sy)
            else:
                mid = None
            if mid is not None and math.hypot(mid[0] - sx, mid[1] - sy) > 0.03:
                trial = [(sx, sy), mid] + list(pts[1:])
                trial[-1] = (gx, gy)
                if _path_clear(trial):
                    pts = trial

        # Clearance exit jog to goal.
        if len(pts) >= 2:
            px, py = pts[-2]
            if abs(gx - px) >= abs(gy - py) and abs(py - gy) > 0.05:
                mid = (gx, py)
            elif abs(gy - py) > abs(gx - px) and abs(px - gx) > 0.05:
                mid = (px, gy)
            else:
                mid = None
            if mid is not None and math.hypot(mid[0] - gx, mid[1] - gy) > 0.03:
                trial = list(pts[:-1]) + [mid, (gx, gy)]
                trial[0] = (sx, sy)
                if _path_clear(trial):
                    pts = trial

    pts[0] = (sx, sy)
    pts[-1] = (gx, gy)

    if not _path_clear(pts):
        pts = list(original)
        pts[0] = (sx, sy)
        pts[-1] = (gx, gy)

    cleaned: List[Tuple[float, float]] = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - cleaned[-1][0], p[1] - cleaned[-1][1]) > 0.02:
            cleaned.append(p)
    cleaned[0] = (sx, sy)
    cleaned[-1] = (gx, gy)

    out: List[Pose2D] = []
    for i, (x, y) in enumerate(cleaned):
        if i < len(cleaned) - 1:
            yaw = math.atan2(cleaned[i + 1][1] - y, cleaned[i + 1][0] - x)
        else:
            yaw = goal_yaw
        out.append((x, y, yaw))
    out[-1] = (cleaned[-1][0], cleaned[-1][1], goal_yaw)
    return out


# Back-compat alias (older callers / tests).
def axis_align_poses(
    poses: List[Pose2D],
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
    straight_clearance_m: float = 0.20,
    axis_tol_deg: float = 55.0,
    max_shift_m: float = 0.80,
    shift_step_m: float = 0.02,
    min_seg_m: float = 0.30,
) -> List[Pose2D]:
    if not poses:
        return []
    start_xy = (poses[0][0], poses[0][1])
    goal_xy = (poses[-1][0], poses[-1][1])
    return force_edt_axis_corridors(
        poses, start_xy, goal_xy, occupancy, edt,
        occupied_threshold=occupied_threshold,
        min_clearance_m=min_clearance_m,
        extra_blocked=extra_blocked,
        map_origin_x=map_origin_x,
        map_origin_y=map_origin_y,
        map_resolution=map_resolution,
        los_sample_step_m=los_sample_step_m,
        straight_clearance_m=straight_clearance_m,
        axis_tol_deg=axis_tol_deg,
        max_shift_m=max_shift_m,
        shift_step_m=shift_step_m,
        min_seg_m=min_seg_m,
    )


def delay_corner_turns(
    poses: List[Pose2D],
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
    corner_clearance_m: float = 0.20,
    min_turn_rad: float = 0.40,
) -> List[Pose2D]:
    """At each turn: straight past the tip, in-place 90°, straight on new leg.

    Replaces sharp corner B (A→B→C) with:
      P1 = B + v_in * m   — keep driving straight past the wall tip
      (P1, yaw_in) → (P1, yaw_out) — rotate in place
      P2 = P1 + v_out * m — straight on the next corridor
    """
    if len(poses) < 3 or corner_clearance_m <= 1e-4:
        return list(poses)

    goal_yaw = poses[-1][2]
    pts: List[Tuple[float, float]] = [(p[0], p[1]) for p in poses]
    out: List[Pose2D] = [poses[0]]

    def _ok_pt(x: float, y: float) -> bool:
        return _world_free(
            x, y, occupancy, edt,
            occupied_threshold=occupied_threshold,
            min_clearance_m=min_clearance_m,
            extra_blocked=extra_blocked,
            map_origin_x=map_origin_x,
            map_origin_y=map_origin_y,
            map_resolution=map_resolution,
        )

    def _ok_seg(x0: float, y0: float, x1: float, y1: float) -> bool:
        return _world_line_clear(
            x0, y0, x1, y1, occupancy, edt,
            occupied_threshold=occupied_threshold,
            min_clearance_m=min_clearance_m,
            extra_blocked=extra_blocked,
            map_origin_x=map_origin_x,
            map_origin_y=map_origin_y,
            map_resolution=map_resolution,
            los_sample_step_m=los_sample_step_m,
        )

    for i in range(1, len(pts) - 1):
        a = pts[i - 1]
        b = pts[i]
        c = pts[i + 1]
        if _turn_abs(a, b, c) < min_turn_rad:
            out.append(poses[i])
            continue

        v_in = _unit_xy(b[0] - a[0], b[1] - a[1])
        v_out = _unit_xy(c[0] - b[0], c[1] - b[1])
        len_in = math.hypot(b[0] - a[0], b[1] - a[1])
        len_out = math.hypot(c[0] - b[0], c[1] - b[1])
        if min(len_in, len_out) < max(0.35, corner_clearance_m * 1.5):
            out.append(poses[i])
            continue
        edt_b = _edt_at_world(
            b[0], b[1], edt, map_origin_x, map_origin_y, map_resolution)
        if edt_b > max(0.18, min_clearance_m * 1.6):
            out.append(poses[i])
            continue
        m_cap = min(corner_clearance_m, 0.45 * len_in, 0.45 * len_out)
        yaw_in = math.atan2(v_in[1], v_in[0])
        yaw_out = math.atan2(v_out[1], v_out[0])

        placed = False
        for scale in (1.0, 0.75, 0.5, 0.35):
            m = m_cap * scale
            if m < 0.04:
                break
            p1 = (b[0] + v_in[0] * m, b[1] + v_in[1] * m)
            p2 = (p1[0] + v_out[0] * m, p1[1] + v_out[1] * m)
            if not (_ok_pt(p1[0], p1[1]) and _ok_pt(p2[0], p2[1])):
                continue
            lx, ly, _ = out[-1]
            if not _ok_seg(lx, ly, p1[0], p1[1]):
                continue
            if not _ok_seg(p1[0], p1[1], p2[0], p2[1]):
                continue
            if not _ok_seg(p2[0], p2[1], c[0], c[1]):
                continue
            out.append((p1[0], p1[1], yaw_in))
            out.append((p1[0], p1[1], yaw_out))
            out.append((p2[0], p2[1], yaw_out))
            placed = True
            break
        if not placed:
            out.append(poses[i])

    out.append(poses[-1])
    out[-1] = (out[-1][0], out[-1][1], goal_yaw)
    return out


def string_pull_poses(
    poses: List[Pose2D],
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
) -> List[Pose2D]:
    """World-space string-pull — farthest visible pose (skip zigzags)."""
    if len(poses) <= 2:
        return list(poses)
    goal_yaw = poses[-1][2]
    pts = [(p[0], p[1]) for p in poses]
    out_i = [0]
    i = 0
    while i < len(pts) - 1:
        best = i + 1
        for j in range(i + 2, len(pts)):
            if _world_line_clear(
                    pts[i][0], pts[i][1], pts[j][0], pts[j][1],
                    occupancy, edt,
                    occupied_threshold=occupied_threshold,
                    min_clearance_m=min_clearance_m,
                    extra_blocked=extra_blocked,
                    map_origin_x=map_origin_x,
                    map_origin_y=map_origin_y,
                    map_resolution=map_resolution,
                    los_sample_step_m=los_sample_step_m):
                best = j
        out_i.append(best)
        i = best
    out: List[Pose2D] = []
    for k, idx in enumerate(out_i):
        x, y = pts[idx]
        if k < len(out_i) - 1:
            nx, ny = pts[out_i[k + 1]]
            yaw = math.atan2(ny - y, nx - x)
        else:
            yaw = goal_yaw
        out.append((x, y, yaw))
    out[-1] = (pts[-1][0], pts[-1][1], goal_yaw)
    return out


def _min_edt_world_segment(
    ax: float, ay: float, bx: float, by: float,
    edt: np.ndarray,
    map_origin_x: float,
    map_origin_y: float,
    map_resolution: float,
    step_m: float,
) -> float:
    dist = math.hypot(bx - ax, by - ay)
    if dist < 1e-6:
        gx, gy = world_to_grid(ax, ay, map_origin_x, map_origin_y, map_resolution)
        h, w = edt.shape
        if 0 <= gx < w and 0 <= gy < h:
            return float(edt[gy, gx])
        return 0.0
    n = max(1, int(math.ceil(dist / max(step_m, 1e-3))))
    h, w = edt.shape
    best = 1e18
    for i in range(n + 1):
        t = i / n
        wx = ax + t * (bx - ax)
        wy = ay + t * (by - ay)
        gx, gy = world_to_grid(wx, wy, map_origin_x, map_origin_y, map_resolution)
        if not (0 <= gx < w and 0 <= gy < h):
            return 0.0
        best = min(best, float(edt[gy, gx]))
    return 0.0 if best > 1e17 else best


def center_straights_on_edt(
    poses: List[Pose2D],
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
    max_shift_m: float = 0.28,
    min_seg_m: float = 0.40,
    shift_step_m: float = 0.025,
) -> List[Pose2D]:
    """Shift long corridor segments laterally onto the EDT ridge (mid-corridor).

    Short legs (corner-clearance stubs) are left alone so square turns stay.
    Start/goal XY are restored by the caller.
    """
    if len(poses) < 2 or max_shift_m < 1e-4:
        return list(poses)

    goal_yaw = poses[-1][2]
    pts: List[Tuple[float, float]] = [(p[0], p[1]) for p in poses]
    new_pts = list(pts)

    def _ok_pt(x: float, y: float) -> bool:
        return _world_free(
            x, y, occupancy, edt,
            occupied_threshold=occupied_threshold,
            min_clearance_m=min_clearance_m,
            extra_blocked=extra_blocked,
            map_origin_x=map_origin_x,
            map_origin_y=map_origin_y,
            map_resolution=map_resolution,
        )

    def _ok_seg(x0: float, y0: float, x1: float, y1: float) -> bool:
        return _world_line_clear(
            x0, y0, x1, y1, occupancy, edt,
            occupied_threshold=occupied_threshold,
            min_clearance_m=min_clearance_m,
            extra_blocked=extra_blocked,
            map_origin_x=map_origin_x,
            map_origin_y=map_origin_y,
            map_resolution=map_resolution,
            los_sample_step_m=los_sample_step_m,
        )

    for i in range(len(pts) - 1):
        a = pts[i]
        b = pts[i + 1]
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy)
        if L < min_seg_m:
            continue
        v = _unit_xy(dx, dy)
        perp = (-v[1], v[0])

        best_o = 0.0
        best_score = _min_edt_world_segment(
            a[0], a[1], b[0], b[1], edt,
            map_origin_x, map_origin_y, map_resolution, los_sample_step_m)

        o = -max_shift_m
        while o <= max_shift_m + 1e-9:
            if abs(o) < 1e-9:
                o += shift_step_m
                continue
            a2 = (a[0] + perp[0] * o, a[1] + perp[1] * o)
            b2 = (b[0] + perp[0] * o, b[1] + perp[1] * o)
            if not (_ok_pt(a2[0], a2[1]) and _ok_pt(b2[0], b2[1])):
                o += shift_step_m
                continue
            if not _ok_seg(a2[0], a2[1], b2[0], b2[1]):
                o += shift_step_m
                continue
            score = _min_edt_world_segment(
                a2[0], a2[1], b2[0], b2[1], edt,
                map_origin_x, map_origin_y, map_resolution, los_sample_step_m)
            if score > best_score + 1e-4:
                best_score = score
                best_o = o
            o += shift_step_m

        if abs(best_o) < 1e-4:
            continue
        # Apply same lateral shift to both ends of this corridor chord.
        new_pts[i] = (a[0] + perp[0] * best_o, a[1] + perp[1] * best_o)
        new_pts[i + 1] = (b[0] + perp[0] * best_o, b[1] + perp[1] * best_o)

    out: List[Pose2D] = []
    for i, (x, y) in enumerate(new_pts):
        if i < len(new_pts) - 1:
            yaw = math.atan2(new_pts[i + 1][1] - y, new_pts[i + 1][0] - x)
        else:
            yaw = goal_yaw
        out.append((x, y, yaw))
    out[-1] = (new_pts[-1][0], new_pts[-1][1], goal_yaw)
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


def drop_start_backtrack(
    poses: List[Pose2D],
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
    max_turn_rad: float = 2.0,
) -> List[Pose2D]:
    """Drop leading waypoints that sit behind the pinned start pose.

    Pose 0 is the live robot XY while pose 1 is a cell centre, so pose 1 can
    land a few centimetres behind the robot. The controller would answer that
    with a near-180 deg turn, a short crawl, and another turn back. Dropping it
    is only allowed when the straight line that replaces it stays clear, which
    is what keeps a genuine U-turn around a wall intact.
    """
    while len(poses) >= 3:
        sx, sy, _ = poses[0]
        bx, by, _ = poses[1]
        cx, cy, _ = poses[2]
        if _turn_abs((sx, sy), (bx, by), (cx, cy)) < max_turn_rad:
            break
        if not _world_line_clear(
            sx, sy, cx, cy, occupancy, edt,
            occupied_threshold=occupied_threshold,
            min_clearance_m=min_clearance_m,
            extra_blocked=extra_blocked,
            map_origin_x=map_origin_x,
            map_origin_y=map_origin_y,
            map_resolution=map_resolution,
            los_sample_step_m=los_sample_step_m,
        ):
            break
        poses = [(sx, sy, math.atan2(cy - sy, cx - sx))] + poses[2:]
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
    corner_clearance_m: float = 0.20,
    ridge_snap_radius_m: float = 0.10,
) -> Optional[List[Pose2D]]:
    """Simple planner: follow EDT ridge, straighten, square corners.

    1. A* costs prefer high EDT → natural mid-corridor
    2. Snap cells to local max EDT within ``ridge_snap_radius_m``
    3. String-pull long straights without dropping ridge clearance
    4. ``corner_clearance_m``: go ahead past tip, then turn
    """
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

    # Sit on highest local EDT (corridor center), then straighten.
    cells = snap_to_edt_ridge(
        cells, occupancy, edt,
        occupied_threshold=occupied_threshold,
        min_clearance_m=min_clearance_m,
        extra_blocked=extra_blocked,
        radius_cells=ridge_snap_radius_cells(
            ridge_snap_radius_m, map_resolution),
        map_origin_x=map_origin_x,
        map_origin_y=map_origin_y,
        map_resolution=map_resolution,
        los_sample_step_m=los_sample_step_m,
    )
    cells = string_pull(
        cells, occupancy, edt,
        occupied_threshold=occupied_threshold,
        min_clearance_m=min_clearance_m,
        extra_blocked=extra_blocked,
        map_origin_x=map_origin_x,
        map_origin_y=map_origin_y,
        map_resolution=map_resolution,
        los_sample_step_m=los_sample_step_m,
        # Keep ~85% of peak ridge EDT so pull stays centered, not wall-hugging.
        clearance_keep=0.85,
    )

    poses = cells_to_poses(
        cells, map_origin_x, map_origin_y, map_resolution, goal_yaw)
    if not poses:
    return None

    sx_w, sy_w = start_x, start_y
    gx_w, gy_w = goal_x, goal_y
    px, py, _ = poses[0]
    if math.hypot(px - sx_w, py - sy_w) > 1e-3:
        yaw0 = math.atan2(py - sy_w, px - sx_w)
        poses = [(sx_w, sy_w, yaw0)] + list(poses)
    else:
        poses[0] = (sx_w, sy_w, poses[0][2])
    poses[-1] = (gx_w, gy_w, goal_yaw)

    poses = drop_start_backtrack(
        poses, occupancy, edt,
        occupied_threshold=occupied_threshold,
        min_clearance_m=min_clearance_m,
        extra_blocked=extra_blocked,
        map_origin_x=map_origin_x,
        map_origin_y=map_origin_y,
        map_resolution=map_resolution,
        los_sample_step_m=los_sample_step_m,
    )

    # Square turns: go ahead past wall tip, then turn.
    poses = delay_corner_turns(
        poses, occupancy, edt,
        occupied_threshold=occupied_threshold,
        min_clearance_m=min_clearance_m,
        extra_blocked=extra_blocked,
        map_origin_x=map_origin_x,
        map_origin_y=map_origin_y,
        map_resolution=map_resolution,
        los_sample_step_m=los_sample_step_m,
        corner_clearance_m=corner_clearance_m,
    )
    poses[0] = (sx_w, sy_w, poses[0][2])
    poses[-1] = (gx_w, gy_w, goal_yaw)
    return poses
