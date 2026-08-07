"""Gentle path smoothing — no ROS.

Rounds sharp planner corners with short quadratic Beziers that stay close to
the polyline (like a hand-drawn soft path). EDT is only a safety check: if a
corner cut is too close to a wall, shrink the fillet or keep the sharp vertex.

Do NOT push corners far "outward" — that creates loops/waves.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from skill23_custom_nav.utils import world_to_grid

Pose2D = Tuple[float, float, float]
Point2 = Tuple[float, float]


def _norm(x: float, y: float) -> float:
    return math.hypot(x, y)


def _unit(dx: float, dy: float) -> Point2:
    n = _norm(dx, dy)
    if n < 1e-9:
        return (1.0, 0.0)
    return (dx / n, dy / n)


def _edt_at(
    x: float,
    y: float,
    edt: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    occupancy: Optional[np.ndarray] = None,
    occupied_thresh: int = 50,
) -> float:
    gx, gy = world_to_grid(x, y, origin_x, origin_y, resolution)
    h, w = edt.shape
    if gx < 0 or gy < 0 or gx >= w or gy >= h:
        return 0.0
    if occupancy is not None and int(occupancy[gy, gx]) >= occupied_thresh:
        return 0.0
    return float(edt[gy, gx])


def _clear_enough(
    x: float,
    y: float,
    edt: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    min_clearance_m: float,
    occupancy: Optional[np.ndarray],
    occupied_thresh: int,
) -> bool:
    return _edt_at(
        x, y, edt, origin_x, origin_y, resolution, occupancy, occupied_thresh,
    ) >= min_clearance_m


def _samples_clear(
    samples: Sequence[Point2],
    edt: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    min_clearance_m: float,
    occupancy: Optional[np.ndarray],
    occupied_thresh: int,
) -> bool:
    return all(
        _clear_enough(
            x, y, edt, origin_x, origin_y, resolution,
            min_clearance_m, occupancy, occupied_thresh)
        for x, y in samples
    )


def _quad_bezier(p0: Point2, p1: Point2, p2: Point2, t: float) -> Point2:
    u = 1.0 - t
    return (
        u * u * p0[0] + 2.0 * u * t * p1[0] + t * t * p2[0],
        u * u * p0[1] + 2.0 * u * t * p1[1] + t * t * p2[1],
    )


def _sample_bezier(p0: Point2, p1: Point2, p2: Point2, step_m: float) -> List[Point2]:
    # Approximate arc length for sample count.
    chord = (
        _norm(p1[0] - p0[0], p1[1] - p0[1])
        + _norm(p2[0] - p1[0], p2[1] - p1[1])
    )
    n = max(3, int(math.ceil(chord / max(step_m, 1e-3))))
    # Exclude endpoints (caller owns them).
    return [_quad_bezier(p0, p1, p2, i / n) for i in range(1, n)]


def _yaw_along(prev: Point2, nxt: Point2, fallback: float) -> float:
    dx = nxt[0] - prev[0]
    dy = nxt[1] - prev[1]
    if _norm(dx, dy) < 1e-6:
        return fallback
    return math.atan2(dy, dx)


def _edt_gradient(
    x: float,
    y: float,
    edt: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    occupancy: Optional[np.ndarray],
    occupied_thresh: int,
) -> Point2:
    """Unit vector toward higher clearance (away from walls)."""
    eps = max(resolution, 0.03)
    e0 = _edt_at(x, y, edt, origin_x, origin_y, resolution, occupancy, occupied_thresh)
    ex = _edt_at(x + eps, y, edt, origin_x, origin_y, resolution, occupancy, occupied_thresh)
    ey = _edt_at(x, y + eps, edt, origin_x, origin_y, resolution, occupancy, occupied_thresh)
    return _unit(ex - e0, ey - e0)


def smooth_path_poses(
    poses: Sequence[Pose2D],
    edt: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    occupancy: Optional[np.ndarray] = None,
    occupied_thresh: int = 50,
    min_turn_rad: float = 0.25,
    fillet_radius_m: float = 0.18,
    sample_step_m: float = 0.04,
    min_clearance_m: float = 0.14,
    max_offset_m: float = 0.08,
) -> List[Pose2D]:
    """Gentle corner rounding. Start/goal XY fixed; final yaw kept.

    At each sharp vertex B between A–B–C:
      Bezier(P0, ctrl, P2) with P0/P2 inset on the legs and ctrl ≈ B
      (optionally nudged a few cm toward higher EDT if still clear).

    That softens the kink without looping away from the planner path.
    """
    if len(poses) < 3:
        return list(poses)

    pts: List[Point2] = [(float(p[0]), float(p[1])) for p in poses]
    goal_yaw = float(poses[-1][2])
    out: List[Point2] = [pts[0]]

    for i in range(1, len(pts) - 1):
        a = pts[i - 1]
        b = pts[i]
        c = pts[i + 1]
        len_in = _norm(b[0] - a[0], b[1] - a[1])
        len_out = _norm(c[0] - b[0], c[1] - b[1])
        if len_in < 1e-4 or len_out < 1e-4:
            out.append(b)
            continue

        v_in = _unit(b[0] - a[0], b[1] - a[1])
        v_out = _unit(c[0] - b[0], c[1] - b[1])
        cross = v_in[0] * v_out[1] - v_in[1] * v_out[0]
        dot = max(-1.0, min(1.0, v_in[0] * v_out[0] + v_in[1] * v_out[1]))
        ang = math.atan2(cross, dot)
        if abs(ang) < min_turn_rad:
            out.append(b)
            continue

        # Inset limited by segment length and fillet radius — keep curves tight.
        inset0 = min(0.45 * len_in, 0.45 * len_out, fillet_radius_m)
        placed = False
        for scale in (1.0, 0.7, 0.45, 0.25):
            inset = inset0 * scale
            if inset < sample_step_m:
                break
            p0 = (b[0] - v_in[0] * inset, b[1] - v_in[1] * inset)
            p2 = (b[0] + v_out[0] * inset, b[1] + v_out[1] * inset)

            # Control at vertex (classic corner cut). Tiny EDT nudge only.
            ctrl = b
            gx, gy = _edt_gradient(
                b[0], b[1], edt, origin_x, origin_y, resolution,
                occupancy, occupied_thresh)
            nudge = min(max_offset_m, 0.35 * inset)
            if nudge > 1e-4 and (gx * gx + gy * gy) > 0.25:
                cand = (b[0] + gx * nudge, b[1] + gy * nudge)
                mid = _quad_bezier(p0, cand, p2, 0.5)
                if _clear_enough(
                        mid[0], mid[1], edt, origin_x, origin_y, resolution,
                        min_clearance_m, occupancy, occupied_thresh):
                    # Prefer nudge only if mid-clearance improves vs plain B.
                    mid0 = _quad_bezier(p0, b, p2, 0.5)
                    if _edt_at(
                            mid[0], mid[1], edt, origin_x, origin_y, resolution,
                            occupancy, occupied_thresh) >= _edt_at(
                            mid0[0], mid0[1], edt, origin_x, origin_y,
                            resolution, occupancy, occupied_thresh) - 1e-3:
                        ctrl = cand

            samples = [p0] + _sample_bezier(p0, ctrl, p2, sample_step_m) + [p2]
            if not _samples_clear(
                    samples, edt, origin_x, origin_y, resolution,
                    min_clearance_m, occupancy, occupied_thresh):
                continue

            for p in samples:
                if _norm(p[0] - out[-1][0], p[1] - out[-1][1]) > sample_step_m * 0.35:
                    out.append(p)
            placed = True
            break

        if not placed:
            out.append(b)

    out.append(pts[-1])

    result: List[Pose2D] = []
    for i, (x, y) in enumerate(out):
        if i == len(out) - 1:
            result.append((x, y, goal_yaw))
            continue
        if i == 0 and len(out) > 1:
            yaw = _yaw_along((x, y), out[1], goal_yaw)
        else:
            yaw = _yaw_along(out[i - 1], out[i + 1], goal_yaw)
        result.append((x, y, yaw))

    result[0] = (pts[0][0], pts[0][1], result[0][2])
    result[-1] = (pts[-1][0], pts[-1][1], goal_yaw)
    return result
