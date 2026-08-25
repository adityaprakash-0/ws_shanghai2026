"""Path smoothing — no ROS.

Goal: hand the controller a path it can track without stopping to re-align at
every vertex, while never straying from the planner's route.

Three stages, each guarded:
  1. Drop cm-scale zigzags (Douglas-Peucker + micro-kink removal)
  2. Round every real bend with a local quadratic fillet
  3. Relax what is left (constrained Laplacian) to erase residual kinks

Both stages 2 and 3 must pass two guards before a point is accepted:
  • clearance floor, and never below ``clearance_keep`` of the clearance the
    planner path already had there — smoothing may not drift toward walls
  • ``max_dev_m`` corridor around the planner polyline — smoothing may not
    invent its own route
"""

from __future__ import annotations

import math
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

from skill23_studio.nav.utils import world_to_grid

Pose2D = Tuple[float, float, float]
Point2 = Tuple[float, float]


def _norm(x: float, y: float) -> float:
    return math.hypot(x, y)


def _unit(dx: float, dy: float) -> Point2:
    n = _norm(dx, dy)
    if n < 1e-9:
        return (1.0, 0.0)
    return (dx / n, dy / n)


def _turn_angle(a: Point2, b: Point2, c: Point2) -> float:
    v_in = _unit(b[0] - a[0], b[1] - a[1])
    v_out = _unit(c[0] - b[0], c[1] - b[1])
    cross = v_in[0] * v_out[1] - v_in[1] * v_out[0]
    dot = max(-1.0, min(1.0, v_in[0] * v_out[0] + v_in[1] * v_out[1]))
    return math.atan2(cross, dot)


def _perp_dist_to_segment(p: Point2, a: Point2, c: Point2) -> float:
    ax, ay = a
    cx, cy = c
    px, py = p
    dx, dy = cx - ax, cy - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return _norm(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return _norm(px - (ax + t * dx), py - (ay + t * dy))


def _douglas_peucker(pts: Sequence[Point2], eps: float) -> List[Point2]:
    n = len(pts)
    if n < 3:
        return [(float(p[0]), float(p[1])) for p in pts]
    pts_f = [(float(p[0]), float(p[1])) for p in pts]

    def _recurse(i0: int, i1: int) -> List[int]:
        if i1 <= i0 + 1:
            return []
        a, c = pts_f[i0], pts_f[i1]
        best_i, best_d = -1, 0.0
        for i in range(i0 + 1, i1):
            d = _perp_dist_to_segment(pts_f[i], a, c)
            if d > best_d:
                best_d = d
                best_i = i
        if best_d <= eps or best_i < 0:
            return []
        return _recurse(i0, best_i) + [best_i] + _recurse(best_i, i1)

    keep = sorted(set([0] + _recurse(0, n - 1) + [n - 1]))
    return [pts_f[i] for i in keep]


def simplify_polyline(
    pts: Sequence[Point2],
    max_dev_m: float = 0.03,
    min_turn_rad: float = 0.20,
) -> List[Point2]:
    """Remove only tiny zigzags — keep planner square-corner geometry."""
    if len(pts) < 3:
        return [(float(p[0]), float(p[1])) for p in pts]

    # Small eps so intentional ~20 cm corner offsets are preserved.
    cur = _douglas_peucker(pts, max_dev_m)

    for _ in range(4):
        if len(cur) < 3:
            break
        new: List[Point2] = [cur[0]]
        removed = False
        i = 1
        while i < len(cur) - 1:
            a = new[-1]
            b = cur[i]
            c = cur[i + 1]
            ang = abs(_turn_angle(a, b, c))
            # Only drop nearly-straight micro kinks.
            if ang < min_turn_rad and _perp_dist_to_segment(b, a, c) <= max_dev_m:
                removed = True
                i += 1
                continue
            new.append(b)
            i += 1
        new.append(cur[-1])
        cur = new
        if not removed:
            break
    return cur


class _Corridor:
    """Distance from a point to the planner polyline, vectorised over segments."""

    def __init__(self, poly: Sequence[Point2], max_dev_m: float) -> None:
        arr = np.asarray([(float(p[0]), float(p[1])) for p in poly], dtype=float)
        self.max_dev = float(max_dev_m)
        self.unbounded = self.max_dev <= 1e-6 or len(arr) < 2
        if len(arr) < 2:
            self._a = np.zeros((0, 2))
            self._d = np.zeros((0, 2))
            self._len2 = np.ones((0,))
            return
        self._a = arr[:-1]
        self._d = arr[1:] - arr[:-1]
        self._len2 = np.maximum((self._d * self._d).sum(axis=1), 1e-12)

    def dist(self, x: float, y: float) -> float:
        if len(self._a) == 0:
            return 0.0
        qx = x - self._a[:, 0]
        qy = y - self._a[:, 1]
        t = np.clip(
            (qx * self._d[:, 0] + qy * self._d[:, 1]) / self._len2, 0.0, 1.0)
        ex = qx - t * self._d[:, 0]
        ey = qy - t * self._d[:, 1]
        return float(np.sqrt(ex * ex + ey * ey).min())

    def inside(self, x: float, y: float) -> bool:
        if self.unbounded:
            return True
        return self.dist(x, y) <= self.max_dev

    def dist_many(self, pts: np.ndarray) -> np.ndarray:
        if len(self._a) == 0 or len(pts) == 0:
            return np.zeros(len(pts))
        qx = pts[:, 0][:, None] - self._a[None, :, 0]
        qy = pts[:, 1][:, None] - self._a[None, :, 1]
        t = np.clip(
            (qx * self._d[None, :, 0] + qy * self._d[None, :, 1])
            / self._len2[None, :], 0.0, 1.0)
        ex = qx - t * self._d[None, :, 0]
        ey = qy - t * self._d[None, :, 1]
        return np.sqrt(ex * ex + ey * ey).min(axis=1)


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


def _edt_many(
    pts: np.ndarray,
    edt: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    occupancy: Optional[np.ndarray],
    occupied_thresh: int,
) -> np.ndarray:
    """Vectorised ``_edt_at``; same cell mapping as utils.world_to_grid."""
    if len(pts) == 0:
        return np.zeros(0)
    h, w = edt.shape
    gx = ((pts[:, 0] - origin_x) / resolution).astype(int)
    gy = ((pts[:, 1] - origin_y) / resolution).astype(int)
    inside = (gx >= 0) & (gy >= 0) & (gx < w) & (gy < h)
    gxc = np.clip(gx, 0, w - 1)
    gyc = np.clip(gy, 0, h - 1)
    vals = edt[gyc, gxc].astype(float)
    if occupancy is not None:
        vals = np.where(occupancy[gyc, gxc] >= occupied_thresh, 0.0, vals)
    return np.where(inside, vals, 0.0)


def _quad_bezier(p0: Point2, p1: Point2, p2: Point2, t: float) -> Point2:
    u = 1.0 - t
    return (
        u * u * p0[0] + 2.0 * u * t * p1[0] + t * t * p2[0],
        u * u * p0[1] + 2.0 * u * t * p1[1] + t * t * p2[1],
    )


def _sample_bezier(p0: Point2, p1: Point2, p2: Point2, step_m: float) -> List[Point2]:
    chord = (
        _norm(p1[0] - p0[0], p1[1] - p0[1])
        + _norm(p2[0] - p1[0], p2[1] - p1[1])
    )
    n = max(3, int(math.ceil(chord / max(step_m, 1e-3))))
    return [_quad_bezier(p0, p1, p2, i / n) for i in range(1, n)]


def _yaw_along(prev: Point2, nxt: Point2, fallback: float) -> float:
    dx = nxt[0] - prev[0]
    dy = nxt[1] - prev[1]
    if _norm(dx, dy) < 1e-6:
        return fallback
    return math.atan2(dy, dx)


def _append_point(out: List[Point2], p: Point2, min_spacing: float) -> None:
    if not out:
        out.append(p)
        return
    if _norm(p[0] - out[-1][0], p[1] - out[-1][1]) > min_spacing:
        out.append(p)


def _densify_segment(a: Point2, b: Point2, step_m: float) -> List[Point2]:
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = _norm(dx, dy)
    if L < step_m * 1.5:
        return []
    n = max(1, int(math.floor(L / step_m)))
    pts: List[Point2] = []
    for i in range(1, n):
        t = i / float(n)
        if t >= 0.999:
            break
        pts.append((a[0] + t * dx, a[1] + t * dy))
    return pts


def _resample_polyline(pts: Sequence[Point2], step_m: float) -> List[Point2]:
    """Even arc-length spacing, endpoints preserved.

    The controller walks the polyline by arc length, so uniform spacing keeps
    its lookahead and progress cursor behaving the same everywhere on the path.
    """
    if len(pts) < 2 or step_m <= 1e-6:
        return [(float(p[0]), float(p[1])) for p in pts]
    out: List[Point2] = [(float(pts[0][0]), float(pts[0][1]))]
    residual = step_m
    for a, b in zip(pts, pts[1:]):
        seg = _norm(b[0] - a[0], b[1] - a[1])
        if seg < 1e-12:
            continue
        t = residual
        while t <= seg + 1e-12:
            u = t / seg
            out.append((a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1])))
            t += step_m
        residual = t - seg
    last = (float(pts[-1][0]), float(pts[-1][1]))
    if _norm(out[-1][0] - last[0], out[-1][1] - last[1]) > step_m * 0.35:
        out.append(last)
    else:
        out[-1] = last
    return out


def _relax(
    pts: Sequence[Point2],
    edt: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    occupancy: Optional[np.ndarray],
    occupied_thresh: int,
    corridor: _Corridor,
    need_clear: Callable[[np.ndarray], np.ndarray],
    iterations: int,
    weight: float,
    turn_ref_rad: float,
) -> List[Point2]:
    """Constrained Laplacian pass: pull each point toward its neighbours' mid.

    This is what removes the leftover zigzag the per-vertex fillets cannot
    reach, notably the planner's squared corners, whose 0.30 m legs are too
    short for a fillet wide enough to matter. Every point is tested against the
    clearance and corridor guards, so relaxing cannot re-route the path or walk
    it into a wall. All points move together (Jacobi) to keep this vectorised —
    the whole pass costs a few milliseconds instead of a few hundred.
    """
    P = np.asarray([(float(p[0]), float(p[1])) for p in pts], dtype=float)
    if len(P) < 3 or iterations <= 0:
        return [(float(x), float(y)) for x, y in P]
    w = float(max(0.0, min(0.5, weight)))
    ref = max(float(turn_ref_rad), 1e-6)
    for _ in range(iterations):
        interior = P[1:-1]
        mid = 0.5 * (P[:-2] + P[2:])

        # Weight each point by how much the path actually bends there. Relaxing
        # everything equally would also pull straight runs toward their chord,
        # which spends the corridor budget where there was nothing to smooth.
        v_in = interior - P[:-2]
        v_out = P[2:] - interior
        turn = np.abs(np.arctan2(
            v_in[:, 0] * v_out[:, 1] - v_in[:, 1] * v_out[:, 0],
            v_in[:, 0] * v_out[:, 0] + v_in[:, 1] * v_out[:, 1]))
        wi = w * np.clip(turn / ref, 0.0, 1.0)
        cand = interior + wi[:, None] * (mid - interior)

        if not corridor.unbounded:
            # Shorten the step at the corridor edge rather than vetoing it.
            # A hard veto freezes some points while their neighbours keep
            # moving, and that discontinuity is itself a new kink.
            d_cur = corridor.dist_many(interior)
            d_new = corridor.dist_many(cand)
            over = d_new > corridor.max_dev
            if over.any():
                frac = np.clip(
                    (corridor.max_dev - d_cur)
                    / np.maximum(d_new - d_cur, 1e-9), 0.0, 1.0)
                cand = np.where(
                    over[:, None],
                    interior + frac[:, None] * (cand - interior),
                    cand)

        step = np.hypot(cand[:, 0] - interior[:, 0], cand[:, 1] - interior[:, 1])
        cur_clear = _edt_many(
            interior, edt, origin_x, origin_y, resolution,
            occupancy, occupied_thresh)
        new_clear = _edt_many(
            cand, edt, origin_x, origin_y, resolution,
            occupancy, occupied_thresh)
        ok = (step > 1e-5) & (new_clear >= need_clear(cur_clear))
        if not ok.any():
            break
        P[1:-1][ok] = cand[ok]
    return [(float(x), float(y)) for x, y in P]


def smooth_path_poses(
    poses: Sequence[Pose2D],
    edt: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    occupancy: Optional[np.ndarray] = None,
    occupied_thresh: int = 50,
    min_turn_rad: float = 0.12,
    fillet_radius_m: float = 0.18,
    sample_step_m: float = 0.04,
    min_clearance_m: float = 0.18,
    max_offset_m: float = 0.0,
    simplify_dev_m: float = 0.03,
    max_dev_m: float = 0.10,
    clearance_keep: float = 0.85,
    clearance_ceiling_m: float = 0.30,
    relax_iterations: int = 80,
    relax_weight: float = 0.40,
    relax_turn_ref_rad: float = 0.10,
) -> List[Pose2D]:
    """Round the planner path so the controller can drive it without stopping.

    ``max_dev_m`` bounds how far the result may leave the planner polyline and
    ``clearance_keep`` bounds how much wall clearance it may give up, so the
    extra smoothing cannot re-route the path or shave a wall.
    """
    if len(poses) < 2:
        return list(poses)

    raw: List[Point2] = [(float(p[0]), float(p[1])) for p in poses]
    goal_yaw = float(poses[-1][2])
    corridor = _Corridor(raw, max_dev_m)
    keep = max(0.0, min(1.0, clearance_keep))
    # Out in the open the planner's clearance can be metres, and demanding
    # ``keep`` of that would veto every fillet for no safety gain. Cap what the
    # guard may ask for: past this much clearance the robot is not near a wall.
    ceiling = max(min_clearance_m, clearance_ceiling_m)

    def needed(ref_clear: float) -> float:
        return max(min_clearance_m, min(keep * ref_clear, ceiling))

    def clear_at(x: float, y: float) -> float:
        return _edt_at(
            x, y, edt, origin_x, origin_y, resolution, occupancy, occupied_thresh)

    def accept(x: float, y: float, need_clear: float) -> bool:
        if not corridor.inside(x, y):
            return False
        return clear_at(x, y) >= max(min_clearance_m, need_clear)

    # Preserve planner corners — only erase cm-scale zigzags.
    pts = simplify_polyline(raw, max_dev_m=simplify_dev_m, min_turn_rad=0.18)
    if len(pts) < 2:
        return list(poses)

    if len(pts) < 3:
        out = _resample_polyline(pts, sample_step_m)
        return _with_yaws(out, goal_yaw)

    dense: List[Point2] = [pts[0]]
    min_spacing = sample_step_m * 0.35
    fillet_cap = max(0.03, fillet_radius_m)
    turn_gate = max(0.05, min_turn_rad)

    for i in range(1, len(pts) - 1):
        a = pts[i - 1]
        b = pts[i]
        c = pts[i + 1]
        len_in = _norm(b[0] - a[0], b[1] - a[1])
        len_out = _norm(c[0] - b[0], c[1] - b[1])
        if len_in < 1e-4 or len_out < 1e-4:
            _append_point(dense, b, min_spacing)
            continue

        ang = _turn_angle(a, b, c)
        if abs(ang) < turn_gate:
            for p in _densify_segment(dense[-1], b, sample_step_m):
                _append_point(dense, p, min_spacing)
            _append_point(dense, b, min_spacing)
            continue

        v_in = _unit(b[0] - a[0], b[1] - a[1])
        v_out = _unit(c[0] - b[0], c[1] - b[1])

        # Clearance the planner already had through this corner. The fillet must
        # keep most of it, so rounding can never pull the path toward the wall
        # the squared corner was avoiding.
        ref_pts = [a, b, c]
        ref_pts += _densify_segment(a, b, sample_step_m)
        ref_pts += _densify_segment(b, c, sample_step_m)
        need = needed(min(clear_at(px, py) for px, py in ref_pts))

        # Largest fillet that fits the legs, shrinking until the guards pass.
        inset0 = min(0.45 * len_in, 0.45 * len_out, fillet_cap)
        placed = False
        for scale in (1.0, 0.8, 0.6, 0.45, 0.3):
            inset = inset0 * scale
            if inset < 0.02:
                break
            p0 = (b[0] - v_in[0] * inset, b[1] - v_in[1] * inset)
            p2 = (b[0] + v_out[0] * inset, b[1] + v_out[1] * inset)
            # Control AT vertex → the curve stays local to the turn.
            samples = [p0] + _sample_bezier(p0, b, p2, sample_step_m) + [p2]
            if not all(accept(px, py, need) for px, py in samples):
                continue
            for p in _densify_segment(dense[-1], p0, sample_step_m):
                _append_point(dense, p, min_spacing)
            for p in samples:
                _append_point(dense, p, min_spacing)
            placed = True
            break

        if not placed:
            for p in _densify_segment(dense[-1], b, sample_step_m):
                _append_point(dense, p, min_spacing)
            _append_point(dense, b, min_spacing)

    for p in _densify_segment(dense[-1], pts[-1], sample_step_m):
        _append_point(dense, p, min_spacing)
    _append_point(dense, pts[-1], min_spacing * 0.5)
    dense[0] = pts[0]
    dense[-1] = pts[-1]

    def need_clear(cur: np.ndarray) -> np.ndarray:
        # A move may not give up more than (1 - keep) of the clearance the point
        # already has, so relaxation cannot creep toward a wall.
        return np.maximum(min_clearance_m, np.minimum(keep * cur, ceiling))

    relaxed = _relax(
        dense, edt, origin_x, origin_y, resolution, occupancy, occupied_thresh,
        corridor, need_clear, relax_iterations, relax_weight,
        relax_turn_ref_rad)
    relaxed[0] = pts[0]
    relaxed[-1] = pts[-1]

    out = _resample_polyline(relaxed, sample_step_m)
    return _with_yaws(out, goal_yaw)


def _with_yaws(pts: Sequence[Point2], goal_yaw: float) -> List[Pose2D]:
    """Tangent yaw per point; the last one keeps the commanded goal yaw."""
    n = len(pts)
    if n == 0:
        return []
    if n == 1:
        return [(float(pts[0][0]), float(pts[0][1]), goal_yaw)]
    result: List[Pose2D] = []
    for i, (x, y) in enumerate(pts):
        if i == n - 1:
            yaw = goal_yaw
        elif i == 0:
            yaw = _yaw_along(pts[0], pts[1], goal_yaw)
        else:
            yaw = _yaw_along(pts[i - 1], pts[i + 1], goal_yaw)
        result.append((float(x), float(y), yaw))
    return result
