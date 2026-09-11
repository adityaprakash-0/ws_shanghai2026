#!/usr/bin/env python3
"""Verify straight_line L collapse + box-detour removal + L-corner fillet arcs."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

Point = Tuple[int, int]
WPoint = Tuple[float, float]


@dataclass
class Cand:
    j: int
    corner: Point
    has_corner: bool
    peak: int
    mean: float
    score: float


def make_score(i: int, j: int, peak: int, mean: float) -> float:
    return 2.5 * peak + mean - 8.0 * (j - i)


def better(a: Cand, b: Cand) -> bool:
    if abs(a.score - b.score) > 1e-6:
        return a.score < b.score
    if a.peak != b.peak:
        return a.peak < b.peak
    if abs(a.mean - b.mean) > 1e-9:
        return a.mean < b.mean
    return a.j > b.j


def axis_metrics(a: Point, b: Point, cost_at) -> Tuple[int, float]:
    n = max(abs(b[0] - a[0]), abs(b[1] - a[1]))
    dx = 0 if n == 0 else (b[0] - a[0]) // n
    dy = 0 if n == 0 else (b[1] - a[1]) // n
    cells = [cost_at(a[0] + k * dx, a[1] + k * dy) for k in range(n + 1)]
    return max(cells), sum(cells) / len(cells)


def orthogonal_smooth(path: List[Point], cost_at) -> List[Point]:
    if len(path) < 3:
        return list(path)
    out: List[Point] = []
    i = 0
    while i + 1 < len(path):
        out.append(path[i])
        cands: List[Cand] = []
        for j in range(i + 1, len(path)):
            a, b = path[i], path[j]
            if a[0] == b[0] or a[1] == b[1]:
                peak, mean = axis_metrics(a, b, cost_at)
                cands.append(Cand(j, a, False, peak, mean, make_score(i, j, peak, mean)))
            else:
                options = [(b[0], a[1]), (a[0], b[1])]
                best = None
                for c in options:
                    p1, m1 = axis_metrics(a, c, cost_at)
                    p2, m2 = axis_metrics(c, b, cost_at)
                    peak = max(p1, p2)
                    mean = 0.5 * (m1 + m2)
                    sc = make_score(i, j, peak, mean)
                    cand = Cand(j, c, True, peak, mean, sc)
                    if best is None or better(cand, best):
                        best = cand
                if best:
                    cands.append(best)
        best = Cand(i + 1, path[i], False, 255, 1e9, 1e100)
        for c in cands:
            if better(c, best):
                best = c
        if best.has_corner:
            out.append(best.corner)
        i = best.j
    out.append(path[-1])
    return out


def count_turns(path: List[Point]) -> int:
    turns = 0
    for i in range(1, len(path) - 1):
        d1 = (path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        d2 = (path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
        if d1 != (0, 0) and d2 != (0, 0) and d1 != d2:
            turns += 1
    return turns


def manhattan(a: Point, b: Point) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def collapse_box_detours(path: List[Point]) -> List[Point]:
    """Mirror of ClearancePlanner::collapseBoxDetours (free-space, cost 0)."""
    path = list(path)
    if len(path) < 4:
        return path

    def subpath_len(i: int, j: int) -> int:
        return sum(manhattan(path[k], path[k + 1]) for k in range(i, j))

    changed = True
    guard = 0
    while changed and guard < 16:
        guard += 1
        changed = False
        out = [path[0]]
        for i in range(1, len(path) - 1):
            prev, cur, nxt = out[-1], path[i], path[i + 1]
            d1 = (cur[0] - prev[0], cur[1] - prev[1])
            d2 = (nxt[0] - cur[0], nxt[1] - cur[1])
            cross = d1[0] * d2[1] - d1[1] * d2[0]
            dot = d1[0] * d2[0] + d1[1] * d2[1]
            if cross == 0 and dot > 0:
                continue
            out.append(cur)
        out.append(path[-1])
        path = out
        if len(path) < 4:
            return path

        for i in range(0, len(path) - 3):
            best_j = None
            best_corner = None
            best_save = 0
            for j in range(len(path) - 1, i + 2, -1):
                chord = manhattan(path[i], path[j])
                if chord <= 0:
                    continue
                save = subpath_len(i, j) - chord
                if save < 2:
                    continue
                a, b = path[i], path[j]
                if a[0] == b[0] or a[1] == b[1]:
                    best_j, best_corner, best_save = j, None, save
                    break
                best_j, best_corner, best_save = j, (b[0], a[1]), save
                break
            if best_save < 2 or best_j is None:
                continue
            new_path = path[: i + 1]
            if best_corner is not None and best_corner not in (path[i], path[best_j]):
                new_path.append(best_corner)
            new_path.extend(path[best_j:])
            path = new_path
            changed = True
            break
    return path


def unit_axis(dx: float, dy: float) -> Optional[Tuple[float, float]]:
    ax, ay = abs(dx), abs(dy)
    if ax < 1e-9 and ay < 1e-9:
        return None
    if ax >= ay * 8.0:
        return (1.0 if dx >= 0 else -1.0, 0.0)
    if ay >= ax * 8.0:
        return (0.0, 1.0 if dy >= 0 else -1.0)
    return None


def fillet_l_path(
    verts: List[WPoint], radius: float, arc_points: int = 8, spacing: float = 0.05
) -> List[WPoint]:
    """Mirror of ClearancePlanner::roundLCornersToWorld (no costmap)."""
    if len(verts) < 3 or radius <= spacing * 0.5:
        return list(verts)

    out: List[WPoint] = [verts[0]]

    def append_line(a: WPoint, b: WPoint) -> None:
        dx, dy = b[0] - a[0], b[1] - a[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-9:
            return
        n = max(1, int(math.ceil(dist / spacing)))
        skip0 = out and math.hypot(out[-1][0] - a[0], out[-1][1] - a[1]) < 1e-6
        for k in range(1 if skip0 else 0, n + 1):
            t = k / n
            out.append((a[0] + t * dx, a[1] + t * dy))

    for i in range(1, len(verts) - 1):
        A, C, B = verts[i - 1], verts[i], verts[i + 1]
        ui = unit_axis(C[0] - A[0], C[1] - A[1])
        uo = unit_axis(B[0] - C[0], B[1] - C[1])
        if ui is None or uo is None:
            append_line(out[-1], C)
            continue
        cross = ui[0] * uo[1] - ui[1] * uo[0]
        dot = ui[0] * uo[0] + ui[1] * uo[1]
        if abs(dot) > 1e-6 or abs(abs(cross) - 1.0) > 1e-6:
            append_line(out[-1], C)
            continue
        len_in = math.hypot(C[0] - A[0], C[1] - A[1])
        len_out = math.hypot(B[0] - C[0], B[1] - C[1])
        r = min(
            radius,
            0.5 * max(0.0, len_in - spacing),
            0.5 * max(0.0, len_out - spacing),
        )
        if r < spacing:
            append_line(out[-1], C)
            continue
        P = (C[0] - ui[0] * r, C[1] - ui[1] * r)
        Q = (C[0] + uo[0] * r, C[1] + uo[1] * r)
        lx, ly = -ui[1], ui[0]
        nx, ny = (lx, ly) if cross > 0 else (-lx, -ly)
        ox, oy = P[0] + nx * r, P[1] + ny * r
        a0 = math.atan2(P[1] - oy, P[0] - ox)
        a1 = math.atan2(Q[1] - oy, Q[0] - ox)
        delta = a1 - a0
        while delta > math.pi:
            delta -= 2 * math.pi
        while delta <= -math.pi:
            delta += 2 * math.pi
        if cross > 0 and delta < 0:
            delta += 2 * math.pi
        elif cross < 0 and delta > 0:
            delta -= 2 * math.pi
        append_line(out[-1], P)
        n_arc = max(2, arc_points)
        for k in range(1, n_arc + 1):
            t = k / n_arc
            a = a0 + t * delta
            out.append((ox + r * math.cos(a), oy + r * math.sin(a)))

    append_line(out[-1], verts[-1])
    return out


def corner_is_rounded(path: List[WPoint], corner: WPoint) -> bool:
    for p in path:
        if math.hypot(p[0] - corner[0], p[1] - corner[1]) < 1e-4:
            return False
    return True


def subpath_len_check(path: List[Point]) -> int:
    return sum(manhattan(path[i], path[i + 1]) for i in range(len(path) - 1))


def main() -> None:
    stair = [(0, 0)]
    x = y = 0
    while x < 6 or y < 4:
        if x < 6:
            x += 1
            stair.append((x, y))
        if y < 4:
            y += 1
            stair.append((x, y))

    def cost0(_x, _y):
        return 0

    out0 = orthogonal_smooth(stair, cost0)
    assert count_turns(out0) == 1 and len(out0) == 3, out0

    box = [(0, 0), (0, 10), (5, 10), (5, 4), (5, 20)]
    flat = collapse_box_detours(box)
    assert count_turns(flat) <= 1, flat
    assert manhattan(flat[0], flat[-1]) == 25, flat
    assert subpath_len_check(flat) == 25, flat

    u = [(0, 0), (0, 10), (6, 10), (6, 0)]
    flat_u = collapse_box_detours(u)
    assert flat_u == [(0, 0), (6, 0)] or count_turns(flat_u) == 0, flat_u
    assert subpath_len_check(flat_u) == 6, flat_u

    stair2 = [(0, 0), (0, 8), (1, 8), (1, 9), (2, 9), (2, 16), (3, 16)]
    flat2 = orthogonal_smooth(stair2, cost0)
    assert count_turns(flat2) == 1, flat2

    # --- L-corner fillet ---
    verts = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]
    r = 0.5
    filleted = fillet_l_path(verts, radius=r, arc_points=8, spacing=0.05)
    assert len(filleted) > 3, filleted
    assert corner_is_rounded(filleted, (2.0, 0.0))
    assert math.hypot(filleted[0][0] - 0.0, filleted[0][1] - 0.0) < 1e-6
    assert math.hypot(filleted[-1][0] - 2.0, filleted[-1][1] - 2.0) < 1e-6
    assert any(math.hypot(p[0] - 1.5, p[1] - 0.0) < 0.02 for p in filleted)
    assert any(math.hypot(p[0] - 2.0, p[1] - 0.5) < 0.02 for p in filleted)
    mid = (1.5 + r / math.sqrt(2), 0.5 - r / math.sqrt(2))  # O=(1.5,0.5), −π/4
    assert any(math.hypot(p[0] - mid[0], p[1] - mid[1]) < 0.08 for p in filleted), filleted
    for p in filleted:
        if p[0] < 1.4:
            assert abs(p[1]) < 1e-6, p
        if p[1] > 0.6:
            assert abs(p[0] - 2.0) < 1e-6, p

    verts_r = [(0.0, 0.0), (2.0, 0.0), (2.0, -2.0)]
    fr = fillet_l_path(verts_r, radius=0.4, arc_points=6, spacing=0.05)
    assert corner_is_rounded(fr, (2.0, 0.0))
    assert any(p[1] < -0.05 for p in fr)

    straight = fillet_l_path([(0.0, 0.0), (3.0, 0.0)], radius=0.5)
    assert all(abs(p[1]) < 1e-9 for p in straight)

    print("OK: freer L + box detours collapsed + L-corner arcs")
    print(f"  stair->{out0}")
    print(f"  box->{flat}")
    print(f"  u->{flat_u}")
    print(f"  stair2->{flat2}")
    print(f"  fillet pts={len(filleted)} mid≈{mid}")


if __name__ == "__main__":
    main()
