"""Unit tests for EDT and simple clearance A* planner."""

import unittest

import numpy as np

from skill23_custom_nav.planning import (
    astar, compress_collinear, drop_start_backtrack, plan_path,
    ridge_snap_radius_cells, snap_to_edt_ridge, _turn_abs,
)
from skill23_custom_nav.utils import compute_edt


class TestEdtAstar(unittest.TestCase):
    def test_ridge_snap_radius_cells_follows_map_resolution(self):
        self.assertEqual(ridge_snap_radius_cells(0.10, 0.05), 2)
        self.assertEqual(ridge_snap_radius_cells(0.10, 0.02), 5)
        self.assertEqual(ridge_snap_radius_cells(0.10, 0.10), 1)
        self.assertEqual(ridge_snap_radius_cells(0.10, 0.20), 1)
        self.assertEqual(ridge_snap_radius_cells(0.10, 0.005), 16)

    def test_compute_edt_open_field(self):
        occ = np.zeros((5, 5), dtype=np.int16)
        occ[2, 2] = 100
        edt = compute_edt(occ, resolution=0.1)
        self.assertAlmostEqual(edt[2, 2], 0.0, places=5)
        self.assertGreater(edt[0, 0], 0.2)

    def test_compute_edt_wall(self):
        occ = np.zeros((3, 5), dtype=np.int16)
        occ[:, 0] = 100
        edt = compute_edt(occ, resolution=0.05)
        self.assertAlmostEqual(edt[1, 0], 0.0)
        self.assertAlmostEqual(edt[1, 1], 0.05, places=3)

    def test_compress_collinear(self):
        cells = [(0, 0), (1, 0), (2, 0), (2, 1)]
        out = compress_collinear(cells)
        self.assertEqual(out, [(0, 0), (2, 0), (2, 1)])

    def test_astar_simple(self):
        occ = np.zeros((10, 10), dtype=np.int16)
        occ[5, :] = 100
        occ[5, 4] = 0
        edt = compute_edt(occ, 0.1)
        path = astar((0, 0), (9, 9), occ, edt, min_clearance_m=0.0)
        self.assertIsNotNone(path)
        assert path is not None
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (9, 9))

    def test_astar_blocked(self):
        occ = np.full((5, 5), 100, dtype=np.int16)
        edt = compute_edt(occ, 0.1)
        path = astar((0, 0), (4, 4), occ, edt)
        self.assertIsNone(path)

    def test_plan_open_room_finds_path(self):
        res = 0.05
        occ = np.zeros((40, 40), dtype=np.int16)
        edt = compute_edt(occ, res)
        poses = plan_path(
            0.25, 0.25, 1.55, 1.05, 0.0,
            occ, edt, 0.0, 0.0, res,
            min_clearance_m=0.0, clearance_weight=4.0,
        )
        self.assertIsNotNone(poses)
        assert poses is not None
        self.assertGreaterEqual(len(poses), 1)

    def test_plan_corridor_prefers_center(self):
        res = 0.05
        w, h = 40, 60
        occ = np.zeros((h, w), dtype=np.int16)
        occ[:, 5] = 100
        occ[:, 25] = 100
        edt = compute_edt(occ, res)
        poses = plan_path(
            0.75, 0.5, 0.75, 2.5, 0.0,
            occ, edt, 0.0, 0.0, res,
            min_clearance_m=0.05, clearance_weight=8.0,
        )
        self.assertIsNotNone(poses)
        assert poses is not None
        mid = 15.5 * res
        for x, _y, _yaw in poses:
            self.assertLess(abs(x - mid), 0.28, msg=f'path x={x} not near center {mid}')

    def test_plan_horizontal_corridor_not_tilted(self):
        """Off-center start must not leave a tilted chord; long leg stays mid-Y."""
        res = 0.05
        w, h = 80, 30
        occ = np.zeros((h, w), dtype=np.int16)
        occ[5, :] = 100
        occ[24, :] = 100
        edt = compute_edt(occ, res)
        # Start near top wall, goal near bottom — old bug made a tilted chord.
        poses = plan_path(
            0.40, 0.40, 3.60, 1.05, 0.0,
            occ, edt, 0.0, 0.0, res,
            min_clearance_m=0.05, clearance_weight=8.0,
            corner_clearance_m=0.20,
        )
        self.assertIsNotNone(poses)
        assert poses is not None
        mid_y = 14.5 * res  # between walls at rows 5 and 24
        # Long corridor travel should sit on the EDT ridge (flat mid-Y).
        long_seg = None
        for i in range(len(poses) - 1):
            a, b = poses[i], poses[i + 1]
            if abs(b[0] - a[0]) > 1.5 and abs(b[1] - a[1]) < 0.15:
                long_seg = (a, b)
                break
        self.assertIsNotNone(long_seg, msg='expected a long near-horizontal corridor leg')
        assert long_seg is not None
        a, b = long_seg
        self.assertLess(abs(a[1] - mid_y), 0.12)
        self.assertLess(abs(b[1] - mid_y), 0.12)
        self.assertLess(abs(a[1] - b[1]), 0.06, msg='long leg still tilted')
        self.assertTrue(all(x >= -0.01 for x, _y, _ in poses))

    def test_plan_never_crosses_walls(self):
        """Post-process must not punch chords through occupied cells."""
        res = 0.05
        w, h = 80, 60
        occ = np.zeros((h, w), dtype=np.int16)
        # Vertical wall with a top gap — must go around, never through.
        occ[15:55, 40] = 100
        edt = compute_edt(occ, res)
        poses = plan_path(
            1.0, 2.0, 3.0, 2.0, 0.0,
            occ, edt, 0.0, 0.0, res,
            min_clearance_m=0.05, clearance_weight=8.0,
            corner_clearance_m=0.20,
        )
        self.assertIsNotNone(poses)
        assert poses is not None
        from skill23_custom_nav.planning import _world_line_clear
        for i in range(len(poses) - 1):
            a, b = poses[i], poses[i + 1]
            self.assertTrue(
                _world_line_clear(
                    a[0], a[1], b[0], b[1], occ, edt,
                    occupied_threshold=50,
                    min_clearance_m=0.05,
                    extra_blocked=None,
                    map_origin_x=0.0,
                    map_origin_y=0.0,
                    map_resolution=res,
                    los_sample_step_m=0.05,
                ),
                msg=f'segment through wall: ({a[0]:.2f},{a[1]:.2f})→({b[0]:.2f},{b[1]:.2f})',
            )
        # Must actually go around (not a single chord through the wall).
        self.assertGreater(len(poses), 2)

    def test_snap_to_edt_ridge_no_backward_snap(self):
        """Ridge snap must not jump to a higher-EDT cell behind the path."""
        res = 0.05
        w, h = 40, 20
        occ = np.zeros((h, w), dtype=np.int16)
        occ[:, 8] = 100
        occ[:, 31] = 100
        edt = compute_edt(occ, res)
        cells = [(12, 15), (13, 15), (14, 15), (15, 15), (16, 15)]
        # Fake a higher ridge one cell behind the middle waypoint.
        edt[15, 11] = float(edt[15, 13]) + 0.08
        out = snap_to_edt_ridge(
            cells, occ, edt,
            occupied_threshold=50,
            min_clearance_m=0.0,
            extra_blocked=None,
            radius_cells=4,
            map_origin_x=0.0,
            map_origin_y=0.0,
            map_resolution=res,
        )
        xs = [c[0] for c in out]
        for i in range(len(xs) - 1):
            self.assertGreaterEqual(
                xs[i + 1], xs[i],
                msg=f'backward snap at index {i}: {out[i]} -> {out[i + 1]}',
            )

    def test_drop_start_backtrack_removes_start_hook(self):
        """Pinned start + cell-centre wp1 must not leave a near-180 deg hook."""
        res = 0.05
        w, h = 40, 20
        occ = np.zeros((h, w), dtype=np.int16)
        edt = compute_edt(occ, res)
        sx, sy = 0.50, 0.50
        # wp1 slightly behind start; wp2 ahead — classic cell-centre hook.
        poses = [
            (sx, sy, 0.0),
            (sx - 0.04, sy, 0.0),
            (sx + 0.30, sy + 0.05, 0.0),
            (sx + 0.60, sy + 0.05, 0.0),
        ]
        before = _turn_abs((sx, sy), poses[1][:2], poses[2][:2])
        self.assertGreater(before, 2.0)
        out = drop_start_backtrack(
            list(poses), occ, edt,
            occupied_threshold=50,
            min_clearance_m=0.0,
            extra_blocked=None,
            map_origin_x=0.0,
            map_origin_y=0.0,
            map_resolution=res,
            los_sample_step_m=0.05,
        )
        self.assertEqual(len(out), len(poses) - 1)
        after = _turn_abs(out[0][:2], out[1][:2], out[2][:2])
        self.assertLess(after, 1.0)


if __name__ == '__main__':
    unittest.main()
