"""Unit tests for EDT and simple clearance A* planner."""

import unittest

import numpy as np

from skill23_custom_nav.planning import astar, compress_collinear, plan_path
from skill23_custom_nav.utils import compute_edt


class TestEdtAstar(unittest.TestCase):
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


if __name__ == '__main__':
    unittest.main()
