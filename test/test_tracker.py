#!/usr/bin/env python3
from __future__ import division

import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))

from mocap_distance_viz.tracker import BodyTracker  # noqa: E402


def feed(tr, pts):
    for p in pts:
        tr.update(p if len(p) == 3 else (p[0], p[1], 0.0))


class TestBodyTracker(unittest.TestCase):

    def test_straight_line(self):
        tr = BodyTracker('b')
        feed(tr, [(i * 0.01, 0.0) for i in range(101)])
        self.assertAlmostEqual(tr.distance, 1.0, places=6)
        self.assertAlmostEqual(tr.displacement(), 1.0, places=6)

    def test_jitter_is_not_accumulated(self):
        random.seed(0)
        tr = BodyTracker('b')
        feed(tr, [(random.gauss(0, 0.0005), random.gauss(0, 0.0005)) for _ in range(5000)])
        self.assertLess(tr.distance, 0.01)

    def test_slow_motion_still_counts(self):
        # 1 mm per sample (< min_step) must still add up via the anchor.
        tr = BodyTracker('b', min_step=0.005)
        feed(tr, [(i * 0.001, 0.0) for i in range(1001)])
        self.assertAlmostEqual(tr.distance, 1.0, delta=0.005)

    def test_jump_is_rejected(self):
        tr = BodyTracker('b', max_step=0.5)
        feed(tr, [(0.0, 0.0), (0.1, 0.0), (3.0, 0.0), (3.1, 0.0)])
        self.assertAlmostEqual(tr.distance, 0.2, places=6)
        self.assertEqual(tr.jumps, 1)

    def test_planar_ignores_z(self):
        tr = BodyTracker('b', planar=True)
        feed(tr, [(0.0, 0.0, 0.0), (0.0, 0.0, 0.3)])
        self.assertEqual(tr.distance, 0.0)
        tr = BodyTracker('b', planar=False)
        feed(tr, [(0.0, 0.0, 0.0), (0.0, 0.0, 0.3)])
        self.assertAlmostEqual(tr.distance, 0.3, places=6)

    def test_square_loop(self):
        tr = BodyTracker('b')
        pts = [(i * 0.01, 0) for i in range(100)] + [(1, i * 0.01) for i in range(100)] \
            + [(1 - i * 0.01, 1) for i in range(100)] + [(0, 1 - i * 0.01) for i in range(101)]
        feed(tr, pts)
        self.assertAlmostEqual(tr.distance, 4.0, places=6)
        self.assertLess(tr.displacement(), 1e-9)

    def test_nan_and_reset(self):
        tr = BodyTracker('b')
        feed(tr, [(0, 0), (0.1, 0), (float('nan'), 0), (float('inf'), 0)])
        self.assertAlmostEqual(tr.distance, 0.1, places=9)
        self.assertEqual(tr.current, (0.1, 0.0, 0.0))
        tr.reset()
        self.assertEqual(tr.distance, 0.0)
        self.assertFalse(tr.has_data)
        self.assertEqual(tr.displacement(), 0.0)


if __name__ == '__main__':
    unittest.main()
