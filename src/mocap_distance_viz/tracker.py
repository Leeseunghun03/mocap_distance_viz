"""ROS-independent travelled-distance bookkeeping for one rigid body."""

from __future__ import division

import math


def distance(a, b, planar=False):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = 0.0 if planar else b[2] - a[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


class BodyTracker(object):
    """Accumulates path length of a stream of (x, y, z) points.

    - Steps shorter than ``min_step`` are not accumulated (mocap jitter).
      The anchor is kept, so slow motion still adds up once it exceeds min_step.
    - Steps longer than ``max_step`` are treated as tracking jumps
      (occlusion / re-acquisition): re-anchored without counting.
    """

    def __init__(self, name, planar=False, min_step=0.005, max_step=0.5):
        self.name = name
        self.planar = planar
        self.min_step = min_step
        self.max_step = max_step
        self.reset()

    def reset(self):
        self.start = None      # first valid point
        self.anchor = None     # last point added to the distance
        self.current = None    # latest point
        self.distance = 0.0
        self.jumps = 0         # rejected jumps since reset

    @property
    def has_data(self):
        return self.current is not None

    def displacement(self):
        if self.start is None or self.current is None:
            return 0.0
        return distance(self.start, self.current, self.planar)

    def update(self, point):
        """Feed one point. Returns True when it was added to the path."""
        if any(math.isnan(v) or math.isinf(v) for v in point):
            return False
        point = tuple(float(v) for v in point)
        self.current = point

        if self.start is None:
            self.start = point
            self.anchor = point
            return True

        step = distance(self.anchor, point, self.planar)
        if self.max_step > 0.0 and step > self.max_step:
            self.anchor = point
            self.jumps += 1
            return False
        if step < self.min_step:
            return False

        self.distance += step
        self.anchor = point
        return True
