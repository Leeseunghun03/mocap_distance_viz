# -*- coding: utf-8 -*-
from __future__ import division

import io
import os
import unittest

from mocap_distance_viz.terminal import BodyRow, Dashboard, summary_line

BLUE = (0.1, 0.6, 1.0)


def rows(distance=1.0, age=0.01, jumps=0, has_data=True):
    return [
        BodyRow('Robot_1', BLUE, has_data, distance, 0.5, (1.0, 2.0, 0.5), age, jumps),
        BodyRow('Robot_2', BLUE, False),
    ]


class TestDashboard(unittest.TestCase):

    def dash(self):
        return Dashboard('mocap_distance_viz', 'rosservice call /x/reset', color=False)

    def test_wide_layout(self):
        d = self.dash()
        lines = d.render(rows(), now=d.start + 83, width=120)
        text = u'\n'.join(lines)
        self.assertIn(u'elapsed 00:01:23', lines[1])
        self.assertIn(u'POSITION', lines[3])
        self.assertIn(u'Robot_1', lines[4])
        self.assertIn(u'1.000', lines[4])
        self.assertIn(u'( 1.000,  2.000,  0.500)', lines[4])
        neg = d.render([BodyRow('a', BLUE, True, 0, 0, (-0.0001, 0, 0), 0.0)], width=120)[4]
        self.assertIn(u'( 0.000,', neg)
        self.assertIn(u'tracking', lines[4])
        self.assertIn(u'waiting', lines[5])
        self.assertIn(u'rosservice call /x/reset', text)
        self.assertNotIn(u'\033', text)

    def test_narrow_layout_drops_position(self):
        d = self.dash()
        lines = d.render(rows(), width=60)
        self.assertNotIn(u'POSITION', lines[3])
        self.assertTrue(all(len(line) <= 60 for line in lines), lines)

    def test_lost_and_jumps(self):
        d = self.dash()
        line = d.render(rows(age=2.14, jumps=2), width=120)[4]
        self.assertIn(u'lost 2.1s', line)
        self.assertIn(u'2 jumps', line)

    def test_speed(self):
        d = self.dash()
        t = 1000.0
        self.assertIsNone(d.speed('b', 0.0, t))
        for i in range(1, 21):
            spd = d.speed('b', 0.05 * i, t + 0.1 * i)
        self.assertAlmostEqual(spd, 0.5, places=6)

    def test_reset_clears_speed_and_sets_event(self):
        d = self.dash()
        d.speed('b', 0.0, 1.0)
        d.reset(now=50.0)
        self.assertEqual(d.history, {})
        self.assertEqual(d.start, 50.0)
        self.assertIn(u'reset', d.render(rows(), now=51.0, width=120)[-1])

    def test_draw_redraws_in_place(self):
        out = io.BytesIO()
        d = Dashboard('t', 'r', color=False, stream=out)
        d.draw(rows())
        first = out.getvalue()
        d.draw(rows())
        second = out.getvalue()[len(first):]
        self.assertTrue(second.startswith(b'\x1b[%dF' % d.drawn_lines))
        self.assertEqual(first.count(b'\n'), d.drawn_lines)

    def test_color_codes(self):
        d = Dashboard('t', 'r', color=True)
        if 'NO_COLOR' in os.environ:
            self.skipTest('NO_COLOR set')
        line = d.render(rows(), width=120)[4]
        self.assertIn(u'\033[', line)

    def test_summary_line(self):
        d = self.dash()
        s = summary_line(rows(age=3.0), d)
        self.assertIn('Robot_1: path=1.000 m disp=0.500 m', s)
        self.assertIn('LOST 3.0s', s)
        self.assertIn('Robot_2: waiting', s)


if __name__ == '__main__':
    unittest.main()
