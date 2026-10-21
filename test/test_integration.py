#!/usr/bin/env python3
from __future__ import division

import time
import unittest

import rospy
import rostest
from nav_msgs.msg import Path
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

BODIES = ['Robot_1', 'Robot_2']


class TestDistanceViz(unittest.TestCase):

    def setUp(self):
        self.data = {}
        for b in BODIES:
            rospy.Subscriber(b + '/distance', Float64, self._store, b + '/distance')
            rospy.Subscriber(b + '/displacement', Float64, self._store, b + '/displacement')
            rospy.Subscriber(b + '/path', Path, self._store, b + '/path')
        rospy.Subscriber('mocap_distance_viz/markers', MarkerArray, self._store, 'markers')

    def _store(self, msg, key):
        self.data[key] = msg

    def wait_for(self, key, timeout=10.0):
        end = time.time() + timeout
        while key not in self.data and time.time() < end and not rospy.is_shutdown():
            time.sleep(0.05)
        self.assertIn(key, self.data, 'no message on ' + key)
        return self.data[key]

    def test_distance_pipeline(self):
        self.wait_for('Robot_1/distance')
        # Wait for the fake lap to finish.
        time.sleep(7.0)
        for b in BODIES:
            dist = self.data[b + '/distance'].data
            disp = self.data[b + '/displacement'].data
            rospy.loginfo('%s distance=%.4f displacement=%.4f', b, dist, disp)
            self.assertAlmostEqual(dist, 4.0, delta=0.15)
            self.assertLess(disp, 0.05)
            path = self.wait_for(b + '/path')
            self.assertGreater(len(path.poses), 100)
            self.assertEqual(path.header.frame_id, 'world')

        # Stationary jitter must not grow the distance.
        before = self.data['Robot_1/distance'].data
        time.sleep(2.0)
        self.assertLess(self.data['Robot_1/distance'].data - before, 0.005)

        # Path is latched.
        late = rospy.wait_for_message('Robot_1/path', Path, timeout=3.0)
        self.assertGreater(len(late.poses), 100)

        markers = self.wait_for('markers').markers
        texts = [m.text for m in markers if m.type == Marker.TEXT_VIEW_FACING]
        self.assertEqual(len(texts), 2)
        self.assertTrue(texts[0].startswith('Robot_1\npath: '))

        rospy.wait_for_service('mocap_distance_viz/reset', timeout=5.0)
        res = rospy.ServiceProxy('mocap_distance_viz/reset', Trigger)()
        self.assertTrue(res.success)
        time.sleep(1.5)
        for b in BODIES:
            self.assertLess(self.data[b + '/distance'].data, 0.005)
            self.assertLess(self.data[b + '/displacement'].data, 0.005)


if __name__ == '__main__':
    rospy.init_node('test_distance_viz')
    rostest.rosrun('mocap_distance_viz', 'test_integration', TestDistanceViz)
