"""Fake mocap_optitrack output for testing without OptiTrack.

Publishes geometry_msgs/PoseStamped on <name>/pose (frame 'world') moving
along a square of side ``side`` [m] at ``speed`` [m/s] with Gaussian jitter.
It stays at the start for ``start_delay`` [s] before moving, and with
``laps`` > 0 it stops (jitter only) after that many laps.
"""

import random

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def square_point(s, side):
    s = s % (4.0 * side)
    if s < side:
        return s, 0.0
    if s < 2 * side:
        return side, s - side
    if s < 3 * side:
        return 3 * side - s, side
    return 0.0, 4 * side - s


class FakeMocap(Node):

    def __init__(self):
        super().__init__('fake_mocap_publisher')
        self.declare_parameter('rigid_bodies', ['Robot_1'])
        self.declare_parameter('rate', 120.0)
        self.declare_parameter('side', 1.0)
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('noise', 0.0005)
        self.declare_parameter('laps', 0.0)
        self.declare_parameter('start_delay', 0.0)
        self.names = list(self.get_parameter('rigid_bodies').value)
        self.rate = float(self.get_parameter('rate').value)
        self.side = float(self.get_parameter('side').value)
        self.speed = float(self.get_parameter('speed').value)
        self.noise = float(self.get_parameter('noise').value)
        laps = float(self.get_parameter('laps').value)
        self.start_delay = float(self.get_parameter('start_delay').value)
        self.s_end = laps * 4.0 * self.side if laps > 0 else None
        self.pubs = [self.create_publisher(PoseStamped, f'{n}/pose', 1000) for n in self.names]
        self.t0 = self.get_clock().now()
        self.create_timer(1.0 / self.rate, self._tick)

    def _tick(self):
        now = self.get_clock().now()
        s = max(0.0, (now - self.t0).nanoseconds * 1e-9 - self.start_delay) * self.speed
        if self.s_end is not None and s >= self.s_end:
            s = self.s_end - 1e-9
        for i, pub in enumerate(self.pubs):
            x, y = square_point(s, self.side)
            msg = PoseStamped()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = 'world'
            msg.pose.position.x = x + 2.0 * i + random.gauss(0.0, self.noise)
            msg.pose.position.y = y + random.gauss(0.0, self.noise)
            msg.pose.position.z = 0.5 + random.gauss(0.0, self.noise)
            msg.pose.orientation.w = 1.0
            pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeMocap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
