"""Travelled-distance visualizer for mocap_optitrack rigid bodies (ROS 2).

Subscribes to the pose topics published by
mocap_optitrack (``<body>/pose`` as geometry_msgs/PoseStamped, or
``<body>/Odom`` as nav_msgs/Odometry) and, per rigid body, publishes:

  <body>/distance       std_msgs/Float64   cumulative path length [m]
  <body>/displacement   std_msgs/Float64   straight-line distance from start [m]
  <body>/path           nav_msgs/Path      travelled trajectory
  ~/markers             visualization_msgs/MarkerArray (text, start point, start->current line)

Service ``~/reset`` (std_srvs/Trigger) restarts all measurements.

The terminal running the node shows a live dashboard (TTY), or a one-line
summary per ``log_period`` when the output is not a terminal.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA, Float64
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from mocap_distance_viz.terminal import BodyRow, Dashboard, is_tty, summary_line
from mocap_distance_viz.tracker import BodyTracker

PALETTE = [
    (0.10, 0.60, 1.00),
    (1.00, 0.45, 0.10),
    (0.20, 0.80, 0.30),
    (0.90, 0.20, 0.50),
    (0.60, 0.40, 1.00),
    (0.95, 0.85, 0.15),
]


class Body:
    """Tracker + ROS state (path, publishers) for one rigid body."""

    def __init__(self, node, name, color, tracker):
        self.name = name
        self.color = color
        self.tracker = tracker
        self.frame_id = 'world'
        self.stamp = None
        self.path = []
        self.path_dirty = False
        self.last_rx = None  # wall time of the last valid sample
        self.pub_distance = node.create_publisher(Float64, f'{name}/distance', 10)
        self.pub_displacement = node.create_publisher(Float64, f'{name}/displacement', 10)
        # Transient local so that late subscribers get the full path.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_path = node.create_publisher(Path, f'{name}/path', latched)

    def reset(self):
        self.tracker.reset()
        self.path = []
        self.path_dirty = True
        self.last_rx = None


class DistanceVizNode(Node):

    def __init__(self):
        super().__init__('mocap_distance_viz')

        self.declare_parameter('rigid_bodies', ['Robot_1'])
        self.declare_parameter('msg_type', 'pose')
        self.declare_parameter('topic_suffix', '')
        self.declare_parameter('planar', False)
        self.declare_parameter('min_step', 0.005)
        self.declare_parameter('max_step', 0.5)
        self.declare_parameter('max_path_length', 20000)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('path_publish_rate', 2.0)
        self.declare_parameter('log_period', 1.0)
        self.declare_parameter('display', 'auto')
        self.declare_parameter('display_rate', 5.0)
        self.declare_parameter('stale_timeout', 0.5)
        self.declare_parameter('text_height', 0.12)
        self.declare_parameter('text_offset_z', 0.35)

        def param(name):
            return self.get_parameter(name).value

        names = list(param('rigid_bodies'))
        msg_type = str(param('msg_type')).lower()
        suffix = param('topic_suffix')
        if msg_type not in ('pose', 'odom'):
            raise ValueError(f"msg_type must be 'pose' or 'odom', got '{msg_type}'")
        if not suffix:
            suffix = 'pose' if msg_type == 'pose' else 'Odom'

        planar = bool(param('planar'))
        min_step = float(param('min_step'))
        max_step = float(param('max_step'))
        self.max_path_len = int(param('max_path_length'))
        self.text_height = float(param('text_height'))
        self.text_offset_z = float(param('text_offset_z'))
        publish_rate = float(param('publish_rate'))
        path_rate = float(param('path_publish_rate'))
        log_period = float(param('log_period'))
        display = str(param('display')).lower()
        display_rate = float(param('display_rate'))
        if display not in ('auto', 'live', 'log', 'off'):
            raise ValueError(f"display must be auto, live, log or off, got '{display}'")
        if display == 'auto':
            display = 'live' if is_tty() else 'log'
        self.live = display == 'live'
        self.dashboard = Dashboard(
            self.get_name(),
            f'ros2 service call {self.get_fully_qualified_name()}/reset std_srvs/srv/Trigger',
            stale_timeout=float(param('stale_timeout')), color=self.live)

        # Single-threaded executor, so no lock is needed.
        self.bodies = []
        for i, name in enumerate(names):
            body = Body(self, name, PALETTE[i % len(PALETTE)],
                        BodyTracker(name, planar, min_step, max_step))
            self.bodies.append(body)
            topic = f'{name}/{suffix}'
            if msg_type == 'pose':
                self.create_subscription(
                    PoseStamped, topic,
                    lambda msg, b=body: self._feed(b, msg.header, msg.pose), 100)
            else:
                self.create_subscription(
                    Odometry, topic,
                    lambda msg, b=body: self._feed(b, msg.header, msg.pose.pose), 100)
            self.get_logger().info(f'[{name}] subscribing {msg_type} on "{topic}"')

        self.pub_markers = self.create_publisher(MarkerArray, '~/markers', 1)
        self.create_service(Trigger, '~/reset', self._on_reset)

        self.create_timer(1.0 / max(publish_rate, 0.1), self._publish)
        self.create_timer(1.0 / max(path_rate, 0.1), self._publish_path)
        self.get_logger().info(
            f'planar={planar} min_step={min_step:.4f} m max_step={max_step:.3f} m')

        if self.live:
            self.create_timer(1.0 / max(display_rate, 0.1), self._display_live)
        elif display == 'log' and log_period > 0.0:
            self.create_timer(log_period, self._display_log)

    def _feed(self, body, header, pose):
        p = pose.position
        point = (p.x, p.y, p.z)
        if any(math.isnan(v) for v in point):
            return
        added = body.tracker.update(point)
        body.last_rx = time.time()
        if header.frame_id:
            body.frame_id = header.frame_id
        body.stamp = header.stamp
        if added:
            ps = PoseStamped()
            ps.header.frame_id = body.frame_id
            ps.header.stamp = header.stamp
            ps.pose = pose
            body.path.append(ps)
            if 0 < self.max_path_len < len(body.path):
                del body.path[:len(body.path) - self.max_path_len]
            body.path_dirty = True

    def _on_reset(self, _request, response):
        for body in self.bodies:
            body.reset()
        self.dashboard.reset()
        clear = Marker()
        clear.action = Marker.DELETEALL
        self.pub_markers.publish(MarkerArray(markers=[clear]))
        response.success = True
        response.message = 'distance reset for: ' + ', '.join(b.name for b in self.bodies)
        # A log line would break the live dashboard redraw.
        if not self.live:
            self.get_logger().info(response.message)
        return response

    def _publish(self):
        markers = []
        for idx, body in enumerate(self.bodies):
            tr = body.tracker
            if not tr.has_data:
                continue
            disp = tr.displacement()
            body.pub_distance.publish(Float64(data=tr.distance))
            body.pub_displacement.publish(Float64(data=disp))
            markers.extend(self._markers_for(idx, body, disp))
        if markers:
            self.pub_markers.publish(MarkerArray(markers=markers))

    def _publish_path(self):
        for body in self.bodies:
            if not body.path_dirty or body.stamp is None:
                continue
            path = Path()
            path.header.frame_id = body.frame_id
            path.header.stamp = body.stamp
            path.poses = list(body.path)
            body.path_dirty = False
            body.pub_path.publish(path)

    def _markers_for(self, idx, body, disp):
        tr = body.tracker
        r, g, b = body.color
        base_id = idx * 10

        def make(mid, mtype, color):
            m = Marker()
            m.header.frame_id = body.frame_id
            m.header.stamp = body.stamp
            m.ns = body.name
            m.id = base_id + mid
            m.type = mtype
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.color = color
            return m

        solid = ColorRGBA(r=r, g=g, b=b, a=1.0)
        cx, cy, cz = tr.current
        sx, sy, sz = tr.start
        cur = Point(x=cx, y=cy, z=cz)
        start = Point(x=sx, y=sy, z=sz)

        text = make(0, Marker.TEXT_VIEW_FACING, ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0))
        text.pose.position = Point(x=cx, y=cy, z=cz + self.text_offset_z)
        text.scale.z = self.text_height
        text.text = f'{body.name}\npath: {tr.distance:.3f} m\ndisp: {disp:.3f} m'

        start_m = make(1, Marker.SPHERE, solid)
        start_m.pose.position = start
        start_m.scale.x = start_m.scale.y = start_m.scale.z = 0.06

        cur_m = make(2, Marker.SPHERE, solid)
        cur_m.pose.position = cur
        cur_m.scale.x = cur_m.scale.y = cur_m.scale.z = 0.05

        line = make(3, Marker.LINE_LIST, ColorRGBA(r=r, g=g, b=b, a=0.5))
        line.scale.x = 0.01
        line.points = [start, cur]

        return [text, start_m, cur_m, line]

    def _rows(self):
        now = time.time()
        rows = []
        for body in self.bodies:
            tr = body.tracker
            if not tr.has_data:
                rows.append(BodyRow(body.name, body.color, False))
                continue
            rows.append(BodyRow(
                body.name, body.color, True, tr.distance, tr.displacement(),
                tr.current, now - body.last_rx, tr.jumps))
        return rows

    def _display_live(self):
        self.dashboard.draw(self._rows())

    def _display_log(self):
        self.get_logger().info(summary_line(self._rows(), self.dashboard))


def main(args=None):
    rclpy.init(args=args)
    node = DistanceVizNode()
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
