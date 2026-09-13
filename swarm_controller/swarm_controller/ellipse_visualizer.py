"""
ellipse_visualizer.py
======================
Publishes the circle-to-ellipse experiment's initial circle and target
ellipse as RViz line-strip markers, in the same 'world' frame as the robot
markers, so the target shape is visible alongside the live robots.

Coordinates are real metres directly (the packaged policy's fixed 2 m x 2 m,
[-1, 1] m workspace) — no sim-coordinate scaling, unlike obstacle_visualizer.

The initial circle (center, radius) is the one described in
circle_to_ellipse_controller/README.md section 1. The target ellipse is
read from target_ellipse.json (center, semiaxes, angle_rad) using the same
parametric-loop + rotation-matrix construction as controller.py's
CircleToEllipseController.__init__, so what's drawn here matches exactly
what the policy was trained against.

ROS Parameters
--------------
controller_package_path : str   Path to circle_to_ellipse_controller dir
                                 (must contain target_ellipse.json).
initial_circle_center   : list[float]  [x, y] metres (default [-0.5, -0.5]).
initial_circle_radius   : float        metres (default 0.169706).
"""

from pathlib import Path
import json

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class EllipseVisualizer(Node):
    def __init__(self):
        super().__init__('ellipse_visualizer')

        self.declare_parameter(
            'controller_package_path',
            '/home/eason/Swarm/circle_to_ellipse_controller')
        self.declare_parameter('initial_circle_center', [-0.5, -0.5])
        self.declare_parameter('initial_circle_radius', 0.169706)

        pkg_path = self.get_parameter('controller_package_path').value
        self._circle_center = tuple(self.get_parameter('initial_circle_center').value)
        self._circle_radius = self.get_parameter('initial_circle_radius').value

        target = json.loads((Path(pkg_path) / 'target_ellipse.json').read_text())
        center = np.asarray(target['center'], dtype=float)
        axes   = np.asarray(target['semiaxes'], dtype=float)
        angle  = float(target['angle_rad'])
        rotation = np.array([[np.cos(angle), -np.sin(angle)],
                             [np.sin(angle), np.cos(angle)]])
        phase = np.linspace(0, 2 * np.pi, 40, endpoint=False)
        self._ellipse_points = center + (
            np.column_stack([np.cos(phase), np.sin(phase)]) * axes
        ) @ rotation.T

        self._pub = self.create_publisher(MarkerArray, '/ellipse_markers', 10)
        self._timer = self.create_timer(1.0, self._publish)
        self.get_logger().info(
            f'ellipse_visualizer ready — initial circle center={self._circle_center} '
            f'r={self._circle_radius} m, target ellipse center={tuple(center)} m')

    @staticmethod
    def _line_strip(now, ns, marker_id, points_xy, r, g, b, width=0.01):
        m = Marker()
        m.header.stamp = now
        m.header.frame_id = 'world'
        m.ns = ns
        m.id = marker_id
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = width
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
        for x, y in points_xy:
            pt = Point(x=float(x), y=float(y), z=0.02)
            m.points.append(pt)
        # Close the loop back to the first point.
        if points_xy:
            x0, y0 = points_xy[0]
            m.points.append(Point(x=float(x0), y=float(y0), z=0.02))
        return m

    def _publish(self):
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        cx, cy = self._circle_center
        phase = np.linspace(0, 2 * np.pi, 40, endpoint=False)
        circle_points = [(cx + self._circle_radius * np.cos(p),
                          cy + self._circle_radius * np.sin(p)) for p in phase]
        arr.markers.append(self._line_strip(
            now, 'initial_circle', 0, circle_points, 0.2, 0.6, 1.0))  # light blue

        arr.markers.append(self._line_strip(
            now, 'target_ellipse', 1, self._ellipse_points.tolist(), 1.0, 0.85, 0.0))  # gold

        self._pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = EllipseVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
