"""
gaussian_visualizer.py
========================
Publishes the Gaussian-cluster-transfer experiment's nominal initial/target
distribution circles and the policy's actual 40 target reference points as
RViz markers, in the same 'world' frame as the robot markers, so the target
is visible alongside the live robots.

Coordinates are real metres directly (the packaged policy's fixed 2 m x 2 m,
[-1, 1] m workspace) — no sim-coordinate scaling, unlike obstacle_visualizer.

The nominal initial/target circles (center, std) are the ones described in
gaussian_cluster_transfer_controller/README.md section 1 — 1-std rings
around N((-0.5, -0.5), 0.12^2 I) and N((0.5, 0.5), 0.12^2 I). The 40 gold
dots are target_gaussian.json's actual reference_points: the exact points
the packaged policy's target moments were computed from, which (per that
README) sample to a center only approximately equal to the nominal (0.5,
0.5) — plotting both shows that gap directly.

ROS Parameters
--------------
controller_package_path : str   Path to gaussian_cluster_transfer_controller
                                 dir (must contain target_gaussian.json).
"""

from pathlib import Path
import json

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class GaussianVisualizer(Node):
    def __init__(self):
        super().__init__('gaussian_visualizer')

        self.declare_parameter(
            'controller_package_path',
            '/home/eason/Swarm/gaussian_cluster_transfer_controller')

        pkg_path = self.get_parameter('controller_package_path').value

        target = json.loads((Path(pkg_path) / 'target_gaussian.json').read_text())
        self._nominal_initial_center = (-0.5, -0.5)
        self._nominal_target_center = tuple(float(c) for c in target['center'])
        self._std = float(target['std'])
        self._reference_points = np.asarray(target['reference_points'], dtype=float)

        self._pub = self.create_publisher(MarkerArray, '/gaussian_markers', 10)
        self._timer = self.create_timer(1.0, self._publish)
        self.get_logger().info(
            f'gaussian_visualizer ready — nominal initial center={self._nominal_initial_center} '
            f'std={self._std} m, nominal target center={self._nominal_target_center} m, '
            f'{len(self._reference_points)} target reference points')

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

    @staticmethod
    def _points(now, ns, marker_id, points_xy, r, g, b, size=0.02):
        m = Marker()
        m.header.stamp = now
        m.header.frame_id = 'world'
        m.ns = ns
        m.id = marker_id
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = size
        m.scale.y = size
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
        for x, y in points_xy:
            m.points.append(Point(x=float(x), y=float(y), z=0.02))
        return m

    def _publish(self):
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()
        phase = np.linspace(0, 2 * np.pi, 40, endpoint=False)

        cx, cy = self._nominal_initial_center
        initial_ring = [(cx + self._std * np.cos(p), cy + self._std * np.sin(p)) for p in phase]
        arr.markers.append(self._line_strip(
            now, 'nominal_initial', 0, initial_ring, 0.2, 0.6, 1.0))  # light blue

        tx, ty = self._nominal_target_center
        target_ring = [(tx + self._std * np.cos(p), ty + self._std * np.sin(p)) for p in phase]
        arr.markers.append(self._line_strip(
            now, 'nominal_target', 1, target_ring, 1.0, 0.85, 0.0))  # gold

        arr.markers.append(self._points(
            now, 'target_reference_points', 2, self._reference_points.tolist(),
            1.0, 0.45, 0.0))  # orange

        self._pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = GaussianVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
