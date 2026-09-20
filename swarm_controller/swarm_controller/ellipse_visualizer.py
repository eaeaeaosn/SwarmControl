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
target_ellipse_display_offset : list[float]  [x, y] metres added only to the
    drawn ellipse marker (default [-0.2, -0.2]) — display-only nudge; the
    real target read from target_ellipse.json (and what the RL policy
    controls the robots toward) is unaffected.
axis_tick_values : list[float]  Tick values to label on both x and y axes
    (default [-1.0, -0.5, 0.0, 0.5, 1.0]).
axis_label_scale : float  Ticks are drawn at value * axis_label_scale instead
    of value (default 0.9) — e.g. the "1.0 m" tick is rendered at 0.9 m so
    the labelled -1..1 m axis spans an actual -0.9..0.9 m square. Purely a
    display fake; does not affect any real coordinates/targets.
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
        self.declare_parameter('target_ellipse_display_offset', [-0.15, -0.2])
        self.declare_parameter('axis_tick_values', [-1.0, -0.5, 0.0, 0.5, 1.0])
        self.declare_parameter('axis_label_scale', 0.9)

        pkg_path = self.get_parameter('controller_package_path').value
        self._circle_center = tuple(self.get_parameter('initial_circle_center').value)
        self._circle_radius = self.get_parameter('initial_circle_radius').value
        display_offset = np.asarray(
            self.get_parameter('target_ellipse_display_offset').value, dtype=float)
        self._display_offset = display_offset
        self._axis_tick_values = list(self.get_parameter('axis_tick_values').value)
        self._axis_label_scale = self.get_parameter('axis_label_scale').value

        target = json.loads((Path(pkg_path) / 'target_ellipse.json').read_text())
        center = np.asarray(target['center'], dtype=float)
        axes   = np.asarray(target['semiaxes'], dtype=float)
        angle  = float(target['angle_rad'])
        rotation = np.array([[np.cos(angle), -np.sin(angle)],
                             [np.sin(angle), np.cos(angle)]])
        phase = np.linspace(0, 2 * np.pi, 40, endpoint=False)
        # display_offset only nudges where the marker is drawn — the real
        # target (center, used by the RL policy via target_ellipse.json) is
        # untouched.
        self._ellipse_points = center + display_offset + (
            np.column_stack([np.cos(phase), np.sin(phase)]) * axes
        ) @ rotation.T

        # Axis tick labels are re-centred so their "0" sits at the midpoint
        # between the initial circle and the (displayed) target ellipse,
        # rather than at the true world origin — display-only.
        ellipse_center_displayed = center + display_offset
        self._tick_origin = (np.asarray(self._circle_center, dtype=float)
                              + ellipse_center_displayed) / 2.0

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

    @staticmethod
    def _text(now, ns, marker_id, x, y, text, r, g, b, height=0.05, z=0.03):
        m = Marker()
        m.header.stamp = now
        m.header.frame_id = 'world'
        m.ns = ns
        m.id = marker_id
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.z = height
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
        m.text = text
        return m

    @staticmethod
    def _fmt_tick(v: float) -> str:
        return f'{v:g}'

    @staticmethod
    def _region_quad(now, ns, marker_id, x0, x1, r, g, b, a, y0=-1.0, y1=1.0, z=0.0):
        """Flat translucent rectangle spanning [x0, x1] x [y0, y1] (two triangles)."""
        m = Marker()
        m.header.stamp = now
        m.header.frame_id = 'world'
        m.ns = ns
        m.id = marker_id
        m.type = Marker.TRIANGLE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
        corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        for i, j, k in ((0, 1, 2), (0, 2, 3)):
            for c in (corners[i], corners[j], corners[k]):
                m.points.append(Point(x=float(c[0]), y=float(c[1]), z=z))
        return m

    def _axis_tick_markers(self, now):
        """Fake XY axis tick labels: value `v` is drawn at v * axis_label_scale,
        so the labelled -1..1 m axis spans an actual -scale..scale m square,
        re-centred so "0" sits at the midpoint between the initial circle and
        the displayed target ellipse — a display-only nudge, not a real
        coordinate change."""
        s = self._axis_label_scale
        ox, oy = self._tick_origin
        markers = []
        for i, v in enumerate(self._axis_tick_values):
            # x-axis ticks along the bottom edge (y = -1 * scale).
            markers.append(self._text(
                now, 'axis_ticks_x', i, v * s + ox, -1.0 * s + oy,
                self._fmt_tick(v), 0.1, 0.1, 0.1))
            # y-axis ticks along the left edge (x = -1 * scale).
            markers.append(self._text(
                now, 'axis_ticks_y', i, -1.0 * s + ox, v * s + oy,
                self._fmt_tick(v), 0.1, 0.1, 0.1))
        return markers

    def _publish(self):
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        # Left/right beta-region shading, matching the trained policy's fixed
        # x=0 split (README.md section 2: x<0 keeps baseline beta, x>=0 uses
        # beta - 0.1) over the fixed [-1, 1] m workspace, nudged 0.1 m left
        # for display.
        region_scale = 0.9
        region_y_offset = -0.1
        left_outer_edge_shift = 0.1    # nudge the blue region's left edge right
        right_outer_edge_shift = -0.05  # nudge the red region's right edge left
        arr.markers.append(self._region_quad(
            now, 'region_left', 0,
            -1.1 * region_scale + left_outer_edge_shift, -0.1 * region_scale,
            0.0, 0.0, 1.0, 0.35,
            y0=-1.0 * region_scale + region_y_offset,
            y1=1.0 * region_scale + region_y_offset))   # blue
        arr.markers.append(self._region_quad(
            now, 'region_right', 0,
            -0.1 * region_scale, 0.9 * region_scale + right_outer_edge_shift,
            1.0, 0.0, 0.0, 0.35,
            y0=-1.0 * region_scale + region_y_offset,
            y1=1.0 * region_scale + region_y_offset))   # red

        cx, cy = self._circle_center
        phase = np.linspace(0, 2 * np.pi, 40, endpoint=False)
        circle_points = [(cx + self._circle_radius * np.cos(p),
                          cy + self._circle_radius * np.sin(p)) for p in phase]
        arr.markers.append(self._line_strip(
            now, 'initial_circle', 0, circle_points, 0.2, 0.6, 1.0))  # light blue

        arr.markers.append(self._line_strip(
            now, 'target_ellipse', 1, self._ellipse_points.tolist(), 1.0, 0.85, 0.0))  # gold

        arr.markers.extend(self._axis_tick_markers(now))

        self._pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = EllipseVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
