"""
mocap_republisher.py
====================
Subscribes to /mocap_rigid_bodies (RigidBodyArray from Optitrack NatNet client)
and republishes each tracked robot's pose as individual PoseStamped topics plus
RViz sphere markers.

ROS Parameters
--------------
robot_ids  : list[int]  — Optitrack rigid body IDs to track (default: [13])
                          robot_ids[0] → robot1, robot_ids[1] → robot2, ...
marker_scale : float    — RViz sphere diameter in metres (default: 0.15)

Published Topics (per robot N = 1, 2, …)
-----------------------------------------
/robotN/pose   (geometry_msgs/PoseStamped)      — robot pose in world frame
/robotN/marker (visualization_msgs/Marker)      — RViz sphere for RViz display
"""

import rclpy
from rclpy.node import Node
from mocap_optitrack_interfaces.msg import RigidBodyArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

# Distinct colors for up to 8 robots (R, G, B in 0.0–1.0)
_ROBOT_COLORS = [
    (1.0, 0.0, 0.0),   # robot1  red
    (0.0, 1.0, 0.0),   # robot2  green
    (0.0, 0.2, 1.0),   # robot3  blue
    (1.0, 0.8, 0.0),   # robot4  yellow
    (1.0, 0.0, 1.0),   # robot5  magenta
    (0.0, 1.0, 1.0),   # robot6  cyan
    (1.0, 0.5, 0.0),   # robot7  orange
    (0.5, 0.0, 1.0),   # robot8  purple
]


class MocapRepublisher(Node):
    def __init__(self):
        super().__init__('mocap_republisher')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('robot_ids', [13])
        self.declare_parameter('marker_scale', 0.15)

        robot_ids: list = self.get_parameter('robot_ids').value
        marker_scale: float = self.get_parameter('marker_scale').value

        # Build lookup: mocap_id → (robot_name, pose_pub, marker_pub, color)
        self._robots: dict = {}
        for idx, mocap_id in enumerate(robot_ids):
            robot_name = f'robot{idx + 1}'
            color = _ROBOT_COLORS[idx % len(_ROBOT_COLORS)]
            pose_pub = self.create_publisher(
                PoseStamped, f'/{robot_name}/pose', 10)
            marker_pub = self.create_publisher(
                Marker, f'/{robot_name}/marker', 10)
            self._robots[int(mocap_id)] = {
                'name':       robot_name,
                'index':      idx + 1,
                'pose_pub':   pose_pub,
                'marker_pub': marker_pub,
                'color':      color,
            }
            self.get_logger().info(
                f'Tracking mocap ID {mocap_id} → /{robot_name}/pose  /{robot_name}/marker')

        self._marker_scale = marker_scale

        # ── Subscriber ────────────────────────────────────────────────────
        self.create_subscription(
            RigidBodyArray, '/mocap_rigid_bodies', self._callback, 10)

        self.get_logger().info(
            f'mocap_republisher ready — tracking {len(robot_ids)} robot(s)')

    # ── Callback ──────────────────────────────────────────────────────────
    def _callback(self, msg: RigidBodyArray):
        now = self.get_clock().now().to_msg()

        for rb in msg.rigid_bodies:
            info = self._robots.get(rb.id)
            if info is None:
                continue  # not a robot we care about

            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = 'world'
            pose.pose = rb.pose_stamped.pose
            info['pose_pub'].publish(pose)

            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = 'world'
            marker.ns = info['name']
            marker.id = rb.id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = rb.pose_stamped.pose
            s = self._marker_scale
            marker.scale.x = s
            marker.scale.y = s
            marker.scale.z = s
            r, g, b = info['color']
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.0
            info['marker_pub'].publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = MocapRepublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
