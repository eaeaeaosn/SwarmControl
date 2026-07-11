"""
obstacle_visualizer.py
=======================
Publishes the 5 static MPC obstacle boxes and the goal region as RViz
markers, in the same 'world' frame as the robot markers, so the avoidance
regions and target are visible in RViz.

Obstacle boxes are given in *simulation* coordinates (domain half-width 5.0)
and MUST be kept in sync with the `obstacles` list in
obstalce_avoidance_mpc_clean/coefCalc.py — that script is the source of
truth for the Legendre collision-cost CSVs; this node only visualises the
same boxes, scaled to real-world metres via arena_half_width.

The goal marker is drawn from the goal_x/goal_y params below, which mirror
swarm_mpc_controller's goal_x/goal_y. NOTE: those params are only used for
phase-1→2 bookkeeping in swarm_mpc_controller — the actual phase-2 target
moments come from DubinsMomentEnv.target_vec, which hardcodes mu_x=mu_y=3.0
in obstalce_avoidance_mpc_clean/environment.py regardless of these params.
Today goal_x=goal_y=3.0 so they agree; if that default is ever changed in
environment.py or swarm_params.yaml, keep both in sync or this marker will
show the wrong spot.

ROS Parameters
--------------
arena_half_width : float   Half-size of the real lab [m] → sim ±5
                            (must match swarm_mpc_controller's value)
goal_x, goal_y    : float  Goal centre in *simulation* coordinates
                            (must match swarm_mpc_controller's value)
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

# Must match `obstacles` in obstalce_avoidance_mpc_clean/coefCalc.py (sim coords).
# (x1, x2, y1, y2, r, g, b) — colors match mpc_dubins.py's visualise() legend.
_OBSTACLES_SIM = [
    (-0.5, 0.5, -0.5, 0.5, 1.0, 0.0, 0.0),   # Obs 1 — red
    ( 3.0, 4.0, -0.5, 0.5, 1.0, 0.5, 0.0),   # Obs 2 — orange
    (-0.5, 0.5,  1.5, 2.5, 0.5, 0.0, 0.5),   # Obs 3 — purple
    (-2.5,-1.5, -0.5, 0.5, 0.6, 0.3, 0.0),   # Obs 4 — brown
    ( 1.5, 2.5, -3.5,-2.5, 0.0, 0.5, 0.5),   # Obs 5 — teal
]


class ObstacleVisualizer(Node):
    def __init__(self):
        super().__init__('obstacle_visualizer')

        self.declare_parameter('arena_half_width', 2.5)
        self.declare_parameter('goal_x', 3.0)
        self.declare_parameter('goal_y', 3.0)
        arena_half_width = self.get_parameter('arena_half_width').value
        self._goal_sim = (self.get_parameter('goal_x').value,
                          self.get_parameter('goal_y').value)
        self._scale = arena_half_width / 5.0  # real = sim * scale

        self._obstacle_pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)
        self._goal_pub = self.create_publisher(Marker, '/goal_marker', 10)
        self._timer = self.create_timer(1.0, self._publish)
        self.get_logger().info(
            f'obstacle_visualizer ready — {len(_OBSTACLES_SIM)} obstacles, '
            f'goal={self._goal_sim} (sim), arena_half_width={arena_half_width} m')

    def _publish(self):
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()
        for i, (x1, x2, y1, y2, r, g, b) in enumerate(_OBSTACLES_SIM):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = 'world'
            m.ns = 'obstacles'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = (x1 + x2) / 2.0 * self._scale
            m.pose.position.y = (y1 + y2) / 2.0 * self._scale
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = (x2 - x1) * self._scale
            m.scale.y = (y2 - y1) * self._scale
            m.scale.z = 0.1
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 0.45
            arr.markers.append(m)
        self._obstacle_pub.publish(arr)

        goal = Marker()
        goal.header.stamp = now
        goal.header.frame_id = 'world'
        goal.ns = 'goal'
        goal.id = 0
        goal.type = Marker.CYLINDER
        goal.action = Marker.ADD
        goal.pose.position.x = self._goal_sim[0] * self._scale
        goal.pose.position.y = self._goal_sim[1] * self._scale
        goal.pose.position.z = 0.02
        goal.pose.orientation.w = 1.0
        goal.scale.x = 2.0 * self._scale   # matches mpc_dubins.py's radius-1.0 target circle
        goal.scale.y = 2.0 * self._scale
        goal.scale.z = 0.02
        goal.color.r = 0.0
        goal.color.g = 1.0
        goal.color.b = 0.0
        goal.color.a = 0.35
        self._goal_pub.publish(goal)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
