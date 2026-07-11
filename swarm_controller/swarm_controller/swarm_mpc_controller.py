"""
swarm_mpc_controller.py
=======================
ROS 2 node that closes the loop between Optitrack motion-capture poses
and the Dubins-swarm MPC (obstacle-avoidance) algorithm.

Pipeline
--------
  /robotN/pose  (PoseStamped)  ──►  this node  ──►  /robotN/cmd_vel  (Twist)

Algorithm
---------
  1. Collect all robot positions (x_real, y_real, theta) from mocap.
  2. Map to simulation coordinates: x_sim = x_real * (5 / arena_half_width).
  3. Pack N_MPC representative particles from the real swarm positions.
  4. Call mpc.make_step(x0) → (u_opt, v_opt) ∈ [-1, 1].
  5. Scale to real velocities and broadcast the same Twist to every robot
     (swarm-level control: all robots get an identical shared command).

Two-phase routing (matches simulation behaviour)
------------------------------------------------
  Phase 1: steer toward WAYPOINT_CENTER until swarm centroid is within
           WAYPOINT_REACH_RADIUS of it.
  Phase 2: steer toward GOAL_CENTER (final target).

ROS Parameters
--------------
robot_names          list[str]   ROS names of robots, e.g. ['robot1','robot2']
arena_half_width     float [m]   Half-size of the real arena → maps to sim ±5
max_linear_vel       float [m/s] Speed at v_opt = 1.0  (default 0.06)
max_angular_vel      float [r/s] Turn-rate at u_opt = 1.0 (default 1.0)
control_dt           float [s]   Control-loop period (default 0.1)
goal_x, goal_y       float       Goal centre in *sim* coordinates (default 3.0)
waypoint_x, waypoint_y float     Waypoint in sim coords (default 2.75, -1.75)
waypoint_radius      float       Centroid-to-waypoint switch distance (default 0.5)
mpc_package_path     str         Path to obstalce_avoidance_mpc_clean directory
n_mpc                int         Representative particles in MPC (default 20)
n_horizon            int         MPC prediction horizon (default 10)
mpc_dt               float       MPC internal timestep — must match sim DT (default 0.05)
moment_order         int         Legendre moment order (default 10)
obstacle_penalty     float       Soft constraint weight (default 5000)
"""

import os
import sys
import math
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw (rad) from a unit quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _pack_state(xs: np.ndarray, ys: np.ndarray, thetas: np.ndarray,
                n_mpc: int) -> np.ndarray:
    """
    Sample n_mpc evenly-spaced representative particles from the real swarm.
    Returns a (3*n_mpc, 1) column vector: [x0..xN | y0..yN | θ0..θN].
    """
    n_real = len(xs)
    idx = np.linspace(0, n_real - 1, n_mpc, dtype=int)
    return np.concatenate([xs[idx], ys[idx], thetas[idx]]).reshape(-1, 1).astype(np.float64)


# ──────────────────────────────────────────────────────────────────────────────
# ROS 2 Node
# ──────────────────────────────────────────────────────────────────────────────

class SwarmMPCController(Node):
    """Closed-loop MPC controller for a Dubins-vehicle swarm."""

    def __init__(self):
        super().__init__('swarm_mpc_controller')

        # ── Declare & read parameters ──────────────────────────────────────
        self.declare_parameter('robot_names',       ['robot1'])
        self.declare_parameter('arena_half_width',  2.5)       # metres
        self.declare_parameter('max_linear_vel',    0.06)      # m/s
        self.declare_parameter('max_angular_vel',   1.0)       # rad/s
        self.declare_parameter('control_dt',        0.1)       # seconds
        self.declare_parameter('goal_x',            3.0)
        self.declare_parameter('goal_y',            3.0)
        self.declare_parameter('waypoint_x',        2.75)
        self.declare_parameter('waypoint_y',       -1.75)
        self.declare_parameter('waypoint_radius',   0.5)
        self.declare_parameter(
            'mpc_package_path',
            '/home/eason/Swarm/obstalce_avoidance_mpc_clean')
        self.declare_parameter('n_mpc',             20)
        self.declare_parameter('n_horizon',         10)
        self.declare_parameter('mpc_dt',            0.05)
        self.declare_parameter('moment_order',      10)
        self.declare_parameter('obstacle_penalty',  5000.0)

        robot_names: list  = self.get_parameter('robot_names').value
        self._arena_hw     = self.get_parameter('arena_half_width').value
        self._max_v        = self.get_parameter('max_linear_vel').value
        self._max_w        = self.get_parameter('max_angular_vel').value
        ctrl_dt            = self.get_parameter('control_dt').value
        goal_x             = self.get_parameter('goal_x').value
        goal_y             = self.get_parameter('goal_y').value
        wp_x               = self.get_parameter('waypoint_x').value
        wp_y               = self.get_parameter('waypoint_y').value
        self._wp_radius    = self.get_parameter('waypoint_radius').value
        mpc_path           = self.get_parameter('mpc_package_path').value
        self._n_mpc        = self.get_parameter('n_mpc').value
        n_horizon          = self.get_parameter('n_horizon').value
        mpc_dt             = self.get_parameter('mpc_dt').value
        moment_order       = self.get_parameter('moment_order').value
        obs_penalty        = self.get_parameter('obstacle_penalty').value

        self._waypoint_sim = (wp_x, wp_y)
        self._goal_sim     = (goal_x, goal_y)

        # ── Robot pose storage (thread-safe lock) ─────────────────────────
        self._lock = threading.Lock()
        self._poses: dict[str, tuple[float, float, float]] = {
            name: None for name in robot_names
        }

        # ── ROS subscribers (one per robot) ───────────────────────────────
        self._subs = []
        for name in robot_names:
            sub = self.create_subscription(
                PoseStamped,
                f'/{name}/pose',
                lambda msg, n=name: self._pose_cb(msg, n),
                10,
            )
            self._subs.append(sub)
            self.get_logger().info(f'Subscribing to /{name}/pose')

        # ── ROS cmd_vel publishers (one per robot) ────────────────────────
        self._cmd_pubs: dict[str, rclpy.publisher.Publisher] = {
            name: self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            for name in robot_names
        }

        # ── Trajectory visualisation: actual (mocap) vs. MPC-predicted ─────
        self._actual_path_pubs = {
            name: self.create_publisher(Path, f'/{name}/path_actual', 10)
            for name in robot_names
        }
        self._predicted_path_pubs = {
            name: self.create_publisher(Path, f'/{name}/path_predicted', 10)
            for name in robot_names
        }
        self._actual_paths: dict[str, list] = {name: [] for name in robot_names}
        self._ACTUAL_PATH_MAX_LEN = 1000   # ~100s of trail at 10Hz control_dt

        # ── MPC state ─────────────────────────────────────────────────────
        self._mpc          = None   # do-mpc controller (phase 1 or 2)
        self._mpc_phase    = 1      # 1 = waypoint, 2 = goal
        self._mpc_ready    = False

        # ── Initialise MPC (may take a few seconds; log progress) ─────────
        self.get_logger().info('Initialising MPC solver — this may take ~30 s …')
        try:
            self._init_mpc(mpc_path, n_horizon, mpc_dt, moment_order, obs_penalty)
        except Exception as exc:
            self.get_logger().error(f'MPC init failed: {exc}')
            raise

        # ── Control timer ─────────────────────────────────────────────────
        self._timer = self.create_timer(ctrl_dt, self._control_cb)
        self.get_logger().info(
            f'SwarmMPCController ready  ({len(robot_names)} robots, '
            f'dt={ctrl_dt}s, arena_half_width={self._arena_hw}m)')

    # ── MPC initialisation ────────────────────────────────────────────────

    def _init_mpc(self, mpc_path: str, n_horizon: int, mpc_dt: float,
                  moment_order: int, obs_penalty: float):
        """
        Import the MPC code from obstalce_avoidance_mpc_clean and build the
        phase-1 (waypoint) solver.  The path to that directory is added to
        sys.path so the existing environment.py / mpc_dubins.py can be imported
        without modification.
        """
        # Add MPC directory to Python path
        if mpc_path not in sys.path:
            sys.path.insert(0, mpc_path)

        # The CSV coefficients are loaded relative to CWD inside environment.py
        orig_cwd = os.getcwd()
        os.chdir(mpc_path)
        try:
            from environment import DubinsMomentEnv          # noqa: PLC0415
            from mpc_dubins import build_model, build_mpc    # noqa: PLC0415

            self.get_logger().info('  [1/3] Creating DubinsMomentEnv …')
            self._env = DubinsMomentEnv(
                num_robots   = max(self._n_mpc, 20),
                moment_order = moment_order,
                dt           = mpc_dt,
            )

            self.get_logger().info('  [2/3] Computing waypoint moments …')
            from mpc_dubins import compute_waypoint_moments   # noqa: PLC0415
            self._wp_vec   = compute_waypoint_moments(
                self._env, *self._waypoint_sim)
            self._goal_vec = self._env.target_vec

            self.get_logger().info('  [3/3] Building phase-1 MPC (waypoint) …')
            self._mpc = self._build_phase_mpc(
                build_model, build_mpc,
                self._wp_vec, n_horizon, mpc_dt, obs_penalty)

            self._build_model_fn = build_model
            self._build_mpc_fn   = build_mpc
            self._n_horizon      = n_horizon
            self._mpc_dt         = mpc_dt
            self._obs_penalty    = obs_penalty

            self._mpc_ready = True
            self.get_logger().info('MPC ready  (phase 1 — tracking waypoint '
                                   f'{self._waypoint_sim})')
        finally:
            os.chdir(orig_cwd)

    def _build_phase_mpc(self, build_model_fn, build_mpc_fn,
                         target_vec, n_horizon, mpc_dt, obs_penalty):
        """Build a fresh do-mpc controller targeting target_vec."""
        model = build_model_fn(
            C_mat      = self._env.C_matrix,
            C_obs      = self._env.C_obs,
            target_vec = target_vec,
            w          = self._env.w,
            N_mpc      = self._n_mpc,
            M          = self._env.M,
            dt         = mpc_dt,
        )
        return build_mpc_fn(model, n_horizon, mpc_dt, obs_penalty)

    # ── Coordinate helpers ────────────────────────────────────────────────

    def _to_sim(self, x_real: float, y_real: float) -> tuple[float, float]:
        """Convert real-world metres to simulation coordinates ∈ [-5, 5]."""
        scale = 5.0 / self._arena_hw
        return x_real * scale, y_real * scale

    def _to_real(self, x_sim: float, y_sim: float) -> tuple[float, float]:
        """Convert simulation coordinates ∈ [-5, 5] back to real-world metres."""
        scale = self._arena_hw / 5.0
        return x_sim * scale, y_sim * scale

    @staticmethod
    def _make_path_msg(stamp, points: list) -> Path:
        """Build a nav_msgs/Path (frame 'world') from a list of (x, y) real-world points."""
        path_msg = Path()
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = 'world'
        for x, y in points:
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = 'world'
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        return path_msg

    # ── ROS callbacks ─────────────────────────────────────────────────────

    def _pose_cb(self, msg: PoseStamped, robot_name: str):
        """Receive a PoseStamped and store as (x_sim, y_sim, theta)."""
        p = msg.pose.position
        q = msg.pose.orientation
        x_sim, y_sim = self._to_sim(p.x, p.y)
        theta = _quat_to_yaw(q.x, q.y, q.z, q.w)
        with self._lock:
            self._poses[robot_name] = (x_sim, y_sim, theta)

    def _control_cb(self):
        """Periodic control callback: run one MPC step and publish cmd_vel."""
        if not self._mpc_ready:
            return

        # Snapshot all poses
        with self._lock:
            poses = {k: v for k, v in self._poses.items()}

        valid_names = [name for name, v in poses.items() if v is not None]
        valid = [poses[name] for name in valid_names]
        if not valid:
            self.get_logger().warn('No robot poses received yet — skipping MPC step',
                                   throttle_duration_sec=5.0)
            return

        xs     = np.array([p[0] for p in valid])
        ys     = np.array([p[1] for p in valid])
        thetas = np.array([p[2] for p in valid])

        # ── Actual-trajectory trail (ground truth from mocap) ──────────────
        now = self.get_clock().now().to_msg()
        for name, x_sim, y_sim in zip(valid_names, xs, ys):
            buf = self._actual_paths[name]
            buf.append(self._to_real(x_sim, y_sim))
            if len(buf) > self._ACTUAL_PATH_MAX_LEN:
                del buf[0]
            self._actual_path_pubs[name].publish(self._make_path_msg(now, buf))

        # ── Phase switch check ────────────────────────────────────────────
        cx, cy = xs.mean(), ys.mean()
        if self._mpc_phase == 1:
            dist = math.hypot(cx - self._waypoint_sim[0],
                              cy - self._waypoint_sim[1])
            if dist < self._wp_radius:
                self.get_logger().info(
                    f'Phase switch → 2 (centroid reached waypoint, dist={dist:.3f})')
                self._mpc_phase = 2
                # Rebuild MPC for final goal (re-uses already-imported functions)
                self._mpc = self._build_phase_mpc(
                    self._build_model_fn, self._build_mpc_fn,
                    self._goal_vec,
                    self._n_horizon, self._mpc_dt, self._obs_penalty)
                self.get_logger().info('Phase-2 MPC ready (goal '
                                       f'{self._goal_sim})')

        # ── Pack state & solve MPC ────────────────────────────────────────
        # particle_idx[i] = which real robot (index into valid_names) representative
        # particle slot i was sampled from — reused below to attribute the MPC's
        # predicted trajectory back to individual robots.
        particle_idx = np.linspace(0, len(xs) - 1, self._n_mpc, dtype=int)
        x0 = _pack_state(xs, ys, thetas, self._n_mpc)
        try:
            u_opt = self._mpc.make_step(x0)
        except Exception as exc:
            self.get_logger().error(f'MPC solver error: {exc}', throttle_duration_sec=2.0)
            return

        # ── Predicted-trajectory publish (MPC's own horizon prediction) ────
        # Best-effort: a failure here must never break the actual control loop.
        try:
            x_pred = self._mpc.data.prediction(('_x', 'x_pos'))[:, :, 0]  # (n_mpc, horizon+1)
            y_pred = self._mpc.data.prediction(('_x', 'y_pos'))[:, :, 0]
            for r, name in enumerate(valid_names):
                slots = np.where(particle_idx == r)[0]
                if slots.size == 0:
                    continue
                slot = slots[0]
                points = [self._to_real(px, py)
                          for px, py in zip(x_pred[slot], y_pred[slot])]
                self._predicted_path_pubs[name].publish(self._make_path_msg(now, points))
        except Exception as exc:
            self.get_logger().warn(f'Predicted-path publish failed: {exc}',
                                   throttle_duration_sec=5.0)

        u_val = float(u_opt[0, 0])   # turn-rate  ∈ [-1, 1]
        v_val = float(u_opt[1, 0])   # speed      ∈ [-1, 1]

        linear_x  = v_val * self._max_v    # m/s
        angular_z = u_val * self._max_w    # rad/s

        self.get_logger().debug(
            f'MPC  u={u_val:+.4f}  v={v_val:+.4f}  '
            f'→  vx={linear_x:+.4f} m/s  wz={angular_z:+.4f} rad/s  '
            f'centroid=({cx:+.2f},{cy:+.2f})')

        # ── Broadcast same Twist to all robots ────────────────────────────
        twist = Twist()
        twist.linear.x  = linear_x
        twist.angular.z = angular_z
        for pub in self._cmd_pubs.values():
            pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmMPCController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
