"""
circle_to_ellipse_controller.py
================================
ROS 2 node that closes the loop between Optitrack motion-capture poses and
the trained PPO circle-to-ellipse policy in
`/home/eason/Swarm/circle_to_ellipse_controller` (package `controller.py`,
class `CircleToEllipseController`).

The policy runs as a SUBPROCESS in that package's own virtualenv
(`<controller_package_path>/.venv`), talking to this node over its
already-documented stdin/stdout line protocol (`python3 controller.py` with
no args: one JSON `[x_m, y_m, theta_rad]` state matrix per input line, one
`[[u, v]]` command per output line, controller state retained between
calls). This is NOT importing controller.py in-process: the package's
selected_model.zip requires `numpy>=2` to unpickle, which conflicts with
this machine's system ROS/OpenCV stack (`cv_bridge` et al. need
`numpy<2`) in one interpreter — running the policy in its own venv/process
avoids that conflict entirely. See CIRCLE_TO_ELLIPSE_EXPERIMENT.md
("Dependencies") for how to create that venv.

This node SUBSTITUTES `swarm_mpc_controller` for the 5-robot circle-to-
ellipse experiment. Unlike the MPC node, it does not broadcast one identical
Twist to every robot — each robot gets its own per-robot Twist, because the
policy's shared [u, v] command is modulated per robot by a fixed "beta"
value that depends on which half of the arena (x < 0 vs x >= 0) the robot is
currently in. See the package's README.md section 2 ("Apply each robot's
beta") for the exact law this file implements.

Pipeline
--------
  /robotN/pose (PoseStamped, N=1..5)  ──►  this node
      │
      ├─ every control_dt (0.2 s): pack all 5 poses → controller.predict()
      │                            → shared (u, v) broadcast command
      │
      └─ every beta_dt (0.05 s): per robot, recompute beta from live x,
                                 apply the vx/vy/omega law, clip speed,
                                 project onto heading, publish /robotN/cmd_vel

Why two timers at different rates: the policy only makes a fresh decision
every 0.2 s, but a robot's region (and therefore its beta) can flip the
instant it crosses x = 0, so beta must be re-evaluated much faster than the
decision itself — 0.05 s, matching the package's simulator update rate.

Why a start-gate service instead of running immediately: this policy encodes
an *open-loop, offline-optimized timed sequence* (not a feedback controller
— see README.md section 3) so its very first decision must coincide with the
robots actually being at their prescribed initial positions. Call the
`~/start` service (std_srvs/Trigger) once all 5 robots are confirmed in
place; `autostart:=true` skips this for quick bench tests without hardware.

Known approximation vs. the trained simulator
-----------------------------------------------
The package's beta law is written for an idealized kinematic model where
vx and vy can be scaled independently of each other (`vx = v*beta*cos(theta)`,
`vy = v/beta*sin(theta)`). A real differential-drive robot cannot move
sideways — it only has a forward speed and a turn rate. This node makes the
best of it by projecting the desired (vx, vy) onto the robot's own current
heading to obtain a realizable forward speed, and sends `omega` (which the
law defines directly, independent of vx/vy) straight through as angular.z.
The perpendicular ("sideways") component of (vx, vy) is not realizable and
is dropped. This is the one unavoidable deviation from the package's
simulated dynamics — watch for it on the first real run.

ROS Parameters
--------------
robot_names             list[str]  ROS names, in the exact order matching
                                   the package's README table (robot1..5 =
                                   columns 1..5 of the state matrix).
controller_package_path str        Path to circle_to_ellipse_controller dir.
venv_python             str        Python interpreter to run controller.py
                                   with (default '' → <controller_package_
                                   path>/.venv/bin/python).
control_dt              float [s]  Policy decision period (default 0.2).
beta_dt                 float [s]  Beta re-evaluation / cmd_vel period
                                   (default 0.05).
max_speed_mps           float      Per-robot |[vx,vy]| clip (default 0.05).
episode_decisions       int        Must match controller.py's
                                   EPISODE_DECISIONS constant (default 805).
subprocess_timeout_s    float      Max wait for one policy response before
                                   skipping that decision (default 1.0).
arena_half_width        float [m]  Sanity-check only; the workspace is the
                                   physical 2 m x 2 m square, coordinates are
                                   used directly in metres (no sim scaling).
autostart               bool       Skip the ~/start gate (default False).
"""

import json
import math
import select
import subprocess
import sys
import threading
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path as PathMsg
from std_srvs.srv import Trigger


def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw (rad) from a unit quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class CircleToEllipseControllerNode(Node):
    """Per-robot beta-scaled bridge between mocap poses and the PPO policy."""

    def __init__(self):
        super().__init__('circle_to_ellipse_controller')

        # ── Declare & read parameters ──────────────────────────────────────
        self.declare_parameter(
            'robot_names',
            ['robot1', 'robot2', 'robot3', 'robot4', 'robot5'])
        self.declare_parameter(
            'controller_package_path',
            '/home/eason/Swarm/circle_to_ellipse_controller')
        self.declare_parameter('venv_python',       '')
        self.declare_parameter('control_dt',        0.2)
        self.declare_parameter('beta_dt',           0.05)
        self.declare_parameter('max_speed_mps',     0.05)
        self.declare_parameter('episode_decisions', 805)
        self.declare_parameter('subprocess_timeout_s', 1.0)
        self.declare_parameter('arena_half_width',  1.0)
        self.declare_parameter('autostart',         False)

        robot_names: list = self.get_parameter('robot_names').value
        pkg_path           = self.get_parameter('controller_package_path').value
        venv_python        = self.get_parameter('venv_python').value
        control_dt         = self.get_parameter('control_dt').value
        beta_dt            = self.get_parameter('beta_dt').value
        self._max_speed    = self.get_parameter('max_speed_mps').value
        self._episode_decisions = self.get_parameter('episode_decisions').value
        self._subprocess_timeout = self.get_parameter('subprocess_timeout_s').value
        arena_hw           = self.get_parameter('arena_half_width').value
        autostart          = self.get_parameter('autostart').value

        self._robot_names = list(robot_names)
        if len(self._robot_names) != 5:
            raise ValueError(
                'circle_to_ellipse_controller requires exactly 5 robot_names '
                f'(got {len(self._robot_names)}) — the packaged policy and '
                'target_ellipse.json are fixed to a 5-robot experiment.')

        # ── Robot pose storage (thread-safe lock) ─────────────────────────
        self._lock = threading.Lock()
        self._poses: dict[str, tuple] = {name: None for name in self._robot_names}

        # ── ROS subscribers (one per robot) ───────────────────────────────
        self._subs = []
        for name in self._robot_names:
            sub = self.create_subscription(
                PoseStamped,
                f'/{name}/pose',
                lambda msg, n=name: self._pose_cb(msg, n),
                10,
            )
            self._subs.append(sub)
            self.get_logger().info(f'Subscribing to /{name}/pose')

        # ── ROS cmd_vel publishers (one per robot — NOT a shared broadcast) ─
        self._cmd_pubs = {
            name: self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            for name in self._robot_names
        }

        # ── Trajectory visualisation: actual (mocap) trail ─────────────────
        self._actual_path_pubs = {
            name: self.create_publisher(PathMsg, f'/{name}/path_actual', 10)
            for name in self._robot_names
        }
        self._actual_paths: dict[str, list] = {name: [] for name in self._robot_names}
        self._ACTUAL_PATH_MAX_LEN = 1000

        # ── Launch the packaged policy subprocess + load per-robot beta table
        self.get_logger().info(f'Loading circle-to-ellipse controller from {pkg_path} …')
        self._load_beta_table(pkg_path, arena_hw)
        self._start_policy_subprocess(pkg_path, venv_python)

        # ── Shared-command state (written by decision timer, read by beta timer)
        self._u = 0.0   # rad/s, shared broadcast turn-rate
        self._v = 0.0   # m/s,   shared broadcast forward speed
        self._decision = 0
        self._episode_active = False
        self._episode_done   = False

        # ── Start gate ──────────────────────────────────────────────────────
        self.create_service(Trigger, '~/start', self._start_cb)

        # ── Timers (always created; both are no-ops until _episode_active) ──
        self._decision_timer = self.create_timer(control_dt, self._decision_cb)
        self._beta_timer     = self.create_timer(beta_dt, self._beta_cb)

        if autostart:
            self.get_logger().warn('autostart=true — starting the 161 s sequence immediately')
            self._episode_active = True
        else:
            self.get_logger().info(
                'Ready. Place all 5 robots at their prescribed initial poses, then call '
                'the ~/start service (std_srvs/Trigger) — e.g. '
                'ros2 service call /circle_to_ellipse_controller/start std_srvs/srv/Trigger {}')

    # ── Setup ──────────────────────────────────────────────────────────────

    def _load_beta_table(self, pkg_path: str, arena_hw: float):
        """Read target_ellipse.json's per-robot beta table (plain JSON, no numpy needed)."""
        target = json.loads((Path(pkg_path) / 'target_ellipse.json').read_text())
        beta_by_id = {
            int(entry['robot_id']): float(entry['baseline_beta'])
            for entry in target['robot_setup']
        }
        if sorted(beta_by_id) != [1, 2, 3, 4, 5]:
            raise ValueError(
                f'target_ellipse.json robot_setup must define robot_id 1..5, got {sorted(beta_by_id)}')

        # robot_names[i] is column i+1 of the state matrix → robot_id i+1's beta.
        self._baseline_beta = {
            name: beta_by_id[i + 1] for i, name in enumerate(self._robot_names)
        }
        self.get_logger().info(
            'Per-robot baseline beta: ' +
            ', '.join(f'{n}={b:.6f}' for n, b in self._baseline_beta.items()))

        half = 1.0  # the package's fixed workspace, x/y in [-1, 1] m
        if abs(arena_hw - half) > 1e-6:
            self.get_logger().warn(
                f'arena_half_width param is {arena_hw} m but the packaged policy was '
                f'trained on a fixed {half} m half-width (2 m x 2 m) workspace — '
                'this param is a sanity check only and does not rescale coordinates.')

    def _start_policy_subprocess(self, pkg_path: str, venv_python: str):
        """Launch `python3 controller.py` (no args → stdin-streaming mode) in
        the package's own venv, and keep it alive for the whole episode.

        Protocol (package's own README/controller.py main()): one JSON state
        matrix per input line, one JSON `[[u, v]]` command per output line,
        state (previous action, elapsed decision count) retained server-side
        between calls.
        """
        python = venv_python or str(Path(pkg_path) / '.venv' / 'bin' / 'python')
        if not Path(python).exists():
            raise RuntimeError(
                f"venv interpreter not found at '{python}'. Create it with: "
                f"python3 -m venv {pkg_path}/.venv && "
                f"{pkg_path}/.venv/bin/python -m pip install -r {pkg_path}/requirements.txt "
                "(see CIRCLE_TO_ELLIPSE_EXPERIMENT.md).")

        self._proc = subprocess.Popen(
            [python, 'controller.py'],
            cwd=pkg_path,
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, bufsize=1,  # line-buffered
        )
        self.get_logger().info(f'Policy subprocess started ({python}, pid={self._proc.pid})')

    def _predict_via_subprocess(self, states: np.ndarray):
        """Send one (3,5) state matrix, return the parsed [[u, v]] response.

        Returns None (and logs) on any I/O error, timeout, or malformed
        response — the caller should skip that decision rather than crash.
        """
        proc = self._proc
        if proc.poll() is not None:
            self.get_logger().error(
                f'Policy subprocess exited (code {proc.returncode}) — no more commands will be issued.',
                throttle_duration_sec=5.0)
            return None
        try:
            proc.stdin.write(json.dumps(states.tolist()) + '\n')
            proc.stdin.flush()
        except BrokenPipeError as exc:
            self.get_logger().error(f'Policy subprocess stdin closed: {exc}')
            return None

        ready, _, _ = select.select([proc.stdout], [], [], self._subprocess_timeout)
        if not ready:
            self.get_logger().error(
                f'Policy subprocess did not respond within {self._subprocess_timeout} s',
                throttle_duration_sec=2.0)
            return None
        line = proc.stdout.readline()
        if not line:
            stderr_tail = proc.stderr.read()
            self.get_logger().error(f'Policy subprocess closed stdout. stderr:\n{stderr_tail}')
            return None
        try:
            control = np.asarray(json.loads(line), dtype=float)
        except (json.JSONDecodeError, ValueError) as exc:
            self.get_logger().error(f'Malformed policy response {line!r}: {exc}')
            return None
        return control

    # ── Coordinate / path helpers ────────────────────────────────────────────

    @staticmethod
    def _make_path_msg(stamp, points: list) -> PathMsg:
        path_msg = PathMsg()
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
        p = msg.pose.position
        q = msg.pose.orientation
        theta = _quat_to_yaw(q.x, q.y, q.z, q.w)
        with self._lock:
            self._poses[robot_name] = (p.x, p.y, theta)

    def _start_cb(self, request, response):
        if self._episode_done:
            response.success = False
            response.message = 'Episode already finished; restart the node to run again.'
        elif self._episode_active:
            response.success = False
            response.message = 'Episode already running.'
        else:
            self._episode_active = True
            response.success = True
            response.message = 'Circle-to-ellipse sequence started (161 s).'
            self.get_logger().info('~/start called — sequence armed.')
        return response

    def _snapshot_poses(self):
        with self._lock:
            return dict(self._poses)

    def _decision_cb(self):
        """Every control_dt: get one fresh shared (u, v) from the policy."""
        if not self._episode_active or self._episode_done:
            return

        poses = self._snapshot_poses()
        missing = [n for n in self._robot_names if poses[n] is None]
        if missing:
            self.get_logger().warn(
                f'No pose yet for {missing} — skipping this decision',
                throttle_duration_sec=2.0)
            return

        states = np.array(
            [[poses[n][0] for n in self._robot_names],
             [poses[n][1] for n in self._robot_names],
             [poses[n][2] for n in self._robot_names]], dtype=float)
        control = self._predict_via_subprocess(states)
        if control is None:
            return  # already logged; keep the previous (u, v) and try again next tick

        self._u, self._v = float(control[0, 0]), float(control[0, 1])
        self._decision += 1
        self.get_logger().debug(
            f'decision {self._decision}/{self._episode_decisions}: u={self._u:+.4f} rad/s v={self._v:+.4f} m/s')

        if self._decision >= self._episode_decisions:
            self._episode_done = True
            self.get_logger().info('Circle-to-ellipse sequence complete — stopping all robots.')
            self.stop_all_robots()

    def _beta_cb(self):
        """Every beta_dt: recompute each robot's beta from its live x and command it."""
        if not self._episode_active or self._episode_done:
            return

        poses = self._snapshot_poses()
        now = self.get_clock().now().to_msg()
        for name in self._robot_names:
            pose = poses[name]
            if pose is None:
                continue
            x, y, theta = pose

            baseline = self._baseline_beta[name]
            beta = baseline if x < 0.0 else baseline - 0.1

            vx = self._v * beta * math.cos(theta)
            vy = self._v / beta * math.sin(theta)
            omega = beta * self._u

            speed = math.hypot(vx, vy)
            if speed > self._max_speed and speed > 0.0:
                scale = self._max_speed / speed
                vx *= scale
                vy *= scale

            # Project the desired world-frame (vx, vy) onto the robot's own
            # heading — the only forward-speed component a differential-drive
            # robot can actually realize (see module docstring).
            linear_x = vx * math.cos(theta) + vy * math.sin(theta)

            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = omega
            self._cmd_pubs[name].publish(twist)

            buf = self._actual_paths[name]
            buf.append((x, y))
            if len(buf) > self._ACTUAL_PATH_MAX_LEN:
                del buf[0]
            self._actual_path_pubs[name].publish(self._make_path_msg(now, buf))

    def stop_all_robots(self):
        zero = Twist()
        for pub in self._cmd_pubs.values():
            try:
                pub.publish(zero)
            except Exception as exc:
                # On SIGINT, rclpy can tear down the context before this
                # explicit safety-stop runs (race with Ctrl+C's own shutdown
                # handler) — not dangerous (the firmware's own 5 s no-command
                # timeout stops the robot regardless). Use plain stderr, not
                # self.get_logger(), since the logger can itself depend on
                # the same (already-invalid) rclpy context and re-raise.
                print(f'stop_all_robots: publish failed during shutdown: {exc}', file=sys.stderr)

    def shutdown(self):
        """Stop all robots and terminate the policy subprocess."""
        self.stop_all_robots()
        proc = getattr(self, '_proc', None)
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()


def main(args=None):
    rclpy.init(args=args)
    node = CircleToEllipseControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
