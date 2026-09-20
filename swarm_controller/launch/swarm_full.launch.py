"""
swarm_full.launch.py
====================
Launches the complete SwarmBot pipeline:

  1. mocap_optitrack_client   — NatNet client → /mocap_rigid_bodies
  2. mocap_republisher        — per-robot /robotN/pose + RViz markers
  3. controller + visualizer  — selected by the `experiment` launch arg:
       experiment:=circle_to_ellipse (default) — 5-robot RL circle→ellipse
         circle_to_ellipse_controller → per-robot /robotN/cmd_vel
         ellipse_visualizer           → /ellipse_markers (RViz)
       experiment:=gaussian_cluster_transfer — 5-robot RL Gaussian transfer
         gaussian_cluster_transfer_controller → per-robot /robotN/cmd_vel
         gaussian_visualizer                  → /gaussian_markers (RViz)
       experiment:=mpc — original Dubins obstacle-avoidance MPC
         swarm_mpc_controller  → broadcast /robotN/cmd_vel
         obstacle_visualizer   → /obstacle_markers (RViz)
  4. foxglove_bridge          — websocket server for Foxglove Studio
                                 (off by default — see note below;
                                 set launch_foxglove:=true to have this
                                 launch file start one itself)
  5. rviz2                    — visualization (optional, set launch_rviz:=true
                                 to also/instead launch it; default off)

See CIRCLE_TO_ELLIPSE_EXPERIMENT.md for the full real-world run procedure
for the circle_to_ellipse experiment, and
GAUSSIAN_CLUSTER_TRANSFER_EXPERIMENT.md for the gaussian_cluster_transfer
experiment.

Foxglove Studio: this file does NOT start foxglove_bridge by default —
start it once as a standing service before any of these launches:
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml
then connect Foxglove Studio to ws://<this machine's IP>:8765. All the same
topics RViz would show (/robotN/pose, /robotN/path_actual, /ellipse_markers
or /gaussian_markers, /mocap_rigid_bodies, ...) appear there automatically —
add a "3D" panel and subscribe to the MarkerArray/Path/PoseStamped topics
you want, same as you'd add displays in RViz. (launch_foxglove:=true makes
this file start its own bridge instead, but only one process can bind the
port at a time.)

Note: mocap_optitrack_w2b is intentionally not launched here — it transforms
a single rigid body into a robot-base frame (one hardcoded base_id), which
doesn't fit the multi-robot swarm case. mocap_republisher reads raw poses
directly from /mocap_rigid_bodies instead.

Usage
-----
  # Standard (circle-to-ellipse RL experiment):
  ros2 launch swarm_controller swarm_full.launch.py

  # Gaussian cluster transfer RL experiment:
  ros2 launch swarm_controller swarm_full.launch.py experiment:=gaussian_cluster_transfer

  # Original MPC obstacle-avoidance experiment:
  ros2 launch swarm_controller swarm_full.launch.py experiment:=mpc

  # Also launch RViz (off by default now that Foxglove is the default viewer):
  ros2 launch swarm_controller swarm_full.launch.py launch_rviz:=true

  # Skip Foxglove bridge:
  ros2 launch swarm_controller swarm_full.launch.py launch_foxglove:=false

  # Override log level:
  ros2 launch swarm_controller swarm_full.launch.py log_level:=debug
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()

    # ── Launch arguments ──────────────────────────────────────────────────
    ld.add_action(DeclareLaunchArgument(
        'log_level', default_value='warn',
        description='ROS log level for all nodes'))

    ld.add_action(DeclareLaunchArgument(
        'launch_rviz', default_value='false',
        description='Set to true to also launch RViz2 (off by default; '
                    'use Foxglove Studio instead, see launch_foxglove)'))

    ld.add_action(DeclareLaunchArgument(
        'launch_foxglove', default_value='false',
        description='Set to true to have this launch file start '
                    'foxglove_bridge itself. Off by default: if you already '
                    'run foxglove_bridge as a standing service (recommended '
                    '— start it once with `ros2 launch foxglove_bridge '
                    'foxglove_bridge_launch.xml` and leave it running), '
                    'setting this true will fail with "Address already in '
                    'use" on the same port.'))

    ld.add_action(DeclareLaunchArgument(
        'foxglove_port', default_value='8765',
        description='foxglove_bridge websocket port'))

    ld.add_action(DeclareLaunchArgument(
        'experiment', default_value='circle_to_ellipse',
        description="'circle_to_ellipse' (5-robot RL policy, default), "
                    "'gaussian_cluster_transfer' (5-robot RL policy), or "
                    "'mpc' (original Dubins obstacle-avoidance MPC)"))

    log_level  = LaunchConfiguration('log_level')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_foxglove = LaunchConfiguration('launch_foxglove')
    foxglove_port = LaunchConfiguration('foxglove_port')
    is_mpc              = LaunchConfigurationEquals('experiment', 'mpc')
    is_circle_to_ellipse = LaunchConfigurationEquals('experiment', 'circle_to_ellipse')
    is_gaussian_cluster_transfer = LaunchConfigurationEquals(
        'experiment', 'gaussian_cluster_transfer')

    # ── Package share directories ─────────────────────────────────────────
    mocap_client_share  = get_package_share_directory('mocap_optitrack_client')
    swarm_share         = get_package_share_directory('swarm_controller')

    # ── swarm_controller shared parameter file ────────────────────────────
    swarm_params = os.path.join(swarm_share, 'config', 'swarm_params.yaml')

    # ── 1. NatNet client ──────────────────────────────────────────────────
    natnet_config = os.path.join(mocap_client_share, 'config', 'natnetclient.yaml')
    natnet_client = Node(
        package    = 'mocap_optitrack_client',
        executable = 'mocap_optitrack_client',
        name       = 'natnet_client',
        parameters = [natnet_config],
        arguments  = ['--ros-args', '--log-level', log_level],
    )

    # ── 2. Mocap republisher ──────────────────────────────────────────────
    mocap_republisher = Node(
        package    = 'swarm_controller',
        executable = 'mocap_republisher',
        name       = 'mocap_republisher',
        parameters = [swarm_params],
        arguments  = ['--ros-args', '--log-level', log_level],
    )

    # ── 3a. Swarm MPC controller (experiment:=mpc) ────────────────────────
    swarm_mpc = Node(
        package    = 'swarm_controller',
        executable = 'swarm_mpc_controller',
        name       = 'swarm_mpc_controller',
        parameters = [swarm_params],
        arguments  = ['--ros-args', '--log-level', log_level],
        condition  = is_mpc,
    )

    # ── 3b. Circle-to-ellipse RL controller (experiment:=circle_to_ellipse) ─
    circle_to_ellipse_controller = Node(
        package    = 'swarm_controller',
        executable = 'circle_to_ellipse_controller',
        name       = 'circle_to_ellipse_controller',
        parameters = [swarm_params],
        arguments  = ['--ros-args', '--log-level', log_level],
        condition  = is_circle_to_ellipse,
    )

    # ── 3c. Gaussian cluster transfer RL controller (experiment:=gaussian_cluster_transfer)
    gaussian_cluster_transfer_controller = Node(
        package    = 'swarm_controller',
        executable = 'gaussian_cluster_transfer_controller',
        name       = 'gaussian_cluster_transfer_controller',
        parameters = [swarm_params],
        arguments  = ['--ros-args', '--log-level', log_level],
        condition  = is_gaussian_cluster_transfer,
    )

    # ── 4a. Obstacle visualizer (experiment:=mpc) ──────────────────────────
    obstacle_visualizer = Node(
        package    = 'swarm_controller',
        executable = 'obstacle_visualizer',
        name       = 'obstacle_visualizer',
        parameters = [swarm_params],
        arguments  = ['--ros-args', '--log-level', log_level],
        condition  = is_mpc,
    )

    # ── 4b. Ellipse visualizer (experiment:=circle_to_ellipse) ─────────────
    ellipse_visualizer = Node(
        package    = 'swarm_controller',
        executable = 'ellipse_visualizer',
        name       = 'ellipse_visualizer',
        parameters = [swarm_params],
        arguments  = ['--ros-args', '--log-level', log_level],
        condition  = is_circle_to_ellipse,
    )

    # ── 4c. Gaussian visualizer (experiment:=gaussian_cluster_transfer) ────
    gaussian_visualizer = Node(
        package    = 'swarm_controller',
        executable = 'gaussian_visualizer',
        name       = 'gaussian_visualizer',
        parameters = [swarm_params],
        arguments  = ['--ros-args', '--log-level', log_level],
        condition  = is_gaussian_cluster_transfer,
    )

    # ── 4d. Foxglove bridge (default viewer) ────────────────────────────────
    foxglove_bridge = Node(
        package    = 'foxglove_bridge',
        executable = 'foxglove_bridge',
        name       = 'foxglove_bridge',
        parameters = [{'port': ParameterValue(foxglove_port, value_type=int)}],
        condition  = IfCondition(launch_foxglove),
    )

    # ── 5. RViz2 (optional) ───────────────────────────────────────────────
    rviz_config = os.path.join(swarm_share, 'config', 'swarm_rviz.rviz')
    rviz_node = Node(
        package    = 'rviz2',
        executable = 'rviz2',
        name       = 'rviz2',
        # Load config file only if it exists; otherwise use defaults.
        arguments  = ['-d', rviz_config] if os.path.exists(rviz_config) else [],
        condition  = IfCondition(launch_rviz),
    )

    # ── Assemble ──────────────────────────────────────────────────────────
    ld.add_action(natnet_client)
    ld.add_action(mocap_republisher)
    ld.add_action(swarm_mpc)
    ld.add_action(circle_to_ellipse_controller)
    ld.add_action(gaussian_cluster_transfer_controller)
    ld.add_action(obstacle_visualizer)
    ld.add_action(ellipse_visualizer)
    ld.add_action(gaussian_visualizer)
    ld.add_action(foxglove_bridge)
    ld.add_action(rviz_node)

    return ld
