"""
swarm_full.launch.py
====================
Launches the complete SwarmBot pipeline:

  1. mocap_optitrack_client   — NatNet client → /mocap_rigid_bodies
  2. mocap_optitrack_w2b      — coordinate-frame transform
  3. mocap_republisher        — per-robot /robotN/pose + RViz markers
  4. swarm_mpc_controller     — MPC obstacle-avoidance → /robotN/cmd_vel
  5. rviz2                    — visualization (optional, set launch_rviz:=false to skip)

Usage
-----
  # Standard (Y-up Motive):
  ros2 launch swarm_controller swarm_full.launch.py

  # Z-up Motive:
  ros2 launch swarm_controller swarm_full.launch.py axis:=z_up

  # Skip RViz:
  ros2 launch swarm_controller swarm_full.launch.py launch_rviz:=false

  # Override log level:
  ros2 launch swarm_controller swarm_full.launch.py log_level:=debug
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # ── Launch arguments ──────────────────────────────────────────────────
    ld.add_action(DeclareLaunchArgument(
        'axis', default_value='y_up',
        description='Motive Up Axis setting: y_up | z_up'))

    ld.add_action(DeclareLaunchArgument(
        'log_level', default_value='warn',
        description='ROS log level for all nodes'))

    ld.add_action(DeclareLaunchArgument(
        'launch_rviz', default_value='true',
        description='Set to false to skip RViz2'))

    axis       = LaunchConfiguration('axis')
    log_level  = LaunchConfiguration('log_level')
    launch_rviz = LaunchConfiguration('launch_rviz')

    # ── Package share directories ─────────────────────────────────────────
    mocap_client_share  = get_package_share_directory('mocap_optitrack_client')
    mocap_w2b_share     = get_package_share_directory('mocap_optitrack_w2b')
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

    # ── 2. World-to-base transform ─────────────────────────────────────────
    # Choose config based on the Motive Up Axis setting.
    # We compute both paths and pick at launch time via a substitution.
    w2b_config_y = os.path.join(mocap_w2b_share, 'config', 'world_to_base_y_up.yaml')
    w2b_config_z = os.path.join(mocap_w2b_share, 'config', 'world_to_base_z_up.yaml')

    # Use y_up config by default; for z_up pass axis:=z_up.
    # (Launch substitution-based conditional config selection)
    from launch.substitutions import PythonExpression   # noqa: PLC0415
    from launch.substitutions import EqualsSubstitution  # noqa: PLC0415

    world_to_base_y = Node(
        package    = 'mocap_optitrack_w2b',
        executable = 'mocap_optitrack_w2b',
        name       = 'world_to_base',
        parameters = [w2b_config_y],
        arguments  = ['--ros-args', '--log-level', log_level],
        condition  = IfCondition(
            PythonExpression(["'", LaunchConfiguration('axis'), "' == 'y_up'"])),
    )

    world_to_base_z = Node(
        package    = 'mocap_optitrack_w2b',
        executable = 'mocap_optitrack_w2b',
        name       = 'world_to_base',
        parameters = [w2b_config_z],
        arguments  = ['--ros-args', '--log-level', log_level],
        condition  = IfCondition(
            PythonExpression(["'", LaunchConfiguration('axis'), "' == 'z_up'"])),
    )

    # ── 3. Mocap republisher ──────────────────────────────────────────────
    mocap_republisher = Node(
        package    = 'swarm_controller',
        executable = 'mocap_republisher',
        name       = 'mocap_republisher',
        parameters = [swarm_params],
        arguments  = ['--ros-args', '--log-level', log_level],
    )

    # ── 4. Swarm MPC controller ───────────────────────────────────────────
    swarm_mpc = Node(
        package    = 'swarm_controller',
        executable = 'swarm_mpc_controller',
        name       = 'swarm_mpc_controller',
        parameters = [swarm_params],
        arguments  = ['--ros-args', '--log-level', log_level],
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
    ld.add_action(world_to_base_y)
    ld.add_action(world_to_base_z)
    ld.add_action(mocap_republisher)
    ld.add_action(swarm_mpc)
    ld.add_action(rviz_node)

    return ld
