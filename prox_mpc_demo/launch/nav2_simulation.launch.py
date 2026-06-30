# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0

"""
Bring up Gazebo Harmonic + Nav2 with the ProxMPC controller plugin.

This is a thin wrapper over the official ``nav2_bringup/tb3_simulation_launch.py``
(the canonical Nav2 Jazzy + Gazebo Harmonic scenario): it reuses that launch's
robot spawn, the gz<->ros bridge, ``GZ_SIM_RESOURCE_PATH`` wiring, the Nav2
bringup, AMCL, and the static map, and overrides only the Nav2 params file (to
repoint ``FollowPath`` to ``prox_mpc_controller::ProxMpcController``) and the
world.

Predictive (dynamic) obstacle avoidance is a single opt-in switch:

  * ``predictive:=False`` (default) uses ``config/nav2_prox_mpc.yaml`` (the
    in-loop obstacle term off — the verified normal-navigation baseline) and does
    not start the tracker.
  * ``predictive:=True`` uses ``config/nav2_prox_mpc_predictive.yaml`` (the
    feature on) and starts ``prox_mpc_obstacle_tracker`` on ``/scan``.

An explicit ``params_file:=<path>`` overrides the file chosen by ``predictive``.

Defaults are tuned for a headless server (no Gazebo GUI, no RViz). The diff-drive
TurtleBot3 waffle is spawned at (-2.0, -0.5), matching the AMCL initial pose in
the params files, so localization seeds itself without a manual "2D Pose Estimate".

Launch arguments (the rest are forwarded to tb3_simulation_launch.py):
  ``world``       full path to the world (default: prox_mpc_open.sdf.xacro).
  ``map``         full path to the occupancy map yaml (must match the world).
  ``params_file`` full path to the Nav2 params (default: chosen from ``predictive``).
  ``headless``    drop the Gazebo GUI / SceneBroadcaster (default: True).
  ``use_rviz``    start RViz (default: False; needs a display).
  ``predictive``  enable predictive obstacle avoidance + tracker (default: False).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Resolve the predictive switch, pick the params file, and build the actions."""
    demo_dir = get_package_share_directory('prox_mpc_demo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    tracker_dir = get_package_share_directory('prox_mpc_obstacle_tracker')

    predictive = LaunchConfiguration('predictive').perform(context).lower() in ('true', '1', 'yes')

    # An explicit params_file wins; otherwise pick by the predictive switch.
    params_file = LaunchConfiguration('params_file').perform(context)
    if not params_file:
        params_file = os.path.join(
            demo_dir, 'config',
            'nav2_prox_mpc_predictive.yaml' if predictive else 'nav2_prox_mpc.yaml')

    tb3_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'tb3_simulation_launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'params_file': params_file,
            'headless': LaunchConfiguration('headless'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'map': LaunchConfiguration('map'),
        }.items(),
    )

    actions = [tb3_sim]

    # Only start the tracker for the predictive feature. It self-configures and
    # activates, publishes /tracked_obstacles, and uses sim time so its TF lookups
    # and stamps share the Gazebo clock with the rest of Nav2.
    if predictive:
        actions.append(Node(
            package='prox_mpc_obstacle_tracker',
            executable='obstacle_tracker',
            name='prox_mpc_obstacle_tracker',
            output='screen',
            parameters=[
                os.path.join(tracker_dir, 'config', 'obstacle_tracker.yaml'),
                {'use_sim_time': True},
            ],
        ))
    return actions


def generate_launch_description():
    """Declare launch arguments and defer the wiring to launch_setup."""
    demo_dir = get_package_share_directory('prox_mpc_demo')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(demo_dir, 'worlds', 'prox_mpc_open.sdf.xacro'),
            description='Full path to the Gazebo world (xacro). Default: the open room '
                        '(clean SUCCEEDED gate). Use prox_mpc_world.sdf.xacro for the '
                        'tb3 pillar maze + baked obstacles.',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(demo_dir, 'maps', 'prox_mpc_open.yaml'),
            description='Full path to the occupancy map yaml (must match the world).',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Full path to the Nav2 params file. Empty -> chosen from the '
                        'predictive switch (baseline or predictive variant).',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='True',
            description='Run Gazebo headless (no GUI / SceneBroadcaster).',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='False',
            description='Start RViz (requires a display).',
        ),
        DeclareLaunchArgument(
            'predictive',
            default_value='False',
            description='Enable predictive (dynamic) obstacle avoidance: use the '
                        'predictive params file and start the obstacle tracker on /scan.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
