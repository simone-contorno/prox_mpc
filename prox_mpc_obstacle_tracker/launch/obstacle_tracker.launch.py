# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0

"""
Launch the standalone prox_mpc_obstacle_tracker.

The bundled YAML is the single source of truth: it is loaded into the node via
``parameters`` and parsed here to extract ``log_level`` so only this node's
logger verbosity is wired (not the global or RMW level). The executable brings
itself up (configure -> activate) and tears itself down on SIGINT/SIGTERM, so no
explicit lifecycle transitions are issued here.

Launch arguments:
  ``params_file`` (default: the package's ``config/obstacle_tracker.yaml``)
    overrides the parameter file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

NODE_NAME = 'prox_mpc_obstacle_tracker'


def launch_setup(context, *args, **kwargs):
    """Resolve the params file at launch time and build the node action."""
    params_file = LaunchConfiguration('params_file').perform(context)

    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)
    log_level = params[NODE_NAME]['ros__parameters'].get('log_level', 'info')

    tracker_node = Node(
        package='prox_mpc_obstacle_tracker',
        executable='obstacle_tracker',
        name=NODE_NAME,
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', f'{NODE_NAME}:={log_level}'],
    )

    return [tracker_node]


def generate_launch_description():
    """Declare the launch arguments and defer node construction to launch_setup."""
    default_params = os.path.join(
        get_package_share_directory('prox_mpc_obstacle_tracker'),
        'config',
        'obstacle_tracker.yaml',
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=default_params,
                description='Path to the tracker parameter YAML.',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
