# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0

"""
Launch the self-contained ProxMPC simulation.

The YAML config is the single source of truth: it is loaded into the node via
``parameters`` and parsed here to extract ``log_level`` so only this node's
logger verbosity is set (not the global or RMW level).

Launch arguments:
  ``model`` (``bike`` | ``r2d2``, default ``bike``) selects the kinematic model.
    It overrides the YAML ``model`` parameter and picks the matching URDF.
  ``rviz`` (default ``false``) launches RViz, ``robot_state_publisher`` and
    ``joint_state_publisher`` so the chosen robot is visualized.

On a clean shutdown the node writes a run CSV; an ``OnProcessExit`` hook then
renders a trajectory PNG next to it. A hard kill (SIGKILL) skips both.
"""

from datetime import datetime
import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

NODE_NAME = 'prox_mpc_simulation'
RESULTS_DIR = '/tmp/prox_mpc_sim'


def launch_setup(context, *args, **kwargs):
    """Resolve the model at launch time and build the node/visualization actions."""
    pkg_share = get_package_share_directory('prox_mpc_demo')
    config = os.path.join(pkg_share, 'config', 'simulation.yaml')

    with open(config, 'r') as f:
        params = yaml.safe_load(f)
    log_level = params[NODE_NAME]['ros__parameters'].get('log_level', 'info')

    # Resolve the model now so we can read its URDF and name the output files.
    model = LaunchConfiguration('model').perform(context)
    use_rviz = LaunchConfiguration('rviz')

    urdf_path = os.path.join(pkg_share, 'urdf', f'{model}.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # Shared output paths so the node and the plotter agree on the file names.
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(RESULTS_DIR, f'{stamp}_{model}.csv')
    png_path = os.path.join(RESULTS_DIR, f'{stamp}_{model}.png')
    plot_script = os.path.join(
        get_package_prefix('prox_mpc_demo'), 'lib', 'prox_mpc_demo', 'plot_results.py'
    )

    sim_node = Node(
        package='prox_mpc_demo',
        executable='prox_mpc_simulation',
        name=NODE_NAME,
        output='screen',
        parameters=[config, {'model': model, 'results_csv': csv_path}],
        arguments=['--ros-args', '--log-level', f'{NODE_NAME}:={log_level}'],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
        condition=IfCondition(use_rviz),
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=IfCondition(use_rviz),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'simulation.rviz')],
        condition=IfCondition(use_rviz),
    )

    # After the sim node exits cleanly (CSV already on disk), render the plot.
    plot_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=sim_node,
            on_exit=[
                ExecuteProcess(
                    cmd=['python3', plot_script, csv_path, png_path],
                    output='screen',
                )
            ],
        )
    )

    return [sim_node, robot_state_publisher, joint_state_publisher, rviz, plot_on_exit]


def generate_launch_description():
    """Declare the launch arguments and defer node construction to launch_setup."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'model',
                default_value='bike',
                choices=['bike', 'r2d2'],
                description='Kinematic model and URDF: bike (bicycle) or r2d2 (unicycle).',
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='false',
                description='Launch RViz + robot_state_publisher + joint_state_publisher.',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
