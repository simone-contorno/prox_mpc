# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0

"""
Launch the self-contained ProxMPC simulation.

The YAML config is the single source of truth: it is loaded into the node via
``parameters`` and parsed here to extract ``log_level`` so only this node's
logger verbosity is set (not the global or RMW level).

Launch arguments:
  ``model`` (``bicycle`` | ``unicycle``, default ``bicycle``) selects the kinematic model.
    It overrides the YAML ``model`` parameter and picks the matching URDF.
  ``rviz`` (default ``false``) launches RViz, ``robot_state_publisher`` and
    ``joint_state_publisher`` so the chosen robot is visualized.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

NODE_NAME = 'prox_mpc_simulation'


def launch_setup(context, *args, **kwargs):
    """Resolve the model at launch time and build the node/visualization actions."""
    pkg_share = get_package_share_directory('prox_mpc_demo')
    config = os.path.join(pkg_share, 'config', 'simulation.yaml')

    with open(config, 'r') as f:
        params = yaml.safe_load(f)
    log_level = params[NODE_NAME]['ros__parameters'].get('log_level', 'info')

    # Resolve the model now so we can read its URDF.
    model = LaunchConfiguration('model').perform(context)
    use_rviz = LaunchConfiguration('rviz')

    # The kinematic model selects its visual body: the R2D2-derived body for the
    # unicycle, and the self-authored blue bicycle for the bicycle.
    urdf_by_model = {'unicycle': 'r2d2.urdf', 'bicycle': 'bike.urdf'}
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_by_model.get(model, 'r2d2.urdf'))
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    sim_node = Node(
        package='prox_mpc_demo',
        executable='prox_mpc_simulation',
        name=NODE_NAME,
        output='screen',
        parameters=[config, {'model': model}],
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

    return [sim_node, robot_state_publisher, joint_state_publisher, rviz]


def generate_launch_description():
    """Declare the launch arguments and defer node construction to launch_setup."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'model',
                default_value='bicycle',
                choices=['bicycle', 'unicycle'],
                description='Kinematic model and URDF: bicycle (4-state) or unicycle (3-state).',
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='false',
                description='Launch RViz + robot_state_publisher + joint_state_publisher.',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
