# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0

"""
Interactive Nav2 demo on the kinematic plant, no Gazebo.

Bring up the mode (b2) Nav2 stack for one kinematic model, publish that model's
visual body, and open RViz with the Nav2 Goal tool so goals can be clicked. The
ProxMPC controller drives the kinematic plant to each clicked goal, and the robot
body -- the R2D2 unicycle or the blue bicycle -- moves with it.

    ros2 launch prox_mpc_benchmark interactive.launch.py            # unicycle (R2D2)
    ros2 launch prox_mpc_benchmark interactive.launch.py model:=bicycle

The stack itself (kinematic plant, Nav2 servers, static map -> odom) is the same
benchmark_nav2.launch.py used by the measurement harness; this file adds the
visual body and RViz. It lives in this package because the kinematic plant and the
Nav2 base config are here (prox_mpc_demo already depends on nothing from here).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'prox_mpc_benchmark'

# Each kinematic model selects its robot pairing (which sets the ProxMPC model
# plugin, wheelbase, and matched speed cap) and its visual body.
ROBOT_BY_MODEL = {'unicycle': 'waffle', 'bicycle': 'ackermann'}
URDF_BY_MODEL = {'unicycle': 'r2d2.urdf', 'bicycle': 'bike.urdf'}


def _setup(context, *args, **kwargs):
    model = LaunchConfiguration('model').perform(context)
    controller = LaunchConfiguration('controller').perform(context)
    map_yaml = LaunchConfiguration('map_yaml').perform(context)
    start_x = LaunchConfiguration('start_x').perform(context)
    start_y = LaunchConfiguration('start_y').perform(context)

    robot = ROBOT_BY_MODEL.get(model, 'waffle')
    urdf_name = URDF_BY_MODEL.get(model, 'r2d2.urdf')

    bench_share = get_package_share_directory(PKG)
    demo_share = get_package_share_directory('prox_mpc_demo')
    nav2_share = get_package_share_directory('nav2_bringup')

    with open(os.path.join(demo_share, 'urdf', urdf_name), 'r') as fh:
        robot_description = fh.read()
    rviz_cfg = os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')

    # The same b2 stack the measurement harness uses: kinematic plant + Nav2 +
    # static map -> odom. No timing wrapper and no obstacle tracker for the demo.
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bench_share, 'launch', 'benchmark_nav2.launch.py')),
        launch_arguments={
            'controller': controller,
            'robot': robot,
            'map_yaml': map_yaml,
            'start_x': start_x,
            'start_y': start_y,
            'start_theta': '0.0',
            'timing': 'false',
            'obstacle_tracker': 'false',
        }.items())

    # The b2 stack runs on wall time (no /clock), so the visualization uses
    # use_sim_time:=false. The joint_state_publisher supplies the R2D2 body's
    # continuous wheel/head joints (the bicycle body is all-fixed).
    return [
        stack,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'use_sim_time': False,
                          'robot_description': robot_description}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='joint_state_publisher', output='screen',
             parameters=[{'use_sim_time': False}]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', rviz_cfg],
             parameters=[{'use_sim_time': False}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value='unicycle', choices=['unicycle', 'bicycle'],
            description='Kinematic model and body: unicycle (R2D2) or bicycle (blue bike).'),
        DeclareLaunchArgument('controller', default_value='proxmpc'),
        DeclareLaunchArgument('map_yaml', default_value='prox_mpc_open.yaml'),
        DeclareLaunchArgument('start_x', default_value='-2.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        OpaqueFunction(function=_setup),
    ])
