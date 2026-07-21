# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0

"""
Launch one benchmark cell (scenario x model x mode), headless.

Mode (b1) - the deterministic standalone core sim - is launched here directly:
the prox_mpc_demo simulation node and the prox_mpc_benchmark metrics node are
brought up against the same scenario geometry. Modes (a) Gazebo+Nav2 and (b2)
Nav2-without-Gazebo are heavier and are driven by run_matrix.py / the demo
bringup; see the package README for status.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from prox_mpc_benchmark.scenario_obstacles import obstacle_arrays
import yaml

PKG = 'prox_mpc_benchmark'


def _setup(context, *args, **kwargs):
    share = get_package_share_directory(PKG)
    scenario = LaunchConfiguration('scenario').perform(context)
    model = LaunchConfiguration('model').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    summary = LaunchConfiguration('summary_json').perform(context)

    if mode != 'b1':
        print(f"[benchmark.launch] mode '{mode}' is driven by run_matrix.py / demo "
              f'bringup, not this launch file.')
        return []

    ctrl = yaml.safe_load(open(os.path.join(share, 'config', 'metrics.yaml')))['control']
    scn = yaml.safe_load(
        open(os.path.join(share, 'config', 'scenarios', f'{scenario}.yaml')))['scenario']

    s = scn['start']
    goal = scn['goals'][-1]
    goals_x = [float(g['x']) for g in scn['goals']]
    goals_y = [float(g['y']) for g in scn['goals']]
    goals_t = [float(g.get('yaw', 0.0)) for g in scn['goals']]
    ref_x = [float(s['x'])] + goals_x
    ref_y = [float(s['y'])] + goals_y
    motion, cx, cy, ex, ey, radius, speed, clearance = obstacle_arrays(scn)

    sim_params = {
        'model': model, 'np': int(ctrl['np']), 'nc': int(ctrl['nc']),
        'dt': float(ctrl['dt']), 'v_ref': float(ctrl['v_ref']),
        'goal_tol': float(ctrl['goal_tol']), 'report_period': 0,
        'publish_diagnostics': True,
        'start_x': float(s['x']), 'start_y': float(s['y']),
        'start_theta': float(s.get('yaw', 0.0)),
        'goals_x': goals_x, 'goals_y': goals_y, 'goals_theta': goals_t,
        'max_obstacles': max(1, len(motion)),
    }
    if motion:
        sim_params.update({
            'obs_motion': motion, 'obs_cx': cx, 'obs_cy': cy,
            'obs_ex': ex, 'obs_ey': ey, 'obs_radius': radius,
            'obs_speed': speed, 'obs_clearance': clearance})

    metrics_params = {
        'ref_x': ref_x, 'ref_y': ref_y,
        'goal_x': float(goal['x']), 'goal_y': float(goal['y']),
        'goal_theta': float(goal.get('yaw', 0.0)),
        'goal_tol': float(ctrl['goal_tol']),
        'diagnostics_topic': '/prox_mpc/diagnostics',
        'summary_json': summary, 'scenario': scenario, 'model': model,
        'mode': 'b1', 'controller': 'proxmpc',
    }

    return [
        Node(package='prox_mpc_demo', executable='prox_mpc_simulation',
             name='prox_mpc_simulation', output='screen', parameters=[sim_params]),
        Node(package=PKG, executable='metrics_node',
             name='prox_mpc_metrics', output='screen', parameters=[metrics_params]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='static_box'),
        DeclareLaunchArgument('model', default_value='bicycle'),
        DeclareLaunchArgument('mode', default_value='b1'),
        DeclareLaunchArgument('summary_json', default_value=''),
        OpaqueFunction(function=_setup),
    ])
