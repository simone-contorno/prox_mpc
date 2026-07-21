# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0

"""
Bring up the mode (b2) Nav2 stack against the kinematic plant, no Gazebo.

This is the controller-agnostic comparison harness (mode b2): it
loads the trimmed base params (config/nav2_b2_base.yaml), injects the selected
controller's FollowPath block from config/controllers/<controller>.yaml (and, for
ProxMPC, the robot's model pairing from config/robots/<robot>.yaml), and starts
map_server, planner_server, controller_server, behavior_server, bt_navigator, the
lifecycle manager, the kinematic plant, and a static map -> odom identity.

The metrics node and the goal sender are launched by run_nav2.py (which owns the
per-run summary and teardown), not here, so this file only stands up the stack.
"""

import atexit
import contextlib
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

# Seconds to hold the obstacle tracker back so it does not publish /tracked_obstacles
# during Nav2 bringup. The tracker self-activates immediately, and at 10 Hz it
# contends with controller_server's activation, intermittently pushing that
# transition past the lifecycle manager's service timeout and aborting the whole
# bringup. Starting it after the stack is up (well inside run_nav2's 10 s warmup)
# removes the race; predictions are flowing before the goal is sent.
TRACKER_START_DELAY_S = 6.0

PKG = 'prox_mpc_benchmark'


def _unlink_quietly(path):
    """Remove path, tolerating an already-removed file at interpreter shutdown."""
    with contextlib.suppress(OSError):
        os.unlink(path)


def _merged_params(context):
    """Load base params, inject the controller preset, return a written file path."""
    share = get_package_share_directory(PKG)
    demo_share = get_package_share_directory('prox_mpc_demo')
    controller = LaunchConfiguration('controller').perform(context)
    robot = LaunchConfiguration('robot').perform(context)
    map_yaml = LaunchConfiguration('map_yaml').perform(context)

    with open(os.path.join(share, 'config', 'nav2_b2_base.yaml')) as fh:
        params = yaml.safe_load(fh)
    with open(os.path.join(share, 'config', 'controllers', f'{controller}.yaml')) as fh:
        follow_path = yaml.safe_load(fh)['FollowPath']

    # ProxMPC needs the robot's prox_mpc model pairing; the stock Nav2 controllers
    # ignore it. The waffle pairing is Unicycle so the body twist matches diff drive.
    if controller in ('proxmpc', 'proxmpc_pred'):
        with open(os.path.join(share, 'config', 'robots', f'{robot}.yaml')) as fh:
            rb = yaml.safe_load(fh)['robot']
        follow_path['model_plugin'] = rb['model_plugin']
        model_params = {'L': float(rb.get('wheelbase', 0.0))}
        # The robot's max_linear_vel bounds the model's linear input, so ProxMPC's
        # commanded speed is hard-capped in the solver exactly like the stock
        # controllers cap their sampler/optimizer (matched-cap fairness). Applied
        # symmetrically (v_min = -v_max); absent/0 keeps the model's built-in limit.
        v_max = float(rb.get('max_linear_vel', 0.0))
        if v_max > 0.0:
            model_params['v_max'] = v_max
            model_params['v_min'] = -v_max
        follow_path['model_params'] = model_params

    # Wrap the real controller in the timing decorator so every controller's
    # per-cycle compute is measured identically on /FollowPath/compute_time_ms.
    timing = LaunchConfiguration('timing').perform(context).lower() in ('true', '1', 'yes')
    if timing:
        follow_path['wrapped_plugin'] = follow_path['plugin']
        follow_path['plugin'] = 'prox_mpc_benchmark::TimingControllerWrapper'

    params['controller_server']['ros__parameters']['FollowPath'] = follow_path
    params['controller_server']['ros__parameters']['controller_plugins'] = ['FollowPath']

    map_path = map_yaml if os.path.isabs(map_yaml) else \
        os.path.join(demo_share, 'maps', map_yaml)
    params['map_server']['ros__parameters']['yaml_filename'] = map_path

    # Pin the BT navigator default trees so bringup does not depend on an implicit
    # default lookup.
    bt_share = get_package_share_directory('nav2_bt_navigator')
    bt_dir = os.path.join(bt_share, 'behavior_trees')
    params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = \
        os.path.join(bt_dir, 'navigate_to_pose_w_replanning_and_recovery.xml')
    params['bt_navigator']['ros__parameters']['default_nav_through_poses_bt_xml'] = \
        os.path.join(bt_dir, 'navigate_through_poses_w_replanning_and_recovery.xml')

    # The launched nodes read this file after this function returns, so it cannot
    # be a context manager; unlink at process exit instead. A full b2 matrix is
    # hundreds of launches, and orphaned files accumulate in /tmp otherwise.
    fd, path = tempfile.mkstemp(prefix=f'b2_{controller}_', suffix='.yaml')
    with os.fdopen(fd, 'w') as fh:
        yaml.safe_dump(params, fh, default_flow_style=None, sort_keys=False)
    atexit.register(_unlink_quietly, path)
    return path


def _setup(context, *args, **kwargs):
    params_file = _merged_params(context)
    start_x = LaunchConfiguration('start_x').perform(context)
    start_y = LaunchConfiguration('start_y').perform(context)
    start_theta = LaunchConfiguration('start_theta').perform(context)
    scan_params = LaunchConfiguration('scan_params_file').perform(context)
    want_tracker = LaunchConfiguration('obstacle_tracker').perform(context).lower() \
        in ('true', '1', 'yes')

    # The scan simulator ray-casts the scenario obstacles into /scan for the local
    # costmap obstacle_layer, so every controller perceives them identically. With
    # no params file (obstacle-free cell) it publishes all-max-range clearing beams.
    scan_node_params = [scan_params] if scan_params and os.path.isfile(scan_params) else [{
        'scan_frame': 'base_link', 'odom_topic': 'odom', 'scan_topic': 'scan',
        'rate_hz': 10.0, 'num_beams': 360, 'range_min': 0.05, 'range_max': 4.0,
    }]

    lifecycle_nodes = [
        'map_server', 'planner_server', 'controller_server',
        'behavior_server', 'bt_navigator',
    ]

    def server(pkg, exe, name):
        return Node(package=pkg, executable=exe, name=name, output='screen',
                    parameters=[params_file])

    # Predictive ProxMPC: start the obstacle tracker on the simulated /scan so a
    # confirmed moving track is fed to the controller as /tracked_obstacles (odom
    # frame). The tracker's own main() self-configures and self-activates, so it is
    # not added to the Nav2 lifecycle manager's node_names (the manager would try to
    # configure an already-active node and abort the whole bringup). It is also held
    # back TRACKER_START_DELAY_S so it does not contend with controller_server's
    # activation. Its config is the single source of truth in prox_mpc_obstacle_tracker.
    tracker_nodes = []
    if want_tracker:
        tracker_share = get_package_share_directory('prox_mpc_obstacle_tracker')
        tracker_cfg = os.path.join(tracker_share, 'config', 'obstacle_tracker.yaml')
        tracker_nodes.append(TimerAction(period=TRACKER_START_DELAY_S, actions=[
            Node(package='prox_mpc_obstacle_tracker', executable='obstacle_tracker',
                 name='prox_mpc_obstacle_tracker', output='screen',
                 parameters=[tracker_cfg])]))

    return [
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='static_map_odom', output='screen',
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom']),
        Node(package=PKG, executable='kinematic_plant', name='kinematic_plant',
             output='screen',
             parameters=[{
                 'start_x': float(start_x), 'start_y': float(start_y),
                 'start_theta': float(start_theta),
                 'cmd_topic': 'cmd_vel', 'odom_topic': 'odom',
                 'odom_frame': 'odom', 'base_frame': 'base_link', 'rate_hz': 50.0,
             }]),
        Node(package=PKG, executable='scan_simulator', name='scan_simulator',
             output='screen', parameters=scan_node_params),
        server('nav2_map_server', 'map_server', 'map_server'),
        server('nav2_planner', 'planner_server', 'planner_server'),
        server('nav2_controller', 'controller_server', 'controller_server'),
        server('nav2_behaviors', 'behavior_server', 'behavior_server'),
        server('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        *tracker_nodes,
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_b2', output='screen',
             parameters=[{'autostart': True, 'node_names': lifecycle_nodes}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='proxmpc'),
        DeclareLaunchArgument('robot', default_value='waffle'),
        DeclareLaunchArgument('timing', default_value='true'),
        DeclareLaunchArgument('map_yaml', default_value='prox_mpc_open.yaml'),
        DeclareLaunchArgument('start_x', default_value='-3.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_theta', default_value='0.0'),
        DeclareLaunchArgument('scan_params_file', default_value=''),
        DeclareLaunchArgument('obstacle_tracker', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
