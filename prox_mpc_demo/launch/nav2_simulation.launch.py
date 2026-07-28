# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0

"""
Bring up Gazebo Harmonic + Nav2 with the ProxMPC controller plugin.

This is a thin wrapper over the official ``nav2_bringup/tb3_simulation_launch.py``
(the canonical Nav2 Jazzy + Gazebo Harmonic scenario): it reuses that launch's
diff-drive TurtleBot3 waffle spawn, the gz<->ros bridge, ``GZ_SIM_RESOURCE_PATH``
wiring, the Nav2 bringup, AMCL, and the static map, and overrides only the Nav2
params file (to repoint ``FollowPath`` to ``prox_mpc_controller::ProxMpcController``,
which uses the ``prox_mpc_core`` Unicycle model that matches the diff-drive waffle)
and the world.

The same launch runs any Nav2 controller and any scenario: pass a different
``params_file`` (the ``FollowPath`` plugin) and a matching ``world`` / ``map``.

Predictive (dynamic) obstacle avoidance is a single opt-in switch:

  * ``predictive:=False`` (default) uses ``config/nav2_prox_mpc.yaml`` (the in-loop
    obstacle term off - plain Nav2 navigation) and does not start the tracker.
  * ``predictive:=True`` uses ``config/nav2_prox_mpc_predictive.yaml`` (the feature
    on) and starts ``prox_mpc_obstacle_tracker`` on ``/scan``.

An explicit ``params_file:=<path>`` overrides the file chosen by ``predictive``.

RViz robot model. The stock ``turtlebot3_waffle.urdf`` points its four RViz meshes
at ``package://nav2_minimal_tb3_sim/models/*.dae``, but those install under
``models/turtlebot3_model/meshes/*.dae``, so an RViz RobotModel raised "Error loading
geometries". This wrapper runs its own ``robot_state_publisher`` on a path-corrected
copy of that URDF so the waffle RobotModel loads. ``tb3_simulation_launch.py``'s own
RSP and ``nav2_default_view.rviz`` are disabled (``use_robot_state_pub:=False``,
``use_rviz:=False``); RViz starts here on ``rviz/nav2_simulation.rviz`` (the stock
Nav2 view plus the ProxMPC local-plan and predicted-obstacle displays).

Defaults are tuned for a headless server (no Gazebo GUI, no RViz). The waffle is
spawned at (-2.0, -0.5), matching the AMCL initial pose in the params files, so
localization seeds itself without a manual "2D Pose Estimate".

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
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# The stock waffle URDF references these meshes under models/<name>.dae, but they
# install under models/turtlebot3_model/meshes/<name>.dae; correcting the subpath is
# what lets RViz load the RobotModel instead of raising "Error loading geometries".
WAFFLE_MESHES = ('waffle_base', 'tire', 'lds', 'r200')


def launch_setup(context, *args, **kwargs):
    """Resolve the switches, pick the params file, and build the launch actions."""
    demo_dir = get_package_share_directory('prox_mpc_demo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    tracker_dir = get_package_share_directory('prox_mpc_obstacle_tracker')
    tb3_sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')

    predictive = LaunchConfiguration('predictive').perform(context).lower() in ('true', '1', 'yes')
    # Resolve use_rviz to a bool here, before any returned action is visited: the tb3
    # include's launch_arguments (use_rviz:=False) leak into this scope, so a
    # LaunchConfiguration-based IfCondition would be clobbered to False and the RViz
    # node would never start. Reading it now captures the caller's value.
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() in ('true', '1', 'yes')

    # An explicit params_file wins; otherwise pick by the predictive switch.
    params_file = LaunchConfiguration('params_file').perform(context)
    if not params_file:
        params_file = os.path.join(
            demo_dir, 'config',
            'nav2_prox_mpc_predictive.yaml' if predictive else 'nav2_prox_mpc.yaml')

    # Path-corrected waffle description for the RViz RobotModel (see WAFFLE_MESHES).
    with open(os.path.join(tb3_sim_dir, 'urdf', 'turtlebot3_waffle.urdf'), 'r') as f:
        waffle_description = f.read()
    for mesh in WAFFLE_MESHES:
        waffle_description = waffle_description.replace(
            f'models/{mesh}.dae', f'models/turtlebot3_model/meshes/{mesh}.dae')

    tb3_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'tb3_simulation_launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'params_file': params_file,
            'headless': LaunchConfiguration('headless'),
            'map': LaunchConfiguration('map'),
            # This wrapper owns the RSP (path-corrected waffle) and RViz, so disable
            # tb3's own robot_state_publisher and nav2_default_view.rviz.
            'use_robot_state_pub': 'False',
            'use_rviz': 'False',
        }.items(),
    )

    # Waffle RSP on the path-corrected description: publishes /robot_description and
    # the base_footprint -> base_link -> base_scan / wheel TF that AMCL and the
    # costmaps need (Gazebo's DiffDrive plugin publishes only odom -> base_footprint).
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': waffle_description}],
    )

    actions = [tb3_sim, robot_state_publisher]

    # RViz only starts under use_rviz (resolved to a bool above, not an IfCondition,
    # to survive the tb3 include's launch-argument leak).
    if use_rviz:
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(demo_dir, 'rviz', 'nav2_simulation.rviz')],
            parameters=[{'use_sim_time': True}],
        ))

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
            description='Full path to the Gazebo world (xacro). Default: the open '
                        'room (open-space navigation). Use prox_mpc_world.sdf.xacro '
                        'for the tb3 pillar maze + baked obstacles.',
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
                        'predictive switch (baseline or predictive variant). Point it at '
                        'a different FollowPath plugin to run another controller.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='True',
            description='Run Gazebo headless (no GUI / SceneBroadcaster).',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='False',
            description='Start RViz on rviz/nav2_simulation.rviz (requires a display).',
        ),
        DeclareLaunchArgument(
            'predictive',
            default_value='False',
            description='Enable predictive (dynamic) obstacle avoidance: use the '
                        'predictive params file and start the obstacle tracker on /scan.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
