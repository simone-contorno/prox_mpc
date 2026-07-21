# Third-party licenses

This file records third-party material redistributed in this package and the
license under which it is used.
The package's own code and assets are covered by the repository `LICENSE`, and
the build/link dependencies are inventoried in the repository
[THIRD_PARTY_LICENSES.md](../THIRD_PARTY_LICENSES.md).

Two files are derived from Apache-2.0 upstream material. Both are in policy; the
entries below satisfy the Apache-2.0 section 4 obligations to retain attribution
and to state that the files were modified. Everything else in this package -
including `models/ackermann_robot/` - is original.

## Scalable Gazebo world

- File: `worlds/prox_mpc_scalable.sdf.xacro`
- Source: derived from
  [`nav2_minimal_tb3_sim`](https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation/tree/main/nav2_minimal_tb3_sim)
  `worlds/tb3_sandbox.sdf.xacro`, the same upstream world the demo package
  attributes in its own inventory
- License: Apache-2.0 (same text as the repository `LICENSE`)
- Modifications: reuses the upstream system-plugin, `sun`, `ground_plane`,
  scene, and physics boilerplate, and replaces the sandbox contents with an
  original parametrically scalable obstacle field driven by xacro arguments.

## Recording RViz configuration

- File: `rviz/recording.rviz`
- Source: derived from
  [`nav2_bringup`](https://github.com/ros-navigation/navigation2)
  `rviz/nav2_default_view.rviz`
- License: Apache-2.0 (same text as the repository `LICENSE`)
- Modifications: the large majority of the upstream display tree, panel, and
  `nav2_rviz_plugins` classes are carried over unchanged. Added the
  comparison-video displays - "ProxMPC Local Plan" (`/prox_mpc_local_plan`),
  "ProxMPC Predicted Obstacles" (`/prox_mpc_predicted_obstacles`), "Local Plan"
  (`/local_plan`), and "Obstacle Bodies" (`/obstacle_bodies`) - added the
  `SetGoal` tool on `/goal_pose`, and switched the view controller to `Orbit`.
