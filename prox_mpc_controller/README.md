# prox_mpc_controller

A [Nav2](https://docs.nav2.org/) `nav2_core::Controller` plugin that drives a
robot along the global plan by solving a nonlinear Model Predictive Control
problem each control step, built on the [prox_mpc_core](../prox_mpc_core) SQP/QP
engine.

The plugin owns the ROS integration: it loads a `prox_mpc::Model` by name, builds
the state and control reference from the global plan, reduces the local costmap
(and, optionally, tracked dynamic obstacles) to the engine's obstacle triples,
solves one SQP cycle, maps the first optimal control to a body `Twist`, and
decelerates within the robot's limits when a solve fails.
The engine math is unchanged and lives in the core.

This plugin is verified in simulation: it runs inside a live `controller_server`
driving a TurtleBot3 waffle under a full Nav2 stack in Gazebo Harmonic (see
[prox_mpc_demo/doc/nav2-simulation.md](../prox_mpc_demo/doc/nav2-simulation.md)).

## Table of Contents

- [Documentation](#documentation)
- [Key Features](#key-features)
- [Prerequisites](#prerequisites)
- [Build](#build)
- [Use in a Nav2 stack](#use-in-a-nav2-stack)
- [Testing](#testing)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## Documentation

- [doc/migration.md](doc/migration.md) - what changed on the released surface
  since 1.0.0, and what a deployment has to do about it.
- [doc/architecture.md](doc/architecture.md) - the Nav2 integration design:
  responsibility split, the controller lifecycle, the per-cycle data flow, the
  interfaces and QoS, the full parameter reference, and the two safety layers.
- [doc/control-law.md](doc/control-law.md) - the controller-side math:
  reference construction, costmap reduction, predictive obstacle propagation,
  the deceleration fallback, speed limits, and the discrete-time CBF coupling.
- Engine math is in the core: [NMPC/SQP/QP](../prox_mpc_core/doc/nmpc.md) and
  [obstacle avoidance](../prox_mpc_core/doc/obstacle-avoidance.md).

## Key Features

- **NMPC behind `nav2_core::Controller`:** one SQP cycle per control step over the
  ProxQP solver; linear models converge in a single QP solve.
- **Model selected by configuration:** the vehicle model is loaded with
  `pluginlib` (`model_plugin`, e.g. `prox_mpc_core/BicycleFrontAxle` or
  `prox_mpc_core/Unicycle`), so switching the robot model needs no code change.
- **Plan-following reference:** arc-length sampling of the global plan with a
  continuous (unwrapped) heading, a curvature-aware steering reference for the
  bicycle, optional curvature-based cruise reduction, and goal-checker approach
  easing.
- **Two-layer obstacle avoidance:** a fast in-loop disc constraint built from the
  local costmap (clustered, windowed scan) shapes the trajectory, and an
  outline-only footprint check, walked along the predicted trajectory as far as
  the robot's own stopping distance, vetoes a command whose rasterised footprint
  perimeter crosses a lethal cell.
  It is a backstop, not a guarantee: it depends on costmap inflation sized to
  the robot and on the local costmap's unknown-space tracking to be effective.
- **Predictive (dynamic) obstacle avoidance (on by default):** consumes tracked
  obstacles, follows each track's tracker-sampled predicted trajectory over the
  horizon (a constant-velocity ray when no samples are provided), binds it to a
  fixed constraint slot, and fills the remaining slots from the costmap (hybrid);
  `predict_obstacles: false` reproduces the costmap-only behavior bit-for-bit.
  The costmap-only fill reads one present-time costmap for every horizon node, so
  a moving obstacle is constrained where it was rather than where it will be.
  It is validated for single-obstacle environments; with more than one mover the
  error closes the gap the plan was routed through, and the tracker is required.
- **Safe failure handling:** a non-converged or non-finite solve decelerates from
  the measured velocity at the robot's limit and escalates to a Nav2 recovery
  after `max_solver_failures` consecutive failures; `cancel()` ramps to a stop
  and `setSpeedLimit()` applies a runtime bound from the next control cycle.
  A model that declares no control (`u`) or control-rate (`du`) bound cannot be
  braked or driven, so it fails `configure()` instead of coming up degraded.

## Prerequisites

- ROS 2 Jazzy on Ubuntu 24.04.
- [prox_mpc_core](../prox_mpc_core) and [prox_mpc_msgs](../prox_mpc_msgs)
  (workspace packages).
- Nav2: `nav2_core`, `nav2_costmap_2d`, `nav2_util`.
- Eigen 3 and ProxQP / proxsuite (transitively, through the core).
- `tf2`, `tf2_ros`, `visualization_msgs`, `rclcpp_lifecycle` (resolved by `rosdep`).

## Build

This package requires Nav2, so it is not built by the core-only overlay unless
Nav2 is installed:

```bash
sudo apt install ros-$ROS_DISTRO-nav2-core ros-$ROS_DISTRO-nav2-costmap-2d ros-$ROS_DISTRO-nav2-util

colcon build --symlink-install --packages-select \
  prox_mpc_msgs prox_mpc_core prox_mpc_controller
source install/setup.bash
```

Confirm the plugin is discoverable:

```bash
ros2 plugin list --package prox_mpc_controller   # lists prox_mpc_controller::ProxMpcController
```

The `--package` value is the package that installs the plugin description, not the
`nav2_core` base the plugin registers against.

## Use in a Nav2 stack

Select the plugin in the `controller_server` parameters and load its settings
from [config/prox_mpc_controller.yaml](config/prox_mpc_controller.yaml), the
single source of truth for the controller's parameters:

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "prox_mpc_controller::ProxMpcController"
      # model, horizons, weights, solver limits, and obstacle settings:
      # see config/prox_mpc_controller.yaml and doc/architecture.md.
```

The plugin class is exported to `nav2_core` through
[prox_mpc_controller_plugin.xml](prox_mpc_controller_plugin.xml).
The full parameter and interface reference is in
[doc/architecture.md](doc/architecture.md).

For an end-to-end, runnable Gazebo + Nav2 bring-up (baseline and predictive),
see [prox_mpc_demo](../prox_mpc_demo) and its
[Nav2 simulation guide](../prox_mpc_demo/doc/nav2-simulation.md).

## Testing

The package ships a GoogleTest suite that brings up a `LifecycleNode`, a `tf2`
buffer, and a `Costmap2DROS`, then drives every `nav2_core::Controller` method and
every fail-safe branch (empty plan, missing transform, solver failure, non-finite
pose/command, footprint veto, cancel ramp) through the plugin's public surface:

```bash
colcon test --packages-select prox_mpc_controller
colcon test-result --all --verbose
```

The non-finite-command branch is exercised with a fault-injection model from
[prox_mpc_test_models](../prox_mpc_test_models), loaded through the same
`pluginlib` path as the production models.
`uncrustify` is the enforced C++ formatter; `cpplint` and `ament_copyright` are
disabled (single formatter, and a short SPDX header per file with the full text in
[LICENSE](../LICENSE)).

## Troubleshooting

- **Plugin not listed by `ros2 plugin list --package prox_mpc_controller`:** the
  overlay is not sourced, or the package failed to build against Nav2.
- **`controller_server` aborts at configure with a model-load error:** the
  `model_plugin` name is wrong or its package is not on the overlay; the valid
  bundled names are `prox_mpc_core/Unicycle`, `prox_mpc_core/BicycleFrontAxle` and `prox_mpc_core/BicycleRearAxle`, plus `prox_mpc_core/Bicycle` as a deprecated alias for the front-axle model.
- **Robot rotates in place instead of translating:** the cruise speed samples the
  reference too close to the robot; raise `desired_linear_vel` toward the model's
  speed bound (see the config notes in the
  [Nav2 simulation guide](../prox_mpc_demo/doc/nav2-simulation.md)).
- **`NoValidControl` recoveries:** the QP is not converging within the configured
  iteration caps for the horizon and weights; review `doc/control-law.md`.
- **`controller_server` aborts at configure with a missing-bound error:** the
  loaded model declares no bound for the named channel. `u[0]` sets the speed cap
  and the `du` bounds set the deceleration ramp, so a model missing either cannot
  move or cannot brake; declare them in the model plugin.
- **`controller_server` aborts at configure on an iteration cap:** `max_int_iter_qp`,
  `max_ext_iter_qp`, and `max_iter_sqp` must all be `>= 1`; 0 is rejected by ProxQP
  and a negative value wraps to an unbounded loop in the core.

## License

[Apache-2.0](../LICENSE).
Each source file carries a short `SPDX-License-Identifier: Apache-2.0` header.
