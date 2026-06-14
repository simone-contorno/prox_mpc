# prox_mpc_controller

A [Nav2](https://docs.nav2.org/) controller plugin built on
[prox_mpc_core](../prox_mpc_core).

See [docs/architecture.md](docs/architecture.md) for the Nav2 integration design
and [docs/control-law.md](docs/control-law.md) for the controller-side math
(reference construction, costmap reduction, and the failure fallback).

> **Status: skeleton.** This package satisfies the `nav2_core::Controller`
> interface, registers as a plugin, and constructs a `prox_mpc::MPC` instance,
> but `computeVelocityCommands()` is **not implemented yet** — it logs a warning
> and commands zero velocity. This is the development surface for the MPC control
> law (reference from the global plan, costmap-to-constraints mapping, solve,
> publish the first control).

## Build

This package requires Nav2 (`nav2_core`, `nav2_costmap_2d`) and is therefore not
built by the bundled overlay unless Nav2 is installed:

```bash
sudo apt install ros-$ROS_DISTRO-nav2-core ros-$ROS_DISTRO-nav2-costmap-2d

colcon build --symlink-install --packages-select prox_mpc_core prox_mpc_controller
source install/setup.bash
```

## Configuration

[config/prox_mpc_controller.yaml](config/prox_mpc_controller.yaml) is the single
source of truth for the controller's parameters: the model plugin and its
constants, the horizons and cost weights, the solver limits, and the
obstacle-avoidance settings. The model is loaded by name through `pluginlib`
(`model_plugin`, e.g. `prox_mpc_core/Bicycle`) and configured from `model_params`,
so a different model can be selected without code changes.

## Solver-failure handling

The MPC core never commands a stop on its own: `MPC::solve` reports convergence
through `qp_info.status` (equal to `PROXQP_SOLVED` only on success) and otherwise
returns the last non-converged iterate unchanged.
Detecting a failure and reacting safely is therefore the controller's job, so the
deceleration policy stays decoupled from the SQP core.

The planned policy is:

1. Check `qp_info.status` after every `solve()`.
2. On success, command the first control mapped through `Model::toTwist`.
3. On failure, do **not** command a hard zero — that is an instantaneous,
   dynamically infeasible stop. Instead ramp the last command toward zero at the
   robot's deceleration limit (the model's velocity-rate bound), so the stop
   respects the robot's limits. This is self-contained and does not assume a
   downstream velocity smoother.
4. Count consecutive failures; once `max_solver_failures` is exceeded, raise a
   controller exception so the Nav2 behavior tree triggers a recovery (stop and
   replan) rather than crawling on a decaying command.

## Use in a Nav2 stack

Once the control law is implemented, select the plugin in your controller-server
parameters:

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "prox_mpc_controller::ProxMpcController"
```

The plugin class is exported to `nav2_core` via
[prox_mpc_controller_plugin.xml](prox_mpc_controller_plugin.xml).
