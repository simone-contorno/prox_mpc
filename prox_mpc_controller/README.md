# prox_mpc_controller

A [Nav2](https://docs.nav2.org/) controller plugin built on
[prox_mpc_core](../prox_mpc_core).

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
