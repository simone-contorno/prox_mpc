# prox_mpc_demo

Runnable demonstrations of the ProxMPC stack, from a standalone closed-loop
simulation of the [prox_mpc_core](../prox_mpc_core) engine to a full Nav2 + Gazebo
bring-up of the [prox_mpc_controller](../prox_mpc_controller) plugin.

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Three ways to run](#three-ways-to-run)
  - [Standalone simulation](#standalone-simulation)
  - [Interactive Nav2 (kinematic plant, no Gazebo)](#interactive-nav2-kinematic-plant-no-gazebo)
  - [Nav2 + Gazebo Harmonic simulation](#nav2--gazebo-harmonic-simulation)
- [Project Structure](#project-structure)
- [License](#license)

## Overview

This package is where the ProxMPC stack becomes something you can launch.
It provides the standalone `prox_mpc_simulation` executable and two launch files:
one closes the control loop on the engine with no external simulator, and one runs
the controller plugin inside a live Nav2 stack against Gazebo Harmonic.
The standalone path needs only the core; the Nav2 path is the real-behaviour gate
for the plugin.

## Prerequisites

- **Operating system:** Ubuntu 24.04 (Noble).
- **ROS 2 distribution:** Jazzy.
- **Build system:** `ament_cmake`.
- **Standalone simulation:** `prox_mpc_core`, `prox_mpc_msgs`, and this package
  only. `prox_mpc_msgs` is an unconditional build dependency: the standalone node
  publishes `prox_mpc_msgs/SolverDiagnostics`.
- **Nav2 + Gazebo bring-up:** additionally Nav2, `ros_gz`, and the canonical
  `nav2_minimal_tb3_sim` scenario, plus `prox_mpc_controller` (and, for the
  predictive mode, `prox_mpc_obstacle_tracker`).

ROS dependencies are declared in `package.xml` and resolved by `rosdep install`.
Third-party assets are attributed in
[THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md).

## Three ways to run

### Standalone simulation

A self-contained, closed-loop driver for the engine with no external simulator: it
drives a bundled kinematic model toward a goal, publishes the command and predicted
path, broadcasts the pose, and logs the solve time.
Use it to run, visualize, and profile the controller without Nav2.
See [doc/simulation.md](doc/simulation.md) for the full parameter and interface
reference.

```bash
colcon build --symlink-install --packages-select prox_mpc_msgs prox_mpc_core prox_mpc_demo
source install/setup.bash
```

Select the model with the `model` launch argument (`bicycle` or `unicycle`) and add
`rviz:=true` to visualize the robot:

```bash
# default: bicycle model, no RViz
ros2 launch prox_mpc_demo simulation.launch.py

# unicycle model with RViz
ros2 launch prox_mpc_demo simulation.launch.py model:=unicycle rviz:=true
```

### Interactive Nav2 (kinematic plant, no Gazebo)

Drive a kinematic model under a full Nav2 stack - global planner, the ProxMPC
`controller_server`, and costmaps - and send goals by clicking **Nav2 Goal** in the
RViz toolbar, with no Gazebo. The model's body (the R2D2 unicycle or the blue
bicycle) moves to each clicked goal. This launch lives in `prox_mpc_benchmark`,
which owns the kinematic plant and the Nav2 base config:

```bash
colcon build --symlink-install --packages-select \
  prox_mpc_msgs prox_mpc_core prox_mpc_controller prox_mpc_benchmark prox_mpc_demo
source install/setup.bash

# unicycle (R2D2 body) - default
ros2 launch prox_mpc_benchmark interactive.launch.py

# bicycle (blue bike body)
ros2 launch prox_mpc_benchmark interactive.launch.py model:=bicycle
```

### Nav2 + Gazebo Harmonic simulation

The real-behaviour gate for the controller plugin: it runs the plugin inside a
live `controller_server` driving a TurtleBot3 waffle under a full Nav2 stack, with
an opt-in predictive obstacle-avoidance mode.
See [doc/nav2-simulation.md](doc/nav2-simulation.md) for the scenarios,
configuration rationale, and verified results.

```bash
colcon build --symlink-install \
  --packages-select prox_mpc_msgs prox_mpc_core prox_mpc_controller prox_mpc_demo
source install/setup.bash

# baseline, headless (no Gazebo GUI, no RViz)
ros2 launch prox_mpc_demo nav2_simulation.launch.py

# Gazebo GUI + RViz + predictive path
ros2 launch prox_mpc_demo nav2_simulation.launch.py predictive:=True headless:=False use_rviz:=True

# send a goal into the running demo
ros2 run prox_mpc_benchmark goal_sender.py --points 2.0,-0.5,0.0 --timeout 120
```

The predictive mode additionally needs `prox_mpc_obstacle_tracker` built, since
`predictive:=True` starts the tracker and turns on the controller's in-loop
obstacle term.

## Project Structure

| Path | Contents |
| --- | --- |
| `src/simulation_node.cpp` | Standalone entry point for the closed-loop simulation node (`prox_mpc_simulation`). |
| `include/prox_mpc_demo/simulation_node.hpp` | The `SimulationNode` implementation (header-only, so tests can drive one cycle). |
| `launch/simulation.launch.py` | Standalone simulation, optional RViz. |
| `launch/nav2_simulation.launch.py` | Nav2 + Gazebo Harmonic bring-up (baseline / predictive). |
| `config/` | `simulation.yaml`, and the Nav2 params `nav2_prox_mpc.yaml` / `nav2_prox_mpc_predictive.yaml`. |
| `worlds/` | Gazebo worlds `prox_mpc_open.sdf.xacro` (open room) and `prox_mpc_world.sdf.xacro` (tb3 pillar maze). |
| `maps/` | Occupancy map `prox_mpc_open.{pgm,yaml}` for the open world. |
| `models/` | The `prox_mpc_static_box` and `prox_mpc_dynamic_actor` Gazebo models. |
| `urdf/` | Robot descriptions: `bike.urdf` (self-authored blue bicycle) and `r2d2.urdf` (R2D2-derived unicycle body). |
| `rviz/` | `simulation.rviz` (standalone simulation) and `nav2_simulation.rviz` (opened by the Nav2 + Gazebo bring-up under `use_rviz:=True`). |
| `doc/` | [simulation.md](doc/simulation.md) (standalone parameter and interface reference) and [nav2-simulation.md](doc/nav2-simulation.md) (Nav2 scenarios, configuration rationale, results). |
| `test/` | `test_simulation_node.cpp`, the GoogleTest suite driving one simulation cycle. |
| `CHANGELOG.rst` | Package changelog. |
| `THIRD_PARTY_LICENSES.md` | Attribution for the bundled third-party assets. |

The Nav2 bring-up depends on Nav2, `ros_gz`, and the canonical
`nav2_minimal_tb3_sim` scenario; the standalone simulation needs only the core.

## License

[Apache-2.0](../LICENSE) for the package code; see
[THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md) for bundled third-party assets.
