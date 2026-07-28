# prox_mpc_core

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)

The ProxMPC math core: a C++17, library-only nonlinear Model Predictive Control.

The controller solves the nonlinear optimal-control problem with a Sequential Quadratic Programming (SQP) scheme that repeatedly builds and solves a Quadratic Program with the [ProxQP](https://github.com/Simple-Robotics/proxsuite) solver, using Eigen for linear algebra.
The same engine handles linear models for free: with linear dynamics the SQP converges in a single QP solve.

This package contains **no ROS node**: it is the reusable library that [prox_mpc_controller](../prox_mpc_controller) (a Nav2 plugin) and [prox_mpc_demo](../prox_mpc_demo) (a self-contained simulation) build on.

## Table of Contents

- [Overview](#overview)
- [Public API](#public-api)
- [Models](#models)
- [Prerequisites](#prerequisites)
- [Build](#build)
- [Architecture](#architecture)
- [Project Structure](#project-structure)
- [Testing](#testing)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## Overview

The core owns the SQP/QP assembly, the tracking cost, the state/control/rate constraints, the disc-based obstacle math, and the `prox_mpc::Model` vehicle interface.
It never needs editing to gain a new vehicle model: a model is a `pluginlib` plugin loaded by name.

See [doc/architecture.md](doc/architecture.md) for the design overview, [doc/nmpc.md](doc/nmpc.md) for the NMPC/SQP/QP math, and [doc/obstacle-avoidance.md](doc/obstacle-avoidance.md) for the obstacle constraints.

## Public API

Everything lives in the `prox_mpc` C++ namespace.

| Header | Contents |
| --- | --- |
| `prox_mpc/structs.hpp` | `ProbDim`, `MPCParams`, `ModelInfo`, `Constraints` (plain data) |
| `prox_mpc/model.hpp` | `Model`: vehicle interface (kinematics + constraints) |
| `prox_mpc/proxqp.hpp` | `ProxQP`: QP assembly and solve wrapper |
| `prox_mpc/mpc.hpp` | `MPC`: SQP driver and configuration |
| `prox_mpc/utils.hpp` | free functions (`normalizeAngle`, `optimPath`), Eigen/ROS aliases |
| `prox_mpc/models/bicycle.hpp`, `prox_mpc/models/unicycle.hpp` | reference kinematic models |

## Models

A model derives from `Model` and implements three pure virtual hooks that supply the Euler linearization (`updateA`, `updateB`, `updatec`).
Two optional hooks make it loadable and usable generically: `configure(params)` sets its constants by name after construction, and `toTwist(u)` maps a control vector to a `geometry_msgs/msg/Twist`.

`Model` is a `pluginlib` base type, and the bundled models are registered as `prox_mpc_core/Bicycle` and `prox_mpc_core/Unicycle`.
A consumer can load a model by name with a `pluginlib::ClassLoader<prox_mpc::Model>` and pass it to `MPC::init`, so adding a model requires no change to this library.

| Plugin name | Class | State | Control |
| --- | --- | --- | --- |
| `prox_mpc_core/Bicycle` | `prox_mpc::Bicycle` | `[x, y, theta, delta]` (n=4) | `[v, delta_dot]` (m=2) |
| `prox_mpc_core/Unicycle` | `prox_mpc::Unicycle` | `[x, y, theta]` (n=3) | `[v, omega]` (m=2) |

Both models enable obstacle avoidance; they differ in their `toTwist` mapping, because the bicycle's second control is a steering rate and derives the yaw rate from the current steering state as `omega = v*sin(delta)/L`, while the unicycle's control is already a body twist and uses the base identity mapping.
Both are exported to `pluginlib` via [prox_mpc_core_plugins.xml](prox_mpc_core_plugins.xml).

## Prerequisites

- ROS 2 Jazzy on Ubuntu 24.04 (the code uses only standard ROS 2 APIs).
- Eigen 3: `sudo apt install libeigen3-dev`.
- ProxQP / proxsuite: the canonical reproducible provisioning is the apt package `ros-jazzy-proxsuite` (version `>= 0.6.5`); the [proxsuite install guide](https://github.com/Simple-Robotics/proxsuite) is the upstream alternative.
- `pluginlib`, `geometry_msgs`, `nav_msgs`, `rclcpp`, resolved by `rosdep`.

ROS dependencies mirror the `<depend>` entries in `package.xml` and are resolved automatically by `rosdep install`.

## Build

Build in a dedicated overlay workspace, never inside the package source tree.

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select prox_mpc_core
source install/setup.bash
```

The core builds `Release` by default to keep the SQP/QP hot path optimized, and defines `EIGEN_NO_DEBUG` to drop Eigen's internal assertions.

<details>
<summary>Discover the bundled model plugins</summary>

After building and sourcing the overlay, list the models this package registers for `pluginlib`:

```bash
ros2 plugin list --package prox_mpc_core
```

This reports `prox_mpc_core/Bicycle` and `prox_mpc_core/Unicycle` under the `prox_mpc::Model` base.

</details>

## Architecture

`Model`, `MPC`, and `ProxQP` compose the plain-data structs by inheritance.
`Model` describes one vehicle; `MPC` owns the SQP loop and a `ProxQP`; `ProxQP` assembles and solves a single QP sub-problem.

```mermaid
flowchart LR
  caller["Consumer<br/>(controller / demo)"] -->|"setPose / setGoal / setObs"| mpc["MPC<br/>SQP driver"]
  mpc -->|"solve()"| qp["ProxQP<br/>QP assembly + solve"]
  qp -->|"updateA / updateB / updatec"| model["Model<br/>(Bicycle / Unicycle / plugin)"]
  qp -->|"increments"| mpc
  mpc -->|"optimal x, u"| caller
```

The only ROS-coupled function is `optimPath()`, which converts an optimal state trajectory into a `nav_msgs/msg/Path` for visualization.
The full class diagram, the mathematical formulation, and the solve data flow are in [doc/architecture.md](doc/architecture.md) and [doc/nmpc.md](doc/nmpc.md).

## Project Structure

```text
prox_mpc_core/
├── doc/
│   ├── architecture.md
│   ├── nmpc.md
│   └── obstacle-avoidance.md
├── include/prox_mpc/
│   ├── model.hpp
│   ├── mpc.hpp
│   ├── proxqp.hpp
│   ├── structs.hpp
│   ├── utils.hpp
│   └── models/
│       ├── bicycle.hpp
│       └── unicycle.hpp
├── src/
│   ├── model.cpp
│   ├── mpc.cpp
│   ├── proxqp.cpp
│   ├── utils.cpp
│   └── plugins.cpp
├── test/
│   ├── test_model_interface.cpp
│   ├── test_mpc_regression.cpp
│   ├── test_custom_model.cpp
│   ├── test_obstacle_k.cpp
│   └── test_utils.cpp
├── CHANGELOG.rst
├── CMakeLists.txt
├── package.xml
├── prox_mpc_core_plugins.xml
└── README.md
```

## Testing

The package ships GoogleTest suites (via `ament_add_gtest`):

- `test_model_interface` - model identity, dimensions, declared bounds, the analytic Euler residual and Jacobians, and the `configure`/`toTwist` hooks.
- `test_mpc_regression` - the obstacle-off solve compared against recorded reference values, plus the SQP wall-clock budget and the move-blocking (`Nc < Np`) case.
- `test_custom_model` - a custom single-integrator model driven through the interface, including move-blocking.
- `test_obstacle_k` - the obstacle-avoidance constraint geometry, indexing, disabling, the CBF-rate effect, and the multi-obstacle case.
- `test_utils` - the ROS-facing helpers `optimPath` and `normalizeAngle`.

Build and run them in a sourced overlay workspace:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon test --packages-select prox_mpc_core
colcon test-result --all --verbose
```

The package also enables `ament_lint_common` (uncrustify, cppcheck, lint_cmake, xmllint).
`cpplint` and `ament_copyright` are intentionally disabled: uncrustify is the single enforced C++ formatter, and files carry a short SPDX header with the full text in [../LICENSE](../LICENSE).

## Troubleshooting

- **A model is not found by `pluginlib::ClassLoader`:** the overlay is not sourced, or the models library was not built; confirm the plugin appears in `ros2 plugin list --package prox_mpc_core`.
- **`proxsuite` not found at configure:** install `ros-jazzy-proxsuite` (or build proxsuite from source) and re-source the overlay.
- **The QP does not converge within the iteration caps:** the horizon, weights, or step (`dt`) make the sub-problem stiff; review the solver limits in [doc/nmpc.md](doc/nmpc.md).

## License

[Apache-2.0](../LICENSE).
Each source file carries a short `SPDX-License-Identifier: Apache-2.0` header.
