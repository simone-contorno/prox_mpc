# prox_mpc_test_models

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)

Test-fixture `prox_mpc::Model` plugins for the ProxMPC stack.

These are fault-injection models that exercise controller fail-safe paths the bundled production models cannot reach.
They are registered against the same `prox_mpc::Model` base as the core models, so [prox_mpc_controller](../prox_mpc_controller) loads them through its ordinary `pluginlib` path during testing - no test-only seam in production code.

> **Not for production use.** This package exists only to drive tests.

## Table of Contents

- [Overview](#overview)
- [Models](#models)
- [How It Is Used](#how-it-is-used)
- [Prerequisites](#prerequisites)
- [Build](#build)
- [Project Structure](#project-structure)
- [License](#license)

## Overview

The package is a header-and-plugin fixture: it contributes one deliberately faulty `prox_mpc::Model` implementation and its `pluginlib` registration, with no executable and no ROS node.
It is a `<test_depend>` of [prox_mpc_controller](../prox_mpc_controller), so it is present only when that package's tests are built.

## Models

| Plugin name | Class | Purpose |
| --- | --- | --- |
| `prox_mpc_test_models/NonFiniteTwist` | `prox_mpc_test_models::NonFiniteTwistModel` | Finite linear dynamics (state `[x, y, theta]`, control `[v, omega]`) so the QP converges and reports `PROXQP_SOLVED` with a finite first control, but `toTwist()` deliberately returns a non-finite command (`linear.x = NaN`). |

The `NonFiniteTwist` model is the only seam that reaches the controller's non-finite-command fail-safe: a finite-mapping model cannot produce a non-finite twist from a finite control, so this fixture is required to cover that branch.
The controller test asserts the controller brakes at the model deceleration limit and then escalates to `nav2_core::NoValidControl` once the failure budget is spent.

## How It Is Used

The package is a `<test_depend>` of `prox_mpc_controller`.
Its plugins are registered against the `prox_mpc::Model` base (through [prox_mpc_test_models_plugins.xml](prox_mpc_test_models_plugins.xml), exported for the `prox_mpc_core` base package), so the controller's `pluginlib::ClassLoader<prox_mpc::Model>("prox_mpc_core", ...)` discovers them by name.

## Prerequisites

- ROS 2 Jazzy on Ubuntu 24.04.
- [prox_mpc_core](../prox_mpc_core) (the `prox_mpc::Model` base and its build export).
- `Eigen3`, `geometry_msgs`, `pluginlib`, `proxsuite` (`>= 0.6.5`), `rclcpp`, resolved by `rosdep`.

## Build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select prox_mpc_core prox_mpc_test_models
source install/setup.bash
ros2 plugin list --package prox_mpc_test_models   # lists prox_mpc_test_models/NonFiniteTwist
```

`ros2 plugin list --package prox_mpc_test_models` reports the fixture's plugin under the `prox_mpc::Model` base; the `--package` value is the package that installs the plugin description, not the base package.

## Project Structure

```text
prox_mpc_test_models/
├── include/prox_mpc_test_models/
│   └── non_finite_twist_model.hpp
├── src/
│   └── plugins.cpp
├── CHANGELOG.rst
├── CMakeLists.txt
├── package.xml
├── prox_mpc_test_models_plugins.xml
└── README.md
```

The package ships no test suite of its own; it is the fixture that the [prox_mpc_controller](../prox_mpc_controller) tests load.

## License

[Apache-2.0](../LICENSE).
Each source file carries a short `SPDX-License-Identifier: Apache-2.0` header.
