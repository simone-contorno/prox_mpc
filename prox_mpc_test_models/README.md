# prox_mpc_test_models

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)

Test-fixture `prox_mpc::Model` plugins for the ProxMPC stack.

These are fault-injection models that exercise controller fail-safe paths the bundled production models cannot reach.
They are registered against the same `prox_mpc::Model` base as the core models, so [prox_mpc_controller](../prox_mpc_controller) loads them through its ordinary `pluginlib` path during testing - no test-only seam in production code.

> **Not for production use.** This package exists only to drive tests.
> Its plugin and class names are not a stable interface: a fixture may be
> renamed, replaced, or removed without notice, unlike the released
> `prox_mpc_core` models it is registered alongside.

## Table of Contents

- [Overview](#overview)
- [Models](#models)
- [How It Is Used](#how-it-is-used)
- [Prerequisites](#prerequisites)
- [Build](#build)
- [Project Structure](#project-structure)
- [License](#license)

## Overview

The package is a header-and-plugin fixture: it contributes `prox_mpc::Model` implementations the bundled production models cannot stand in for (a deliberately faulty one, an asymmetric-bounds one, a permuted-state one) and their `pluginlib` registration, with no executable and no ROS node.
It is a `<test_depend>` of [prox_mpc_controller](../prox_mpc_controller), so it is present only when that package's tests are built.

## Models

| Plugin name | Class | Purpose |
| --- | --- | --- |
| `prox_mpc_test_models/NonFiniteTwist` | `prox_mpc_test_models::NonFiniteTwistModel` | Finite linear dynamics (state `[x, y, theta]`, control `[v, omega]`) so the QP converges and reports `PROXQP_SOLVED` with a finite first control, but `toTwist()` deliberately returns a non-finite command (`linear.x = NaN`). |
| `prox_mpc_test_models/AsymmetricBounds` | `prox_mpc_test_models::AsymmetricBoundsModel` | Same finite linear dynamics as `NonFiniteTwist`, but every declared bound is asymmetric (reverse speed capped tighter than forward speed; braking rate harder than accelerating rate). Both bundled production models declare symmetric bounds, so this is the only fixture that can show the controller preserving an asymmetric range instead of assuming symmetry. |
| `prox_mpc_test_models/NonFiniteTwistOtherAxes` | `prox_mpc_test_models::NonFiniteTwistOtherAxesModel` | Same finite linear dynamics as `NonFiniteTwist`, but `toTwist()` fills `linear.y` and `angular.x` with non-finite values instead of `linear.x` -- the four `Twist` components a planar Nav2 consumer never reads, so nothing exercises the controller's validation of them without this fixture. |
| `prox_mpc_test_models/PermutedPlanarMapping` | `prox_mpc_test_models::PermutedPlanarMappingModel` | Unicycle dynamics written against a state vector ordered `[theta, x, y]`, so its declared planar mapping is `idx_yaw = 0`, `idx_x = 1`, `idx_y = 2`, with obstacle avoidance on so the in-loop keep-out term is covered against a permuted state too. Every bundled model maps its position to columns 0 and 1, so this is the only fixture that can show the controller and the solver honouring the declared mapping rather than the ordering they assumed before the mapping hook existed. |

The `NonFiniteTwist` model is the only seam that reaches the controller's non-finite-command fail-safe: a finite-mapping model cannot produce a non-finite twist from a finite control, so this fixture is required to cover that branch.
The controller test asserts the controller brakes at the model deceleration limit and then escalates to `nav2_core::NoValidControl` once the failure budget is spent.

The `AsymmetricBounds` model is the only seam that reaches the controller's asymmetry-preserving speed-limit and brake-ramp logic, for the same reason: a model with symmetric bounds cannot show a controller defect that only discards sign asymmetry.

The `PermutedPlanarMapping` model is the only seam that reaches the controller's use of the declared mapping in its own assembly steps, for the same reason again: a model whose state is already `[x, y, theta]` cannot show a controller that ignores the mapping and indexes by position.

A fixture that overrides `toTwist()` with a finite mapping must also override `fromTwist()`.
The controller seeds its deceleration ramp from every control channel the inverse reports as determined, so a fixture that inherits the base `fromTwist()` while emitting its own twist hands that ramp the inverse of a mapping it does not use, and the resulting wrong value looks like a plausible control rather than an obvious fault.
`NonFiniteTwistModel` and `NonFiniteTwistOtherAxesModel` are the exception, and deliberately so: their whole purpose is to emit a non-finite twist, which cannot round-trip through any inverse, so they override neither and are excluded from the bundled models' round-trip check (`FromTwistAgreesWithToTwistOnEveryDeterminedChannel` in `prox_mpc_core`).
A new fixture with a finite twist mapping is not exempt.

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
│   ├── asymmetric_bounds_model.hpp
│   ├── non_finite_twist_model.hpp
│   ├── non_finite_twist_other_axes_model.hpp
│   └── permuted_planar_mapping_model.hpp
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
