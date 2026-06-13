# prox_mpc_core

The ProxMPC math core: a C++17, library-only nonlinear Model Predictive Control.

The controller solves the nonlinear optimal-control problem with a Sequential
Quadratic Programming (SQP) scheme that repeatedly builds and solves a Quadratic
Program with the [ProxQP](https://github.com/Simple-Robotics/proxsuite) solver,
using Eigen for linear algebra.
The same engine handles linear models for free: with linear dynamics the SQP
converges in a single QP solve.

This package contains **no ROS node**: it is the reusable library that
[prox_mpc_controller](../prox_mpc_controller) (a Nav2 plugin) and
[prox_mpc_demo](../prox_mpc_demo) (a self-contained simulation) build on.

See [../docs/architecture.md](../docs/architecture.md) for the class structure,
the QP formulation, and the solve data flow.

## Public API

Everything lives in the `prox_mpc` C++ namespace.

| Header | Contents |
| --- | --- |
| `prox_mpc/structs.hpp` | `ProbDim`, `MPCParams`, `ModelInfo`, `Constraints` (plain data) |
| `prox_mpc/model.hpp` | `Model`: vehicle interface (kinematics + constraints) |
| `prox_mpc/proxqp.hpp` | `ProxQP`: QP assembly and solve wrapper |
| `prox_mpc/mpc.hpp` | `MPC`: SQP driver and configuration |
| `prox_mpc/utils.hpp` | free functions, Eigen/ROS aliases |
| `prox_mpc/models/bike.hpp`, `prox_mpc/models/r2d2.hpp` | reference kinematic models |

## Prerequisites

- ROS 2 (developed and tested on Jazzy; the code uses only standard ROS 2 APIs).
- Eigen 3: `sudo apt install libeigen3-dev`.
- ProxQP / proxsuite: see the
  [proxsuite install guide](https://github.com/Simple-Robotics/proxsuite).

## Build

Build in an overlay workspace, never inside the package source tree.

```bash
colcon build --symlink-install --packages-select prox_mpc_core
source install/setup.bash
```

## Lint

The package enables `ament_lint_common` (uncrustify, cppcheck, lint_cmake,
xmllint). `cpplint` and `ament_copyright` are intentionally disabled: uncrustify
is the single enforced C++ formatter, and files carry a short SPDX header with
the full text in [../LICENSE](../LICENSE).

```bash
colcon test --packages-select prox_mpc_core
colcon test-result --all --verbose
```
