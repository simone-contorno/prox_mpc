# ProxMPC

Nonlinear Model Predictive Control for ROS 2, packaged as a reusable core and a
[Nav2](https://docs.nav2.org/) controller plugin.

The controller solves the nonlinear optimal-control problem with a Sequential
Quadratic Programming (SQP) scheme that repeatedly builds and solves a Quadratic
Program with the [ProxQP](https://github.com/Simple-Robotics/proxsuite) solver,
using Eigen for linear algebra.
The same engine handles linear models for free: with linear dynamics the SQP
converges in a single QP solve.

## Packages

| Package | What it is |
| --- | --- |
| [prox_mpc_core](prox_mpc_core) | The math core (`prox_mpc::MPC` / `ProxQP` / `Model`) — the reusable SQP/QP library, no ROS node. |
| [prox_mpc_controller](prox_mpc_controller) | A Nav2 `nav2_core::Controller` plugin built on the core, verified in simulation under a full Nav2 stack. |
| [prox_mpc_obstacle_tracker](prox_mpc_obstacle_tracker) | An in-house 2D-lidar dynamic-obstacle detector and Kalman tracker; feeds the controller's predictive avoidance. |
| [prox_mpc_msgs](prox_mpc_msgs) | The `Obstacle` / `ObstacleArray` message contract between the tracker and the controller (interface-only). |
| [prox_mpc_demo](prox_mpc_demo) | Runnable demos: a standalone closed-loop simulation and a full Nav2 + Gazebo Harmonic bring-up. |
| [prox_mpc_test_models](prox_mpc_test_models) | Fault-injection `prox_mpc::Model` plugins for the controller's tests (not for production). |
| [prox_mpc_benchmark](prox_mpc_benchmark) | The scenario-driven benchmarking harness that measures accuracy, precision, and real-time behaviour across the scenario × model × controller × mode matrix, and compares ProxMPC against the stock Nav2 controllers. |

## Architecture and docs

[docs/architecture.md](docs/architecture.md) is the full-stack overview: how the
packages depend on and communicate with each other, and the runtime data flow for
the standalone, Nav2, and predictive paths.

Each package keeps its own `docs/`:

- core: [architecture](prox_mpc_core/docs/architecture.md),
  [NMPC/SQP/QP math](prox_mpc_core/docs/nmpc.md), and
  [obstacle avoidance](prox_mpc_core/docs/obstacle-avoidance.md);
- controller: [architecture](prox_mpc_controller/docs/architecture.md) and
  [control law](prox_mpc_controller/docs/control-law.md);
- obstacle tracker: [architecture](prox_mpc_obstacle_tracker/docs/architecture.md);
- demo: [standalone simulation](prox_mpc_demo/docs/simulation.md) and the
  [Nav2 + Gazebo guide](prox_mpc_demo/docs/nav2-simulation.md);
- benchmark: [harness README](prox_mpc_benchmark/README.md) and the
  [controller-comparison results](docs/controller-comparison-results.md).

A single top-to-bottom reading path across every package is in
[docs/prox-mpc.md](docs/prox-mpc.md).

## Requirements

- ROS 2 (developed and tested on **Jazzy**; the code uses only standard ROS 2 APIs).
- Eigen 3: `sudo apt install libeigen3-dev`.
- ProxQP / proxsuite: see the
  [proxsuite install guide](https://github.com/Simple-Robotics/proxsuite).
- Nav2 (`nav2_core`, `nav2_costmap_2d`) — only for `prox_mpc_controller`.

## Build

Build in an overlay workspace, never inside the package source tree.

```bash
# msgs + core + demo (no Nav2 required)
colcon build --symlink-install --packages-select prox_mpc_msgs prox_mpc_core prox_mpc_demo
source install/setup.bash

# run the bundled simulation
ros2 launch prox_mpc_demo simulation.launch.py
```

Building `prox_mpc_controller` (and, for predictive avoidance, the obstacle
tracker) additionally requires Nav2:

```bash
colcon build --symlink-install --packages-select \
  prox_mpc_msgs prox_mpc_core prox_mpc_controller prox_mpc_obstacle_tracker prox_mpc_demo
source install/setup.bash
```

See each package README and [docs/architecture.md](docs/architecture.md) for the
dependency graph.

### Target tuning (Jetson / packaging)

The portable high-optimization default is `CMAKE_BUILD_TYPE=Release` (GCC `-O3
-DNDEBUG`), set in each package behind an `if(NOT CMAKE_BUILD_TYPE)` guard, plus
`EIGEN_NO_DEBUG`. Keep architecture and link-time tuning **out of the source** and
apply it at build/packaging time so the tree stays portable across x86 CI and the
Orin/Thor boards:

- Per-board CPU tuning via a CMake toolchain file or `--cmake-args`, e.g.
  `-DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG -mcpu=cortex-a78ae"` (Orin) or the
  bloom/debian `rules` flags. Never hardcode `-march=native` / `-mcpu=native`
  (it bakes the build host CPU into the binary and breaks cross/CI builds).
- LTO via `-DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON`, guarded by
  `check_ipo_supported()` and measured — not hardcoded.
- **Never** `-Ofast` / `-ffast-math` for the solver: it breaks the IEEE-754
  semantics the SQP/QP convergence and the NaN / `isfinite` guards rely on.

Verify the loop meets `1/dt` on the actual Jetson with the demo's solve-time
logger and `tegrastats`.

## Test and lint

```bash
colcon test --packages-select prox_mpc_core prox_mpc_demo
colcon test-result --all --verbose
```

`prox_mpc_core` ships GoogleTest suites that cover the model interface and its
analytic Jacobians, an obstacle-off regression against recorded reference values,
a custom model driven through the interface, and the obstacle-avoidance
constraints. `prox_mpc_controller` drives every `nav2_core::Controller` method and
fail-safe branch through the plugin's public surface, and
`prox_mpc_obstacle_tracker` unit-tests its ROS-free clustering and tracking core.
The C++ style is enforced by `uncrustify` (the ROS 2 default formatter); `cpplint`
and `ament_copyright` are disabled (single enforced formatter, and a short SPDX
header per file with the full text in [LICENSE](LICENSE)).

## Provenance

The math core originates from the author's EMARO+ master thesis (University of
Genoa / École Centrale de Nantes, LS2N), released as the
[mynmpc](https://github.com/simone-contorno/mynmpc) repository. ProxMPC is a fresh
repository that reuses that proven core and builds a Nav2 controller plugin around
it; the SQP/QP solver, cost function, constraints, and numerical results are
carried over unchanged from that validated implementation.

## License

[Apache-2.0](LICENSE) — chosen for ROS 2 ecosystem alignment (ROS 2 and Nav2 are
Apache-2.0) and its explicit patent grant. Each source file carries a short
`SPDX-License-Identifier: Apache-2.0` header; the full text is in
[LICENSE](LICENSE) and attribution in [NOTICE](NOTICE). To cite this work:

```bibtex
@misc{ProxMPC,
  title  = {ProxMPC: Nonlinear Model Predictive Control for ROS 2},
  author = {Simone Contorno},
  year   = {2026},
  note   = {Core from the MyNMPC master thesis},
  url    = {https://github.com/simone-contorno/mynmpc}
}
```
