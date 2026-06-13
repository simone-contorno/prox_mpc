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
| [prox_mpc_controller](prox_mpc_controller) | A Nav2 `nav2_core::Controller` plugin built on the core. **Skeleton**: the control law is under development. |
| [prox_mpc_demo](prox_mpc_demo) | A self-contained closed-loop simulation and benchmark for the core — no external simulator. |

See [docs/architecture.md](docs/architecture.md) for the class structure, the QP
formulation, and the solve data flow.

## Requirements

- ROS 2 (developed and tested on **Jazzy**; the code uses only standard ROS 2 APIs).
- Eigen 3: `sudo apt install libeigen3-dev`.
- ProxQP / proxsuite: see the
  [proxsuite install guide](https://github.com/Simple-Robotics/proxsuite).
- Nav2 (`nav2_core`, `nav2_costmap_2d`) — only for `prox_mpc_controller`.

## Build

Build in an overlay workspace, never inside the package source tree.

```bash
# core + demo (no Nav2 required)
colcon build --symlink-install --packages-select prox_mpc_core prox_mpc_demo
source install/setup.bash

# run the bundled simulation
ros2 launch prox_mpc_demo simulation.launch.py
```

Building `prox_mpc_controller` additionally requires Nav2; see its
[README](prox_mpc_controller/README.md).

## Lint

```bash
colcon test --packages-select prox_mpc_core prox_mpc_demo
colcon test-result --all --verbose
```

The C++ style is enforced by `uncrustify` (the ROS 2 default formatter);
`cpplint` and `ament_copyright` are disabled (single enforced formatter, and a
short SPDX header per file with the full text in [LICENSE](LICENSE)).

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
