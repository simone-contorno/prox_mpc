# prox_mpc_msgs

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)

The message contract between the ProxMPC obstacle tracker and the Nav2 controller plugin.

This is an interface-only package (`rosidl` messages, no node, no library code).
It defines the messages that carry tracked dynamic obstacles from [prox_mpc_obstacle_tracker](../prox_mpc_obstacle_tracker) to [prox_mpc_controller](../prox_mpc_controller) for predictive obstacle avoidance, plus a per-cycle solver-telemetry message used for benchmarking.

## Table of Contents

- [Overview](#overview)
- [Messages](#messages)
  - [Obstacle](#obstacle)
  - [ObstacleArray](#obstaclearray)
  - [SolverDiagnostics](#solverdiagnostics)
- [Interface Contract](#interface-contract)
- [Prerequisites](#prerequisites)
- [Build](#build)
- [Project Structure](#project-structure)
- [License](#license)

## Overview

The package generates three messages with `rosidl` and exports the runtime through `rosidl_default_runtime`.
It depends only on `std_msgs` and `geometry_msgs`, so any package on the workspace can consume the obstacle feed without pulling in the tracker or the controller.

| Message | Purpose |
| --- | --- |
| `prox_mpc_msgs/msg/Obstacle` | One tracked dynamic obstacle: id, planar position, velocity, enclosing radius, covariances, and sampled predicted positions. |
| `prox_mpc_msgs/msg/ObstacleArray` | A set of tracked obstacles published once per processed scan, with a header carrying the scan stamp and tracking frame. |
| `prox_mpc_msgs/msg/SolverDiagnostics` | Per-control-cycle NMPC/QP solver telemetry for benchmarking (real-time, feasibility, and accuracy inputs). |

## Messages

### Obstacle

A single tracked dynamic obstacle, expressed in the frame of the enclosing `ObstacleArray` header.

| Field | Type | Unit | Meaning |
| --- | --- | --- | --- |
| `id` | `uint32` | - | Stable track identifier, reused across scans for the same physical object. |
| `position` | `geometry_msgs/Point` | m | Planar centroid; `z` unused (kept 0) by the 2D tracker. |
| `velocity` | `geometry_msgs/Vector3` | m/s | Estimated velocity; `z` unused (kept 0). |
| `radius` | `float64` | m | Enclosing radius of the detected cluster. |
| `position_covariance` | `float64[4]` | m² | 2x2 position covariance, row-major `[xx, xy, yx, yy]`. Informational: the tracker fills it, but the bundled controller reads neither covariance field. |
| `velocity_covariance` | `float64[4]` | m²/s² | 2x2 velocity covariance, row-major `[xx, xy, yx, yy]`. Informational: the bundled controller's clearance growth is driven by its own `prediction_uncertainty_growth` parameter, not this field. |
| `predicted_positions` | `geometry_msgs/Point[]` | m | Sampled predicted centroid positions in the `ObstacleArray` header frame; sample `k` (0-based) is the prediction at `header.stamp + (k+1) * prediction_dt`. `z` unused (kept 0). Empty when prediction sampling is disabled (`prediction_steps: 0`); consumers then fall back to a straight constant-velocity ray. |
| `prediction_dt` | `float64` | s | Spacing between predicted samples; `0.0` when `predicted_positions` is empty. |

### ObstacleArray

A set of tracked obstacles published once per processed scan.

| Field | Type | Meaning |
| --- | --- | --- |
| `header` | `std_msgs/Header` | `stamp` is the source scan time; `frame_id` is the tracking frame (a non-rotating frame, for example `odom`). |
| `obstacles` | `Obstacle[]` | The confirmed tracks for this scan. |

### SolverDiagnostics

Per-control-cycle NMPC/QP solver telemetry, filled from fields the core already computes (`qp_info`, `sqp_iter`, `qp_iter_ext`, slack `w`).
Publishers are opt-in: on by default in the standalone simulation, off by default and only-when-subscribed in the production controller plugin (`publish_diagnostics`).

| Field | Type | Unit | Meaning |
| --- | --- | --- | --- |
| `header` | `std_msgs/Header` | - | Control-cycle stamp; `frame_id` is the robot base frame the cycle was solved in. |
| `solve_time_ms` | `float64` | ms | Wall solve time measured around `MPC::solve()`. |
| `qp_solve_time_ms` | `float64` | ms | QP-reported solve/run time from `qp_info`. |
| `status` | `uint8` | - | Outcome of the last QP, as one of this message's own `STATUS_*` constants (see below). |
| `converged` | `bool` | - | `true` when the cycle converged (`status == STATUS_SOLVED` and the applied iterate is finite); the caller's fail-safe runs otherwise. |
| `sqp_iters` | `uint32` | - | SQP iterations. |
| `qp_iters_ext` | `uint32` | - | Total external QP iterations summed over the SQP loop. |
| `primal_residual` | `float64` | - | Final QP primal residual (`qp_info.pri_res`). |
| `dual_residual` | `float64` | - | Final QP dual residual (`qp_info.dua_res`). |
| `objective` | `float64` | - | Final QP objective value (`qp_info.objValue`). |
| `max_obstacle_slack` | `float64` | m | Max obstacle soft-keep-out slack `w` over the horizon; `> 0` means the keep-out was relaxed (a safety-feasibility signal). 0 when avoidance is disabled. |
| `control_period_ms` | `float64` | ms | Measured inter-cycle wall period; `NaN` on the first cycle. |
| `deadline_missed` | `bool` | - | `true` when `solve_time_ms` exceeds the `1000*dt` budget. `control_period_ms` is deliberately excluded: the nominal period equals the budget by construction, so any period threshold would need an arbitrary slack. |
| `num_active_obstacles` | `uint16` | - | Number of filled (non-sentinel) obstacle slots considered this cycle. |

The `status` field takes one of the following constants.
They are this message's own contract, not a copy of PROXQP's `QPSolverOutput` ordering: the publisher maps the solver enum onto them explicitly, because proxsuite 0.6.5 inserted `PROXQP_SOLVED_CLOSEST_PRIMAL_FEASIBLE` in the middle of its enum and a mirrored value would have silently shifted meaning.
New states are appended, so a recorded value never changes meaning.

| Constant | Value | Meaning |
| --- | --- | --- |
| `STATUS_SOLVED` | 0 | Converged. |
| `STATUS_MAX_ITER_REACHED` | 1 | Iteration cap reached before convergence. |
| `STATUS_PRIMAL_INFEASIBLE` | 2 | Primal infeasible. |
| `STATUS_DUAL_INFEASIBLE` | 3 | Dual infeasible. |
| `STATUS_NOT_RUN` | 4 | Solver was not run this cycle. |
| `STATUS_SOLVED_CLOSEST_PRIMAL_FEASIBLE` | 5 | The closest (L2 sense) primal-feasible problem was solved. |
| `STATUS_UNKNOWN` | 255 | A solver state this message does not model (an upstream enumerator added after the mapping was written). |

## Interface Contract

The producer ([prox_mpc_obstacle_tracker](../prox_mpc_obstacle_tracker)) publishes an `ObstacleArray` per processed scan on `tracked_obstacles` (`rclcpp::QoS(KeepLast(5))`, reliable), with positions and velocities expressed in a fixed, non-rotating tracking frame and the header stamp set to the scan time.

The consumer ([prox_mpc_controller](../prox_mpc_controller)) subscribes when `predict_obstacles` is enabled (reliable, depth 5).
It uses `header.stamp` to age the prediction, `header.frame_id` to transform the obstacles into the costmap global frame, and the `radius` to size the keep-out clearance.
When `predicted_positions` is non-empty and valid (`prediction_dt` finite and positive, all samples finite), the controller interpolates the sampled polyline at its horizon times (extrapolating along the last segment beyond the span); otherwise it falls back to the straight constant-velocity ray `position + velocity * t`.
See the controller's [control-law.md](../prox_mpc_controller/doc/control-law.md) for how the fields drive predictive avoidance.

`SolverDiagnostics` is published by the controller on `<plugin>/diagnostics` (for example `FollowPath/diagnostics`, reliable, depth 10) when `publish_diagnostics` is set, and is consumed by the benchmarking tooling rather than by the control loop.

## Prerequisites

- ROS 2 Jazzy on Ubuntu 24.04.
- `rosidl_default_generators` (build) and `rosidl_default_runtime` (runtime), resolved by `rosdep`.
- `std_msgs` and `geometry_msgs`.

## Build

Build in an overlay workspace, then inspect the generated interfaces:

```bash
colcon build --symlink-install --packages-select prox_mpc_msgs
source install/setup.bash
ros2 interface show prox_mpc_msgs/msg/ObstacleArray
ros2 interface show prox_mpc_msgs/msg/SolverDiagnostics
```

## Project Structure

```text
prox_mpc_msgs/
├── msg/
│   ├── Obstacle.msg
│   ├── ObstacleArray.msg
│   └── SolverDiagnostics.msg
├── CHANGELOG.rst
├── CMakeLists.txt
├── package.xml
└── README.md
```

## License

[Apache-2.0](../LICENSE).
