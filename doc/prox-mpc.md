# ProxMPC - The Complete Guide

This document is the single, top-to-bottom reading path for the ProxMPC workspace.
It explains, for every package, **what it is**, **how it works**, and **how to use it**, in a deliberate order that goes from the theory, through the implementation, to running the stack.

It is intentionally a *map*, not a copy: where a deeper reference already exists, this guide summarises the idea and links to it rather than repeating it.
The two companion documents it leans on most are the system-level [architecture overview](architecture.md) (how the packages fit together and the runtime data flow) and the [controller-comparison results](controller-comparison-results.md) (how ProxMPC measures up against the stock Nav2 controllers).

## How to read this guide

If you read the sections in order you will build the stack up from its math core to a full navigation system:

1. [The idea](#1-the-idea-nonlinear-mpc-by-sqpqp) - what problem ProxMPC solves and the numerical method it uses.
2. [`prox_mpc_core`](#2-prox_mpc_core---the-engine) - the solver and the vehicle-model interface, with no ROS.
3. [`prox_mpc_msgs`](#3-prox_mpc_msgs---the-obstacle-contract) - the small message contract for dynamic obstacles.
4. [`prox_mpc_controller`](#4-prox_mpc_controller---the-nav2-plugin) - the engine wrapped as a Nav2 controller.
5. [`prox_mpc_obstacle_tracker`](#5-prox_mpc_obstacle_tracker---dynamic-obstacle-perception) - the lidar tracker that feeds predictive avoidance.
6. [`prox_mpc_demo`](#6-prox_mpc_demo---runnable-demonstrations) - the two runnable bring-ups (standalone and Nav2 + Gazebo).
7. [`prox_mpc_test_models`](#7-prox_mpc_test_models---fault-injection-fixtures) - the fault-injection models used only by tests.
8. [`prox_mpc_benchmark`](#8-prox_mpc_benchmark---the-measurement-harness) - the harness that measures accuracy, precision, and real-time behaviour.
9. [Running the stack](#9-running-the-stack-the-three-modes) - the three run modes and when to use each.
10. [Extending ProxMPC](#10-extending-proxmpc-add-a-vehicle-model) - adding a vehicle model without touching the consumers.
11. [Cross-cutting conventions](#11-cross-cutting-conventions) - frames, types, safety, and licensing.

A reader who only wants to *run* something can jump to [Running the stack](#9-running-the-stack-the-three-modes); a reader who only wants the *numbers* can jump to the [comparison results](controller-comparison-results.md).

## 1. The idea: nonlinear MPC by SQP/QP

Model Predictive Control turns "follow this path safely" into an optimisation solved afresh every control step.
At each step the controller looks a fixed horizon into the future, predicts where the vehicle would go under a candidate sequence of controls, scores that prediction against a reference (track the plan, hit the cruise speed, stay smooth, avoid obstacles), and applies only the first optimal control - then repeats.

The dynamics of a real vehicle are nonlinear, so the optimal-control problem is nonlinear.
ProxMPC solves it with **Sequential Quadratic Programming (SQP)**: it linearises the dynamics around the current guess, assembles a **Quadratic Program (QP)**, and solves that QP with the [ProxQP](https://github.com/Simple-Robotics/proxsuite) solver, re-linearising and retrying only while the sub-problem fails to converge. In the nominal case that is one linearise-solve-update pass per control cycle, with the linearisation refined across cycles through the trajectory warm start ([nmpc.md](../prox_mpc_core/doc/nmpc.md)).
A useful consequence falls out for free: if the model is already linear, the first QP is exact and the SQP converges in a single solve.

Safety is split into two layers that recur throughout the stack: a **fast, convex** keep-out constraint lives *inside* the optimisation so the solver shapes a trajectory that avoids obstacles, and an **outline-only footprint veto** sits *outside* it as a backstop on the single pose the solver predicts one step ahead.
The veto rasterises only the footprint perimeter and reports the maximum edge cost, with no interior fill and no sweep between the current and the next commanded pose, so it is a backstop rather than a guarantee: it earns its keep only when costmap inflation is sized to the robot's real footprint and the local costmap's unknown-space tracking matches the deployment ([Section 4](#4-prox_mpc_controller---the-nav2-plugin)).

The full derivation - cost function, the Euler linearisation, the KKT/QP assembly, convergence, and the obstacle constraints - is in the engine's own docs:

- [`prox_mpc_core/doc/nmpc.md`](../prox_mpc_core/doc/nmpc.md) - the NMPC/SQP/QP math.
- [`prox_mpc_core/doc/obstacle-avoidance.md`](../prox_mpc_core/doc/obstacle-avoidance.md) - the in-loop disc constraint and the discrete-time control-barrier coupling.

## 2. `prox_mpc_core` - the engine

**What it is.**
A C++17, library-only NMPC engine with **no ROS node**.
It is the reusable core that everything else builds on, and it originates from the author's master-thesis solver ([mynmpc](https://github.com/simone-contorno/mynmpc)), carried over unchanged.

**How it works.**
Three classes in the `prox_mpc` namespace do the work:

- `prox_mpc::Model` - the vehicle interface.
  A model supplies the Euler linearisation (`updateA`, `updateB`, `updatec`) and, optionally, `configure(params)` (set constants by name) and `toTwist(u)` (map a control vector to a `geometry_msgs/msg/Twist`).
  `Model` is a `pluginlib` base type; the bundled `prox_mpc_core/BicycleFrontAxle` and `prox_mpc_core/BicycleRearAxle` (4-state, with a steering angle) and `prox_mpc_core/Unicycle` (3-state) are registered against it, along with `prox_mpc_core/Bicycle` as a deprecated alias for the front-axle model.
- `prox_mpc::ProxQP` - assembles the QP for one linearisation and solves it with ProxQP.
- `prox_mpc::MPC` - the SQP driver: it holds the horizons, weights, and obstacle capacity, runs the SQP loop, and exposes the solution plus the solver telemetry (`qp_info`, `sqp_iter`, `qp_iter_ext`, and the peak obstacle slack).

Because the model is loaded *by name*, adding a vehicle needs no change to the engine or its consumers - that is the extension seam the whole workspace is organised around. That holds with obstacle avoidance on as well: the obstacle-constraint assembly reads the planar position's state indices from the model's declared mapping rather than assuming a layout.

**How to use it.**
Construct an `MPC`, set the horizons/weights/obstacle capacity, `init()` it with a `Model`, then each cycle set the reference, pose, and obstacles and call `solve()`.
The engine takes no safety action of its own - convergence and finiteness gating belong to the caller (the controller and the demo both do this).

**Read more.**
[`prox_mpc_core/README.md`](../prox_mpc_core/README.md) (public API table, build/test) and [`prox_mpc_core/doc/architecture.md`](../prox_mpc_core/doc/architecture.md) (design).

## 3. `prox_mpc_msgs` - the obstacle contract

**What it is.**
An interface-only `rosidl` package: three messages, no node, no library.
It is the contract that lets the tracker and the controller evolve independently.

**How it works.**
`prox_mpc_msgs/msg/Obstacle` carries one tracked object (stable `id`, planar `position`/`velocity`, enclosing `radius`, and 2x2 position/velocity covariances); `prox_mpc_msgs/msg/ObstacleArray` carries the confirmed set for one scan, with a `header` whose `stamp` is the scan time (used to *age* the prediction) and whose `frame_id` is the non-rotating tracking frame (used to *transform* obstacles into the costmap global frame).

> The same package also defines `prox_mpc_msgs/msg/SolverDiagnostics`, the per-cycle solver-telemetry message (status, solve/QP time, residuals, SQP/QP iterations, deadline-missed flag, obstacle slack).
> It is what the controller and the standalone sim publish so the [benchmark](#8-prox_mpc_benchmark---the-measurement-harness) can observe real-time and feasibility behaviour.

**How to use it.**
Subscribe or publish the types directly; the producer/consumer QoS and field semantics are fixed by the contract.

**Read more.**
[`prox_mpc_msgs/README.md`](../prox_mpc_msgs/README.md) - full field tables and the producer/consumer QoS contract.

## 4. `prox_mpc_controller` - the Nav2 plugin

**What it is.**
A `nav2_core::Controller` plugin (`prox_mpc_controller::ProxMpcController`) that drives a robot along the global plan by solving one SQP cycle per control step on the [core](#2-prox_mpc_core---the-engine).
The engine math is unchanged; this package owns the ROS integration.

**How it works.**
Each `computeVelocityCommands` cycle the plugin:

1. transforms the global plan into the costmap global frame and samples a state/control reference along it by arc length (continuous, unwrapped heading; a curvature-aware steering reference for the bicycle; goal-approach easing);
2. reduces the local costmap to the engine's obstacle triples - a clustered, windowed scan for occupied cells - and, when predictive avoidance is on, propagates tracked dynamic obstacles over the horizon and binds each to a constraint slot (hybrid fill);
3. solves one SQP cycle, times it, and (opt-in) publishes a `SolverDiagnostics`;
4. gates the result: a non-converged or non-finite solve, or a command that fails the **outline-only footprint veto**, decelerates from the measured velocity at the model's limit and escalates to a Nav2 recovery after `max_solver_failures` consecutive faults (the veto counting on its own budget).

This is the two-layer safety split in practice: the in-loop disc constraint shapes the trajectory; the footprint veto is an outline-only backstop, not a guarantee.

**How to use it.**
Point `controller_server`'s `FollowPath` at the plugin and load its parameters; the bundled config is the single source of truth.

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "prox_mpc_controller::ProxMpcController"
      model_plugin: "prox_mpc_core/Unicycle"   # or prox_mpc_core/BicycleFrontAxle / BicycleRearAxle
      # horizons, weights, solver limits, obstacle settings:
      # see config/prox_mpc_controller.yaml and doc/architecture.md
```

**Read more.**
[`prox_mpc_controller/README.md`](../prox_mpc_controller/README.md) (features, build, troubleshooting), [`prox_mpc_controller/doc/architecture.md`](../prox_mpc_controller/doc/architecture.md) (lifecycle, interfaces/QoS, the full parameter reference, the two safety layers), and [`prox_mpc_controller/doc/control-law.md`](../prox_mpc_controller/doc/control-law.md) (the controller-side math).

## 5. `prox_mpc_obstacle_tracker` - dynamic-obstacle perception

**What it is.**
A managed lifecycle node that turns a 2D lidar scan into confirmed dynamic-obstacle tracks, written from scratch on Eigen (no third-party tracker), so it is license-clean and unit-testable without ROS.

**How it works.**
The pipeline is `LaserScan -> planar points -> adjacency clusters -> tracking-frame centroids -> IMM (CV+CTRV) tracks`, with gated greedy nearest-neighbour association, one IMM filter per object (a constant-velocity Kalman filter and a constant-turn-rate-and-velocity EKF run in parallel, blended by model probability; `imm_enabled: false` restores a legacy single-CV path), a birth/confirm/death lifecycle, and a cluster-radius cap that rejects extended structure (walls) so they are not tracked as phantom fast movers.
Confirmed tracks are published as a [`prox_mpc_msgs/ObstacleArray`](#3-prox_mpc_msgs---the-obstacle-contract) on `tracked_obstacles`, which the controller consumes for predictive avoidance.

**How to use it.**
Run the self-activating standalone node with its config, or let the demo's Nav2 launch start it automatically with `predictive:=True`.

```bash
ros2 run prox_mpc_obstacle_tracker obstacle_tracker \
  --ros-args --params-file \
  $(ros2 pkg prefix prox_mpc_obstacle_tracker)/share/prox_mpc_obstacle_tracker/config/obstacle_tracker.yaml
ros2 topic echo /tracked_obstacles
```

**Read more.**
[`prox_mpc_obstacle_tracker/README.md`](../prox_mpc_obstacle_tracker/README.md) (interfaces, run, test) and [`prox_mpc_obstacle_tracker/doc/architecture.md`](../prox_mpc_obstacle_tracker/doc/architecture.md) (algorithm, parameters, lifecycle).

## 6. `prox_mpc_demo` - runnable demonstrations

**What it is.**
The package that makes the stack runnable two ways: a standalone closed-loop simulation and a full Nav2 + Gazebo Harmonic bring-up.

**How it works.**

- **Standalone simulation** (`prox_mpc_simulation`) closes the loop on the engine with no external simulator: it solves, commands `/robot/cmd_vel`, publishes the predicted path, advances the simulated pose to the model's own prediction, and (opt-in) publishes `SolverDiagnostics`.
  It also follows a waypoint set, drives time-varying obstacles (static / circle / line), and *holds at its final goal* so the goal error is the stopping accuracy.
- **Nav2 + Gazebo** loads the controller plugin inside a live `controller_server` driving a TurtleBot3 waffle through the stock Nav2 velocity chain, with an opt-in predictive obstacle-avoidance mode.

**How to use it.**

```bash
# standalone (msgs + core + demo)
ros2 launch prox_mpc_demo simulation.launch.py

# Nav2 + Gazebo (baseline, then predictive)
ros2 launch prox_mpc_demo nav2_simulation.launch.py
ros2 launch prox_mpc_demo nav2_simulation.launch.py predictive:=True
```

**Read more.**
[`prox_mpc_demo/README.md`](../prox_mpc_demo/README.md), [`prox_mpc_demo/doc/simulation.md`](../prox_mpc_demo/doc/simulation.md) (standalone parameters), and [`prox_mpc_demo/doc/nav2-simulation.md`](../prox_mpc_demo/doc/nav2-simulation.md) (the Gazebo + Nav2 guide and configuration rationale).

## 7. `prox_mpc_test_models` - fault-injection fixtures

**What it is.**
Test-only `prox_mpc::Model` plugins that reach controller fail-safe paths the production models cannot.

**How it works.**
`prox_mpc_test_models/NonFiniteTwist` has finite linear dynamics - so the QP converges and reports `PROXQP_SOLVED` with a finite first control - but its `toTwist()` deliberately returns a non-finite command.
That is the only seam that exercises the controller's non-finite-command brake-and-escalate branch, and it is loaded through the *same* `pluginlib` path as the production models, so there is no test-only code in the production path.

**How to use it.**
It is a `<test_depend>` of the controller and is exercised by `colcon test`; it is **not for production use**.

**Read more.**
[`prox_mpc_test_models/README.md`](../prox_mpc_test_models/README.md).

## 8. `prox_mpc_benchmark` - the measurement harness

**What it is.**
The scenario-driven harness that *measures* the stack across three metric classes - **accuracy** (cross-track / goal error), **precision** (mean ± std over repeats), and **real-time / feasibility** (solver diagnostics) - for a matrix of *scenario x model x controller x run mode*.

**How it works.**
A controller-agnostic C++ live metrics node measures pose-based accuracy (cross-track against the scenario reference polyline, goal error, time-to-goal, path length) from generic signals (TF pose) and taps the `SolverDiagnostics` stream for the real-time/feasibility class, writing one per-run JSON.
Installed Python tooling orchestrates the matrix, generates scaled maps, sends goals, and aggregates the per-run JSONs into mean ± std tables.
A small C++ **kinematic plant** integrates a controller's body twist as a unicycle and publishes `/odom` + TF, so the stock Nav2 controllers and ProxMPC can be compared on the identical plant without Gazebo (mode b2).
On that comparison a **resource sampler** records the `controller_server` process's CPU and memory (from `/proc`) and the achieved control rate, and a **`nav2_core::Controller` timing decorator** wraps whichever controller is under test and wall-clock times its `computeVelocityCommands` identically, so the cross-controller comparison covers the runtime cost (per-cycle compute, CPU, RAM, frequency) at a fairly-matched operating point, not only tracking - see the [comparison results](controller-comparison-results.md#4-per-cycle-compute-cost-and-process-resources).

**How to use it.**

```bash
# one standalone (b1) cell, reproducibly
ros2 run prox_mpc_benchmark run_matrix.py --modes b1 --scenarios static_box --models bicycle
# the cross-controller comparison on the open-world cell (mode b2, no Gazebo)
ros2 run prox_mpc_benchmark run_nav2.py --controllers proxmpc,dwb,mppi,regulated_pure_pursuit
# render the result tables into the package README
ros2 run prox_mpc_benchmark aggregate.py
```

The result artifacts are local and **gitignored** (the harness performs no git operations).

**Read more.**
[`prox_mpc_benchmark/README.md`](../prox_mpc_benchmark/README.md) (layout, run modes, coverage/status, and the auto-generated result tables) and the [controller-comparison results](controller-comparison-results.md) (the narrative comparison and the conclusion on ProxMPC).

## 9. Running the stack: the three modes

The same controller is exercised three ways, in increasing fidelity and cost:

| Mode | Plant | Localization | Use it for | Where |
| --- | --- | --- | --- | --- |
| **b1** standalone | the engine's own model rollout | exact (closed on the model) | deterministic regression of the engine; solve-time profiling | [`prox_mpc_demo` standalone](../prox_mpc_demo/doc/simulation.md) |
| **b2** Nav2, no Gazebo | the [kinematic plant](#8-prox_mpc_benchmark---the-measurement-harness) | exact (static `map -> odom`) | controller-agnostic comparison under the real Nav2 loop without physics cost | [`prox_mpc_benchmark`](../prox_mpc_benchmark/README.md) |
| **a** Nav2 + Gazebo | Gazebo Harmonic physics | AMCL | the real-behaviour gate (sensors, costmaps, full velocity chain) | [`prox_mpc_demo` Nav2 guide](../prox_mpc_demo/doc/nav2-simulation.md) |

The [architecture overview](architecture.md) draws the runtime data flow for each, and the [comparison results](controller-comparison-results.md) report what each measured.

## 10. Extending ProxMPC: add a vehicle model

The `prox_mpc::Model` interface is the one seam you extend to support a new vehicle:

1. Derive from `prox_mpc::Model` and implement `updateA`/`updateB`/`updatec` (the Euler linearisation), plus `configure(params)` and `toTwist(u)`. A model whose state is not ordered `[x, y, yaw, ...]` overrides `getPlanarMapping()` to say where each planar quantity sits; the engine's obstacle-constraint assembly indexes through that declaration, so obstacle avoidance works for any ordering.
2. Register it as a `pluginlib` plugin against the `prox_mpc::Model` base (as `prox_mpc_core` does for `BicycleFrontAxle`/`BicycleRearAxle`/`Unicycle` and `prox_mpc_test_models` does for its fixture).
3. Select it by name (`model_plugin`) in the controller or the demo - no consumer code changes.

The model interface details are in [`prox_mpc_core/README.md`](../prox_mpc_core/README.md) and [`prox_mpc_core/doc/architecture.md`](../prox_mpc_core/doc/architecture.md); `prox_mpc_test_models` is a minimal worked example of a third-party model registered against the core base.

## 11. Cross-cutting conventions

These hold across the workspace and are documented once in the [architecture overview](architecture.md#cross-cutting-conventions):

- **Frames** - REP-103 conventions; the tracker estimates velocity in a fixed, non-rotating frame and the controller transforms plans and obstacles into the costmap global frame via `tf2`.
- **Safety split** - a fast convex disc constraint inside the optimisation, plus an outline-only, single-endpoint footprint veto and a limit-respecting deceleration outside it.
- **Types** - all MPC quantities are `double`; ROS parameters are `double` / `int` / `bool` / `string` only.
- **Build** - `Release` (`-O3 -DNDEBUG`) by default behind an `if(NOT CMAKE_BUILD_TYPE)` guard; never `-Ofast` / `-ffast-math` for the solver (it breaks the IEEE-754 semantics the convergence and finiteness guards rely on); per-board CPU tuning stays out of the source.
- **License** - Apache-2.0 across the workspace, with a short `SPDX-License-Identifier` header per file and the full text in `LICENSE`.

## License

[Apache-2.0](../LICENSE).
