# prox_mpc_benchmark

Scenario-driven benchmarking harness for the ProxMPC stack.
It measures three metric classes — **accuracy** (cross-track / goal error),
**precision** (mean ± std over repeats), and **real-time / feasibility** (solver
diagnostics) — across a matrix of *scenario × model × controller × run mode*.

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Build](#build)
- [Project structure](#project-structure)
- [Run modes](#run-modes)
- [Coverage and status](#coverage-and-status)
- [Notes on the metrics](#notes-on-the-metrics)
- [Configuration](#configuration)
- [Usage](#usage)
  - [Quick start](#quick-start)
  - [Standalone matrix (b1)](#standalone-matrix-b1)
  - [Cross-controller comparison (b2)](#cross-controller-comparison-b2)
  - [Aggregate the tables](#aggregate-the-tables)
- [Demonstration videos](#demonstration-videos)
- [Results](#results)
- [License](#license)

## Overview

The package contains a controller-agnostic **C++ live metrics node**
([src/metrics_node.cpp](src/metrics_node.cpp)) plus installed **Python tooling**
([scripts/](scripts/)) for orchestration, map generation, goal sending, bag
reduction, and aggregation. It **reuses** the demo worlds/maps/models rather
than duplicating them, and **owns the result artifacts**, which stay local and
gitignored — the framework performs no git operations.

The narrative companion — how ProxMPC compares against the stock Nav2 controllers
and what the suite concluded — is in
[doc/controller-comparison-results.md](../doc/controller-comparison-results.md).

## Prerequisites

- **Operating system:** Ubuntu 24.04 (Noble).
- **ROS 2 distribution:** Jazzy.
- **Build system:** `ament_cmake`.
- **Always needed:** `prox_mpc_core`, `prox_mpc_msgs`, and `prox_mpc_demo` (the
  reused worlds, maps, and models).
- **Modes a / b2:** additionally Nav2 and the stock Nav2 controllers under
  comparison (DWB, MPPI, Regulated Pure Pursuit), plus `prox_mpc_controller`;
  mode a also needs Gazebo Harmonic and `ros_gz`.

ROS dependencies are declared in `package.xml` and resolved by `rosdep install`.

## Build

Build the harness and its dependencies in an overlay workspace:

```bash
colcon build --symlink-install --packages-select \
  prox_mpc_msgs prox_mpc_core prox_mpc_controller prox_mpc_demo prox_mpc_benchmark
source install/setup.bash
```

## Project structure

- `config/scenarios/` — the four motion scenarios (`static_box`, `dynamic_circle`,
  `dynamic_line_forward`, `dynamic_line_backward`) driven by the standalone matrix,
  plus `nav2_open` for the cross-controller cell; one YAML each.
- `config/controllers/` — one preset per Nav2 controller under test: `proxmpc`,
  `dwb`, `mppi`, `regulated_pure_pursuit`.
- `config/robots/` — robot ↔ prox_mpc model pairing (`waffle`→Unicycle,
  `ackermann`→Bicycle).
- `config/metrics.yaml` — metric set, pass thresholds, repeats, shared control params.
- `config/nav2_b2_base.yaml` — the shared Nav2 stack the mode-b2 launch injects each controller preset into.
- `src/metrics_node.cpp` — live cross-track/goal-error + SolverDiagnostics tap; writes a per-run JSON.
- `src/kinematic_plant.cpp` — mode (b2) plant: integrates `/cmd_vel` as a unicycle, publishes `/odom` + TF.
- `src/timing_controller_wrapper.cpp` — a `nav2_core::Controller` decorator that wall-clock times the
  wrapped controller's `computeVelocityCommands` so every controller's per-cycle compute is measured identically.
- `launch/benchmark.launch.py` — standalone (b1) sim + metrics node for one scenario × model.
- `launch/benchmark_nav2.launch.py` — mode (b2) Nav2 + kinematic plant with the selected controller preset.
- `scripts/run_matrix.py` — orchestrate the standalone (b1) matrix × repeats.
- `scripts/run_nav2.py` — orchestrate the mode (b2) cross-controller comparison (Nav2 + plant, no Gazebo).
- `scripts/resource_sampler.py` — sample the controller_server process CPU/RSS + `/cmd_vel` rate (b2).
- `scripts/generate_map.py` — world+map generation for the scale presets (7/15/30 m).
- `scripts/goal_sender.py` — auto-send NavigateToPose / NavigateThroughPoses (modes a/b2).
- `scripts/compute_metrics.py` — bag → per-run JSON (modes a/b2).
- `scripts/aggregate.py` — per-run JSON → mean ± std tables → this README.
- `init.sh` — one-command reproducible build + single-scenario (b1) run.
- `results/` — per-run JSON, the `scenarios.json` index, progress files (gitignored).

## Run modes

- **(b1) standalone core sim** — the deterministic, ProxMPC-only regression path
  (the core drives its own model; no Gazebo). Both models. Fully wired and exercised.
- **(a) Gazebo + Nav2** — full stack, ground-truth accuracy, real-time/feasibility
  telemetry from the controller plugin's `<plugin>/diagnostics`.
- **(b2) Nav2 without Gazebo** — a kinematic-plant node (`kinematic_plant`) +
  Nav2 controller_server for controller-agnostic comparison without Gazebo physics
  cost. Wired (`benchmark_nav2.launch.py`, `run_nav2.py`) and exercised.

## Coverage and status

Harness coverage (ROS 2 Jazzy, Gazebo Harmonic, full Nav2 with the seven
controllers compared):

| Mode / capability | Coverage |
| --- | --- |
| Solver telemetry (`SolverDiagnostics`, core slack getter, standalone + plugin publishers) | live diagnostics; plugin tap under Nav2 |
| Mode **b1** — standalone core sim | both models × the 4 single-obstacle scenarios × 5 repeats |
| Mode **a** — Gazebo Harmonic + full Nav2 | waffle / Unicycle + ProxMPC, open world, goal → `SUCCEEDED` |
| Mode **b2** — Nav2 + kinematic plant + scan simulator, no Gazebo (`run_nav2.py`) | **the reported comparison**: 7 controllers × 11 scenarios × 5 repeats (10 for MPPI on obstacle cells) |
| Map scaling (`generate_map.py` 7/15/30 m + scalable world xacro) | maps matched to world geometry |
| Ackermann robot SDF + gz AckermannSteering bridge | mode-a bicycle bringup |

The cross-controller comparison runs in mode **b2** via `run_nav2.py`: each of the
seven controllers has a preset in
`config/controllers/`, `benchmark_nav2.launch.py` injects it into `FollowPath`, and
every controller drives the *identical* kinematic plant from the same start to the
same goal. On the obstacle scenarios the `scan_simulator` ray-casts the scenario
obstacles into `/scan` and a costmap `obstacle_layer` marks them, so every
controller perceives the *same* obstacle through the *same* costmap; the
controller-agnostic metrics node adds a min-clearance / collision metric. The
narrative and the conclusion on ProxMPC are in
[doc/controller-comparison-results.md](../doc/controller-comparison-results.md).

## Notes on the metrics

<details>
<summary>How the metrics are defined and why (precision, deadline-miss, obstacle placement, resources, fair tuning)</summary>

- **Precision.** Mode b1 is deterministic, so the geometric metrics (path,
  goal error, cross-track) have ≈0 std across repeats — the precision signal is
  exact reproducibility; only wall-clock timing jitters. Mode a shows small real
  variance (Gazebo non-determinism), which **is** the precision signal there.
- **`deadline_missed`.** A pure compute-overrun signal: `solve_time_ms > 1000*dt`
  (the solve did not fit the cycle budget). The measured `control_period_ms` is
  published as a separate field but is deliberately not folded into the flag — the
  nominal period equals the budget by construction, so any period threshold would
  need an arbitrary slack. The actionable real-time signal is the
  `solve_p50/p95/max` distribution.
- **Scenario obstacle placement.** Obstacles are offset off the dead-centre line:
  a perfectly head-on symmetric point obstacle gives a soft-constraint avoider no
  lateral preference and stalls it (not a meaningful avoidance test). The offset
  gives a clear side to pass while still forcing a measurable detour.
- **Standalone goal-stop.** The b1 sim holds at its final goal (a waypoint
  follower stops rather than driving through), so `goal_error_m` reflects the
  stopping accuracy; the solver keeps running so telemetry still flows.
- **Embedded-resource metrics (mode b2).** `resource_sampler.py` reads the
  `controller_server` process's CPU (% of one core) and RSS from `/proc`, the
  achieved `/cmd_vel` rate, and — via the timing decorator — each controller's
  per-cycle `computeVelocityCommands` time, so the cross-controller comparison
  includes `control_rate_hz`, `cpu_mean_pct`/`cpu_peak_pct`, `rss_peak_mb`, and
  `compute_ms_p50`/`p95`/`max` (`n/a` for b1 / mode a). Process CPU/RSS are
  per-process (the identical local costmap is included); `compute_ms_*` is the
  cleaner controller-only measure. Measured on the x86 dev host (Intel Core
  i7-10750H, 6C/12T, 31 GiB RAM, Ubuntu 24.04.4) — the ranking transfers,
  absolutes are indicative.
- **Fair tuning (cross-controller).** The predictive controllers share a **2.0 s
  prediction horizon** (ProxMPC `np·dt`, DWB `sim_time`, MPPI `time_steps·model_dt`),
  a **genuine matched hard 0.5 m/s cap** (enforced in every controller's
  solver/sampler — for ProxMPC via the model's `v_max` input bound, sourced from
  the robot config), 20 Hz rate, and a
  0.24 m goal checker strictly inside the 0.25 m scored tolerance. Each method's
  intrinsic sampling (MPPI `batch_size`, DWB sample grid) stays at its upstream
  default. Every controller received a good-faith tuning pass on its own
  documented avoidance/completion knobs within ProxMPC's iteration budget;
  `regulated_pure_pursuit`/`graceful` kept genuine wins, `dwb`/`mppi` reverted to
  stock obstacle costs after harder settings only regressed them. MPPI (the one
  unseeded stochastic controller) runs n=10 per obstacle cell; the rest n=5.
- **Result summary (matched 0.5 m/s cap).** At a matched speed cap ProxMPC holds
  the largest static-obstacle margin of the field (+0.35 m) and clears the single
  crossing and orbiting obstacles reactively, including the orbit that DWB and
  Graceful collide on. On simultaneous two-mover cells it leads its sampling peer
  MPPI (26/60 collisions) and Graceful (17/30) and matches DWB (8/30) and RPP
  (10/30) within run-to-run noise — 11/30 reactive, 9/30 predictive. Its one weak
  cell is `blind_multi_0`, the tightest two-mover geometry, where the linearised
  keep-out cannot make the non-convex per-obstacle side-choice. Per-cycle it
  computes ~1.7–4× lighter than DWB and MPPI at 5–8 % CPU against their 8.6–9.6 %.
  The full method, per-cell numbers, and conclusion are in
  [doc/controller-comparison-results.md](../doc/controller-comparison-results.md).

</details>

## Configuration

Every run is configured from the YAML files in `config/`, which are the single
source of truth for the harness.
`config/metrics.yaml` holds the metric set, pass thresholds, repeat count, and the
shared control parameters; `config/scenarios/<name>.yaml` defines each scenario's
geometry, reference polyline, and obstacles; `config/controllers/<name>.yaml`
holds one preset per Nav2 controller under comparison; and
`config/robots/<name>.yaml` pairs a robot with its prox_mpc model.
The mode-b2 launch injects the selected controller preset into the shared
`config/nav2_b2_base.yaml` stack so every controller runs against an identical Nav2
configuration.

The orchestration scripts select from these files by name.
`run_matrix.py` accepts `--scenarios`, `--models`, `--modes` (b1 here),
`--controller`, `--repeats`, and `--results-dir`; `run_nav2.py` accepts
`--scenario`, `--controllers`, `--robot`, `--repeats`, `--warmup`, and
`--results-dir`.
The launch files parametrise a single cell: `benchmark.launch.py` takes
`scenario`, `model`, `mode`, and `summary_json`; `benchmark_nav2.launch.py` takes
`controller`, `robot`, `timing`, `map_yaml`, `start_x`, `start_y`, and
`start_theta`.

## Usage

### Quick start

Run one scenario end to end — the script sources the overlay, builds the affected
packages in `~/ros2_ws`, runs the standalone (b1) cell, and prints the per-run
summary JSON.
`init.sh` lives in the package root, so run it from there:

```bash
cd ~/ros2_ws/src/prox_mpc/prox_mpc_benchmark
./init.sh static_box bicycle
```

Its usage is `./init.sh [scenario] [model]`, where `scenario` is one of
`static_box`, `dynamic_circle`, `dynamic_line_forward`, `dynamic_line_backward`
(default `static_box`) and `model` is `unicycle` or `bicycle` (default `bicycle`).
Override the workspace path with the `ROS2_WS` environment variable if the
workspace is not at `~/ros2_ws`.

### Standalone matrix (b1)

Run the whole deterministic standalone matrix (both models × the four motion
scenarios × repeats):

```bash
ros2 run prox_mpc_benchmark run_matrix.py --modes b1
```

Narrow the run to specific cells with `--scenarios` and `--models`, for example
`--scenarios static_box --models bicycle`.

### Cross-controller comparison (b2)

Compare ProxMPC against the stock Nav2 controllers on the open-world cell, Nav2 +
kinematic plant, no Gazebo:

```bash
# full peer set, one scenario
ros2 run prox_mpc_benchmark run_nav2.py --controllers proxmpc,dwb,mppi,regulated_pure_pursuit,graceful,vector_pursuit --scenario static_box

# single-scenario reproduce
ros2 run prox_mpc_benchmark run_nav2.py --scenario <nav2_open|static_box|dynamic_circle|dynamic_line_forward|dynamic_line_backward>
```

`goal_sender.py` (used internally by `run_nav2.py` and mode a) can also send a
one-off goal into a running stack:

```bash
ros2 run prox_mpc_benchmark goal_sender.py --points 2.0,-0.5,0.0 --timeout 120
```

### Aggregate the tables

Reduce the per-run JSONs into the mean ± std tables and render them into the
[Results](#results) block below:

```bash
ros2 run prox_mpc_benchmark aggregate.py
```

## Demonstration videos

Per-scenario **four-controller** comparison grids: ProxMPC, DWB, MPPI, and
Regulated Pure Pursuit driving the *same* mode (b2) scenario side by side (RViz on
the kinematic plant), so the avoidance behaviours are directly comparable. Each
obstacle is drawn as a ground-truth cylinder next to its costmap footprint. Every
GIF loops inline; **click it** for the full-resolution mp4. Cell order:
ProxMPC (top-left) · DWB (top-right) · MPPI (bottom-left) · RPP (bottom-right).

### No obstacle

[![No obstacle — ProxMPC / DWB / MPPI / RPP](doc/media/nav2_open_controllers.gif)](doc/media/nav2_open_controllers.mp4)

### Static box (dead-centre)

[![Static box — ProxMPC / DWB / MPPI / RPP](doc/media/static_box_controllers.gif)](doc/media/static_box_controllers.mp4)

### Dynamic line

[![Dynamic line — ProxMPC / DWB / MPPI / RPP](doc/media/dynamic_line_forward_controllers.gif)](doc/media/dynamic_line_forward_controllers.mp4)

### Dynamic circle

[![Dynamic circle — ProxMPC / DWB / MPPI / RPP](doc/media/dynamic_circle_controllers.gif)](doc/media/dynamic_circle_controllers.mp4)

The single-controller ProxMPC-across-scenarios grid is in the
[root README](../README.md#demonstration). Full workflow, parameters, and the
Wayland/Xvfb note are in [doc/videos.md](doc/videos.md). In short — record each
controller into its own folder, then combine per scenario:

```bash
for c in proxmpc_pred dwb mppi regulated_pure_pursuit; do
  ros2 run prox_mpc_benchmark record_scenarios.py --controller "$c" \
    --out-dir results/videos/"$c"
done
ros2 run prox_mpc_benchmark combine_grid.sh \
  --inputs results/videos/proxmpc_pred/static_box.mp4,results/videos/dwb/static_box.mp4,results/videos/mppi/static_box.mp4,results/videos/regulated_pure_pursuit/static_box.mp4 \
  --labels ProxMPC,DWB,MPPI,RPP --output doc/media/static_box_controllers.mp4
```

Per-controller clips land under `results/videos/<controller>/` (gitignored); the
committed grids + posters are under `doc/media/`.

## Results

<!-- BENCHMARK_RESULTS_START -->

*Auto-generated by `aggregate.py` from `results/scenarios.json` (477 runs). Values are mean±std (population) over repeats — the precision signal. Not committed.*

### Mode `a`

| scenario | model | controller | n | success | t_goal[s] | path[m] | goal_err[m] | ct_rms[m] | ct_max[m] | obs_gap[m] | coll | solve_p50[ms] | solve_p95[ms] | solve_max[ms] | miss | sqp | qp_ext | infeas | slack[m] | rate[Hz] | cpu[%] | rss[MB] | cmp_p50[ms] | cmp_p95[ms] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| nav2_open | unicycle | proxmpc | 2 | 100% | 12.28±0.08 | 3.79±0.01 | 0.224±0.002 | 0.006±0.001 | 0.016±0.005 | n/a | n/a | 0.238±0.012 | 0.438±0.010 | 1.952±0.009 | 0.0±0.0% | 1.00±0.00 | 4.37±0.10 | 0.0±0.0% | 0.000±0.000 | n/a | n/a | n/a | n/a | n/a |

### Mode `b1`

| scenario | model | controller | n | success | t_goal[s] | path[m] | goal_err[m] | ct_rms[m] | ct_max[m] | obs_gap[m] | coll | solve_p50[ms] | solve_p95[ms] | solve_max[ms] | miss | sqp | qp_ext | infeas | slack[m] | rate[Hz] | cpu[%] | rss[MB] | cmp_p50[ms] | cmp_p95[ms] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| dynamic_circle | bicycle | proxmpc | 5 | 100% | 5.44±0.09 | 4.80±0.01 | 0.193±0.000 | 0.005±0.000 | 0.010±0.000 | n/a | n/a | 1.357±0.043 | 4.021±0.673 | 5.713±1.230 | 0.0±0.0% | 1.00±0.00 | 9.88±0.01 | 0.0±0.0% | 0.070±0.000 | n/a | n/a | n/a | n/a | n/a |
| dynamic_circle | unicycle | proxmpc | 5 | 100% | 5.39±0.15 | 4.74±0.02 | 0.246±0.000 | 0.007±0.000 | 0.015±0.000 | n/a | n/a | 0.967±0.045 | 2.418±0.067 | 3.447±0.526 | 0.0±0.0% | 1.00±0.00 | 9.93±0.07 | 0.0±0.0% | 0.067±0.000 | n/a | n/a | n/a | n/a | n/a |
| dynamic_line_backward | bicycle | proxmpc | 5 | 100% | 5.15±0.05 | 4.81±0.00 | 0.193±0.000 | 0.036±0.000 | 0.091±0.000 | n/a | n/a | 1.715±0.020 | 3.726±0.056 | 4.611±0.045 | 0.0±0.0% | 1.00±0.00 | 10.19±0.00 | 0.0±0.0% | 0.260±0.000 | n/a | n/a | n/a | n/a | n/a |
| dynamic_line_backward | unicycle | proxmpc | 5 | 100% | 5.08±0.08 | 4.79±0.01 | 0.206±0.000 | 0.042±0.000 | 0.104±0.000 | n/a | n/a | 1.239±0.013 | 2.580±0.080 | 3.597±0.056 | 0.0±0.0% | 1.00±0.00 | 10.32±0.04 | 0.0±0.0% | 0.263±0.000 | n/a | n/a | n/a | n/a | n/a |
| dynamic_line_forward | bicycle | proxmpc | 5 | 100% | 5.13±0.03 | 4.81±0.00 | 0.193±0.000 | 0.036±0.000 | 0.091±0.000 | n/a | n/a | 1.762±0.080 | 3.848±0.116 | 4.947±0.438 | 0.0±0.0% | 1.00±0.00 | 10.19±0.01 | 0.0±0.0% | 0.260±0.000 | n/a | n/a | n/a | n/a | n/a |
| dynamic_line_forward | unicycle | proxmpc | 5 | 100% | 5.04±0.14 | 4.78±0.03 | 0.206±0.000 | 0.042±0.001 | 0.104±0.000 | n/a | n/a | 1.381±0.047 | 3.016±0.370 | 3.870±0.105 | 0.0±0.0% | 1.00±0.00 | 10.34±0.08 | 0.0±0.0% | 0.263±0.000 | n/a | n/a | n/a | n/a | n/a |
| static_box | bicycle | proxmpc | 5 | 100% | 5.14±0.12 | 4.81±0.01 | 0.178±0.000 | 0.000±0.000 | 0.000±0.000 | n/a | n/a | 0.804±0.025 | 1.450±0.033 | 2.032±0.159 | 0.0±0.0% | 1.00±0.00 | 10.00±0.00 | 0.0±0.0% | 0.496±0.000 | n/a | n/a | n/a | n/a | n/a |
| static_box | unicycle | proxmpc | 5 | 100% | 5.14±0.05 | 4.81±0.00 | 0.178±0.000 | 0.000±0.000 | 0.000±0.000 | n/a | n/a | 0.658±0.024 | 1.136±0.010 | 1.566±0.141 | 0.0±0.0% | 1.00±0.00 | 10.00±0.03 | 0.0±0.0% | 0.496±0.000 | n/a | n/a | n/a | n/a | n/a |

### Mode `b2`

| scenario | model | controller | n | success | t_goal[s] | path[m] | goal_err[m] | ct_rms[m] | ct_max[m] | obs_gap[m] | coll | solve_p50[ms] | solve_p95[ms] | solve_max[ms] | miss | sqp | qp_ext | infeas | slack[m] | rate[Hz] | cpu[%] | rss[MB] | cmp_p50[ms] | cmp_p95[ms] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| blind_multi_0 | unicycle | dwb | 5 | 100% | 14.61±0.67 | 4.78±0.01 | 0.238±0.001 | 0.053±0.021 | 0.118±0.022 | -0.016±0.061 | 40% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.08±0.00 | 9.19±0.11 | 59.74±0.15 | 2.734±0.090 | 3.509±0.079 |
| blind_multi_0 | unicycle | graceful | 5 | 100% | 16.38±0.46 | 4.85±0.02 | 0.209±0.008 | 0.057±0.010 | 0.139±0.017 | 0.033±0.063 | 40% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.09±0.02 | 4.38±0.19 | 55.85±0.07 | 0.154±0.006 | 0.200±0.014 |
| blind_multi_0 | unicycle | mppi | 10 | 100% | 21.65±6.42 | 5.95±1.11 | 0.233±0.005 | 0.101±0.039 | 0.240±0.131 | -0.205±0.100 | 100% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.06±0.04 | 9.13±0.27 | 63.87±0.91 | 2.755±0.122 | 3.266±0.121 |
| blind_multi_0 | unicycle | proxmpc | 5 | 100% | 26.42±2.35 | 5.36±0.23 | 0.198±0.035 | 0.119±0.051 | 0.246±0.064 | -0.143±0.081 | 100% | 1.452±0.187 | 5.180±0.477 | 11.608±3.679 | 0.0±0.0% | 1.00±0.00 | 8.27±0.31 | 0.0±0.0% | 0.419±0.100 | 16.78±1.70 | 7.60±0.22 | 58.50±0.06 | 1.441±0.168 | 5.037±0.404 |
| blind_multi_0 | unicycle | proxmpc_pred | 5 | 80% | 33.84±3.57 | 6.15±0.21 | 0.336±0.218 | 0.203±0.077 | 0.440±0.115 | -0.165±0.174 | 80% | 1.538±0.110 | 4.673±0.827 | 15.241±2.124 | 0.0±0.0% | 1.00±0.00 | 8.84±0.26 | 0.0±0.0% | 0.843±0.656 | 15.72±0.44 | 7.04±0.78 | 59.83±0.08 | 1.605±0.098 | 4.458±0.862 |
| blind_multi_0 | unicycle | regulated_pure_pursuit | 5 | 100% | 15.18±1.09 | 4.83±0.02 | 0.235±0.005 | 0.094±0.004 | 0.178±0.014 | 0.026±0.095 | 60% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 19.22±0.31 | 4.59±0.09 | 55.22±0.07 | 0.212±0.008 | 0.300±0.040 |
| blind_multi_0 | unicycle | vector_pursuit | 5 | 100% | 24.05±3.55 | 4.75±0.12 | 0.236±0.001 | 0.057±0.015 | 0.122±0.023 | -0.024±0.070 | 40% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 15.36±1.15 | 4.19±0.10 | 55.07±0.05 | 0.200±0.012 | 0.324±0.020 |
| blind_multi_1 | unicycle | dwb | 5 | 100% | 14.91±0.48 | 4.78±0.01 | 0.237±0.003 | 0.042±0.012 | 0.096±0.028 | 0.199±0.038 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.08±0.00 | 9.09±0.11 | 59.58±0.07 | 2.751±0.068 | 3.251±0.090 |
| blind_multi_1 | unicycle | graceful | 5 | 100% | 15.82±1.18 | 4.84±0.04 | 0.228±0.009 | 0.064±0.030 | 0.153±0.067 | 0.078±0.053 | 20% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.07±0.00 | 4.39±0.15 | 55.75±0.06 | 0.180±0.013 | 0.275±0.032 |
| blind_multi_1 | unicycle | mppi | 10 | 100% | 15.74±0.84 | 4.84±0.04 | 0.231±0.006 | 0.062±0.018 | 0.133±0.032 | -0.074±0.155 | 70% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.07±0.00 | 8.92±0.17 | 63.21±0.65 | 2.727±0.097 | 3.297±0.119 |
| blind_multi_1 | unicycle | proxmpc | 5 | 100% | 20.24±5.42 | 4.99±0.22 | 0.220±0.026 | 0.126±0.060 | 0.260±0.086 | 0.025±0.174 | 20% | 1.278±0.160 | 4.352±0.849 | 8.053±1.120 | 0.0±0.0% | 1.00±0.00 | 7.97±0.10 | 0.0±0.0% | 0.317±0.004 | 18.72±1.62 | 7.27±0.12 | 58.46±0.15 | 1.374±0.111 | 4.565±0.889 |
| blind_multi_1 | unicycle | proxmpc_pred | 5 | 100% | 22.88±6.56 | 5.50±0.62 | 0.204±0.046 | 0.202±0.037 | 0.363±0.044 | -0.056±0.164 | 60% | 1.280±0.077 | 4.700±1.018 | 13.914±0.817 | 0.0±0.0% | 1.00±0.00 | 8.27±0.21 | 0.0±0.0% | 0.347±0.068 | 17.70±2.10 | 7.56±0.79 | 59.73±0.08 | 1.355±0.096 | 4.369±1.015 |
| blind_multi_1 | unicycle | regulated_pure_pursuit | 5 | 100% | 16.87±3.81 | 4.94±0.18 | 0.232±0.002 | 0.144±0.072 | 0.247±0.089 | 0.063±0.130 | 40% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 18.17±2.28 | 4.26±0.17 | 55.21±0.07 | 0.213±0.006 | 0.308±0.041 |
| blind_multi_1 | unicycle | vector_pursuit | 5 | 100% | 20.19±2.72 | 4.86±0.01 | 0.236±0.001 | 0.157±0.025 | 0.266±0.033 | -0.006±0.085 | 40% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 17.60±1.58 | 4.28±0.18 | 55.15±0.06 | 0.207±0.012 | 0.322±0.023 |
| blind_multi_2 | unicycle | dwb | 5 | 80% | 13.87±1.03 | 3.82±1.91 | 1.188±1.906 | 0.031±0.017 | 0.058±0.030 | 0.209±0.932 | 80% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.05±0.09 | 9.08±0.17 | 59.76±0.06 | 2.712±0.029 | 3.462±0.111 |
| blind_multi_2 | unicycle | graceful | 5 | 100% | 16.47±1.04 | 4.86±0.05 | 0.224±0.012 | 0.106±0.058 | 0.198±0.105 | -0.125±0.013 | 100% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.00±0.07 | 4.35±0.18 | 55.82±0.18 | 0.157±0.006 | 0.241±0.021 |
| blind_multi_2 | unicycle | mppi | 10 | 100% | 17.89±1.13 | 5.26±0.12 | 0.233±0.004 | 0.083±0.025 | 0.173±0.051 | 0.035±0.126 | 10% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.06±0.00 | 9.00±0.25 | 63.67±0.66 | 2.757±0.115 | 3.288±0.189 |
| blind_multi_2 | unicycle | proxmpc | 5 | 100% | 18.75±1.08 | 5.08±0.04 | 0.228±0.021 | 0.110±0.080 | 0.238±0.155 | 0.040±0.067 | 40% | 1.200±0.137 | 4.661±0.710 | 10.066±0.977 | 0.0±0.0% | 1.00±0.00 | 7.93±0.27 | 0.0±0.0% | 0.351±0.022 | 19.02±0.21 | 7.38±0.27 | 58.43±0.12 | 1.268±0.168 | 4.279±0.497 |
| blind_multi_2 | unicycle | proxmpc_pred | 5 | 100% | 18.95±1.50 | 4.91±0.24 | 0.226±0.015 | 0.116±0.032 | 0.252±0.052 | 0.160±0.075 | 0% | 1.303±0.102 | 3.805±0.309 | 12.195±1.092 | 0.0±0.0% | 1.00±0.00 | 8.14±0.22 | 0.0±0.0% | 0.247±0.076 | 19.13±0.37 | 7.27±0.28 | 59.84±0.12 | 1.338±0.098 | 3.444±0.392 |
| blind_multi_2 | unicycle | regulated_pure_pursuit | 5 | 100% | 15.89±1.06 | 4.84±0.05 | 0.234±0.004 | 0.099±0.037 | 0.196±0.070 | -0.120±0.068 | 100% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 18.96±0.68 | 4.41±0.13 | 55.26±0.06 | 0.214±0.007 | 0.330±0.025 |
| blind_multi_2 | unicycle | vector_pursuit | 5 | 100% | 19.67±1.31 | 4.83±0.02 | 0.236±0.002 | 0.107±0.030 | 0.215±0.055 | -0.047±0.032 | 100% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 17.42±1.71 | 4.37±0.13 | 55.08±0.10 | 0.214±0.007 | 0.330±0.034 |
| blind_multi_3 | unicycle | dwb | 5 | 100% | 15.72±0.66 | 4.79±0.01 | 0.236±0.002 | 0.054±0.007 | 0.121±0.022 | 0.277±0.016 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.07±0.00 | 9.41±0.30 | 59.57±0.12 | 2.861±0.086 | 3.548±0.239 |
| blind_multi_3 | unicycle | graceful | 5 | 100% | 15.73±1.93 | 4.75±0.28 | 0.216±0.008 | 0.119±0.045 | 0.237±0.081 | 0.025±0.176 | 20% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 19.99±0.16 | 4.23±0.13 | 55.71±0.08 | 0.163±0.017 | 0.249±0.054 |
| blind_multi_3 | unicycle | mppi | 10 | 100% | 17.30±1.44 | 4.81±0.05 | 0.234±0.006 | 0.097±0.033 | 0.187±0.050 | -0.005±0.076 | 60% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.08±0.04 | 8.94±0.29 | 62.97±0.21 | 2.721±0.080 | 3.336±0.172 |
| blind_multi_3 | unicycle | proxmpc | 5 | 80% | 18.20±1.70 | 4.02±2.01 | 1.185±1.908 | 0.106±0.056 | 0.233±0.123 | 0.669±0.832 | 0% | 1.401±0.029 | 4.227±0.197 | 6.829±1.030 | 0.0±0.0% | 0.80±0.40 | 6.41±3.21 | 0.0±0.0% | 0.264±0.132 | 20.05±0.03 | 7.53±0.11 | 58.28±0.09 | 1.537±0.039 | 4.409±0.229 |
| blind_multi_3 | unicycle | proxmpc_pred | 5 | 80% | 16.63±1.74 | 4.07±1.77 | 1.019±1.588 | 0.112±0.080 | 0.243±0.171 | 0.325±0.259 | 0% | 1.071±0.154 | 3.354±0.876 | 8.844±4.880 | 0.0±0.0% | 1.00±0.00 | 7.75±0.37 | 0.0±0.0% | 0.167±0.104 | 19.94±0.17 | 6.76±1.36 | 59.62±0.11 | 0.975±0.499 | 2.606±0.910 |
| blind_multi_3 | unicycle | regulated_pure_pursuit | 5 | 100% | 16.00±1.38 | 4.86±0.02 | 0.234±0.004 | 0.077±0.025 | 0.170±0.035 | 0.219±0.058 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 19.53±0.49 | 4.24±0.09 | 55.32±0.07 | 0.215±0.004 | 0.323±0.023 |
| blind_multi_3 | unicycle | vector_pursuit | 5 | 100% | 20.72±3.95 | 4.81±0.07 | 0.237±0.001 | 0.134±0.036 | 0.243±0.035 | 0.020±0.036 | 20% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 17.31±1.52 | 4.22±0.07 | 55.01±0.10 | 0.210±0.010 | 0.286±0.029 |
| dynamic_circle | unicycle | dwb | 5 | 100% | 15.65±1.07 | 4.78±0.02 | 0.235±0.002 | 0.038±0.029 | 0.080±0.057 | -0.261±0.065 | 100% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.08±0.00 | 8.96±0.12 | 59.75±0.10 | 2.616±0.084 | 3.473±0.123 |
| dynamic_circle | unicycle | graceful | 5 | 100% | 16.90±0.35 | 4.84±0.06 | 0.229±0.011 | 0.048±0.028 | 0.116±0.068 | -0.039±0.043 | 60% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.06±0.02 | 4.34±0.13 | 55.78±0.09 | 0.153±0.003 | 0.223±0.012 |
| dynamic_circle | unicycle | mppi | 10 | 100% | 17.46±1.52 | 5.10±0.03 | 0.233±0.003 | 0.031±0.009 | 0.078±0.012 | 0.172±0.017 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.06±0.00 | 8.95±0.16 | 62.90±0.11 | 2.747±0.092 | 3.262±0.169 |
| dynamic_circle | unicycle | proxmpc | 5 | 100% | 20.53±1.01 | 5.30±0.13 | 0.162±0.051 | 0.134±0.018 | 0.354±0.061 | 0.161±0.020 | 0% | 1.149±0.080 | 5.016±0.396 | 11.707±1.266 | 0.0±0.0% | 1.00±0.00 | 7.82±0.14 | 0.0±0.0% | 1.212±1.412 | 18.75±0.62 | 7.25±0.17 | 58.50±0.08 | 1.191±0.089 | 4.238±0.502 |
| dynamic_circle | unicycle | proxmpc_pred | 5 | 100% | 16.62±0.66 | 5.22±0.10 | 0.224±0.011 | 0.206±0.049 | 0.397±0.080 | 0.141±0.056 | 0% | 1.050±0.067 | 3.461±0.295 | 8.026±1.202 | 0.0±0.0% | 1.00±0.00 | 7.79±0.27 | 0.0±0.0% | 0.168±0.033 | 19.60±0.42 | 7.16±0.45 | 59.73±0.18 | 1.170±0.181 | 3.621±0.417 |
| dynamic_circle | unicycle | regulated_pure_pursuit | 5 | 100% | 18.56±1.49 | 4.90±0.01 | 0.234±0.005 | 0.136±0.026 | 0.261±0.042 | 0.097±0.039 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 18.08±1.96 | 4.33±0.09 | 55.29±0.04 | 0.208±0.005 | 0.302±0.042 |
| dynamic_circle | unicycle | vector_pursuit | 5 | 100% | 19.43±1.37 | 4.81±0.03 | 0.236±0.002 | 0.070±0.031 | 0.151±0.059 | 0.114±0.044 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 18.49±0.39 | 4.28±0.12 | 55.05±0.06 | 0.208±0.008 | 0.297±0.039 |
| dynamic_line_backward | unicycle | dwb | 5 | 100% | 13.11±0.96 | 4.77±0.00 | 0.237±0.002 | 0.037±0.013 | 0.067±0.021 | 0.488±0.013 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.09±0.01 | 8.88±0.12 | 59.61±0.07 | 2.685±0.098 | 3.417±0.212 |
| dynamic_line_backward | unicycle | graceful | 5 | 100% | 12.98±1.06 | 4.81±0.04 | 0.221±0.017 | 0.051±0.034 | 0.100±0.058 | 0.338±0.062 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.09±0.00 | 4.16±0.13 | 55.74±0.03 | 0.150±0.002 | 0.242±0.019 |
| dynamic_line_backward | unicycle | mppi | 10 | 100% | 13.75±1.22 | 4.78±0.01 | 0.234±0.005 | 0.034±0.018 | 0.061±0.025 | 0.611±0.025 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.04±0.14 | 8.86±0.28 | 62.92±0.11 | 2.774±0.142 | 3.431±0.270 |
| dynamic_line_backward | unicycle | proxmpc | 5 | 100% | 14.92±1.23 | 4.86±0.03 | 0.224±0.025 | 0.096±0.035 | 0.177±0.049 | 0.551±0.028 | 0% | 0.848±0.069 | 3.490±0.733 | 6.902±0.621 | 0.0±0.0% | 1.00±0.00 | 7.34±0.19 | 0.0±0.0% | 0.275±0.038 | 20.07±0.03 | 6.46±0.44 | 58.26±0.10 | 0.974±0.067 | 3.747±0.757 |
| dynamic_line_backward | unicycle | proxmpc_pred | 5 | 100% | 15.22±0.39 | 4.83±0.04 | 0.238±0.001 | 0.048±0.009 | 0.088±0.018 | 0.380±0.014 | 0% | 0.771±0.053 | 3.162±0.572 | 9.516±2.058 | 0.0±0.0% | 1.00±0.00 | 6.79±0.22 | 0.0±0.0% | 0.279±0.055 | 20.08±0.00 | 6.45±0.23 | 59.60±0.11 | 0.869±0.061 | 3.309±0.541 |
| dynamic_line_backward | unicycle | regulated_pure_pursuit | 5 | 100% | 12.95±0.86 | 4.81±0.02 | 0.235±0.002 | 0.078±0.017 | 0.131±0.028 | 0.412±0.009 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.05±0.08 | 4.48±0.15 | 55.19±0.12 | 0.211±0.003 | 0.317±0.031 |
| dynamic_line_backward | unicycle | vector_pursuit | 5 | 100% | 14.05±0.58 | 4.78±0.01 | 0.237±0.001 | 0.045±0.022 | 0.075±0.034 | 0.451±0.008 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 19.45±0.53 | 4.12±0.16 | 55.06±0.03 | 0.211±0.006 | 0.362±0.045 |
| dynamic_line_forward | unicycle | dwb | 5 | 100% | 13.29±1.44 | 4.70±0.14 | 0.235±0.003 | 0.053±0.009 | 0.095±0.013 | 0.450±0.077 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.12±0.06 | 8.83±0.11 | 59.64±0.05 | 2.655±0.100 | 3.073±0.106 |
| dynamic_line_forward | unicycle | graceful | 5 | 100% | 13.37±1.04 | 4.79±0.01 | 0.229±0.006 | 0.030±0.014 | 0.067±0.015 | 0.368±0.081 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.09±0.01 | 4.32±0.18 | 55.70±0.12 | 0.152±0.006 | 0.236±0.023 |
| dynamic_line_forward | unicycle | mppi | 10 | 100% | 14.24±1.48 | 4.77±0.05 | 0.234±0.004 | 0.041±0.018 | 0.072±0.026 | 0.565±0.070 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.08±0.00 | 8.79±0.23 | 62.90±0.15 | 2.712±0.092 | 3.110±0.156 |
| dynamic_line_forward | unicycle | proxmpc | 5 | 100% | 14.72±1.52 | 4.85±0.06 | 0.219±0.028 | 0.106±0.035 | 0.185±0.063 | 0.541±0.030 | 0% | 0.803±0.044 | 4.097±0.961 | 7.862±2.570 | 0.0±0.0% | 1.00±0.00 | 7.19±0.21 | 0.0±0.0% | 0.289±0.015 | 20.07±0.03 | 6.50±0.37 | 58.22±0.11 | 0.920±0.048 | 4.334±0.978 |
| dynamic_line_forward | unicycle | proxmpc_pred | 5 | 100% | 14.91±1.04 | 4.84±0.04 | 0.226±0.027 | 0.057±0.038 | 0.108±0.053 | 0.379±0.018 | 0% | 0.678±0.018 | 4.297±1.770 | 11.895±2.592 | 0.0±0.0% | 1.00±0.00 | 6.75±0.15 | 0.0±0.0% | 0.274±0.050 | 20.06±0.03 | 6.55±0.42 | 59.60±0.19 | 0.778±0.019 | 4.383±1.730 |
| dynamic_line_forward | unicycle | regulated_pure_pursuit | 5 | 100% | 13.35±0.48 | 4.80±0.02 | 0.235±0.003 | 0.063±0.026 | 0.115±0.035 | 0.406±0.017 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.07±0.10 | 4.31±0.19 | 55.26±0.11 | 0.211±0.004 | 0.263±0.012 |
| dynamic_line_forward | unicycle | vector_pursuit | 5 | 100% | 14.10±1.35 | 4.79±0.01 | 0.237±0.001 | 0.073±0.026 | 0.127±0.040 | 0.458±0.007 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 19.48±0.17 | 4.23±0.09 | 55.05±0.09 | 0.217±0.004 | 0.328±0.047 |
| dynamic_multi | unicycle | dwb | 5 | 100% | 16.10±1.16 | 4.79±0.01 | 0.237±0.002 | 0.040±0.008 | 0.072±0.013 | 0.027±0.121 | 20% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.08±0.02 | 9.56±0.18 | 59.69±0.12 | 2.826±0.088 | 3.527±0.076 |
| dynamic_multi | unicycle | graceful | 5 | 100% | 16.79±1.09 | 4.85±0.04 | 0.225±0.009 | 0.093±0.042 | 0.185±0.060 | -0.038±0.008 | 100% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.07±0.00 | 4.57±0.06 | 55.83±0.10 | 0.178±0.026 | 0.267±0.060 |
| dynamic_multi | unicycle | mppi | 10 | 90% | 17.16±1.21 | 4.52±1.51 | 0.708±1.431 | 0.050±0.027 | 0.107±0.060 | 0.342±0.625 | 10% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.06±0.00 | 9.07±0.32 | 63.18±0.57 | 2.751±0.066 | 3.534±0.250 |
| dynamic_multi | unicycle | proxmpc | 5 | 80% | 21.12±4.24 | 4.16±2.09 | 1.191±1.905 | 0.118±0.114 | 0.224±0.207 | 0.415±0.925 | 40% | 1.457±0.162 | 5.016±0.930 | 9.783±0.573 | 0.0±0.0% | 0.80±0.40 | 6.66±3.34 | 0.0±0.0% | 0.265±0.133 | 18.81±1.88 | 7.48±0.20 | 58.44±0.14 | 1.508±0.172 | 4.773±0.448 |
| dynamic_multi | unicycle | proxmpc_pred | 5 | 80% | 18.31±2.02 | 4.14±2.09 | 1.172±1.914 | 0.121±0.106 | 0.251±0.200 | 0.561±0.824 | 20% | 1.300±0.082 | 4.294±0.567 | 9.961±2.129 | 0.0±0.0% | 0.80±0.40 | 6.45±3.23 | 0.0±0.0% | 0.180±0.114 | 19.82±0.27 | 7.71±0.18 | 59.74±0.14 | 1.377±0.088 | 4.046±0.682 |
| dynamic_multi | unicycle | regulated_pure_pursuit | 5 | 100% | 16.20±1.25 | 4.87±0.02 | 0.237±0.002 | 0.085±0.008 | 0.208±0.028 | 0.150±0.034 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 19.74±0.15 | 4.62±0.12 | 55.32±0.05 | 0.220±0.004 | 0.340±0.028 |
| dynamic_multi | unicycle | vector_pursuit | 5 | 100% | 22.83±3.26 | 4.83±0.02 | 0.238±0.001 | 0.085±0.040 | 0.167±0.059 | 0.147±0.102 | 20% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 17.62±1.67 | 4.26±0.19 | 55.04±0.05 | 0.213±0.006 | 0.352±0.033 |
| dynamic_multi_noise | unicycle | dwb | 5 | 100% | 15.81±1.66 | 4.79±0.01 | 0.237±0.002 | 0.056±0.025 | 0.106±0.045 | -0.010±0.165 | 20% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.08±0.01 | 9.40±0.15 | 59.74±0.06 | 2.820±0.082 | 3.535±0.075 |
| dynamic_multi_noise | unicycle | graceful | 5 | 100% | 17.17±1.37 | 4.85±0.03 | 0.222±0.009 | 0.074±0.027 | 0.158±0.054 | -0.012±0.025 | 60% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 19.70±0.45 | 4.42±0.13 | 55.87±0.09 | 0.159±0.012 | 0.229±0.030 |
| dynamic_multi_noise | unicycle | mppi | 10 | 100% | 17.77±1.46 | 5.10±0.11 | 0.231±0.006 | 0.062±0.028 | 0.125±0.045 | 0.080±0.085 | 10% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.06±0.00 | 9.13±0.24 | 63.10±0.38 | 2.769±0.098 | 3.385±0.163 |
| dynamic_multi_noise | unicycle | proxmpc | 5 | 100% | 23.25±8.99 | 5.34±0.49 | 0.201±0.047 | 0.143±0.090 | 0.309±0.156 | 0.126±0.220 | 20% | 1.497±0.219 | 5.435±0.485 | 10.800±0.691 | 0.0±0.0% | 1.00±0.00 | 8.21±0.41 | 0.0±0.0% | 0.344±0.023 | 18.70±2.49 | 7.87±0.32 | 58.37±0.09 | 1.547±0.181 | 5.236±0.560 |
| dynamic_multi_noise | unicycle | proxmpc_pred | 5 | 80% | 18.32±1.25 | 4.97±0.10 | 0.527±0.622 | 0.119±0.076 | 0.252±0.152 | 0.121±0.212 | 20% | 1.455±0.430 | 4.401±1.161 | 12.769±2.989 | 0.0±0.0% | 1.00±0.00 | 8.17±0.49 | 0.0±0.0% | 0.435±0.368 | 17.81±2.84 | 7.08±0.81 | 59.78±0.11 | 1.553±0.465 | 4.276±1.656 |
| dynamic_multi_noise | unicycle | regulated_pure_pursuit | 5 | 100% | 16.66±1.03 | 4.87±0.01 | 0.235±0.003 | 0.086±0.010 | 0.214±0.028 | 0.170±0.019 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 19.73±0.09 | 4.46±0.08 | 55.14±0.04 | 0.212±0.004 | 0.316±0.039 |
| dynamic_multi_noise | unicycle | vector_pursuit | 5 | 100% | 19.88±1.11 | 4.83±0.02 | 0.237±0.002 | 0.085±0.028 | 0.182±0.046 | 0.154±0.030 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 18.55±0.63 | 4.30±0.16 | 55.11±0.07 | 0.213±0.005 | 0.301±0.041 |
| nav2_open | unicycle | dwb | 5 | 100% | 12.68±1.13 | 4.76±0.01 | 0.237±0.000 | 0.000±0.000 | 0.000±0.000 | n/a | n/a | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.09±0.00 | 8.61±0.20 | 59.65±0.10 | 2.540±0.035 | 2.913±0.242 |
| nav2_open | unicycle | graceful | 5 | 100% | 10.48±0.98 | 4.76±0.02 | 0.224±0.005 | 0.000±0.000 | 0.000±0.000 | n/a | n/a | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.11±0.00 | 4.23±0.12 | 55.75±0.04 | 0.148±0.004 | 0.208±0.025 |
| nav2_open | unicycle | mppi | 5 | 100% | 11.84±1.70 | 4.65±0.23 | 0.233±0.007 | 0.003±0.000 | 0.004±0.000 | n/a | n/a | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.17±0.09 | 8.58±0.19 | 63.24±0.34 | 2.736±0.062 | 3.159±0.073 |
| nav2_open | unicycle | proxmpc | 5 | 100% | 12.45±0.50 | 4.76±0.00 | 0.239±0.000 | 0.000±0.000 | 0.001±0.000 | n/a | n/a | 0.523±0.016 | 1.039±0.040 | 2.396±0.081 | 0.0±0.0% | 1.00±0.00 | 6.27±0.03 | 0.0±0.0% | 0.140±0.000 | 20.15±0.11 | 5.05±0.13 | 58.36±0.16 | 0.652±0.019 | 1.113±0.023 |
| nav2_open | unicycle | proxmpc_pred | 5 | 100% | 13.65±0.88 | 4.76±0.00 | 0.240±0.000 | 0.000±0.000 | 0.000±0.000 | n/a | n/a | 0.454±0.015 | 0.998±0.052 | 1.804±0.078 | 0.0±0.0% | 1.00±0.00 | 5.86±0.04 | 0.0±0.0% | 0.040±0.000 | 20.09±0.00 | 5.31±0.20 | 59.62±0.06 | 0.563±0.020 | 1.090±0.054 |
| nav2_open | unicycle | regulated_pure_pursuit | 5 | 100% | 11.76±1.33 | 4.76±0.00 | 0.235±0.000 | 0.000±0.000 | 0.000±0.000 | n/a | n/a | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.01±0.20 | 4.38±0.12 | 55.12±0.11 | 0.207±0.005 | 0.254±0.025 |
| nav2_open | unicycle | vector_pursuit | 5 | 100% | 13.03±1.05 | 4.76±0.00 | 0.237±0.001 | 0.000±0.000 | 0.000±0.000 | n/a | n/a | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.10±0.01 | 4.23±0.15 | 54.95±0.06 | 0.210±0.007 | 0.253±0.015 |
| static_box | unicycle | dwb | 5 | 100% | 12.69±0.79 | 4.87±0.01 | 0.237±0.002 | 0.259±0.006 | 0.450±0.004 | 0.093±0.004 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.09±0.01 | 8.99±0.08 | 59.63±0.08 | 2.686±0.071 | 3.134±0.189 |
| static_box | unicycle | graceful | 5 | 100% | 13.02±0.78 | 4.96±0.01 | 0.207±0.008 | 0.320±0.011 | 0.565±0.003 | 0.208±0.005 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.09±0.00 | 4.35±0.12 | 55.69±0.08 | 0.161±0.012 | 0.215±0.036 |
| static_box | unicycle | mppi | 10 | 100% | 12.63±1.09 | 4.94±0.00 | 0.233±0.006 | 0.315±0.015 | 0.572±0.003 | 0.206±0.002 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.09±0.00 | 8.64±0.21 | 62.87±0.11 | 2.722±0.070 | 3.080±0.118 |
| static_box | unicycle | proxmpc | 5 | 100% | 13.34±1.96 | 5.03±0.16 | 0.212±0.024 | 0.354±0.034 | 0.702±0.005 | 0.346±0.009 | 0% | 0.950±0.112 | 1.717±0.216 | 3.776±0.728 | 0.0±0.0% | 1.00±0.00 | 7.24±0.17 | 0.0±0.0% | 0.121±0.037 | 20.08±0.04 | 6.14±0.23 | 58.31±0.07 | 1.052±0.096 | 1.867±0.199 |
| static_box | unicycle | proxmpc_pred | 5 | 100% | 14.28±1.30 | 5.03±0.03 | 0.221±0.014 | 0.325±0.027 | 0.650±0.002 | 0.299±0.002 | 0% | 0.979±0.093 | 2.047±0.185 | 4.505±0.989 | 0.0±0.0% | 1.00±0.00 | 7.37±0.39 | 0.0±0.0% | 0.040±0.000 | 20.07±0.03 | 6.43±0.33 | 59.50±0.04 | 1.079±0.089 | 2.153±0.191 |
| static_box | unicycle | regulated_pure_pursuit | 5 | 100% | 11.62±1.06 | 4.94±0.00 | 0.231±0.000 | 0.334±0.013 | 0.567±0.003 | 0.213±0.003 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.10±0.01 | 4.28±0.13 | 55.10±0.03 | 0.210±0.003 | 0.272±0.044 |
| static_box | unicycle | vector_pursuit | 5 | 0% | n/a | 2.31±0.15 | 2.779±0.146 | 0.417±0.029 | 0.433±0.028 | 0.165±0.075 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 18.36±0.04 | 4.46±0.08 | 55.16±0.17 | 0.253±0.019 | 0.394±0.054 |

<!-- BENCHMARK_RESULTS_END -->

## License

[Apache-2.0](../LICENSE) — the full text is in [LICENSE](../LICENSE) and
attribution in [NOTICE](../NOTICE).
