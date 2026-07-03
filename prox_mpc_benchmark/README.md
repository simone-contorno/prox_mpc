# prox_mpc_benchmark

Scenario-driven benchmarking harness for the ProxMPC stack (SIM_SPEC Phase 2).
It measures three metric classes — **accuracy** (cross-track / goal error),
**precision** (mean ± std over repeats), and **real-time / feasibility** (solver
diagnostics) — across a matrix of *scenario × model × controller × run mode*.

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Build](#build)
- [Project structure](#project-structure)
- [Run modes](#run-modes)
- [Coverage and status (this run)](#coverage-and-status-this-run)
- [Notes on the metrics](#notes-on-the-metrics)
- [Configuration](#configuration)
- [Usage](#usage)
  - [Quick start](#quick-start)
  - [Standalone matrix (b1)](#standalone-matrix-b1)
  - [Cross-controller comparison (b2)](#cross-controller-comparison-b2)
  - [Aggregate the tables](#aggregate-the-tables)
- [Results](#results)
- [License](#license)

## Overview

The package contains a controller-agnostic **C++ live metrics node**
([src/metrics_node.cpp](src/metrics_node.cpp)) plus installed **Python tooling**
([scripts/](scripts/)) for orchestration, map generation, goal sending, bag
reduction, and aggregation (D4). It **reuses** the demo worlds/maps/models rather
than duplicating them, and **owns the result artifacts**, which stay local and
gitignored (D7) — the framework performs no git operations.

The narrative companion — how ProxMPC compares against the stock Nav2 controllers
and what the suite concluded — is in
[docs/controller-comparison-results.md](../docs/controller-comparison-results.md).

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
  plus `nav2_open` for the cross-controller cell; one YAML each (section 2, D10).
- `config/controllers/` — one preset per Nav2 controller under test: `proxmpc`,
  `dwb`, `mppi`, `regulated_pure_pursuit` (D2).
- `config/robots/` — robot ↔ prox_mpc model pairing (`waffle`→Unicycle,
  `ackermann`→Bicycle, D3).
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
- `results/` — per-run JSON, the `scenarios.json` index, progress files (gitignored, D7).

## Run modes

- **(b1) standalone core sim** — the deterministic, ProxMPC-only regression path
  (the core drives its own model; no Gazebo). Both models. Fully wired and run here.
- **(a) Gazebo + Nav2** — full stack, ground-truth accuracy, real-time/feasibility
  telemetry from the controller plugin's `<plugin>/diagnostics`.
- **(b2) Nav2 without Gazebo** — a kinematic-plant node (`kinematic_plant`) +
  Nav2 controller_server for controller-agnostic comparison without Gazebo physics
  cost. Wired (`benchmark_nav2.launch.py`, `run_nav2.py`) and run here.

## Coverage and status (this run)

What was actually built and executed in this environment (ROS 2 Jazzy, Gazebo
Harmonic 8.11, full Nav2 with the ProxMPC / DWB / MPPI / RPP controllers compared):

| Capability | Status |
| --- | --- |
| Phase-1 telemetry (`SolverDiagnostics`, core slack getter, standalone + plugin publishers) | **Built + verified** (live data; plugin tap confirmed under Nav2) |
| Mode **b1** — both models × all 4 scenarios × 5 repeats | **Run: 40/40 success** (table below) |
| Mode **a** — waffle / Unicycle + ProxMPC, open world, goal → `SUCCEEDED` | **Run: real end-to-end cells** (table below) |
| Mode **b2** — Nav2 + kinematic plant + scan simulator, no Gazebo (`run_nav2.py`) | **Run: 5 controllers × 5 scenarios × 5 repeats** (open cell + obstacle avoidance; tables below) |
| Cross-controller matrix (D2): ProxMPC / DWB / MPPI / RPP / Graceful, open cell + obstacle scenarios | **Run: shared-costmap obstacle avoidance** ([comparison](../docs/controller-comparison-results.md)) |
| Map scaling (`generate_map.py` 7/15/30 m + scalable world xacro) | **Built + verified** (maps match world geometry) |
| Ackermann robot SDF + gz AckermannSteering bridge (mode-a bicycle, D3) | **Built + `gz sdf --check` valid**; full mode-a bicycle bringup not executed |
| Mode **a** obstacle scenarios + mode-a Ackermann/bike full bringup | **Scaffolded** (launch, goal_sender, compute_metrics) **— not executed this run** |

The cross-controller comparison (D2) runs in mode **b2** via `run_nav2.py`: each
Nav2 controller (ProxMPC, DWB, MPPI, RPP, Graceful) has a preset in
`config/controllers/`, `benchmark_nav2.launch.py` injects it into `FollowPath`, and
every controller drives the *identical* kinematic plant from the same start to the
same goal. On the obstacle scenarios the `scan_simulator` ray-casts the scenario
obstacles into `/scan` and a costmap `obstacle_layer` marks them, so every
controller perceives the *same* obstacle through the *same* costmap; the
controller-agnostic metrics node adds a min-clearance / collision metric. The
narrative and the conclusion on ProxMPC are in
[docs/controller-comparison-results.md](../docs/controller-comparison-results.md).

## Notes on the metrics

<details>
<summary>How the metrics are defined and why (precision, deadline-miss, obstacle placement, resources, fair tuning)</summary>

- **Precision (D9).** Mode b1 is deterministic, so the geometric metrics (path,
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
  cleaner controller-only measure. Measured on the x86 dev host, **not a Jetson** —
  the ranking transfers, absolutes are indicative.
- **Fair tuning (cross-controller).** The predictive controllers share a **2.0 s
  prediction horizon** (ProxMPC `np·dt`, DWB `sim_time`, MPPI `time_steps·model_dt`),
  the same 0.5 m/s speed cap, 20 Hz rate, and 0.25 m goal tolerance. Each method's
  intrinsic sampling (MPPI `batch_size`, DWB sample grid) stays at its upstream
  default — those are not cross-equalisable. See
  [docs/controller-comparison-results.md](../docs/controller-comparison-results.md).

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
./init.sh static_box bike
```

Its usage is `./init.sh [scenario] [model]`, where `scenario` is one of
`static_box`, `dynamic_circle`, `dynamic_line_forward`, `dynamic_line_backward`
(default `static_box`) and `model` is `r2d2` or `bike` (default `bike`).
Override the workspace path with the `ROS2_WS` environment variable if the
workspace is not at `~/ros2_ws`.

### Standalone matrix (b1)

Run the whole deterministic standalone matrix (both models × the four motion
scenarios × repeats):

```bash
ros2 run prox_mpc_benchmark run_matrix.py --modes b1
```

Narrow the run to specific cells with `--scenarios` and `--models`, for example
`--scenarios static_box --models bike`.

### Cross-controller comparison (b2)

Compare ProxMPC against the stock Nav2 controllers on the open-world cell, Nav2 +
kinematic plant, no Gazebo:

```bash
ros2 run prox_mpc_benchmark run_nav2.py --controllers proxmpc,dwb,mppi,regulated_pure_pursuit
```

### Aggregate the tables

Reduce the per-run JSONs into the mean ± std tables and render them into the
[Results](#results) block below:

```bash
ros2 run prox_mpc_benchmark aggregate.py
```

## Results

<!-- BENCHMARK_RESULTS_START -->

_Auto-generated by `aggregate.py` from `results/scenarios.json` (192 runs). Values are mean±std (population) over repeats — the precision signal (D9). Not committed (D7)._

### Mode `a`

| scenario | model | controller | n | success | t_goal[s] | path[m] | goal_err[m] | ct_rms[m] | ct_max[m] | obs_gap[m] | coll | solve_p50[ms] | solve_p95[ms] | solve_max[ms] | miss | sqp | qp_ext | infeas | slack[m] | rate[Hz] | cpu[%] | rss[MB] | cmp_p50[ms] | cmp_p95[ms] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| nav2_open | r2d2 | proxmpc | 2 | 100% | 12.28±0.08 | 3.79±0.01 | 0.224±0.002 | 0.006±0.001 | 0.016±0.005 | n/a | n/a | 0.238±0.012 | 0.438±0.010 | 1.952±0.009 | 0.0±0.0% | 1.00±0.00 | 4.37±0.10 | 0.0±0.0% | 0.000±0.000 | n/a | n/a | n/a | n/a | n/a |

### Mode `b1`

| scenario | model | controller | n | success | t_goal[s] | path[m] | goal_err[m] | ct_rms[m] | ct_max[m] | obs_gap[m] | coll | solve_p50[ms] | solve_p95[ms] | solve_max[ms] | miss | sqp | qp_ext | infeas | slack[m] | rate[Hz] | cpu[%] | rss[MB] | cmp_p50[ms] | cmp_p95[ms] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| dynamic_circle | bike | proxmpc | 5 | 100% | 5.38±0.04 | 4.83±0.00 | 0.170±0.000 | 0.021±0.000 | 0.055±0.000 | n/a | n/a | 1.559±0.022 | 4.312±0.049 | 8.843±2.564 | 0.0±0.0% | 1.00±0.00 | 10.13±0.01 | 0.0±0.0% | 0.121±0.000 | n/a | n/a | n/a | n/a | n/a |
| dynamic_circle | r2d2 | proxmpc | 5 | 100% | 5.36±0.08 | 4.82±0.01 | 0.172±0.000 | 0.022±0.000 | 0.057±0.000 | n/a | n/a | 1.182±0.024 | 2.930±0.080 | 5.272±1.723 | 0.0±0.0% | 1.00±0.00 | 10.08±0.03 | 0.0±0.0% | 0.115±0.000 | n/a | n/a | n/a | n/a | n/a |
| dynamic_line_backward | bike | proxmpc | 5 | 100% | 5.12±0.10 | 4.86±0.01 | 0.159±0.000 | 0.096±0.001 | 0.223±0.000 | n/a | n/a | 2.176±0.271 | 5.194±1.104 | 9.296±0.876 | 0.0±0.0% | 1.00±0.00 | 10.45±0.05 | 0.0±0.0% | 0.343±0.000 | n/a | n/a | n/a | n/a | n/a |
| dynamic_line_backward | r2d2 | proxmpc | 5 | 100% | 5.16±0.08 | 4.85±0.01 | 0.178±0.000 | 0.096±0.001 | 0.224±0.000 | n/a | n/a | 1.641±0.178 | 3.385±0.473 | 4.382±1.101 | 0.0±0.0% | 1.00±0.00 | 10.40±0.04 | 0.0±0.0% | 0.344±0.000 | n/a | n/a | n/a | n/a | n/a |
| dynamic_line_forward | bike | proxmpc | 5 | 100% | 5.08±0.10 | 4.86±0.01 | 0.159±0.000 | 0.096±0.001 | 0.223±0.000 | n/a | n/a | 2.073±0.164 | 4.960±0.408 | 8.343±1.107 | 0.0±0.0% | 1.00±0.00 | 10.47±0.05 | 0.0±0.0% | 0.343±0.000 | n/a | n/a | n/a | n/a | n/a |
| dynamic_line_forward | r2d2 | proxmpc | 5 | 100% | 5.18±0.04 | 4.85±0.00 | 0.178±0.000 | 0.096±0.000 | 0.224±0.000 | n/a | n/a | 1.431±0.063 | 3.292±0.307 | 6.344±2.875 | 0.0±0.0% | 1.00±0.00 | 10.38±0.00 | 0.0±0.0% | 0.344±0.000 | n/a | n/a | n/a | n/a | n/a |
| static_box | bike | proxmpc | 5 | 100% | 4.86±0.08 | 4.79±0.01 | 0.207±0.000 | 0.029±0.000 | 0.061±0.000 | n/a | n/a | 2.399±0.122 | 6.904±0.160 | 9.874±1.497 | 0.0±0.0% | 1.00±0.00 | 10.13±0.04 | 0.0±0.0% | 0.394±0.000 | n/a | n/a | n/a | n/a | n/a |
| static_box | r2d2 | proxmpc | 5 | 100% | 4.82±0.10 | 4.84±0.01 | 0.154±0.000 | 0.028±0.000 | 0.058±0.000 | n/a | n/a | 1.746±0.093 | 3.715±0.293 | 5.188±1.064 | 0.0±0.0% | 1.00±0.00 | 9.64±0.05 | 0.0±0.0% | 0.336±0.000 | n/a | n/a | n/a | n/a | n/a |

### Mode `b2`

| scenario | model | controller | n | success | t_goal[s] | path[m] | goal_err[m] | ct_rms[m] | ct_max[m] | obs_gap[m] | coll | solve_p50[ms] | solve_p95[ms] | solve_max[ms] | miss | sqp | qp_ext | infeas | slack[m] | rate[Hz] | cpu[%] | rss[MB] | cmp_p50[ms] | cmp_p95[ms] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| dynamic_circle | r2d2 | dwb | 5 | 100% | 29.96±1.37 | 4.57±0.37 | 0.246±0.002 | 0.030±0.010 | 0.057±0.020 | -0.029±0.020 | 80% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.03±0.06 | 10.03±0.11 | 59.76±0.04 | 3.218±0.005 | 3.970±0.285 |
| dynamic_circle | r2d2 | graceful | 5 | 80% | 18.26±1.41 | 4.79±0.03 | 0.238±0.014 | 0.034±0.019 | 0.103±0.048 | -0.066±0.028 | 100% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 16.07±2.21 | 4.29±0.07 | 55.81±0.05 | 0.131±0.047 | 0.210±0.040 |
| dynamic_circle | r2d2 | mppi | 5 | 100% | 29.46±4.92 | 5.24±0.08 | 0.241±0.005 | 0.166±0.015 | 0.414±0.016 | -0.020±0.034 | 40% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 19.92±0.13 | 9.03±0.19 | 64.65±0.27 | 2.638±0.060 | 3.346±0.229 |
| dynamic_circle | r2d2 | proxmpc | 5 | 80% | 29.51±0.85 | 5.88±0.58 | 0.511±0.589 | 0.360±0.162 | 0.559±0.171 | -0.325±0.137 | 100% | 1.936±0.200 | 5.755±0.425 | 13.097±4.553 | 0.0±0.0% | 1.00±0.00 | 7.83±0.21 | 0.0±0.0% | 1.035±0.354 | 13.98±1.25 | 6.57±0.89 | 58.36±0.03 | 1.524±0.458 | 6.035±0.748 |
| dynamic_circle | r2d2 | proxmpc_pred | 5 | 40% | 27.91±1.39 | 5.14±0.67 | 1.048±0.517 | 0.277±0.098 | 0.395±0.079 | -0.227±0.140 | 100% | 1.870±0.105 | 4.974±0.363 | 14.727±2.304 | 0.0±0.0% | 1.00±0.00 | 8.19±0.21 | 0.0±0.0% | 0.679±0.130 | 13.89±1.38 | 5.99±0.91 | 59.59±0.08 | 1.919±0.143 | 5.541±0.476 |
| dynamic_circle | r2d2 | regulated_pure_pursuit | 5 | 100% | 19.66±5.63 | 4.80±0.00 | 0.247±0.002 | 0.091±0.011 | 0.156±0.002 | -0.073±0.221 | 40% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 15.65±2.52 | 4.09±0.11 | 55.22±0.11 | 0.212±0.007 | 0.344±0.044 |
| dynamic_line_backward | r2d2 | dwb | 5 | 100% | 23.58±3.27 | 4.76±0.00 | 0.246±0.001 | 0.019±0.000 | 0.039±0.002 | -0.054±0.109 | 40% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.05±0.04 | 9.75±0.24 | 59.77±0.07 | 3.122±0.152 | 4.215±0.404 |
| dynamic_line_backward | r2d2 | graceful | 5 | 0% | n/a | 4.35±0.49 | 0.259±0.004 | 0.000±0.000 | 0.000±0.000 | -0.033±0.257 | 40% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 18.35±0.08 | 4.22±0.10 | 55.73±0.02 | 0.038±0.001 | 0.159±0.004 |
| dynamic_line_backward | r2d2 | mppi | 5 | 100% | 31.99±3.27 | 4.97±0.46 | 0.245±0.003 | 0.043±0.021 | 0.091±0.044 | 0.099±0.079 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 19.86±0.15 | 9.07±0.06 | 64.47±0.46 | 2.667±0.046 | 3.439±0.294 |
| dynamic_line_backward | r2d2 | proxmpc | 5 | 100% | 32.71±5.92 | 5.10±0.22 | 0.247±0.002 | 0.173±0.074 | 0.273±0.136 | -0.213±0.190 | 80% | 1.158±0.108 | 3.803±0.474 | 10.557±3.672 | 0.0±0.0% | 1.00±0.00 | 7.10±0.36 | 0.0±0.0% | 0.520±0.264 | 17.93±1.56 | 6.78±0.31 | 58.39±0.06 | 1.365±0.178 | 4.115±0.474 |
| dynamic_line_backward | r2d2 | proxmpc_pred | 5 | 100% | 13.90±1.70 | 4.64±0.37 | 0.233±0.018 | 0.074±0.039 | 0.142±0.076 | 0.267±0.153 | 20% | 0.655±0.100 | 4.084±0.471 | 13.768±8.830 | 0.0±0.0% | 1.00±0.00 | 5.98±0.41 | 0.0±0.0% | 0.285±0.007 | 19.44±0.32 | 6.53±0.24 | 59.50±0.04 | 0.752±0.095 | 3.688±0.294 |
| dynamic_line_backward | r2d2 | regulated_pure_pursuit | 5 | 100% | 12.47±0.99 | 4.76±0.00 | 0.245±0.002 | 0.000±0.000 | 0.000±0.000 | 0.412±0.010 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 18.76±0.44 | 4.03±0.14 | 55.10±0.12 | 0.207±0.007 | 0.308±0.024 |
| dynamic_line_forward | r2d2 | dwb | 5 | 100% | 21.67±0.93 | 4.76±0.00 | 0.246±0.001 | 0.018±0.004 | 0.035±0.008 | -0.001±0.002 | 40% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.02±0.05 | 9.66±0.09 | 59.77±0.07 | 3.141±0.114 | 4.044±0.317 |
| dynamic_line_forward | r2d2 | graceful | 5 | 40% | 12.02±0.84 | 4.77±0.05 | 0.279±0.055 | 0.000±0.000 | 0.000±0.001 | 0.165±0.001 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 18.68±0.67 | 4.23±0.18 | 55.70±0.04 | 0.084±0.057 | 0.180±0.034 |
| dynamic_line_forward | r2d2 | mppi | 5 | 100% | 28.96±7.31 | 4.90±0.18 | 0.242±0.004 | 0.044±0.030 | 0.087±0.061 | -0.049±0.258 | 60% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 19.79±0.27 | 8.95±0.07 | 64.65±0.61 | 2.634±0.037 | 3.277±0.129 |
| dynamic_line_forward | r2d2 | proxmpc | 5 | 100% | 23.38±5.00 | 4.95±0.14 | 0.242±0.007 | 0.196±0.061 | 0.360±0.135 | 0.054±0.208 | 20% | 0.998±0.269 | 3.667±0.334 | 10.252±3.990 | 0.0±0.0% | 1.00±0.00 | 6.87±0.62 | 0.0±0.0% | 0.371±0.117 | 18.93±1.46 | 6.66±0.31 | 58.35±0.16 | 1.090±0.183 | 4.105±0.427 |
| dynamic_line_forward | r2d2 | proxmpc_pred | 5 | 100% | 13.38±1.04 | 4.77±0.01 | 0.248±0.001 | 0.044±0.012 | 0.069±0.020 | 0.348±0.002 | 0% | 0.636±0.050 | 3.806±0.273 | 15.191±5.303 | 0.0±0.0% | 1.00±0.00 | 5.60±0.21 | 0.0±0.0% | 0.293±0.030 | 19.06±0.57 | 6.25±0.12 | 59.47±0.06 | 0.744±0.061 | 3.715±0.326 |
| dynamic_line_forward | r2d2 | regulated_pure_pursuit | 5 | 100% | 12.18±2.19 | 4.48±0.55 | 0.243±0.004 | 0.000±0.000 | 0.000±0.000 | 0.260±0.356 | 20% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 18.36±0.15 | 4.17±0.05 | 55.14±0.07 | 0.206±0.006 | 0.324±0.034 |
| nav2_open | r2d2 | dwb | 5 | 100% | 12.14±0.88 | 4.54±0.42 | 0.246±0.000 | 0.000±0.000 | 0.001±0.000 | n/a | n/a | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.00±0.18 | 8.41±0.15 | 59.58±0.12 | 2.553±0.052 | 3.438±0.516 |
| nav2_open | r2d2 | graceful | 5 | 0% | n/a | 4.75±0.00 | 0.250±0.000 | 0.000±0.000 | 0.000±0.000 | n/a | n/a | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 18.71±0.51 | 4.12±0.06 | 55.73±0.08 | 0.037±0.001 | 0.154±0.010 |
| nav2_open | r2d2 | mppi | 5 | 100% | 10.91±0.97 | 4.76±0.00 | 0.240±0.005 | 0.003±0.000 | 0.004±0.000 | n/a | n/a | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.12±0.01 | 8.30±0.22 | 62.67±0.11 | 2.682±0.042 | 3.683±0.424 |
| nav2_open | r2d2 | proxmpc | 5 | 100% | 12.69±0.76 | 4.75±0.00 | 0.246±0.000 | 0.000±0.000 | 0.001±0.000 | n/a | n/a | 0.297±0.014 | 0.742±0.082 | 3.442±0.115 | 0.0±0.0% | 1.00±0.00 | 2.80±0.05 | 0.0±0.0% | 0.140±0.000 | 20.16±0.12 | 4.45±0.08 | 57.99±0.14 | 0.397±0.013 | 0.845±0.118 |
| nav2_open | r2d2 | proxmpc_pred | 5 | 100% | 12.34±1.72 | 4.59±0.34 | 0.247±0.002 | 0.000±0.000 | 0.001±0.000 | n/a | n/a | 0.344±0.019 | 0.787±0.062 | 3.674±1.478 | 0.0±0.0% | 1.00±0.00 | 2.86±0.02 | 0.0±0.0% | 0.116±0.047 | 20.13±0.06 | 4.83±0.18 | 59.26±0.07 | 0.461±0.021 | 0.945±0.066 |
| nav2_open | r2d2 | regulated_pure_pursuit | 5 | 100% | 10.31±0.29 | 4.75±0.00 | 0.247±0.000 | 0.000±0.000 | 0.000±0.000 | n/a | n/a | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.11±0.01 | 4.03±0.16 | 54.98±0.08 | 0.214±0.012 | 0.309±0.043 |
| static_box | r2d2 | dwb | 5 | 100% | 12.75±0.20 | 4.75±0.00 | 0.247±0.001 | 0.004±0.001 | 0.006±0.001 | 0.002±0.000 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.09±0.01 | 8.95±0.32 | 59.65±0.09 | 2.560±0.084 | 4.590±0.723 |
| static_box | r2d2 | graceful | 5 | 20% | 25.04±0.00 | 5.33±0.01 | 0.265±0.013 | 0.023±0.002 | 0.089±0.007 | 0.040±0.001 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 16.98±1.54 | 4.21±0.07 | 55.78±0.10 | 0.061±0.047 | 0.221±0.052 |
| static_box | r2d2 | mppi | 5 | 100% | 10.92±1.01 | 4.46±0.40 | 0.239±0.003 | 0.062±0.008 | 0.101±0.014 | 0.091±0.009 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.10±0.01 | 8.73±0.09 | 62.86±0.09 | 2.755±0.055 | 3.809±0.276 |
| static_box | r2d2 | proxmpc | 5 | 100% | 11.82±1.13 | 4.85±0.02 | 0.225±0.018 | 0.156±0.007 | 0.294±0.002 | 0.283±0.002 | 0% | 0.745±0.040 | 2.540±0.264 | 8.371±3.878 | 0.0±0.0% | 1.00±0.00 | 6.36±0.17 | 0.0±0.0% | 0.140±0.000 | 20.10±0.01 | 5.90±0.10 | 58.07±0.10 | 0.901±0.053 | 2.808±0.385 |
| static_box | r2d2 | proxmpc_pred | 5 | 100% | 12.76±0.89 | 4.85±0.03 | 0.226±0.019 | 0.137±0.029 | 0.284±0.016 | 0.276±0.012 | 0% | 0.732±0.054 | 2.653±0.496 | 6.854±4.220 | 0.0±0.0% | 1.00±0.00 | 6.21±0.27 | 0.0±0.0% | 0.140±0.000 | 20.10±0.02 | 6.40±0.16 | 59.29±0.19 | 0.878±0.065 | 2.913±0.677 |
| static_box | r2d2 | regulated_pure_pursuit | 5 | 0% | n/a | 2.51±0.02 | 3.080±0.021 | 0.018±0.001 | 0.110±0.002 | 0.264±0.014 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 11.77±0.44 | 3.87±0.10 | 55.13±0.06 | 0.224±0.017 | 0.334±0.056 |

<!-- BENCHMARK_RESULTS_END -->

## License

[Apache-2.0](../LICENSE) — the full text is in [LICENSE](../LICENSE) and
attribution in [NOTICE](../NOTICE).
