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
  a **genuine matched hard 0.5 m/s cap** (enforced in every controller's
  solver/sampler — for ProxMPC via the model's `v_max` input bound, sourced from
  the robot config; see Stage 6 in `fair-comparison-log.md`), 20 Hz rate, and a
  0.24 m goal checker strictly inside the 0.25 m scored tolerance. Each method's
  intrinsic sampling (MPPI `batch_size`, DWB sample grid) stays at its upstream
  default. Every controller received a good-faith tuning pass on its own
  documented avoidance/completion knobs within ProxMPC's iteration budget;
  `regulated_pure_pursuit`/`graceful` kept genuine wins, `dwb`/`mppi` reverted to
  stock obstacle costs after harder settings only regressed them. MPPI (the one
  unseeded stochastic controller) runs n=10 per obstacle cell; the rest n=5.
- **Fair-comparison result (matched 0.5 m/s cap).**
  At a genuine matched speed cap, on the **single-obstacle** dynamic cells
  (circle, line fwd/back) + static box, **`proxmpc_pred` is the safest controller
  by a wide margin — 0 collisions where reactive controllers collide 20-100%.**
  Across those cells it totals 3 collisions vs 11 (best reactive,
  `regulated_pure_pursuit`), 15/18/22 (graceful/reactive-proxmpc/dwb), 33/60
  (mppi). **Multi-obstacle is a genuine LIMITATION, not a strength:** the
  hand-picked `dynamic_multi` cell (pred 2/5) was author-selected; a blind,
  un-screened generator puts pred at 16/20 on two-mover geometry (tied with
  graceful). A controller-independent space-time check
  (`space_time_feasible.py`) confirms those blind cells are provably solvable,
  and feeding pred perfect obstacle predictions does not help (it stalls) — so
  the failure is the controller formulation, not the cells or perception
  (Stage 7/7b in `fair-comparison-log.md`). **The avoidance advantage is
  single-obstacle only.** The cap
  removed the earlier speed confound: `proxmpc_pred` is **no longer the fastest**
  (it now yields and routes around, so it is mid-pack on `t_goal`; the geometric
  controllers are faster but collide 4-5x more) — the remaining advantage is
  collision avoidance at matched speed. The reactive ProxMPC ablation (same
  solver, same cap, prediction off) collides 18/30 and stalls on both held-out
  cells, confirming the advantage is **prediction**, not the solver, speed, or
  tuning. CPU: `proxmpc_pred` 7.3% + 1.0% tracker vs `dwb` 9.7% / `mppi` 9.1%
  (the rollout controllers that attempt avoidance) — cheaper than both; the
  geometric controllers are ~4.2% but pay with 37-50% obstacle collision rates.
  Full audit trail: `fair-comparison-log.md` + `stage5-fairness-review.md`.

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

_Auto-generated by `aggregate.py` from `results/scenarios.json` (282 runs). Values are mean±std (population) over repeats — the precision signal (D9). Not committed (D7)._

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
| dynamic_circle | r2d2 | dwb | 5 | 100% | 32.00±1.25 | 4.77±0.00 | 0.239±0.002 | 0.034±0.003 | 0.066±0.011 | -0.031±0.003 | 100% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.02±0.03 | 10.05±0.10 | 61.03±0.14 | 3.234±0.045 | 3.583±0.131 |
| dynamic_circle | r2d2 | graceful | 5 | 100% | 19.70±1.56 | 4.77±0.00 | 0.232±0.005 | 0.002±0.001 | 0.009±0.006 | -0.197±0.097 | 100% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 13.57±0.21 | 4.26±0.14 | 56.96±0.06 | 0.151±0.003 | 0.208±0.017 |
| dynamic_circle | r2d2 | mppi | 10 | 90% | 34.25±13.49 | 5.29±0.42 | 0.319±0.316 | 0.166±0.030 | 0.409±0.031 | -0.028±0.032 | 80% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 19.67±0.66 | 9.30±0.14 | 65.62±0.56 | 2.693±0.080 | 3.345±0.246 |
| dynamic_circle | r2d2 | proxmpc | 5 | 100% | 26.40±5.03 | 5.08±0.15 | 0.225±0.025 | 0.108±0.034 | 0.206±0.023 | -0.003±0.031 | 60% | 1.620±0.139 | 4.974±0.512 | 11.174±1.943 | 0.0±0.0% | 1.00±0.00 | 8.30±0.09 | 0.0±0.0% | 0.571±0.191 | 16.49±1.45 | 7.31±0.51 | 59.61±0.15 | 1.671±0.099 | 5.267±0.469 |
| dynamic_circle | r2d2 | proxmpc_pred | 5 | 100% | 19.53±0.45 | 5.35±0.26 | 0.179±0.076 | 0.260±0.073 | 0.552±0.143 | 0.254±0.074 | 0% | 1.583±0.087 | 4.761±0.830 | 11.842±2.075 | 0.0±0.0% | 1.00±0.00 | 8.68±0.21 | 0.0±0.0% | 0.336±0.046 | 19.50±0.28 | 8.08±0.49 | 60.95±0.09 | 1.641±0.098 | 4.688±0.914 |
| dynamic_circle | r2d2 | regulated_pure_pursuit | 5 | 100% | 20.73±3.04 | 4.80±0.00 | 0.237±0.003 | 0.058±0.008 | 0.109±0.002 | -0.051±0.215 | 20% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 17.33±1.62 | 4.28±0.09 | 56.39±0.09 | 0.209±0.003 | 0.320±0.014 |
| dynamic_line_backward | r2d2 | dwb | 5 | 100% | 35.17±11.37 | 4.77±0.00 | 0.236±0.002 | 0.017±0.004 | 0.037±0.011 | -0.149±0.167 | 60% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 19.90±0.20 | 9.90±0.24 | 61.04±0.12 | 3.084±0.203 | 3.583±0.079 |
| dynamic_line_backward | r2d2 | graceful | 5 | 100% | 11.90±0.99 | 4.78±0.01 | 0.224±0.007 | 0.000±0.000 | 0.000±0.000 | 0.139±0.003 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.01±0.08 | 4.30±0.11 | 56.92±0.12 | 0.155±0.008 | 0.239±0.045 |
| dynamic_line_backward | r2d2 | mppi | 10 | 100% | 35.53±1.99 | 5.09±0.30 | 0.232±0.005 | 0.040±0.015 | 0.092±0.041 | 0.071±0.051 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 19.69±0.15 | 9.07±0.19 | 65.79±0.44 | 2.611±0.065 | 3.130±0.195 |
| dynamic_line_backward | r2d2 | proxmpc | 5 | 80% | 31.02±1.97 | 4.52±0.79 | 0.748±1.053 | 0.129±0.006 | 0.225±0.045 | -0.239±0.117 | 100% | 1.121±0.252 | 3.436±0.507 | 9.067±4.410 | 0.0±0.0% | 1.00±0.00 | 7.39±0.59 | 0.0±0.0% | 0.427±0.202 | 17.12±1.54 | 6.47±0.22 | 59.60±0.10 | 1.230±0.312 | 3.620±0.518 |
| dynamic_line_backward | r2d2 | proxmpc_pred | 5 | 100% | 15.66±1.02 | 4.90±0.07 | 0.232±0.012 | 0.074±0.019 | 0.145±0.037 | 0.245±0.033 | 0% | 0.997±0.111 | 3.691±0.240 | 9.063±0.648 | 0.0±0.0% | 1.00±0.00 | 7.89±0.38 | 0.0±0.0% | 0.249±0.015 | 19.89±0.52 | 7.10±0.35 | 60.98±0.13 | 1.068±0.107 | 3.587±0.323 |
| dynamic_line_backward | r2d2 | regulated_pure_pursuit | 5 | 80% | 12.41±1.74 | 3.96±1.22 | 0.865±1.261 | 0.000±0.000 | 0.000±0.000 | 0.332±0.125 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 19.34±0.59 | 4.24±0.30 | 56.29±0.09 | 0.212±0.004 | 0.274±0.042 |
| dynamic_line_forward | r2d2 | dwb | 5 | 100% | 33.15±12.78 | 4.77±0.00 | 0.236±0.002 | 0.018±0.002 | 0.040±0.002 | -0.196±0.202 | 80% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 19.59±0.84 | 9.82±0.24 | 61.01±0.13 | 3.187±0.179 | 3.593±0.110 |
| dynamic_line_forward | r2d2 | graceful | 5 | 100% | 11.35±0.64 | 4.77±0.01 | 0.226±0.005 | 0.000±0.000 | 0.000±0.000 | 0.141±0.000 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.14±0.34 | 4.38±0.13 | 56.99±0.15 | 0.158±0.007 | 0.245±0.028 |
| dynamic_line_forward | r2d2 | mppi | 10 | 100% | 36.51±4.47 | 5.20±0.26 | 0.234±0.004 | 0.039±0.018 | 0.086±0.039 | -0.054±0.220 | 50% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 19.82±0.14 | 9.09±0.15 | 66.03±0.39 | 2.594±0.040 | 3.250±0.324 |
| dynamic_line_forward | r2d2 | proxmpc | 5 | 100% | 23.20±2.66 | 5.05±0.19 | 0.224±0.012 | 0.228±0.061 | 0.468±0.180 | 0.132±0.024 | 0% | 1.281±0.095 | 4.253±0.293 | 8.557±1.266 | 0.0±0.0% | 1.00±0.00 | 7.89±0.21 | 0.0±0.0% | 0.317±0.006 | 19.19±0.14 | 7.56±0.12 | 59.69±0.09 | 1.389±0.136 | 4.509±0.277 |
| dynamic_line_forward | r2d2 | proxmpc_pred | 5 | 100% | 23.20±11.98 | 5.34±0.71 | 0.238±0.002 | 0.067±0.043 | 0.144±0.084 | 0.199±0.101 | 0% | 1.210±0.415 | 4.287±0.578 | 15.037±6.708 | 0.0±0.0% | 1.00±0.00 | 8.13±0.88 | 0.0±0.0% | 0.494±0.408 | 18.74±0.94 | 7.39±0.74 | 60.97±0.11 | 1.305±0.458 | 3.865±1.053 |
| dynamic_line_forward | r2d2 | regulated_pure_pursuit | 5 | 100% | 13.14±0.56 | 4.77±0.00 | 0.234±0.004 | 0.000±0.000 | 0.000±0.000 | 0.397±0.013 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 19.23±0.64 | 4.37±0.19 | 56.31±0.09 | 0.208±0.005 | 0.308±0.027 |
| dynamic_multi | r2d2 | dwb | 5 | 100% | 31.54±1.57 | 4.76±0.02 | 0.236±0.002 | 0.054±0.009 | 0.079±0.012 | -0.241±0.016 | 100% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.01±0.02 | 10.29±0.16 | 61.13±0.10 | 3.203±0.032 | 3.952±0.306 |
| dynamic_multi | r2d2 | graceful | 5 | 100% | 18.04±2.23 | 4.76±0.02 | 0.234±0.006 | 0.001±0.001 | 0.006±0.005 | -0.166±0.114 | 100% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 14.99±2.41 | 4.16±0.14 | 57.02±0.12 | 0.152±0.007 | 0.240±0.040 |
| dynamic_multi | r2d2 | mppi | 10 | 100% | 48.80±13.55 | 5.72±0.35 | 0.232±0.005 | 0.108±0.028 | 0.215±0.061 | -0.373±0.077 | 100% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 19.43±0.65 | 9.14±0.20 | 66.41±0.62 | 2.570±0.046 | 3.288±0.123 |
| dynamic_multi | r2d2 | proxmpc | 5 | 20% | 38.00±0.00 | 4.22±1.01 | 1.473±0.876 | 0.396±0.054 | 0.583±0.078 | -0.462±0.026 | 100% | 2.212±0.271 | 6.321±0.849 | 16.230±2.960 | 0.0±0.0% | 1.00±0.00 | 8.82±0.30 | 0.0±0.0% | 0.619±0.127 | 11.89±0.72 | 5.55±0.77 | 59.65±0.06 | 2.240±0.309 | 6.637±1.051 |
| dynamic_multi | r2d2 | proxmpc_pred | 5 | 80% | 32.76±11.65 | 5.93±1.30 | 0.619±0.937 | 0.423±0.236 | 0.752±0.308 | -0.030±0.268 | 40% | 2.225±0.370 | 6.169±1.398 | 20.114±4.918 | 0.0±0.0% | 1.00±0.00 | 9.38±0.55 | 0.0±0.0% | 0.741±0.412 | 16.36±2.72 | 8.20±1.28 | 61.03±0.07 | 2.325±0.390 | 6.261±1.257 |
| dynamic_multi | r2d2 | regulated_pure_pursuit | 5 | 60% | 29.49±0.49 | 4.59±1.13 | 0.920±0.910 | 0.081±0.016 | 0.117±0.008 | -0.428±0.052 | 100% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 13.21±1.83 | 4.12±0.18 | 56.46±0.07 | 0.214±0.006 | 0.358±0.034 |
| dynamic_multi_noise | r2d2 | dwb | 5 | 100% | 31.63±1.34 | 4.77±0.01 | 0.235±0.002 | 0.052±0.012 | 0.083±0.022 | -0.249±0.026 | 100% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 19.97±0.11 | 10.25±0.08 | 61.12±0.16 | 3.257±0.045 | 4.183±0.250 |
| dynamic_multi_noise | r2d2 | graceful | 5 | 100% | 22.86±4.47 | 4.78±0.01 | 0.228±0.004 | 0.022±0.029 | 0.047±0.046 | -0.095±0.026 | 100% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 13.25±0.61 | 4.05±0.11 | 57.04±0.08 | 0.155±0.009 | 0.246±0.021 |
| dynamic_multi_noise | r2d2 | mppi | 10 | 90% | 43.04±12.73 | 5.74±0.61 | 0.266±0.118 | 0.117±0.039 | 0.287±0.118 | -0.319±0.136 | 100% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 19.63±0.55 | 9.12±0.23 | 66.65±0.55 | 2.548±0.056 | 3.319±0.223 |
| dynamic_multi_noise | r2d2 | proxmpc | 5 | 0% | n/a | 3.46±0.51 | 2.110±0.611 | 0.366±0.074 | 0.452±0.096 | -0.407±0.033 | 100% | 2.558±0.145 | 7.239±0.210 | 16.289±2.253 | 0.0±0.0% | 1.00±0.00 | 8.99±0.15 | 0.0±0.0% | 0.628±0.239 | 11.29±0.92 | 5.29±0.23 | 59.75±0.15 | 2.486±0.080 | 8.020±0.624 |
| dynamic_multi_noise | r2d2 | proxmpc_pred | 5 | 80% | 26.20±8.90 | 5.97±0.60 | 0.603±1.026 | 0.461±0.163 | 0.905±0.269 | 0.089±0.088 | 20% | 1.914±0.194 | 5.298±0.557 | 15.015±4.341 | 0.0±0.0% | 1.00±0.00 | 9.45±0.47 | 0.0±0.0% | 1.257±1.689 | 17.84±1.95 | 8.24±1.06 | 61.05±0.13 | 2.036±0.212 | 5.525±0.625 |
| dynamic_multi_noise | r2d2 | regulated_pure_pursuit | 5 | 80% | 28.78±1.92 | 4.84±0.65 | 0.498±0.522 | 0.093±0.040 | 0.130±0.041 | -0.388±0.044 | 100% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 13.92±1.16 | 4.15±0.14 | 56.49±0.15 | 0.210±0.002 | 0.373±0.016 |
| nav2_open | r2d2 | dwb | 5 | 100% | 13.84±2.41 | 4.60±0.20 | 0.236±0.000 | 0.001±0.000 | 0.001±0.000 | n/a | n/a | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.08±0.01 | 8.67±0.11 | 60.67±0.11 | 2.515±0.038 | 2.955±0.309 |
| nav2_open | r2d2 | graceful | 5 | 100% | 11.64±1.12 | 4.77±0.00 | 0.226±0.005 | 0.000±0.000 | 0.000±0.000 | n/a | n/a | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.22±0.20 | 4.20±0.19 | 56.73±0.08 | 0.147±0.005 | 0.175±0.018 |
| nav2_open | r2d2 | mppi | 5 | 100% | 11.36±0.45 | 4.76±0.00 | 0.236±0.002 | 0.003±0.000 | 0.004±0.000 | n/a | n/a | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.11±0.01 | 8.63±0.18 | 64.09±0.12 | 2.661±0.085 | 3.264±0.284 |
| nav2_open | r2d2 | proxmpc | 5 | 100% | 12.44±0.66 | 4.76±0.00 | 0.239±0.001 | 0.000±0.000 | 0.001±0.000 | n/a | n/a | 0.538±0.034 | 1.057±0.052 | 2.413±0.047 | 0.0±0.0% | 1.00±0.00 | 6.20±0.10 | 0.0±0.0% | 0.140±0.000 | 20.10±0.00 | 5.30±0.23 | 59.48±0.33 | 0.646±0.020 | 1.061±0.034 |
| nav2_open | r2d2 | proxmpc_pred | 5 | 100% | 13.82±0.77 | 4.76±0.00 | 0.239±0.001 | 0.000±0.000 | 0.000±0.000 | n/a | n/a | 0.436±0.017 | 1.004±0.055 | 1.705±0.040 | 0.0±0.0% | 1.00±0.00 | 5.78±0.13 | 0.0±0.0% | 0.040±0.000 | 20.09±0.00 | 5.53±0.30 | 60.96±0.12 | 0.542±0.015 | 1.071±0.046 |
| nav2_open | r2d2 | regulated_pure_pursuit | 5 | 100% | 13.07±1.33 | 4.77±0.00 | 0.232±0.002 | 0.000±0.000 | 0.000±0.000 | n/a | n/a | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.10±0.00 | 4.27±0.08 | 56.25±0.10 | 0.208±0.002 | 0.308±0.019 |
| static_box | r2d2 | dwb | 5 | 100% | 13.84±0.88 | 4.76±0.00 | 0.238±0.002 | 0.004±0.001 | 0.007±0.002 | 0.002±0.000 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.08±0.00 | 8.79±0.04 | 60.82±0.15 | 2.540±0.046 | 3.066±0.100 |
| static_box | r2d2 | graceful | 5 | 100% | 22.36±1.06 | 5.25±0.24 | 0.233±0.003 | 0.022±0.002 | 0.059±0.000 | 0.024±0.009 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 13.54±0.15 | 4.17±0.10 | 56.89±0.14 | 0.154±0.003 | 0.220±0.032 |
| static_box | r2d2 | mppi | 10 | 100% | 13.61±0.74 | 4.78±0.00 | 0.234±0.003 | 0.059±0.007 | 0.104±0.010 | 0.093±0.007 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.09±0.00 | 8.89±0.24 | 64.07±0.17 | 2.727±0.093 | 3.451±0.547 |
| static_box | r2d2 | proxmpc | 5 | 100% | 13.27±1.05 | 4.84±0.00 | 0.237±0.001 | 0.171±0.006 | 0.338±0.000 | 0.308±0.001 | 0% | 0.890±0.038 | 1.890±0.059 | 3.118±0.367 | 0.0±0.0% | 1.00±0.00 | 7.29±0.05 | 0.0±0.0% | 0.140±0.000 | 20.10±0.01 | 6.14±0.07 | 59.41±0.09 | 1.020±0.042 | 2.056±0.068 |
| static_box | r2d2 | proxmpc_pred | 5 | 100% | 13.38±0.96 | 4.87±0.02 | 0.213±0.013 | 0.161±0.005 | 0.296±0.000 | 0.294±0.000 | 0% | 0.863±0.017 | 2.214±0.077 | 3.673±0.295 | 0.0±0.0% | 1.00±0.00 | 7.15±0.16 | 0.0±0.0% | 0.040±0.000 | 20.02±0.03 | 6.50±0.17 | 60.83±0.09 | 0.967±0.013 | 2.386±0.087 |
| static_box | r2d2 | regulated_pure_pursuit | 5 | 100% | 26.63±0.85 | 5.33±0.07 | 0.236±0.001 | 0.039±0.001 | 0.086±0.005 | 0.050±0.000 | 0% | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 14.08±0.27 | 4.28±0.19 | 56.43±0.11 | 0.218±0.017 | 0.342±0.048 |

<!-- BENCHMARK_RESULTS_END -->

## License

[Apache-2.0](../LICENSE) — the full text is in [LICENSE](../LICENSE) and
attribution in [NOTICE](../NOTICE).
