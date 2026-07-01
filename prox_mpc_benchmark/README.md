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
| Mode **b2** — Nav2 + kinematic plant, no Gazebo (`run_nav2.py`) | **Run: 12/12 cells, 100% success** (cross-controller table below) |
| Cross-controller matrix (D2): ProxMPC / DWB / MPPI / RPP, open-world cell | **Run: 4 controllers × 3 repeats** ([comparison](../docs/controller-comparison-results.md)) |
| Map scaling (`generate_map.py` 7/15/30 m + scalable world xacro) | **Built + verified** (maps match world geometry) |
| Ackermann robot SDF + gz AckermannSteering bridge (mode-a bicycle, D3) | **Built + `gz sdf --check` valid**; full mode-a bicycle bringup not executed |
| Mode **a** obstacle scenarios + mode-a Ackermann/bike full bringup | **Scaffolded** (launch, goal_sender, compute_metrics) **— not executed this run** |

The cross-controller comparison (D2) runs in mode **b2** via `run_nav2.py`: each
Nav2 controller has a preset in `config/controllers/`, `benchmark_nav2.launch.py`
injects it into `FollowPath`, and every controller drives the *identical*
kinematic plant from the same start to the same goal, measured by the same
controller-agnostic metrics node. The narrative and the conclusion on ProxMPC are
in [docs/controller-comparison-results.md](../docs/controller-comparison-results.md).

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

_Auto-generated by `aggregate.py` from `results/scenarios.json` (54 runs). Values are mean±std (population) over repeats — the precision signal (D9). Not committed (D7)._

### Mode `a`

| scenario | model | n | success | t_goal[s] | path[m] | goal_err[m] | ct_rms[m] | ct_max[m] | solve_p50[ms] | solve_p95[ms] | solve_max[ms] | miss | sqp | qp_ext | infeas | slack[m] | rate[Hz] | cpu[%] | rss[MB] | cmp_p50[ms] | cmp_p95[ms] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| nav2_open | r2d2 | 2 | 100% | 12.28±0.08 | 3.79±0.01 | 0.224±0.002 | 0.006±0.001 | 0.016±0.005 | 0.238±0.012 | 0.438±0.010 | 1.952±0.009 | 0.0±0.0% | 1.00±0.00 | 4.37±0.10 | 0.0±0.0% | 0.000±0.000 | n/a | n/a | n/a | n/a | n/a |

### Mode `b1`

| scenario | model | n | success | t_goal[s] | path[m] | goal_err[m] | ct_rms[m] | ct_max[m] | solve_p50[ms] | solve_p95[ms] | solve_max[ms] | miss | sqp | qp_ext | infeas | slack[m] | rate[Hz] | cpu[%] | rss[MB] | cmp_p50[ms] | cmp_p95[ms] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| dynamic_circle | bike | 5 | 100% | 5.86±0.08 | 5.78±0.01 | 0.221±0.000 | 0.063±0.000 | 0.154±0.000 | 1.678±0.033 | 3.591±0.149 | 4.307±0.165 | 0.0±0.0% | 1.00±0.00 | 10.03±0.03 | 0.0±0.0% | 0.223±0.000 | n/a | n/a | n/a | n/a | n/a |
| dynamic_circle | r2d2 | 5 | 100% | 5.78±0.19 | 5.77±0.03 | 0.224±0.000 | 0.057±0.001 | 0.139±0.000 | 1.302±0.064 | 2.396±0.181 | 3.162±0.218 | 0.3±0.6% | 1.00±0.00 | 10.12±0.07 | 0.0±0.0% | 0.209±0.000 | n/a | n/a | n/a | n/a | n/a |
| dynamic_line_backward | bike | 5 | 100% | 5.67±0.19 | 5.74±0.03 | 0.238±0.000 | 0.011±0.000 | 0.034±0.000 | 1.472±0.029 | 3.607±0.139 | 4.574±0.115 | 0.0±0.0% | 1.00±0.00 | 10.05±0.07 | 0.0±0.0% | 0.245±0.000 | n/a | n/a | n/a | n/a | n/a |
| dynamic_line_backward | r2d2 | 5 | 100% | 5.88±0.04 | 5.81±0.00 | 0.184±0.000 | 0.020±0.000 | 0.045±0.000 | 1.225±0.031 | 2.923±0.074 | 3.367±0.305 | 0.0±0.0% | 1.00±0.00 | 10.03±0.00 | 0.0±0.0% | 0.165±0.000 | n/a | n/a | n/a | n/a | n/a |
| dynamic_line_forward | bike | 5 | 100% | 5.70±0.16 | 5.75±0.02 | 0.238±0.000 | 0.011±0.000 | 0.034±0.000 | 1.558±0.060 | 3.656±0.054 | 4.836±0.492 | 0.3±0.6% | 1.00±0.00 | 10.05±0.07 | 0.0±0.0% | 0.245±0.000 | n/a | n/a | n/a | n/a | n/a |
| dynamic_line_forward | r2d2 | 5 | 100% | 5.90±0.00 | 5.82±0.00 | 0.184±0.000 | 0.020±0.000 | 0.045±0.000 | 1.245±0.025 | 2.926±0.039 | 3.558±0.391 | 0.0±0.0% | 1.00±0.00 | 10.03±0.00 | 0.0±0.0% | 0.165±0.000 | n/a | n/a | n/a | n/a | n/a |
| static_box | bike | 5 | 100% | 5.15±0.07 | 5.76±0.00 | 0.235±0.000 | 0.022±0.000 | 0.045±0.000 | 2.267±0.112 | 5.885±0.145 | 6.633±0.103 | 0.0±0.0% | 1.00±0.00 | 10.25±0.01 | 0.0±0.0% | 0.626±0.000 | n/a | n/a | n/a | n/a | n/a |
| static_box | r2d2 | 5 | 100% | 5.26±0.05 | 5.83±0.00 | 0.175±0.000 | 0.055±0.000 | 0.108±0.000 | 1.570±0.023 | 4.490±0.144 | 4.844±0.300 | 0.3±0.6% | 1.00±0.00 | 9.99±0.01 | 0.0±0.0% | 0.278±0.000 | n/a | n/a | n/a | n/a | n/a |

### Mode `b2`

| scenario | model | n | success | t_goal[s] | path[m] | goal_err[m] | ct_rms[m] | ct_max[m] | solve_p50[ms] | solve_p95[ms] | solve_max[ms] | miss | sqp | qp_ext | infeas | slack[m] | rate[Hz] | cpu[%] | rss[MB] | cmp_p50[ms] | cmp_p95[ms] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| nav2_open | r2d2 | 3 | 100% | 13.46±1.24 | 4.75±0.00 | 0.246±0.000 | 0.001±0.000 | 0.001±0.000 | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.08±0.00 | 8.16±0.16 | 58.33±0.03 | 2.578±0.080 | 3.407±0.314 |
| nav2_open | r2d2 | 3 | 100% | 9.93±1.10 | 4.51±0.35 | 0.236±0.000 | 0.003±0.000 | 0.004±0.000 | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.10±0.00 | 8.28±0.04 | 61.50±0.03 | 2.685±0.016 | 3.347±0.254 |
| nav2_open | r2d2 | 3 | 100% | 11.19±0.01 | 4.75±0.00 | 0.246±0.000 | 0.000±0.000 | 0.001±0.000 | 0.277±0.005 | 0.667±0.052 | 3.518±0.120 | 0.0±0.0% | 1.00±0.00 | 2.80±0.07 | 0.0±0.0% | 0.140±0.000 | 20.15±0.07 | 4.32±0.24 | 56.80±0.09 | 0.378±0.017 | 0.770±0.066 |
| nav2_open | r2d2 | 3 | 100% | 10.73±0.74 | 4.75±0.00 | 0.246±0.002 | 0.000±0.000 | 0.000±0.000 | n/a | n/a | n/a | 0.0±0.0% | 0.00±0.00 | 0.00±0.00 | 0.0±0.0% | 0.000±0.000 | 20.10±0.00 | 3.70±0.05 | 53.78±0.05 | 0.207±0.004 | 0.281±0.020 |

<!-- BENCHMARK_RESULTS_END -->

## License

[Apache-2.0](../LICENSE) — the full text is in [LICENSE](../LICENSE) and
attribution in [NOTICE](../NOTICE).
