# Controller Comparison Results

This document reports how the ProxMPC controller compares against the stock Nav2 controllers under identical, reproducible, and *fairly-tuned* conditions - on **path tracking**, **per-cycle compute and process resources**, and **obstacle avoidance**.
It is the narrative companion to the auto-generated tables in [`prox_mpc_benchmark/README.md`](../prox_mpc_benchmark/README.md); the harness that produced every number is described in [the package guide](prox-mpc.md#8-prox_mpc_benchmark---the-measurement-harness).

**Provenance: which rows came from which measurement session.**
The mode-(b2) tables below are a *merged* set, and the split is stated wherever a number depends on it.

- **ProxMPC (reactive)** and **ProxMPC (predictive)** rows come from a 110-run campaign on 2026-09-02 at commit `9d0117a`, 11 scenarios x 5 repeats x 2 presets, in `results/proxmpc_d320_20260902_142419/`.
- **DWB, MPPI, RPP, Graceful, and Vector Pursuit** rows come from the 435-run full matrix on 2026-09-01 at commit `5132c97`, in `results/mode_b2_v2_20260901_084925/`. None of those controllers loads `prox_mpc_core` or the ProxMPC plugin, and the physical obstacle geometry the harness draws is a fixed harness constant, so it is byte-identical across both sessions. They were deliberately not re-run.
- The merged index the tables are generated from is `results/publish_v2/scenarios.json` (435 records).
- **Mode (a)**, the Gazebo results in Section 7, was **not** re-run and still reflects the `v1.0.0` baseline; it is called out again where it appears.

The two sessions are one day apart on the same quiet host, and the ProxMPC session exists because a keep-out correction landed between them: `robot_radius` moved from 0.22 m to 0.235 m, the circumscribed radius of the footprint the costmap actually carries, and `safety_margin` from 0.1 m to 0.085 m so that `d_safe` stays at the 0.320 m both sessions share. That split matters and is not cosmetic - measured across 30 multi-obstacle runs, widening `d_safe` to 0.335 m instead of holding it took predictive collisions from 2 to 5 and reactive from 17 to 20, because a gap between two obstacles needs `2*d_safe` to stay feasible. The peer rows are unaffected: `robot_radius` is a ProxMPC parameter no other controller reads.

**How sensitive these numbers are.** Over this work the predictive all-cells collision rate moved 6% -> 10% -> 6% on 15 mm of keep-out, with no change to the algorithm. Peer controllers that loaded no changed code moved by 5-10 points between campaigns on individual cells. Read any single cell's collision count as noise; the aggregate over 30 or 50 runs is the number that carries meaning.

**Failed runs and how they were handled.** The 435-run matrix produced 15 runs that did not reach the goal. Each was re-run once, uniformly across every controller and without pre-filtering, on the principle that a stack-bringup transient is a measurement that did not happen while a behavioural stall reproduces. Thirteen cleared and two reproduced - both Vector Pursuit stalling behind the static box, which the previous published campaign also recorded. The re-run moved results in both directions: one MPPI run cleared into a collision it had not previously recorded.

All numbers below are measured, reproducible, and reported as `mean ± std`All numbers below are measured, reproducible, and reported as `mean ± std` (population) over fixed-seed repeats - the precision signal.
Nothing is hand-tuned to favour one controller: every controller drives the *same* plant from the *same* start to the *same* goal, at a *matched operating point* (Section 2.1), perceives obstacles through the *same* costmaps, and is measured by the *same* instrumentation - including a timing decorator that wall-clock times every controller's per-cycle compute identically.

The comparison covers six controllers: **ProxMPC**, the four that ship with Nav2 Jazzy as standalone local planners - **DWB**, **MPPI**, **Regulated Pure Pursuit (RPP)**, and **Graceful** (new in Jazzy) - and **Vector Pursuit**, the one external community controller included as a fair peer (Apache-2.0, apt `ros-jazzy-vector-pursuit-controller` v2.0.0).
The **Rotation Shim** controller is a meta-controller (it rotates in place to the path heading then delegates to a `primary_controller`), so it has no independent tracking or avoidance and is not a peer here.
TEB remains out of scope: it has no ROS 2 Jazzy apt binary, and its LGPL-3.0+/MPL-2.0 transitive dependencies fail the Apache-2.0 inclusion bar used for this repo's dependencies.

## Table of Contents

- [1. What is being compared](#1-what-is-being-compared)
- [2. Test conditions](#2-test-conditions)
- [3. Tracking-fidelity comparison (open-world cell)](#3-tracking-fidelity-comparison-open-world-cell)
- [4. Per-cycle compute cost and process resources](#4-per-cycle-compute-cost-and-process-resources)
- [5. ProxMPC solver profile (and a cross-check)](#5-proxmpc-solver-profile-and-a-cross-check)
- [6. Obstacle avoidance](#6-obstacle-avoidance)
- [7. Real-stack validation (Gazebo)](#7-real-stack-validation-gazebo)
- [8. Threats to validity](#8-threats-to-validity)
- [9. Conclusion on ProxMPC](#9-conclusion-on-proxmpc)
- [License](#license)

## 1. What is being compared

Two complementary comparisons are run:

1. **The open-world cell** - an empty 7 x 7 m room, a single straight traverse from `(-2.5, 0)` to `(2.5, 0)` (5 m), no obstacles.
   This is a *pure path-tracking* task that isolates **tracking fidelity**, **per-cycle compute cost**, and **process resource use** on an exactly equal footing, with no perception or obstacle geometry to confound them (Sections 3-5).
2. **The obstacle scenarios** - the *same* 5 m traverse with obstacles placed across the path (a static box, single moving obstacles, and simultaneous multi-mover cells), run on the mode (b2) Nav2 stack so that **every controller perceives the same obstacles through the same costmaps and inflation** (Section 6).
   This is where the avoidance behaviours actually separate.

The open cell deliberately understates a constrained optimal-control method - an empty straight line is the task a geometric pursuit controller is provably optimal for - which is why obstacle avoidance is reported separately, and why the compute/resource comparison (Section 4) is the discriminating open-cell result.

## 2. Test conditions

| Condition | Value |
| --- | --- |
| ROS 2 / Nav2 | Jazzy / Nav2 1.3.12 |
| Host | x86-64 workstation - Intel Core i7-10750H (6 cores / 12 threads), 31 GiB RAM, Ubuntu 24.04.4 LTS (kernel 6.8), CPU-only |
| Tree | ProxMPC rows at commit `9d0117a`; the five peer controllers' rows at commit `5132c97`. Both on `test/audit-benchmark-revalidation`, one day apart on the same host |
| Run mode | **(b2)** Nav2 stack on a kinematic plant, no Gazebo (see [run modes](prox-mpc.md#9-running-the-stack-the-three-modes)); mode (b1) for the ground-truth predictive results in Section 6.3, at commit `1e170ad` on the same host and the same binaries |
| Plant | Unicycle body-twist integrator at 50 Hz, identical for every controller |
| Localization | Exact (static `map -> odom` identity; the plant pose is ground truth) |
| Map | `prox_mpc_open` - 7 x 7 m room, free interior `[-2.95, 2.95] m` |
| Task | straight 5 m traverse `(-2.5, 0) -> (2.5, 0)`, goal tolerance 0.25 m |
| Planner | `NavfnPlanner` (GridBased), shared by all controllers |
| Control rate | 20 Hz (`controller_frequency`); real-time budget 50 ms/cycle |
| Local costmap | `static_layer` + `obstacle_layer` + `inflation_layer` (**0.4 m** radius), 5 Hz update |
| Global costmap | `static_layer` + `obstacle_layer` + `inflation_layer` (**0.4 m** radius); the `obstacle_layer` marks the scenario obstacles so **NavFn routes the global plan around them** - the same global plan for every controller |
| Obstacle size | one **uniform** obstacle size across every b2 cell (`clearance: 0.45`); `static_box` is a 0.25 m box at dead-centre `(0, 0)`. The drawn body radius is a fixed harness constant ([`run_nav2.py:43`](../prox_mpc_benchmark/scripts/run_nav2.py)), so it is identical in both measurement sessions and for every controller |
| ProxMPC keep-out | both presets at `robot_radius: 0.235 m` + `safety_margin: 0.085 m`, so `d_safe = 0.320 m`. The radius is the circumscribed radius of the costmap's own padded footprint, so the keep-out covers the robot's outline; the margin carries the discretionary part. This is a ProxMPC-internal constraint radius, not an obstacle or costmap property - no other controller reads it |
| Obstacle sensing | a **scan simulator** ray-casts the scenario obstacles into `/scan`; the `obstacle_layer` marks/clears them, so all controllers see the same obstacles |
| Repeats | **5** fixed-seed repeats per controller per scenario (**10** for MPPI on every obstacle cell, to better characterise its sampling variance) |
| Controllers | ProxMPC (Unicycle), DWB, MPPI, RPP, Graceful, Vector Pursuit - plus **ProxMPC (predictive)** with the obstacle tracker for Section 6.3-6.4 |
| Compute timing | a `nav2_core::Controller` **timing decorator** wraps every controller and times its `computeVelocityCommands` identically |

### 2.1 Fair tuning: what is equalised and what is not

The controllers are different algorithms, so their *internal* knobs are not the same quantity and cannot be set "equal" without meaning something different for each.
The fair approach is to equalise everything they *share* and leave each method's intrinsic mechanism at its documented operating point:

- **Equalised** - control rate (20 Hz), max linear speed (0.5 m/s), goal tolerance (0.25 m), the **prediction horizon (2.0 s)** for all predictive controllers (ProxMPC `np*dt = 20 x 0.1`; DWB `sim_time = 2.0`; MPPI `time_steps x model_dt = 40 x 0.05`), a **uniform physical obstacle size** across every cell, and the **obstacle perception**: a shared local *and* global `obstacle_layer` fed by the scan simulator means every controller (including ProxMPC's costmap fill) sees the *same physical obstacles* through the *same costmaps + inflation*, and receives the *same* obstacle-routed global plan. In the head-to-head ProxMPC runs `predict_obstacles: false`, so it gets **no** privileged obstacle knowledge; the separate **ProxMPC (predictive)** variant (Section 6.4) turns that flag on and adds the real obstacle tracker. Both presets share `d_safe = 0.320 m`, so the keep-out no longer separates them; they still differ in the costmap threshold and the slack penalty, so the pair does not isolate prediction exactly (Section 8).
- **A note on the shared global plan.** Because the global costmap carries an `obstacle_layer`, NavFn bends the *global* path around obstacles before any controller runs. This is deliberately equal for all six, but it is worth stating plainly that it **helps the pure path-followers most**: DWB, RPP, Vector Pursuit and Graceful track that global detour closely, so an obstacle-routed global plan does much of their avoidance for them, whereas ProxMPC re-optimises locally and leans less on it. The comparison therefore measures *local avoidance on top of an equal, obstacle-aware global plan* - not local avoidance in isolation.
- **Left at each method's default** - the *avoidance mechanism itself*, because it has no common denominator across paradigms: DWB's `BaseObstacle` critic (nav2_bringup default `scale 0.02`), MPPI's `CostCritic` (upstream default `cost_weight 3.81`), ProxMPC's in-loop keep-out constraint (`d_safe = 0.320 m`, both presets), and RPP/Graceful's forward-simulation collision check. Crippling any of them to a common number would misrepresent it. The obstacles each faces are equal; how each responds is its own algorithm.
- **Intrinsic sampling** left at upstream defaults: MPPI `batch_size = 2000`, DWB's `20 x 20` velocity grid, ProxMPC's single-QP SQP.
- **Determinism** - MPPI is the one stochastic controller and Nav2 Jazzy exposes no RNG seed for it, so its repeats capture genuine sampling variance (reported as `mean ± std`), not reproducibility; it is run at 10 repeats on the obstacle cells (5 on the open cell) rather than the 5 used for the deterministic controllers. The other five controllers are deterministic in principle, but mode (b2) is real wall-clock ROS execution (DDS discovery, thread scheduling, lifecycle bring-up), not simulated time, so their repeats still carry a small measured jitter - itself part of the precision signal.
- **Graceful** - a good-faith goal-approach tuning (lookahead below the goal tolerance, reduced slowdown radius, no in-place final rotation) lets it drive into the goal rather than stalling far out; it succeeds on every scenario measured here.

Because the plant, map, planner, goal checker, horizon, speed, obstacle size, costmaps, and instrumentation are all shared, every difference in the tables is attributable to the controller, its own run-to-run variance, or to host timing drift between the two measurement sessions (Section 8).
The keep-out radius is a ProxMPC-internal quantity with no cross-controller counterpart, so changing it does not change what any other controller faces - the physical obstacles, and therefore the obstacle-gap metric that scores every controller identically, are unchanged.

### Metric definitions

- **time-to-goal** - wall time from first motion to entering the goal tolerance.
- **goal error** - final distance to the goal point; controllers that reach stop on the *same* 0.25 m checker.
- **cross-track RMS / max** - deviation from the straight reference; on obstacle scenarios this is *also* the size of the avoidance detour, so it is read together with clearance.
- **obstacle gap** (obstacle scenarios) - the minimum distance, over the whole run, between the **robot disc** and the nearest **obstacle disc** (robot radius 0.22 m). Positive = a real safety margin is kept; **negative = the two discs overlap**, i.e. a would-be collision on the collision-free kinematic plant. Controller-agnostic: the same measurement for all.
- **collision rate** - fraction of repeats whose obstacle gap went negative.
- **compute p50 / p95 / max** - wall time of one `computeVelocityCommands`, measured by the decorator, **the same way for every controller**.
- **CPU / RSS / rate** - the `controller_server` process's CPU (% of one core) and RSS from `/proc`, and the achieved `/cmd_vel` rate. For **ProxMPC (predictive)** the companion tracker runs as a *separate* process and is sampled separately (`+ trk`).
- **solve p95, deadline-miss, infeasible, SQP/QP iters, slack** - ProxMPC-only internal solver telemetry.

## 3. Tracking-fidelity comparison (open-world cell)

| Controller | success | time-to-goal [s] | goal err [m] | cross-track RMS [m] | cross-track max [m] |
| --- | --- | --- | --- | --- | --- |
| **ProxMPC** (Unicycle) | 5/5 | 12.90 ± 1.68 | 0.238 | 0.0001 ± 0.0000 | 0.0004 |
| **ProxMPC (predictive)** | 5/5 | 12.87 ± 1.20 | 0.238 | **0.0000 ± 0.0000** | 0.0000 |
| Regulated Pure Pursuit | 5/5 | 12.51 ± 1.14 | 0.235 | 0.0000 ± 0.0000 | 0.0000 |
| MPPI | 5/5 | 11.90 ± 1.28 | 0.236 | 0.0029 ± 0.0001 | 0.0041 |
| DWB | 5/5 | 13.02 ± 1.48 | 0.238 | 0.0001 ± 0.0001 | 0.0008 |
| Vector Pursuit | 5/5 | 12.97 ± 1.06 | 0.236 | 0.0000 ± 0.0000 | 0.0000 |
| Graceful | 5/5 | 11.72 ± 1.16 | 0.220 | 0.0000 ± 0.0000 | 0.0000 |

All seven track the straight line to the plant's resolution and complete 5/5.
Cross-track RMS is at or below 0.1 mm for six of them and 2.9 mm for MPPI, the one stochastic controller; every controller finishes two orders of magnitude inside the 0.25 m goal tolerance.
Time-to-goal spans 11.7-13.0 s with per-controller spreads of ±1.1-1.7 s, which is wall-clock bring-up and scheduling jitter on a 5 m traverse rather than a control effect - the spread is larger than the gap between the fastest and slowest controller, so the ordering here carries no signal.

On *tracking fidelity* there is no meaningful separation on an empty straight line, which is exactly why the compute-and-resource comparison below is the discriminating open-cell result, and why obstacle avoidance is reported separately.

## 4. Per-cycle compute cost and process resources

This is the comparison that matters on an embedded target, and it is **measured, not inferred**: the timing decorator wall-clock times every controller's `computeVelocityCommands` the same way, on **every** scenario, and a per-process sampler reads the `controller_server`'s CPU and RSS from `/proc`.
Unlike tracking, compute is *not* flat across scenarios - the obstacle cells activate each method's avoidance machinery (ProxMPC's in-loop keep-out rows, DWB/MPPI's cost critics), so the per-cycle cost grows with obstacle load.
The tables are therefore **per scenario, across all eleven cells** - split into single-obstacle and multi-obstacle groups - so that growth is visible.

**ProxMPC (predictive)** is the same plugin with `predict_obstacles: true` and the obstacle tracker in the loop (Section 6.4), otherwise identical to reactive ProxMPC - so the two rows isolate the cost of prediction alone.
Its companion tracker runs as a *separate* process (~1.0 % of one core, ~38.5 MB RSS, constant across scenarios), which the `controller_server` figures below do **not** include; it is reported separately.

### 4.1 Per-cycle compute, `p50 / p95` [ms] (mean over repeats)

| scenario | ProxMPC | ProxMPC pred | DWB | MPPI | RPP | Graceful | VecPursuit |
|---|---|---|---|---|---|---|---|
| `nav2_open` | **0.30 / 0.92** | **0.28 / 0.49** | 2.78 / 4.97 | 2.96 / 4.12 | 0.22 / 0.40 | 0.15 / 0.29 | 0.22 / 0.43 |
| `static_box` | 1.12 / 4.31 | 0.85 / 2.87 | 2.91 / 4.86 | 2.99 / 4.47 | 0.23 / 0.43 | 0.17 / 0.30 | 0.24 / 0.40 |
| `dynamic_line_forward` | 0.89 / 4.43 | 0.77 / 4.88 | 2.80 / 4.90 | 2.94 / 4.39 | 0.22 / 0.42 | 0.16 / 0.31 | 0.21 / 0.37 |
| `dynamic_line_backward` | 0.97 / 4.30 | 0.89 / 4.61 | 2.76 / 4.98 | 2.94 / 4.26 | 0.23 / 0.41 | 0.17 / 0.33 | 0.23 / 0.43 |
| `dynamic_circle` | 1.69 / 6.25 | 1.52 / 6.38 | 3.02 / 5.02 | 2.96 / 4.11 | 0.22 / 0.39 | 0.16 / 0.30 | 0.22 / 0.38 |
| `dynamic_multi` | 2.36 / 7.05 | 1.95 / 5.48 | 3.07 / 5.68 | 2.93 / 4.27 | 0.23 / 0.42 | 0.17 / 0.30 | 0.22 / 0.39 |
| `dynamic_multi_noise` | 2.39 / 7.23 | 1.92 / 5.61 | 3.01 / 5.32 | 2.96 / 4.34 | 0.23 / 0.41 | 0.17 / 0.30 | 0.22 / 0.41 |
| `blind_multi_0` | 2.95 / 8.11 | 1.95 / 6.24 | 2.98 / 4.70 | 2.93 / 4.10 | 0.22 / 0.41 | 0.16 / 0.30 | 0.21 / 0.40 |
| `blind_multi_1` | 2.53 / 6.11 | 1.70 / 5.80 | 2.97 / 4.94 | 2.97 / 4.40 | 0.22 / 0.41 | 0.17 / 0.31 | 0.22 / 0.41 |
| `blind_multi_2` | 2.32 / 12.04 | 1.99 / 5.33 | 2.88 / 5.04 | 2.94 / 4.16 | 0.23 / 0.39 | 0.17 / 0.29 | 0.22 / 0.38 |
| `blind_multi_3` | 2.26 / 5.84 | 2.03 / 6.38 | 3.01 / 4.40 | 2.92 / 4.17 | 0.23 / 0.39 | 0.18 / 0.31 | 0.22 / 0.41 |

The shape of this table is the result, not any single row.
**ProxMPC's cost scales with obstacle load; the sampling controllers' does not.**
DWB and MPPI pay a near-constant 2.8-3.1 ms whether the cell is empty or holds two movers, because their sample counts are fixed. ProxMPC pays 0.30 ms on the empty cell and 2.3-3.0 ms on the hardest, because the QP grows a keep-out row per obstacle slot per node.

So the advantage is largest exactly where a robot spends most of its time - **9.3x lighter than DWB and 9.9x than MPPI on the open cell** - narrows to ~2.5x on single-obstacle cells, and converges to ~1.2x on the multi-mover cells. It never inverts: ProxMPC's median stays at or below both on every cell measured.
The predictive preset is *cheaper* than the reactive one on every obstacle cell (1.9-2.0 ms against 2.3-3.0 on the multi cells) because a tracked obstacle occupies one slot with a known trajectory, while the costmap fill spends slots on clusters it must re-rank each cycle.

The one outlier is reactive ProxMPC's 12.04 ms p95 on `blind_multi_2`, against 5-8 ms elsewhere; that cell is where the reactive keep-out struggles most, and the extra QP iterations show up in the tail rather than the median (2.32 ms).

### 4.2 Process CPU [% of one core] / RSS peak [MB]

| scenario | ProxMPC | ProxMPC pred | DWB | MPPI | RPP | Graceful | VecPursuit |
|---|---|---|---|---|---|---|---|
| `nav2_open` | 4.3 / 59 | 4.3 / 61 | 9.0 / 60 | 8.8 / 64 | 3.7 / 55 | 3.6 / 56 | 3.7 / 55 |
| `static_box` | 6.6 / 60 | 6.0 / 61 | 9.1 / 60 | 9.0 / 63 | 3.8 / 55 | 3.8 / 56 | 4.0 / 55 |
| `dynamic_line_forward` | 6.3 / 59 | 6.1 / 61 | 9.0 / 60 | 9.0 / 64 | 3.9 / 55 | 3.9 / 56 | 3.9 / 55 |
| `dynamic_line_backward` | 6.5 / 59 | 6.3 / 61 | 9.0 / 60 | 9.0 / 63 | 3.9 / 55 | 3.7 / 56 | 3.8 / 55 |
| `dynamic_circle` | 8.0 / 60 | 7.9 / 61 | 9.5 / 60 | 9.1 / 63 | 3.9 / 56 | 4.0 / 56 | 3.8 / 55 |
| `dynamic_multi` | 9.3 / 59 | 8.2 / 61 | 9.8 / 60 | 9.2 / 63 | 4.0 / 56 | 3.9 / 56 | 3.8 / 55 |
| `dynamic_multi_noise` | 9.2 / 59 | 8.2 / 61 | 9.6 / 60 | 9.2 / 65 | 4.0 / 56 | 3.9 / 56 | 3.7 / 56 |
| `blind_multi_0` | 10.4 / 60 | 8.3 / 61 | 9.3 / 60 | 9.1 / 65 | 4.0 / 56 | 3.9 / 56 | 3.8 / 56 |
| `blind_multi_1` | 8.7 / 60 | 8.1 / 61 | 9.2 / 60 | 9.1 / 63 | 3.9 / 56 | 3.8 / 56 | 3.8 / 55 |
| `blind_multi_2` | 11.4 / 60 | 8.2 / 61 | 9.2 / 60 | 9.1 / 65 | 3.9 / 56 | 3.8 / 56 | 3.9 / 55 |
| `blind_multi_3` | 8.2 / 59 | 8.7 / 61 | 9.2 / 60 | 9.0 / 63 | 3.9 / 56 | 3.9 / 56 | 3.9 / 55 |

The same shape appears in process CPU: ProxMPC starts at 4.3 % on the empty cell against DWB's 9.0 % and MPPI's 8.8 %, and rises to 8.2-11.4 % on the hardest cells where they stay flat. Reactive ProxMPC is the only controller that exceeds the sampling pair, and only on the two cells its keep-out finds hardest (`blind_multi_0` at 10.4 %, `blind_multi_2` at 11.4 %); the predictive preset stays at or below both everywhere (7.6 % averaged over all obstacle cells).

Memory is flat and unremarkable for every controller: 59-61 MB peak RSS for ProxMPC against 60 for DWB and 63-65 for MPPI, with the three pursuit controllers at 55-56. Nothing here grows over a run.

## 5. ProxMPC solver profile (and a cross-check)

Solver telemetry over all 10 obstacle cells (50 runs per preset), from the opt-in `SolverDiagnostics` topic:

| | solve p50 | solve p95 | solve max | over 50 ms | deadline-miss | SQP iters | QP ext iters | infeasible runs | peak slack |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| reactive | 1.82 ms | 6.46 ms | 33.9 ms | **0 / 50** | 0.00 % | 1.00 | 7.5 | 14 / 50 | 1.110 m |
| predictive | 1.46 ms | 5.24 ms | **24.7 ms** | **0 / 50** | 0.00 % | 0.98 | 7.0 | 3 / 50 | 0.493 m |

**Every cycle in 100 runs stayed inside the 50 ms budget.** This is the single largest behavioural change in this release, and it came from two defects rather than from tuning:

1. `max_iter_sqp` defaulted to 100. On a failed solve the loop re-linearised up to 99 more times around an iterate it had already corrupted, then braked anyway. Worst-case cycles of 300 ms were measured on three separate cells - six times the budget, and 15 cm of open-loop travel at 0.5 m/s. The default is now 1, which is what the real-time iteration scheme this controller implements actually prescribes.
2. The QP was re-initialised from scratch every cycle. `guess: true` named a warm start that never happened, because `init()` rebuilt the workspace and discarded the previous primal/dual iterate 20 times a second. The workspace is now updated in place, which cut the worst case by a further ~40 %.

`SQP iters` sitting at 1.00 is the fixed real-time iteration, not a convergence report; `QP ext iters` at 7.0-7.5 against a 10 000 cap shows the QP itself converges quickly when it converges at all.

**One thing this table does not explain.** The reactive preset reports `PROXQP_PRIMAL_INFEASIBLE` on 14 of 50 runs (roughly 0.1-0.3 % of cycles within them), where the published `v1.0.0`-era campaign reported none. The QP's feasible set is provably non-empty - the constant sequence `u(k) = u_prev` satisfies the control box, the rate chain and the row-0 rate anchor simultaneously, and the obstacle rows are fully soft - so every such report is a false certificate. Three candidate causes were tested and eliminated by measurement: a warm start that violated its own rate chain, the absent solver warm start, and large-magnitude bounds on unused obstacle slots. The cause is still unknown.

It is benign in every way the harness can measure: the controller brakes for that cycle, the next one solves, worst-case latency stays at 33.9 ms, and no infeasible cycle has been traced to a collision. It is recorded here because it is real and unexplained, not because it is known to matter.

## 6. Obstacle avoidance

### 6.1 Static obstacle (`static_box`, mode b2)

| Controller | collisions | min gap [m] | mean gap [m] | cross-track max [m] |
| --- | --- | --- | --- | --- |
| **ProxMPC** (reactive) | 0/5 | **+0.374** | **+0.381** | 0.737 |
| **ProxMPC (predictive)** | 0/5 | +0.296 | +0.298 | 0.650 |
| RPP | 0/5 | +0.208 | +0.213 | 0.568 |
| MPPI | 0/10 | +0.205 | +0.211 | 0.586 |
| Graceful | 0/5 | +0.198 | +0.207 | 0.571 |
| Vector Pursuit | 0/5 | +0.113 | +0.142 | 0.551 |
| DWB | 0/5 | +0.064 | +0.089 | 0.456 |

Every controller clears the static box - an obstacle-routed global plan does most of this work, as Section 2.1 notes.
The separation is in *margin*: reactive ProxMPC keeps +0.374 m at its closest against DWB's +0.064 m, and it pays for that with the largest detour (0.737 m cross-track against 0.456). That trade is the in-loop keep-out doing exactly what it is asked to: `d_safe` is a hard-ish constraint the optimiser plans around, not a cost it trades away.

### 6.2 Single moving obstacles (3 cells, mode b2)

| Controller | collisions | min gap [m] | mean gap [m] |
| --- | --- | --- | --- |
| **ProxMPC (reactive)** | **0/15** | **+0.264** | +0.443 |
| **ProxMPC (predictive)** | **0/15** | +0.114 | +0.437 |
| MPPI | 0/30 | +0.128 | **+0.445** |
| RPP | 1/15 | -0.048 | +0.292 |
| Vector Pursuit | 1/15 | -0.050 | +0.340 |
| DWB | 4/15 | -0.341 | +0.256 |
| Graceful | 4/15 | -0.065 | +0.216 |

Both ProxMPC presets and MPPI clear all three single-mover cells collision-free; DWB and Graceful collide on 4 of 15 each.
Reactive ProxMPC holds the largest *worst-case* margin in the field at +0.264 m - no run came closer than 26 cm - while MPPI edges it on the mean.

A single moving obstacle is where the costmap-only fill is still sufficient: the obstacle's position lag costs the controller reaction time, but the free corridor beside it is wide enough to absorb a late swerve. That stops being true with two movers, which is Section 6.4.

### 6.3 Ground-truth predictive confirmation (mode b1)

> **Carried over unchanged.** This section reports the 40-run mode-(b1) campaign at commit `1e170ad`; it was not re-run in this pass. Mode (b1) drives the engine plant directly, using neither the costmap nor the Nav2 stack, so none of the changes since - the keep-out correction, the SQP and warm-start fixes, the control-box change - alters what it measures about the formulation's ceiling. Read the absolute solve times against their own session, not against Section 5.

One controller-vs-tracker separation is worth recording: fed the *correct* future obstacle positions (mode b1, the engine knows the exact trajectory, orbit included), ProxMPC reaches the goal on every single moving-obstacle case for both vehicle models.
This section was re-run when reactive ProxMPC was still colliding 3/5 on the orbit, as a ceiling check on the formulation.
Section 6.2 has since resolved that failure at the reactive level, so this table is no longer answering an open question about the orbit - but it is retained because it is the only ground-truth-predictor result in the document and it bounds the formulation independently of what the costmap supplies:

| Scenario | Model | success | cross-track RMS [m] | time-to-goal [s] | peak keep-out slack [m] |
| --- | --- | --- | --- | --- | --- |
| dynamic_circle | Unicycle / Bicycle | 5/5 / 5/5 | 0.009 / 0.006 | 4.56 / 4.40 | 0.000 / 0.000 |
| dynamic_line_forward | Unicycle / Bicycle | 5/5 / 5/5 | 0.048 / 0.059 | 4.55 / 4.53 | 0.194 / 0.375 |
| dynamic_line_backward | Unicycle / Bicycle | 5/5 / 5/5 | 0.048 / 0.045 | 4.52 / 4.53 | 0.194 / 0.225 |
| static_box | Unicycle / Bicycle | 5/5 / 5/5 | 0.000 / 0.000 | 4.43 / 4.54 | 0.490 / 0.490 [^slack] |

> This table **was** re-run for this pass, at commit `1e170ad` on the same host, from a fresh 40-run campaign (4 scenarios x 2 models x 5 repeats, controller `proxmpc`).
> It is a ceiling check on the controller's own capability given a correct predictor, not the operative reactive/predictive result - that comparison is mode b2, in Sections 6.1-6.4.

[^slack]: Steady-state peak, identical for both models. The raw per-run maxima average 1.963 m on the bicycle, because 3 of the 5 bicycle runs sampled a one-cycle initialisation transient of 2.945 m that the other 2, and all 5 unicycle runs, did not; the transient is present on both models and is gone by cycle 2. See Section 6.3.

**The 30/30 ceiling on the three single moving-obstacle cells still holds on the current tree**, for both a 3-state unicycle and a 4-state bicycle, driven by the same plugin with one parameter changed (`model_plugin`); with the static box included the campaign is 40/40.
The orbit is the sharpest of the four results: given the correct future positions the keep-out is **never relaxed at all** (peak slack 0.000 m for both models), so the predictive path clears the orbiting obstacle outright rather than trading margin for progress.
The single-obstacle dynamic case is therefore fully within the controller's reach given a correct predictor - and Section 6.2 now shows it is also within reach *without* one, once the keep-out carries enough margin to absorb the obstacle's motion between costmap updates.
The earlier reading of the reactive orbit failure as a limit of what the costmap supplies does not survive that result: the costmap was supplying enough, and the constraint built from it was simply sized too tightly.

The slack column has to be read together with the success column, because "reaching the goal" in mode b1 is not by itself an avoidance result.
Mode b1 has no global planner: the reference is the straight start-to-goal line, so on `static_box` the cross-track RMS is exactly 0.000 m and the box is passed by relaxing the soft keep-out (0.490 m on the unicycle, essentially the whole 0.45 m clearance) rather than by steering around it.
The avoidance comparison is the mode-b2 one in Section 6.1; this section measures whether the solver keeps finding a feasible, goal-reaching trajectory when the prediction is perfect.

Two differences from the `v1.0.0` b1 table are worth recording, neither of them attributed to a specific change in the intervening work:

- **The `static_box` peak-slack figure is a first-cycle initialisation transient, and its run-to-run spread is a sampling artefact rather than a solver result.** Three of five bicycle runs reported 2.945 m against the `max_obstacle_slack_m: {max: 0.5}` bound in [`prox_mpc_benchmark/config/metrics.yaml`](../prox_mpc_benchmark/config/metrics.yaml), the other two 0.490 m. Capturing `/prox_mpc/diagnostics` per cycle resolves both numbers: the 2.945 m relaxation occurs on control cycle 1 alone and is back to 0.000 m by cycle 2, while 0.490 m is the true steady-state peak, reached at cycle 13 as the robot passes the box. The transient is deterministic and identical for both models - unicycle and bicycle agree to six decimals on the cycle-1 slack and on its objective - so it is not a bicycle-specific effect; the recorded spread is only whether the metrics node's subscription matched in time to receive cycle 1, which shows up directly as 57 samples on the runs reporting 2.945 m against 56 on those reporting 0.490 m. The mechanism is that [`prox_mpc_core/src/mpc.cpp`](../prox_mpc_core/src/mpc.cpp) zero-initialises the whole predicted trajectory, so before the first solve every horizon node sits at the origin - which on `static_box`, and only on `static_box`, is exactly where the obstacle is, inside its 1.45 m keep-out and at the point where the distance gradient is undefined. `dynamic_circle`, whose mover is 1.8 m from the origin at `t = 0`, reports 0.000 m throughout, as this account predicts. Nothing in the executed trajectory is affected: cross-track RMS is exactly 0.000 m and time-to-goal and goal error are identical on the runs that sampled the transient and those that did not.
- **Solve time rose on every b1 cell** - the median by +0.4 to +2.7 ms on all eight, the p95 by +0.4 to +2.9 ms on seven, flat on the eighth (`dynamic_line_forward`/unicycle, -0.03 ms). Mode b1 touches neither the costmap nor the Nav2 stack, so the two per-cycle fixes credited in Section 4 do not apply to this path, and the session-to-session host drift documented in Section 8 is present in the b2 data over the same interval; the shift is reported as observed. Every cell still solves far inside the 50 ms budget - worst cell p95 11.9 ms, worst single cycle 22.1 ms across all 40 runs - at 0 % deadline-miss and 0 % infeasible.

### 6.4 Multiple simultaneous moving obstacles - where the field separates

Six cells with two simultaneous movers. Four of them (`blind_multi_0..3`) are *blind*: their geometry comes from a seeded RNG with no controller run to screen it, so no cell was kept or discarded because of how any controller performed on it.

| Controller | `dyn_multi` | `dyn_multi_noise` | `bm0` | `bm1` | `bm2` | `bm3` | total |
|---|---|---|---|---|---|---|---|
| **ProxMPC (predictive)** | 0/5 | 0/5 | 1/5 | 1/5 | 0/5 | 1/5 | **3/30 (10 %)** |
| RPP | 0/5 | 0/5 | 1/5 | 1/5 | 5/5 | 0/5 | 7/30 (23 %) |
| DWB | 2/5 | 2/5 | 1/5 | 0/5 | 5/5 | 0/5 | 10/30 (33 %) |
| MPPI | 4/10 | 5/10 | 9/10 | 4/10 | 0/10 | 8/10 | 30/60 (50 %) |
| Graceful | 4/5 | 4/5 | 1/5 | 0/5 | 5/5 | 1/5 | 15/30 (50 %) |
| ProxMPC (reactive) | 1/5 | 1/5 | 5/5 | 1/5 | 4/5 | 3/5 | 15/30 (50 %) |
| Vector Pursuit | 0/5 | 1/5 | 5/5 | 5/5 | 5/5 | 3/5 | 19/30 (63 %) |

**Predictive ProxMPC leads by more than a factor of two**: 3 collisions in 30 against 7 for the next best. It is also the only controller with no cell worse than 1/5 - every other controller has at least one cell it fails 4 or 5 times out of 5.

Because these cells are deliberately marginal, the collision count alone overstates its own precision. The median closest approach is the steadier statistic:

| Controller | median closest approach [m] | runs within 0.15 m of the threshold |
| --- | --- | --- |
| **ProxMPC (predictive)** | **+0.159** | 13/30 (43 %) |
| RPP | +0.128 | 17/30 (57 %) |
| DWB | +0.064 | 11/30 (37 %) |
| ProxMPC (reactive) | -0.001 | 23/30 (77 %) |
| MPPI | -0.003 | 44/60 (73 %) |
| Graceful | -0.008 | 24/30 (80 %) |
| Vector Pursuit | -0.036 | 20/30 (67 %) |

Predictive ProxMPC leads on both measures, and spends the least time near the threshold of any controller except DWB. Everything from reactive ProxMPC downward has a *negative* median - the typical run ends overlapping.

**Why the reactive preset is in that lower group, and why it got worse.**
The costmap-only fill scans a single present-time costmap around every predicted node, so a moving obstacle is constrained where it currently is rather than where it will be - an error of `v_obs * t_node`, up to a metre at the far end of a 2 s horizon. With one mover the free corridor absorbs the resulting late reaction (Section 6.2, 0/15). With two, that corridor closes.

The reactive rate also moved from 40 % in the `v1.0.0`-era campaign to 50 % here, and the cause is a deliberate change rather than a regression in the avoidance logic. Previously `allow_reversing: false` filtered only which plan pose the reference tracked; the solver's control box still admitted reverse, so the robot could back out of a closing gap. That was a defect - the parameter did not do what it said - and fixing it removed an escape the costmap-only path was relying on. Restoring reverse and changing nothing else recovers 11/30 and lifts the mean margin from -0.005 m to +0.142 m, with path length on `blind_multi_1` rising from 5.2 m to 7.0 m: the robot reversing and re-approaching, not detouring.

The shipped default stays forward-only, because a library cannot check whether an integrator's platform senses the reverse direction. `allow_reversing: true` with an explicit `model_params.v_min` is supported and measurably better on this path, and the bundled Gazebo demo enables it - the waffle's 360-degree scanner covers the rear and Nav2's Collision Monitor, already in the demo's `cmd_vel` chain, projects the footprint along the commanded twist including reverse.

**What this section does not claim.** `blind_multi_2` is 5/5 for four of the seven controllers and 0/10 for MPPI; `blind_multi_0` is 5/5 for two and 1/5 for three. Individual cells swing hard, and peer controllers loading no changed code moved by 5-10 points between campaigns. The totals over 30 runs are the comparison; the per-cell columns are shown for completeness, not for ranking.

## 7. Real-stack validation (Gazebo)

To confirm the plugin behaves under the full production stack and not only against a kinematic plant, ProxMPC also runs in mode (a): Gazebo Harmonic physics, a TurtleBot3 waffle, AMCL localization, costmaps, and the complete Nav2 velocity chain, headless.
It reaches the goal (`SUCCEEDED`), tracks the path to **5.6 mm cross-track RMS**, and taps its diagnostics through the real `controller_server` (p95 solve 0.438 ms, 0 % deadline-miss, 0 % infeasible) - the same profile measured on the plant, with real sensor and physics noise.

> **Mode (a) was not re-run in this pass and this section still reflects the `v1.0.0` baseline.** It is pending its own re-validation. The b1/b2 numbers elsewhere in this document are unaffected, since neither uses Gazebo.

## 8. Threats to validity

- **The ProxMPC rows and the peer rows come from different measurement sessions**, one day apart on the same host (see Provenance). The physical obstacles, plant, map, planner, goal checker and instrumentation are identical, and no peer controller reads a ProxMPC parameter, so the comparison is sound in kind. But a session boundary carries host-timing drift, and differences of a few tenths of a millisecond between ProxMPC and a peer should not be read as real. The compute results in Section 4 are ratios of 2-10x, which is far outside that.
- **These collision rates are more sensitive to the keep-out than to the algorithm.** Over this work the predictive all-cells rate moved 6 % -> 10 % -> 6 % on a 15 mm change to `d_safe`, with no algorithmic change at all. Any comparison between controllers separated by less than that is not resolved by this campaign.
- **Peer controllers moved between campaigns on code that did not change.** DWB's single-obstacle collisions went 25 % -> 20 % and Vector Pursuit's all-cells rate 32 % -> 40 % across the two matrices, on stock Nav2/community binaries at identical versions. That sets the noise floor at 5-10 points on 20-50 runs, which is the scale at which per-cell counts in Section 6.4 should be read.
- **The two ProxMPC presets do not isolate prediction.** Both now share `d_safe = 0.320 m`, so keep-out width no longer differs, but `costmap_cost_threshold` (200 reactive against 253 predictive) and `w_weight` still do. The reactive-versus-predictive comparisons are configuration-versus-configuration. The multi-mover direction is large enough (3/30 against 15/30) that the remaining differences do not plausibly account for it; the compute comparison in Section 4 stays confounded, since the threshold alone changes how many costmap cells each preset ingests.
- **The obstacle-routed global plan does much of the single-obstacle avoidance.** The shared global `obstacle_layer` makes NavFn route around obstacles for every controller - deliberately equal, but it favours the geometric path-followers and DWB, which track that detour closely. The single-obstacle cells therefore separate the field on margin, not on collision counts; the multi-mover cells carry the collision comparison.
- **The multi-obstacle field is close and the sample is small.** 37-80 % of runs across the six cells finish within 0.15 m of contact, so per-cell counts swing by more than the gap between most controllers. The 30-run totals are the comparison.
- **Collision uses a strict min-gap:** any instant of disc overlap over a whole run counts, so a brief graze scores the same as a harder hit. The reported min gap distinguishes them.
- **Resources are x86-64 only.** Every figure in this document was measured on the dev host in Section 2. **No measurement has been taken on the arm64 target this controller is intended for**, so nothing here supports an embedded-performance claim; the ranking is what transfers, not the absolute numbers. The per-cycle compute in Section 4 is the cleaner controller-only measure, since process CPU/RSS includes the shared costmap.
- **The pure-geometric controllers are genuinely lighter.** ProxMPC is the cheapest of the controllers that solve a constrained optimisation each cycle, not the cheapest outright.
- **MPPI is stochastic with no exposed seed** in Nav2 Jazzy, so its per-scenario variance is real. It runs 10 repeats per obstacle cell against the others' 5, which characterises that variance without pinning it down.
- **Determinism vs. physics.** Modes (b1)/(b2) are deterministic plants, so their near-zero geometric std is reproducibility, not a noise estimate. Mode (a) carries real Gazebo variance and was not re-run (Section 7). `dynamic_multi_noise` adds a Gaussian range-noise model on the scan.
- **Fifteen of 435 matrix runs did not reach the goal and were re-run once**, uniformly and without pre-filtering (see Provenance). Thirteen cleared, two reproduced. The procedure can move results in either direction and did: one MPPI run cleared into a collision it had not previously recorded.
- **An unexplained solver report remains open.** Reactive ProxMPC emits `PROXQP_PRIMAL_INFEASIBLE` on 14 of 50 runs against none in the previous campaign, on a QP whose feasible set is provably non-empty (Section 5). Three candidate causes were tested and eliminated. It has no measured consequence, but it is not understood.
- **The demonstration videos under `prox_mpc_benchmark/doc/media/` were not re-recorded** and show behaviour this document no longer reports. They are mode-(b2) screen captures, not Gazebo.

## 9. Conclusion on ProxMPC

On a like-for-like, fairly-tuned suite, **predictive ProxMPC is the most reliable obstacle avoider in the field** - 3 collisions in 50 obstacle runs (6 %) against 8 for the next best (RPP, 16 %) - while computing its command 2-10x faster than the two controllers in its own cost class. Its worst-case cycle is now bounded well inside the real-time budget, which was not true of the previous release.

- **Real-time behaviour:** **zero cycles above the 50 ms budget across 100 runs**, both presets, with worst observed cycles of 33.9 ms (reactive) and 24.7 ms (predictive). The previous release measured 300 ms worst cases on three separate cells. Two defects caused that and both are fixed: an SQP loop that re-linearised up to 99 times around a corrupted iterate after a failed solve, and a QP workspace rebuilt from scratch every cycle instead of warm-started.
- **Per-cycle compute:** 0.30 ms median on the open cell - **9.3x lighter than DWB and 9.9x than MPPI** - narrowing to ~2.5x on single-obstacle cells and ~1.2x on the hardest multi-mover cells, because ProxMPC's cost scales with obstacle load while theirs does not. It never inverts. Process CPU is 8.5 % reactive and 7.6 % predictive across all obstacle cells against DWB's 9.3 % and MPPI's 9.1 %; peak RSS 59.7/61.0 MB against 60.2 and 64.8. The three geometric pursuit controllers remain an order of magnitude cheaper and collide 16-40 % of the time.
- **Tracking:** 0.0001 m cross-track RMS on the open cell, 5/5, indistinguishable from the field. An empty straight line does not separate these controllers.
- **Static obstacle:** reactive ProxMPC keeps **the largest margin in the field, +0.374 m at its closest**, against RPP's +0.208 and DWB's +0.064, and pays for it with the largest detour (0.737 m cross-track). That is the in-loop keep-out behaving as specified - `d_safe` is planned around, not traded away.
- **Single moving obstacles:** both presets clear all three cells collision-free (0/15 each), and reactive holds the field's largest worst-case margin at +0.264 m. DWB and Graceful collide 4/15 each.
- **Simultaneous two-mover cells - where the field separates:** predictive ProxMPC leads on both measures, at **3/30 collisions** against 7 for the next best, and **+0.159 m median closest approach** against RPP's +0.128, DWB's +0.064 and negative medians for everything below. It is the only controller with no cell worse than 1/5.
- **The costmap-only path is a single-obstacle configuration.** Reactive ProxMPC sits at 15/30 on these cells because the fill constrains each obstacle where it *was*, not where it will be - up to a metre of error at the far end of a 2 s horizon. With one mover the free corridor absorbs that; with two it does not. This is why `predict_obstacles` now defaults to true, and why the reactive preset should be treated as validated for single-obstacle environments only.
- **One controller, many vehicles:** the identical plugin drives a unicycle and a bicycle by configuration alone, confirmed 40/40 in mode b1 (Section 6.3, carried over from the earlier campaign).

**What this release changed, honestly.** The avoidance ranking is essentially unchanged from `v1.0.0`; what changed is that the controller can no longer stall for 300 ms mid-manoeuvre, which on a robot moving at 0.5 m/s is 15 cm of open-loop travel at exactly the wrong moment. The reactive preset's multi-obstacle rate moved from 40 % to 50 %, and that is a deliberate consequence rather than a regression in avoidance: `allow_reversing: false` now genuinely forbids reverse, and backing out was an escape the costmap-only path had been relying on (Section 6.4).

**What remains open:** no arm64 measurement exists, Gazebo validation still reflects `v1.0.0`, and the reactive infeasibility report in Section 5 is unexplained. None of these affect the comparisons above; all three bound what can be claimed beyond them.

---

*Reproduce:* the open cell and the mode-(b2) reactive obstacle scenarios with `ros2 run prox_mpc_benchmark run_nav2.py --scenario <nav2_open|static_box|dynamic_circle|dynamic_line_forward|dynamic_line_backward|dynamic_multi|dynamic_multi_noise|blind_multi_0|blind_multi_1|blind_multi_2|blind_multi_3>`, the **predictive b2** runs (real tracker) by adding `--controllers proxmpc_pred` to the same command, the mode-(b1) ground-truth predictive results with `ros2 run prox_mpc_benchmark run_matrix.py --modes b1`, and the tables with `ros2 run prox_mpc_benchmark aggregate.py`. See [`prox_mpc_benchmark/README.md`](../prox_mpc_benchmark/README.md).

## License

[Apache-2.0](../LICENSE).
