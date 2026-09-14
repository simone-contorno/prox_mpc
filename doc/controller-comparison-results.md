# Controller Comparison Results

This document reports how the ProxMPC controller compares against the stock Nav2 controllers under identical, reproducible, and *fairly-tuned* conditions - on **path tracking**, **per-cycle compute and process resources**, and **obstacle avoidance**.
It is the narrative companion to the auto-generated tables in [`prox_mpc_benchmark/README.md`](../prox_mpc_benchmark/README.md); the harness that produced every number is described in [the package guide](prox-mpc.md#8-prox_mpc_benchmark---the-measurement-harness).

**Provenance: one campaign, one machine state.**
Every mode-(b2) number below comes from a **single 435-run matrix recorded on 2026-09-12** - all seven controllers, 11 scenarios, 5 repeats each (10 for MPPI on the obstacle cells) - run back-to-back on one quiet host with nothing else executing, and aggregated from `results/runs/`.
The mode-(b1) rows in Section 6.3 come from the 40-run core-sim matrix recorded immediately afterwards in the same session.

This replaces the merged two-session set published previously, and it removes the cross-session caveat entirely: there is no longer any row whose comparability depends on two campaigns being close enough.

- **Tree:** the `2.0.0` release of the `test/audit-benchmark-revalidation` branch. The measured binary is the one described in Sections 5, 6.2 and 6.4, built from the sources that release carries.
- **Mode (a)**, the Gazebo results in Section 7, was re-run on 2026-09-13 on the same host: 5 runs of the demo's open-room traverse on the shipped demo parameters.

**How sensitive these numbers are.** Over this work the predictive all-cells collision rate moved 6% -> 10% -> 6% on 15 mm of keep-out, with no change to the algorithm. Re-running the identical stock binaries in this campaign moved peer controllers by up to 8 points against the previous matrix (MPPI's all-cells rate 26% -> 18%, Graceful 40% -> 36%, RPP 20% -> 16%, DWB 26% -> 30%, Vector Pursuit 22% -> 26%), on code that did not change. Read any single cell's collision count as noise; the aggregate over 30 or 50 runs is the number that carries meaning, and a gap smaller than about 8 points is not resolved by this campaign.

**Failed runs and how they were handled.** Every cell was checked against its own stack log for the controller actually receiving a path, and a cell whose Nav2 bring-up never got that far was discarded and re-run rather than recorded - a stack-bringup transient is a measurement that did not happen, while a behavioural stall reproduces and is kept. Four cells needed one such re-run; none needed more than one. All 435 records in the final set show a real path delivered to the controller. Five runs then failed *behaviourally* and are reported as failures: four Vector Pursuit runs stalling behind the static box, and one Graceful run on `dynamic_multi`.

All numbers below are measured, reproducible, and reported as `mean ± std` (population) over fixed-seed repeats - the precision signal.
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
| Tree | one tree for every row: the `2.0.0` release of `test/audit-benchmark-revalidation`, all seven controllers recorded back-to-back on 2026-09-12 |
| Run mode | **(b2)** Nav2 stack on a kinematic plant, no Gazebo (see [run modes](prox-mpc.md#9-running-the-stack-the-three-modes)); mode (b1) for the ground-truth predictive results in Section 6.3, recorded in the same session on the same binaries |
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
| Predictive preset, new in this campaign | the predictive preset now also ships `allow_reversing: true`, `obstacle_yield_band_m: 0.5` and `obstacle_yield_caps_speed: true` - an obstacle-aware cruise that caps the solver's *forward speed bound* when a mover is inside the band, rather than only lowering a cruise target the obstacle term outweighs. The reactive preset is unchanged, so the two presets now differ by these three keys as well as by prediction (Section 8) |
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
| **ProxMPC** (Unicycle) | 5/5 | 12.26 ± 1.74 | 0.237 | 0.0001 ± 0.0000 | 0.0002 |
| **ProxMPC (predictive)** | 5/5 | 13.54 ± 0.89 | 0.238 | **0.0000 ± 0.0000** | 0.0000 |
| Regulated Pure Pursuit | 5/5 | 11.74 ± 0.99 | 0.235 | 0.0000 ± 0.0000 | 0.0000 |
| MPPI | 5/5 | 12.21 ± 0.63 | 0.228 | 0.0029 ± 0.0001 | 0.0041 |
| DWB | 5/5 | 12.34 ± 1.00 | 0.237 | 0.0001 ± 0.0001 | 0.0008 |
| Vector Pursuit | 5/5 | 13.15 ± 1.09 | 0.236 | 0.0000 ± 0.0000 | 0.0000 |
| Graceful | 5/5 | 11.20 ± 1.26 | 0.224 | 0.0000 ± 0.0000 | 0.0000 |

All seven track the straight line to the plant's resolution and complete 5/5.
Cross-track RMS is at or below 0.1 mm for six of them and 2.9 mm for MPPI, the one stochastic controller; every controller finishes two orders of magnitude inside the 0.25 m goal tolerance.
Time-to-goal spans 11.2-13.5 s with per-controller spreads of ±0.6-1.7 s, which is wall-clock bring-up and scheduling jitter on a 5 m traverse rather than a control effect - the spread is comparable to the gap between the fastest and slowest controller, so the ordering here carries no signal.

On *tracking fidelity* there is no meaningful separation on an empty straight line, which is exactly why the compute-and-resource comparison below is the discriminating open-cell result, and why obstacle avoidance is reported separately.

## 4. Per-cycle compute cost and process resources

This comparison is **measured, not inferred**: the timing decorator wall-clock times every controller's `computeVelocityCommands` the same way, on **every** scenario, and a per-process sampler reads the `controller_server`'s CPU and RSS from `/proc`.
Unlike tracking, compute is *not* flat across scenarios - the obstacle cells activate each method's avoidance machinery (ProxMPC's in-loop keep-out rows, DWB/MPPI's cost critics), so the per-cycle cost grows with obstacle load.
The tables are therefore **per scenario, across all eleven cells** - split into single-obstacle and multi-obstacle groups - so that growth is visible.

**ProxMPC (predictive)** is the same plugin with `predict_obstacles: true` and the obstacle tracker in the loop (Section 6.4), otherwise identical to reactive ProxMPC - so the two rows isolate the cost of prediction alone.
Its companion tracker runs as a *separate* process (0.96 % of one core, 38.3 MB RSS, constant across scenarios), which the `controller_server` figures below do **not** include; it is reported separately.

### 4.1 Per-cycle compute, `p50 / p95` [ms] (mean over repeats)

| scenario | ProxMPC | ProxMPC pred | DWB | MPPI | RPP | Graceful | VecPursuit |
|---|---|---|---|---|---|---|---|
| `nav2_open` | **0.30 / 1.02** | **0.29 / 0.60** | 2.96 / 4.41 | 3.11 / 4.05 | 0.23 / 0.36 | 0.18 / 0.32 | 0.23 / 0.38 |
| `static_box` | 1.06 / 3.41 | 0.96 / 2.88 | 3.05 / 4.22 | 3.15 / 4.22 | 0.23 / 0.40 | 0.17 / 0.29 | 0.23 / 0.37 |
| `dynamic_line_forward` | 0.92 / 4.37 | 0.82 / 4.73 | 2.96 / 4.59 | 3.06 / 4.31 | 0.22 / 0.36 | 0.17 / 0.29 | 0.23 / 0.39 |
| `dynamic_line_backward` | 1.05 / 4.22 | 0.87 / 4.22 | 2.84 / 4.16 | 3.07 / 4.27 | 0.22 / 0.40 | 0.17 / 0.30 | 0.23 / 0.36 |
| `dynamic_circle` | 1.72 / 6.42 | 1.59 / 5.20 | 3.14 / 4.76 | 3.07 / 4.09 | 0.24 / 0.40 | 0.18 / 0.29 | 0.23 / 0.37 |
| `dynamic_multi` | 2.39 / 7.32 | 1.75 / 5.39 | 3.09 / 4.45 | 2.99 / 4.06 | 0.23 / 0.40 | 0.17 / 0.28 | 0.23 / 0.40 |
| `dynamic_multi_noise` | 2.47 / 7.23 | 1.70 / 5.70 | 3.16 / 4.85 | 3.05 / 4.22 | 0.24 / 0.40 | 0.18 / 0.31 | 0.23 / 0.38 |
| `blind_multi_0` | 2.73 / 8.66 | 2.06 / 6.61 | 3.06 / 4.51 | 3.09 / 4.30 | 0.23 / 0.38 | 0.17 / 0.31 | 0.21 / 0.36 |
| `blind_multi_1` | 2.56 / 7.32 | 1.34 / 5.74 | 3.12 / 5.04 | 3.07 / 4.09 | 0.23 / 0.38 | 0.18 / 0.29 | 0.23 / 0.39 |
| `blind_multi_2` | 2.34 / 11.40 | 1.30 / 4.49 | 3.05 / 4.78 | 3.03 / 4.11 | 0.23 / 0.38 | 0.17 / 0.29 | 0.23 / 0.39 |
| `blind_multi_3` | 2.23 / 5.85 | 2.15 / 5.64 | 3.14 / 5.03 | 3.03 / 4.13 | 0.23 / 0.39 | 0.17 / 0.29 | 0.22 / 0.35 |

The shape of this table is the result, not any single row.
**ProxMPC's cost scales with obstacle load; the sampling controllers' does not.**
DWB and MPPI pay a near-constant 2.8-3.2 ms whether the cell is empty or holds two movers, because their sample counts are fixed. ProxMPC pays 0.30 ms on the empty cell and 2.2-2.7 ms on the hardest, because the QP grows a keep-out row per obstacle slot per node.

So the advantage is largest exactly where a robot spends most of its time - **9.9x lighter than DWB and 10.4x than MPPI on the open cell** - narrows to ~2.5-3x on single-obstacle cells, and converges to ~1.3x on the multi-mover cells. It never inverts: ProxMPC's median stays at or below both on every cell measured.
The predictive preset is *cheaper* than the reactive one on every obstacle cell (1.3-2.2 ms against 2.2-2.7 on the multi cells) because a tracked obstacle occupies one slot with a known trajectory, while the costmap fill spends slots on clusters it must re-rank each cycle.

The one outlier is reactive ProxMPC's 11.40 ms p95 on `blind_multi_2`, against 6-9 ms elsewhere; that cell is where the reactive keep-out struggles most (it collides 5/5 there, Section 6.4), and the extra QP iterations show up in the tail rather than the median (2.34 ms). The predictive preset, which clears that same cell 0/5, pays only 4.49 ms at p95 on it.

### 4.2 Process CPU [% of one core] / RSS peak [MB]

| scenario | ProxMPC | ProxMPC pred | DWB | MPPI | RPP | Graceful | VecPursuit |
|---|---|---|---|---|---|---|---|
| `nav2_open` | 4.6 / 59 | 4.7 / 60 | 9.0 / 60 | 9.1 / 63 | 3.8 / 55 | 3.8 / 56 | 3.9 / 55 |
| `static_box` | 6.3 / 59 | 6.4 / 60 | 9.4 / 60 | 9.3 / 63 | 4.0 / 55 | 4.0 / 56 | 4.1 / 55 |
| `dynamic_line_forward` | 6.3 / 59 | 6.3 / 60 | 9.3 / 60 | 9.3 / 63 | 4.1 / 55 | 3.9 / 56 | 4.0 / 55 |
| `dynamic_line_backward` | 6.5 / 59 | 6.4 / 60 | 8.9 / 60 | 9.2 / 63 | 4.0 / 55 | 3.9 / 56 | 4.0 / 55 |
| `dynamic_circle` | 7.9 / 59 | 7.5 / 60 | 9.5 / 60 | 9.4 / 63 | 4.0 / 55 | 4.1 / 56 | 3.9 / 55 |
| `dynamic_multi` | 9.1 / 59 | 8.1 / 60 | 9.5 / 60 | 9.3 / 63 | 4.1 / 55 | 3.9 / 56 | 4.0 / 55 |
| `dynamic_multi_noise` | 9.2 / 59 | 7.9 / 60 | 9.7 / 60 | 9.4 / 63 | 4.1 / 55 | 4.0 / 56 | 3.9 / 55 |
| `blind_multi_0` | 10.4 / 59 | 9.0 / 60 | 9.6 / 60 | 9.5 / 63 | 4.1 / 55 | 4.1 / 56 | 3.9 / 55 |
| `blind_multi_1` | 9.5 / 59 | 7.7 / 60 | 9.5 / 60 | 9.3 / 63 | 4.0 / 55 | 4.0 / 56 | 3.9 / 55 |
| `blind_multi_2` | 11.1 / 59 | 7.3 / 60 | 9.4 / 60 | 9.4 / 64 | 4.0 / 55 | 3.9 / 56 | 4.0 / 55 |
| `blind_multi_3` | 8.6 / 59 | 8.4 / 60 | 9.7 / 60 | 9.3 / 63 | 4.1 / 55 | 4.0 / 56 | 4.0 / 55 |

The same shape appears in process CPU: ProxMPC starts at 4.6 % on the empty cell against DWB's 9.0 % and MPPI's 9.1 %, and rises to 7.3-11.1 % on the hardest cells where they stay flat. Reactive ProxMPC is the only controller that exceeds the sampling pair, and only on the two cells its keep-out finds hardest (`blind_multi_0` at 10.4 %, `blind_multi_2` at 11.1 %); the predictive preset stays at or below both everywhere (7.5 % averaged over all obstacle cells, against reactive's 8.5 %, DWB's 9.4 % and MPPI's 9.3 %).

Memory is flat and unremarkable for every controller: 59-60 MB peak RSS for ProxMPC against 60 for DWB and 63-64 for MPPI, with the three pursuit controllers at 55-56. Nothing here grows over a run.
The predictive preset's companion tracker, sampled as a separate process across all 55 predictive runs, costs **0.96 % of one core and 38.3 MB RSS** on top of the `controller_server` figures above.

## 5. ProxMPC solver profile (and a cross-check)

Solver telemetry over all 10 obstacle cells (50 runs per preset), from the opt-in `SolverDiagnostics` topic:

| | solve p50 | solve p95 | solve max | over 50 ms | deadline-miss | SQP iters | QP ext iters | infeasible runs | peak slack |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| reactive | 1.82 ms | 6.49 ms | 33.5 ms | **0 / 50** | 0.00 % | 1.00 | 7.5 | 9 / 50 | 0.553 m |
| predictive | 1.31 ms | 4.91 ms | **31.1 ms** | **0 / 50** | 0.00 % | 1.00 | 6.9 | 3 / 50 | 0.766 m |

**Every cycle in 100 runs stayed inside the 50 ms budget.** This is the single largest behavioural change in this release, and it came from two defects rather than from tuning:

1. `max_iter_sqp` defaulted to 100. On a failed solve the loop re-linearised up to 99 more times around an iterate it had already corrupted, then braked anyway. Worst-case cycles of 300 ms were measured on three separate cells - six times the budget, and 15 cm of open-loop travel at 0.5 m/s. The default is now 1, which is what the real-time iteration scheme this controller implements actually prescribes.
2. The QP was re-initialised from scratch every cycle. `guess: true` named a warm start that never happened, because `init()` rebuilt the workspace and discarded the previous primal/dual iterate 20 times a second. The workspace is now updated in place, which cut the worst case by a further ~40 %.

`SQP iters` sitting at 1.00 is the fixed real-time iteration, not a convergence report; `QP ext iters` at 6.9-7.5 against a 10 000 cap shows the QP itself converges quickly when it converges at all.

**One thing this table does not explain.** The reactive preset reports `PROXQP_PRIMAL_INFEASIBLE` on 9 of 50 runs (roughly 0.1-0.3 % of cycles within them), where the published `v1.0.0`-era campaign reported none. The QP's feasible set is provably non-empty - the constant sequence `u(k) = u_prev` satisfies the control box, the rate chain and the row-0 rate anchor simultaneously, and the obstacle rows are fully soft - so every such report is a false certificate. Three candidate causes were tested and eliminated by measurement: a warm start that violated its own rate chain, the absent solver warm start, and large-magnitude bounds on unused obstacle slots. The cause is still unknown.

It is benign in every way the harness can measure: the controller brakes for that cycle, the next one solves, worst-case latency stays at 33.5 ms, and no infeasible cycle has been traced to a collision. It is recorded here because it is real and unexplained, not because it is known to matter.

The predictive preset's peak keep-out slack is now the higher of the two (0.766 m against the reactive 0.553 m), which is the yield behaving as designed rather than a safety regression: the predictive path relaxes the *inflated soft margin* while it slows and lets a mover pass, and it is simultaneously the only preset that never touches the physical keep-out on a single-mover cell (Section 6.2, worst gap +0.212 m against reactive's +0.186 m). Peak slack measures how far the soft constraint was relaxed, not how close the robot came.

## 6. Obstacle avoidance

### 6.1 Static obstacle (`static_box`, mode b2)

| Controller | success | collisions | min gap [m] | mean gap [m] | cross-track max [m] |
| --- | --- | --- | --- | --- | --- |
| **ProxMPC** (reactive) | 5/5 | 0/5 | **+0.382** | **+0.382** | 0.737 |
| **ProxMPC (predictive)** | 5/5 | 0/5 | +0.297 | +0.298 | 0.650 |
| RPP | 5/5 | 0/5 | +0.211 | +0.213 | 0.569 |
| MPPI | 10/10 | 0/10 | +0.204 | +0.208 | 0.588 |
| Graceful | 5/5 | 0/5 | +0.195 | +0.211 | 0.574 |
| Vector Pursuit | **1/5** | 0/5 | +0.134 | +0.247 | 0.551 |
| DWB | 5/5 | 0/5 | +0.069 | +0.090 | 0.455 |

Every controller that reaches the goal clears the static box without contact - an obstacle-routed global plan does most of this work, as Section 2.1 notes.
The separation is in *margin*: reactive ProxMPC keeps +0.382 m at its closest against DWB's +0.069 m, and it pays for that with the largest detour (0.737 m cross-track against 0.455). That trade is the in-loop keep-out doing exactly what it is asked to: `d_safe` is a hard-ish constraint the optimiser plans around, not a cost it trades away.

**Vector Pursuit stalls on this cell.** It reached the goal on only 1 of 5 runs, against 3 of 5 in the previous campaign; the four failures are the same behaviour recorded there - it stops behind the box and does not recover within the 60 s timeout. Its gap figures are computed over all five runs and therefore describe a controller that mostly stopped short, not one that passed the box with margin. This is a peer's own behaviour on stock binaries at an unchanged version, not an effect of anything in this release.

### 6.2 Single moving obstacles (3 cells, mode b2)

| Controller | collisions | min gap [m] | mean gap [m] | cross-track max [m] |
| --- | --- | --- | --- | --- |
| **ProxMPC (predictive)** | **0/15** | **+0.212** | +0.346 | 0.376 |
| **ProxMPC (reactive)** | **0/15** | +0.186 | +0.398 | 0.604 |
| MPPI | 0/30 | +0.136 | **+0.456** | 0.283 |
| Vector Pursuit | 0/15 | +0.044 | +0.357 | 0.263 |
| RPP | 2/15 | -0.015 | +0.286 | 0.208 |
| Graceful | 5/15 | -0.065 | +0.206 | 0.103 |
| DWB | 5/15 | -0.345 | +0.238 | 0.113 |

Four controllers clear all three single-mover cells collision-free - both ProxMPC presets, MPPI and Vector Pursuit; DWB and Graceful collide on 5 of 15 each and RPP on 2.
**The predictive preset now holds the largest worst-case margin in the field at +0.212 m** - no run in fifteen came closer than 21 cm - having overtaken the reactive preset (+0.186 m) on this measure for the first time.

That reordering is the obstacle-aware speed cap doing its job. In the previous campaign the predictive preset's worst case was +0.064 m, the second *worst* in the field: it had the right prediction but no way to act on it, because the yield only lowered a cruise target that the keep-out term outweighed, so the solver swerved past a closing mover at full speed instead of slowing for it. Capping the solver's forward speed bound instead of its cruise reference raised that worst case by 15 cm, and it did so while *shrinking* the detour - cross-track max falls from the reactive preset's 0.604 m to 0.376 m. The robot now yields by slowing rather than by swerving, which is both safer and less disruptive to the path.

A single moving obstacle is where the costmap-only fill is still sufficient: the obstacle's position lag costs the controller reaction time, but the free corridor beside it is wide enough to absorb a late swerve. That stops being true with two movers, which is Section 6.4.

### 6.3 Ground-truth predictive confirmation (mode b1)

> **Re-run in this campaign.** The 40-run mode-(b1) matrix was recorded in the same session as the b2 matrix above, on the same binaries and the same host, so its solve times are directly comparable with Section 5 for the first time.

One controller-vs-tracker separation is worth recording: fed the *correct* future obstacle positions (mode b1, the engine knows the exact trajectory, orbit included), ProxMPC reaches the goal on every single moving-obstacle case for both vehicle models.
This section was re-run when reactive ProxMPC was still colliding 3/5 on the orbit, as a ceiling check on the formulation.
Section 6.2 has since resolved that failure at the reactive level, so this table is no longer answering an open question about the orbit - but it is retained because it is the only ground-truth-predictor result in the document and it bounds the formulation independently of what the costmap supplies:

| Scenario | Model | success | cross-track RMS [m] | time-to-goal [s] | peak keep-out slack [m] |
| --- | --- | --- | --- | --- | --- |
| dynamic_circle | Unicycle / Bicycle | 5/5 / 5/5 | 0.007 / 0.007 | 4.56 / 4.58 | 0.000 / 0.000 |
| dynamic_line_forward | Unicycle / Bicycle | 5/5 / 5/5 | 0.045 / 0.058 | 4.56 / 4.58 | 0.222 / 0.375 |
| dynamic_line_backward | Unicycle / Bicycle | 5/5 / 5/5 | 0.048 / 0.044 | 4.54 / 4.56 | 0.194 / 0.225 |
| static_box | Unicycle / Bicycle | 5/5 / 5/5 | 0.000 / 0.000 | 4.49 / 4.56 | 2.945 / 2.945 [^slack] |

> This table is a fresh 40-run campaign (4 scenarios x 2 models x 5 repeats, controller `proxmpc`), recorded in the same session as the b2 matrix.
> It is a ceiling check on the controller's own capability given a correct predictor, not the operative reactive/predictive result - that comparison is mode b2, in Sections 6.1-6.4.

[^slack]: This is the one-cycle initialisation transient, not a steady-state figure: in this campaign every run of both models sampled it, where the previous campaign caught it on only 3 of 5 bicycle runs and none of the unicycle runs. The steady-state peak is 0.490 m. That the transient now appears uniformly is what the sampling account below predicts - whether it is recorded depends only on the metrics node's subscription matching in time, not on the model. See Section 6.3.

**The 30/30 ceiling on the three single moving-obstacle cells still holds on the current tree**, for both a 3-state unicycle and a 4-state bicycle, driven by the same plugin with one parameter changed (`model_plugin`); with the static box included the campaign is 40/40.
The orbit is the sharpest of the four results: given the correct future positions the keep-out is **never relaxed at all** (peak slack 0.000 m for both models), so the predictive path clears the orbiting obstacle outright rather than trading margin for progress.
The single-obstacle dynamic case is therefore fully within the controller's reach given a correct predictor - and Section 6.2 now shows it is also within reach *without* one, once the keep-out carries enough margin to absorb the obstacle's motion between costmap updates.
The earlier reading of the reactive orbit failure as a limit of what the costmap supplies does not survive that result: the costmap was supplying enough, and the constraint built from it was simply sized too tightly.

The slack column has to be read together with the success column, because "reaching the goal" in mode b1 is not by itself an avoidance result.
Mode b1 has no global planner: the reference is the straight start-to-goal line, so on `static_box` the cross-track RMS is exactly 0.000 m and the box is passed by relaxing the soft keep-out (0.490 m on the unicycle, essentially the whole 0.45 m clearance) rather than by steering around it.
The avoidance comparison is the mode-b2 one in Section 6.1; this section measures whether the solver keeps finding a feasible, goal-reaching trajectory when the prediction is perfect.

Two observations about this table are worth recording, neither of them attributed to a specific change in the intervening work:

- **The `static_box` peak-slack figure is a first-cycle initialisation transient, and its run-to-run spread is a sampling artefact rather than a solver result.** In this campaign all ten runs reported 2.945 m against the `max_obstacle_slack_m: {max: 0.5}` bound in [`prox_mpc_benchmark/config/metrics.yaml`](../prox_mpc_benchmark/config/metrics.yaml); in the previous one, three of five bicycle runs reported 2.945 m and the other two 0.490 m, with all five unicycle runs at 0.490 m. Capturing `/prox_mpc/diagnostics` per cycle resolves both numbers: the 2.945 m relaxation occurs on control cycle 1 alone and is back to 0.000 m by cycle 2, while 0.490 m is the true steady-state peak, reached at cycle 13 as the robot passes the box. The transient is deterministic and identical for both models - unicycle and bicycle agree to six decimals on the cycle-1 slack and on its objective - so it is not a bicycle-specific effect; what varies between runs is only whether the metrics node's subscription matched in time to receive cycle 1. **This campaign is the direct confirmation of that account**: the transient is now recorded on every run of both models, which is what a subscription race predicts and what a model-dependent or solver-dependent effect does not. The mechanism is that [`prox_mpc_core/src/mpc.cpp`](../prox_mpc_core/src/mpc.cpp) zero-initialises the whole predicted trajectory, so before the first solve every horizon node sits at the origin - which on `static_box`, and only on `static_box`, is exactly where the obstacle is, inside its 1.45 m keep-out and at the point where the distance gradient is undefined. `dynamic_circle`, whose mover is 1.8 m from the origin at `t = 0`, reports 0.000 m throughout, as this account predicts. Nothing in the executed trajectory is affected: cross-track RMS is exactly 0.000 m, and time-to-goal and goal error are unchanged from the runs that did not sample the transient.
- **Mode b1 remains far inside the real-time budget**: worst cell p95 16.3 ms, worst single cycle 36.4 ms across all 40 runs, at 0 % deadline-miss and 0 % infeasible, with 40/40 success. Mode b1 touches neither the costmap nor the Nav2 stack, so the per-cycle fixes credited in Section 4 do not apply to this path and its absolute solve times are not comparable with the b2 figures in Section 5 despite sharing a session.

### 6.4 Multiple simultaneous moving obstacles - where the field separates

Six cells with two simultaneous movers. Four of them (`blind_multi_0..3`) are *blind*: their geometry comes from a seeded RNG with no controller run to screen it, so no cell was kept or discarded because of how any controller performed on it.

| Controller | `dyn_multi` | `dyn_multi_noise` | `bm0` | `bm1` | `bm2` | `bm3` | total |
|---|---|---|---|---|---|---|---|
| **ProxMPC (predictive)** | 0/5 | 0/5 | 3/5 | 0/5 | 0/5 | 0/5 | **3/30 (10 %)** |
| RPP | 0/5 | 0/5 | 1/5 | 0/5 | 5/5 | 0/5 | 6/30 (20 %) |
| MPPI | 0/10 | 1/10 | 8/10 | 3/10 | 0/10 | 6/10 | 18/60 (30 %) |
| DWB | 2/5 | 2/5 | 1/5 | 0/5 | 5/5 | 0/5 | 10/30 (33 %) |
| Graceful | 4/5 | 3/5 | 1/5 | 0/5 | 5/5 | 0/5 | 13/30 (43 %) |
| Vector Pursuit | 1/5 | 0/5 | 5/5 | 0/5 | 5/5 | 2/5 | 13/30 (43 %) |
| ProxMPC (reactive) | 0/5 | 1/5 | 4/5 | 1/5 | 5/5 | 5/5 | 16/30 (53 %) |

**Predictive ProxMPC leads by a factor of two**: 3 collisions in 30 against 6 for the next best.
Its remaining failures are now concentrated in a single cell - `blind_multi_0`, 3/5 - and it is **collision-free on the other five**, including `blind_multi_2`, which four of the seven controllers fail 5 times out of 5.

Because these cells are deliberately marginal, the collision count alone overstates its own precision. The median closest approach is the steadier statistic:

| Controller | median closest approach [m] | runs within 0.15 m of the threshold |
| --- | --- | --- |
| **ProxMPC (predictive)** | **+0.151** | 15/30 (50 %) |
| RPP | +0.113 | 21/30 (70 %) |
| MPPI | +0.063 | 46/60 (77 %) |
| DWB | +0.057 | 20/30 (67 %) |
| Vector Pursuit | +0.020 | 24/30 (80 %) |
| Graceful | +0.000 | 28/30 (93 %) |
| ProxMPC (reactive) | -0.023 | 28/30 (93 %) |

Predictive ProxMPC leads on both measures and spends the least time near the threshold of any controller in the field. Reactive ProxMPC has a *negative* median - the typical run ends overlapping - and it and Graceful sit within 0.15 m of contact on 93 % of runs.

**Why the reactive preset is in that lower group, and why it got worse.**
The costmap-only fill scans a single present-time costmap around every predicted node, so a moving obstacle is constrained where it currently is rather than where it will be - an error of `v_obs * t_node`, up to a metre at the far end of a 2 s horizon. With one mover the free corridor absorbs the resulting late reaction (Section 6.2, 0/15). With two, that corridor closes.

The reactive rate is 53 % here against 37 % for the same preset in the previous campaign, and the cause is a deliberate change rather than a regression in the avoidance logic. Previously `allow_reversing: false` filtered only which plan pose the reference tracked; the solver's control box still admitted reverse, so the robot could back out of a closing gap. That was a defect - the parameter did not do what it said - and fixing it removed an escape the costmap-only path was relying on. Measured separately during that work, restoring reverse and changing nothing else recovered the rate to 11/30 and lifted the mean margin from -0.005 m to +0.142 m, with path length on `blind_multi_1` rising from 5.2 m to 7.0 m: the robot reversing and re-approaching, not detouring.

The reactive preset's shipped default stays forward-only, because a library cannot check whether an integrator's platform senses the reverse direction. `allow_reversing: true` with an explicit `model_params.v_min` is supported and measurably better on this path; **the predictive preset now enables it by default**, because the obstacle-aware speed cap it also ships needs somewhere to go - a robot that must slow for a mover but cannot back off can be boxed in by a second one (Section 2.1). The bundled Gazebo demo enables it for the same reason: the waffle's 360-degree scanner covers the rear and Nav2's Collision Monitor, already in the demo's `cmd_vel` chain, projects the footprint along the commanded twist including reverse.

**What this section does not claim.** `blind_multi_2` is 5/5 for four of the seven controllers and 0/5 for predictive ProxMPC and 0/10 for MPPI; `blind_multi_0` is 5/5 for Vector Pursuit, 4/5 for reactive ProxMPC and 1/5 for three others. Individual cells swing hard, and the same stock peer binaries moved by up to 8 points between this campaign and the last on code that did not change. The totals over 30 runs are the comparison; the per-cell columns are shown for completeness, not for ranking. In particular, the gap between RPP at 6/30 and DWB at 10/30 is not resolved by this sample, while the gap between predictive ProxMPC at 3/30 and the rest of the field is larger than that noise band.

## 7. Real-stack validation (Gazebo)

To confirm the plugin behaves under the full production stack and not only against a kinematic plant, ProxMPC also runs in mode (a): Gazebo Harmonic physics, a TurtleBot3 waffle, AMCL localization, costmaps, and the complete Nav2 velocity chain, headless.
The cell is the demo's open-room traverse - spawn `(-2.0, -0.5)`, goal `(2.0, -0.5)`, about 4 m - driven by the shipped [`nav2_prox_mpc.yaml`](../prox_mpc_demo/config/nav2_prox_mpc.yaml) with only `publish_diagnostics` turned on, which is opt-in telemetry that publishes when subscribed and does not touch control.

| Measure | Mode (a), 5 runs | Mode (b2) open cell, same controller |
| --- | --- | --- |
| Goal reached | **5/5**, 0 recoveries | 5/5 |
| Cross-track RMS | **5.76 ± 1.23 mm** | 0.1 mm |
| Cross-track max | 16.7 ± 4.5 mm | - |
| Solve p50 | 0.364 ± 0.036 ms | 0.189 ± 0.016 ms |
| Solve p95 | 1.382 ± 0.199 ms | 0.910 ± 0.082 ms |
| Solve max (worst run) | 11.85 ms | 3.63 ms |
| Cycles over the 50 ms budget | **0** | 0 |
| Deadline-miss / infeasible | **0 % / 0 %** | 0 % / 0 % |
| Mean SQP iterations | 1.00 | 1.00 |
| QP status histogram | **966/966 `PROXQP_SOLVED`** | - |

Three things this establishes. The **real-time iteration holds under real physics**: mean SQP iterations is exactly 1.00 across 966 control cycles, and not one exceeded the 50 ms budget. **Tracking survives real sensing**: 5.8 mm RMS against AMCL localization and a simulated lidar, where the kinematic plant tracks to a tenth of a millimetre against ground truth - the gap is what localization and physics cost, not controller error. And the **`PROXQP_PRIMAL_INFEASIBLE` reports discussed in Section 5 do not appear here at all**: every one of the 966 cycles returned `PROXQP_SOLVED`.

Per-cycle compute is about 1.9x the mode-(b2) median for the same controller, and 1.5x at p95. That is expected rather than a regression: mode (a) runs Gazebo physics, the sensor pipeline and AMCL on the same host as the controller, while mode (b2) drives a kinematic plant. The absolute numbers stay far inside budget either way.

> **Not a controlled comparison with the `v1.0.0` figures this section used to carry** (5.6 mm RMS, 0.438 ms p95). Those were measured on a different controller configuration - `max_iter_sqp: 100`, `w_weight: 100`, `safety_margin: 0.1`, no reverse travel - as well as different code, two months earlier on a thermally variable laptop. The tracking figure is unchanged within its spread; the solve-time difference is **not attributed** here, because nothing in these two runs isolates a cause.

Mode (a) remains a single open-cell gate on one machine: it answers "does the plugin behave under the production stack", not "how does it compare", which is what modes (b1) and (b2) are for. The `2.0.0` reverse-travel and obstacle-yield behaviour was measured in Gazebo separately, and those runs are reported in [`prox_mpc_demo/doc/nav2-simulation.md`](../prox_mpc_demo/doc/nav2-simulation.md).

## 8. Threats to validity

- **Every row comes from one campaign on one machine state** (see Provenance), which removes the cross-session caveat the previous publication carried. What it does not remove is run-to-run variance: see the two bullets below.
- **These collision rates are more sensitive to the keep-out than to the algorithm.** Over this work the predictive all-cells rate moved 6 % -> 10 % -> 6 % on a 15 mm change to `d_safe`, with no algorithmic change at all. Any comparison between controllers separated by less than that is not resolved by this campaign.
- **Peer controllers moved between campaigns on code that did not change.** Re-running the identical stock binaries moved MPPI's all-cells rate 26 % -> 18 %, Graceful's 40 % -> 36 %, RPP's 20 % -> 16 %, DWB's 26 % -> 30 % and Vector Pursuit's 22 % -> 26 %. That sets the noise floor at about 8 points on 50 runs, which is the scale at which the totals in Section 6.4 should be read - and the scale the predictive preset's 6 % has to clear to mean anything, which it does.
- **Vector Pursuit's static-box result is a stall, not an avoidance measurement.** It reached the goal on 1 of 5 runs here against 3 of 5 previously (Section 6.1). Its margin figures on that cell average a controller that mostly stopped short.
- **The two ProxMPC presets do not isolate prediction, and now differ by more than before.** Both share `d_safe = 0.320 m`, but `costmap_cost_threshold` (200 reactive against 253 predictive), `w_weight`, and - new in this campaign - `allow_reversing`, `obstacle_yield_band_m` and `obstacle_yield_caps_speed` all differ. The reactive-versus-predictive comparisons are configuration-versus-configuration, and the gap between them is now attributable to the yield and reverse defaults at least as much as to prediction. The multi-mover direction is large enough (3/30 against 16/30) that the combination clearly works; which of its parts carries how much is not separated here. The compute comparison in Section 4 stays confounded for the same reason.
- **The obstacle-routed global plan does much of the single-obstacle avoidance.** The shared global `obstacle_layer` makes NavFn route around obstacles for every controller - deliberately equal, but it favours the geometric path-followers and DWB, which track that detour closely. The single-obstacle cells therefore separate the field on margin, not on collision counts; the multi-mover cells carry the collision comparison.
- **The multi-obstacle field is close and the sample is small.** 50-93 % of runs across the six cells finish within 0.15 m of contact, so per-cell counts swing by more than the gap between most controllers. The 30-run totals are the comparison.
- **Collision uses a strict min-gap:** any instant of disc overlap over a whole run counts, so a brief graze scores the same as a harder hit. The reported min gap distinguishes them.
- **Every figure comes from one machine.** The whole document was measured on the x86-64 dev host named in Section 2 and on no other, so the absolute numbers are that host's; what transfers to a different machine is the ranking, not the values. The per-cycle compute in Section 4 is the cleaner controller-only measure, since process CPU/RSS includes the shared costmap.
- **The pure-geometric controllers are genuinely lighter.** ProxMPC is the cheapest of the controllers that solve a constrained optimisation each cycle, not the cheapest outright.
- **MPPI is stochastic with no exposed seed** in Nav2 Jazzy, so its per-scenario variance is real. It runs 10 repeats per obstacle cell against the others' 5, which characterises that variance without pinning it down.
- **Determinism vs. physics.** Modes (b1)/(b2) are deterministic plants, so their near-zero geometric std is reproducibility, not a noise estimate. Mode (a) carries real Gazebo variance, which is why its five runs are reported with a spread (Section 7). `dynamic_multi_noise` adds a Gaussian range-noise model on the scan.
- **Four of 435 matrix cells failed to bring up Nav2 and were discarded and re-run**, detected by the controller never receiving a path rather than by their results (see Provenance). All 435 records in the final set show a real path delivered. Five further runs failed behaviourally and are reported as failures, not re-run.
- **An unexplained solver report remains open.** Reactive ProxMPC emits `PROXQP_PRIMAL_INFEASIBLE` on 9 of 50 runs against none in the `v1.0.0`-era campaign, on a QP whose feasible set is provably non-empty (Section 5). Three candidate causes were tested and eliminated. It has no measured consequence, but it is not understood.
- **`blind_multi_0` is the predictive preset's one remaining weak cell** (3/5, against 0/5 on the other five multi cells). It is not diagnosed here.

## 9. Conclusion on ProxMPC

On a like-for-like, fairly-tuned suite recorded as a single campaign, **predictive ProxMPC is the most reliable obstacle avoider in the field** - 3 collisions in 50 obstacle runs (6 %) against 8 for the next best (RPP, 16 %) - while computing its command 3-10x faster than the two controllers in its own cost class. Its worst-case cycle is bounded well inside the real-time budget, which was not true of the previous release.

- **Real-time behaviour:** **zero cycles above the 50 ms budget across 100 runs**, both presets, with worst observed cycles of 33.5 ms (reactive) and 31.1 ms (predictive). The previous release measured 300 ms worst cases on three separate cells. Two defects caused that and both are fixed: an SQP loop that re-linearised up to 99 times around a corrupted iterate after a failed solve, and a QP workspace rebuilt from scratch every cycle instead of warm-started.
- **Per-cycle compute:** 0.30 ms median on the open cell - **9.9x lighter than DWB and 10.4x than MPPI** - narrowing to ~2.5-3x on single-obstacle cells and ~1.3x on the hardest multi-mover cells, because ProxMPC's cost scales with obstacle load while theirs does not. It never inverts. Process CPU is 8.5 % reactive and 7.5 % predictive across all obstacle cells against DWB's 9.4 % and MPPI's 9.3 %; peak RSS 59/60 MB against 60 and 63. The predictive preset's tracker adds 0.96 % of a core and 38.3 MB in its own process. The three geometric pursuit controllers remain an order of magnitude cheaper and collide 16-36 % of the time.
- **Tracking:** 0.0001 m cross-track RMS on the open cell, 5/5, indistinguishable from the field. An empty straight line does not separate these controllers.
- **Static obstacle:** reactive ProxMPC keeps **the largest margin in the field, +0.382 m at its closest**, against RPP's +0.211 and DWB's +0.069, and pays for it with the largest detour (0.737 m cross-track). That is the in-loop keep-out behaving as specified - `d_safe` is planned around, not traded away.
- **Single moving obstacles:** both ProxMPC presets, MPPI and Vector Pursuit clear all three cells collision-free, and **the predictive preset now holds the field's largest worst-case margin at +0.212 m** while cutting its detour to 0.376 m cross-track - it yields by slowing rather than swerving. DWB and Graceful collide 5/15 each.
- **Simultaneous two-mover cells - where the field separates:** predictive ProxMPC leads on both measures, at **3/30 collisions** against 6 for the next best, and **+0.151 m median closest approach** against RPP's +0.113, MPPI's +0.063 and a negative median for reactive ProxMPC. It is collision-free on five of the six cells.
- **The costmap-only path is a single-obstacle configuration.** Reactive ProxMPC sits at 16/30 on these cells because the fill constrains each obstacle where it *was*, not where it will be - up to a metre of error at the far end of a 2 s horizon. With one mover the free corridor absorbs that; with two it does not. This is why `predict_obstacles` now defaults to true, and why the reactive preset should be treated as validated for single-obstacle environments only.
- **One controller, many vehicles:** the identical plugin drives a unicycle and a bicycle by configuration alone, confirmed 40/40 in mode b1 (Section 6.3).

**What this release changed, honestly.** Two things changed, and only one of them is an avoidance result.

The first is real-time behaviour: the controller can no longer stall for 300 ms mid-manoeuvre, which on a robot moving at 0.5 m/s is 15 cm of open-loop travel at exactly the wrong moment.

The second is that **the predictive preset's avoidance genuinely improved, and this campaign is the first to measure it against a re-run field.** Its all-cells collision rate went 16 % -> 6 % and its multi-mover rate 30 % -> 10 %, while its worst-case single-mover margin went +0.064 m -> +0.212 m. Peer binaries that did not change moved by up to 8 points over the same interval, so the peer noise floor is about 8 points on 50 runs; the predictive preset's 10-point all-cells move sits outside it, its 20-point multi-mover move well outside it, and the margin improvement is not a count statistic at all. The cause is identified and was measured in isolation (Section 6.2): the previous yield only lowered a cruise *target* that the keep-out term outweighed, so the controller had a correct prediction and no way to act on it; capping the solver's forward speed *bound* is what converted the prediction into behaviour.

The reactive preset moved the other way, 37 % -> 53 % on the multi-mover cells. Part of that is the deliberate reverse fix (Section 6.4) and part is within the 8-point noise band applied twice; it is reported as measured, and it is the reason the reactive preset is documented as a single-obstacle configuration.

**What remains open:** every figure comes from the single x86-64 host in Section 2, Gazebo validation is a single open-cell gate rather than a comparison, the reactive infeasibility report in Section 5 is unexplained, and `blind_multi_0` remains the predictive preset's one weak cell at 3/5. None of these affect the comparisons above; all four bound what can be claimed beyond them.

---

*Reproduce:* the open cell and the mode-(b2) reactive obstacle scenarios with `ros2 run prox_mpc_benchmark run_nav2.py --scenario <nav2_open|static_box|dynamic_circle|dynamic_line_forward|dynamic_line_backward|dynamic_multi|dynamic_multi_noise|blind_multi_0|blind_multi_1|blind_multi_2|blind_multi_3>`, the **predictive b2** runs (real tracker) by adding `--controllers proxmpc_pred` to the same command, the mode-(b1) ground-truth predictive results with `ros2 run prox_mpc_benchmark run_matrix.py --modes b1`, and the tables with `ros2 run prox_mpc_benchmark aggregate.py`. See [`prox_mpc_benchmark/README.md`](../prox_mpc_benchmark/README.md).

## License

[Apache-2.0](../LICENSE).
