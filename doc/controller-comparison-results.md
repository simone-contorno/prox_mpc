# Controller Comparison Results

This document reports how the ProxMPC controller compares against the stock Nav2 controllers under identical, reproducible, and *fairly-tuned* conditions - on **path tracking**, **per-cycle compute and process resources**, and **obstacle avoidance**.
It is the narrative companion to the auto-generated tables in [`prox_mpc_benchmark/README.md`](../prox_mpc_benchmark/README.md); the harness that produced every number is described in [the package guide](prox-mpc.md#8-prox_mpc_benchmark---the-measurement-harness).

**Provenance: which rows came from which measurement session.**
The mode-(b2) tables below are a *merged* set, and the split is stated wherever a number depends on it.

- **Reactive ProxMPC** rows were measured on 2026-08-27 at commit `40d1e61` (branch `test/audit-benchmark-revalidation`), after the controller default `safety_margin` was raised from 0.1 m to 0.2 m - 11 scenarios x 5 repeats = 55 runs, from `results/mode_b2_margin020_20260827_074543/`.
- **DWB, MPPI, RPP, Graceful, Vector Pursuit, and ProxMPC (predictive)** rows carry over unchanged from the 2026-08-26 campaign at commit `988db0a` (branch `chore/audit-warning-hardening`), 380 runs, on the same host. None of those presets reads the changed default - the predictive preset pins `safety_margin: 0.1` explicitly - and the *physical* obstacle geometry the harness draws is computed from a fixed harness constant, so it is byte-identical across both sessions. They were deliberately not re-run.
- **Mode (b1)**, the ground-truth predictive results in Section 6.3, is a separate 40-run campaign at commit `1e170ad` on the same host. It drives the engine plant rather than the Nav2 stack and does not read the changed default, so it is unaffected.
- **Mode (a)**, the Gazebo results in Section 7, was **not** re-run and still reflects the `v1.0.0` baseline; it is called out again where it appears.

Commit `988db0a` is the tip of three change waves plus review fixes, an obstacle-mapping generalization, and warning hardening landed after the `v1.0.0` baseline this document previously reported - among them the endpoint-veto/unknown-space handling, the goal-approach taper, the obstacle-slot binding and ranking, the plan-projection method, the control-space brake, and two per-cycle performance fixes.
That work is one bundled set of simultaneous behaviour changes, so a shift between `v1.0.0` and `988db0a` is attributed to a specific commit only where the mechanism is traceable in the diff.
The `988db0a` -> `40d1e61` step is the opposite case: it is a **single-parameter change** on the reactive preset, so shifts in the ProxMPC rows below are attributed to it directly, and comparisons against the other six controllers remain valid because their rows and the physical obstacles are unchanged.

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
| Tree | reactive-ProxMPC rows at commit `40d1e61` (`test/audit-benchmark-revalidation`); the other six controllers' rows at commit `988db0a` (`chore/audit-warning-hardening`) |
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
| ProxMPC keep-out | reactive preset `safety_margin: 0.2 m`, so `d_safe = robot_radius + safety_margin = 0.42 m` (was 0.32 m in the `988db0a` session). The predictive preset pins `safety_margin: 0.1 m` (`d_safe = 0.32 m`) and is unchanged. This is a ProxMPC-internal constraint radius, not an obstacle or costmap property - no other controller reads it |
| Obstacle sensing | a **scan simulator** ray-casts the scenario obstacles into `/scan`; the `obstacle_layer` marks/clears them, so all controllers see the same obstacles |
| Repeats | **5** fixed-seed repeats per controller per scenario (**10** for MPPI on every obstacle cell, to better characterise its sampling variance) |
| Controllers | ProxMPC (Unicycle), DWB, MPPI, RPP, Graceful, Vector Pursuit - plus **ProxMPC (predictive)** with the obstacle tracker for Section 6.3-6.4 |
| Compute timing | a `nav2_core::Controller` **timing decorator** wraps every controller and times its `computeVelocityCommands` identically |

### 2.1 Fair tuning: what is equalised and what is not

The controllers are different algorithms, so their *internal* knobs are not the same quantity and cannot be set "equal" without meaning something different for each.
The fair approach is to equalise everything they *share* and leave each method's intrinsic mechanism at its documented operating point:

- **Equalised** - control rate (20 Hz), max linear speed (0.5 m/s), goal tolerance (0.25 m), the **prediction horizon (2.0 s)** for all predictive controllers (ProxMPC `np*dt = 20 x 0.1`; DWB `sim_time = 2.0`; MPPI `time_steps x model_dt = 40 x 0.05`), a **uniform physical obstacle size** across every cell, and the **obstacle perception**: a shared local *and* global `obstacle_layer` fed by the scan simulator means every controller (including ProxMPC's costmap fill) sees the *same physical obstacles* through the *same costmaps + inflation*, and receives the *same* obstacle-routed global plan. In the head-to-head ProxMPC runs `predict_obstacles: false`, so it gets **no** privileged obstacle knowledge; the separate **ProxMPC (predictive)** variant (Section 6.4) turns that flag on and adds the real obstacle tracker. The two presets previously differed only in that flag, its slack penalty, and the costmap threshold, and so came close to isolating prediction; since the reactive preset moved to `safety_margin: 0.2` while the predictive one pins 0.1, they now also differ by a 0.10 m keep-out and the pair no longer isolates prediction alone (Section 8).
- **A note on the shared global plan.** Because the global costmap carries an `obstacle_layer`, NavFn bends the *global* path around obstacles before any controller runs. This is deliberately equal for all six, but it is worth stating plainly that it **helps the pure path-followers most**: DWB, RPP, Vector Pursuit and Graceful track that global detour closely, so an obstacle-routed global plan does much of their avoidance for them, whereas ProxMPC re-optimises locally and leans less on it. The comparison therefore measures *local avoidance on top of an equal, obstacle-aware global plan* - not local avoidance in isolation.
- **Left at each method's default** - the *avoidance mechanism itself*, because it has no common denominator across paradigms: DWB's `BaseObstacle` critic (nav2_bringup default `scale 0.02`), MPPI's `CostCritic` (upstream default `cost_weight 3.81`), ProxMPC's in-loop keep-out constraint (`d_safe = 0.42 m` reactive, 0.32 m predictive), and RPP/Graceful's forward-simulation collision check. Crippling any of them to a common number would misrepresent it. The obstacles each faces are equal; how each responds is its own algorithm.
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
| **ProxMPC** (Unicycle) | 5/5 | 13.01 ± 1.96 | 0.237 | **0.0002 ± 0.0000** | 0.0005 |
| Regulated Pure Pursuit | 5/5 | 11.91 ± 1.44 | 0.235 | 0.0000 ± 0.0000 | 0.0000 |
| MPPI | 5/5 | 12.66 ± 0.35 | 0.231 | 0.0028 ± 0.0001 | 0.0040 |
| DWB | 5/5 | 12.08 ± 0.94 | 0.237 | 0.0002 ± 0.0001 | 0.0005 |
| Vector Pursuit | 5/5 | 11.48 ± 1.10 | 0.236 | 0.0000 ± 0.0000 | 0.0000 |
| Graceful | 5/5 | 10.66 ± 0.66 | 0.218 | 0.0000 ± 0.0000 | 0.0000 |

All six track the straight line essentially perfectly (sub-millimetre cross-track) and complete 5/5; ProxMPC and DWB are exact to the plant resolution and MPPI is within 3 mm.
On *tracking fidelity* there is no meaningful separation on an empty straight line - which is exactly why the compute-and-resource comparison below is the discriminating open-cell result for the predictive controllers.
The wider keep-out leaves tracking untouched in any practical sense: ProxMPC's cross-track RMS moved from 0.0001 m to 0.0002 m, matching DWB and still two orders of magnitude inside the 0.25 m goal tolerance.
Its time-to-goal spread widened (±1.01 s to ±1.96 s over five runs, range 9.9-15.9 s) without a change in the mean; that is wall-clock bring-up and scheduling jitter on a 5 m traverse, not a control effect, and Section 8 bounds how far such a shift can be read.

## 4. Per-cycle compute cost and process resources

This is the comparison that matters on an embedded target, and it is **measured, not inferred**: the timing decorator wall-clock times every controller's `computeVelocityCommands` the same way, on **every** scenario, and a per-process sampler reads the `controller_server`'s CPU and RSS from `/proc`.
Unlike tracking, compute is *not* flat across scenarios - the obstacle cells activate each method's avoidance machinery (ProxMPC's in-loop keep-out rows, DWB/MPPI's cost critics), so the per-cycle cost grows with obstacle load.
The tables are therefore **per scenario, across all eleven cells** - split into single-obstacle and multi-obstacle groups - so that growth is visible.

**ProxMPC (predictive)** is the same plugin with `predict_obstacles: true` and the obstacle tracker in the loop (Section 6.4), otherwise identical to reactive ProxMPC - so the two rows isolate the cost of prediction alone.
Its companion tracker runs as a *separate* process (~1.0 % of one core, ~38.5 MB RSS, constant across scenarios), which the `controller_server` figures below do **not** include; it is reported separately.

### 4.1 Per-cycle compute, `p50 / p95` [ms] (mean over repeats)

**Single-obstacle cells:**

| Controller | open | static_box | line_fwd | line_bwd | circle |
| --- | --- | --- | --- | --- | --- |
| Graceful | 0.16 / 0.28 | 0.16 / 0.27 | 0.16 / 0.29 | 0.16 / 0.30 | 0.16 / 0.28 |
| Regulated Pure Pursuit | 0.21 / 0.39 | 0.22 / 0.41 | 0.22 / 0.38 | 0.22 / 0.41 | 0.21 / 0.37 |
| Vector Pursuit | 0.22 / 0.41 | 0.21 / 0.37 | 0.22 / 0.40 | 0.22 / 0.38 | 0.22 / 0.38 |
| **ProxMPC** | **0.65 / 1.93** | 1.53 / 4.22 | 1.70 / 5.54 | 1.64 / 5.72 | 2.26 / 8.87 |
| **ProxMPC (predictive)** | 0.35 / 0.61 | 0.94 / 2.77 | 0.63 / 3.38 | 1.02 / 4.92 | 1.62 / 5.87 |
| DWB | 2.65 / 4.56 | 2.76 / 4.43 | 2.75 / 4.58 | 2.72 / 4.43 | 2.82 / 4.31 |
| MPPI | 2.87 / 4.36 | 2.85 / 4.17 | 2.87 / 4.13 | 2.84 / 4.12 | 2.87 / 3.90 |

**Multi-obstacle cells:**

| Controller | dyn_multi | dyn_multi_noise | blind_0 | blind_1 | blind_2 | blind_3 |
| --- | --- | --- | --- | --- | --- | --- |
| Graceful | 0.17 / 0.29 | 0.16 / 0.25 | 0.16 / 0.25 | 0.17 / 0.29 | 0.17 / 0.24 | 0.16 / 0.23 |
| Regulated Pure Pursuit | 0.22 / 0.38 | 0.22 / 0.34 | 0.21 / 0.32 | 0.21 / 0.31 | 0.21 / 0.31 | 0.22 / 0.31 |
| Vector Pursuit | 0.22 / 0.34 | 0.21 / 0.30 | 0.21 / 0.31 | 0.21 / 0.33 | 0.21 / 0.32 | 0.22 / 0.34 |
| **ProxMPC** | 3.29 / 9.42 | 3.21 / 9.28 | 3.18 / 8.23 | 2.95 / 6.96 | 2.44 / 8.54 | 2.65 / 5.84 |
| **ProxMPC (predictive)** | 2.18 / 5.27 | 2.14 / 5.60 | 1.69 / 4.98 | 1.89 / 5.28 | 2.14 / 5.25 | 1.96 / 5.52 |
| DWB | 2.91 / 4.48 | 2.99 / 5.08 | 2.77 / 3.50 | 2.81 / 3.66 | 2.77 / 3.70 | 2.89 / 3.44 |
| MPPI | 2.86 / 4.01 | 2.81 / 3.43 | 2.76 / 3.39 | 2.74 / 3.29 | 2.81 / 3.35 | 2.80 / 3.38 |

### 4.2 Process CPU [% of one core] / RSS peak [MB]

**Single-obstacle cells:**

| Controller | open | static_box | line_fwd | line_bwd | circle |
| --- | --- | --- | --- | --- | --- |
| Regulated Pure Pursuit | 3.9 / 55 | 3.9 / 55 | 3.9 / 55 | 3.9 / 55 | 4.0 / 55 |
| Vector Pursuit | 3.9 / 55 | 4.0 / 55 | 3.9 / 55 | 3.8 / 55 | 3.9 / 55 |
| Graceful | 3.9 / 56 | 4.0 / 56 | 3.9 / 56 | 3.8 / 56 | 3.9 / 56 |
| **ProxMPC** | 6.2 / 59 | 7.6 / 59 | 8.3 / 59 | 8.2 / 59 | 10.4 / 59 |
| **ProxMPC (predictive)** | 4.5 / 60 | 6.3 / 60 | 5.8 / 60 | 6.6 / 60 | 8.0 / 60 |
| DWB | 8.6 / 60 | 8.9 / 60 | 8.9 / 60 | 8.8 / 60 | 9.1 / 60 |
| MPPI | 8.7 / 63 | 8.7 / 63 | 8.9 / 63 | 8.8 / 63 | 9.0 / 63 |

**Multi-obstacle cells:**

| Controller | dyn_multi | dyn_multi_noise | blind_0 | blind_1 | blind_2 | blind_3 |
| --- | --- | --- | --- | --- | --- | --- |
| Regulated Pure Pursuit | 4.0 / 55 | 4.1 / 56 | 4.2 / 55 | 4.2 / 55 | 4.0 / 55 | 4.2 / 55 |
| Vector Pursuit | 3.9 / 55 | 4.0 / 55 | 4.0 / 55 | 4.1 / 55 | 4.0 / 55 | 4.0 / 55 |
| Graceful | 4.0 / 56 | 4.2 / 56 | 4.1 / 56 | 4.2 / 56 | 4.2 / 56 | 4.1 / 56 |
| **ProxMPC** | 11.8 / 59 | 11.9 / 59 | 11.8 / 59 | 10.4 / 59 | 10.8 / 59 | 9.3 / 59 |
| **ProxMPC (predictive)** | 8.9 / 60 | 8.5 / 60 | 8.1 / 60 | 8.7 / 60 | 8.6 / 60 | 8.6 / 60 |
| DWB | 9.3 / 60 | 9.4 / 60 | 8.9 / 60 | 9.2 / 60 | 9.1 / 60 | 9.1 / 60 |
| MPPI | 9.0 / 63 | 9.0 / 63 | 9.1 / 64 | 8.9 / 63 | 9.0 / 64 | 9.0 / 63 |

> ProxMPC-predictive additionally spends ~1.0 % core and ~38.5 MB in the obstacle-tracker process, constant across every scenario - a small overhead the tables above exclude.

The compute comparison across the eleven cells:

- **The wider keep-out costs per-cycle compute on every one of the eleven cells, and the mechanism is traceable to the diff.** Raising `safety_margin` from 0.1 m to 0.2 m raises `d_safe` from 0.32 m to 0.42 m, and [`prox_mpc_controller.cpp:1733`](../prox_mpc_controller/src/prox_mpc_controller.cpp) sizes the costmap scan window as `search_radius = d_safe + obstacle_cluster_radius`. That radius went from 0.62 m to 0.72 m, so at the 0.05 m costmap resolution the per-node scan window grew from 13 to 15 cells - about 1.3x the cells read per horizon node per cycle - and more of the cells it finds fall inside `d_safe` and bind to a QP slot. Both halves are visible in the telemetry: on the open cell the scan-and-reduce portion (decorator compute minus internal QP solve) grew from 0.09 ms to 0.16 ms while the QP solve itself grew from 0.25 ms to 0.50 ms, with SQP iterations flat at 1.00 and external QP iterations flat (3.13 to 3.18). The QP is carrying more rows, not iterating more.
- **The open-cell advantage roughly halved, and it is no longer the campaign's headline result.** ProxMPC computes a command in **0.65 ms median / 1.93 ms p95** on the open cell, against 0.34 / 1.02 ms at `safety_margin: 0.1` - **~4.1x faster than DWB and ~4.4x than MPPI at the median**, down from ~7.8x/~8.4x. The per-cycle allocation and locking fixes credited in the previous campaign are still in the binary; what changed is that a wider keep-out reaches the room's end walls on a cell that has no scenario obstacle at all, so the "empty" cell is no longer empty from the solver's point of view. The rising peak keep-out slack on that cell (0.131 m to 0.192 m) is the same effect seen directly.
- **On the multi-obstacle cells ProxMPC is now at parity with, or slightly behind, DWB and MPPI at the median.** Per-cell median ratios against DWB are 0.88x (`dynamic_multi`), 0.93x (`dynamic_multi_noise`), 0.87x (`blind_multi_0`), 0.95x (`blind_multi_1`), 1.13x (`blind_multi_2`) and 1.09x (`blind_multi_3`) - so on four of the six it is **slightly heavier per cycle than DWB**, where at `safety_margin: 0.1` it was 1.2-2.7x lighter on every obstacle cell. It remains 1.6-1.9x lighter on the static box and the two straight patrols and 1.25x on the orbit. The blanket claim that ProxMPC is cheaper per cycle than the sampling controllers on obstacle cells no longer holds and has been withdrawn.
- **Process CPU crossed over on the same cells.** ProxMPC now runs at **6.2 %** of one core on the open cell rising to **11.9 %** on the hardest cell, against DWB/MPPI's steady **8.6-9.4 %**. It is lighter on the open cell, the static box and the two patrols, and **heavier than both on the orbit and all six multi-mover cells**. ProxMPC-predictive, unchanged at `safety_margin: 0.1`, is steadier at 4.5-8.9 % and is now the cheaper of the two ProxMPC configurations on every cell. RSS is unchanged at 59 MB for ProxMPC and 55-64 MB across the field, MPPI the heaviest.
- **The geometric controllers are unaffected and remain far lighter** - RPP, Graceful and Vector Pursuit compute in ~0.16-0.22 ms at ~3.8-4.2 % CPU, carrying no optimisation or constraint machinery. ProxMPC's premium over them widened with the keep-out, from ~0.5-6.5 % to ~2.3-7.9 % of one core.
- **Prediction is now cheaper than reactive on every cell, not just most.** ProxMPC-predictive sits below reactive at the median on all eleven cells (e.g. open 0.35 vs 0.65 ms, `dynamic_multi_noise` 2.14 vs 3.21 ms) with a lighter p95 tail throughout. Part of that is now a keep-out difference rather than a prediction difference - the predictive preset pins `safety_margin: 0.1` and a `costmap_cost_threshold` of 253 against the reactive preset's 200 - so the two rows no longer isolate the cost of prediction alone, and this comparison should be read as configuration-vs-configuration until the predictive preset is re-measured at a matched margin. The companion tracker's ~1.0 % core and ~38.5 MB remain outside these figures.
- **The tail grew on every cell and the worst case still overruns the budget.** ProxMPC's p95 reaches **4.2-9.4 ms** on the obstacle cells (from 3.3-8.9 ms), against DWB's and MPPI's steadier 3.3-5.1 ms; it now exceeds both on every obstacle cell except the static box. Every p95 still sits well inside the 50 ms control budget, but **4 of the 110 ProxMPC-family runs recorded a single cycle above 50 ms, peaking at 255 ms** (see Section 5 for the exact population and cells). This remains a direct consequence of rebuilding the QP factorization every cycle.
- **The achieved command rate holds at the nominal 20 Hz on every cell.** ProxMPC's sampled `/cmd_vel` rate spans **19.96-20.09 Hz** across all eleven cells, against 19.86-20.22 Hz before, so the extra per-cycle work is absorbed inside the 50 ms budget and does not show up as a dropped command. Only 2 of the 435 b2 runs report a non-zero deadline-miss rate at all (Section 5). The reactive path also stopped losing runs on the two-mover cells: success went from 29/30 to **30/30** (Section 6.4).

## 5. ProxMPC solver profile (and a cross-check)

ProxMPC is the only controller that also publishes its *internal* solver telemetry ([`SolverDiagnostics`](../prox_mpc_msgs/README.md)):

| Metric | Open cell | With obstacle constraints active |
| --- | --- | --- |
| decorator compute p95 (whole `computeVelocityCommands`) | 1.93 ms | 4.2-9.4 ms |
| internal QP solve p95 | 1.71 ms | 4.1-9.4 ms |
| worst single-cycle solve (max) | 4.28 ms | up to 243 ms |
| deadline-miss rate (compute > 50 ms budget) | 0.0 % | 0.0 % mean, nonzero on 2 of 435 runs |
| infeasible rate (QP status != SOLVED) | 0.0 % | 0.0 % mean, nonzero on 2 of 110 ProxMPC-family runs |
| mean SQP iterations | 1.00 | 1.00-1.06 |

The two independent instruments still agree on the open cell: the QP solve (1.71 ms p95) accounts for almost all of the measured per-cycle compute (1.93 ms p95), the small remainder being the costmap reduction and the exact footprint veto.
Both roughly doubled against the 0.32 m keep-out (0.91 ms and 1.02 ms), and the split between them is what identifies the cause - Section 4 reads it in detail.
The worst single open-cell cycle stays **~12x inside** the 50 ms budget, which is why ProxMPC holds 20 Hz with 0 % deadline-miss and 0 % infeasible.
With the in-loop obstacle constraints active the per-cycle cost rises to a p95 of 4.2-9.4 ms across the ten obstacle scenarios.

Two populations matter here, and they are reported separately rather than blended.
Across **all 435 b2 runs**, deadline-miss rate is nonzero on exactly **2**: `dynamic_multi`/ProxMPC-pred repeat 2 at 0.53 % (carried over unchanged) and `dynamic_circle`/ProxMPC repeat 3 at 0.31 % (new).
Across the **110 ProxMPC-family runs** - the only runs where `solve_ms_max` is populated - a single cycle exceeded the 50 ms budget on exactly **4**, up from 3: 255 ms on `dynamic_multi`/ProxMPC-pred repeat 2 and 65 ms on `blind_multi_1`/ProxMPC-pred repeat 2 both carry over from the previous session, while the two reactive outliers moved with the re-run, from 229 ms on `blind_multi_0` repeat 3 to **243 ms on `dynamic_circle` repeat 3** and 65 ms on `dynamic_multi` repeat 4.
The campaign peak of 255 ms therefore belongs to the predictive preset, which this pass did not re-measure; the reactive peak is 243 ms.

That the reactive outlier moved from a two-mover cell to the orbit cell is worth stating plainly but not over-reading: one cycle in one run out of five is a tail sample, the same "boundary-sensitive" population Section 8 documents, and nothing about the wider keep-out predicts which cycle pays the full symbolic cost.
Each cycle rebuilds the QP factorization from scratch, so any hard cycle can.
The solver returned a feasible QP solution in time on 433 of 435 runs; where reactive ProxMPC misses on a multi cell (Section 6.4) it is the *geometry* of the convex keep-out that fails, not the solver's timing or feasibility - and Section 6.4 shows a wider keep-out does not fix that geometry.

## 6. Obstacle avoidance

The open cell cannot show avoidance, so these scenarios place obstacles across the path and let every controller perceive them through the same local and global `obstacle_layer`.
Because the kinematic plant has no collision physics, "reaching the goal" alone does not prove avoidance - a controller can drive straight through an obstacle and still "succeed" - so the discriminating metric is the **obstacle gap** (robot-disc to obstacle-disc; negative = overlap = would-be collision).

The global costmap carries an `obstacle_layer`, so NavFn routes the global plan around obstacles for every controller, and every cell uses one uniform obstacle size (`clearance 0.45`). The single-obstacle cells (Section 6.1-6.2) are within reach of the whole field and separate the controllers on *margin* and on the orbiting case; the multi-obstacle cells (Section 6.4) are where collision counts spread. Two comparisons carry the result: ProxMPC against the other optimisation- and sampling-based controllers - **DWB** and **MPPI**, its per-cycle-cost peers - and ProxMPC against the geometric pursuit controllers - **RPP**, **Graceful** and **Vector Pursuit** - which trade avoidance headroom for near-zero compute.

### 6.1 Static obstacle (`static_box`, mode b2)

A 0.25 m box placed **dead-centre** on the path at `(0, 0)`; with the obstacle-routed global plan every controller has a path that already bends around it.
The last two columns carry the per-scenario **cost of that avoidance** (the same decorator/process instruments as Section 4), so clearance and its price are read together.

| Controller | success | obstacle gap [m] | collision | compute p50/p95 [ms] | CPU [%] |
| --- | --- | --- | --- | --- | --- |
| **ProxMPC** | 5/5 | **+0.448 ± 0.001** | **0/5** | 1.53 / 4.22 | 7.6 |
| **ProxMPC (predictive)** | 5/5 | +0.298 ± 0.001 | **0/5** | 0.94 / 2.77 | 6.3 |
| Regulated Pure Pursuit | 5/5 | +0.223 ± 0.021 | 0/5 | 0.22 / 0.41 | 3.9 |
| Graceful | 5/5 | +0.212 ± 0.004 | 0/5 | 0.16 / 0.27 | 4.0 |
| MPPI | 10/10 | +0.208 ± 0.003 | 0/10 | 2.85 / 4.17 | 8.7 |
| DWB | 5/5 | +0.087 ± 0.008 | 0/5 | 2.76 / 4.43 | 8.9 |
| Vector Pursuit | **3/5** | +0.212 ± 0.086 | 0/5 | 0.21 / 0.37 | 4.0 |

Every controller clears the static box collision-free, so the discrimination is in the *margin*, not in whether they hit.
**ProxMPC keeps the largest margin of the field - +0.448 m**, up from +0.350 m at the 0.32 m keep-out, and it is the tightest-clustered result in the whole campaign (± 0.001 m over five runs, individual gaps +0.446 to +0.449 m).
That near-zero spread is the clearest single piece of evidence for how the constraint behaves: the solver rides *exactly* on `d_safe`, so the measured clearance tracks the parameter almost one-for-one - a +0.10 m increase in `safety_margin` bought +0.098 m of measured margin.
The behaviour is unchanged - swerve wide and recover, what the in-loop keep-out constraint produces - and it still costs **~1.8x less per-cycle compute than DWB and MPPI**, down from ~2.4x.
Predictive ProxMPC holds +0.298 m on its unchanged 0.32 m keep-out; the tracker reports the box at ~0 velocity, so the hybrid fill treats it much like the reactive path, and it is now the *narrower* of the two ProxMPC configurations here by 0.15 m.
Among the cost-class peers, **DWB still cuts it finest at +0.087 m** - its soft `BaseObstacle` critic rides close to the inflated edge - while MPPI holds +0.208 m; among the geometric controllers RPP and Graceful hold ~+0.21-0.22 m.
**Vector Pursuit now completes 3 of 5 runs (was 0/5 in the previous campaign)**, with no source change on either the plugin or the harness: its own forward-collision check (`use_collision_detection`) is close enough to a decision boundary here that bring-up and discovery timing alone flip the outcome run to run. The 2 stopped-short runs report a gap of ~0.31 m (a stopped-short distance, not a steered avoidance); the 3 completed runs steer around at ~0.13-0.18 m. The blended +0.212 ± 0.086 m in the table above mixes both behaviours and should be read with that in mind, not as a single steering margin.

### 6.2 Single moving obstacles (mode b2, reactive)

Three single-mover scenarios - a patrol crossing the path forward (`dynamic_line_forward`) and backward (`dynamic_line_backward`) at 0.6 m/s, and an obstacle orbiting near mid-path (`dynamic_circle`) at 0.5 m/s.
Most of the field clears these reactively; the orbiting case is where the field splits, and reactive ProxMPC has moved from the losing side of that split to the winning one:

| Scenario | metric | ProxMPC | ProxMPC-pred | DWB | MPPI | RPP | Graceful | VecPursuit |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| line_forward | collision | 0/5 | 0/5 | 0/5 | 0/10 | 0/5 | 0/5 | 0/5 |
| | min gap [m] | **+0.59** | +0.35 | +0.47 | +0.55 | +0.40 | +0.32 | +0.46 |
| line_backward | collision | 0/5 | 0/5 | 0/5 | 0/10 | 0/5 | 0/5 | 0/5 |
| | min gap [m] | +0.56 | +0.39 | +0.49 | **+0.60** | +0.41 | +0.30 | +0.46 |
| circle | collision | **0/5** | 0/5 | **5/5** | 0/10 | 1/5 | **5/5** | 0/5 |
| | min gap [m] | +0.09 | +0.10 | **-0.28** | +0.17 | +0.05 | **-0.07** | +0.14 |

**On the two straight patrols every controller keeps a real margin (0 collisions)**, and reactive ProxMPC now holds the widest gap of the field on `line_forward` (+0.59 m, from +0.50 m) and second only to MPPI on `line_backward` (+0.56 m, from +0.54 m).

**On the orbiting obstacle the reactive result reverses: ProxMPC now clears it 0/5 at +0.093 m, where at the 0.32 m keep-out it collided 3/5 at a mean gap of -0.005 m.**
Every one of the five runs is positive (+0.009, +0.033, +0.038, +0.073, +0.310 m), so this is not a near-boundary coin flip resolving favourably - the whole distribution moved off the boundary.
The mechanism is specific and worth stating plainly, because the previous revision of this document could not name one.

The orbit failure was a *margin* problem, not a prediction problem. The keep-out is a hard constraint built from the obstacle's **current** costmap position, and the solver rides exactly on it - Section 6.1's ± 0.001 m static-box clustering is that behaviour measured directly. At `d_safe = 0.32 m` there was therefore nothing left over to absorb the obstacle's own displacement between one costmap update and the next, which on a 0.5 m/s orbiter is several centimetres per cycle - the exact size of the negative gaps recorded. Bisect established that the reactive path had previously been leaning on an *implicit* margin from a different mechanism: commit `6d12bd9` raised the endpoint footprint-veto threshold from `INSCRIBED_INFLATED_OBSTACLE` to `LETHAL_OBSTACLE`, matching upstream Nav2, and in doing so removed a veto that had been rejecting endpoints in the inflation ring. Raising `safety_margin` restores that margin in the QP keep-out, where a margin belongs, rather than in a footprint veto that was enforcing it as a side effect.

This is the one place in this campaign where the wider keep-out is unambiguously the right trade. Predictive ProxMPC is unchanged (0/5, +0.10 m), and reactive ProxMPC now matches it on this cell without a tracker. DWB (5/5) and Graceful (5/5) remain the field's clear failures here - both still commit to a trajectory the obstacle then orbits into, and the routed global plan, computed once against the obstacle's initial footprint, still does not track the moving object.

The price is compute: the orbit cell is where the wider keep-out costs most among the single-obstacle cells, at 2.26 ms median (from 1.92 ms) and a p95 of 8.87 ms (from 7.75 ms), the highest single-obstacle p95 in the campaign, and it is the one single-obstacle cell where ProxMPC's process CPU (10.4 %) now exceeds DWB's and MPPI's (~9.0 %). ProxMPC stays 1.25x lighter than both at the median here, down from 1.5x.

### 6.3 Ground-truth predictive confirmation (mode b1)

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

The single-obstacle cells discriminate a little more than before (Section 6.1-6.2), but the collision comparison still rests on the **multi-mover** set: `dynamic_multi` and `dynamic_multi_noise` (two hand-built simultaneous movers, the latter with sensor noise), and `blind_multi_0`-`blind_multi_3` (blind, author-independent two-mover cells, generated without hand-screening).
Six cells, 30 runs per controller (60 for MPPI), collision per cell:

| Scenario | ProxMPC | ProxMPC-pred | DWB | MPPI | RPP | Graceful | VecPursuit |
| --- | --- | --- | --- | --- | --- | --- | --- |
| dynamic_multi | 4/5 | 1/5 | 1/5 | 4/10 | 0/5 | 3/5 | 2/5 |
| dynamic_multi_noise | 5/5 | 0/5 | 2/5 | 3/10 | 0/5 | 4/5 | 1/5 |
| blind_multi_0 | 2/5 | 0/5 | 0/5 | 10/10 | 2/5 | 2/5 | 5/5 |
| blind_multi_1 | 1/5 | 0/5 | 0/5 | 6/10 | 1/5 | 2/5 | 0/5 |
| blind_multi_2 | 5/5 | 0/5 | 5/5 | 0/10 | 5/5 | 5/5 | 5/5 |
| blind_multi_3 | 2/5 | 0/5 | 0/5 | 7/10 | 0/5 | 0/5 | 2/5 |
| **aggregate collisions** | **19/30** | **1/30** | **8/30** | **30/60** | **8/30** | **16/30** | **16/30** |
| **aggregate success** | 30/30 | 30/30 | 30/30 | 60/60 | 30/30 | 30/30 | 29/30 |
| **median margin [m]** | -0.062 | **+0.157** | +0.098 | +0.002 | +0.124 | -0.006 | -0.020 |
| **runs within 0.15 m of contact** | 29/30 | **13/30** | 21/30 | 51/60 | 19/30 | 30/30 | 25/30 |

**Read the margin rows, not the collision counts.** These cells are deliberately marginal: 63-100 % of the six non-predictive controllers' runs finish within 0.15 m of contact, so a few centimetres of scheduling jitter flips a near-miss into a collision. Reactive ProxMPC is now at 29 of 30 runs inside that band, the tightest-packed of the field after Graceful. The median closest approach is the more stable statistic and is the discriminator this section rests on.

**The wider keep-out does not help here, and slightly hurts.** This is the central negative result of the re-measurement. Raising `d_safe` from 0.32 m to 0.42 m moved reactive ProxMPC's aggregate collisions from 18/30 to **19/30** and its median margin from -0.047 m to **-0.062 m** - the wrong direction on both, though by less than the cell-to-cell noise this section documents. Per cell the changes cancel rather than accumulate: `dynamic_multi` 3/5 to 4/5, `dynamic_multi_noise` 4/5 to 5/5 and `blind_multi_3` 1/5 to 2/5 got worse, `blind_multi_0` 3/5 to 2/5 and `blind_multi_1` 2/5 to 1/5 got better, and `blind_multi_2` stayed at 5/5. The one unambiguous gain is completion: aggregate success went from 29/30 to **30/30**, so the reactive path no longer loses a run outright on these cells.

The reason a wider keep-out cannot fix these cells is the same reason it fixed the orbit, read in reverse. On the orbit there is one obstacle and one feasible side to pass it, so more margin is simply more room. On a two-mover cell the controller must choose *which side of each obstacle* to pass, and the linearised keep-out represents only one convex half-plane per obstacle per horizon node - it cannot represent the disjunction. Widening `d_safe` enlarges an already-wrong convex region rather than fixing the side-choice, and on the cells where the two movers close from opposite sides it removes feasible corridor that the narrower disc could still thread. That is a limitation of the constraint's *form*, not its size, and no value of `safety_margin` addresses it.

By margin the ordering is otherwise unchanged, since the other six controllers were not re-run: **predictive ProxMPC leads the field at +0.157 m**, ahead of RPP (+0.124), DWB (+0.098), MPPI (+0.002), Graceful (-0.006) and Vector Pursuit (-0.020). Predictive ProxMPC is still the only controller with fewer than half its runs inside the marginal band (13/30). **Reactive ProxMPC remains the field's narrowest, now at -0.062 m**, and the gap to its nearest peers (Vector Pursuit -0.020, Graceful -0.006) widened again after having closed in the previous campaign.

Prediction therefore remains the effective answer on multi-mover cells, and this re-measurement sharpens rather than weakens that conclusion: it moves ProxMPC from the field's narrowest margin to its widest and from 19/30 collisions to 1/30, and the cheaper intervention - more margin on the reactive path - has now been measured and does not substitute for it. Note that the two ProxMPC rows no longer differ by prediction alone; the predictive preset also keeps the 0.32 m keep-out and a different `costmap_cost_threshold`, so part of the remaining gap is configuration (Section 8).

The cell where ProxMPC's reactive keep-out is weakest by margin is still `blind_multi_2`, and it got tighter rather than looser (median -0.245 m, from -0.197 m). `blind_multi_2` is also DWB's worst cell (-0.236 m) and Graceful's worst (-0.144 m) - it remains the hardest geometry for several of the field at once. Predictive ProxMPC's worst cell, `blind_multi_1` at +0.120 m, is still comfortably positive, and every one of its six per-cell medians is positive. No controller is collision-free across the blind cells, and the winner is still cell-dependent - DWB clears three cells outright but collides 5/5 on `blind_multi_2`.

The per-cell breakdown for every controller is in [`prox_mpc_benchmark/README.md`](../prox_mpc_benchmark/README.md) and the raw `results/`.
In summary: ProxMPC keeps the largest static margin (Section 6.1), now clears all three single moving-obstacle cells reactively including the orbit that previously beat it (Section 6.2), and remains the narrowest of the field on the two-mover cells, where prediction rather than margin is what closes the gap - all at a per-cycle cost that the wider keep-out raised enough to erase its median advantage over DWB and MPPI on the hardest cells (Section 4).

## 7. Real-stack validation (Gazebo)

To confirm the plugin behaves under the full production stack and not only against a kinematic plant, ProxMPC also runs in mode (a): Gazebo Harmonic physics, a TurtleBot3 waffle, AMCL localization, costmaps, and the complete Nav2 velocity chain, headless.
It reaches the goal (`SUCCEEDED`), tracks the path to **5.6 mm cross-track RMS**, and taps its diagnostics through the real `controller_server` (p95 solve 0.438 ms, 0 % deadline-miss, 0 % infeasible) - the same profile measured on the plant, with real sensor and physics noise.

> Mode (a) was not re-run in this pass (it needs a working GPU driver, and the host used for every campaign to date renders Gazebo in software, which starves the control loop). This section still reflects the `v1.0.0` baseline and is pending its own re-validation; the b1/b2 numbers elsewhere in this document are unaffected, since neither uses Gazebo.

## 8. Threats to validity

- **This re-validation surfaces a systematic host-timing drift between the two measurement sessions, visible on plugin code this repository never touched.** DWB, MPPI, Regulated Pure Pursuit, Graceful, and Vector Pursuit are stock Nav2/community binaries (confirmed at the same versions as the previous campaign - Nav2 1.3.12, `vector_pursuit_controller` 2.0.0), and the shared harness code (`scan_simulator.cpp`, `kinematic_plant.cpp`, every `config/` file this campaign used) is byte-identical to the previous campaign. Yet DWB and MPPI's per-cycle compute p95 rose by roughly 0.6-1.7 ms on almost every scenario, and RPP/Graceful/Vector Pursuit's CPU reading dropped by roughly 0.3-0.6 points almost everywhere. Since none of that code changed, the most defensible explanation is background load, thermal/turbo state, or scheduler variance between the two sessions on the same physical host - not a software effect. This bounds how confidently any *ProxMPC-side* shift of similar magnitude can be attributed to the intervening code changes versus the same host effect; the two per-cycle open-cell wins reported in Section 4 are large enough (halving, not shifting by tens of percent) and directionally opposite to the host drift to stand on their own, but smaller shifts should be read with this in mind.
- **The ProxMPC rows and the other six controllers' rows come from different measurement sessions**, one day apart on the same host (see Provenance at the top). This is sound for the comparisons this document draws - the physical obstacles, plant, map, planner and instrumentation are identical, and the other controllers do not read the changed parameter - but it means any ProxMPC-versus-peer difference carries one session boundary's worth of host-timing drift on top of the parameter effect. The drift documented in the bullet above was measured at roughly 0.6-1.7 ms of p95 on stock plugin code; ProxMPC's compute shifts in Section 4 are larger than that (roughly doubling on the open cell) and are additionally explained by a traceable mechanism in the diff, so they stand. Differences of a few tenths of a millisecond between ProxMPC and a peer should not be read as real.
- **Vector Pursuit's static-box completion flipped between campaigns on plugin code that did not change** (0/5 to 3/5, Section 6.1), and remains carried over unchanged here. It is treated as evidence that this specific decision point sits close to a boundary that run-to-run timing can cross, not as an attributed regression.
- **The two ProxMPC rows no longer isolate prediction.** Reactive ProxMPC now runs `safety_margin: 0.2` and `costmap_cost_threshold: 200`; the predictive preset keeps `safety_margin: 0.1` and `costmap_cost_threshold: 253`. Before this change the two presets differed only in `predict_obstacles`, `w_weight`, the threshold and the tracker, so the pair came close to isolating prediction's cost and benefit. They no longer do, and the reactive-versus-predictive comparisons in Sections 4 and 6.4 are configuration-versus-configuration until the predictive preset is re-measured at a matched margin. The direction of the multi-mover result is large enough (1/30 against 19/30 collisions) that a 0.10 m keep-out difference does not plausibly account for it, but the compute comparison is now confounded and the margin comparison partly so.
- **The obstacle-routed global plan does much of the single-obstacle avoidance** (Section 6). The shared global `obstacle_layer` makes NavFn route around obstacles for every controller - deliberately equal, but it favours the geometric path-followers (RPP, Graceful, Vector Pursuit) and DWB, which track the global detour closely. The single-obstacle cells therefore separate the field on margin and on the orbiting case, not on collision counts; the multi-mover cells (Section 6.4) carry the collision comparison.
- **The multi-obstacle field is close and the sample is small** (Section 6.4). 63-100 % of runs across the six cells finish within 0.15 m of contact, so the per-cell collision counts swing by more than the gap between most controllers. Reactive ProxMPC's aggregate moved by one run (18/30 to 19/30) and its median margin by 15 mm across this change - **both inside the noise this section documents**, which is why Section 6.4 states that the wider keep-out does not help rather than that it hurts. The per-cell counts moved in both directions (three cells worse, two better, one flat), which is the signature of noise rather than a directed effect. What is *not* inside the noise is the direction of the whole-distribution shift on the orbit cell (Section 6.2), where all five runs moved off the boundary together.
- **A five-repeat sample cannot resolve a 15 mm median shift.** Section 6.4's conclusion that margin is the wrong lever for multi-mover cells rests on the mechanism (a convex keep-out cannot represent the which-side disjunction) and on the absence of improvement, not on the sign of the 15 mm change. Distinguishing "slightly worse" from "unchanged" on these cells would need more repeats than this campaign runs.
- **Collision uses a strict min-gap** - any instant of disc overlap over the whole run counts as a collision, so a brief graze is flagged the same as a harder hit; the reported min gap distinguishes the two.
- **The per-cycle cost of the wider keep-out is measured on this map's geometry, and does not generalise unchanged.** The open-cell doubling in Section 4 happens because a 0.72 m scan radius reaches the walls of a 7 x 7 m room near the start and goal. In a larger space, or with obstacles farther from the path, the same parameter change would cost less; in a tighter one it would cost more. What transfers is the mechanism and its direction, not the factor.
- **Resources are measured on the x86 host** above (confirmed identical hardware, CPU model, and OS version to every prior campaign via `/proc/cpuinfo` and `/etc/os-release`), and process CPU/RAM is per-process (includes the shared costmap); the per-cycle compute (Section 4) is the cleaner controller-only measure, and the ranking is what transfers. The pure-geometric controllers are genuinely lighter than ProxMPC - ProxMPC is the cheapest of the controllers that solve a constrained optimisation each cycle, not the cheapest outright.
- **MPPI is stochastic with no exposed seed** (Nav2 Jazzy), so its per-scenario variance is real; it runs 10 repeats per obstacle cell (5 on the open cell) to better characterise that variance, but this does not pin it down exactly. Its multi-obstacle median margin fell from +0.048 to +0.002 this campaign - within the same "close field, small sample" caveat as the rest of Section 6.4.
- **Determinism vs. physics.** Modes (b1)/(b2) are deterministic plants, so their near-zero geometric std is reproducibility, not a noise estimate; mode (a) carries real Gazebo variance, and `dynamic_multi_noise` adds a Gaussian range-noise model on the scan. Mode (a) was not re-run in this pass (Section 7) and still reflects the `v1.0.0` baseline; mode (b1) was re-run (Section 6.3).
- **`max_obstacle_slack_m` is a max over sampled cycles, so it is only as reliable as the diagnostics subscription that feeds it** (Section 6.3). On `static_box` the metric's whole run-to-run spread turned out to be whether the metrics node received control cycle 1, whose one-cycle initialisation transient is five times the steady-state peak. The metric is sound for comparing steady-state relaxation once that first cycle is excluded, but a pass/fail gate wired to it as it stands would fail on a subscription race rather than on solver behaviour. This bounds what the slack column can be used for, here and in any future campaign; it does not affect the success, geometry, feasibility, or deadline results, which are unchanged in kind from the `v1.0.0` table.
- **b1 solve time rose on all eight cells** (median +0.4 to +2.7 ms) with no candidate cause in the diff, since mode b1 uses neither the costmap nor the Nav2 stack; it is reported as observed, consistent with the session-to-session host drift above. Every cell still solves far inside the 50 ms budget at 0 % deadline-miss and 0 % infeasible. The first-cycle transient in Section 6.3 is not the cause: measured per cycle, cycle 1 is never the slowest solve of a run.
- **The demonstration videos under `prox_mpc_benchmark/doc/media/` were not re-recorded.** The `dynamic_circle` grid in particular now shows behaviour this document no longer reports: it was recorded with reactive ProxMPC colliding on the orbit, which at `safety_margin: 0.2` it no longer does. The `static_box` grid shows a 0.098 m narrower ProxMPC margin than the current figure. Whether and when to re-record is an open question for the maintainer, not decided by this pass.

## 9. Conclusion on ProxMPC

On a like-for-like, fairly-tuned suite, ProxMPC matches the best stock Nav2 controller on tracking, holds the largest static-obstacle margin of the field, and - with the reactive keep-out widened to `d_safe = 0.42 m` - now clears every single moving-obstacle cell reactively, including the orbit that beat it at 0.32 m. The cost is per-cycle compute, which rose on all eleven cells and erased its median advantage over DWB and MPPI on the hardest ones. On simultaneous two-mover cells margin is not the lever; prediction is.

- **Per-cycle compute and process resources (measured, all eleven cells):** ProxMPC computes a command in **0.65 ms median / 1.93 ms p95** on the open cell - **~4.1x lighter than DWB and ~4.4x than MPPI**, down from ~7.8x/~8.4x at the narrower keep-out. It stays 1.6-1.9x lighter on the static box and the straight patrols, 1.25x on the orbit, and is **0.87-0.95x - marginally heavier than DWB** - on four of the six multi-mover cells. Process CPU is 6.2-11.9 % against their 8.6-9.4 %, so it is lighter on the easy cells and heavier on the hard ones. RSS is unchanged at 59 MB. Deadline misses remain essentially absent (2 of 435 b2 runs non-zero), but the tail grew: p95 reaches 4.2-9.4 ms on the hard cells against the samplers' steadier 3.3-5.1 ms, and **4 of 110 ProxMPC-family runs recorded one cycle above the 50 ms budget** (campaign peak 255 ms on the predictive preset, which this pass did not re-measure; reactive peak 243 ms) - a direct consequence of rebuilding the QP factorization every cycle. The geometric controllers (RPP, Graceful, Vector Pursuit) are lighter still; ProxMPC's premium over them widened to ~2.3-7.9 % of one core.
- **Tracking:** sub-millimetre (0.0002 m RMS), on par with DWB and the field, 100 % success. The wider keep-out does not measurably affect tracking on an empty line.
- **Static-obstacle avoidance:** **ProxMPC keeps the largest real margin, +0.448 m**, up from +0.350 m and far ahead of its cost-class peers MPPI (+0.208 m) and DWB (+0.087 m) and of the geometric RPP and Graceful (~+0.21-0.22 m). The ± 0.001 m spread over five runs is the campaign's tightest and shows the solver riding exactly on `d_safe`, so measured clearance tracks the parameter almost one-for-one.
- **Single moving obstacles:** all three cells are now cleared reactively - the two straight patrols at the field's widest or near-widest gaps (+0.59 m, +0.56 m), and **the orbit at 0/5 collisions and +0.093 m, where the 0.32 m keep-out collided 3/5 at -0.005 m**. All five orbit runs are positive, so the whole distribution moved rather than a boundary case resolving favourably. The cause is identified rather than observed: the keep-out is built from the obstacle's *current* position and the solver rides exactly on it, so at 0.32 m nothing absorbed a 0.5 m/s obstacle's displacement between costmap updates; commit `6d12bd9` had removed the implicit margin the reactive path was leaning on when it raised the endpoint footprint-veto threshold to `LETHAL_OBSTACLE`, and `safety_margin: 0.2` restores that margin in the QP keep-out where it belongs. DWB (5/5) and Graceful (5/5) remain the field's clear failures here.
- **Multiple simultaneous moving obstacles:** margin is not the lever. Widening the keep-out moved reactive ProxMPC's aggregate collisions from 18/30 to **19/30** and its median margin from -0.047 m to **-0.062 m** - the wrong direction on both, though within the noise of cells where 63-100 % of runs finish inside 0.15 m of contact. It did lift completion to **30/30**. A convex half-plane per obstacle per node cannot represent the which-side-of-each-obstacle disjunction these cells demand, so a larger disc enlarges an already-wrong feasible region; the limitation is the constraint's form, not its size. Measured by median closest approach, **predictive ProxMPC still leads the entire field at +0.157 m**, ahead of RPP (+0.124), DWB (+0.098), MPPI (+0.002), Graceful (-0.006) and Vector Pursuit (-0.020), with only 13 of 30 runs inside the marginal band. **Reactive ProxMPC remains the field's narrowest at -0.062 m**, and its weakest cell is still `blind_multi_2` (-0.245 m).
- **One controller, many vehicles:** the identical plugin drives a unicycle and a bicycle by configuration alone, confirmed 40/40 in mode b1, with no model-specific result separating the two.

Against DWB and MPPI - its per-cycle-cost peers - ProxMPC is now roughly four times lighter per cycle on the open cell at equal tracking accuracy, holds a much larger static margin, and clears an orbiting obstacle that DWB does not; but it has given up its median compute advantage on the hardest multi-mover cells and now runs slightly heavier there. Against the geometric pursuit controllers it holds larger avoidance margins for a CPU premium that this change roughly doubled. Its one clear remaining weak point is the simultaneous two-mover geometry, where the reactive formulation is limited by the shape of its constraint rather than by its size, and where the obstacle tracker - not a wider keep-out - is what closes the gap.

---

*Reproduce:* the open cell and the mode-(b2) reactive obstacle scenarios with `ros2 run prox_mpc_benchmark run_nav2.py --scenario <nav2_open|static_box|dynamic_circle|dynamic_line_forward|dynamic_line_backward|dynamic_multi|dynamic_multi_noise|blind_multi_0|blind_multi_1|blind_multi_2|blind_multi_3>`, the **predictive b2** runs (real tracker) by adding `--controllers proxmpc_pred` to the same command, the mode-(b1) ground-truth predictive results with `ros2 run prox_mpc_benchmark run_matrix.py --modes b1`, and the tables with `ros2 run prox_mpc_benchmark aggregate.py`. See [`prox_mpc_benchmark/README.md`](../prox_mpc_benchmark/README.md).

## License

[Apache-2.0](../LICENSE).
