# Controller Comparison Results

This document reports how the ProxMPC controller compares against the stock Nav2 controllers under identical, reproducible, and *fairly-tuned* conditions - on **path tracking**, **per-cycle compute and process resources**, and **obstacle avoidance**.
It is the narrative companion to the auto-generated tables in [`prox_mpc_benchmark/README.md`](../prox_mpc_benchmark/README.md); the harness that produced every number is described in [the package guide](prox-mpc.md#8-prox_mpc_benchmark---the-measurement-harness).

**Re-validation.** The figures below were re-measured against `chore/audit-warning-hardening` at commit `988db0a`, on the same host as every prior campaign.
That commit is the tip of three change waves plus review fixes, an obstacle-mapping generalization, and warning hardening landed after the `v1.0.0` baseline this document previously reported - among them the endpoint-veto/unknown-space handling, the goal-approach taper, the obstacle-slot binding and ranking, the plan-projection method, the control-space brake, and two per-cycle performance fixes (dropping two heap allocations from the reference build and narrowing the costmap lock to the cell reads).
The intervening work is one bundled set of simultaneous behaviour changes: a shift is attributed to a specific commit only where the mechanism is traceable in the diff and the metric is specific to that change; most shifts below are reported as observed, not attributed.
Mode (b1), the ground-truth predictive results in Section 6.3, was **not** re-run in this pass (out of scope for this re-validation) and still reflects the `v1.0.0` baseline; it is called out again where it appears.

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
| Tree | `chore/audit-warning-hardening` at commit `988db0a` |
| Run mode | **(b2)** Nav2 stack on a kinematic plant, no Gazebo (see [run modes](prox-mpc.md#9-running-the-stack-the-three-modes)); mode (b1) for the ground-truth predictive results in Section 6.3, not re-run in this pass |
| Plant | Unicycle body-twist integrator at 50 Hz, identical for every controller |
| Localization | Exact (static `map -> odom` identity; the plant pose is ground truth) |
| Map | `prox_mpc_open` - 7 x 7 m room, free interior `[-2.95, 2.95] m` |
| Task | straight 5 m traverse `(-2.5, 0) -> (2.5, 0)`, goal tolerance 0.25 m |
| Planner | `NavfnPlanner` (GridBased), shared by all controllers |
| Control rate | 20 Hz (`controller_frequency`); real-time budget 50 ms/cycle |
| Local costmap | `static_layer` + `obstacle_layer` + `inflation_layer` (**0.4 m** radius), 5 Hz update |
| Global costmap | `static_layer` + `obstacle_layer` + `inflation_layer` (**0.4 m** radius); the `obstacle_layer` marks the scenario obstacles so **NavFn routes the global plan around them** - the same global plan for every controller |
| Obstacle size | one **uniform** obstacle size across every b2 cell (`clearance: 0.45`); `static_box` is a 0.25 m box at dead-centre `(0, 0)` |
| Obstacle sensing | a **scan simulator** ray-casts the scenario obstacles into `/scan`; the `obstacle_layer` marks/clears them, so all controllers see the same obstacles |
| Repeats | **5** fixed-seed repeats per controller per scenario (**10** for MPPI on every obstacle cell, to better characterise its sampling variance) |
| Controllers | ProxMPC (Unicycle), DWB, MPPI, RPP, Graceful, Vector Pursuit - plus **ProxMPC (predictive)** with the obstacle tracker for Section 6.3-6.4 |
| Compute timing | a `nav2_core::Controller` **timing decorator** wraps every controller and times its `computeVelocityCommands` identically |

### 2.1 Fair tuning: what is equalised and what is not

The controllers are different algorithms, so their *internal* knobs are not the same quantity and cannot be set "equal" without meaning something different for each.
The fair approach is to equalise everything they *share* and leave each method's intrinsic mechanism at its documented operating point:

- **Equalised** - control rate (20 Hz), max linear speed (0.5 m/s), goal tolerance (0.25 m), the **prediction horizon (2.0 s)** for all predictive controllers (ProxMPC `np*dt = 20 x 0.1`; DWB `sim_time = 2.0`; MPPI `time_steps x model_dt = 40 x 0.05`), a **uniform physical obstacle size** across every cell, and the **obstacle perception**: a shared local *and* global `obstacle_layer` fed by the scan simulator means every controller (including ProxMPC's costmap fill) sees the *same physical obstacles* through the *same costmaps + inflation*, and receives the *same* obstacle-routed global plan. In the head-to-head ProxMPC runs `predict_obstacles: false`, so it gets **no** privileged obstacle knowledge; the separate **ProxMPC (predictive)** variant (Section 6.4) turns that flag on and adds the real obstacle tracker, and is otherwise the identical preset so the comparison isolates prediction.
- **A note on the shared global plan.** Because the global costmap carries an `obstacle_layer`, NavFn bends the *global* path around obstacles before any controller runs. This is deliberately equal for all six, but it is worth stating plainly that it **helps the pure path-followers most**: DWB, RPP, Vector Pursuit and Graceful track that global detour closely, so an obstacle-routed global plan does much of their avoidance for them, whereas ProxMPC re-optimises locally and leans less on it. The comparison therefore measures *local avoidance on top of an equal, obstacle-aware global plan* - not local avoidance in isolation.
- **Left at each method's default** - the *avoidance mechanism itself*, because it has no common denominator across paradigms: DWB's `BaseObstacle` critic (nav2_bringup default `scale 0.02`), MPPI's `CostCritic` (upstream default `cost_weight 3.81`), ProxMPC's in-loop keep-out constraint, and RPP/Graceful's forward-simulation collision check. Crippling any of them to a common number would misrepresent it. The obstacles each faces are equal; how each responds is its own algorithm.
- **Intrinsic sampling** left at upstream defaults: MPPI `batch_size = 2000`, DWB's `20 x 20` velocity grid, ProxMPC's single-QP SQP.
- **Determinism** - MPPI is the one stochastic controller and Nav2 Jazzy exposes no RNG seed for it, so its repeats capture genuine sampling variance (reported as `mean ± std`), not reproducibility; it is run at 10 repeats on the obstacle cells (5 on the open cell) rather than the 5 used for the deterministic controllers. The other five controllers are deterministic in principle, but mode (b2) is real wall-clock ROS execution (DDS discovery, thread scheduling, lifecycle bring-up), not simulated time, so their repeats still carry a small measured jitter - itself part of the precision signal.
- **Graceful** - a good-faith goal-approach tuning (lookahead below the goal tolerance, reduced slowdown radius, no in-place final rotation) lets it drive into the goal rather than stalling far out; it succeeds on every scenario measured here.

Because the plant, map, planner, goal checker, horizon, speed, obstacle size, costmaps, and instrumentation are all shared, every difference in the tables is attributable to the controller, its own run-to-run variance, or - as this re-validation makes visible in several places - to genuine host timing drift between two measurement sessions on unmodified stock-controller code (Section 8).

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
| **ProxMPC** (Unicycle) | 5/5 | 13.30 ± 1.01 | 0.237 | **0.0001 ± 0.0000** | 0.0002 |
| Regulated Pure Pursuit | 5/5 | 11.91 ± 1.44 | 0.235 | 0.0000 ± 0.0000 | 0.0000 |
| MPPI | 5/5 | 12.66 ± 0.35 | 0.231 | 0.0028 ± 0.0001 | 0.0040 |
| DWB | 5/5 | 12.08 ± 0.94 | 0.237 | 0.0002 ± 0.0001 | 0.0005 |
| Vector Pursuit | 5/5 | 11.48 ± 1.10 | 0.236 | 0.0000 ± 0.0000 | 0.0000 |
| Graceful | 5/5 | 10.66 ± 0.66 | 0.218 | 0.0000 ± 0.0000 | 0.0000 |

All six track the straight line essentially perfectly (sub-millimetre cross-track) and complete 5/5; ProxMPC and DWB are exact to the plant resolution and MPPI is within 3 mm.
On *tracking fidelity* there is no meaningful separation on an empty straight line - which is exactly why the compute-and-resource comparison below is the discriminating open-cell result for the predictive controllers.
Time-to-goal shifted by a second or so in either direction for several controllers relative to the previous campaign, ProxMPC included; none of that shift is outside what wall-clock bring-up timing alone can produce; see Section 8.

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
| **ProxMPC** | **0.34 / 1.02** | 1.15 / 3.31 | 1.09 / 5.27 | 1.05 / 5.45 | 1.92 / 7.75 |
| **ProxMPC (predictive)** | 0.35 / 0.61 | 0.94 / 2.77 | 0.63 / 3.38 | 1.02 / 4.92 | 1.62 / 5.87 |
| DWB | 2.65 / 4.56 | 2.76 / 4.43 | 2.75 / 4.58 | 2.72 / 4.43 | 2.82 / 4.31 |
| MPPI | 2.87 / 4.36 | 2.85 / 4.17 | 2.87 / 4.13 | 2.84 / 4.12 | 2.87 / 3.90 |

**Multi-obstacle cells:**

| Controller | dyn_multi | dyn_multi_noise | blind_0 | blind_1 | blind_2 | blind_3 |
| --- | --- | --- | --- | --- | --- | --- |
| Graceful | 0.17 / 0.29 | 0.16 / 0.25 | 0.16 / 0.25 | 0.17 / 0.29 | 0.17 / 0.24 | 0.16 / 0.23 |
| Regulated Pure Pursuit | 0.22 / 0.38 | 0.22 / 0.34 | 0.21 / 0.32 | 0.21 / 0.31 | 0.21 / 0.31 | 0.22 / 0.31 |
| Vector Pursuit | 0.22 / 0.34 | 0.21 / 0.30 | 0.21 / 0.31 | 0.21 / 0.33 | 0.21 / 0.32 | 0.22 / 0.34 |
| **ProxMPC** | 2.16 / 8.93 | 1.39 / 6.76 | 2.19 / 7.17 | 2.15 / 5.54 | 2.17 / 7.14 | 2.33 / 5.34 |
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
| **ProxMPC** | 4.4 / 59 | 6.4 / 59 | 6.8 / 59 | 6.7 / 59 | 9.1 / 59 |
| **ProxMPC (predictive)** | 4.5 / 60 | 6.3 / 60 | 5.8 / 60 | 6.6 / 60 | 8.0 / 60 |
| DWB | 8.6 / 60 | 8.9 / 60 | 8.9 / 60 | 8.8 / 60 | 9.1 / 60 |
| MPPI | 8.7 / 63 | 8.7 / 63 | 8.9 / 63 | 8.8 / 63 | 9.0 / 63 |

**Multi-obstacle cells:**

| Controller | dyn_multi | dyn_multi_noise | blind_0 | blind_1 | blind_2 | blind_3 |
| --- | --- | --- | --- | --- | --- | --- |
| Regulated Pure Pursuit | 4.0 / 55 | 4.1 / 56 | 4.2 / 55 | 4.2 / 55 | 4.0 / 55 | 4.2 / 55 |
| Vector Pursuit | 3.9 / 55 | 4.0 / 55 | 4.0 / 55 | 4.1 / 55 | 4.0 / 55 | 4.0 / 55 |
| Graceful | 4.0 / 56 | 4.2 / 56 | 4.1 / 56 | 4.2 / 56 | 4.2 / 56 | 4.1 / 56 |
| **ProxMPC** | 10.4 / 59 | 8.5 / 59 | 9.6 / 59 | 8.3 / 59 | 9.4 / 59 | 8.7 / 59 |
| **ProxMPC (predictive)** | 8.9 / 60 | 8.5 / 60 | 8.1 / 60 | 8.7 / 60 | 8.6 / 60 | 8.6 / 60 |
| DWB | 9.3 / 60 | 9.4 / 60 | 8.9 / 60 | 9.2 / 60 | 9.1 / 60 | 9.1 / 60 |
| MPPI | 9.0 / 63 | 9.0 / 63 | 9.1 / 64 | 8.9 / 63 | 9.0 / 64 | 9.0 / 63 |

> ProxMPC-predictive additionally spends ~1.0 % core and ~38.5 MB in the obstacle-tracker process, constant across every scenario - a small overhead the tables above exclude.

The compute comparison across the eleven cells:

- **ProxMPC's median advantage on the open cell roughly doubled, and the mechanism is traceable.** ProxMPC now computes a command in **0.34 ms median / 1.02 ms p95** on the open cell - **~7.8x faster than DWB and ~8.4x faster than MPPI at the median**, up from ~3.3x/3.5x in the previous campaign. DWB and MPPI, whose plugin code this repository does not touch, moved in the opposite direction (their own open-cell p50 rose slightly, consistent with the host-drift discussion in Section 8), so the ProxMPC-side improvement is not an artifact of a faster host. Two perf commits landed in the intervening work with a mechanism that plausibly explains it: dropping two per-cycle heap allocations from the reference-sampler-to-state-reference copy, and narrowing the costmap grid lock to just the cell reads (releasing it before the sort, clustering, and slot binding) - both changes sit in the base per-cycle path that runs even with zero obstacles, and neither has an analogue in DWB or MPPI's own code. The obstacle cells do not show the same doubling, because the obstacle-handling work (slot binding, ranking, and construction) that runs on top of that base path is unchanged or heavier - see the next point.
- **On the obstacle cells the advantage narrows to 1.2-2.7x, similar to before**, and the tail widened. ProxMPC's median obstacle-cell compute (1.05-2.33 ms) is close to the previous campaign's; the p95 tail grew on several cells (e.g. `dynamic_circle` 5.7 -> 7.75 ms, `dynamic_multi` 6.5 -> 8.93 ms). Two obstacle-handling fixes landed in the intervening work that changed which physical object occupies which QP slot at each horizon node (binding a slot to one object per cycle instead of letting it churn, and ranking slots by time-to-encounter instead of closest-approach-anywhere-on-horizon) - a plausible mechanism for a heavier worst-case QP on some cycles, since a persistently-bound slot can hold a harder constraint across more nodes than a churning one did. This is a plausible explanation for the *direction* of the tail growth, not a proven cause for any specific cell; see Section 5 and Section 8.
- **Process CPU tracks the same ordering**, compressed by the shared ~4 % costmap floor: ProxMPC **4.4 %** (open) rising to 10.4 % on the hardest cell, against DWB/MPPI's **8.6-9.4 %**; ProxMPC-predictive is steadier at 4.5-8.9 %. RSS is 55-64 MB throughout, MPPI the heaviest (63-64 MB), the geometric controllers the lightest (55-56 MB).
- **The geometric controllers are lighter still** - RPP, Graceful and Vector Pursuit compute in ~0.16-0.22 ms at ~3.9-4.2 % CPU, carrying no optimisation or constraint machinery; their own CPU reading dropped by a few tenths of a point almost everywhere relative to the previous campaign, in the same direction as ProxMPC's open-cell compute win but on plugin code this repository does not touch - see Section 8. ProxMPC is the cheapest of the controllers that solve a constrained optimisation each cycle, not the cheapest controller outright.
- **Prediction is cheaper than reactive on most obstacle cells, and now also on the open cell.** ProxMPC-predictive sits at or below reactive at the median on the open cell (0.35 ms vs 0.34 ms - now essentially tied, both having dropped) and on four of the six multi cells, with a consistently lighter p95 tail (4.98-5.60 ms against reactive's 5.34-8.93 ms on the multi cells). A single confirmed track is a smaller, cleaner constraint set than the reactive costmap's clustered wall cells. The predictive capability's real extra price is the companion tracker (~1.0 % core, ~38.5 MB), not the controller cycle.
- **The tail is heavier on the hard cells than before, and the worst case still overruns the budget.** On the dynamic and multi-obstacle cells ProxMPC's **p95 reaches 3.3-8.9 ms** (predictive 2.8-5.6 ms), against DWB's and MPPI's steadier 3.3-5.1 ms. Every p95 still sits well inside the 50 ms control budget, but **3 of the 110 ProxMPC-family runs recorded a single cycle above 50 ms, peaking at 255 ms** (both on multi-obstacle cells; see Section 5 for the exact population and cells). This is a direct consequence of rebuilding the QP factorization every cycle, plausibly compounded by the obstacle-slot changes above. ProxMPC is much cheaper on average but more variable at the tail than the sampling controllers.
- **Achieved command rate improved markedly on several ProxMPC cells that previously stalled.** ProxMPC's sampled rate on `blind_multi_0`, `blind_multi_1`, `blind_multi_2`, and `dynamic_multi_noise` rose from 15.7-18.6 Hz to 19.9-20.0 Hz - consistent with less time-to-goal and fewer stalls on exactly the cells where the endpoint-veto/unknown-space fix and the goal-tolerance-as-a-radial-bound fix (Section 8) would be expected to reduce spurious braking. Only 2 of the 435 b2 runs report a non-zero deadline-miss rate at all (Section 5), so the rate change is a stall/stop symptom, not a compute deadline miss.

## 5. ProxMPC solver profile (and a cross-check)

ProxMPC is the only controller that also publishes its *internal* solver telemetry ([`SolverDiagnostics`](../prox_mpc_msgs/README.md)):

| Metric | Open cell | With obstacle constraints active |
| --- | --- | --- |
| decorator compute p95 (whole `computeVelocityCommands`) | 1.02 ms | 3.3-8.9 ms |
| internal QP solve p95 | 0.91 ms | 3.2-8.8 ms |
| worst single-cycle solve (max) | 2.47 ms | up to 255 ms |
| deadline-miss rate (compute > 50 ms budget) | 0.0 % | 0.0 % mean, nonzero on 2 of 435 runs |
| infeasible rate (QP status != SOLVED) | 0.0 % | 0.0 % mean, nonzero on 1 of 110 ProxMPC-family runs |
| mean SQP iterations | 1.00 | 1.00-1.01 |

The two independent instruments agree on the open cell: the QP solve (0.91 ms p95) accounts for almost all of the measured per-cycle compute (1.02 ms p95), the small remainder being the costmap reduction and the exact footprint veto.
The worst single open-cell cycle stays **~20x inside** the 50 ms budget, which is why ProxMPC holds 20 Hz with 0 % deadline-miss and 0 % infeasible.
With the in-loop obstacle constraints active the per-cycle cost rises to a p95 of 3.3-8.9 ms across the ten obstacle scenarios (more active half-plane rows on the dynamic and multi cells than the static one, and a heavier tail than the previous campaign - see Section 4).
Two populations matter here, and this document reports them separately rather than blending them as the previous campaign's text did: across **all 435 b2 runs**, deadline-miss rate is nonzero on exactly **2** (`dynamic_multi`/ProxMPC-pred repeat 2 at 0.53 %, `blind_multi_0`/ProxMPC repeat 3 at 0.27 %); across the **110 ProxMPC-family runs** (the only runs where `solve_ms_max` is populated), a single cycle exceeded the 50 ms budget on exactly **3**, peaking at **255 ms** on `dynamic_multi`/ProxMPC-pred repeat 2, followed by 229 ms on `blind_multi_0`/ProxMPC repeat 3 and 65 ms on `blind_multi_1`/ProxMPC-pred repeat 2.
Each cycle rebuilds the QP factorization from scratch, so a hard cycle pays the full symbolic cost; the two obstacle-slot fixes described in Section 4 are a plausible but unproven contributor to which cycles land in that tail.
The solver never fails to return a feasible QP solution in time on 434 of 435 runs; where reactive ProxMPC misses on a multi cell (Section 6.4) it is overwhelmingly the *geometry* of the convex keep-out that fails, not the solver's timing or feasibility.

## 6. Obstacle avoidance

The open cell cannot show avoidance, so these scenarios place obstacles across the path and let every controller perceive them through the same local and global `obstacle_layer`.
Because the kinematic plant has no collision physics, "reaching the goal" alone does not prove avoidance - a controller can drive straight through an obstacle and still "succeed" - so the discriminating metric is the **obstacle gap** (robot-disc to obstacle-disc; negative = overlap = would-be collision).

The global costmap carries an `obstacle_layer`, so NavFn routes the global plan around obstacles for every controller, and every cell uses one uniform obstacle size (`clearance 0.45`). The single-obstacle cells (Section 6.1-6.2) are within reach of the whole field and separate the controllers on *margin* and on the orbiting case; the multi-obstacle cells (Section 6.4) are where collision counts spread. Two comparisons carry the result: ProxMPC against the other optimisation- and sampling-based controllers - **DWB** and **MPPI**, its per-cycle-cost peers - and ProxMPC against the geometric pursuit controllers - **RPP**, **Graceful** and **Vector Pursuit** - which trade avoidance headroom for near-zero compute.

### 6.1 Static obstacle (`static_box`, mode b2)

A 0.25 m box placed **dead-centre** on the path at `(0, 0)`; with the obstacle-routed global plan every controller has a path that already bends around it.
The last two columns carry the per-scenario **cost of that avoidance** (the same decorator/process instruments as Section 4), so clearance and its price are read together.

| Controller | success | obstacle gap [m] | collision | compute p50/p95 [ms] | CPU [%] |
| --- | --- | --- | --- | --- | --- |
| **ProxMPC** | 5/5 | **+0.350 ± 0.009** | **0/5** | 1.15 / 3.31 | 6.4 |
| **ProxMPC (predictive)** | 5/5 | +0.298 ± 0.001 | **0/5** | 0.94 / 2.77 | 6.3 |
| Regulated Pure Pursuit | 5/5 | +0.223 ± 0.021 | 0/5 | 0.22 / 0.41 | 3.9 |
| Graceful | 5/5 | +0.212 ± 0.004 | 0/5 | 0.16 / 0.27 | 4.0 |
| MPPI | 10/10 | +0.208 ± 0.003 | 0/10 | 2.85 / 4.17 | 8.7 |
| DWB | 5/5 | +0.087 ± 0.008 | 0/5 | 2.76 / 4.43 | 8.9 |
| Vector Pursuit | **3/5** | +0.212 ± 0.086 | 0/5 | 0.21 / 0.37 | 4.0 |

Every controller clears the static box collision-free, so the discrimination is in the *margin*, not in whether they hit.
**ProxMPC keeps the largest margin of the field - +0.350 m** (unchanged from the previous campaign within measurement noise) - by swerving wide and recovering, the behaviour the in-loop keep-out constraint produces, at **~2.4x less per-cycle compute than DWB and MPPI**.
Predictive ProxMPC holds a close +0.298 m: the tracker reports the box at ~0 velocity, so the hybrid fill treats it much like the reactive path.
Among the cost-class peers, **DWB cuts it finest at +0.087 m** - its soft `BaseObstacle` critic rides close to the inflated edge - while MPPI holds +0.208 m; among the geometric controllers RPP and Graceful hold ~+0.21-0.22 m.
**Vector Pursuit now completes 3 of 5 runs (was 0/5 in the previous campaign)**, with no source change on either the plugin or the harness: its own forward-collision check (`use_collision_detection`) is close enough to a decision boundary here that bring-up and discovery timing alone flip the outcome run to run. The 2 stopped-short runs report a gap of ~0.31 m (a stopped-short distance, not a steered avoidance); the 3 completed runs steer around at ~0.13-0.18 m. The blended +0.212 ± 0.086 m in the table above mixes both behaviours and should be read with that in mind, not as a single steering margin.

### 6.2 Single moving obstacles (mode b2, reactive)

Three single-mover scenarios - a patrol crossing the path forward (`dynamic_line_forward`) and backward (`dynamic_line_backward`) at 0.6 m/s, and an obstacle orbiting near mid-path (`dynamic_circle`) at 0.5 m/s.
Most of the field still clears these reactively; the orbiting case is where the field splits, and it splits differently than before:

| Scenario | metric | ProxMPC | ProxMPC-pred | DWB | MPPI | RPP | Graceful | VecPursuit |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| line_forward | collision | 0/5 | 0/5 | 0/5 | 0/10 | 0/5 | 0/5 | 0/5 |
| | min gap [m] | +0.50 | +0.35 | +0.47 | +0.55 | +0.40 | +0.32 | +0.46 |
| line_backward | collision | 0/5 | 0/5 | 0/5 | 0/10 | 0/5 | 0/5 | 0/5 |
| | min gap [m] | +0.54 | +0.39 | +0.49 | +0.60 | +0.41 | +0.30 | +0.46 |
| circle | collision | **3/5** | 0/5 | **5/5** | 0/10 | 1/5 | **5/5** | 0/5 |
| | min gap [m] | **-0.00** | +0.10 | **-0.28** | +0.17 | +0.05 | **-0.07** | +0.14 |

**On the two straight patrols every controller still keeps a real margin (0 collisions)**, at gaps a few centimetres tighter across the board than the previous campaign for most controllers - a small, uniform shift consistent with run-to-run timing rather than a directed change (Section 8).
**On the orbiting obstacle, reactive ProxMPC now collides 3 of 5 runs (was 0/5), at a mean gap of essentially zero (-0.00 m, individual runs between -0.026 m and +0.022 m)** - a real narrowing of typical clearance from the previous campaign's comfortable +0.12 m to a margin so thin that whether any given run registers as a collision is itself noise-sensitive, much like the multi-obstacle cells in Section 6.4. No specific commit is isolated as the cause: the obstacle-slot and plan-projection changes in the intervening work are plausible contributors, but the shift is inside the same "boundary-sensitive" pattern documented for Vector Pursuit's static-box flip (Section 6.1) and the multi-obstacle collision counts, not a clean regression to a large negative gap. Predictive ProxMPC is unaffected (0/5, +0.10 m) and DWB (5/5, unchanged) and Graceful (5/5, was 4/5) remain the field's clear failures here - both still commit to a trajectory the obstacle then orbits into, and the routed global plan (computed once against the obstacle's initial footprint) still does not track the moving object.
Compute stays in the Section 4 ordering: ProxMPC 1.5-2.4x lighter than DWB and MPPI at the median while carrying the active keep-out rows, with a p95 tail up to ~7.75 ms on the orbit cell specifically (the highest single-obstacle p95 in this campaign).

### 6.3 Ground-truth predictive confirmation (mode b1)

One controller-vs-tracker separation is worth recording: fed the *correct* future obstacle positions (mode b1, the engine knows the exact trajectory, orbit included), ProxMPC solves every single moving-obstacle case for both vehicle models:

| Scenario | Model | success | cross-track RMS [m] | time-to-goal [s] |
| --- | --- | --- | --- | --- |
| dynamic_circle | Unicycle / Bicycle | 5/5 / 5/5 | 0.005 / 0.021 | 4.57 / 4.52 |
| dynamic_line_forward | Unicycle / Bicycle | 5/5 / 5/5 | 0.045 / 0.058 | 4.48 / 4.58 |
| dynamic_line_backward | Unicycle / Bicycle | 5/5 / 5/5 | 0.048 / 0.044 | 4.53 / 4.59 |
| static_box | Unicycle / Bicycle | 5/5 / 5/5 | 0.000 / 0.000 | 4.52 / 4.60 |

> This table is **not** part of the re-validation: mode (b1) was out of scope for this pass (see the caveat at the top of this document) and still reflects the `v1.0.0` baseline. It is retained because it is a ceiling check on the controller's own capability, not the operative reactive/predictive result, and is pending its own re-run.

With ground-truth prediction ProxMPC reaches the goal on **every** single moving-obstacle run for both a 3-state unicycle and a 4-state bicycle (40/40, static box included), the orbiting obstacle included, driven by the same plugin with one parameter changed (`model_plugin`).
The single-obstacle dynamic case is fully within the controller's reach given a correct predictor. The reactive costmap now shows one real failure mode on the single movers (the orbit, Section 6.2) that it did not before, so this ground-truth ceiling check is worth re-running alongside the rest of mode (b1) to confirm it still holds on the current tree.

### 6.4 Multiple simultaneous moving obstacles - where the field separates

The single-obstacle cells discriminate a little more than before (Section 6.1-6.2), but the collision comparison still rests on the **multi-mover** set: `dynamic_multi` and `dynamic_multi_noise` (two hand-built simultaneous movers, the latter with sensor noise), and `blind_multi_0`-`blind_multi_3` (blind, author-independent two-mover cells, generated without hand-screening).
Six cells, 30 runs per controller (60 for MPPI), collision per cell:

| Scenario | ProxMPC | ProxMPC-pred | DWB | MPPI | RPP | Graceful | VecPursuit |
| --- | --- | --- | --- | --- | --- | --- | --- |
| dynamic_multi | 3/5 | 1/5 | 1/5 | 4/10 | 0/5 | 3/5 | 2/5 |
| dynamic_multi_noise | 4/5 | 0/5 | 2/5 | 3/10 | 0/5 | 4/5 | 1/5 |
| blind_multi_0 | 3/5 | 0/5 | 0/5 | 10/10 | 2/5 | 2/5 | 5/5 |
| blind_multi_1 | 2/5 | 0/5 | 0/5 | 6/10 | 1/5 | 2/5 | 0/5 |
| blind_multi_2 | 5/5 | 0/5 | 5/5 | 0/10 | 4/5 | 5/5 | 5/5 |
| blind_multi_3 | 1/5 | 0/5 | 0/5 | 7/10 | 0/5 | 0/5 | 2/5 |
| **aggregate collisions** | **18/30** | **1/30** | **8/30** | **30/60** | **8/30** | **16/30** | **16/30** |
| **aggregate success** | 29/30 | 30/30 | 30/30 | 60/60 | 30/30 | 30/30 | 29/30 |
| **median margin [m]** | -0.047 | **+0.157** | +0.098 | +0.002 | +0.124 | -0.006 | -0.020 |
| **runs within 0.15 m of contact** | 28/30 | **13/30** | 21/30 | 51/60 | 19/30 | 30/30 | 25/30 |

**Read the margin rows, not the collision counts.** These cells are deliberately marginal: 63-100 % of all runs now finish within 0.15 m of contact (up from 40-80 %), so a few centimetres of scheduling jitter flips a near-miss into a collision. The previous campaign already documented a single cell returning 1/5, 5/5 and 2/5 collisions on three runs of the same binary; this re-validation adds a fourth data point of the same kind on `blind_multi_2`/ProxMPC, which swung from 1/5 in the previous campaign to 5/5 here, on a different tree. The median closest approach is the more stable statistic and is the discriminator this section rests on.

By margin the leader is unchanged: **predictive ProxMPC still leads the field, now at +0.157 m** (was +0.190 m), ahead of RPP (+0.124, essentially flat), DWB (+0.098, up from +0.080), MPPI (+0.002, down sharply from +0.048), Graceful (-0.006, up slightly from -0.013) and Vector Pursuit (-0.020, down from +0.024, now negative). Predictive ProxMPC is also the only controller with fewer than half its runs inside the marginal band (13/30). **Reactive ProxMPC remains the field's narrowest at -0.047 m**, though the gap to its nearest peers (Vector Pursuit -0.020, Graceful -0.006) closed substantially from the previous campaign's -0.118 m against a next-worst of -0.013 m.

Prediction remains a large, and now cleaner, effect: it moves ProxMPC from the field's narrowest margin to its widest, and from 18/30 collisions to 1/30 - a sharper collision-count improvement than the previous campaign's 20/30 to 6/30, even as the margin advantage itself narrowed slightly.

The cell where ProxMPC's reactive keep-out is weakest by margin is now `blind_multi_2` (median -0.197 m), not `blind_multi_0` as previously measured (`blind_multi_0`'s reactive-ProxMPC median moved to -0.046 m). `blind_multi_2` is also DWB's worst cell (-0.236 m), RPP's worst (-0.084 m), and Graceful's worst (-0.144 m) - four of seven controllers now find their hardest geometry on the same cell, which was not true of the previous campaign's per-cell breakdown. Predictive ProxMPC's worst cell, `blind_multi_1` at +0.120 m, is still comfortably positive - every one of its six per-cell medians is positive this campaign, which was not reported before. No controller is collision-free across the blind cells, and the winner is still cell-dependent - RPP clears three cells outright but collides 4/5 on `blind_multi_2`; DWB clears three cells but collides 5/5 on `blind_multi_2`.

The per-cell breakdown for every controller is in [`prox_mpc_benchmark/README.md`](../prox_mpc_benchmark/README.md) and the raw `results/`.
In summary: ProxMPC keeps the largest static margin (Section 6.1), now shows one real reactive weak point on a single mover (the orbit, Section 6.2) that it did not before, and still leads its sampling peer and matches or leads every other controller on the multi-mover set at a fraction of the sampling controllers' compute (Section 4), with prediction closing most of the gap the reactive path leaves open.

## 7. Real-stack validation (Gazebo)

To confirm the plugin behaves under the full production stack and not only against a kinematic plant, ProxMPC also runs in mode (a): Gazebo Harmonic physics, a TurtleBot3 waffle, AMCL localization, costmaps, and the complete Nav2 velocity chain, headless.
It reaches the goal (`SUCCEEDED`), tracks the path to **5.6 mm cross-track RMS**, and taps its diagnostics through the real `controller_server` (p95 solve 0.438 ms, 0 % deadline-miss, 0 % infeasible) - the same profile measured on the plant, with real sensor and physics noise.

> Mode (a) was not re-run in this pass (it needs a working GPU driver, and the host used for every campaign to date renders Gazebo in software, which starves the control loop). This section still reflects the `v1.0.0` baseline and is pending its own re-validation; the b1/b2 numbers elsewhere in this document are unaffected, since neither uses Gazebo.

## 8. Threats to validity

- **This re-validation surfaces a systematic host-timing drift between the two measurement sessions, visible on plugin code this repository never touched.** DWB, MPPI, Regulated Pure Pursuit, Graceful, and Vector Pursuit are stock Nav2/community binaries (confirmed at the same versions as the previous campaign - Nav2 1.3.12, `vector_pursuit_controller` 2.0.0), and the shared harness code (`scan_simulator.cpp`, `kinematic_plant.cpp`, every `config/` file this campaign used) is byte-identical to the previous campaign. Yet DWB and MPPI's per-cycle compute p95 rose by roughly 0.6-1.7 ms on almost every scenario, and RPP/Graceful/Vector Pursuit's CPU reading dropped by roughly 0.3-0.6 points almost everywhere. Since none of that code changed, the most defensible explanation is background load, thermal/turbo state, or scheduler variance between the two sessions on the same physical host - not a software effect. This bounds how confidently any *ProxMPC-side* shift of similar magnitude can be attributed to the intervening code changes versus the same host effect; the two per-cycle open-cell wins reported in Section 4 are large enough (halving, not shifting by tens of percent) and directionally opposite to the host drift to stand on their own, but smaller shifts should be read with this in mind.
- **Two run outcomes flipped between campaigns on plugin code that did not change, and are boundary-sensitive rather than regressions.** Vector Pursuit's static-box completion (0/5 to 3/5, Section 6.1) and reactive ProxMPC's orbit collision rate (0/5 to 3/5, Section 6.2) both moved substantially without a source change that explains them (Vector Pursuit's plugin is unmodified; the orbit is a single-obstacle cell where the multi-obstacle slot-ranking fixes do not obviously apply). Both are treated here as evidence that these specific decision points sit close to a boundary that run-to-run timing can cross, not as attributed regressions.
- **The obstacle-routed global plan does much of the single-obstacle avoidance** (Section 6). The shared global `obstacle_layer` makes NavFn route around obstacles for every controller - deliberately equal, but it favours the geometric path-followers (RPP, Graceful, Vector Pursuit) and DWB, which track the global detour closely. The single-obstacle cells therefore separate the field on margin and on the orbiting case, not on collision counts; the multi-mover cells (Section 6.4) carry the collision comparison.
- **The multi-obstacle field is close and the sample is small** (Section 6.4). 63-100 % of runs across the six cells now finish within 0.15 m of contact (up from 40-80 % in the previous campaign), so the per-cell collision counts swing by more than the gap between most controllers - `blind_multi_2`/ProxMPC alone moved from 1/5 to 5/5 across the two campaigns. The trustworthy signals are the class comparisons - predictive ProxMPC leads the field by margin, MPPI's margin fell sharply, and `blind_multi_2` is now the hardest cell for four of seven controllers - not the specific per-cell collision ordering, which is why Section 6.4 rests on the median margin.
- **`blind_multi_2` is now the tightest cell for most of the field, `blind_multi_0` no longer stands out for ProxMPC specifically** (Section 6.4). This is a genuine, measured shift from the previous campaign's narrative, but it is reported as observed rather than attributed: nothing in the intervening diff singles out one cell's geometry, and the multi-obstacle collision counts are already documented as noisy at this sample size.
- **Collision uses a strict min-gap** - any instant of disc overlap over the whole run counts as a collision, so a brief graze is flagged the same as a harder hit; the reported min gap distinguishes the two.
- **Achieved command rate improved on several previously-stalling ProxMPC cells** (Section 4), consistent with, but not conclusively proven by, the endpoint-veto/unknown-space and goal-tolerance fixes in the intervening work; only 2 of 435 b2 runs report a non-zero deadline-miss rate at all (Section 5), so this is a stall/stop symptom, not a compute deadline effect.
- **Resources are measured on the x86 host** above (confirmed identical hardware, CPU model, and OS version to every prior campaign via `/proc/cpuinfo` and `/etc/os-release`), and process CPU/RAM is per-process (includes the shared costmap); the per-cycle compute (Section 4) is the cleaner controller-only measure, and the ranking is what transfers. The pure-geometric controllers are genuinely lighter than ProxMPC - ProxMPC is the cheapest of the controllers that solve a constrained optimisation each cycle, not the cheapest outright.
- **MPPI is stochastic with no exposed seed** (Nav2 Jazzy), so its per-scenario variance is real; it runs 10 repeats per obstacle cell (5 on the open cell) to better characterise that variance, but this does not pin it down exactly. Its multi-obstacle median margin fell from +0.048 to +0.002 this campaign - within the same "close field, small sample" caveat as the rest of Section 6.4.
- **Determinism vs. physics.** Modes (b1)/(b2) are deterministic plants, so their near-zero geometric std is reproducibility, not a noise estimate; mode (a) carries real Gazebo variance, and `dynamic_multi_noise` adds a Gaussian range-noise model on the scan. Mode (b1) and mode (a) were not re-run in this pass (Sections 6.3, 7) and still reflect the `v1.0.0` baseline.
- **The demonstration videos under `prox_mpc_benchmark/doc/media/` were not re-recorded for this re-validation.** Given the orbit and static-box outcome shifts above, at least the `dynamic_circle` and `static_box` grids may no longer show the exact behaviour this document now reports for ProxMPC and Vector Pursuit on those cells. Whether and when to re-record is an open question for the maintainer, not decided by this pass.

## 9. Conclusion on ProxMPC

On a like-for-like, fairly-tuned suite, re-validated on the post-audit-chain tree, ProxMPC matches the best stock Nav2 controller on tracking, runs markedly lighter per cycle than the optimisation- and sampling-based controllers - now by a wider margin on the open cell specifically - holds the largest static-obstacle margin, and leads or matches every controller on moving obstacles, with one single-obstacle cell (the orbit) now showing a real weak point it did not show before:

- **Per-cycle compute and process resources (measured, all eleven cells):** ProxMPC computes a command in **0.34 ms median / 1.02 ms p95** on the open cell - **~7.8x lighter than DWB and ~8.4x than MPPI**, up from ~3.3x/3.5x, traceable to two per-cycle allocation/locking fixes in the intervening work (Section 4) - and stays **1.2-2.7x lighter at the median on every obstacle cell**. Process CPU is 4.4-10.4 % against their 8.6-9.4 %. Deadline misses are essentially absent (2 of 435 b2 runs non-zero). The tail is the weak point and grew somewhat: p95 reaches 3.3-8.9 ms on the hard cells against the samplers' steadier 3.3-5.1 ms, and **3 of 110 ProxMPC-family runs recorded one cycle above the 50 ms budget, peaking at 255 ms** - a direct consequence of rebuilding the QP factorization every cycle, plausibly compounded by changes to which obstacle occupies which QP slot. The geometric controllers (RPP, Graceful, Vector Pursuit) are lighter still, carrying no model, constraints, or horizon optimisation; against them ProxMPC's premium is ~0.5-6.5 % of one core.
- **Tracking:** sub-millimetre (0.0001 m RMS), on par with the field, 100 % success.
- **Static-obstacle avoidance:** **ProxMPC keeps the largest real margin, +0.350 m**, unchanged within noise from the previous campaign - ahead of its cost-class peers MPPI (+0.208 m) and DWB (+0.087 m), and of the geometric RPP and Graceful (~+0.21-0.22 m). Vector Pursuit now completes 3/5 runs (was 0/5), a boundary-sensitive flip on unmodified plugin code rather than a change in either controller.
- **Single moving obstacles:** the two straight patrols are still cleared by the whole field; the orbiting obstacle is no longer clean for reactive ProxMPC (3/5 collisions at an essentially-zero mean margin, was 0/5), while DWB (5/5) and Graceful (5/5, was 4/5) remain the field's clear failures there. This is the one genuinely new weak spot this re-validation found, and it is not attributed to a specific commit (Section 6.2, Section 8). Ground-truth prediction (mode b1, not re-run this pass) previously solved all three single moving-obstacle cases 30/30 for both a unicycle and a bicycle; that ceiling check is worth re-confirming given the new reactive result.
- **Multiple simultaneous moving obstacles:** measured by median closest approach - the stable statistic, since the collision *count* on these deliberately marginal cells swings by more than the gap between controllers, and now more than before (63-100 % of runs land within the 0.15 m marginal band, up from 40-80 %) - **predictive ProxMPC still leads the entire field, now at +0.157 m**, ahead of RPP (+0.124), DWB (+0.098), MPPI (+0.002, down sharply), Graceful (-0.006) and Vector Pursuit (-0.020, now negative), with only 13 of 30 runs inside the 0.15 m marginal band against 19-30 for the others. **Reactive ProxMPC remains the field's narrowest at -0.047 m**, though its gap to the next-worst controllers closed substantially. The cell where the convex keep-out is weakest is now `blind_multi_2`, not `blind_multi_0` as previously measured - a genuine, observed shift with no single traceable cause.
- **One controller, many vehicles:** the identical plugin drives a unicycle and a bicycle by configuration alone (mode b1, not re-run this pass).

Against DWB and MPPI - its per-cycle-cost peers - ProxMPC is now roughly eight times lighter per cycle on the open cell at equal tracking accuracy (up from four), holds a larger static margin, and leads MPPI on multiple movers by a wider margin than before, though it no longer cleanly clears the orbiting obstacle DWB also misses. Against the geometric pursuit controllers it holds larger avoidance margins for a small CPU premium. Its clearest weak points are the tightest two-mover cell (which moved from `blind_multi_0` to `blind_multi_2`) and, newly, the single orbiting obstacle - both observed in this re-validation, neither attributed to a specific change in the intervening work.

---

*Reproduce:* the open cell and the mode-(b2) reactive obstacle scenarios with `ros2 run prox_mpc_benchmark run_nav2.py --scenario <nav2_open|static_box|dynamic_circle|dynamic_line_forward|dynamic_line_backward|dynamic_multi|dynamic_multi_noise|blind_multi_0|blind_multi_1|blind_multi_2|blind_multi_3>`, the **predictive b2** runs (real tracker) by adding `--controllers proxmpc_pred` to the same command, the mode-(b1) ground-truth predictive results with `ros2 run prox_mpc_benchmark run_matrix.py --modes b1`, and the tables with `ros2 run prox_mpc_benchmark aggregate.py`. See [`prox_mpc_benchmark/README.md`](../prox_mpc_benchmark/README.md).

## License

[Apache-2.0](../LICENSE).
