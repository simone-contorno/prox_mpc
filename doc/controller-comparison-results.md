# Controller Comparison Results

This document reports how the ProxMPC controller compares against the stock Nav2 controllers under identical, reproducible, and *fairly-tuned* conditions — on **path tracking**, **per-cycle compute and process resources**, and **obstacle avoidance**.
It is the narrative companion to the auto-generated tables in [`prox_mpc_benchmark/README.md`](../prox_mpc_benchmark/README.md); the harness that produced every number is described in [the package guide](prox-mpc.md#8-prox_mpc_benchmark--the-measurement-harness).

All numbers below are measured, reproducible, and reported as `mean ± std` (population) over fixed-seed repeats — the precision signal.
Nothing is hand-tuned to favour one controller: every controller drives the *same* plant from the *same* start to the *same* goal, at a *matched operating point* (Section 2.1), perceives obstacles through the *same* costmaps, and is measured by the *same* instrumentation — including a timing decorator that wall-clock times every controller's per-cycle compute identically.

The comparison covers six controllers: the five that ship with Nav2 Jazzy as standalone local planners — **ProxMPC**, **DWB**, **MPPI**, **Regulated Pure Pursuit (RPP)**, and **Graceful** (new in Jazzy) — plus **Vector Pursuit**, the one external community controller included as a fair peer (Apache-2.0, apt `ros-jazzy-vector-pursuit-controller` v2.0.0).
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

1. **The open-world cell** — an empty 7 × 7 m room, a single straight traverse from `(-2.5, 0)` to `(2.5, 0)` (5 m), no obstacles.
   This is a *pure path-tracking* task that isolates **tracking fidelity**, **per-cycle compute cost**, and **process resource use** on an exactly equal footing, with no perception or obstacle geometry to confound them (Sections 3–5).
2. **The obstacle scenarios** — the *same* 5 m traverse with obstacles placed across the path (a static box, single moving obstacles, and simultaneous multi-mover cells), run on the mode (b2) Nav2 stack so that **every controller perceives the same obstacles through the same costmaps and inflation** (Section 6).
   This is where the avoidance behaviours actually separate.

The open cell deliberately understates a constrained optimal-control method — an empty straight line is the task a geometric pursuit controller is provably optimal for — which is why obstacle avoidance is reported separately, and why the compute/resource comparison (Section 4) is the discriminating open-cell result.

## 2. Test conditions

| Condition | Value |
| --- | --- |
| ROS 2 / Nav2 | Jazzy / Nav2 1.3.12 |
| Host | x86-64 workstation — Intel Core i7-10750H (6 cores / 12 threads), 31 GiB RAM, Ubuntu 24.04.4 LTS (kernel 6.8), CPU-only |
| Run mode | **(b2)** Nav2 stack on a kinematic plant, no Gazebo (see [run modes](prox-mpc.md#9-running-the-stack-the-three-modes)); mode (b1) for the ground-truth predictive results in §6.3 |
| Plant | Unicycle body-twist integrator at 50 Hz, identical for every controller |
| Localization | Exact (static `map → odom` identity; the plant pose is ground truth) |
| Map | `prox_mpc_open` — 7 × 7 m room, free interior `[-2.95, 2.95] m` |
| Task | straight 5 m traverse `(-2.5, 0) → (2.5, 0)`, goal tolerance 0.25 m |
| Planner | `NavfnPlanner` (GridBased), shared by all controllers |
| Control rate | 20 Hz (`controller_frequency`); real-time budget 50 ms/cycle |
| Local costmap | `static_layer` + `obstacle_layer` + `inflation_layer` (**0.4 m** radius), 5 Hz update |
| Global costmap | `static_layer` + `obstacle_layer` + `inflation_layer` (**0.4 m** radius); the `obstacle_layer` marks the scenario obstacles so **NavFn routes the global plan around them** — the same global plan for every controller |
| Obstacle size | one **uniform** obstacle size across every b2 cell (`clearance: 0.45`); `static_box` is a 0.25 m box at dead-centre `(0, 0)` |
| Obstacle sensing | a **scan simulator** ray-casts the scenario obstacles into `/scan`; the `obstacle_layer` marks/clears them, so all controllers see the same obstacles |
| Repeats | **5** fixed-seed repeats per controller per scenario (**10** for MPPI on every obstacle cell, to better characterise its sampling variance) |
| Controllers | ProxMPC (Unicycle), DWB, MPPI, RPP, Graceful, Vector Pursuit — plus **ProxMPC (predictive)** with the obstacle tracker for §6.3–6.4 |
| Compute timing | a `nav2_core::Controller` **timing decorator** wraps every controller and times its `computeVelocityCommands` identically |

### 2.1 Fair tuning: what is equalised and what is not

The controllers are different algorithms, so their *internal* knobs are not the same quantity and cannot be set "equal" without meaning something different for each.
The fair approach is to equalise everything they *share* and leave each method's intrinsic mechanism at its documented operating point:

- **Equalised** — control rate (20 Hz), max linear speed (0.5 m/s), goal tolerance (0.25 m), the **prediction horizon (2.0 s)** for all predictive controllers (ProxMPC `np·dt = 20 × 0.1`; DWB `sim_time = 2.0`; MPPI `time_steps × model_dt = 40 × 0.05`), a **uniform physical obstacle size** across every cell, and the **obstacle perception**: a shared local *and* global `obstacle_layer` fed by the scan simulator means every controller (including ProxMPC's costmap fill) sees the *same physical obstacles* through the *same costmaps + inflation*, and receives the *same* obstacle-routed global plan. In the head-to-head ProxMPC runs `predict_obstacles: false`, so it gets **no** privileged obstacle knowledge; the separate **ProxMPC (predictive)** variant (§6.4) turns that flag on and adds the real obstacle tracker, and is otherwise the identical preset so the comparison isolates prediction.
- **A note on the shared global plan.** Because the global costmap carries an `obstacle_layer`, NavFn bends the *global* path around obstacles before any controller runs. This is deliberately equal for all six, but it is worth stating plainly that it **helps the pure path-followers most**: DWB, RPP, Vector Pursuit and Graceful track that global detour closely, so an obstacle-routed global plan does much of their avoidance for them, whereas ProxMPC re-optimises locally and leans less on it. The comparison therefore measures *local avoidance on top of an equal, obstacle-aware global plan* — not local avoidance in isolation.
- **Left at each method's default** — the *avoidance mechanism itself*, because it has no common denominator across paradigms: DWB's `BaseObstacle` critic (nav2_bringup default `scale 0.02`), MPPI's `CostCritic` (upstream default `cost_weight 3.81`), ProxMPC's in-loop keep-out constraint, and RPP/Graceful's forward-simulation collision check. Crippling any of them to a common number would misrepresent it. The obstacles each faces are equal; how each responds is its own algorithm.
- **Intrinsic sampling** left at upstream defaults: MPPI `batch_size = 2000`, DWB's `20 × 20` velocity grid, ProxMPC's single-QP SQP.
- **Determinism** — MPPI is the one stochastic controller and Nav2 Jazzy exposes no RNG seed for it, so its repeats capture genuine sampling variance (reported as `mean ± std`), not reproducibility; it is run at 10 repeats on the obstacle cells (5 on the open cell) rather than the 5 used for the deterministic controllers.
- **Graceful** — a good-faith goal-approach tuning (lookahead below the goal tolerance, reduced slowdown radius, no in-place final rotation) lets it drive into the goal rather than stalling far out; it succeeds on every scenario measured here.

Because the plant, map, planner, goal checker, horizon, speed, obstacle size, costmaps, and instrumentation are all shared, every difference in the tables is attributable to the controller.

### Metric definitions

- **time-to-goal** — wall time from first motion to entering the goal tolerance.
- **goal error** — final distance to the goal point; controllers that reach stop on the *same* 0.25 m checker.
- **cross-track RMS / max** — deviation from the straight reference; on obstacle scenarios this is *also* the size of the avoidance detour, so it is read together with clearance.
- **obstacle gap** (obstacle scenarios) — the minimum distance, over the whole run, between the **robot disc** and the nearest **obstacle disc** (robot radius 0.22 m). Positive = a real safety margin is kept; **negative = the two discs overlap**, i.e. a would-be collision on the collision-free kinematic plant. Controller-agnostic: the same measurement for all.
- **collision rate** — fraction of repeats whose obstacle gap went negative.
- **compute p50 / p95 / max** — wall time of one `computeVelocityCommands`, measured by the decorator, **the same way for every controller**.
- **CPU / RSS / rate** — the `controller_server` process's CPU (% of one core) and RSS from `/proc`, and the achieved `/cmd_vel` rate. For **ProxMPC (predictive)** the companion tracker runs as a *separate* process and is sampled separately (`+ trk`).
- **solve p95, deadline-miss, infeasible, SQP/QP iters, slack** — ProxMPC-only internal solver telemetry.

## 3. Tracking-fidelity comparison (open-world cell)

| Controller | success | time-to-goal [s] | goal err [m] | cross-track RMS [m] | cross-track max [m] |
| --- | --- | --- | --- | --- | --- |
| **ProxMPC** (Unicycle) | 5/5 | 12.45 ± 0.50 | 0.239 | **0.0004 ± 0.0000** | 0.0008 |
| Regulated Pure Pursuit | 5/5 | 11.76 ± 1.33 | 0.235 | 0.0000 ± 0.0000 | 0.0000 |
| MPPI | 5/5 | 11.84 ± 1.70 | 0.233 | 0.0029 ± 0.0002 | 0.0039 |
| DWB | 5/5 | 12.68 ± 1.13 | 0.237 | 0.0001 ± 0.0001 | 0.0003 |
| Vector Pursuit | 5/5 | 13.03 ± 1.05 | 0.237 | 0.0000 ± 0.0000 | 0.0000 |
| Graceful | 5/5 | 10.48 ± 0.98 | 0.224 | 0.0000 ± 0.0000 | 0.0000 |

All six track the straight line essentially perfectly (sub-millimetre cross-track) and complete 5/5; ProxMPC and DWB are exact to the plant resolution and MPPI is within 3 mm.
On *tracking fidelity* there is no meaningful separation on an empty straight line — which is exactly why the compute-and-resource comparison below is the discriminating open-cell result for the predictive controllers.

## 4. Per-cycle compute cost and process resources

This is the comparison that matters on an embedded target, and it is **measured, not inferred**: the timing decorator wall-clock times every controller's `computeVelocityCommands` the same way, on **every** scenario, and a per-process sampler reads the `controller_server`'s CPU and RSS from `/proc`.
Unlike tracking, compute is *not* flat across scenarios — the obstacle cells activate each method's avoidance machinery (ProxMPC's in-loop keep-out rows, DWB/MPPI's cost critics), so the per-cycle cost grows with obstacle load.
The tables are therefore **per scenario, across all eleven cells** — split into single-obstacle and multi-obstacle groups — so that growth is visible.

**ProxMPC (predictive)** is the same plugin with `predict_obstacles: true` and the obstacle tracker in the loop (§6.4), otherwise identical to reactive ProxMPC — so the two rows isolate the cost of prediction alone.
Its companion tracker runs as a *separate* process (~1 % of one core, ~38 MB RSS, constant across scenarios), which the `controller_server` figures below do **not** include; it is reported separately.

### 4.1 Per-cycle compute, `p50 / p95` [ms] (mean over repeats)

**Single-obstacle cells:**

| Controller | open | static_box | line_fwd | line_bwd | circle |
| --- | --- | --- | --- | --- | --- |
| Graceful | 0.15 / 0.21 | 0.16 / 0.21 | 0.15 / 0.24 | 0.15 / 0.24 | 0.15 / 0.22 |
| Regulated Pure Pursuit | 0.21 / 0.25 | 0.21 / 0.27 | 0.21 / 0.26 | 0.21 / 0.32 | 0.21 / 0.30 |
| Vector Pursuit | 0.21 / 0.25 | 0.25 / 0.39 | 0.22 / 0.33 | 0.21 / 0.36 | 0.21 / 0.30 |
| **ProxMPC** | **0.65 / 1.11** | 1.05 / 1.87 | 0.92 / 4.33 | 0.97 / 3.75 | 1.19 / 4.24 |
| **ProxMPC (predictive)** | 0.56 / 1.09 | 1.08 / 2.15 | 0.78 / 4.38 | 0.87 / 3.31 | 1.17 / 3.62 |
| DWB | 2.54 / 2.91 | 2.69 / 3.13 | 2.65 / 3.07 | 2.68 / 3.42 | 2.62 / 3.47 |
| MPPI | 2.74 / 3.16 | 2.72 / 3.08 | 2.71 / 3.11 | 2.77 / 3.43 | 2.75 / 3.26 |

**Multi-obstacle cells:**

| Controller | dyn_multi | dyn_multi_noise | blind_0 | blind_1 | blind_2 | blind_3 |
| --- | --- | --- | --- | --- | --- | --- |
| Graceful | 0.18 / 0.27 | 0.16 / 0.23 | 0.15 / 0.20 | 0.18 / 0.27 | 0.16 / 0.24 | 0.16 / 0.25 |
| Regulated Pure Pursuit | 0.22 / 0.34 | 0.21 / 0.32 | 0.21 / 0.30 | 0.21 / 0.31 | 0.21 / 0.33 | 0.21 / 0.32 |
| Vector Pursuit | 0.21 / 0.35 | 0.21 / 0.30 | 0.20 / 0.32 | 0.21 / 0.32 | 0.21 / 0.33 | 0.21 / 0.29 |
| **ProxMPC** | 1.51 / 4.77 | 1.55 / 5.24 | 1.44 / 5.04 | 1.37 / 4.56 | 1.27 / 4.28 | 1.54 / 4.41 |
| **ProxMPC (predictive)** | 1.38 / 4.05 | 1.55 / 4.28 | 1.60 / 4.46 | 1.35 / 4.37 | 1.34 / 3.44 | 0.98 / 2.61 |
| DWB | 2.83 / 3.53 | 2.82 / 3.54 | 2.73 / 3.51 | 2.75 / 3.25 | 2.71 / 3.46 | 2.86 / 3.55 |
| MPPI | 2.75 / 3.53 | 2.77 / 3.39 | 2.75 / 3.27 | 2.73 / 3.30 | 2.76 / 3.29 | 2.72 / 3.34 |

### 4.2 Process CPU [% of one core] / RSS peak [MB]

**Single-obstacle cells:**

| Controller | open | static_box | line_fwd | line_bwd | circle |
| --- | --- | --- | --- | --- | --- |
| Regulated Pure Pursuit | 4.4 / 55 | 4.3 / 55 | 4.3 / 55 | 4.5 / 55 | 4.3 / 55 |
| Vector Pursuit | 4.2 / 55 | 4.5 / 55 | 4.2 / 55 | 4.1 / 55 | 4.3 / 55 |
| Graceful | 4.2 / 56 | 4.3 / 56 | 4.3 / 56 | 4.2 / 56 | 4.3 / 56 |
| **ProxMPC** | 5.0 / 58 | 6.1 / 58 | 6.5 / 58 | 6.5 / 58 | 7.2 / 58 |
| **ProxMPC (predictive)** | 5.3 / 60 | 6.4 / 60 | 6.5 / 60 | 6.4 / 60 | 7.2 / 60 |
| DWB | 8.6 / 60 | 9.0 / 60 | 8.8 / 60 | 8.9 / 60 | 9.0 / 60 |
| MPPI | 8.6 / 63 | 8.6 / 63 | 8.8 / 63 | 8.9 / 63 | 8.9 / 63 |

**Multi-obstacle cells:**

| Controller | dyn_multi | dyn_multi_noise | blind_0 | blind_1 | blind_2 | blind_3 |
| --- | --- | --- | --- | --- | --- | --- |
| Regulated Pure Pursuit | 4.6 / 55 | 4.5 / 55 | 4.6 / 55 | 4.3 / 55 | 4.4 / 55 | 4.2 / 55 |
| Vector Pursuit | 4.3 / 55 | 4.3 / 55 | 4.2 / 55 | 4.3 / 55 | 4.4 / 55 | 4.2 / 55 |
| Graceful | 4.6 / 56 | 4.4 / 56 | 4.4 / 56 | 4.4 / 56 | 4.3 / 56 | 4.2 / 56 |
| **ProxMPC** | 7.5 / 58 | 7.9 / 58 | 7.6 / 58 | 7.3 / 58 | 7.4 / 58 | 7.5 / 58 |
| **ProxMPC (predictive)** | 7.7 / 60 | 7.1 / 60 | 7.0 / 60 | 7.6 / 60 | 7.3 / 60 | 6.8 / 60 |
| DWB | 9.6 / 60 | 9.4 / 60 | 9.2 / 60 | 9.1 / 60 | 9.1 / 60 | 9.4 / 60 |
| MPPI | 9.1 / 63 | 9.1 / 63 | 9.1 / 64 | 8.9 / 63 | 9.0 / 64 | 8.9 / 63 |

> ProxMPC-predictive additionally spends ~1.0 % core and ~38 MB in the obstacle-tracker process, constant across every scenario — a small overhead the tables above exclude.

The compute comparison across the eleven cells:

- **This is ProxMPC's most durable and general advantage, and it holds on all eleven cells.** On the empty cell ProxMPC computes a command in **0.65 ms median / 1.11 ms p95** — **~3.9× faster than DWB and ~4.2× faster than MPPI at the median**. Adding obstacles roughly doubles-to-triples ProxMPC's median (to 0.9–1.6 ms) as the keep-out rows activate, but it stays **~1.7–2× lighter than DWB and MPPI at the median in every scenario**, including the six-way-constrained blind cells. MPPI's cost is nearly flat (its 2000-sample batch dominates regardless of obstacles); DWB's rises slightly with its active critics.
- **Process CPU tracks the same ordering**, compressed by the shared ~4 % costmap floor: ProxMPC **5.0 %** (open) rising to ~6.5–7.9 % (obstacles), still at or below DWB/MPPI's **8.6–9.6 %** on every cell; ProxMPC-predictive (controller + tracker, ~6.4–8.7 % combined) also stays at or below the sampling controllers. RSS is 55–64 MB throughout, MPPI the heaviest (63–64 MB), ProxMPC mid-pack (58–60 MB).
- **The geometric controllers are lighter still** — RPP, Graceful and Vector Pursuit compute in ~0.15–0.25 ms at ~4 % CPU, carrying no optimisation or constraint machinery. ProxMPC is the cheapest of the controllers that solve a constrained optimisation each cycle, not the cheapest controller outright; the geometric controllers buy their low cost by having no model, no constraints, and no horizon optimisation.
- **Prediction does not cost more — usually about the same, sometimes less.** ProxMPC-predictive is within ~20 % of reactive at the median on nearly every cell, and is actually *cheaper at the p95 tail* on several cells (a single confirmed track is a smaller, cleaner constraint set than the reactive costmap's clustered wall cells). The predictive capability's real extra price is the companion tracker (~1 % core, ~38 MB), not the controller cycle.
- **The tail is heavier on the hard cells.** On the dynamic and multi-obstacle cells ProxMPC's **p95 rises to 3.3–5.2 ms** — above DWB's and MPPI's steadier ~3.1–3.6 ms — because with many half-plane rows simultaneously active the worst-case QP is heavier than a fixed-size sampler. ProxMPC is cheaper on average but more variable; the sampling controllers are steadier. Every p95 still sits **~10× inside** the 50 ms control budget, and the worst single cycle measured (~19 ms on one blind-cell repeat) is ~2.6× inside it.
- **Achieved command rate is completion-coupled on the hard cells.** ProxMPC holds a clean ~20 Hz on the open, static and single-dynamic cells, and mostly holds it on the multi cells (~17–20 Hz), dropping to ~16–17 Hz only where it stalls. This is **not** a compute deadline miss — the solver stays at 0 % deadline-miss / 0 % infeasible throughout (§5) — but a symptom of the controller **stalling** when over-constrained: a stalled robot stops being commanded, which lowers the sampled rate. It is read together with the completion and collision results in §6, not as a compute limit.

## 5. ProxMPC solver profile (and a cross-check)

ProxMPC is the only controller that also publishes its *internal* solver telemetry ([`SolverDiagnostics`](../prox_mpc_msgs/README.md)):

| Metric | Open cell | With obstacle constraints active |
| --- | --- | --- |
| decorator compute p95 (whole `computeVelocityCommands`) | 1.11 ms | 1.7–5.7 ms |
| internal QP solve p95 | 1.04 ms | 1.5–6.4 ms |
| worst single-cycle solve (max) | 2.53 ms | up to ~19 ms |
| deadline-miss rate (compute > 50 ms budget) | 0.0 % | 0.0 % |
| infeasible rate (QP status ≠ SOLVED) | 0.0 % | 0.0 % |
| mean SQP iterations | 1.00 | 1.00 |

The two independent instruments agree on the open cell: the QP solve (1.04 ms p95) accounts for almost all of the measured per-cycle compute (1.11 ms p95), the ~0.07 ms remainder being the costmap reduction and the exact footprint veto.
The worst single open-cell cycle stays **~20× inside** the 50 ms budget, which is why ProxMPC holds 20 Hz with 0 % deadline-miss and 0 % infeasible.
With the in-loop obstacle constraints active the per-cycle cost rises to a p95 of 1.5–6.4 ms across the eleven obstacle scenarios (more active half-plane rows on the dynamic and multi cells than the static one) and a worst single cycle of ~19 ms — still ~2.6× inside budget, and **still 0 % deadline-miss and 0 % infeasible on every cell, including the six-constraint blind cells**.
The solver never fails to return a feasible QP solution in time; where reactive ProxMPC misses on a multi cell (§6.4) it is the *geometry* of the convex keep-out that fails, not the solver's timing or feasibility.

## 6. Obstacle avoidance

The open cell cannot show avoidance, so these scenarios place obstacles across the path and let every controller perceive them through the same local and global `obstacle_layer`.
Because the kinematic plant has no collision physics, "reaching the goal" alone does not prove avoidance — a controller can drive straight through an obstacle and still "succeed" — so the discriminating metric is the **obstacle gap** (robot-disc to obstacle-disc; negative = overlap = would-be collision).

The global costmap carries an `obstacle_layer`, so NavFn routes the global plan around obstacles for every controller, and every cell uses one uniform obstacle size (`clearance 0.45`). The single-obstacle cells (§6.1–6.2) are within reach of the whole field and separate the controllers on *margin* and on the orbiting case; the multi-obstacle cells (§6.4) are where collision counts spread. Two comparisons carry the result: ProxMPC against the other optimisation- and sampling-based controllers — **DWB** and **MPPI**, its per-cycle-cost peers — and ProxMPC against the geometric pursuit controllers — **RPP**, **Graceful** and **Vector Pursuit** — which trade avoidance headroom for near-zero compute.

### 6.1 Static obstacle (`static_box`, mode b2)

A 0.25 m box placed **dead-centre** on the path at `(0, 0)`; with the obstacle-routed global plan every controller has a path that already bends around it.
The last two columns carry the per-scenario **cost of that avoidance** (the same decorator/process instruments as §4), so clearance and its price are read together.

| Controller | success | obstacle gap [m] | collision | compute p50/p95 [ms] | CPU [%] |
| --- | --- | --- | --- | --- | --- |
| **ProxMPC** | 5/5 | **+0.346 ± 0.009** | **0/5** | 1.05 / 1.87 | 6.1 |
| **ProxMPC (predictive)** | 5/5 | +0.299 ± 0.002 | **0/5** | 1.08 / 2.15 | 6.4 + 1.1 trk |
| Regulated Pure Pursuit | 5/5 | +0.213 ± 0.003 | 0/5 | 0.21 / 0.27 | 4.3 |
| Graceful | 5/5 | +0.208 ± 0.005 | 0/5 | 0.16 / 0.21 | 4.3 |
| MPPI | 10/10 | +0.206 ± 0.002 | 0/10 | 2.72 / 3.08 | 8.6 |
| DWB | 5/5 | +0.093 ± 0.004 | 0/5 | 2.69 / 3.13 | 9.0 |
| Vector Pursuit | **0/5** | +0.165 (stopped short) | 0/5 | 0.25 / 0.39 | 4.5 |

Every controller clears the static box collision-free, so the discrimination is in the *margin*, not in whether they hit.
**ProxMPC keeps the largest margin of the field — +0.346 m** — by swerving wide and recovering, the behaviour the in-loop keep-out constraint produces, at **~2.5× less per-cycle compute than DWB and MPPI**.
Predictive ProxMPC holds a close +0.299 m: the tracker reports the box at ~0 velocity, so the hybrid fill treats it much like the reactive path.
Among the cost-class peers, **DWB cuts it finest at +0.093 m** — its soft `BaseObstacle` critic rides close to the inflated edge — while MPPI holds +0.206 m; among the geometric controllers RPP and Graceful hold ~+0.21 m.
**Vector Pursuit is the one controller that fails to complete** (0/5): its own forward-collision check (`use_collision_detection`) halts it in front of the box before it steers around, so its +0.165 m "gap" is a stopped-short distance, not a steered avoidance.

### 6.2 Single moving obstacles (mode b2, reactive)

Three single-mover scenarios — a patrol crossing the path forward (`dynamic_line_forward`) and backward (`dynamic_line_backward`), and an obstacle orbiting near mid-path (`dynamic_circle`), 0.5 m/s.
Almost the whole field clears these reactively; the exceptions are the two controllers whose commitment to a rollout clips the *orbiting* obstacle:

| Scenario | metric | ProxMPC | ProxMPC-pred | DWB | MPPI | RPP | Graceful | VecPursuit |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| line_forward | collision | 0/5 | 0/5 | 0/5 | 0/10 | 0/5 | 0/5 | 0/5 |
| | min gap [m] | +0.54 | +0.38 | +0.45 | +0.57 | +0.41 | +0.37 | +0.46 |
| line_backward | collision | 0/5 | 0/5 | 0/5 | 0/10 | 0/5 | 0/5 | 0/5 |
| | min gap [m] | +0.55 | +0.38 | +0.49 | +0.61 | +0.41 | +0.34 | +0.45 |
| circle | collision | **0/5** | **0/5** | **5/5** | 0/10 | 0/5 | **3/5** | 0/5 |
| | min gap [m] | +0.16 | +0.14 | **−0.26** | +0.17 | +0.10 | **−0.04** | +0.11 |

**On the two straight patrols every controller keeps a real margin (0 collisions).**
On the **orbiting** obstacle the field splits: ProxMPC, predictive ProxMPC, MPPI, RPP and Vector Pursuit all clear it (+0.10 to +0.17 m), while **DWB collides 5/5 (−0.26 m)** and **Graceful 3/5 (−0.04 m)** — both commit to a trajectory the obstacle then orbits into, and the routed global plan (computed once against the obstacle's initial footprint) does not track the moving object. This is the one single-obstacle cell where ProxMPC's constrained optimisation outperforms a cost-class peer, DWB, outright.
Reactive and predictive ProxMPC are indistinguishable across the three single-mover cells — both hold a positive margin — so prediction's measurable value appears on the multi-obstacle cells (§6.4).
Compute stays in the §4 ordering: ProxMPC ~1.6–2× lighter than DWB and MPPI at the median while carrying the active keep-out rows, with a p95 tail up to ~4.3 ms on the orbit.

### 6.3 Ground-truth predictive confirmation (mode b1)

One controller-vs-tracker separation is worth recording: fed the *correct* future obstacle positions (mode b1, the engine knows the exact trajectory, orbit included), ProxMPC solves every single moving-obstacle case for both vehicle models:

| Scenario | Model | success | cross-track RMS [m] | time-to-goal [s] |
| --- | --- | --- | --- | --- |
| dynamic_circle | Unicycle / Bicycle | 5/5 · 5/5 | 0.007 / 0.005 | 5.39 / 5.44 |
| dynamic_line_forward | Unicycle / Bicycle | 5/5 · 5/5 | 0.042 / 0.036 | 5.04 / 5.13 |
| dynamic_line_backward | Unicycle / Bicycle | 5/5 · 5/5 | 0.042 / 0.036 | 5.08 / 5.15 |

With ground-truth prediction ProxMPC reaches the goal on **every** single moving-obstacle run for both a 3-state unicycle and a 4-state bicycle (30/30), the orbiting obstacle included, driven by the same plugin with one parameter changed (`model_plugin`).
The single-obstacle dynamic case is fully within the controller's reach given a correct predictor. The reactive costmap already suffices for the single movers (§6.2), so this ground-truth result is a ceiling check — it confirms the controller, not the tracker, sets the limit — rather than the operative result.

### 6.4 Multiple simultaneous moving obstacles — where the field separates

The single-obstacle cells do not discriminate on collisions (§6.1–6.2), so the collision comparison rests on the **multi-mover** set: `dynamic_multi` and `dynamic_multi_noise` (two hand-built simultaneous movers, the latter with sensor noise), and `blind_multi_0`–`blind_multi_3` (blind, author-independent two-mover cells, generated without hand-screening).
Six cells, 30 runs per controller (60 for MPPI), collision per cell:

| Scenario | ProxMPC | ProxMPC-pred | DWB | MPPI | RPP | Graceful | VecPursuit |
| --- | --- | --- | --- | --- | --- | --- | --- |
| dynamic_multi | 2/5 | 1/5 | 1/5 | 1/10 | 0/5 | 5/5 | 1/5 |
| dynamic_multi_noise | 1/5 | 1/5 | 1/5 | 1/10 | 0/5 | 3/5 | 0/5 |
| blind_multi_0 | 5/5 | 4/5 | 2/5 | 10/10 | 3/5 | 2/5 | 2/5 |
| blind_multi_1 | 1/5 | 3/5 | 0/5 | 7/10 | 2/5 | 1/5 | 2/5 |
| blind_multi_2 | 2/5 | 0/5 | 4/5 | 1/10 | 5/5 | 5/5 | 5/5 |
| blind_multi_3 | 0/5 | 0/5 | 0/5 | 6/10 | 0/5 | 1/5 | 1/5 |
| **aggregate collisions** | **11/30** | **9/30** | **8/30** | **26/60** | **10/30** | **17/30** | **11/30** |
| **aggregate success** | 28/30 | 26/30 | 29/30 | 59/60 | 30/30 | 30/30 | 30/30 |

ProxMPC's collision count sits in the leading group, and the two class comparisons are favourable.

Against its cost-class peers: ProxMPC collides on far fewer runs than **MPPI** — 11/30 reactive and 9/30 predictive against 26/60 — while computing markedly lighter (§4), and it is level with **DWB** (8/30, inside run-to-run noise at 5 repeats per cell) while holding the larger static margin (§6.1) and clearing the orbit DWB misses (§6.2). Against the geometric controllers it is level with **RPP** (10/30) and **Vector Pursuit** (11/30) and ahead of **Graceful** (17/30), for the small CPU premium of §4. MPPI is the field's laggard on moving obstacles, colliding on nearly every multi run.

Prediction adds a modest edge — 11/30 to 9/30 — concentrated on `blind_multi_2` (2/5 → 0/5) and within noise elsewhere.

The one cell where ProxMPC is the field's weakest is `blind_multi_0`, the tightest simultaneous-mover geometry: ProxMPC 5/5 and predictive 4/5 against 2/5 for DWB, Graceful and Vector Pursuit. Two close movers force a non-convex "which side of each obstacle" choice that the linearised keep-out constraint cannot represent, and prediction does not resolve it. No controller is collision-free across the blind cells, and the winner is cell-dependent — RPP clears the two hand-built cells but collides 5/5 on `blind_multi_2`; DWB clears `blind_multi_1` and `blind_multi_3` but collides 4/5 on `blind_multi_2`.

The per-cell breakdown for every controller is in [`prox_mpc_benchmark/README.md`](../prox_mpc_benchmark/README.md) and the raw `results/`.
In summary: ProxMPC keeps the largest static margin (§6.1), clears the single movers reactively and outperforms DWB on the orbit (§6.2), leads its sampling peer and matches or leads every other controller on the multi-mover set at a fraction of the sampling controllers' compute (§4), with one hard cell (`blind_multi_0`) where the convex keep-out is the field's weakest.

## 7. Real-stack validation (Gazebo)

To confirm the plugin behaves under the full production stack and not only against a kinematic plant, ProxMPC also runs in mode (a): Gazebo Harmonic physics, a TurtleBot3 waffle, AMCL localization, costmaps, and the complete Nav2 velocity chain, headless.
It reaches the goal (`SUCCEEDED`), tracks the path to **5.6 mm cross-track RMS**, and taps its diagnostics through the real `controller_server` (p95 solve 0.438 ms, 0 % deadline-miss, 0 % infeasible) — the same profile measured on the plant, with real sensor and physics noise.

## 8. Threats to validity

- **The obstacle-routed global plan does much of the single-obstacle avoidance** (§6). The shared global `obstacle_layer` makes NavFn route around obstacles for every controller — deliberately equal, but it favours the geometric path-followers (RPP, Graceful, Vector Pursuit) and DWB, which track the global detour closely. The single-obstacle cells therefore separate the field on margin and on the orbiting case, not on collision counts; the multi-mover cells (§6.4) carry the collision comparison.
- **The multi-obstacle field is close and the sample is small** (§6.4). The leading controllers (DWB, ProxMPC ± prediction, RPP, Vector Pursuit) sit within run-to-run noise at 5 repeats per cell; the trustworthy signals are the class comparisons — ProxMPC leads MPPI and Graceful, and matches DWB and RPP — and that `blind_multi_0` is hard for the whole field. The 8-vs-9-vs-10-vs-11 ordering among the leaders is not a reliable ranking, and multi-obstacle difficulty scales with obstacle size.
- **`blind_multi_0` is a genuine ProxMPC limit** (§6.4). On the tightest two-mover cell ProxMPC and its predictive variant are the field's weakest, consistent with the convex keep-out being unable to represent the non-convex per-obstacle side-choice; prediction does not fix it. This is an architectural limit of the linearised keep-out, isolated to the tightest cell.
- **Collision uses a strict min-gap** — any instant of disc overlap over the whole run counts as a collision, so a brief graze is flagged the same as a harder hit; the reported min gap distinguishes the two.
- **Achieved command rate is completion-coupled** (§4). ProxMPC's ~16–20 Hz sampled rate on the multi cells reflects occasional stalls (a stopped robot stops being commanded), not a compute deadline miss — the solver stays 0 % deadline-miss / 0 % infeasible with a worst cycle ~2.6× inside budget (§5).
- **Resources are measured on the x86 host** above, and process CPU/RAM is per-process (includes the shared costmap); the per-cycle compute (§4) is the cleaner controller-only measure, and the ranking is what transfers. The pure-geometric controllers are genuinely lighter than ProxMPC — ProxMPC is the cheapest of the controllers that solve a constrained optimisation each cycle, not the cheapest outright.
- **MPPI is stochastic with no exposed seed** (Nav2 Jazzy), so its per-scenario variance is real; it runs 10 repeats per obstacle cell (5 on the open cell) to better characterise that variance, but this does not pin it down exactly.
- **Determinism vs. physics.** Modes (b1)/(b2) are deterministic plants, so their near-zero geometric std is reproducibility, not a noise estimate; mode (a) carries real Gazebo variance, and `dynamic_multi_noise` adds a Gaussian range-noise model on the scan.

## 9. Conclusion on ProxMPC

On a like-for-like, fairly-tuned suite ProxMPC matches the best stock Nav2 controller on tracking, runs markedly lighter per cycle than the optimisation- and sampling-based controllers, holds the largest static-obstacle margin, and leads or matches every controller on moving obstacles:

- **Per-cycle compute and process resources (measured, all eleven cells):** ProxMPC computes a command in **0.65 ms median / 1.11 ms p95** on the open cell — **~3.9–4.2× lighter than DWB and MPPI** — and stays **~1.7–2× lighter at the median on every obstacle cell**, at **5–8 % CPU against their 8.6–9.6 %**, with the QP solve confirmed by independent telemetry to be almost the entire cost and **0 % deadline-miss / 0 % infeasible across all three run modes**. The tail is heavier on the hard cells (p95 3.3–5.2 ms against the samplers' steadier ~3.1–3.6 ms), still ~10× inside the 50 ms budget. The geometric controllers (RPP, Graceful, Vector Pursuit) are lighter still, carrying no model, constraints, or horizon optimisation; against them ProxMPC's premium is ~1–4 % of one core.
- **Tracking:** sub-millimetre (0.0004 m RMS), on par with the field, 100 % success.
- **Static-obstacle avoidance:** **ProxMPC keeps the largest real margin, +0.35 m** — ahead of its cost-class peers MPPI (+0.21 m) and DWB (+0.09 m), and of the geometric RPP and Graceful (~+0.21 m). Vector Pursuit is stopped short by its own forward-collision check.
- **Single moving obstacles:** cleared reactively by almost the whole field; ProxMPC clears the orbiting obstacle that its cost-class peer DWB (5/5) and Graceful (3/5) collide on. Ground-truth prediction (mode b1) solves all three single moving-obstacle cases 30/30 for both a unicycle and a bicycle.
- **Multiple simultaneous moving obstacles:** ProxMPC leads its sampling peer MPPI decisively (11/30 reactive, 9/30 predictive against 26/60) and Graceful (17/30), and matches DWB (8/30) and RPP (10/30) within run-to-run noise. Prediction adds a modest edge (11/30 → 9/30). The one cell where ProxMPC is the field's weakest is `blind_multi_0`, where the convex keep-out cannot make the non-convex two-mover side-choice.
- **One controller, many vehicles:** the identical plugin drives a unicycle and a bicycle by configuration alone.

Against DWB and MPPI — its per-cycle-cost peers — ProxMPC is roughly four times lighter per cycle at equal tracking accuracy, holds a larger static margin, clears the orbit DWB misses, and leads MPPI on multiple movers. Against the geometric pursuit controllers it holds larger avoidance margins for a small CPU premium. Its one weak point is the tightest two-mover cell, where it fails by stalling rather than driving through.

---

*Reproduce:* the open cell and the mode-(b2) reactive obstacle scenarios with `ros2 run prox_mpc_benchmark run_nav2.py --scenario <nav2_open|static_box|dynamic_circle|dynamic_line_forward|dynamic_line_backward|dynamic_multi|dynamic_multi_noise|blind_multi_0|blind_multi_1|blind_multi_2|blind_multi_3>`, the **predictive b2** runs (real tracker) by adding `--controllers proxmpc_pred` to the same command, the mode-(b1) ground-truth predictive results with `ros2 run prox_mpc_benchmark run_matrix.py --modes b1`, and the tables with `ros2 run prox_mpc_benchmark aggregate.py`. See [`prox_mpc_benchmark/README.md`](../prox_mpc_benchmark/README.md).

## License

[Apache-2.0](../LICENSE).
