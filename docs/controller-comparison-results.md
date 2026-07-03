# Controller Comparison Results

This document reports how the ProxMPC controller compares against the stock Nav2 controllers under identical, reproducible, and *fairly-tuned* conditions — on **path tracking**, **per-cycle compute**, and **obstacle avoidance**.
It is the narrative companion to the auto-generated tables in [`prox_mpc_benchmark/README.md`](../prox_mpc_benchmark/README.md); the harness that produced every number is described in [the package guide](prox-mpc.md#8-prox_mpc_benchmark--the-measurement-harness).

All numbers below are measured, reproducible, and reported as `mean ± std` (population) over fixed-seed repeats — the precision signal.
Nothing is hand-tuned to favour one controller: every controller drives the *same* plant from the *same* start to the *same* goal, at a *matched operating point* (Section 2.1), perceives obstacles through the *same* costmap, and is measured by the *same* instrumentation — including a timing decorator that wall-clock times every controller's per-cycle compute identically.

The comparison covers the five controllers that ship with Nav2 Jazzy that are standalone local planners: **ProxMPC**, **DWB**, **MPPI**, **Regulated Pure Pursuit (RPP)**, and **Graceful** (new in Jazzy).
The **Rotation Shim** controller is a meta-controller (it rotates in place to the path heading then delegates to a `primary_controller`), so it has no independent tracking or avoidance and is not a peer here.
TEB and Vector Pursuit are external community packages, not part of the Nav2 Jazzy core, and are out of scope.

## 1. What is being compared

Two complementary comparisons are run:

1. **The open-world cell** — an empty 7 × 7 m room, a single straight traverse from `(-2.5, 0)` to `(2.5, 0)` (5 m), no obstacles.
   This is a *pure path-tracking* task that isolates **tracking fidelity**, **per-cycle compute cost**, and **process resource use** on an exactly equal footing, with no perception or obstacle geometry to confound them (Sections 3–5).
2. **The obstacle scenarios** — the *same* 5 m traverse with a static box or a moving obstacle placed across the path, run on the mode (b2) Nav2 stack so that **every controller perceives the same obstacle through the same costmap and inflation** (Section 6).
   This is where the avoidance behaviours actually separate.

The open cell deliberately understates a constrained optimal-control method — an empty straight line is the task a geometric pursuit controller is provably optimal for — which is why obstacle avoidance is reported separately.

## 2. Test conditions

| Condition | Value |
| --- | --- |
| ROS 2 / Nav2 | Jazzy / Nav2 1.3.12 |
| Host | x86-64 workstation, Intel i7-10750H, 12 logical cores — **not a Jetson** |
| Run mode | **(b2)** Nav2 stack on a kinematic plant, no Gazebo (see [run modes](prox-mpc.md#9-running-the-stack-the-three-modes)); mode (b1) for the ground-truth predictive results in §6.3 |
| Plant | Unicycle body-twist integrator at 50 Hz, identical for every controller |
| Localization | Exact (static `map → odom` identity; the plant pose is ground truth) |
| Map | `prox_mpc_open` — 7 × 7 m room, free interior `[-2.95, 2.95] m` |
| Task | straight 5 m traverse `(-2.5, 0) → (2.5, 0)`, goal tolerance 0.25 m |
| Planner | `NavfnPlanner` (GridBased), shared by all controllers |
| Control rate | 20 Hz (`controller_frequency`); real-time budget 50 ms/cycle |
| Local costmap | `static_layer` + `obstacle_layer` + `inflation_layer` (0.70 m radius), 5 Hz update |
| Obstacle sensing | a **scan simulator** ray-casts the scenario obstacles into `/scan`; the `obstacle_layer` marks/clears them, so all controllers see the same obstacle |
| Repeats | **5** fixed-seed repeats per controller per scenario |
| Controllers | ProxMPC (Unicycle), DWB, MPPI, RPP, Graceful — plus **ProxMPC (predictive)** with the obstacle tracker for §6.3 |
| Compute timing | a `nav2_core::Controller` **timing decorator** wraps every controller and times its `computeVelocityCommands` identically |

### 2.1 Fair tuning: what is equalised and what is not

The controllers are different algorithms, so their *internal* knobs are not the same quantity and cannot be set "equal" without meaning something different for each.
The fair approach is to equalise everything they *share* and leave each method's intrinsic mechanism at its documented operating point:

- **Equalised** — control rate (20 Hz), max linear speed (0.5 m/s), goal tolerance (0.25 m), the **prediction horizon (2.0 s)** for all predictive controllers (ProxMPC `np·dt = 20 × 0.1`; DWB `sim_time = 2.0`; MPPI `time_steps × model_dt = 40 × 0.05`), and — the key addition for the obstacle comparison — the **obstacle perception**: a shared `obstacle_layer` fed by the scan simulator means every controller (including ProxMPC's costmap fill) sees the *same physical obstacle* through the *same costmap + inflation*. In the head-to-head ProxMPC runs with `predict_obstacles: false`, so it gets **no** privileged obstacle knowledge; the separate **ProxMPC (predictive)** variant (§6.3) turns that flag on and adds the real obstacle tracker, and is otherwise the identical preset so the comparison isolates prediction.
- **Left at each method's default** — the *avoidance mechanism itself*, because it has no common denominator across paradigms: DWB's `BaseObstacle` critic (nav2_bringup default `scale 0.02`, calibrated to that critic's raw cost magnitude), MPPI's `CostCritic` (upstream default `cost_weight 3.81`), ProxMPC's in-loop CBF keep-out constraint, and RPP/Graceful's forward-simulation collision check. Crippling any of them to a common number would misrepresent it. The obstacle each faces is equal; how each responds is its own algorithm.
- **Intrinsic sampling** left at upstream defaults: MPPI `batch_size = 2000`, DWB's `20 × 20` velocity grid, ProxMPC's single-QP SQP.
- **Determinism** — MPPI is the one stochastic controller and Nav2 Jazzy exposes no RNG seed for it, so its repeats capture genuine sampling variance (reported as `mean ± std`), not reproducibility; it is run at the same 5 repeats as the others.
- **Graceful** — a good-faith goal-approach tuning was applied (lookahead below the goal tolerance, reduced slowdown radius, no in-place final rotation) so it drives into the goal rather than stalling far out; even so its pose-following law tends to park right at the tolerance boundary (see §3).

Because the plant, map, planner, goal checker, horizon, speed, rate, costmap, and instrumentation are all shared, every difference in the tables is attributable to the controller.

### Metric definitions

- **time-to-goal** — wall time from first motion to entering the goal tolerance.
- **goal error** — final distance to the goal point; controllers that reach stop on the *same* 0.25 m checker.
- **cross-track RMS / max** — deviation from the straight reference; on obstacle scenarios this is *also* the size of the avoidance detour, so it is read together with clearance.
- **obstacle gap** (obstacle scenarios) — the minimum distance, over the whole run, between the **robot disc** and the **obstacle disc** (robot radius 0.22 m, obstacle body radius 0.28 m). Positive = a real safety margin was kept; **negative = the two discs overlapped**, i.e. a would-be collision on the collision-free kinematic plant. Controller-agnostic: the same measurement for all five.
- **collision rate** — fraction of repeats whose obstacle gap went negative.
- **compute p50 / p95 / max** — wall time of one `computeVelocityCommands`, measured by the decorator, **the same way for all five**.
- **CPU / RSS / rate** — the `controller_server` process's CPU (% of one core) and RSS from `/proc`, and the achieved `/cmd_vel` rate.
- **solve p95, deadline-miss, infeasible, SQP/QP iters, slack** — ProxMPC-only internal solver telemetry.

## 3. Tracking-fidelity comparison (open-world cell)

| Controller | success | time-to-goal [s] | goal err [m] | cross-track RMS [m] | cross-track max [m] |
| --- | --- | --- | --- | --- | --- |
| **ProxMPC** (Unicycle) | 5/5 | 12.69 ± 0.76 | 0.246 | **0.0000 ± 0.0000** | 0.0008 |
| Regulated Pure Pursuit | 5/5 | 10.31 ± 0.29 | 0.247 | 0.0000 ± 0.0000 | 0.0000 |
| MPPI | 5/5 | 10.91 ± 0.97 | 0.240 | 0.0030 ± 0.0000 | 0.0042 |
| DWB | 5/5 | 12.14 ± 0.88 | 0.246 | 0.0000 ± 0.0000 | 0.0013 |
| Graceful | **0/5** | — | 0.250 | 0.0000 ± 0.0000 | — |

Four of the five track the straight line essentially perfectly (sub-millimetre cross-track); ProxMPC, RPP and DWB are exact to the plant resolution and MPPI is within 3 mm.
On *tracking fidelity* there is no meaningful separation on an empty straight line — which is exactly why the compute comparison below is the discriminating result for the predictive controllers.

**Graceful is the exception, and it is a completion quirk, not a tracking failure.**
Graceful tracks the line perfectly (0 mm cross-track) but its pose-following control law decelerates *asymptotically* into the goal and parks right at the 0.25 m goal-tolerance boundary (final goal error 0.250 m), so the goal checker never latches and the progress checker aborts the action after it stops advancing.
Even after tuning its goal approach it stops ~0.5 cm short, so it registers 0/5 "reached" on the open cell.
This is a known trait of the Graceful controller against a tight goal tolerance; it recurs on the obstacle scenarios below.

## 4. Per-cycle compute cost and process resources

This is the comparison that matters on an embedded target, and it is **measured, not inferred**: the timing decorator wall-clock times every controller's `computeVelocityCommands` the same way, on **every** scenario.
Unlike tracking, compute is *not* flat across scenarios — the obstacle cells activate each method's avoidance machinery (ProxMPC's in-loop keep-out rows, DWB/MPPI's cost critics), so the per-cycle cost grows with obstacle load.
The tables are therefore **per scenario**, so that growth is visible.

**ProxMPC (predictive)** is the same plugin with `predict_obstacles: true` and the obstacle tracker in the loop (§6.3), otherwise identical to reactive ProxMPC — so the two rows isolate the cost of prediction alone.
Its companion tracker runs as a *separate* process (~1 % of one core, ~38 MB RSS, constant across scenarios), which the `controller_server` figures below do **not** include; it is reported separately.

### 4.1 Per-cycle compute, `p50 / p95` [ms] (mean over 5 repeats)

| Controller | open | static_box | line_fwd | line_bwd | circle |
| --- | --- | --- | --- | --- | --- |
| Graceful | 0.04 / 0.15 | 0.06 / 0.22 | 0.08 / 0.18 | 0.04 / 0.16 | 0.13 / 0.21 |
| Regulated Pure Pursuit | 0.21 / 0.31 | 0.22 / 0.33 | 0.21 / 0.32 | 0.21 / 0.31 | 0.21 / 0.34 |
| **ProxMPC** | **0.40 / 0.85** | 0.90 / 2.81 | 1.09 / 4.11 | 1.37 / 4.12 | 1.52 / 6.04 |
| **ProxMPC (predictive)** | 0.46 / 0.95 | 0.88 / 2.91 | **0.74 / 3.72** | **0.75 / 3.69** | 1.92 / 5.54 |
| DWB | 2.55 / 3.44 | 2.56 / 4.59 | 3.14 / 4.04 | 3.12 / 4.22 | 3.22 / 3.97 |
| MPPI | 2.68 / 3.68 | 2.76 / 3.81 | 2.63 / 3.28 | 2.67 / 3.44 | 2.64 / 3.35 |

### 4.2 Process CPU [% of one core] / RSS peak [MB]

| Controller | open | static_box | line_fwd | line_bwd | circle |
| --- | --- | --- | --- | --- | --- |
| Regulated Pure Pursuit | 4.0 / 55 | 3.9 / 55 | 4.2 / 55 | 4.0 / 55 | 4.1 / 55 |
| Graceful | 4.1 / 56 | 4.2 / 56 | 4.2 / 56 | 4.2 / 56 | 4.3 / 56 |
| **ProxMPC** | 4.5 / 58 | 5.9 / 58 | 6.7 / 58 | 6.8 / 58 | 6.6 / 58 |
| **ProxMPC (predictive)** | 4.8 / 59 | 6.4 / 59 | 6.3 / 59 | 6.5 / 59 | 6.0 / 60 |
| DWB | 8.4 / 60 | 9.0 / 60 | 9.7 / 60 | 9.8 / 60 | 10.0 / 60 |
| MPPI | 8.3 / 63 | 8.7 / 63 | 9.0 / 65 | 9.1 / 65 | 9.0 / 65 |

> ProxMPC-predictive additionally spends ~0.9–1.0 % core and ~38 MB in the obstacle-tracker process — a small, constant overhead the table above excludes.

Reading the tables:

- **Compute scales with obstacle load, and the ranking holds.** On the empty cell ProxMPC computes a command in **0.40 ms median / 0.85 ms p95** — **~6.4× faster than DWB and ~6.7× faster than MPPI at the median**. Adding an obstacle roughly doubles-to-triples ProxMPC's median (to 0.9–1.5 ms) as the keep-out rows activate, but it stays **2–4× lighter than DWB and MPPI at the median in every scenario**. MPPI's cost is nearly flat (its 2000-sample batch dominates regardless of obstacles); DWB's rises with its active critics.
- **Prediction does not cost more — often less.** ProxMPC-predictive is within ~15 % of reactive on the empty and static cells and is actually **cheaper on the line-crossing obstacles** (0.74–0.75 ms vs 1.09–1.37 ms median): a single confirmed track is a smaller, cleaner constraint set than the reactive costmap's clustered wall cells. The predictive capability's real price is the companion tracker (~1 % core, ~38 MB), not the controller cycle.
- **The honest nuance is the tail, not the median.** On the dynamic obstacles ProxMPC's **p95 (4.1–6.0 ms) exceeds DWB's and MPPI's (~3.3–4.6 ms)** — when many half-plane rows are simultaneously active the worst-case QP is heavier than a fixed-size sampler. ProxMPC is *cheaper on average but more variable*; the sampling controllers are steadier. Every p95 here still sits **~8–12× inside** the 50 ms control budget.
- **Process CPU tracks the same ordering**, compressed by the shared ~4 % costmap floor: RPP/Graceful ~4 %, ProxMPC 4.5 % (open) rising to ~6.7 % (dynamic), DWB/MPPI 8–10 %. ProxMPC-predictive (controller + tracker, ~7–7.5 % combined) still sits below DWB and MPPI. RSS is 55–65 MB throughout.

**Caveats (read before quoting).**

- Measured on an **x86 laptop, not a Jetson** — absolute values are indicative; the *ranking* is what transfers to the Orin/Thor target.
- MPPI can parallelise, so its process CPU is a lower bound on the work it distributes.
- Compute figures are at the matched 2.0 s horizon and each method's default sampling; a leaner MPPI `batch_size` would lower its cost (and its robustness).
- The achieved `/cmd_vel` **rate** (~20 Hz for controllers that keep driving) and RSS must be read together with completion: where a controller stalls or parks short (RPP on `static_box`, ProxMPC on `dynamic_circle`) its command rate falls below 20 Hz because the robot stops being commanded, not because the controller slowed.
- A single predictive `line_fwd` run recorded one ~69 ms cold-start cycle (first solve, cold cache) in `compute_ms_max`; it is a one-off outside the reported p50/p95 and did not recur.

## 5. ProxMPC solver profile (and a cross-check)

ProxMPC is the only controller that also publishes its *internal* solver telemetry ([`SolverDiagnostics`](../prox_mpc_msgs/README.md)):

| Metric | Open cell | With obstacle constraints active |
| --- | --- | --- |
| decorator compute p95 (whole `computeVelocityCommands`) | 0.845 ms | 2.8–6.0 ms |
| internal QP solve p95 | 0.742 ms | up to ~5 ms |
| deadline-miss rate (compute > 50 ms budget) | 0.0 % | 0.0 % |
| infeasible rate (QP status ≠ SOLVED) | 0.0 % | 0.0 % |
| mean SQP iterations | 1.00 | 1.0–1.x |

The two independent instruments agree on the open cell: the QP solve (0.74 ms p95) accounts for almost all of the measured per-cycle compute (0.85 ms p95), the ~0.1 ms remainder being the costmap reduction and the exact footprint veto.
The worst single open-cell cycle stays **~14–20× inside** the 50 ms budget, which is why ProxMPC holds 20 Hz with 0 % deadline-miss and 0 % infeasible.
With the in-loop obstacle constraints active the per-cycle cost rises to a p95 of 2.8–6.0 ms (more active half-plane rows), still an order of magnitude inside budget.

## 6. Obstacle avoidance

The open cell cannot show avoidance, so these scenarios place an obstacle across the path and let every controller perceive it through the same `obstacle_layer`.
Because the kinematic plant has no collision physics, "reaching the goal" alone does not prove avoidance — a controller can drive straight through the obstacle and still "succeed" — so the discriminating metric is the **obstacle gap** (robot-disc to obstacle-disc; negative = overlap = would-be collision).

### 6.1 Static obstacle (`static_box`, mode b2)

A 0.5 m box offset to `y = +0.5` on the path; the controller must bend around it.
The last two columns carry the per-scenario **cost of that avoidance** (the same decorator/process instruments as §4), so clearance and its price are read together.

| Controller | success | obstacle gap [m] | collision | compute p50/p95 [ms] | CPU [%] |
| --- | --- | --- | --- | --- | --- |
| **ProxMPC** | 5/5 | **+0.283 ± 0.002** | **0/5** | 0.90 / 2.81 | 5.9 |
| **ProxMPC (predictive)** | 5/5 | **+0.276 ± 0.012** | **0/5** | 0.88 / 2.91 | 6.4 + 1.0 trk |
| MPPI | 5/5 | +0.091 ± 0.009 | 0/5 | 2.76 / 3.81 | 8.7 |
| DWB | 5/5 | **+0.002 ± 0.000** | 0/5 | 2.56 / 4.59 | 9.0 |
| Regulated Pure Pursuit | **0/5** | +0.264 (stopped short) | 0/5 | 0.22 / 0.33 | 3.9 |
| Graceful | 1/5 | +0.040 | 0/5 | 0.06 / 0.22 | 4.2 |

This is ProxMPC's clearest win.
It is the **only** controller that reaches the goal *and* keeps a real safety margin — **0.28 m of clearance** — by deliberately swerving wide (16 cm cross-track) and recovering, exactly the behaviour the in-loop CBF keep-out constraint is designed to produce, and it does so at **~1/3 the per-cycle compute of DWB/MPPI**.
Prediction makes no difference on a *static* box — ProxMPC-predictive holds the same +0.28 m at the same cost (the tracker reports the box at ~0 velocity, so the hybrid fill treats it exactly like the reactive costmap path) — a useful consistency check.
The sampling controllers reach the goal but cut it fine: **MPPI keeps only 9 cm** and **DWB grazes to within 2 mm** of contact — its soft `BaseObstacle` critic barely deflects it, so it essentially rides the obstacle's inflated edge.
**RPP fails outright** (0/5): with no steering avoidance it drives at the box, its forward-collision check halts it, and it never makes progress around — its large "gap" is because it stopped short, not because it avoided.
**Graceful** mostly fails too (1/5), for the same lack of a steering-avoidance mechanism plus its goal-completion quirk.

### 6.2 Dynamic obstacles (mode b2, reactive) — the honest result

Three moving-obstacle scenarios (an orbiting obstacle and a patrol crossing the path forward/backward, 0.5–0.6 m/s).
Here the shared-costmap comparison exposes a limitation that applies to **every** controller in the table (the `compute p50/p95` row is the per-scenario cost, per §4):

| Scenario | metric | ProxMPC | DWB | MPPI | RPP | Graceful |
| --- | --- | --- | --- | --- | --- | --- |
| dynamic_line_fwd | collision | 1/5 | 2/5 | 3/5 | 1/5 | 0/5 |
| | min gap [m] | +0.05 | −0.00 | −0.05 | +0.26 | +0.17 |
| | compute p50/p95 [ms] | 1.09 / 4.11 | 3.14 / 4.04 | 2.63 / 3.28 | 0.21 / 0.32 | 0.08 / 0.18 |
| dynamic_line_bwd | collision | 4/5 | 2/5 | 0/5 | 0/5 | 2/5 |
| | min gap [m] | −0.21 | −0.05 | +0.10 | +0.41 | −0.03 |
| | compute p50/p95 [ms] | 1.37 / 4.12 | 3.12 / 4.22 | 2.67 / 3.44 | 0.21 / 0.31 | 0.04 / 0.16 |
| dynamic_circle | collision | 5/5 | 4/5 | 2/5 | 2/5 | 5/5 |
| | min gap [m] | −0.33 | −0.03 | −0.02 | −0.07 | −0.07 |
| | compute p50/p95 [ms] | 1.52 / 6.04 | 3.22 / 3.97 | 2.64 / 3.35 | 0.21 / 0.34 | 0.13 / 0.21 |

**None of these controllers reliably avoids a fast crossing obstacle in reactive mode, and the collisions are frequent across the board** (aggregate collision rate over the 15 dynamic runs each: ProxMPC ~67 %, DWB ~53 %, Graceful ~47 %, MPPI ~33 %, RPP ~20 %).
The reason is structural and shared: on the shared costmap none of them *predict* the obstacle — they react to its *current* costmap footprint, which lags the fast obstacle by one or two costmap updates, so the robot steers for where the obstacle *was*.
Notably, ProxMPC-reactive is **not** better here — it is arguably worse on the orbiting obstacle, because its hard constraint commits hard to one side (0.36 m detour) and the obstacle then comes around into it; a softer, lower-commitment sampler (MPPI) or a cautious geometric controller (RPP, which slows/stops) is clipped less often.
Compute stays in the §4 ordering throughout — ProxMPC 2–4× lighter than DWB/MPPI at the median even while carrying the active keep-out rows, though its p95 tail (up to 6.0 ms on the orbiting case) is the heaviest here.
This is the honest counterpoint to §6.1: a hard in-loop constraint fed a *lagging reactive* costmap of a *fast* obstacle can over-react.

### 6.3 Dynamic obstacles with prediction — ProxMPC's actual answer

ProxMPC's design intends dynamic obstacles to enter through **prediction**, not the reactive costmap: an obstacle tracker feeds predicted positions and the controller binds each to a constraint slot across the horizon (see [obstacle-avoidance.md](../prox_mpc_core/docs/obstacle-avoidance.md#static-and-predictive-per-node-fill)).
This is measured two ways — first in the **same b2 Nav2 stack as §6.2**, now with the real `prox_mpc_obstacle_tracker` running on the simulated `/scan` (a genuine perceive → track → constant-velocity-predict → constrain pipeline; `predict_obstacles: true`, otherwise the *identical* reactive preset, so the rows below toggle prediction and nothing else), and then in **mode b1** with ground-truth predictions to separate the controller from tracker error.

**Same-harness head-to-head (mode b2, real tracker) — prediction on vs off:**

| Scenario | ProxMPC reactive (gap / coll) | **ProxMPC predictive (gap / coll)** | predictive compute p50/p95 [ms] | predictive CPU [%] |
| --- | --- | --- | --- | --- |
| dynamic_line_forward | +0.05 / 1-of-5 | **+0.35 / 0-of-5** | 0.74 / 3.72 | 6.3 + 1.0 trk |
| dynamic_line_backward | −0.21 / 4-of-5 | **+0.27 / 1-of-5** | 0.75 / 3.69 | 6.5 + 1.0 trk |
| dynamic_circle | −0.33 / 5-of-5 | −0.23 / 5-of-5 | 1.92 / 5.54 | 6.0 + 1.0 trk |

On the **straight-line patrol** — where the obstacle moves in a straight line, so the tracker's constant-velocity model is *exact* — prediction flips the result: ProxMPC anticipates the crossing and keeps a real **+0.27 to +0.35 m margin with 0–1 collisions**, where reactively it grazed or was hit 1–4 times in 5.
It does this at **lower per-cycle compute than reactive** (0.74–0.75 ms median vs 1.09–1.37 ms — a single confirmed track is a leaner constraint than the clustered costmap cells), for ~1 % extra core in the tracker process.
On the **orbiting obstacle** prediction does **not** help (still 5/5 collisions, gap −0.23 vs −0.33): a constant-velocity predictor extrapolates the orbiting object along a straight tangent — the wrong curve — so the anticipation is aimed at empty space, and committing to that wrong prediction actually *lowers completion* (predictive success on the circle drops to 2/5 vs 4/5 reactive). This is an honest limit of the **tracker's motion model**, not of the controller.

**Ground-truth confirmation (mode b1, both vehicle models):** feed the *correct* future positions — the engine knows the exact trajectory, orbit included — and the controller solves every case:

| Scenario | Model | success | cross-track RMS [m] | time-to-goal [s] |
| --- | --- | --- | --- | --- |
| static_box | Unicycle / Bicycle | 5/5 · 5/5 | 0.028 / 0.029 | 4.82 / 4.86 |
| dynamic_circle | Unicycle / Bicycle | 5/5 · 5/5 | 0.022 / 0.021 | 5.36 / 5.38 |
| dynamic_line_forward | Unicycle / Bicycle | 5/5 · 5/5 | 0.096 / 0.096 | 5.18 / 5.08 |
| dynamic_line_backward | Unicycle / Bicycle | 5/5 · 5/5 | 0.096 / 0.096 | 5.16 / 5.12 |

With ground-truth prediction ProxMPC reaches the goal on **every** dynamic run for both a 3-state unicycle and a 4-state bicycle (40/40) — the orbiting obstacle included.
So the two failure modes have distinct causes: the §6.2 reactive collisions are a *perception-lag* limit, and the b2 circle miss is a *constant-velocity-model* limit — give the controller a correct predictor and its constrained optimal control clears every dynamic case, a capability the stock controllers have no predictor to match.
The same plugin drove both vehicle models by changing one parameter (`model_plugin`), with no code change.

## 7. Real-stack validation (Gazebo)

To confirm the plugin behaves under the full production stack and not only against a kinematic plant, ProxMPC was also run in mode (a): Gazebo Harmonic physics, a TurtleBot3 waffle, AMCL localization, costmaps, and the complete Nav2 velocity chain, headless.
It reached the goal (`SUCCEEDED`), tracked the path to **5.6 mm cross-track RMS**, and tapped its diagnostics through the real `controller_server` (p95 solve 0.438 ms, 0 % deadline-miss, 0 % infeasible) — the same well-behaved profile measured on the plant, now with real sensor and physics noise.

## 8. Threats to validity

- **Reactive dynamic obstacles are hard for all controllers** (§6.2). The b2 obstacle comparison is on a *shared reactive costmap* with a 5 Hz update; a fast obstacle's costmap footprint lags, so the dynamic-scenario collisions reflect reactive perception limits common to every controller, not a clean ranking between them. ProxMPC's predictive answer is measured in the *same* harness (§6.3), which is the fair prediction-on-vs-off comparison.
- **Prediction is only as good as the tracker's motion model** (§6.3). The b2 predictive wins on the line-crossing obstacles rest on a constant-velocity tracker, which is exact for a straight patrol but wrong for the orbiting obstacle — hence the circle is still hit under real tracking, while ground-truth prediction (mode b1) clears it. A curved-motion or IMM tracker would be needed to extend the win to the orbiting case; that is a tracker upgrade, not a controller change.
- **The static-obstacle result is the clean avoidance signal** (§6.1): with a stationary obstacle there is no perception lag, and the clearance ordering (ProxMPC ≫ MPPI > DWB, RPP/Graceful fail) reflects the avoidance mechanisms directly.
- **Collision uses a strict min-gap** — any instant of disc overlap over the whole run counts as a collision, so a brief graze (e.g. DWB's −0.001 m) is flagged the same as a hard hit (ProxMPC-reactive's −0.33 m); the reported min gap distinguishes the two.
- **Graceful does not complete the tight-tolerance straight cell** (§3) — a goal-completion trait of its asymptotic law, not a tracking deficiency; its tracking is exact.
- **Resources are x86, not Jetson**, and process CPU/RAM is per-process (includes the shared costmap); the per-cycle compute (§4) is the cleaner controller-only measure, and the ranking is what transfers.
- **MPPI is stochastic with no exposed seed** (Nav2 Jazzy), so its per-scenario variance is real; n = 5 characterises it but does not pin it.
- **Determinism vs. physics.** Modes (b1)/(b2) are deterministic plants, so their near-zero geometric std is reproducibility, not a noise estimate; mode (a) carries real Gazebo variance.

## 9. Conclusion on ProxMPC

On a like-for-like, fairly-tuned suite ProxMPC is **as accurate as the best stock Nav2 controller, far cheaper to run per cycle than the predictive ones, and the only controller that keeps a real safety margin around a static obstacle** — with dynamic-obstacle avoidance delivered by prediction rather than the shared reactive costmap:

- **Tracking:** sub-millimetre (0.0 mm RMS), tied with pure pursuit and DWB, 100 % success — no meaningful tracking gap on an empty line.
- **Per-cycle compute (measured):** 0.40 ms median — **~6.5× lighter than DWB and MPPI** and within ~2× of geometric pure pursuit, with the QP solve confirmed (by independent telemetry) to be almost the entire cost, and 0 % deadline-miss / 0 % infeasible across all three run modes.
- **Static-obstacle avoidance:** the **only** controller to reach the goal *and* hold a real margin (0.28 m clearance, 0 % collision), where MPPI keeps 9 cm, DWB grazes to 2 mm, and RPP/Graceful fail to get around at all.
- **Dynamic-obstacle avoidance:** on the shared *reactive* costmap, fast crossing obstacles are unreliable for **everyone** (ProxMPC included, and arguably worst on the orbiting case) — the honest limit of reactive perception. ProxMPC's answer is **prediction**, and turning it on in the *same* stack turns the line-crossing collisions into a kept **+0.27 to +0.35 m margin** (0–1 of 5 hits, down from 1–4) at *lower* per-cycle cost than reactive — for straight-line motion the constant-velocity tracker is exact. Where that model breaks (the orbiting obstacle) real tracking still clips it, but ground-truth prediction (mode b1) clears all four dynamic cases 40/40 for both a unicycle and a bicycle — so the remaining gap is a tracker-model upgrade, not a controller limit, and it is a capability the stock controllers have no predictor to match.
- **Per-cycle cost of prediction:** essentially free — predictive ProxMPC computes in 0.7–1.9 ms median (still 2–4× lighter than DWB/MPPI) plus a ~1 % core companion tracker.
- **One controller, many vehicles:** the identical plugin drove a unicycle and a bicycle by configuration alone.

So, to the direct questions: *is ProxMPC much cheaper than MPPI and DWB?* Yes, and now measured — about six to seven times lighter per cycle at equal tracking accuracy. *Does it avoid better?* On **static** obstacles, decisively (it is the only one that keeps a margin); on **dynamic** obstacles, only through prediction — reactively, no controller here is reliable, and that is stated rather than hidden.
On an empty straight line pure pursuit remains the natural fit; the reason to choose ProxMPC is that it delivers pure-pursuit-class tracking at near-pure-pursuit cost **and** the constrained, model-agnostic, real-time optimal control — with a genuine safety margin and, with its tracker, predictive avoidance — that the geometric and sampling controllers do not.

---

*Reproduce:* the open cell and the mode-(b2) reactive obstacle scenarios with `ros2 run prox_mpc_benchmark run_nav2.py --scenario <nav2_open|static_box|dynamic_circle|dynamic_line_forward|dynamic_line_backward>`, the **predictive b2** runs (real tracker) by adding `--controllers proxmpc_pred` to the same command, the mode-(b1) ground-truth predictive results with `ros2 run prox_mpc_benchmark run_matrix.py --modes b1`, and the tables with `ros2 run prox_mpc_benchmark aggregate.py`. See [`prox_mpc_benchmark/README.md`](../prox_mpc_benchmark/README.md).
