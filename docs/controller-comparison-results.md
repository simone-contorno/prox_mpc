# Controller Comparison Results

This document reports how the ProxMPC controller compares against the stock Nav2 controllers under identical, reproducible, and *fairly-tuned* conditions, and what the ProxMPC stack measured across its own scenario suite.
It is the narrative companion to the auto-generated tables in [`prox_mpc_benchmark/README.md`](../prox_mpc_benchmark/README.md); the harness that produced every number is described in [the package guide](prox-mpc.md#8-prox_mpc_benchmark--the-measurement-harness).

All numbers below are measured, reproducible, and reported as `mean ± std` (population) over fixed-seed repeats — the precision signal.
Nothing is hand-tuned to favour one controller: every controller drives the *same* plant from the *same* start to the *same* goal, at a *matched operating point* (Section 2.1), and is measured by the *same* instrumentation — including a timing decorator that wall-clock times every controller's per-cycle compute identically.

## 1. What is being compared, and what is not

The head-to-head comparison runs on the **open-world cell**: an empty 7 × 7 m room, a single straight traverse from `(-2.5, 0)` to `(2.5, 0)` (5 m), no obstacles.
This cell is deliberately a *pure path-tracking* task.
It isolates three things on an exactly equal footing — **tracking fidelity** (how tightly each controller holds the reference), **per-cycle compute cost** (measured, not inferred), and **process resource use** (CPU, memory, achieved rate) — without confounding them with perception or obstacle geometry.

It is **not** a test of obstacle avoidance.
An empty straight line is the task a geometric pursuit controller is provably optimal for, so the open cell *understates* the advantage of a constrained optimal-control method.
ProxMPC's avoidance behaviour is therefore reported separately, in [Section 6](#6-where-the-mpc-formulation-pays-off-obstacle-scenarios), on the scenarios that actually contain moving and static obstacles.

## 2. Test conditions

| Condition | Value |
| --- | --- |
| ROS 2 / Nav2 | Jazzy / Nav2 1.3.12 |
| Host | x86-64 workstation, Intel i7-10750H, 12 logical cores — **not a Jetson** |
| Run mode | **(b2)** Nav2 stack on a kinematic plant, no Gazebo (see [run modes](prox-mpc.md#9-running-the-stack-the-three-modes)) |
| Plant | Unicycle body-twist integrator at 50 Hz, identical for every controller |
| Localization | Exact (static `map → odom` identity; the plant pose is ground truth) |
| Map | `prox_mpc_open` — empty 7 × 7 m room, free interior `[-2.95, 2.95] m` |
| Task | straight 5 m traverse `(-2.5, 0) → (2.5, 0)`, goal tolerance 0.25 m |
| Planner | `NavfnPlanner` (GridBased), shared by all controllers |
| Control rate | 20 Hz (`controller_frequency`); real-time budget 50 ms/cycle |
| Repeats | 3 fixed-seed repeats per controller |
| Controllers | ProxMPC (Unicycle), DWB, MPPI, Regulated Pure Pursuit |
| Compute timing | a `nav2_core::Controller` **timing decorator** wraps every controller and times its `computeVelocityCommands` identically |

### 2.1 Fair tuning: what is equalised and what is not

The four controllers are different algorithms, so their *internal* knobs are not the same quantity and cannot be set "equal" without meaning something different for each.
The fair approach is to equalise the **shared operating envelope** and leave each method's intrinsic sampling at its upstream default:

- **Equalised** — control rate (20 Hz), max linear speed (0.5 m/s), goal tolerance (0.25 m), and the **prediction horizon: 2.0 s for all predictive controllers** (ProxMPC `np·dt = 20 × 0.1`; DWB `sim_time = 2.0`; MPPI `time_steps × model_dt = 40 × 0.05`). Regulated Pure Pursuit is geometric and has no prediction horizon.
- **Left at upstream defaults** — the intrinsic sampling counts: **MPPI `batch_size = 2000`** (the number of randomly-perturbed candidate trajectories it rolls out and importance-weights per cycle — its dominant cost knob), DWB's `20 × 20` velocity-sample grid, and ProxMPC's single-QP SQP. These have no common denominator, so equalising them would be apples-to-oranges and would misrepresent each method's real operating cost. They are reported as-is, and the compute numbers below are therefore each controller's honest cost at its standard configuration and a matched horizon.

Because the plant, map, planner, goal checker, horizon, speed, rate, and instrumentation are all shared, every difference in the tables is attributable to the controller.

### Metric definitions

- **time-to-goal** — wall time from first motion to entering the goal tolerance.
- **goal error** — final distance to the goal point; all controllers stop on the *same* 0.25 m goal checker, so ~0.24 m is the checker firing, not tracking error.
- **cross-track RMS / max** — deviation from the straight reference; the tracking-fidelity metric.
- **compute p50 / p95 / max** — wall time of one `computeVelocityCommands`, measured by the decorator, **the same way for all four**.
- **CPU / RSS / rate** — the `controller_server` process's CPU (% of one core) and RSS from `/proc`, and the achieved `/cmd_vel` rate.
- **solve p50/p95/max, deadline-miss, infeasible, SQP/QP iters** — ProxMPC-only internal solver telemetry.

## 3. Tracking-fidelity comparison (open-world cell)

All four controllers reached the goal on every repeat (**12/12 runs, 100 % success**), each holding the straight path to ≈ 4.75 m of travel.

| Controller | success | time-to-goal [s] | goal err [m] | cross-track RMS [m] | cross-track max [m] |
| --- | --- | --- | --- | --- | --- |
| **ProxMPC** (Unicycle) | 3/3 | 11.19 ± 0.01 | 0.246 ± 0.000 | **0.0004 ± 0.0000** | 0.0008 ± 0.0000 |
| Regulated Pure Pursuit | 3/3 | 10.73 ± 0.74 | 0.246 ± 0.002 | 0.0000 ± 0.0000 | 0.0000 ± 0.0000 |
| MPPI | 3/3 | 9.93 ± 1.10 | 0.236 ± 0.000 | 0.0032 ± 0.0002 | 0.0042 ± 0.0000 |
| DWB | 3/3 | 13.46 ± 1.24 | 0.246 ± 0.000 | 0.0005 ± 0.0001 | 0.0014 ± 0.0002 |

At the matched 2.0 s horizon the tracking is excellent across the board: RPP is exact on the straight line (its geometric best case), and **ProxMPC (0.4 mm RMS), DWB (0.5 mm), and MPPI (3.2 mm)** all track tightly — the fair-tuning horizon notably improved the sampling controllers here versus their stock presets.
Time-to-goal spans ~9.9–13.5 s and reflects how cautiously each accelerates from the start under the shared 0.5 m/s cap; MPPI is quickest and DWB slowest/most variable, but this is a tuning-level difference, not a fidelity one.
So on *tracking*, all four are good and no controller is meaningfully "better" on an empty straight line — which is exactly why the compute comparison below is the discriminating result.

## 4. Per-cycle compute cost and process resources

This is the comparison that matters on an embedded target, and it is now **measured, not inferred**: the timing decorator wall-clock times every controller's `computeVelocityCommands` the same way, and the `controller_server` process is sampled from `/proc`.

| Controller | compute p50 [ms] | compute p95 [ms] | compute max [ms] | CPU mean [% core] | RSS peak [MB] | rate [Hz] |
| --- | --- | --- | --- | --- | --- | --- |
| Regulated Pure Pursuit | **0.207 ± 0.004** | **0.281 ± 0.020** | 0.34 | **3.70 ± 0.05** | **53.8** | 20.10 |
| **ProxMPC** (Unicycle) | **0.378 ± 0.017** | **0.770 ± 0.066** | 2.30 | 4.32 ± 0.24 | 56.8 | 20.15 |
| DWB | 2.578 ± 0.080 | 3.407 ± 0.314 | 4.27 | 8.16 ± 0.16 | 58.3 | 20.08 |
| MPPI | 2.685 ± 0.016 | 3.347 ± 0.254 | 5.32 | 8.28 ± 0.04 | 61.5 | 20.10 |

Reading the table:

- **Per-cycle compute (the headline).** ProxMPC computes a command in **0.38 ms median / 0.77 ms p95**. That is **~6.8× faster than DWB and ~7.1× faster than MPPI at the median** (≈ 4.3–4.4× at p95), and only ~1.8× slower than geometrically-trivial pure pursuit. The single-QP SQP is doing far less work per cycle than DWB's velocity-grid rollouts or MPPI's 2000-sample batch — and now that horizon and rate are matched, that gap is the pure algorithmic cost.
- **Process CPU.** Same ordering, compressed: RPP 3.7 %, ProxMPC 4.3 %, DWB 8.2 %, MPPI 8.3 %. The whole-process figure dilutes the per-cycle gap because it also carries the identical local costmap (~3.7 % floor, ≈ the RPP figure), so it reads as ~1.9× rather than ~7× — the per-cycle number is the cleaner controller-only measure.
- **Memory.** RPP 53.8 MB, ProxMPC 56.8 MB, DWB 58.3 MB, MPPI 61.5 MB peak — the plugin-attributable memory over the shared floor is ~3 MB (ProxMPC), ~4.5 MB (DWB), ~7.7 MB (MPPI), the last driven by MPPI's `batch_size × time_steps` buffers.
- **Rate.** All four sustain ~20 Hz on this host; frequency separates nobody here, but the per-cycle headroom (below) is what decides that on a slower board.

So on an empty straight line where the tracking is a wash, ProxMPC is **the second-cheapest controller to run** — a fraction of a millisecond per cycle, close to pure pursuit, and roughly seven times lighter per cycle than either sampling-based predictive controller.

**Caveats (read before quoting).**

- Measured on an **x86 laptop, not a Jetson** — absolute values are indicative; the *ranking* is what transfers to the Orin/Thor target.
- CPU is a fraction of one core; MPPI can parallelise, so its process CPU is a lower bound on the work it distributes.
- The compute figures are at the matched 2.0 s horizon and each method's default sampling; a leaner MPPI `batch_size` would lower its cost (and its robustness), so these are honest operating-point numbers, not lower bounds.

## 5. ProxMPC solver profile (and a cross-check)

ProxMPC is the only controller that also publishes its *internal* solver telemetry ([`SolverDiagnostics`](../prox_mpc_msgs/README.md)), which lets us decompose its per-cycle cost:

| Metric | Value |
| --- | --- |
| decorator compute time p95 (whole `computeVelocityCommands`) | 0.770 ms |
| internal QP solve p95 | 0.667 ms |
| deadline-miss rate (compute > 50 ms budget) | 0.0 % |
| infeasible rate (QP status ≠ SOLVED) | 0.0 % |
| mean SQP iterations | 1.00 |

The two independent instruments agree: the QP solve (0.67 ms p95) accounts for almost all of the measured per-cycle compute (0.77 ms p95), with the ~0.1 ms remainder being the costmap reduction and the exact footprint veto — a good validation that both numbers are real.
The worst single cycle stays **~14–20× inside** the 50 ms budget, which is why ProxMPC holds 20 Hz with a 0 % deadline-miss and 0 % infeasible rate.
The same profile holds in Gazebo (mode a: p95 solve 0.438 ms) and, with the in-loop obstacle constraints active, across the obstacle scenarios below (p95 2.4–5.9 ms — still an order of magnitude inside budget).

## 6. Where the MPC formulation pays off: obstacle scenarios

The open cell has no obstacles, so it cannot show ProxMPC's distinguishing capability: avoiding obstacles **inside** the optimisation while tracking the plan.
These scenarios were run in mode (b1) — the engine's own deterministic plant, 5 fixed-seed repeats each — across both vehicle models (`r2d2` = Unicycle, `bike` = Bicycle), with a single static box and three moving-obstacle patterns.
ProxMPC reached the goal on **every run (40/40), with 0 % infeasible cycles**, bending around each obstacle via the soft keep-out constraint and recovering onto the path.

| Scenario | Model | success | goal err [m] | cross-track RMS [m] | peak obstacle slack [m] | solve p95 [ms] |
| --- | --- | --- | --- | --- | --- | --- |
| static_box | Unicycle | 5/5 | 0.175 | 0.055 | 0.278 | 4.49 |
| static_box | Bicycle | 5/5 | 0.235 | 0.022 | 0.626 | 5.89 |
| dynamic_circle | Unicycle | 5/5 | 0.224 | 0.057 | 0.209 | 2.40 |
| dynamic_circle | Bicycle | 5/5 | 0.221 | 0.063 | 0.223 | 3.59 |
| dynamic_line_forward | Unicycle | 5/5 | 0.184 | 0.020 | 0.165 | 2.93 |
| dynamic_line_forward | Bicycle | 5/5 | 0.238 | 0.011 | 0.245 | 3.66 |
| dynamic_line_backward | Unicycle | 5/5 | 0.184 | 0.020 | 0.165 | 2.93 |
| dynamic_line_backward | Bicycle | 5/5 | 0.238 | 0.011 | 0.245 | 3.61 |

The non-zero **cross-track RMS** here is the controller *deliberately leaving* the reference to clear the obstacle, and the **peak obstacle slack** quantifies how far the soft keep-out was relaxed (largest, 0.63 m, for the static box the Bicycle must swing widest around).
The same controller plugin produced all of this for both a 3-state unicycle and a 4-state bicycle by changing one parameter (`model_plugin`) — no code change — the second differentiator the open cell cannot show.

## 7. Real-stack validation (Gazebo)

To confirm the plugin behaves under the full production stack and not only against a kinematic plant, ProxMPC was also run in mode (a): Gazebo Harmonic physics, a TurtleBot3 waffle, AMCL localization, costmaps, and the complete Nav2 velocity chain, headless.
It reached the goal (`SUCCEEDED`), tracked the path to **5.6 mm cross-track RMS**, and tapped its diagnostics through the real `controller_server` (p95 solve 0.438 ms, 0 % deadline-miss, 0 % infeasible) — the same well-behaved profile measured on the plant, now with real sensor and physics noise.

## 8. Threats to validity

- **The open cell is trivial on tracking** — an obstacle-free straight line cannot separate the controllers on fidelity; it is chosen to compare compute and resources on equal footing. Capability differences show in the obstacle scenarios (Section 6), where only ProxMPC was run.
- **Resources are x86, not Jetson**, and process CPU/RAM is per-process (includes the shared costmap); the per-cycle compute (Section 4) is the cleaner controller-only measure, and the ranking is what transfers.
- **Compute is at chosen sampling defaults.** MPPI's cost scales with `batch_size`; these are operating-point figures at a matched horizon, not each method's absolute floor.
- **Determinism vs. physics.** Modes (b1)/(b2) are deterministic plants, so their near-zero geometric std is reproducibility, not a noise estimate; mode (a) carries real Gazebo variance.

## 9. Conclusion on ProxMPC

On a like-for-like, fairly-tuned path-tracking task ProxMPC is **as accurate as the best stock Nav2 controller and far cheaper to run per cycle than the predictive ones**:

- **Tracking:** sub-millimetre (0.4 mm RMS), tied with pure pursuit and DWB, all four at 100 % success — no meaningful tracking gap on this task.
- **Per-cycle compute (measured):** 0.38 ms median — **~7× lighter than DWB and MPPI** and within ~2× of geometric pure pursuit, with the QP solve confirmed (by independent telemetry) to be almost the entire cost.
- **Real-time headroom:** worst cycle ~14–20× inside the 50 ms budget, a sustained 20 Hz, and 0 % deadline-miss / 0 % infeasible across all three run modes — the margin that matters when the loop must share an Orin/Thor with perception and planning.
- **Memory:** second-lightest of the four, ~3 MB of plugin state over the shared floor.

And the two things the trivial cell cannot show but the rest of the suite does:

- **Constrained avoidance in the loop** — 40/40 obstacle runs, 0 % infeasible, shaping the trajectory through the optimisation with an exact footprint veto as backstop.
- **One controller, many vehicles** — the identical plugin drove a unicycle and a bicycle by configuration alone.

So, to the direct question — *is ProxMPC much faster than MPPI and DWB?* On **per-cycle compute, yes, and now measured: about seven times lighter at the median** (roughly four times at the p95 tail), at equal tracking accuracy and a matched horizon.
On *time-to-goal* it is not the quickest (that is a tuning choice, not a compute limit), and on an empty straight line pure pursuit remains the natural fit.
The reason to choose ProxMPC is that it delivers pure-pursuit-class tracking at near-pure-pursuit cost **and** the constrained, model-agnostic, real-time optimal control that the geometric and sampling controllers do not — the capability that the straight-line cell deliberately leaves on the table.

---

*Reproduce:* the cross-controller cell (with per-cycle timing + CPU/RAM/rate sampling) with `ros2 run prox_mpc_benchmark run_nav2.py`, the obstacle scenarios with `ros2 run prox_mpc_benchmark run_matrix.py --modes b1`, and the tables with `ros2 run prox_mpc_benchmark aggregate.py`. See [`prox_mpc_benchmark/README.md`](../prox_mpc_benchmark/README.md).
