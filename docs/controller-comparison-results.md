# Controller Comparison Results

This document reports how the ProxMPC controller compares against the stock Nav2 controllers under identical, reproducible conditions, and what the ProxMPC stack measured across its own scenario suite.
It is the narrative companion to the auto-generated tables in [`prox_mpc_benchmark/README.md`](../prox_mpc_benchmark/README.md); the harness that produced every number is described in [the package guide](prox-mpc.md#8-prox_mpc_benchmark--the-measurement-harness).

All numbers below are measured, reproducible, and reported as `mean ± std` (population) over fixed-seed repeats — the precision signal.
Nothing here is hand-tuned per controller: every controller drives the *same* plant from the *same* start to the *same* goal and is measured by the *same* controller-agnostic instrumentation.

## 1. What is being compared, and what is not

The head-to-head comparison runs on the **open-world cell**: an empty 7 × 7 m room, a single straight traverse from `(-2.5, 0)` to `(2.5, 0)` (5 m), no obstacles.
This cell is deliberately a *pure path-tracking* task.
It isolates three things on an exactly equal footing — **tracking fidelity** (how tightly each controller holds the reference and reaches the goal), **real-time behaviour** (the achieved control rate), and **embedded resource cost** (CPU and memory) — without confounding them with perception or obstacle geometry.

It is **not** a test of obstacle avoidance.
An empty straight line is the task a geometric pursuit controller is provably optimal for, so the open cell *understates* the advantage of a constrained optimal-control method.
ProxMPC's avoidance behaviour is therefore reported separately, in [Section 6](#6-where-the-mpc-formulation-pays-off-obstacle-scenarios), on the scenarios that actually contain moving and static obstacles.

This split is intentional: compare like-for-like where the task is identical, and show the differentiator where it exists, rather than claiming a single scenario settles everything.

## 2. Test conditions

| Condition | Value |
| --- | --- |
| ROS 2 / Nav2 | Jazzy / Nav2 1.3.12 |
| Host | x86-64 developer workstation, Intel i7-10750H, 12 logical cores — **not a Jetson** |
| Run mode | **(b2)** Nav2 stack on a kinematic plant, no Gazebo (see [run modes](prox-mpc.md#9-running-the-stack-the-three-modes)) |
| Plant | Unicycle body-twist integrator at 50 Hz, identical for every controller |
| Localization | Exact (static `map → odom` identity; the plant pose is ground truth) |
| Map | `prox_mpc_open` — empty 7 × 7 m room, free interior `[-2.95, 2.95] m` |
| Task | straight 5 m traverse `(-2.5, 0) → (2.5, 0)`, goal tolerance 0.25 m |
| Planner | `NavfnPlanner` (GridBased), shared by all controllers |
| Control rate | 20 Hz (`controller_frequency`); real-time budget 50 ms/cycle |
| Repeats | 3 fixed-seed repeats per controller |
| Controllers | ProxMPC (Unicycle), DWB, MPPI, Regulated Pure Pursuit |

Because the plant, map, planner, goal checker, and instrumentation are shared, every difference in the tables below is attributable to the controller alone.
Each controller runs in its **own `controller_server` process**, which is what the resource metrics sample (Section 4).

### Metric definitions

- **time-to-goal** — wall time from first motion to entering the goal tolerance.
- **path length** — integrated travelled distance (the straight-line optimum is ≈ 5 m minus the goal-tolerance stand-off).
- **goal error** — final distance to the goal point. All controllers stop on the *same* shared goal checker at 0.25 m, so a value near 0.25 m is the checker firing, not a tracking error.
- **cross-track RMS / max** — deviation from the straight reference polyline; the discriminating tracking-fidelity metric on this cell.
- **control rate** — achieved publish rate of `/cmd_vel` while moving (vs the 20 Hz target).
- **CPU / RSS** — the `controller_server` process's CPU (% of one core) and resident memory, sampled from `/proc` during navigation (Section 4).
- **solve p50/p95/max, deadline-miss, infeasible, SQP/QP iters** — ProxMPC-only solver telemetry (the stock controllers do not publish it).

## 3. Tracking-fidelity comparison (open-world cell)

All four controllers reached the goal on every repeat (**12/12 runs, 100 % success**).

| Controller | success | time-to-goal [s] | path [m] | goal err [m] | cross-track RMS [m] | cross-track max [m] |
| --- | --- | --- | --- | --- | --- | --- |
| **ProxMPC** (Unicycle) | 3/3 | 11.37 ± 0.12 | 4.754 ± 0.000 | 0.246 ± 0.000 | **0.0004 ± 0.0000** | **0.0008 ± 0.0000** |
| Regulated Pure Pursuit | 3/3 | **10.24 ± 0.09** | 4.754 ± 0.003 | 0.246 ± 0.003 | 0.0000 ± 0.0000 | 0.0000 ± 0.0000 |
| MPPI | 3/3 | 10.60 ± 0.14 | 4.757 ± 0.002 | 0.244 ± 0.002 | 0.0095 ± 0.0001 | 0.0138 ± 0.0000 |
| DWB | 3/3 | 12.39 ± 0.42 | 4.752 ± 0.004 | 0.245 ± 0.001 | 0.0017 ± 0.0001 | 0.0040 ± 0.0003 |

Reading the table:

- **Tracking fidelity.** Regulated Pure Pursuit holds the straight line exactly (0 cross-track) — expected, since pursuing a carrot down a straight reference is the geometric case it is built for. ProxMPC is effectively tied with it: **0.4 mm RMS, 0.8 mm peak** — sub-millimetre tracking from solving the full optimal-control problem. DWB (sampling-based) sits at ~1.7 mm, and MPPI, whose strength is stochastic sampling, is the least tight here at ~9.5 mm RMS — its sampled rollouts wiggle around a trivially straight reference.
- **Goal error.** All four land at ~0.245 m, i.e. on the shared 0.25 m goal checker; this column shows the controllers agree on *where* the goal is, not a quality difference.
- **Speed.** Regulated Pure Pursuit is quickest (10.2 s), MPPI next (10.6 s), then ProxMPC (11.4 s) and DWB (12.4 s). The spread is ~2 s over a 5 m run and reflects how aggressively each controller accelerates out of the start under the same 0.5 m/s cap, not a fidelity difference.
- **Reproducibility.** The geometric metrics are tightly repeatable for ProxMPC and RPP (≈ 0 std); DWB shows the largest time spread (± 0.42 s) and MPPI the largest cross-track — consistent with their sampling nature.

The honest summary of this cell: **on a trivial open straight line, all four controllers are good, and the geometric pursuit controller is marginally fastest with zero cross-track.** ProxMPC matches it on fidelity to sub-millimetre and, as the next sections show, does so at near-pursuit resource cost while solving a formulation that generalises to obstacles and other vehicle models.

## 4. Real-time and embedded resource cost

This is the comparison that matters on an embedded target.
Each controller's `controller_server` process was sampled from `/proc` during navigation for CPU and resident memory, and the achieved control rate was measured from the `/cmd_vel` stream.

| Controller | control rate [Hz] | CPU mean [% core] | CPU peak [% core] | RSS peak [MB] | RSS mean [MB] |
| --- | --- | --- | --- | --- | --- |
| Regulated Pure Pursuit | 20.10 ± 0.00 | **3.68 ± 0.11** | 20.0 ± 0.0 | **53.3 ± 0.0** | 53.3 ± 0.0 |
| **ProxMPC** (Unicycle) | 20.09 ± 0.00 | **4.14 ± 0.13** | 20.0 ± 0.0 | 56.4 ± 0.0 | 56.4 ± 0.0 |
| DWB | 20.08 ± 0.00 | 7.76 ± 0.05 | 20.1 ± 0.1 | 57.7 ± 0.0 | 57.6 ± 0.0 |
| MPPI | 20.10 ± 0.00 | 10.19 ± 0.04 | 29.6 ± 7.7 | 63.5 ± 0.0 | 61.0 ± 0.0 |

Reading the table:

- **Control rate.** All four sustain the 20 Hz target on this host — none falls behind the loop. Frequency therefore does not separate them *here*; it is the metric that would separate them first on a constrained board, where the heaviest controller is the one at risk of missing the budget.
- **CPU.** Regulated Pure Pursuit is lightest (3.7 % of a core), and **ProxMPC is the next lightest at 4.1 %** — roughly half DWB's 7.8 % and ~40 % of MPPI's 10.2 %. The single SQP iteration and sub-millisecond solve make the optimal-control controller genuinely cheap. MPPI's CPU is both the highest and the burstiest (peak 29.6 % with a large ± 7.7 spread), reflecting its batch of sampled rollouts.
- **Memory.** Same ordering: RPP 53.3 MB, **ProxMPC 56.4 MB**, DWB 57.7 MB, MPPI 63.5 MB peak. The ~53 MB RPP figure approximates the common floor (the `controller_server` machinery plus the identical local costmap), so the plugin-attributable memory is roughly +3 MB for ProxMPC, +4 MB for DWB, and +10 MB for MPPI — MPPI's `batch_size × time_steps` trajectory buffers are the cost.

So on the embedded axes ProxMPC sits just above the geometrically-trivial pure-pursuit floor on both CPU and RAM, and clearly below the two sampling controllers — while delivering the sub-millimetre tracking of Section 3.

**Caveats (read before quoting these numbers).**

- Measured on an **x86-64 laptop, not a Jetson** — absolute CPU/RAM values are indicative; the *ranking* is what transfers to the Orin/Thor target.
- CPU is reported as a fraction of one core; a controller that parallelises (MPPI) can exceed 100 %, and its peak here is both higher and more variable.
- The `controller_server` process includes the identical local costmap and server machinery, so the figures are per-process, not pure-plugin; the differences over the shared floor are plugin-attributable.
- All controllers were given identical tunings from `config/controllers/`; resource cost scales with sampling/horizon settings, so these are operating-point figures, not absolute lower bounds.

## 5. ProxMPC solver profile

ProxMPC is the only controller in the set that publishes per-cycle solver telemetry ([`SolverDiagnostics`](../prox_mpc_msgs/README.md)), so its real-time cost is measured at the solve level, not just inferred from process CPU.
On the same open-world cell, at a 20 Hz control rate (50 ms budget):

| Metric | Value |
| --- | --- |
| solve time p50 / p95 / max | 0.313 / 0.777 / **3.630** ms |
| deadline-miss rate (solve > 50 ms budget) | **0.0 %** |
| infeasible rate (QP status ≠ SOLVED) | **0.0 %** |
| mean SQP iterations | 1.00 |
| mean QP outer iterations | 2.84 |

The solve completes in well under a millisecond at the median, with the worst observed single solve (3.6 ms) still **~14× inside** the 50 ms cycle budget — the headroom that keeps ProxMPC at 20 Hz and explains its low process CPU.
The single SQP iteration is the expected result for this near-linear tracking regime (a linear model converges in one QP solve), and zero infeasible cycles means the QP was always solvable.

This profile holds across run modes: under the full Gazebo + Nav2 stack (mode a) ProxMPC measured a p95 solve of 0.438 ms with 0 % deadline-miss and 0 % infeasible, and across the obstacle scenarios below the p95 stayed between 2.4 and 5.9 ms — still an order of magnitude inside budget even with the in-loop obstacle constraints active.

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
The same controller plugin produced all of this for both a 3-state unicycle and a 4-state bicycle by changing one parameter (`model_plugin`) — no code change — which is the second differentiator the open cell cannot show.

## 7. Real-stack validation (Gazebo)

To confirm the plugin behaves under the full production stack and not only against a kinematic plant, ProxMPC was also run in mode (a): Gazebo Harmonic physics, a TurtleBot3 waffle, AMCL localization, costmaps, and the complete Nav2 velocity chain, headless.
It reached the goal (`SUCCEEDED`), tracked the path to **5.6 mm cross-track RMS**, and tapped its diagnostics through the real `controller_server` (p95 solve 0.438 ms, 0 % deadline-miss, 0 % infeasible) — the same well-behaved profile measured on the plant, now with real sensor and physics noise.

## 8. Threats to validity

- **The open cell is trivial.** A straight, obstacle-free traverse cannot separate the controllers strongly on tracking; it is chosen to compare tracking, real-time, and resource cost on equal footing, not to crown a winner. The obstacle scenarios (Section 6) are where capability differs, and there only ProxMPC was run.
- **Resources are measured on x86, not a Jetson**, and per-process rather than per-plugin (Section 4 caveats). The ranking transfers; the absolute numbers do not.
- **Solver telemetry is one-sided.** Only ProxMPC instruments its per-cycle solve time, so Section 5 reports ProxMPC's solve profile in absolute terms; the cross-controller compute comparison is the process-level CPU of Section 4.
- **Determinism vs. physics.** Modes (b1)/(b2) are deterministic plants, so their near-zero geometric std is reproducibility, not a noise estimate; mode (a) carries real Gazebo variance. The two are complementary.

## 9. Conclusion on ProxMPC

On a like-for-like path-tracking task ProxMPC is **competitive with the best stock Nav2 controller**: it matches Regulated Pure Pursuit's straight-line tracking to sub-millimetre (0.4 mm RMS vs. 0.0 mm), reaches the goal as reliably (100 % success), and does so at **near-pursuit resource cost** — 4.1 % CPU and 56 MB, the second-lightest of the four on both axes, behind only the geometrically-trivial pure-pursuit controller and well below DWB and MPPI. It gives up only ~1.1 s of time-to-goal on a 5 m run, a tuning-level gap, not a capability gap.

What sets it apart is what the trivial cell cannot show:

- **Constrained avoidance in the loop.** ProxMPC reaches the goal on every obstacle scenario (40/40) with zero infeasible cycles, shaping the trajectory around static and moving obstacles through the optimisation itself, with an exact footprint veto as the backstop — not by deferring entirely to a planner/costmap.
- **Real-time guarantees, measured.** Sub-millisecond median solves, a worst case ~14× inside the control budget, a sustained 20 Hz, and a 0 % deadline-miss / 0 % infeasible rate, reported from real telemetry across all three run modes.
- **Embedded-friendly cost.** The cheap single-SQP solve translates into the lowest CPU and memory of any controller here except pure pursuit — the headroom that matters when the same loop has to run on an Orin or Thor alongside perception and planning.
- **One controller, many vehicles.** The identical plugin drove a unicycle and a bicycle by configuration alone, because the vehicle model is a loadable plugin.

In short, ProxMPC does not beat a pure-pursuit controller at the one thing pure pursuit is optimal for — following an empty straight line — and it does not need to.
It delivers that same tracking fidelity at comparable embedded cost **and** the constrained, model-agnostic, real-time optimal control that the geometric and sampling controllers do not, which is the reason to choose an MPC controller in the first place.

---

*Reproduce:* the cross-controller cell (with CPU/RAM/rate sampling) with `ros2 run prox_mpc_benchmark run_nav2.py`, the obstacle scenarios with `ros2 run prox_mpc_benchmark run_matrix.py --modes b1`, and the tables with `ros2 run prox_mpc_benchmark aggregate.py`. See [`prox_mpc_benchmark/README.md`](../prox_mpc_benchmark/README.md).
