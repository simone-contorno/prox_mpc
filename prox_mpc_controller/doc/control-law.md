# ProxMpcController - Control Law

This document covers the math the controller adds around the engine: turning the
global plan into a reference, reducing the costmap into obstacle triples,
propagating tracked dynamic obstacles, tracking the model state, and the failure
fallback.
The engine itself (the SQP loop, the QP, and the obstacle half-planes) is
documented in the core:
[NMPC/SQP/QP](../../prox_mpc_core/doc/nmpc.md) and
[obstacle avoidance](../../prox_mpc_core/doc/obstacle-avoidance.md).
For how these pieces fit into the Nav2 lifecycle and the full parameter reference,
see [architecture.md](architecture.md).

## Reference construction

The engine tracks a reference supplied as `goal_x` (an $(N_p+1) \times n$ state
reference) and `goal_u` (an $N_c \times m$ control reference).
The controller builds them from the global plan, relative to the current robot
pose, in the costmap global frame.

The plan is first transformed into the costmap global frame with a single planar
transform, and its cumulative arc length is computed.
The current pose is projected onto the plan forward-only (from the last projection
index), giving the arc-length offset $s_0$.
For each predicted node $k$ the plan is sampled at
$s_k = s_0 + v_\text{ref}\, k\, \Delta t$; sampling past the plan end holds the
final pose, with its heading taken from the last path tangent.
At each sample the reference state is the plan position $(x_k^\text{ref},
y_k^\text{ref})$ and the path-tangent heading $\theta_k^\text{ref}$.

The cruise speed $v_\text{ref}$ starts from the configured `desired_linear_vel`,
clamped by the model speed bound and by the remaining plan length over the horizon
($\,(s_\text{end} - s_0)/(N_p \Delta t)\,$) so the horizon never overshoots the
plan end.
Two optional reductions then apply:

- **Goal-checker easing.** When the goal checker reports a finite, positive xy
  tolerance, $v_\text{ref}$ is scaled by $\mathrm{clamp}(r_\text{rem}/\tau,\,0,\,1)$,
  where $r_\text{rem}$ is the remaining distance to the goal and $\tau$ is the
  `xy` tolerance, so the robot settles into the goal region.
  Unmeasured tolerance fields (reported as `lowest()`) are ignored.
- **Curvature reduction.** With a positive `curvature_gain` $g$, the peak path
  curvature $\kappa_\text{max}$ over the horizon is estimated from heading samples
  and $v_\text{ref}$ is divided by $1 + g\,\kappa_\text{max}$, so high-curvature
  segments are sampled more slowly.

### Continuous heading

Reference headings are kept continuous: each node's heading is unwrapped relative
to the previous one (and the first relative to the robot heading) with
$\theta \mathrel{+}= \mathrm{remainder}(\theta - \theta_\text{prev}, 2\pi)$.
This keeps the QP heading error from wrapping near $\pm\pi$, so the controller
turns the short way rather than spinning the long way around.

### Bicycle steering reference

For a model that declares a steering state through `getPlanarMapping()`, the
reference steering is pre-positioned from the per-node path curvature
$\kappa_k = d\theta/ds$ and the model's declared reference-point offset $a$
from `base_link` along the body x axis (`0` for `BicycleRearAxle`, the
wheelbase $L$ for `BicycleFrontAxle`):

$$
\delta_k^\text{ref} = \arctan(L\,z_k), \qquad
z_k = \frac{\kappa_k}{\sqrt{1 - (a\,\kappa_k)^2}}.
$$

At $a = 0$ (the rear axle) this reduces to the textbook rear-axle inverse
$\delta_k^\text{ref} = \arctan(L\,\kappa_k)$; at $a = L$ (the front axle) it is
algebraically the front-axle inverse $\delta_k^\text{ref} = \arcsin(L\,\kappa_k)$,
expressed as an `arctan` of the offset-adjusted $z_k$ so both plugins share one
formula.
A model with no steering state (the unicycle, $n = 3$) keeps the go-straight
default.
The control reference `goal_u` carries $v_\text{ref}$ in the speed channel and
zero elsewhere.

## Reducing the costmap to obstacle triples

The engine consumes obstacles as up to $K$ triples $(o_x, o_y, d_\text{safe})$ per
predicted node (see the core
[obstacle document](../../prox_mpc_core/doc/obstacle-avoidance.md)).
The controller produces them from the local costmap.

Every slot is first defaulted to the far sentinel `MPC::kObsFarSentinel`, which
the engine treats as non-binding, so a fixed capacity $K$ degrades cleanly to
fewer active obstacles.
For each predicted node position the controller scans a square costmap window
sized to the search radius $d_\text{safe} + \rho$, where $\rho$ is the
`obstacle_cluster_radius` (the half-window is capped at `max_obstacle_scan_cells`,
with a throttled warning when truncated).
Cells at or above `costmap_cost_threshold` (and not `NO_INFORMATION`) within the
search radius are collected, sorted by distance to the node, and clustered: the
nearest representatives at least `obstacle_cluster_radius` apart are kept, so a
wall contributes one representative point rather than many.
Each selected point becomes a triple with

$$
d_\text{safe} = r_\text{robot} + m_\text{margin},
$$

where $r_\text{robot}$ is `robot_radius` and $m_\text{margin}$ is `safety_margin`.
This reduction is entirely controller-side; the engine only consumes the triples
and re-linearizes the half-plane each iteration.

## Predictive (dynamic) obstacle avoidance

When `predict_obstacles` is set and a tracked-obstacle message is fresh (its stamp
within `obstacle_timeout` of the command stamp), the controller augments the
costmap fill with predicted-trajectory obstacles.
Obstacles are transformed into the costmap global frame; a missing transform
degrades to costmap-only for that cycle.

Each track with speed above `dynamic_speed_threshold` is a candidate (slower
tracks are left to the costmap).
A track whose radius exceeds `max_dynamic_obstacle_radius` (when set) is rejected,
guarding against extended structure (walls) reported as a moving object - its
centroid drifts at roughly robot speed and would otherwise inflate $d_\text{safe}$
and erase real costmap cells.
Candidates are ranked by their closest approach to the reference trajectory over
the horizon, and the nearest $\min(K, M)$ are kept, where $M$ is `max_dynamic_obstacles`.

Each selected obstacle $j$ is propagated to every node and bound to slot $j$ for
the whole horizon (so the half-planes track one object across nodes).
When the message carries prediction samples (`prediction_dt > 0` and a non-empty,
finite `predicted_positions` polyline - the tracker's IMM CV+CTRV forward
prediction), the controller follows that sampled trajectory at each horizon time
$k\,\Delta t + \text{age}$: piecewise-linear interpolation inside the sampled span,
and straight-line extrapolation along the last segment beyond it.
When no samples are provided (the legacy single-CV tracker path, or an empty or
ill-formed sample set), it falls back to the straight constant-velocity ray

$$
p_k = p_0 + v\,(k\,\Delta t + \text{age}).
$$

Either way the clearance for the slot grows with prediction time,

$$
d_\text{safe} = r_\text{robot} + r_\text{obs} + m_\text{margin} +
  \beta\,(k\,\Delta t + \text{age}),
$$

where $\beta$ is the `prediction_uncertainty_growth` gain and $\text{age}$ is the
message age, so the keep-out widens to cover the growing prediction error as the
prediction ages.
The remaining slots are filled from the costmap (hybrid), excluding cells inside
each dynamic obstacle's **current** footprint (its radius plus the cluster radius)
so the moving object is not counted twice.
With predictions off, no message, or a stale one, the fill reduces exactly to the
costmap-only result.

## Tracking the model state

`setPose` needs a full $n$-dimensional state.
The unicycle state $[x, y, \theta]$ maps directly to the Nav2 planar pose.
The bicycle adds a steering angle $\delta$ that the Nav2 pose does not provide, so
the controller tracks it from the previous solve (it stores $x_\text{sol}(1, 3)$
as the next steering state) and feeds the full $[x, y, \theta, \delta]$ to the
engine.

## Control-to-Twist mapping

On a converged solve the first optimal control $u_0$ is mapped to a body twist by
the model, `model->toTwist(u0)`, after setting the current state on the model.
The unicycle maps $[v, \omega]$ directly; the bicycle maps $[v, \dot{\delta}]$ to
$v$ and the derived yaw rate $\omega = v \sin(\delta) / L$.
Keeping the mapping in the model means the controller is agnostic to the model's
control semantics.

## Speed limits

Nav2 may impose a runtime speed limit through `setSpeedLimit(limit, percentage)`.
The controller converts it to an absolute speed (a fraction of the model maximum
when `percentage` is set, restoring the model bound on `NO_SPEED_LIMIT`), clamps
it to the model bound, and applies it to the retained model handle with
`model->updateIneq("u", 0, -v_lim, v_lim)`, so the bound takes effect
on the next solve without rebuilding the problem.
`setSpeedLimit` itself only caches the request: it runs on the node's executor
thread while `computeVelocityCommands` runs on the action server's thread, so the
model's inequality map is mutated on the control thread only - at the top of the
next control cycle, or in `configure` for a limit received before the model was
loaded. A new limit therefore takes effect from the next cycle, never mid-solve.

## Failure fallback (braking)

`MPC::solve` reports convergence through `qp_info.status` and takes no safety
action on failure (see the [NMPC document](../../prox_mpc_core/doc/nmpc.md)).
The controller owns the reaction so the policy stays decoupled from the engine.

On a non-converged cycle the controller does **not** command a hard zero, which
would be an instantaneous, dynamically infeasible stop.
Instead it ramps the model's own controls toward zero under the model's own
control-rate (`du`) bounds, respecting each bound's sign asymmetry, and maps
the ramped controls through the model's `toTwist()` - the same seam the
accepted command path uses, rather than writing `angular.z` directly.
This matters because the bicycle's second control is a steering rate, not a
body yaw rate: `du[1]` bounds `delta_dot` (a steering acceleration, not a yaw
acceleration), so only a control-space ramp maps correctly for that model,
while the unicycle's second control already *is* a body yaw rate and the two
forms coincide for it.

Each control channel ramps down from the velocity the `controller_server`
measured for this cycle (RPP/MPPI style), so the brake tracks the robot's
actual motion rather than a stale command.
The measurement is a `base_link` twist and the channels are model controls, so
it is carried across by the model's own `fromTwist()` inverse rather than
written in directly: for a front-axle-referenced model the speed control is a
front-wheel speed, which only the linear and angular components together
determine.
That inverse states, per channel, what a body twist determines, and the
controller seeds exactly the channels it reports as determined; it names no
channel of its own.
A channel reported undetermined - a steering rate, which a twist shows the
effect of but not the value of - keeps its last commanded value, and so does one
whose measurement is itself non-finite (NaN or $\pm\infty$), so a broken
velocity estimate decelerates the last command instead of entering the ramp and
an infinite measured velocity can never reach the published command.
Each step advances by `brake_period_s` when the operator set a positive value,
otherwise by the measured inter-cycle period, clamped between the configured
`dt` and twice `dt` so a server running slower than `dt` still brakes at the
model's declared rate while a stale measurement cannot collapse the ramp into
one step.
A model that declares a steering state has its belief decayed toward zero at
its own declared steering-rate bound on every rejected cycle, rather than
frozen at the last accepted value, so the next solve does not linearize about
an angle a downstream twist-to-steering converter has already moved the wheels
away from under the braking command.

Because a zero deceleration limit would leave the ramp stuck at the current
velocity forever, a model that declares no `du` bound for either channel (or no
`u[0]` bound for the speed cap) fails `configure` with a
`nav2_core::ControllerException` naming the missing bound.
The same ramp serves the cancel request, the footprint veto, a non-finite pose,
and a non-finite command.
A counter tracks consecutive solver failures; once it exceeds
`max_solver_failures`, the controller raises `nav2_core::NoValidControl` so the
Nav2 behavior tree stops and replans rather than crawling on a decaying command.
A footprint veto and a non-finite pose use the same brake but are handled
distinctly: the veto counts on its own counter, so it does not consume the
solver-failure budget, though a veto persisting beyond that same budget raises
`nav2_core::NoValidControl` in turn.

## Discrete-time control barrier coupling

The engine's per-node half-plane is the $\gamma = 1$ case of a discrete-time
control-barrier-function constraint $h(x_{k+1}) \ge (1 - \gamma)\, h(x_k)$, which
couples consecutive nodes for smoother avoidance.
The coupling requires the **same** obstacle at nodes $k$ and $k+1$. Both the
predictive fill and the costmap (static) fill provide this: the predictive
fill binds each tracked obstacle to a fixed slot across the horizon, and the
costmap fill now binds each scanned object to a fixed slot across every node
it appears at, rather than re-selecting nearest candidates independently per
node (see [obstacle avoidance](../../prox_mpc_core/doc/obstacle-avoidance.md)
for the core-side guard this pairs with).
That stability holds only *within* one control cycle; nothing yet holds a slot
to the same object *across* cycles, so a `cbf_gamma` below 1 can still couple
different physical objects at a cycle boundary when two candidates are close
in rank.

`cbf_gamma` exposes $\gamma$: 1.0 is the pointwise constraint, and a value
below 1 lets the safety margin decay gradually instead of binding at every
node. The coupling is enforced through the slack penalty like every other
obstacle row, not as a hard barrier, so whether a given `cbf_gamma` measurably
changes a trajectory at the shipped `w_weight` has not yet been benchmarked;
treat it as a tunable option protected by the core-side guard and the
within-cycle slot stability above, not as a settled recommendation for dense
fields.
