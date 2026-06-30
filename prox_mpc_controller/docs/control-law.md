# ProxMpcController — Control Law

This document covers the math the controller adds around the engine: turning the
global plan into a reference, reducing the costmap into obstacle triples,
propagating tracked dynamic obstacles, tracking the model state, and the failure
fallback.
The engine itself (the SQP loop, the QP, and the obstacle half-planes) is
documented in the core:
[NMPC/SQP/QP](../../prox_mpc_core/docs/nmpc.md) and
[obstacle avoidance](../../prox_mpc_core/docs/obstacle-avoidance.md).
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
  tolerance, $v_\text{ref}$ is scaled by $\operatorname{clamp}(\text{remaining} /
  \text{xy\_tol},\,0,\,1)$ so the robot settles into the goal region.
  Unmeasured tolerance fields (reported as `lowest()`) are ignored.
- **Curvature reduction.** When `curvature_gain` $> 0$, the peak path curvature
  $\kappa_\text{max}$ over the horizon is estimated from heading samples and
  $v_\text{ref}$ is divided by $1 + \text{curvature\_gain}\cdot\kappa_\text{max}$,
  so high-curvature segments are sampled more slowly.

### Continuous heading

Reference headings are kept continuous: each node's heading is unwrapped relative
to the previous one (and the first relative to the robot heading) with
$\theta \mathrel{+}= \operatorname{remainder}(\theta - \theta_\text{prev}, 2\pi)$.
This keeps the QP heading error from wrapping near $\pm\pi$, so the controller
turns the short way rather than spinning the long way around.

### Bicycle steering reference

For a model with a steering state ($n > 3$) the reference steering is
pre-positioned to the per-node path curvature $\kappa_k = d\theta/ds$,
$\delta_k^\text{ref} = \arctan(L\,\kappa_k)$, using the configured wheelbase $L$.
A model without a steering state (the unicycle, $n = 3$) keeps the go-straight
default.
The control reference `goal_u` carries $v_\text{ref}$ in the speed channel and
zero elsewhere.

## Reducing the costmap to obstacle triples

The engine consumes obstacles as up to $K$ triples $(o_x, o_y, d_\text{safe})$ per
predicted node (see the core
[obstacle document](../../prox_mpc_core/docs/obstacle-avoidance.md)).
The controller produces them from the local costmap.

Every slot is first defaulted to the far sentinel `MPC::kObsFarSentinel`, which
the engine treats as non-binding, so a fixed capacity $K$ degrades cleanly to
fewer active obstacles.
For each predicted node position the controller scans a square costmap window
sized to the search radius $d_\text{safe} + \text{obstacle\_cluster\_radius}$ (the
half-window is capped at `max_obstacle_scan_cells`, with a throttled warning when
truncated).
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
costmap fill with constant-velocity predictions.
Obstacles are transformed into the costmap global frame; a missing transform
degrades to costmap-only for that cycle.

Each track with speed above `dynamic_speed_threshold` is a candidate (slower
tracks are left to the costmap).
A track whose radius exceeds `max_dynamic_obstacle_radius` (when set) is rejected,
guarding against extended structure (walls) reported as a moving object — its
centroid drifts at roughly robot speed and would otherwise inflate $d_\text{safe}$
and erase real costmap cells.
Candidates are ranked by their closest approach to the reference trajectory over
the horizon, and the nearest $\min(K, \text{max\_dynamic\_obstacles})$ are kept.

Each selected obstacle $j$ is propagated to every node and bound to slot $j$ for
the whole horizon (so the half-planes track one object across nodes):

$$
p_k = p_0 + v\,(k\,\Delta t + \text{age}), \qquad
d_\text{safe} = r_\text{robot} + r_\text{obs} + m_\text{margin}
  + \text{prediction\_uncertainty\_growth}\cdot(k\,\Delta t + \text{age}),
$$

where $\text{age}$ is the message age, so the clearance grows as the
constant-velocity assumption ages.
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
`model->updateIneq("u", 0, -v_\text{lim}, v_\text{lim})`, so the bound takes effect
on the next solve without rebuilding the problem.
A limit received before the model is loaded is cached and re-applied in
`configure`.

## Failure fallback (braking)

`MPC::solve` reports convergence through `qp_info.status` and takes no safety
action on failure (see the [NMPC document](../../prox_mpc_core/docs/nmpc.md)).
The controller owns the reaction so the policy stays decoupled from the engine.

On a non-converged cycle the controller does **not** command a hard zero, which
would be an instantaneous, dynamically infeasible stop.
Instead it decelerates the last command toward zero at the robot's deceleration
limit, read as the magnitude of the model's control-rate (`du`) bounds for the
speed and yaw channels, $a_\text{dec} = \lvert a_\text{min} \rvert$, so over one
step

$$
v_\text{cmd} = \max\!\big(0,\; v_\text{prev} - a_\text{dec}\, \Delta t\big),
$$

and the yaw rate is ramped toward zero the same way (respecting its sign).
The same ramp serves the cancel request, the footprint veto, a non-finite pose,
and a non-finite command.
A counter tracks consecutive solver failures; once it exceeds
`max_solver_failures`, the controller raises `nav2_core::NoValidControl` so the
Nav2 behavior tree stops and replans rather than crawling on a decaying command.
A footprint veto and a non-finite pose use the same brake but are handled
distinctly: the veto does not consume the failure budget.

## Discrete-time control barrier coupling

The engine's per-node half-plane is the $\gamma = 1$ case of a discrete-time
control-barrier-function constraint $h(x_{k+1}) \ge (1 - \gamma)\, h(x_k)$, which
couples consecutive nodes for smoother avoidance.
The coupling requires the **same** obstacle at nodes $k$ and $k+1$, which the
predictive fill provides by binding each tracked obstacle to a fixed slot across
the horizon.
`cbf_gamma` exposes $\gamma$: 1.0 is the pointwise constraint, and a value below 1
lets the safety margin decay gradually, which makes a dense obstacle field viable
where the pointwise term would stall the robot (see the verified predictive
configuration in
[../../prox_mpc_demo/docs/nav2-simulation.md](../../prox_mpc_demo/docs/nav2-simulation.md)).
