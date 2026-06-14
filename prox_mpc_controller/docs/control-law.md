# ProxMpcController — Control Law

This document covers the math the controller adds around the engine: turning the
global plan into a reference, reducing the costmap into obstacle triples, tracking
the model state, and the failure fallback.
The engine itself (the SQP loop, the QP, and the obstacle half-planes) is
documented in the core:
[NMPC/SQP/QP](../../prox_mpc_core/docs/nmpc.md) and
[obstacle avoidance](../../prox_mpc_core/docs/obstacle-avoidance.md).
For how these pieces fit into the Nav2 lifecycle see
[architecture.md](architecture.md).

> **Status: design.**
> The control law is under development; this document defines the intended math
> and the contract with the engine, not current behavior.

## Reference construction

The engine tracks a reference trajectory supplied as `goal_x` (an
$(N_p+1) \times n$ state reference) and `goal_u` (an $N_c \times m$ control
reference).
The controller builds them from the global plan — a sequence of poses — relative
to the current robot pose.

For each predicted node $k$ the plan is sampled at the arc length the robot is
expected to have travelled, roughly $s_k = s_0 + v_\text{ref}\, k\, \Delta t$ from
the projection $s_0$ of the current pose onto the plan.
At that point the reference state is the plan position $(x_k^\text{ref},
y_k^\text{ref})$ and the heading $\theta_k^\text{ref}$ taken from the path tangent.
The reference speed $v_\text{ref}$ is the configured cruise speed, reduced near
high-curvature segments and clamped by any active speed limit.
For the bicycle, the reference steering follows the path curvature
$\kappa$, $\delta_k^\text{ref} = \arctan(L\,\kappa_k)$; the unicycle has no
steering state.
The control reference `goal_u` carries $v_\text{ref}$ in the speed channel and
zero in the steering-rate or yaw-rate channel.

## Reducing the costmap to obstacle triples

The engine consumes obstacles as up to $K$ triples $(o_x, o_y, d_\text{safe})$ per
predicted node (see the core
[obstacle document](../../prox_mpc_core/docs/obstacle-avoidance.md)).
The controller produces them from the local costmap and any sensor obstacle
layers.

Occupied cells are those at or above `costmap_cost_threshold`.
For each predicted position $p_k$ the controller selects the $K$ nearest occupied
points or clusters; a distance transform (or ESDF) of the local costmap makes the
nearest-obstacle query cheap, and `obstacle_cluster_radius` groups adjacent cells
so a wall contributes one representative point rather than many.
Each selected point becomes a triple with

$$
d_\text{safe} = r_\text{robot} + r_\text{inflation} + m_\text{margin},
$$

where $r_\text{robot}$ is the robot disc radius (`robot_radius`),
$r_\text{inflation}$ is the costmap inflation already baked into the cost, and
$m_\text{margin}$ is `safety_margin`.
Nodes or slots with no nearby obstacle are filled with the far sentinel
`MPC::kObsFarSentinel`, which the engine treats as non-binding, so a fixed
capacity $K$ degrades cleanly to fewer active obstacles.
This reduction is entirely controller-side; the engine only consumes the triples
and re-linearizes the half-plane each iteration.

## Tracking the model state

`setPose` needs a full $n$-dimensional state.
The unicycle state $[x, y, \theta]$ maps directly to the Nav2 planar pose.
The bicycle adds a steering angle $\delta$ that the Nav2 pose does not provide, so
the controller tracks it — integrating $\delta \mathrel{+}= \Delta t\,
\dot{\delta}$ from the previous steering-rate command, or reading a steering
sensor when available — and feeds the full $[x, y, \theta, \delta]$ to the engine.
The same $\delta$ must be set on the model before `toTwist`, because the bicycle's
yaw rate is derived from it as $\omega = v \sin(\delta) / L$.

## Control-to-Twist mapping

On a converged solve the first optimal control $u_0$ is mapped to a body twist by
the model, `model->toTwist(u0)`.
The unicycle maps $[v, \omega]$ directly; the bicycle maps $[v, \dot{\delta}]$ to
$v$ and the derived $\omega$.
Keeping the mapping in the model means the controller is agnostic to the model's
control semantics.

## Speed limits

Nav2 may impose a runtime speed limit through `setSpeedLimit(limit, percentage)`.
The controller converts it to an absolute speed (a fraction of the model's maximum
when `percentage` is set) and applies it to the retained model handle with
`model->updateIneq("u", 0, -v_\text{lim}, v_\text{lim})`, so the bound takes effect
on the next solve without rebuilding the problem.

## Failure fallback (braking)

`MPC::solve` reports convergence through `qp_info.status` and takes no safety
action on failure (see the [NMPC document](../../prox_mpc_core/docs/nmpc.md)).
The controller owns the reaction so the policy stays decoupled from the engine.

On a non-converged cycle the controller does **not** command a hard zero, which
would be an instantaneous, dynamically infeasible stop.
Instead it decelerates the last command toward zero at the robot's deceleration
limit.
The limit is the magnitude of the model's lower control-rate bound on speed
(`du` for the speed channel), $a_\text{dec} = \lvert a_\text{min} \rvert$, so over
one step

$$
v_\text{cmd} = \max\!\big(0,\; v_\text{prev} - a_\text{dec}\, \Delta t\big),
$$

and the yaw rate is ramped toward zero the same way with the angular limit.
This is self-contained and does not assume a downstream velocity smoother; a
smoother, if present, only adds further smoothing.

A counter tracks consecutive failures.
Once it exceeds `max_solver_failures`, the controller raises a recovery so the
Nav2 behavior tree stops and replans, rather than crawling on a decaying command.

## Looking ahead: control barrier functions

The engine's per-node half-plane is the $\gamma = 1$ case of a discrete-time
control-barrier-function constraint
$h(x_{k+1}) \ge (1 - \gamma)\, h(x_k)$, which couples consecutive nodes for
smoother avoidance.
That coupling requires the **same** obstacle at nodes $k$ and $k+1$, so enabling
it depends on the controller associating obstacles across the horizon rather than
feeding the nearest point independently per node.
Until then `cbf_gamma` stays at `1.0` and reduces to the pointwise constraint.
