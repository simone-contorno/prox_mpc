# ProxMPC - Obstacle Avoidance

This document derives the obstacle-avoidance constraints used by `prox_mpc_core`
and explains why this formulation is the standard choice for optimization-based
controllers.
For the surrounding SQP/QP machinery see [nmpc.md](nmpc.md); for the overall
design see [architecture.md](architecture.md).

## The problem and why it is hard

At each predicted node `k` the robot has a planar position
$p_k = (x_k, y_k)$.
An obstacle is reduced to a point $o = (o_x, o_y)$ with a required clearance
$d_\text{safe}$, which already folds in the robot radius, the obstacle inflation,
and a safety margin.
The robot must stay at least $d_\text{safe}$ away from the obstacle:

$$
\lVert p_k - o \rVert \ge d_\text{safe}.
$$

The feasible set here is *everything outside a disc* of radius $d_\text{safe}$
centered at $o$.
That set is **non-convex**: the straight segment between two safe points can pass
through the disc.
A Quadratic Program requires **convex** (linear) constraints, so this condition
cannot be handed to the QP directly.
The standard remedy is to replace the disc with a local convex approximation that
is refreshed as the solution evolves.

## The signed distance and its gradient

Define the **signed distance** to the inflated obstacle:

$$
h(p) = \lVert p - o \rVert - d_\text{safe}.
$$

The robot is safe exactly when $h(p) \ge 0$.
Because $\lVert p - o \rVert = \sqrt{(p_x - o_x)^2 + (p_y - o_y)^2}$, the partial
derivatives are

$$
\frac{\partial h}{\partial p_x} = \frac{p_x - o_x}{\lVert p - o \rVert},
\qquad
\frac{\partial h}{\partial p_y} = \frac{p_y - o_y}{\lVert p - o \rVert},
$$

so the gradient is the **unit vector pointing from the obstacle to the robot**:

$$
\nabla h(p) = \frac{p - o}{\lVert p - o \rVert} = n.
$$

`n` is the direction in which the robot moves *away* from the obstacle fastest.

## From the gradient to the linear constraint (the Taylor step)

A differentiable scalar function $f$ can be approximated near a point $a$ by its
**first-order Taylor expansion**:

$$
f(x) \approx f(a) + \nabla f(a)^\top (x - a).
$$

This is just the value at $a$ plus the linear change predicted by the gradient;
geometrically it is the tangent plane of $f$ at $a$.

Applying it to $h$ at the current iterate $p_k$ (the predicted position from the
previous SQP iteration) and substituting $\nabla h(p_k) = n$:

$$
h(p) \approx h(p_k) + \nabla h(p_k)^\top (p - p_k)
        = h(p_k) + n^\top (p - p_k).
$$

That is the whole step: the gradient $n$ *is* the slope of the tangent plane, and
the Taylor formula assembles the value $h(p_k)$ and that slope into a linear
function of $p$.
Requiring the approximation to be non-negative, $h(p) \ge 0$, and moving
$h(p_k)$ to the right-hand side gives a **linear** constraint in $p$:

$$
n^\top (p - p_k) \ge -h(p_k).
$$

The right-hand side is just the definition of $h$ written out, since
$-h(p_k) = -(\lVert p_k - o \rVert - d_\text{safe}) = d_\text{safe} - \lVert p_k - o \rVert$:

$$
n^\top (p - p_k) \ge d_\text{safe} - \lVert p_k - o \rVert.
$$

This already works, but it is written about the moving reference point $p_k$.
A tidier, fixed form is obtained with one substitution.
First split the left-hand side by adding and subtracting $o$:

$$
n^\top (p - p_k) = n^\top (p - o) - n^\top (p_k - o).
$$

Now use the defining property of $n$.
Because $n = (p_k - o) / \lVert p_k - o \rVert$, dotting it with the very vector it
normalizes returns that vector's length:

$$
n^\top (p_k - o)
= \frac{(p_k - o)^\top (p_k - o)}{\lVert p_k - o \rVert}
= \frac{\lVert p_k - o \rVert^2}{\lVert p_k - o \rVert}
= \lVert p_k - o \rVert.
$$

Substituting this back, the two $\lVert p_k - o \rVert$ terms cancel:

$$
\underbrace{n^\top (p - o) - \lVert p_k - o \rVert}_{n^\top (p - p_k)}
\ge d_\text{safe} - \lVert p_k - o \rVert
\quad\Longrightarrow\quad
n^\top (p - o) \ge d_\text{safe}.
$$

So the constraint is the equivalent and more intuitive **half-plane** form

$$
n^\top (p - o) \ge d_\text{safe}.
$$

Geometrically, the circular keep-out disc is replaced by the **tangent line to
the $d_\text{safe}$ circle at the point closest to the current robot position**.
"Stay outside the disc" becomes "stay on the far side of this line" - convex, and
exactly what a QP can enforce.
Because $n$ and $h(p_k)$ depend on the current iterate, they are recomputed every
SQP iteration: as the iterate moves, the tangent line rotates to follow the true
circle, so the sequence of linear approximations converges to the non-convex
boundary.

## Keeping the problem feasible: soft constraints

Several half-planes at once (multiple obstacles, or a narrow gap) can be jointly
infeasible.
To guarantee the QP always has a solution, each constraint is **softened** with a
non-negative slack $s$ that is penalized in the cost by the weight $W$:

$$
n^\top (p_k - o) \ge d_\text{safe} - s, \qquad s \ge 0.
$$

The solver drives $s$ to zero whenever the constraint can be met, and only pays
the penalty $W s^2$ to violate it when feasibility would otherwise be lost.
This is what keeps a real-time control loop from failing when the geometry is
momentarily impossible.

## Increment form (what the code assembles)

The SQP solves for **increments** about the current iterate: the position
increment $\Delta p = p - p_k$ and the slack increment $\Delta w = s - w$, where
$w$ is the current slack value.
Substituting and using $n^\top (p_k - o) = \lVert p_k - o \rVert$:

$$
n^\top \Delta p + \Delta w \ge d_\text{safe} - \lVert p_k - o \rVert - w = -h - w.
$$

This maps line for line onto the assembly:

```text
setC : C(row, p_x) = n_x,  C(row, p_y) = n_y,  C(row, slack) = 1
       obs_h = ||p_k - o|| - d_safe           (the signed distance h)
setd : low(row) = -obs_h - w,  upp(row) = +inf
```

A second block enforces $s \ge 0$ in increment form, $\Delta w \ge -w$, as a
plain lower bound on the slack variable.

## Discrete-time control-barrier-function coupling

The pointwise constraint above requires every node to be safe *independently*
($h(p_{k+1}) \ge 0$). Near a dense obstacle field this can make a near-zero
("stay put") command locally optimal - the solver still reports `SOLVED`, but the
robot stalls. The fix is a **discrete-time control-barrier-function (CBF)**
coupling between consecutive nodes (Zeng, Zhang & Sreenath, ACC 2021):

$$
h(p_{k+1}) \ge (1 - \gamma)\, h(p_k), \qquad \gamma \in (0, 1].
$$

Instead of demanding absolute safety at every node, this only requires the safety
margin to **decay no faster than the rate $\gamma$**, which stays feasible while
the robot advances and removes the stay-put local optimum. With $\gamma = 1$ it
reduces exactly to the pointwise constraint $h(p_{k+1}) \ge 0$, so the parameter
default preserves the original behavior bit-for-bit.

In increment form, with the slack and the previous-node signed distance $h(p_k)$
held constant per SQP iteration (only the constrained node $k+1$ is linearized;
the re-linearization across iterations recovers the exact value):

$$
n^\top \Delta p_{k+1} + \Delta w \ge (1 - \gamma)\, h(p_k) - h(p_{k+1}) - w.
$$

```text
setC : obs_h_prev = ||p_k - o_k|| - d_safe   (signed distance at the previous node)
setd : low(row) = (1 - cbf_gamma) * obs_h_prev - obs_h - w
```

Here $o_k$ is the obstacle position **at the previous node's time**: with a static
fill $o_k = o_{k+1}$, but with the predictive (moving) fill each node carries a
different obstacle position, so $h(p_k)$ uses the previous node's obstacle slot
(`obs[slot - max_obs]`). Node $k = 0$ is the fixed current pose, so it reuses the
node-1 obstacle. Only the constrained node's gradient $n^\top \Delta p_{k+1}$ enters
$C$; the previous node's signed distance is held constant per SQP iteration, which is
exact at convergence.

The rate $\gamma$ is the `cbf_gamma` parameter, forwarded from the controller
through `MPC::setCbfGamma`. This lets the predictive NMPC obstacle term run
alongside Nav2's planner/costmaps: Nav2 replans the global path while the MPC
predicts the robot against per-node obstacles over the horizon.

## Bounded capacity and the far sentinel

The number of obstacle constraints must be fixed before the QP is sized, so the
core reserves a **capacity** $K$ of obstacle slots per node (`setMaxObs`).
For each predicted node $k$ and slot $j$ the controller supplies a triple
$(o_x, o_y, d_\text{safe})$ through `setObs`; there are $N_p \cdot K$ slots, and
each obstacle constraint row shares a single $(k, j)$ index with its slack, so the
slack block is exactly $N_p \cdot K$ with no dangling entries.

When fewer than $K$ obstacles are present near a node, the unused slots are filled
with a **far sentinel** - a point at $(10^6, 10^6)$ with zero clearance.
Then $\lVert p_k - o \rVert$ is enormous, the constraint is satisfied with zero
slack, and the slot is provably non-binding.
This lets a fixed capacity $K$ reduce cleanly to fewer active obstacles without
resizing the problem.

## Static and predictive per-node fill

The core is agnostic to *how* the controller chooses the per-node triple
$(o_x, o_y, d_\text{safe})_{k,j}$: it only enforces the half-plane and the CBF
coupling on whatever positions it is handed.
Two fills exist on the controller side, both writing the same `setObs` contract:

- **static (costmap)** - the default. For each node the controller scans the
  local costmap around the robot's *reference* position and emits the nearest
  occupied cells. The obstacle position varies across nodes only because the robot
  moves, so the term constrains the robot against where obstacles are *now*.
- **predictive (tracked obstacles)** - opt-in. A dynamic track is propagated
  along its tracker-sampled predicted trajectory (the IMM CV+CTRV forward
  prediction), or a constant-velocity ray $o_{k,j} = p_j + v_j\,\Delta t_k$ as the
  fallback when the track carries no samples, and bound to the same slot $j$ for
  every node, so the half-planes track one physical object across the horizon;
  $d_\text{safe}$ may grow with $\Delta t_k$ as the prediction ages. Remaining
  slots are filled by the static scan (a hybrid fill).

Because both fills produce identical `(node, slot)` triples, the CBF coupling and
the rest of this derivation are unchanged. The predictive fill is specified in the
controller's [control-law.md](../../prox_mpc_controller/doc/control-law.md).

## Footprint: disc here, exact polygon elsewhere

The core treats the robot as a **disc** whose radius is folded into
$d_\text{safe}$.
A disc is rotation-invariant, so the half-plane needs only the position, not the
heading, which keeps the constraint simple and convex.
Exact polygon-footprint collision checking is intentionally **not** done in the
optimizer; it belongs to a separate safety layer (for example a collision monitor
or a footprint collision checker) that can veto a command the optimizer produced.
Separating an approximate, fast, convex avoidance term in the optimizer from an
exact, conservative safety check is the conventional division of responsibility.

## Why this is the common choice

- **Convexification.** Replacing a non-convex keep-out region by a separating
  half-plane that is re-linearized each iteration is the textbook approach
  (sequential convex programming) for collision avoidance in gradient- and
  QP-based optimization.
- **Reuse of the solver.** The constraint is linear, so it slots into the
  existing QP/SQP machinery with no special solver.
- **Costmap alignment.** $d_\text{safe}$ encodes the robot radius plus the
  costmap inflation plus a margin, so the controller can build the
  $(o_x, o_y, d_\text{safe})$ triples directly from the nearest occupied cells of
  an inflated local costmap.
- **Real-time robustness.** The soft slack guarantees the QP stays feasible, so
  the control loop never stalls on an infeasible instance.
- **A CBF generalization.** The pointwise half-plane is the $\gamma = 1$ case of
  the discrete-time control-barrier-function constraint
  $h(x_{k+1}) \ge (1 - \gamma)\, h(x_k)$ implemented above, which couples
  consecutive nodes for smoother avoidance; the cached signed distance makes it a
  localized addition rather than a redesign.

A note on scope: the controllers bundled with Nav2 are sampling- or
geometry-based rather than QP-based, so they avoid obstacles differently.
The signed-distance half-plane described here is the standard formulation for
**optimization-based (QP/SQP) model-predictive controllers**, which is what
ProxMPC is.

## Numerical guard

When the predicted position coincides with the obstacle, $\lVert p_k - o \rVert$
is zero and the normal $n = (p_k - o) / \lVert p_k - o \rVert$ is undefined.
The norm is therefore floored at a small constant before the division, which
avoids injecting `NaN` into the QP at the cost of an arbitrary (but bounded)
normal direction in that degenerate instant.
