# Migrating from prox_mpc_core 1.0.0

What changed on the released surface of `prox_mpc_core`, and what a 1.0.0 caller
has to do about it.
The version number this lands under is fixed by the maintainer against the final
diff and is not stated here.

## Summary

| Change | Source compatible | ABI compatible | Wire compatible |
| --- | --- | --- | --- |
| `prox_mpc::Model` gains non-pure virtuals `getPlanarMapping()` and `fromTwist()` | yes | **no** | n/a |
| `prox_mpc::MPC` gains `solveCandidate()`, `commitCandidate()`, `getCandidateFinite()`, `setU0()` and candidate members | yes | **no** | n/a |
| `MPC::init()` rejects a nonsymmetric or indefinite weight matrix | yes | yes | n/a |
| `MPC::setGoalX` / `setGoalU` reject an undersized matrix | yes | yes | n/a |
| `MPC::setNp` and `MPC::init()` reject `Nc > Np` | yes | yes | n/a |
| Structural `MPC` setters reject a post-`init()` call | yes | yes | n/a |
| `Model::updateIneq` rejects an index the model declares no bound for | yes | yes | n/a |
| The bicycle splits into two plugins; `prox_mpc_core/Bicycle` becomes a deprecated alias | yes | yes | n/a |
| `prox_mpc::Bicycle`'s emitted twist is corrected | yes | yes | n/a |
| The coupled obstacle constraint carries its previous-node gradient | yes | yes | n/a |

Nothing in this package touches a message, so nothing here is a wire event.

## Every `Model` plugin must be rebuilt

`prox_mpc::Model` gains two virtual members, `getPlanarMapping()` and
`fromTwist()`, appended after `toTwist()` in that order so they take the last two
vtable slots and every earlier slot keeps its index.

A plugin that is recompiled against the new header needs no edit: the base class
supplies a default for each. `getPlanarMapping()` reproduces the convention the
controller assumed before the hook existed - state `[x, y, yaw, (delta)]`,
control `[v, ...]`, the state referenced to `base_link`, and a steering angle at
state index 3 for any model with more than three states. `fromTwist()` is the
inverse of the default `toTwist()`: `linear.x` into control 0, `angular.z` into
control 1, every further control left non-finite to mark it undetermined.

A plugin `.so` built against 1.0.0 and **not** rebuilt dispatches through a stale
vtable. There is no build-time signal for this and no runtime warning; the
symptom is wrong dispatch inside a motion-control path. Rebuild every out-of-tree
`prox_mpc::Model` plugin against the new `prox_mpc_core`. Packages installed from
the buildfarm are covered, because jazzy sets `abi_incompatibility_assumed: true`
and rebuilds dependents.

### Declaring a mapping

Override `getPlanarMapping()` when the model does not follow the default
convention, and **always** when it carries a steering angle: the bundled
controller now reads the wheelbase from the model rather than from the
`model_params.L` parameter it forwards, and rejects at `configure()` a model that
carries a steering angle without declaring a usable wheelbase.

```cpp
prox_mpc::PlanarMapping getPlanarMapping() const override
{
  prox_mpc::PlanarMapping mapping;
  mapping.idx_steering = 3;
  mapping.idx_steer_rate = 1;              // control carrying the steering rate
  mapping.ref_offset_x = this->params(0);  // reference point in base_link [m]
  mapping.wheelbase = this->params(0);
  return mapping;
}
```

`idx_steer_rate` names the control whose declared bound sets how fast a consumer
may move its belief about the steering angle. The default reproduces the previous
inference - control index 1 for a model with more than three states and more than
one control - so a model that declares nothing behaves exactly as it did.

### Inverting the twist mapping

Override `fromTwist()` when the model overrides `toTwist()` and its controls are
still recoverable from a body twist, so the two mappings agree. A consumer that
has a measured twist and needs the control that produced it - a deceleration
ramp seeding itself from the robot's actual velocity, for instance - reads this
hook; without an override it would read the base class default, which is the
inverse of a mapping the model does not use.

The return value is a per-channel declaration, not just a value: a finite entry
is a measurement of that control, and a non-finite one says the twist does not
determine it, leaving the consumer to keep whatever value it already holds.
The bundled Nav2 plugin applies exactly that, so a channel this hook misreports
is a channel that plugin will seed wrongly.

`BicycleFrontAxle` overrides it, because control 0 is a front-wheel speed rather
than the `base_link` speed `toTwist()` emits:

```cpp
VectorXd fromTwist(const geometry_msgs::msg::Twist & twist) const override
{
  VectorXd u = VectorXd::Constant(2, std::numeric_limits<double>::quiet_NaN());
  const double wheel_arc = twist.angular.z * this->params(0);
  const double speed = std::sqrt(twist.linear.x * twist.linear.x + wheel_arc * wheel_arc);
  u(0) = (twist.linear.x < 0.0) ? -speed : speed;
  return u;
}
```

The inverse is taken from both twist components rather than by dividing
`linear.x` by `cos(delta)`, which vanishes at that model's own `+/- pi/2`
steering bound. Control 1 is left non-finite: a twist shows the steering angle's
effect, not the rate the angle is changing at, so the caller keeps whatever
value it already holds for that channel.

`BicycleRearAxle` overrides it as well, for the second half of the same reason.
Its control 0 *is* the `base_link` speed, so the default would read that channel
correctly, but its control 1 is a steering rate while the default returns
`angular.z` there - a body yaw rate, and the two are not the same quantity. A
model that overrides `toTwist()` needs this hook even when its speed channel
happens to agree with the default.

## `MPC` gains candidate state

`MPC` gains four public methods and six data members. The methods are non-virtual
and symbol-additive; the data members change `sizeof(MPC)`, so a consumer that
constructs, holds by value, or derives from `MPC` against the 1.0.0 header while
linking the new library is binary-incompatible. Rebuild.

`solve()` keeps its 1.0.0 contract exactly: it runs the cycle, retains the
increments whatever the QP reported, and advances the previous control only on a
converged solve. A caller that wants a cycle it can reject uses the staged pair
instead:

```cpp
auto [x, u] = mpc->solveCandidate();   // retains nothing
if (mpc->qp_info.status == PROXQP_SOLVED && mpc->getCandidateFinite() && myGatesPass(x, u)) {
  mpc->commitCandidate();              // now the state advances
}
```

`commitCandidate()` refuses a candidate that did not converge or is not finite
over the whole horizon, and returns whether it committed. A caller that applies
something other than the candidate's first control - a deceleration ramp, say -
calls `setU0()` with what it applied, so the next cycle's control-rate constraint
is anchored on the command the robot actually received.

## Configuration-time validation now throws

Five inputs that 1.0.0 accepted silently are now rejected.

- `setGoalX` and `setGoalU` throw `std::invalid_argument` for a matrix with too
  few rows for the configured horizon, and for a wrong column count once
  `init()` has read the model's dimensions. 1.0.0 stored the matrix and read out
  of bounds during assembly, which `EIGEN_NO_DEBUG` left unchecked.
- `init()` throws `std::invalid_argument` unless `Q`, `S`, `R` and `W` are
  finite, symmetric and positive semidefinite. 1.0.0 accepted an asymmetric or
  indefinite weight and silently solved the symmetrised problem instead.
- `setQ`, `setR`, `setS`, `setW`, `setNp`, `setNc`, `setdt`, `setT`,
  `setMaxIntIterQP`, `setMaxExtIterQP`, `setGuess`, `setQPtype`, `setCbfGamma`
  and `setMaxObs` throw `std::logic_error` when called after `init()`. 1.0.0
  returned quietly while mutating only their own members, leaving the buffers
  and the QP object sized for the previous configuration. `setMaxObs` sizes the
  obstacle matrix and the QP's slot count at `init()`; `setCbfGamma` reaches the
  solver only through `configProxQP()`, which `init()` runs once, so a later
  call changed the `MPC` member and nothing the solver reads.
- `setNp` throws `std::invalid_argument` for a prediction horizon shorter than
  an already-set control horizon, and `init()` throws it for a pair that reached
  it unchecked. 1.0.0 compared the two inside `setNc` alone, and only once `Np`
  was already known, so `setNc` before `setNp` - and `setNc` with no `setNp` at
  all - sized a QP with more control nodes than prediction nodes.
- `Model::updateIneq` throws `std::invalid_argument` when the model declares no
  existing bound for the `var` and index it is given. 1.0.0 fell through its
  four search branches and returned having changed nothing, which made the call
  safe to issue without first establishing that the bound exists - the pattern
  the bundled controller's own `applySpeedLimit` uses, calling `updateIneq` on
  the model's speed control with no preceding existence check. A caller that
  relied on the no-op must now declare the bound with `setIneq` first, or read
  `getIneq(var)` and skip the update when the index is absent.

Call the structural setters before `init()`, size the goal matrices to the
horizon, pass symmetric positive-semidefinite weights, and update only a bound
the model already declares. The bundled Nav2 plugin already did all four: it
rejects a model that declares no bound on its speed control at `configure()`,
before any speed limit can reach `updateIneq`. A stack that uses it sees no
change.

## The bicycle model

`prox_mpc_core/Bicycle` still resolves and still loads. It is now a deprecated
alias for `prox_mpc_core/BicycleFrontAxle`, warns once per process, and is
removed in the next major release. The warning names both changes below, so an
operator who reads only the log still learns that the alias is not the model
1.0.0 gave them. Two things about it changed:

- `toTwist()` returns the body twist of `base_link`. Its `linear.x` is
  `v cos(delta)`, the projection of the front-wheel speed on the body axis, not
  the front-wheel speed itself. `angular.z` is unchanged.
- The bundled controller now transforms the model's pose to `base_link` before
  the footprint check, and derives the steering reference for the front axle
  (`asin(L * kappa)`) rather than for the rear axle (`atan(L * kappa)`).

Select `prox_mpc_core/BicycleRearAxle` for a vehicle whose model state refers to
the rear axle. It propagates `v cos(theta)` with `theta_dot = v tan(delta) / L`,
declares its reference point at the `base_link` origin, and bounds its steering
angle at 1.0 rad, because its yaw law diverges as the angle approaches `pi/2`.

## The coupled obstacle constraint

The discrete-time CBF constraint now carries the previous node's clearance
gradient, completing a linearisation that was first-order exact in one node's
position only.

Two further corrections land with it, both gated on `cbf_gamma < 1` and both
inert at the shipped default. The first coupled constraint compares the clearance
at the current pose against the clearance one step ahead; the obstacle position
used on the current-pose side is now the obstacle's position *now*, recovered by
extending the first two blocks backward. It was previously the obstacle's
one-step-ahead position, which evaluated the two sides at different obstacle times
and was anti-conservative for a closing obstacle. Measured closed loop against a
0.5 m/s crossing obstacle, the realized closest approach moves by at most 0.2 mm,
in the direction of more clearance; a static fill is bit-identical, because its
blocks are equal and the reconstruction returns the first one unchanged.

The gradient columns the first node would have written are also dropped. The
initial state is pinned to the current pose by an identity equality, so those
coefficients could never influence the solution while still enlarging the
constraint matrix. The nonzero count at the bundled configuration falls from 238
to 236 below `cbf_gamma = 1.0` and stays at 198 at it.

At the shipped `cbf_gamma = 1.0` nothing changes: the block is not written, so
it does not enter the constraint matrix's sparsity pattern, and the constraint
matrix's nonzero count at the bundled configuration is unchanged.

Below 1.0 the mode behaves as the rate condition it is, which it did not before:
the clearance is allowed to decay toward the keep-out as `cbf_gamma` falls,
producing a closer and less lateral pass. Measured on the bundled fixture with a
1.0 m keep-out, closest approach moves from 0.906 m to 0.793 m at
`cbf_gamma = 0.3` and from 0.910 m to 0.860 m at 0.5. A configuration that lowers
`cbf_gamma` should re-check its `robot_radius` and `safety_margin` against that.

## Deliberately not done

`Model::getIneq()` still returns `const std::map<int, std::vector<double>> &`,
with the bound's vector index stored as a `double` in the value. Changing the
index to an integral type is a source and ABI break on a released signature that
every out-of-tree model compiles against, for a typing improvement the existing
range validation already covers at the point of use. It is left as it is.
