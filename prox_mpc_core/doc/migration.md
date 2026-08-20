# Migrating from prox_mpc_core 1.0.0

What changed on the released surface of `prox_mpc_core`, and what a 1.0.0 caller
has to do about it.
The version number this lands under is fixed by the maintainer against the final
diff and is not stated here.

## Summary

| Change | Source compatible | ABI compatible | Wire compatible |
| --- | --- | --- | --- |
| `prox_mpc::Model` gains a non-pure virtual `getPlanarMapping()` | yes | **no** | n/a |
| `MPC::setObs` accepts a second block layout | yes | yes | n/a |
| `prox_mpc::MPC` gains `solveCandidate()`, `commitCandidate()`, `getCandidateFinite()`, `setU0()` and candidate members | yes | **no** | n/a |
| `MPC::init()` rejects a nonsymmetric or indefinite weight matrix | yes | yes | n/a |
| `MPC::setGoalX` / `setGoalU` reject an undersized matrix | yes | yes | n/a |
| Structural `MPC` setters reject a post-`init()` call | yes | yes | n/a |
| The bicycle splits into two plugins; `prox_mpc_core/Bicycle` becomes a deprecated alias | yes | yes | n/a |
| `prox_mpc::Bicycle`'s emitted twist is corrected | yes | yes | n/a |
| The coupled obstacle constraint carries its previous-node gradient | yes | yes | n/a |

Nothing in this package touches a message, so nothing here is a wire event.

## Every `Model` plugin must be rebuilt

`prox_mpc::Model` gains one virtual member, `getPlanarMapping()`, appended after
`toTwist()` so it takes the last vtable slot.

A plugin that is recompiled against the new header needs no edit: the base class
supplies a default that reproduces the convention the controller assumed before
the hook existed - state `[x, y, yaw, (delta)]`, control `[v, ...]`, the state
referenced to `base_link`, and a steering angle at state index 3 for any model
with more than three states.

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

## `setObs` accepts a second block layout

`void MPC::setObs(MatrixXd)` and `void ProxQP::setObs(MatrixXd)` are unchanged:
same name, same parameter type, same return type, same mangled symbol. **A caller
passing the shape it passed before compiles, links and behaves identically.**

What is new is a second accepted shape, distinguished by the row count:

| Rows | Block `j` holds | Current-time position |
| --- | --- | --- |
| `Np * K` | predicted state `j + 1` | reconstructed from the first two blocks |
| `(Np + 1) * K` | predicted state `j` | supplied in the leading block |

Both are validated. A matrix that is neither shape now throws
`std::invalid_argument` rather than being reinterpreted; previously only the
first shape was accepted, so this widens what is legal and narrows nothing.

Supply the longer form when the caller knows where the obstacle is now, which
removes the reconstruction entirely. The bundled controller does.

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

Three inputs that 1.0.0 accepted silently are now rejected.

- `setGoalX` and `setGoalU` throw `std::invalid_argument` for a matrix with too
  few rows for the configured horizon, and for a wrong column count once
  `init()` has read the model's dimensions. 1.0.0 stored the matrix and read out
  of bounds during assembly, which `EIGEN_NO_DEBUG` left unchecked.
- `init()` throws `std::invalid_argument` unless `Q`, `S`, `R` and `W` are
  finite, symmetric and positive semidefinite. 1.0.0 accepted an asymmetric or
  indefinite weight and silently solved the symmetrised problem instead.
- `setQ`, `setR`, `setS`, `setW`, `setNp`, `setNc`, `setMaxIntIterQP`,
  `setMaxExtIterQP`, `setGuess` and `setQPtype` throw `std::logic_error` when
  called after `init()`. 1.0.0 returned quietly while mutating only their own
  members, leaving the buffers and the QP object sized for the previous
  configuration.

Call the structural setters before `init()`, size the goal matrices to the
horizon, and pass symmetric positive-semidefinite weights. The bundled Nav2
plugin already did all three, so a stack that uses it sees no change.

## The bicycle model

`prox_mpc_core/Bicycle` still resolves and still loads. It is now a deprecated
alias for `prox_mpc_core/BicycleFrontAxle`, warns once per construction, and is
removed in the next major release. Two things about it changed:

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
used on the current-pose side is now the obstacle's position *now*, taken from the
leading block when the caller supplies one and otherwise reconstructed by
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
