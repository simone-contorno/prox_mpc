# Migrating from prox_mpc_controller 1.0.0

What changed on the released surface of `prox_mpc_controller`, and what a 1.0.0
deployment has to do about it.
The version number this lands under is fixed by the maintainer against the final
diff and is not stated here.

## Summary

| Change | Source compatible | ABI compatible | Wire compatible |
| --- | --- | --- | --- |
| `model_plugin` defaults to `prox_mpc_core/Unicycle` | yes | yes | yes |
| New `allow_reversing` parameter, default `false` | yes | yes | yes |
| `ProxMpcController` gains `readModelMapping()` and cached mapping members | yes | **no** | yes |
| The model contract is validated at `configure()` and rejects what cannot be driven | yes | yes | yes |
| The in-loop keep-out is centred on `base_link` | yes | yes | yes |
| `SolverDiagnostics.converged` reports the applied command, not the QP status | yes | yes | **yes, layout and hash unchanged** |
| A rejected cycle no longer advances the solver's retained state | yes | yes | yes |
| Terminal-heading reference past the plan end | yes | yes | yes |

No `.msg` field is added, removed, renamed, retyped or reordered, so nothing here
changes a type hash or a CDR layout.

## The default model changed

`model_plugin` now defaults to `prox_mpc_core/Unicycle`. It previously defaulted
to `prox_mpc_core/Bicycle`, which no longer describes one model.

A `params.yaml` that sets `model_plugin` explicitly is unaffected, including one
that sets `prox_mpc_core/Bicycle`: that name still resolves, to the front-axle
bicycle, and logs a deprecation warning naming its replacement.

A deployment that relied on the default to get a four-state steered model must
now set it:

```yaml
FollowPath:
  model_plugin: "prox_mpc_core/BicycleFrontAxle"   # or BicycleRearAxle
```

Choose by where the model's state refers: `BicycleFrontAxle` puts `(x, y)` at the
front axle, `BicycleRearAxle` at the rear axle, which is the `base_link` origin
for a car-like base. `prox_mpc_core/Bicycle` maps to the front-axle model.

## Closed-loop behaviour of a bicycle model changed

The previous bicycle mixed three conventions. Each new plugin is consistent, so a
deployment using one will see different motion:

- The commanded twist is the body twist of `base_link`. For the front-axle model
  that is `linear.x = v cos(delta)`, not the front-wheel speed.
- The pose checked against the costmap footprint is the `base_link` pose, carried
  back from the model's reference point. The footprint check was previously
  performed at the front axle for a front-axle model.
- The steering reference is derived for the model's own reference point:
  `asin(L * kappa)` for the front axle, `atan(L * kappa)` for the rear.
- The wheelbase is read from the loaded model, not from `model_params.L`. The two
  agree for the bundled models, which apply that key.

The in-loop keep-out disc is centred on `base_link`, which is the point the robot
disc and the costmap footprint are both defined about, whatever point the model's
state refers to. The solver constrains the model's reference point, so each
obstacle is written into the solver's frame by the rotated reference-point offset
and the costmap scan is centred on `base_link`. The QP half-plane and the
footprint veto therefore protect the same physical point, which they did not for
a front-axle model before.

`d_safe` is unchanged at `robot_radius + safety_margin`, and so is the scan
window. For a model referenced to `base_link` - every model that declares no
offset - the shift is exactly zero and the obstacle fill is bit-identical.

## A model this controller cannot drive is now rejected

`configure()` reads the model's declared planar mapping and throws
`nav2_core::ControllerException` rather than indexing out of range, which
`EIGEN_NO_DEBUG` would leave unchecked. It rejects a model with fewer than three
states or no control, an index outside the model's own dimensions, two planar
quantities sharing one index, a steering angle without a usable declared
wheelbase or with a lateral reference offset, and - when `max_obstacles > 0` - a
position mapped away from state columns 0 and 1, which is where the solver's
keep-out rows read it.

A custom `prox_mpc::Model` that follows the previous implicit convention passes
unchanged, except that one carrying a steering angle must now declare its
wheelbase. See `prox_mpc_core/doc/migration.md`.

## Command validation covers every twist component

The command gate tested `linear.x` and `angular.z` while all six components of
the published twist come from the model. All six are now checked for finiteness,
on the accepted path and on the deceleration ramp. A model that fills only the
two planar components is unaffected.

## `SolverDiagnostics.converged` means something different

The message defines `converged` as a converged solve whose applied iterate is
finite. The controller previously set it from the QP status alone and published
before its own gates ran, so a cycle that was vetoed by the footprint check and
braked was recorded as converged.

It is now set only past the finiteness, footprint-veto and command-conversion
gates, and publication moves with it.

**The message is unchanged**: field names, types, order and defaults are
identical, the generated IDL is identical, and the RIHS01 type hash is
unchanged. A subscriber needs no rebuild. What changed is the runtime meaning of
one field, so a bag recorded before this and a bag recorded after are not
directly comparable on `converged` counts. The QP's own outcome is still
available, unchanged, in the `status` field; compare on that instead when a
cross-version comparison is needed.

## A rejected cycle no longer advances the solver

The controller now solves into a candidate and commits it only once its gates
accept. A cycle that fails to converge, produces a non-finite iterate, is vetoed
by the footprint check, or maps to a non-finite command leaves the solver's
retained trajectory, slack and previous control exactly as the last accepted
cycle left them, and the deceleration ramp anchors the next cycle's control-rate
constraint on the command it actually sent.

This changes motion after a rejected cycle: the next solve no longer plans a step
away from a command the robot never received. No configuration changes.

## New behaviour behind parameters

| Parameter | Type | Default | Units | Meaning |
| --- | --- | --- | --- | --- |
| `allow_reversing` | bool | `false` | - | Follow the plan's own pose orientations into reverse travel, signing the reference speed and truncating the reference at the first direction change. |
| `brake_period_s` | double | `0.0` | s | Step the deceleration ramp advances by on a braking cycle. `0.0` measures the inter-cycle period instead and clamps it into `[dt, 2 * dt]`; a positive value overrides the measurement and is used as-is. |

`allow_reversing`'s default reproduces the previous forward-only reference. With
it on, a model that declares no reverse travel keeps the forward-only reference
and logs a warning at `configure()`.

`brake_period_s`'s default of `0.0` reproduces 1.0.0's ramp on a
`controller_server` running at `dt`, which stepped by the configured `dt`
unconditionally. On a server running slower than `dt` the measured period is
used instead, so the ramp still decelerates at the rate the model declares
rather than at a fraction of it. Set a positive value to make the ramp
independent of scheduling jitter.

The terminal-heading reference is not behind a parameter. Past the plan end the
reference pose is now the goal pose rather than the final segment's tangent,
engaged only when the goal checker publishes a yaw tolerance it enforces. Goal
approach on a plan whose final orientation differs from its final segment tangent
will differ from 1.0.0.

## Object layout

`ProxMpcController` gains two protected methods and eleven protected members,
which changes `sizeof(ProxMpcController)`. The pluginlib load path is unaffected,
because the class is allocated and freed inside the same library. A downstream
package that links the exported target and derives from or holds
`ProxMpcController` directly must be rebuilt.

Two protected names changed with it. `fillStaticObstacles` and `reduceCostmap`
lose their leading plan-reference parameter, which the scan stopped reading when
it moved onto the solver's own nominal trajectory, and `a_dec_lin_`/`a_dec_ang_`
become `fallback_ramp_lin_`/`fallback_ramp_ang_`, which is what they hold: the
rates the last-resort twist ramp steps by, not a linear/angular acceleration
pair. A derived controller that overrode or called either must be updated, which
it must be rebuilt for in any case. One protected method, `keepOutShift`, is
added.
